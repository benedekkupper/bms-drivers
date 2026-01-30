/*
 * Copyright (c) The Libre Solar Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_bq769x0

#include "bq769x0_registers.h"

#include <bms/bms_common.h>
#include <drivers/bms_ic.h>

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h> /* for abs() function */
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/crc.h>

LOG_MODULE_REGISTER(bms_ic_bq769x0, CONFIG_BMS_IC_LOG_LEVEL);

#define BQ769X0_READ_MAX_ATTEMPTS      (3)
#define BQ769X0_TEMPERATURE_INTERVAL_S (2)

/* read-only driver configuration */
struct bms_ic_bq769x0_config
{
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec alert_gpio;
    struct gpio_dt_spec ntc_en_gpio;
    uint32_t shunt_resistor_uohm;
    uint32_t board_max_current;
    float thermistor_beta;
    uint8_t used_cell_count;
    uint8_t num_sections;
    uint8_t num_thermistors;
    const struct adc_dt_spec *extra_ntc_channels;
    uint8_t num_extra_thermistors;
};

/* driver run-time data */
struct bms_ic_bq769x0_data
{
    struct bms_ic_data ic_data;
    const struct device *dev;
    struct k_work_delayable alert_work;
    struct k_work_delayable balancing_work;
    /** ADC gain, factory-calibrated, read out from chip (uV/LSB) */
    int adc_gain;
    /** ADC offset, factory-calibrated, read out from chip (mV) */
    int adc_offset;
    struct gpio_callback alert_cb;
    /** Cached data from struct bms_ic_conf required for handling errors etc. in sofware */
    struct
    {
        /* Cell voltage limits */
        uint32_t cell_ov_reset;
        uint32_t cell_uv_reset;

        /* Cell temperature limits */
        int8_t overtemp_limit;
        int8_t undertemp_limit;
        int8_t temp_limit_hyst;

        /* Balancing settings */
        bool auto_balancing;
        uint32_t bal_cell_voltage_diff;
        uint32_t bal_cell_voltage_min;
        uint32_t bal_idle_current;
        uint32_t bal_idle_delay;

        bms_ic_event_callback_t event_callback;
    } ic_conf;
    int64_t active_timestamp;
    uint32_t error_timestamp_s;
    uint32_t poll_timestamp_s;
    uint32_t balancing_status;
    uint32_t adc_channels;
    float adc_lsb_mV;
    bool crc_enabled;
};

static int bq769x0_set_balancing_switches(const struct device *dev, uint32_t cells);
static int bq769x0_activate(const struct device *dev);
#ifdef CONFIG_BMS_IC_SWITCHES
static int bms_ic_bq769x0_set_switches(const struct device *dev, uint8_t switches, bool enabled);
#endif

/*
 * The bq769x0 drives the ALERT pin high if the SYS_STAT register contains
 * a new value (either new CC reading or an error)
 */
static void bq769x0_alert_isr(const struct device *port, struct gpio_callback *cb,
                              gpio_port_pins_t pins)
{
    struct bms_ic_bq769x0_data *data = CONTAINER_OF(cb, struct bms_ic_bq769x0_data, alert_cb);

    k_work_schedule(&data->alert_work, K_NO_WAIT);
}

static int bq769x0_write_byte(const struct device *dev, uint8_t reg_addr, uint8_t data)
{
    const struct bms_ic_bq769x0_config *dev_config = dev->config;
    const struct bms_ic_bq769x0_data *dev_data = dev->data;

    uint8_t buf[4] = {
        dev_config->i2c.addr << 1, /* target address for CRC calculation */
        reg_addr,
        data,
    };

    if (dev_data->crc_enabled) {
        buf[3] = crc8_ccitt(0, buf, 3);
        return i2c_write_dt(&dev_config->i2c, buf + 1, 3);
    }
    else {
        return i2c_write_dt(&dev_config->i2c, buf + 1, 2);
    }
}

static int bq769x0_read_bytes(const struct device *dev, uint8_t reg_addr, uint8_t *data,
                              size_t num_bytes)
{
    const struct bms_ic_bq769x0_config *dev_config = dev->config;
    const struct bms_ic_bq769x0_data *dev_data = dev->data;
    uint8_t buf[5] = {
        (dev_config->i2c.addr << 1) | 1U, /* target address for CRC calculation */
    };
    int err;

    if (num_bytes < 1 || num_bytes > 2) {
        return -EINVAL;
    }

    if (dev_data->crc_enabled) {
        for (int attempts = 1; attempts <= BQ769X0_READ_MAX_ATTEMPTS; attempts++) {
            err = i2c_write_read_dt(&dev_config->i2c, &reg_addr, 1, buf + 1, num_bytes * 2);
            if (err != 0) {
                return err;
            }

            /*
             * First CRC includes target address (incl. R/W bit) and data byte, subsequent CRCs
             * only consider data.
             */
            if (crc8_ccitt(0, buf, 2) == buf[2]) {
                data[0] = buf[1];
                if (num_bytes == 1) {
                    return 0;
                }
                else if (crc8_ccitt(0, buf + 3, 1) == buf[4]) {
                    data[1] = buf[3];
                    return 0;
                }
            }
        }

        LOG_ERR("Failed to read 0x%02X after %d attempts", reg_addr, BQ769X0_READ_MAX_ATTEMPTS);
        return -EIO;
    }
    else {
        return i2c_write_read_dt(&dev_config->i2c, &reg_addr, 1, data, num_bytes);
    }
}

static inline int bq769x0_read_byte(const struct device *dev, uint8_t reg_addr, uint8_t *byte)
{
    return bq769x0_read_bytes(dev, reg_addr, byte, sizeof(uint8_t));
}

static inline int bq769x0_read_word(const struct device *dev, uint8_t reg_addr, uint16_t *word)
{
    uint8_t buf[2];
    int err;

    err = bq769x0_read_bytes(dev, reg_addr, buf, sizeof(buf));
    if (err == 0) {
        *word = buf[0] << 8 | buf[1];
    }

    return err;
}

static int bq769x0_detect_crc(const struct device *dev)
{
    struct bms_ic_bq769x0_data *dev_data = dev->data;
    uint8_t cc_cfg = 0;
    int err = 0;

    dev_data->crc_enabled = true;
    err = bq769x0_write_byte(dev, BQ769X0_CC_CFG, 0x19);
    err |= bq769x0_read_byte(dev, BQ769X0_CC_CFG, &cc_cfg);
    if (err == 0 && cc_cfg == 0x19) {
        return 0;
    }

    dev_data->crc_enabled = false;
    err = bq769x0_write_byte(dev, BQ769X0_CC_CFG, 0x19);
    err |= bq769x0_read_byte(dev, BQ769X0_CC_CFG, &cc_cfg);
    if (err == 0 && cc_cfg == 0x19) {
        return 0;
    }

    return -EIO;
}

static uint16_t bq769x0_adc_to_mV(struct bms_ic_bq769x0_data *dev_data, uint16_t adc_value)
{
    return (adc_value * dev_data->adc_gain) / 1000 + dev_data->adc_offset;
}

static uint16_t bq769x0_mV_to_adc(struct bms_ic_bq769x0_data *dev_data, uint16_t mV_value)
{
    return (mV_value - dev_data->adc_offset) * 1000 / dev_data->adc_gain;
}

static int bq769x0_configure_cell_vp(const struct device *dev, const struct bms_ic_conf *ic_conf)
{
    struct bms_ic_bq769x0_data *dev_data = dev->data;
    union bq769x0_protect3 protect3;
    int ov_trip = 0;
    int uv_trip = 0;
    int err;

    /* TODO: update the reset values if the trip points are saturated */
    dev_data->ic_conf.cell_ov_reset = ic_conf->cell_ov_reset_mV;
    dev_data->ic_conf.cell_uv_reset = ic_conf->cell_uv_reset_mV;

    err = bq769x0_read_byte(dev, BQ769X0_PROTECT3, &protect3.byte);
    if (err != 0) {
        return err;
    }

    ov_trip = bq769x0_mV_to_adc(dev_data, ic_conf->cell_ov_limit_mV) >> 4;
    if (ov_trip > 0x2ff) {
        ov_trip = 0x2ff;
        LOG_DBG("OV limit too high, setting to maximum possible value %d mV",
                bq769x0_adc_to_mV(dev_data, ov_trip << 4));
    }
    else if (ov_trip < 0x200) {
        ov_trip = 0x200;
        LOG_DBG("OV limit too low, setting to minimum possible value %d mV",
                bq769x0_adc_to_mV(dev_data, ov_trip << 4));
    }
    err = bq769x0_write_byte(dev, BQ769X0_OV_TRIP, ov_trip);
    if (err != 0) {
        return err;
    }

    uv_trip = bq769x0_mV_to_adc(dev_data, ic_conf->cell_uv_limit_mV) >> 4;
    if (uv_trip < 0x100) {
        uv_trip = 0x100;
        LOG_DBG("UV limit too low, setting to minimum possible value %d mV",
                bq769x0_adc_to_mV(dev_data, uv_trip << 4));
    }
    else if (uv_trip > 0x1ff) {
        uv_trip = 0x1ff;
        LOG_DBG("UV limit too high, setting to maximum possible value %d mV",
                bq769x0_adc_to_mV(dev_data, uv_trip << 4));
    }
    err = bq769x0_write_byte(dev, BQ769X0_UV_TRIP, uv_trip);
    if (err != 0) {
        return err;
    }

    protect3.OV_DELAY = 0;
    for (int i = ARRAY_SIZE(bq769x0_ov_delays) - 1; i > 0; i--) {
        if (ic_conf->cell_ov_delay_ms >= bq769x0_ov_delays[i]) {
            protect3.OV_DELAY = i;
            break;
        }
    }
    protect3.UV_DELAY = 0;
    for (int i = ARRAY_SIZE(bq769x0_uv_delays) - 1; i > 0; i--) {
        if (ic_conf->cell_uv_delay_ms >= bq769x0_uv_delays[i]) {
            protect3.UV_DELAY = i;
            break;
        }
    }

    err = bq769x0_write_byte(dev, BQ769X0_PROTECT3, protect3.byte);
    return err;
}

#if 0
static int bq769x0_get_cell_vp(const struct device *dev, struct bms_ic_conf *ic_conf)
{
    struct bms_ic_bq769x0_data *dev_data = dev->data;
    union bq769x0_protect3 protect3;
    uint8_t ov_trip;
    uint8_t uv_trip;
    int err;

    err = bq769x0_read_byte(dev, BQ769X0_PROTECT3, &protect3.byte);
    if (err != 0) {
        return err;
    }
    err = bq769x0_read_byte(dev, BQ769X0_OV_TRIP, &ov_trip);
    if (err != 0) {
        return err;
    }
    ic_conf->cell_ov_limit_mV = bq769x0_adc_to_mV(dev_data, (0x200 | ov_trip) << 4);
    ic_conf->cell_ov_delay_ms = bq769x0_ov_delays[protect3.OV_DELAY];

    err = bq769x0_read_byte(dev, BQ769X0_UV_TRIP, &uv_trip);
    if (err != 0) {
        return err;
    }
    ic_conf->cell_uv_limit_mV = bq769x0_adc_to_mV(dev_data, (0x100 | uv_trip) << 4);
    ic_conf->cell_uv_delay_ms = bq769x0_uv_delays[protect3.UV_DELAY];
    return 0;
}
#endif

static int bq769x0_configure_temp_limits(const struct device *dev,
                                         const struct bms_ic_conf *ic_conf)
{
    struct bms_ic_bq769x0_data *dev_data = dev->data;

    dev_data->ic_conf.undertemp_limit = MAX(ic_conf->dis_ut_limit, ic_conf->chg_ut_limit);
    dev_data->ic_conf.overtemp_limit = MIN(ic_conf->dis_ot_limit, ic_conf->chg_ot_limit);
    dev_data->ic_conf.temp_limit_hyst = ic_conf->temp_limit_hyst;
    return 0;
}

#ifdef CONFIG_BMS_IC_CURRENT_MONITORING

static int bq769x0_configure_dis_ocp(const struct device *dev, const struct bms_ic_conf *ic_conf)
{
    const struct bms_ic_bq769x0_config *dev_config = dev->config;
    union bq769x0_protect2 protect2;
    int err;

    /* Remark: RSNS must be set to 1 in PROTECT1 register */

    protect2.OCD_THRESH = 0;
    for (int i = ARRAY_SIZE(bq769x0_ocd_thresholds) - 1; i > 0; i--) {
        if ((ic_conf->dis_oc_limit_mA * dev_config->shunt_resistor_uohm / 1000000)
            >= bq769x0_ocd_thresholds[i])
        {
            protect2.OCD_THRESH = i;
            break;
        }
    }

    protect2.OCD_DELAY = 0;
    for (int i = ARRAY_SIZE(bq769x0_ocd_delays) - 1; i > 0; i--) {
        if (ic_conf->dis_oc_delay_ms >= bq769x0_ocd_delays[i]) {
            protect2.OCD_DELAY = i;
            break;
        }
    }

    err = bq769x0_write_byte(dev, BQ769X0_PROTECT2, protect2.byte);
    return err;
}

#if 0
static int bq769x0_get_dis_ocp(const struct device *dev, struct bms_ic_conf *ic_conf)
{
    const struct bms_ic_bq769x0_config *dev_config = dev->config;
    union bq769x0_protect2 protect2;
    int err;

    err = bq769x0_read_byte(dev, BQ769X0_PROTECT2, &protect2.byte);
    if (err != 0) {
        return err;
    }

    ic_conf->dis_oc_limit_mA =
        bq769x0_ocd_thresholds[protect2.OCD_THRESH] * 1000000 / dev_config->shunt_resistor_uohm;
    ic_conf->dis_oc_delay_ms = bq769x0_ocd_delays[protect2.OCD_DELAY];

    return 0;
}
#endif

static int bq769x0_configure_dis_scp(const struct device *dev, const struct bms_ic_conf *ic_conf)
{
    const struct bms_ic_bq769x0_config *dev_config = dev->config;
    union bq769x0_protect1 protect1;
    int err;

    /* only RSNS = 1 considered */
    protect1.RSNS = 1;

    protect1.SCD_THRESH = 0;
    for (int i = ARRAY_SIZE(bq769x0_scd_thresholds) - 1; i > 0; i--) {
        if ((ic_conf->dis_sc_limit_mA * dev_config->shunt_resistor_uohm / 1000000)
            >= bq769x0_scd_thresholds[i])
        {
            protect1.SCD_THRESH = i;
            break;
        }
    }

    protect1.SCD_DELAY = 0;
    for (int i = ARRAY_SIZE(bq769x0_scd_delays) - 1; i > 0; i--) {
        if (ic_conf->dis_sc_delay_us >= bq769x0_scd_delays[i]) {
            protect1.SCD_DELAY = i;
            break;
        }
    }

    err = bq769x0_write_byte(dev, BQ769X0_PROTECT1, protect1.byte);
    return err;
}

#if 0
static int bq769x0_get_dis_scp(const struct device *dev, struct bms_ic_conf *ic_conf)
{
    const struct bms_ic_bq769x0_config *dev_config = dev->config;
    union bq769x0_protect1 protect1;
    int err;

    err = bq769x0_read_byte(dev, BQ769X0_PROTECT1, &protect1.byte);
    if (err != 0) {
        return err;
    }

    ic_conf->dis_sc_limit_mA =
        bq769x0_scd_thresholds[protect1.SCD_THRESH] * 1000000 / dev_config->shunt_resistor_uohm;
    ic_conf->dis_sc_delay_us = bq769x0_scd_delays[protect1.SCD_DELAY];

    return 0;
}
#endif

#endif /* CONFIG_BMS_IC_CURRENT_MONITORING */

static int bq769x0_configure_balancing(const struct device *dev, const struct bms_ic_conf *ic_conf)
{
    struct bms_ic_bq769x0_data *dev_data = dev->data;
    struct k_work_sync work_sync;

    dev_data->ic_conf.bal_cell_voltage_diff = ic_conf->bal_cell_voltage_diff_mV;
    dev_data->ic_conf.bal_cell_voltage_min = ic_conf->bal_cell_voltage_min_mV;
    dev_data->ic_conf.bal_idle_current = ic_conf->bal_idle_current_mA;
    dev_data->ic_conf.bal_idle_delay = ic_conf->bal_idle_delay_s * 1000;
    dev_data->ic_conf.auto_balancing = ic_conf->auto_balancing;

    if (ic_conf->auto_balancing) {
        k_work_schedule(&dev_data->balancing_work, K_NO_WAIT);
        return 0;
    }
    else {
        k_work_cancel_delayable_sync(&dev_data->balancing_work, &work_sync);
        return bq769x0_set_balancing_switches(dev, 0x0);
    }
}

static int bms_ic_bq769x0_configure(const struct device *dev, const struct bms_ic_conf *ic_conf,
                                    uint32_t flags)
{
    struct bms_ic_bq769x0_data *dev_data = dev->data;
    uint32_t actual_flags = 0;
    int err = 0;

    err = bq769x0_activate(dev);

    dev_data->ic_conf.event_callback = ic_conf->event_callback;

    if (flags & BMS_IC_CONF_VOLTAGE_LIMITS) {
        err |= bq769x0_configure_cell_vp(dev, ic_conf);
        actual_flags |= BMS_IC_CONF_VOLTAGE_LIMITS;
    }

    if (flags & BMS_IC_CONF_TEMP_LIMITS) {
        err |= bq769x0_configure_temp_limits(dev, ic_conf);
        actual_flags |= BMS_IC_CONF_TEMP_LIMITS;
    }

#ifdef CONFIG_BMS_IC_CURRENT_MONITORING
    if (flags & BMS_IC_CONF_CURRENT_LIMITS) {
        err |= bq769x0_configure_dis_ocp(dev, ic_conf);
        err |= bq769x0_configure_dis_scp(dev, ic_conf);
        actual_flags |= BMS_IC_CONF_CURRENT_LIMITS;
    }
#endif /* CONFIG_BMS_IC_CURRENT_MONITORING */

    if (flags & BMS_IC_CONF_BALANCING) {
        err |= bq769x0_configure_balancing(dev, ic_conf);
        actual_flags |= BMS_IC_CONF_BALANCING;
    }

    if (err != 0) {
        return -EIO;
    }

    return (actual_flags != 0) ? actual_flags : -ENOTSUP;
}

static int bq769x0_read_cell_voltages(const struct device *dev, struct bms_ic_data *ic_data)
{
    const struct bms_ic_bq769x0_config *dev_config = dev->config;
    struct bms_ic_bq769x0_data *dev_data = dev->data;
    uint16_t adc_raw;
    uint32_t sum_voltages = 0;
    uint32_t v_max = 0, v_min = 10000;
    int err;

    for (int i = 0; i < dev_config->used_cell_count; i++) {
        err = bq769x0_read_word(dev, BQ769X0_VC1_HI_BYTE + i * 2, &adc_raw);
        if (err != 0) {
            return err;
        }

        adc_raw &= 0x3FFF;
        ic_data->cell_voltages[i] = bq769x0_adc_to_mV(dev_data, adc_raw);
        sum_voltages += ic_data->cell_voltages[i];
        if (ic_data->cell_voltages[i] > v_max) {
            v_max = ic_data->cell_voltages[i];
        }
        if (ic_data->cell_voltages[i] < v_min && ic_data->cell_voltages[i] > 500) {
            v_min = ic_data->cell_voltages[i];
        }
    }
    ic_data->cell_voltage_avg = sum_voltages / dev_config->used_cell_count;
    ic_data->cell_voltage_min = v_min;
    ic_data->cell_voltage_max = v_max;

    return 0;
}

#ifdef CONFIG_BMS_IC_POLLING_READ_API
static int bq769x0_read_total_voltages(const struct device *dev, struct bms_ic_data *ic_data)
{
    const struct bms_ic_bq769x0_config *dev_config = dev->config;
    struct bms_ic_bq769x0_data *dev_data = dev->data;
    uint16_t adc_raw;
    int err;

    err = bq769x0_read_word(dev, BQ769X0_BAT_HI_BYTE, &adc_raw);
    if (err != 0) {
        return err;
    }

    ic_data->total_voltage = 4 * dev_data->adc_gain * adc_raw / 1000
                             + dev_config->used_cell_count * dev_data->adc_offset;

    return 0;
}
#endif /* CONFIG_BMS_IC_POLLING_READ_API */

static int bq769x0_read_temperatures(const struct device *dev, struct bms_ic_data *ic_data)
{
    const struct bms_ic_bq769x0_config *dev_config = dev->config;
    struct bms_ic_bq769x0_data *dev_data = dev->data;
    float tmp = 0;
    uint16_t adc_raw[CONFIG_BMS_IC_MAX_THERMISTORS] = { 0 };
    int vtsx = 0;
    unsigned long rts = 0;
    float sum_temps = 0.0F;
    int err;

    /* power the external NTCs only for the duration of the measurement,
     * to avoid self-heating */
    if (gpio_is_ready_dt(&dev_config->ntc_en_gpio)) {
        gpio_pin_set_dt(&dev_config->ntc_en_gpio, 1);
        k_sleep(K_MSEC(1));
    }

    /* read thermistor values from BMS IC (new sample is taken every 2s) */
    for (int i = 0; i < dev_config->num_thermistors; i++) {
        err = bq769x0_read_word(dev, BQ769X0_TS1_HI_BYTE + i * 2, &adc_raw[i]);
        if (err != 0) {
            return err;
        }
    }
    /* read extra thermistor values from ADC */
    if (dev_config->num_extra_thermistors) {
        struct adc_sequence sequence = {
            .channels = dev_data->adc_channels,
            .buffer = &adc_raw[dev_config->num_thermistors],
            .buffer_size = dev_config->num_extra_thermistors * sizeof(uint16_t),
            .resolution = dev_config->extra_ntc_channels[0].resolution,
            .oversampling = dev_config->extra_ntc_channels[0].oversampling,
        };

        err = adc_read_dt(dev_config->extra_ntc_channels, &sequence);
        if (err != 0) {
            LOG_ERR("ADC read error for extra thermistors: %d", err);
            return err;
        }
    }

    if (gpio_is_ready_dt(&dev_config->ntc_en_gpio)) {
        gpio_pin_set_dt(&dev_config->ntc_en_gpio, 0);
    }

    /* calculate temperatures */
    for (int i = 0; i < (dev_config->num_thermistors + dev_config->num_extra_thermistors); i++) {
        if (i < dev_config->num_thermistors) {
            adc_raw[i] &= 0x3FFF;
            vtsx = adc_raw[i] * 0.382F; /* mV */
        }
        else {
            vtsx = (float)adc_raw[i] * dev_data->adc_lsb_mV; /* mV */
        }
        rts = 10000.0F * vtsx / (3300.0F - vtsx); /* Ohm */

        /*
         * Temperature calculation using Beta equation
         * - According to bq769x0 datasheet, only 10k thermistors should be used
         * - 25°C reference temperature for Beta equation assumed
         */
        tmp =
            1.0F
            / (1.0F / (273.15F + 25.F) + 1.0F / dev_config->thermistor_beta * logf(rts / 10000.0F));
        tmp -= 273.15F;
        ic_data->cell_temps[i] = tmp + 0.5F; /* round to nearest integer */
        if (i == 0) {
            ic_data->cell_temp_min = ic_data->cell_temps[i];
            ic_data->cell_temp_max = ic_data->cell_temps[i];
        }
        else {
            if (ic_data->cell_temps[i] < ic_data->cell_temp_min) {
                ic_data->cell_temp_min = ic_data->cell_temps[i];
            }
            if (ic_data->cell_temps[i] > ic_data->cell_temp_max) {
                ic_data->cell_temp_max = ic_data->cell_temps[i];
            }
        }
        sum_temps += tmp;
    }
    ic_data->cell_temp_avg =
        sum_temps / (float)(dev_config->num_thermistors + dev_config->num_extra_thermistors) + 0.5F;

    /* update error flags */
    int8_t hyst = (ic_data->error_flags & BMS_ERR_OVERTEMP) ? dev_data->ic_conf.temp_limit_hyst : 0;
    if (ic_data->cell_temp_max > dev_data->ic_conf.overtemp_limit - hyst) {
        ic_data->error_flags |= BMS_ERR_OVERTEMP;
    }
    else {
        ic_data->error_flags &= ~BMS_ERR_OVERTEMP;
    }

    hyst = (ic_data->error_flags & BMS_ERR_UNDERTEMP) ? dev_data->ic_conf.temp_limit_hyst : 0;
    if (ic_data->cell_temp_min < dev_data->ic_conf.undertemp_limit + hyst) {
        ic_data->error_flags |= BMS_ERR_UNDERTEMP;
    }
    else {
        ic_data->error_flags &= ~BMS_ERR_UNDERTEMP;
    }

    return 0;
}

#ifdef CONFIG_BMS_IC_CURRENT_MONITORING

static int bq769x0_read_current(const struct device *dev, struct bms_ic_data *ic_data)
{
    const struct bms_ic_bq769x0_config *dev_config = dev->config;
    struct bms_ic_bq769x0_data *dev_data = dev->data;
    int16_t adc_raw;

    int err = bq769x0_read_word(dev, BQ769X0_CC_HI_BYTE, (uint16_t *)&adc_raw);
    if (err != 0) {
        LOG_ERR("Error reading current measurement");
        return err;
    }

    /* remove noise around 0 A */
    if (adc_raw > -2 && adc_raw < 2) {
        adc_raw = 0;
    }

    int32_t current_mA = adc_raw * 8440 / (int32_t)dev_config->shunt_resistor_uohm;

    ic_data->current = current_mA;

    /* reset active timestamp */
    if (fabsf(ic_data->current) > dev_data->ic_conf.bal_idle_current) {
        dev_data->active_timestamp = k_uptime_get();
    }

    return 0;
}

#endif /* CONFIG_BMS_IC_CURRENT_MONITORING */

#ifdef CONFIG_BMS_IC_POLLING_READ_API
static int bq769x0_read_error_flags(const struct device *dev, struct bms_ic_data *ic_data)
{
    union bq769x0_sys_stat sys_stat;
    struct bms_ic_bq769x0_data *dev_data = dev->data;
    uint32_t error_flags = ic_data->error_flags
                           & ~(BMS_ERR_CELL_UNDERVOLTAGE | BMS_ERR_CELL_OVERVOLTAGE
                               | BMS_ERR_SHORT_CIRCUIT | BMS_ERR_DIS_OVERCURRENT);

    int err = bq769x0_read_byte(dev, BQ769X0_SYS_STAT, &sys_stat.byte);
    if (err != 0) {
        return err;
    }

    error_flags |= (sys_stat.UV * UINT32_MAX) & BMS_ERR_CELL_UNDERVOLTAGE;
    error_flags |= (sys_stat.OV * UINT32_MAX) & BMS_ERR_CELL_OVERVOLTAGE;
    error_flags |= (sys_stat.SCD * UINT32_MAX) & BMS_ERR_SHORT_CIRCUIT;
    error_flags |= (sys_stat.OCD * UINT32_MAX) & BMS_ERR_DIS_OVERCURRENT;

    int8_t hyst = (ic_data->error_flags & BMS_ERR_OVERTEMP) ? dev_data->ic_conf.temp_limit_hyst : 0;
    if (ic_data->cell_temp_max > dev_data->ic_conf.overtemp_limit - hyst) {
        ic_data->error_flags |= BMS_ERR_OVERTEMP;
    }
    else {
        ic_data->error_flags &= ~BMS_ERR_OVERTEMP;
    }

    hyst = (ic_data->error_flags & BMS_ERR_UNDERTEMP) ? dev_data->ic_conf.temp_limit_hyst : 0;
    if (ic_data->cell_temp_min < dev_data->ic_conf.undertemp_limit + hyst) {
        ic_data->error_flags |= BMS_ERR_UNDERTEMP;
    }
    else {
        ic_data->error_flags &= ~BMS_ERR_UNDERTEMP;
    }

    ic_data->error_flags = error_flags;

    return 0;
}
#endif /* CONFIG_BMS_IC_POLLING_READ_API */

static uint32_t bq769x0_update_status(const struct device *dev, struct bms_ic_data *ic_data)
{
    struct bms_ic_bq769x0_data *dev_data = dev->data;
    uint32_t event_flags = 0;
    int err;

    union bq769x0_sys_stat sys_stat = { 0 };
    err = bq769x0_read_byte(dev, BQ769X0_SYS_STAT, &sys_stat.byte);

#ifdef CONFIG_BMS_IC_CURRENT_MONITORING
    /* get new current reading if available */
    if (sys_stat.CC_READY == 1) {
        int prev_current = ic_data->current;
        bq769x0_read_current(dev, ic_data);
        LOG_DBG("New current reading: %d mA", ic_data->current);

        err = bq769x0_write_byte(dev, BQ769X0_SYS_STAT, BQ769X0_SYS_STAT_CC_READY);
        if (err != 0) {
            LOG_ERR("Failed to clear CC_READY flag");
        }

        /* deliver new values to application */
        if ((ic_data->current != 0) || (prev_current != 0)) {
            event_flags |= BMS_IC_DATA_CURRENT;
        }
    }
#endif /* CONFIG_BMS_IC_CURRENT_MONITORING */

    uint32_t current_time_s = k_uptime_seconds();
    uint32_t prev_error_flags = ic_data->error_flags;

    /* keep only the errors for the rest */
    sys_stat.byte &= BQ769X0_SYS_STAT_ERROR_MASK;

    /* read voltages periodically, and when error is reported */
    if (((current_time_s - dev_data->poll_timestamp_s) >= BQ769X0_TEMPERATURE_INTERVAL_S)
        || (sys_stat.byte & (BQ769X0_SYS_STAT_UV | BQ769X0_SYS_STAT_OV)))
    {
        bq769x0_read_cell_voltages(dev, ic_data);
        ic_data->total_voltage = 0;
        for (int i = 0; i < ic_data->connected_cells; i++) {
            ic_data->total_voltage += ic_data->cell_voltages[i];
        }
        event_flags |= BMS_IC_DATA_CELL_VOLTAGES | BMS_IC_DATA_PACK_VOLTAGES;
    }

    /* read temperatures periodically */
    if ((current_time_s - dev_data->poll_timestamp_s) >= BQ769X0_TEMPERATURE_INTERVAL_S) {
        bq769x0_read_temperatures(dev, ic_data);
#ifdef CONFIG_BMS_IC_SWITCHES
        /* React on temperature errors by disabling switches */
        if ((ic_data->error_flags & (BMS_ERR_UNDERTEMP | BMS_ERR_OVERTEMP))
            && (prev_error_flags & (BMS_ERR_UNDERTEMP | BMS_ERR_OVERTEMP)) == 0)
        {
            LOG_INF("Disabling switches due to temperature error 0x%X",
                    ic_data->error_flags & (BMS_ERR_UNDERTEMP | BMS_ERR_OVERTEMP));
            if (ic_data->active_switches != 0) {
                event_flags |= BMS_IC_DATA_SWITCH_STATE;
                bms_ic_bq769x0_set_switches(dev, BMS_SWITCH_CHG | BMS_SWITCH_DIS, false);
            }
        }
#endif
        dev_data->poll_timestamp_s = current_time_s;
        event_flags |= BMS_IC_DATA_TEMPERATURES;
    }

    /* handle potential errors */
    if (sys_stat.byte != 0) {
#ifdef CONFIG_BMS_IC_SWITCHES
        if (ic_data->active_switches != 0) {
            union bq769x0_sys_ctrl2 sys_ctrl2;

            /* Errors change the switch state, need to sync it */
            err = bq769x0_read_byte(dev, BQ769X0_SYS_CTRL2, &sys_ctrl2.byte);
            if (err == 0) {
                uint8_t active_switches = ic_data->active_switches;
                ic_data->active_switches = (sys_ctrl2.CHG_ON ? BMS_SWITCH_CHG : 0)
                                           | (sys_ctrl2.DSG_ON ? BMS_SWITCH_DIS : 0);
                if (ic_data->active_switches != active_switches) {
                    LOG_WRN("Switch state changed due to errors 0x%X: %d -> %d", sys_stat.byte,
                            active_switches, ic_data->active_switches);
                    event_flags |= BMS_IC_DATA_SWITCH_STATE;
                }
            }
        }
#endif
        if (dev_data->error_timestamp_s == UINT32_MAX) {
            dev_data->error_timestamp_s = current_time_s;
        }
        uint32_t elapsed_time_s = current_time_s - dev_data->error_timestamp_s;
        err = 0;

        if (sys_stat.DEVICE_XREADY || sys_stat.OVRD_ALERT) {
            ic_data->error_flags |= BMS_ERR_IC;
        }
        else {
            ic_data->error_flags &= ~BMS_ERR_IC;
        }
        if (sys_stat.DEVICE_XREADY) {
            /* datasheet recommendation: try to clear after waiting a few seconds */
            if (elapsed_time_s > 3) {
                LOG_DBG("Clearing XR error");
                err |= bq769x0_write_byte(dev, BQ769X0_SYS_STAT, BQ769X0_SYS_STAT_DEVICE_XREADY);
            }
        }
        if (sys_stat.OVRD_ALERT) {
            if (elapsed_time_s > 10) {
                LOG_DBG("Clearing Override Alert error");
                err |= bq769x0_write_byte(dev, BQ769X0_SYS_STAT, BQ769X0_SYS_STAT_OVRD_ALERT);
            }
        }
        if (sys_stat.UV) {
            ic_data->error_flags |= BMS_ERR_CELL_UNDERVOLTAGE;
            if (ic_data->cell_voltage_min > dev_data->ic_conf.cell_uv_reset) {
                LOG_DBG("Clearing UV error");
                err |= bq769x0_write_byte(dev, BQ769X0_SYS_STAT, BQ769X0_SYS_STAT_UV);
            }
        }
        else {
            ic_data->error_flags &= ~BMS_ERR_CELL_UNDERVOLTAGE;
        }
        if (sys_stat.OV) {
            ic_data->error_flags |= BMS_ERR_CELL_OVERVOLTAGE;
            if (ic_data->cell_voltage_max < dev_data->ic_conf.cell_ov_reset) {
                LOG_DBG("Clearing OV error");
                err |= bq769x0_write_byte(dev, BQ769X0_SYS_STAT, BQ769X0_SYS_STAT_OV);
            }
        }
        else {
            ic_data->error_flags &= ~BMS_ERR_CELL_OVERVOLTAGE;
        }
        if (sys_stat.SCD) {
            ic_data->error_flags |= BMS_ERR_SHORT_CIRCUIT;
            if (elapsed_time_s > 60) {
                LOG_DBG("Clearing SCD error");
                err |= bq769x0_write_byte(dev, BQ769X0_SYS_STAT, BQ769X0_SYS_STAT_SCD);
            }
        }
        else {
            ic_data->error_flags &= ~BMS_ERR_SHORT_CIRCUIT;
        }
        if (sys_stat.OCD) {
            ic_data->error_flags |= BMS_ERR_DIS_OVERCURRENT;
            if (elapsed_time_s > 60) {
                LOG_DBG("Clearing OCD error");
                err |= bq769x0_write_byte(dev, BQ769X0_SYS_STAT, BQ769X0_SYS_STAT_OCD);
            }
        }
        else {
            ic_data->error_flags &= ~BMS_ERR_DIS_OVERCURRENT;
        }

        if (err != 0) {
            LOG_ERR("Attempts to clear error flags failed");
        }

        /* While errors persist, the ALERT pin is asserted, need to poll for updates */
        k_work_reschedule(&dev_data->alert_work, K_SECONDS(1));
    }
    else {
        /* errors have been cleared */
        dev_data->error_timestamp_s = UINT32_MAX;
        ic_data->error_flags &= ~(BMS_ERR_IC | BMS_ERR_CELL_UNDERVOLTAGE | BMS_ERR_CELL_OVERVOLTAGE
                                  | BMS_ERR_SHORT_CIRCUIT | BMS_ERR_DIS_OVERCURRENT);

#ifndef CONFIG_BMS_IC_CURRENT_MONITORING
        /* without current monitoring the ALERT pin doesn't get asserted without errors */
        k_work_reschedule(&dev_data->alert_work, K_SECONDS(BQ769X0_TEMPERATURE_INTERVAL_S));
#endif
    }

    if (prev_error_flags != ic_data->error_flags) {
        LOG_INF("Error flags updated: 0x%X -> 0x%X", prev_error_flags, ic_data->error_flags);
        event_flags |= BMS_IC_DATA_ERROR_FLAGS;
    }

    return event_flags;
}

static void bq769x0_alert_handler(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct bms_ic_bq769x0_data *dev_data =
        CONTAINER_OF(dwork, struct bms_ic_bq769x0_data, alert_work);
    const struct device *dev = dev_data->dev;
    struct bms_ic_data *ic_data = &dev_data->ic_data;
    uint32_t event_flags = bq769x0_update_status(dev, ic_data);

    /* notify application about new data / errors */
    if ((event_flags != 0) && (dev_data->ic_conf.event_callback != NULL)) {
        dev_data->ic_conf.event_callback(dev, event_flags, ic_data);
    }
}

static int bms_ic_bq769x0_read_data(const struct device *dev, struct bms_ic_data **data_ptr,
                                    uint32_t flags)
{
    struct bms_ic_bq769x0_data *dev_data = dev->data;
    struct bms_ic_data *ic_data = &dev_data->ic_data;
    uint32_t actual_flags = 0;
    int err = 0;

    *data_ptr = ic_data;

#ifdef CONFIG_BMS_IC_POLLING_READ_API
    if (flags & BMS_IC_DATA_CELL_VOLTAGES) {
        err |= bq769x0_read_cell_voltages(dev, ic_data);
        actual_flags |= BMS_IC_DATA_CELL_VOLTAGES;
    }

    if (flags & BMS_IC_DATA_PACK_VOLTAGES) {
        err |= bq769x0_read_total_voltages(dev, ic_data);
        actual_flags |= BMS_IC_DATA_PACK_VOLTAGES;
    }

    if (flags & BMS_IC_DATA_TEMPERATURES) {
        err |= bq769x0_read_temperatures(dev, ic_data);
        actual_flags |= BMS_IC_DATA_TEMPERATURES;
    }

#ifdef CONFIG_BMS_IC_CURRENT_MONITORING
    if (flags & BMS_IC_DATA_CURRENT) {
        err |= bq769x0_read_current(dev, ic_data);
        actual_flags |= BMS_IC_DATA_CURRENT;
    }
#endif /* CONFIG_BMS_IC_CURRENT_MONITORING */

    if (flags & BMS_IC_DATA_BALANCING) {
        ic_data->balancing_status = dev_data->balancing_status;
        actual_flags |= BMS_IC_DATA_BALANCING;
    }

    if (flags & BMS_IC_DATA_ERROR_FLAGS) {
        err |= bq769x0_read_error_flags(dev, ic_data);
        actual_flags |= BMS_IC_DATA_ERROR_FLAGS;
    }

#ifdef CONFIG_BMS_IC_SWITCHES
    if (flags & BMS_IC_DATA_SWITCH_STATE) {
        union bq769x0_sys_ctrl2 sys_ctrl2;
        err |= bq769x0_read_byte(dev, BQ769X0_SYS_CTRL2, &sys_ctrl2.byte);
        ic_data->active_switches =
            (sys_ctrl2.CHG_ON ? BMS_SWITCH_CHG : 0) | (sys_ctrl2.DSG_ON ? BMS_SWITCH_DIS : 0);
        actual_flags |= BMS_IC_DATA_SWITCH_STATE;
    }
#endif

    if (err != 0) {
        return -EIO;
    }

    return (flags == actual_flags) ? 0 : -EINVAL;
#else
    return 0;
#endif /* CONFIG_BMS_IC_POLLING_READ_API */
}

#ifdef CONFIG_BMS_IC_SWITCHES

static int bms_ic_bq769x0_set_switches(const struct device *dev, uint8_t switches, bool enabled)
{
    struct bms_ic_bq769x0_data *dev_data = dev->data;
    union bq769x0_sys_ctrl2 sys_ctrl2;
    int err;

    if ((switches & (BMS_SWITCH_CHG | BMS_SWITCH_DIS)) != switches) {
        return -EINVAL;
    }

    err = bq769x0_read_byte(dev, BQ769X0_SYS_CTRL2, &sys_ctrl2.byte);
    if (err != 0) {
        return err;
    }

    if (switches & BMS_SWITCH_CHG) {
        sys_ctrl2.CHG_ON = enabled ? 1 : 0;
    }
    if (switches & BMS_SWITCH_DIS) {
        sys_ctrl2.DSG_ON = enabled ? 1 : 0;
    }

    err = bq769x0_write_byte(dev, BQ769X0_SYS_CTRL2, sys_ctrl2.byte);
    if (err != 0) {
        return err;
    }
    dev_data->ic_data.active_switches =
        (sys_ctrl2.CHG_ON ? BMS_SWITCH_CHG : 0) | (sys_ctrl2.DSG_ON ? BMS_SWITCH_DIS : 0);

    return 0;
}

#endif /* CONFIG_BMS_IC_SWITCHES */

static int bq769x0_set_balancing_switches(const struct device *dev, uint32_t cells)
{
    const struct bms_ic_bq769x0_config *dev_config = dev->config;
    struct bms_ic_bq769x0_data *dev_data = dev->data;
    int err;

    for (int section = 0; section < dev_config->num_sections; section++) {
        uint8_t cells_section = (cells >> section * 5) & 0x1F;
        if (((cells_section << 1) & cells_section) || ((cells_section >> 1) & cells_section)) {
            /* balancing of adjacent cells within one section not allowed */
            return -EINVAL;
        }

        err = bq769x0_write_byte(dev, BQ769X0_CELLBAL1 + section, cells_section);
        if (err != 0) {
            return err;
        }
    }

    dev_data->balancing_status = cells;

    return 0;
}

static void bq769x0_balancing_work_handler(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct bms_ic_bq769x0_data *dev_data =
        CONTAINER_OF(dwork, struct bms_ic_bq769x0_data, balancing_work);
    const struct device *dev = dev_data->dev;
    const struct bms_ic_bq769x0_config *dev_config = dev->config;
    struct bms_ic_data *ic_data = &dev_data->ic_data;
    int err;

    if (k_uptime_delta(&dev_data->active_timestamp) >= dev_data->ic_conf.bal_idle_delay
        && ic_data->cell_voltage_max > dev_data->ic_conf.bal_cell_voltage_min
        && (ic_data->cell_voltage_max - ic_data->cell_voltage_min)
               > dev_data->ic_conf.bal_cell_voltage_diff)
    {
        ic_data->balancing_status = 0; /* current status will be set in following loop */

        int balancing_flags;
        int balancing_flags_target;

        for (int section = 0; section < dev_config->num_sections; section++) {
            /* find cells which should be balanced and sort them by voltage descending */
            int cell_list[5];
            int cell_counter = 0;
            for (int i = 0; i < 5; i++) {
                if ((ic_data->cell_voltages[section * 5 + i] - ic_data->cell_voltage_min)
                    > dev_data->ic_conf.bal_cell_voltage_diff)
                {
                    int j = cell_counter;
                    while (j > 0
                           && ic_data->cell_voltages[section * 5 + cell_list[j - 1]]
                                  < ic_data->cell_voltages[section * 5 + i])
                    {
                        cell_list[j] = cell_list[j - 1];
                        j--;
                    }
                    cell_list[j] = i;
                    cell_counter++;
                }
            }

            balancing_flags = 0;
            for (int i = 0; i < cell_counter; i++) {
                /* try to enable balancing of current cell */
                balancing_flags_target = balancing_flags | (1 << cell_list[i]);

                /* check if attempting to balance adjacent cells */
                bool adjacent_cell_collision = ((balancing_flags_target << 1) & balancing_flags)
                                               || ((balancing_flags << 1) & balancing_flags_target);

                if (adjacent_cell_collision == false) {
                    balancing_flags = balancing_flags_target;
                }
            }

            ic_data->balancing_status |= balancing_flags << section * 5;

            /* set balancing register for this section */
            err = bq769x0_write_byte(dev, BQ769X0_CELLBAL1 + section, balancing_flags);
            if (err == 0) {
                LOG_DBG("Set CELLBAL%d register to 0x%02X", section + 1, balancing_flags);
            }
            else {
                LOG_ERR("Failed to set CELBAL%d register: %d", section + 1, err);
            }
        }
    }
    else if (ic_data->balancing_status > 0) {
        /* clear all CELLBAL registers */
        for (int section = 0; section < dev_config->num_sections; section++) {
            err = bq769x0_write_byte(dev, BQ769X0_CELLBAL1 + section, 0x0);
            if (err == 0) {
                LOG_DBG("Cleared CELLBAL%d register", section + 1);
            }
            else {
                LOG_ERR("Clearing CELBAL%d register failed: %d", section + 1, err);
            }
        }

        ic_data->balancing_status = 0;
    }

    k_work_schedule(dwork, K_SECONDS(1));
}

static int bms_ic_bq769x0_balance(const struct device *dev, uint32_t cells)
{
    struct bms_ic_bq769x0_data *dev_data = dev->data;

    if (dev_data->ic_conf.auto_balancing) {
        return -EBUSY;
    }

    return bq769x0_set_balancing_switches(dev, cells);
}

static int bq769x0_activate(const struct device *dev)
{
    const struct bms_ic_bq769x0_config *dev_config = dev->config;
    struct bms_ic_bq769x0_data *dev_data = dev->data;
    int err;

    if (dev_data->adc_gain != 0) {
        /* already activated */
        return 0;
    }
    /* Datasheet: 10 ms delay (t_BOOTREADY) */
    k_sleep(K_MSEC(10));

    err = bq769x0_detect_crc(dev);
    if (err == 0) {
        uint8_t adcoffset;
        uint8_t adcgain1;
        uint8_t adcgain2;

        /* switch external thermistor and ADC on */
        err |= bq769x0_write_byte(dev, BQ769X0_SYS_CTRL1, 0b00011000);
#ifdef CONFIG_BMS_IC_CURRENT_MONITORING
        /* switch CC_EN on */
        err |= bq769x0_write_byte(dev, BQ769X0_SYS_CTRL2, 0b01000000);
#else
        err |= bq769x0_write_byte(dev, BQ769X0_SYS_CTRL2, 0b00000000);
#endif

        /* get ADC offset (2's complement) and gain */
        err |= bq769x0_read_byte(dev, BQ769X0_ADCOFFSET, &adcoffset);
        dev_data->adc_offset = (signed int)adcoffset;

        err |= bq769x0_read_byte(dev, BQ769X0_ADCGAIN1, &adcgain1);
        err |= bq769x0_read_byte(dev, BQ769X0_ADCGAIN2, &adcgain2);

        dev_data->adc_gain =
            365 + (((adcgain1 & 0b00001100) << 1) | ((adcgain2 & 0b11100000) >> 5));

        if (err != 0) {
            return -EIO;
        }
    }
    else {
        LOG_ERR("BMS communication error");
        return err;
    }

    err = gpio_pin_configure_dt(&dev_config->alert_gpio, GPIO_INPUT);
    err = gpio_pin_interrupt_configure_dt(&dev_config->alert_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&dev_data->alert_cb, bq769x0_alert_isr, BIT(dev_config->alert_gpio.pin));
    err = gpio_add_callback_dt(&dev_config->alert_gpio, &dev_data->alert_cb);

    /* run processing once at start-up to check and clear errors */
    k_work_schedule(&dev_data->alert_work, K_NO_WAIT);

    return 0;
}

static int bms_ic_bq769x0_set_mode(const struct device *dev, enum bms_ic_mode mode)
{
    int err = 0;

    switch (mode) {
        case BMS_IC_MODE_ACTIVE:
            return bq769x0_activate(dev);
        case BMS_IC_MODE_OFF:
            /* put IC into SHIP mode (i.e. switched off) */
            err |= bq769x0_write_byte(dev, BQ769X0_SYS_CTRL1, 0x0);
            err |= bq769x0_write_byte(dev, BQ769X0_SYS_CTRL1, 0x1);
            err |= bq769x0_write_byte(dev, BQ769X0_SYS_CTRL1, 0x2);
            return err == 0 ? 0 : -EIO;
        default:
            return -ENOTSUP;
    }
}

static int bq769x0_init(const struct device *dev)
{
    const struct bms_ic_bq769x0_config *dev_config = dev->config;
    struct bms_ic_bq769x0_data *dev_data = dev->data;

    if (!i2c_is_ready_dt(&dev_config->i2c)) {
        LOG_ERR("I2C device not ready");
        return -ENODEV;
    }
    if (!gpio_is_ready_dt(&dev_config->alert_gpio)) {
        LOG_ERR("Alert GPIO not ready");
        return -ENODEV;
    }

    dev_data->dev = dev;
    dev_data->ic_data.connected_cells = dev_config->used_cell_count;
    dev_data->ic_data.used_thermistors =
        dev_config->num_thermistors + dev_config->num_extra_thermistors;

    /* set initial error flag, so callback is triggered at startup when no errors are present */
    dev_data->ic_data.error_flags = BMS_ERR_IC;

    k_work_init_delayable(&dev_data->alert_work, bq769x0_alert_handler);
    k_work_init_delayable(&dev_data->balancing_work, bq769x0_balancing_work_handler);

    for (int i = 0; i < dev_config->num_extra_thermistors; i++) {
        if (!adc_is_ready_dt(&dev_config->extra_ntc_channels[i])) {
            LOG_ERR("Extra NTC ADC channel %d not ready", i);
            return -ENODEV;
        }
        if ((dev_config->extra_ntc_channels[i].dev != dev_config->extra_ntc_channels[0].dev)
            || (dev_config->extra_ntc_channels[i].resolution
                != dev_config->extra_ntc_channels[0].resolution)
            || (dev_config->extra_ntc_channels[i].oversampling
                != dev_config->extra_ntc_channels[0].oversampling))
        {
            LOG_ERR("All NTC ADC channels share the same configuration values");
            return -EINVAL;
        }
        int err = adc_channel_setup_dt(&dev_config->extra_ntc_channels[i]);
        if (err < 0) {
            LOG_ERR("Could not setup channel #%d (%d)\n", i, err);
            return err;
        }
        dev_data->adc_channels |= BIT(dev_config->extra_ntc_channels[i].channel_id);
    }

    if (dev_config->num_extra_thermistors) {
        const struct adc_dt_spec *spec = &dev_config->extra_ntc_channels[0];
        float vref_mv;

        if (spec->channel_cfg_dt_node_exists && (spec->channel_cfg.reference == ADC_REF_INTERNAL)) {
            vref_mv = (float)adc_ref_internal(spec->dev);
        }
        else if (spec->vref_mv) {
            vref_mv = (float)spec->vref_mv;
        }
        else {
            vref_mv = 3300.0F;
        }
        dev_data->adc_lsb_mV = vref_mv / (float)((1 << spec->resolution) - 1);
    }

    if (gpio_is_ready_dt(&dev_config->ntc_en_gpio)) {
        int err = gpio_pin_configure_dt(&dev_config->ntc_en_gpio, GPIO_OUTPUT_INACTIVE);
        if (err < 0) {
            return err;
        }
    }

    return 0;
}

static const struct bms_ic_driver_api bq769x0_driver_api = {
    .configure = bms_ic_bq769x0_configure,
    .read_data = bms_ic_bq769x0_read_data,
#ifdef CONFIG_BMS_IC_SWITCHES
    .set_switches = bms_ic_bq769x0_set_switches,
#endif
    .balance = bms_ic_bq769x0_balance,
    .set_mode = bms_ic_bq769x0_set_mode,
};

#define BQ769X0_ASSERT_CURRENT_MONITORING_PROP_GREATER_ZERO(index, prop) \
    BUILD_ASSERT(COND_CODE_0(IS_ENABLED(CONFIG_BMS_IC_CURRENT_MONITORING), (1), \
                             (DT_INST_PROP_OR(index, prop, 0) > 0)), \
                 "Devicetree properties shunt-resistor-uohm and board-max-current " \
                 "must be greater than 0 for CONFIG_BMS_IC_CURRENT_MONITORING=y")

#define BQ769X0_NUM_SECTIONS(index) ((DT_INST_PROP(index, used_cell_count) + 4) / 5)

#define DT_SPEC_AND_COMMA(node_id, prop, idx) ADC_DT_SPEC_GET_BY_IDX(node_id, idx)

#define BQ769X0_INIT(index) \
    static struct bms_ic_bq769x0_data bq769x0_data_##index = { 0 }; \
    BQ769X0_ASSERT_CURRENT_MONITORING_PROP_GREATER_ZERO(index, shunt_resistor_uohm); \
    BQ769X0_ASSERT_CURRENT_MONITORING_PROP_GREATER_ZERO(index, board_max_current); \
    IF_ENABLED( \
        DT_INST_NODE_HAS_PROP(index, io_channels), \
        (static const struct adc_dt_spec bq769x0_extra_ntc_##index[] = { \
             DT_INST_FOREACH_PROP_ELEM_SEP(index, io_channels, DT_SPEC_AND_COMMA, (, )) };)) \
    static const struct bms_ic_bq769x0_config bq769x0_config_##index = { \
        .i2c = I2C_DT_SPEC_INST_GET(index), \
        .alert_gpio = GPIO_DT_SPEC_INST_GET(index, alert_gpios), \
        .ntc_en_gpio = GPIO_DT_SPEC_INST_GET_OR(index, ntc_enable_gpios, { 0 }), \
        .shunt_resistor_uohm = DT_INST_PROP_OR(index, shunt_resistor_uohm, 1000), \
        .board_max_current = DT_INST_PROP_OR(index, board_max_current, 0), \
        .thermistor_beta = (float)DT_INST_PROP(index, thermistor_beta), \
        .used_cell_count = DT_INST_PROP(index, used_cell_count), \
        .num_sections = BQ769X0_NUM_SECTIONS(index), \
        .num_thermistors = MIN(CONFIG_BMS_IC_MAX_THERMISTORS, BQ769X0_NUM_SECTIONS(index)), \
        IF_ENABLED(DT_INST_NODE_HAS_PROP(index, io_channels), \
                   (.extra_ntc_channels = bq769x0_extra_ntc_##index, )) \
            .num_extra_thermistors = DT_INST_PROP_LEN_OR(index, io_channels, 0), \
    }; \
    DEVICE_DT_INST_DEFINE(index, &bq769x0_init, NULL, &bq769x0_data_##index, \
                          &bq769x0_config_##index, POST_KERNEL, CONFIG_BMS_IC_INIT_PRIORITY, \
                          &bq769x0_driver_api);

DT_INST_FOREACH_STATUS_OKAY(BQ769X0_INIT)
