/*
 * Copyright (c) The Libre Solar Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef INCLUDE_ZEPHYR_BMS_BMS_COMMON_H_
#define INCLUDE_ZEPHYR_BMS_BMS_COMMON_H_

#include <zephyr/sys/util.h>
/**
 * @file
 * @brief Battery Management System (BMS) common defines and structs
 */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * BMS switches (MOSFETs or contactors)
 */
#define BMS_SWITCH_CHG  BIT(0)
#define BMS_SWITCH_DIS  BIT(1)
#define BMS_SWITCH_PDSG BIT(2)
#define BMS_SWITCH_PCHG BIT(3)

/*
 * BMS error flags
 */
typedef enum
{
    BMS_ERR_NONE = 0,
    BMS_ERR_CELL_UNDERVOLTAGE = BIT(0), ///< Cell undervoltage flag
    BMS_ERR_CELL_OVERVOLTAGE = BIT(1),  ///< Cell overvoltage flag
    BMS_ERR_SHORT_CIRCUIT = BIT(2),     ///< Pack short circuit (discharge direction)
    BMS_ERR_DIS_OVERCURRENT = BIT(3),   ///< Pack overcurrent (discharge direction)
    BMS_ERR_CHG_OVERCURRENT = BIT(4),   ///< Pack overcurrent (charge direction)
    BMS_ERR_OPEN_WIRE = BIT(5),         ///< Cell open wire
    BMS_ERR_DIS_UNDERTEMP = BIT(6),     ///< Temperature below discharge minimum limit
    BMS_ERR_DIS_OVERTEMP = BIT(7),      ///< Temperature above discharge maximum limit
    BMS_ERR_CHG_UNDERTEMP = BIT(8),     ///< Temperature below charge maximum limit
    BMS_ERR_CHG_OVERTEMP = BIT(9),      ///< Temperature above charge maximum limit
    BMS_ERR_INT_OVERTEMP = BIT(10),     ///< Internal temperature above limit (e.g. BMS IC)
    BMS_ERR_CELL_FAILURE = BIT(11),     ///< Cell failure (too high voltage difference)
    BMS_ERR_DIS_OFF = BIT(12),          ///< Discharge FET is off even though it should be on
    BMS_ERR_CHG_OFF = BIT(13),          ///< Charge FET is off even though it should be on
    BMS_ERR_FET_OVERTEMP = BIT(14),     ///< MOSFET temperature above limit
    BMS_ERR_IC = BIT(15),               ///< BMS IC fault
    BMS_ERR_UNDERTEMP = (BMS_ERR_DIS_UNDERTEMP | BMS_ERR_CHG_UNDERTEMP),
    BMS_ERR_OVERTEMP = (BMS_ERR_DIS_OVERTEMP | BMS_ERR_CHG_OVERTEMP),
    BMS_ERR_ALL = GENMASK(15, 0)
} bms_error_flags_t;

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_ZEPHYR_BMS_BMS_COMMON_H_ */
