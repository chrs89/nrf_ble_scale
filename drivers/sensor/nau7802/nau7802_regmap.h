/*
 * Copyright (c) 2025 Bauer-IoT.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**************************************************************************/
/*!
    @brief  NAU7802 register definition from Adafruit_NAU7802 library.
    @brief  https://github.com/adafruit/Adafruit_NAU7802/tree/master
*/
/**************************************************************************/

#ifndef __NAU7802_REGMAP_H__
#define __NAU7802_REGMAP_H__

/** Default NAU7802 I2C address. */
#define NAU7802_I2CADDR_DEFAULT 0x2A ///< I2C address
#define NAU7802_PU_CTRL 0x00         ///< Power control register
#define NAU7802_MASK_PU_CTRL_AVDDS BIT(7)
#define NAU7802_SHIFT_PU_CTRL_AVDDS 7
#define NAU7802_MASK_PU_CTRL_PUR BIT(3)
#define NAU7802_SHIFT_PU_CTRL_PUR 3
#define NAU7802_MASK_PU_CTRL_PUA BIT(2)
#define NAU7802_SHIFT_PU_CTRL_PUA 2
#define NAU7802_MASK_PU_CTRL_PUD BIT(1)
#define NAU7802_SHIFT_PU_CTRL_PUD 1
#define NAU7802_MASK_PU_CTRL_RR BIT(0)
#define NAU7802_SHIFT_PU_CTRL_RR 0

#define NAU7802_CTRL1 0x01 ///< Control/config register #1
#define NAU7802_MASK_CTRL1_VLDO (BIT(5) | BIT(4) | BIT(3))
#define NAU7802_SHIFT_CTRL1_VLDO 3
#define NAU7802_MASK_CTRL1_GAINS (BIT(2) | BIT(1) | BIT(0))
#define NAU7802_SHIFT_CTRL1_GAINS 0

#define NAU7802_CTRL2 0x02 ///< Control/config register #2
#define NAU7802_MASK_CTRL2_CRS (BIT(6) | BIT(5) | BIT(4))
#define NAU7802_SHIFT_CTRL2_CRS 4
#define NAU7802_MASK_CTRL2_CAL_ERR BIT(3)
#define NAU7802_SHIFT_CTRL2_CAL_ERR 3
#define NAU7802_MASK_CTRL2_CALS BIT(2)
#define NAU7802_SHIFT_CTRL2_CALS 2
#define NAU7802_MASK_CTRL2_CALMOD (BIT(1) | BIT(0))
#define NAU7802_SHIFT_CTRL2_CALMOD 0

#define NAU7802_ADCO_B2 0x12 ///< ADC ouput LSB

#define NAU7802_ADC 0x15 ///< ADC / chopper control
#define NAU7802_MASK_ADC_REG_CHPS (BIT(5) | BIT(4))
#define NAU7802_SHIFT_ADC_REG_CHPS 4

#define NAU7802_PGA 0x1B ///< PGA control
#define NAU7802_MASK_PGA_LDOMODE BIT(6)
#define NAU7802_SHIFT_PGA_LDOMODE 6

#define NAU7802_POWER 0x1C ///< power control
#define NAU7802_MASK_POWER_PGA_CAP_EN BIT(7)
#define NAU7802_SHIFT_POWER_PGA_CAP_EN 7

#define NAU7802_REVISION_ID 0x1F ///< Chip revision ID

#endif