/*
 * Copyright 2018-2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v5.0
processor: MIMXRT1062xxxxA
package_id: MIMXRT1062DVL6A
mcu_data: ksdk2_0
processor_version: 0.0.18
pin_labels:
- {pin_num: C14, pin_signal: GPIO_B1_14, label: SD0_VSELECT, identifier: SD0_VSELECT}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "fsl_gpio.h"
#include "pin_mux.h"

/* FUNCTION ************************************************************************************************************
 * 
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 * 
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void) {
    BOARD_InitPins();
}

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: M12, peripheral: GPIO1, signal: 'gpio_io, 19', pin_signal: GPIO_AD_B1_03, software_input_on: Disable, hysteresis_enable: Disable, pull_up_down_config: Pull_Down_100K_Ohm,
    pull_keeper_select: Keeper, pull_keeper_enable: Enable, open_drain: Disable, speed: MHZ_100, drive_strength: R0_6, slew_rate: Slow}
  - {pin_num: D13, peripheral: GPIO2, signal: 'gpio_io, 28', pin_signal: GPIO_B1_12, software_input_on: Disable, hysteresis_enable: Enable, pull_up_down_config: Pull_Up_47K_Ohm,
    pull_keeper_select: Pull, pull_keeper_enable: Enable, open_drain: Disable, speed: MHZ_100, drive_strength: R0, slew_rate: Fast}
  - {pin_num: L14, peripheral: LPUART1, signal: RX, pin_signal: GPIO_AD_B0_13, software_input_on: Disable, hysteresis_enable: Disable, pull_up_down_config: Pull_Down_100K_Ohm,
    pull_keeper_select: Keeper, pull_keeper_enable: Enable, open_drain: Disable, speed: MHZ_100, drive_strength: R0_6, slew_rate: Slow}
  - {pin_num: K14, peripheral: LPUART1, signal: TX, pin_signal: GPIO_AD_B0_12, software_input_on: Disable, hysteresis_enable: Disable, pull_up_down_config: Pull_Down_100K_Ohm,
    pull_keeper_select: Keeper, pull_keeper_enable: Enable, open_drain: Disable, speed: MHZ_100, drive_strength: R0_6, slew_rate: Slow}
  - {pin_num: J3, peripheral: USDHC1, signal: usdhc_clk, pin_signal: GPIO_SD_B0_01, software_input_on: Disable, hysteresis_enable: Disable, pull_up_down_config: Pull_Up_100K_Ohm,
    pull_keeper_select: Keeper, pull_keeper_enable: Disable, open_drain: Disable, speed: MHZ_100, drive_strength: R0_7, slew_rate: Fast}
  - {pin_num: J4, peripheral: USDHC1, signal: usdhc_cmd, pin_signal: GPIO_SD_B0_00, software_input_on: Disable, hysteresis_enable: Disable, pull_up_down_config: Pull_Up_100K_Ohm,
    pull_keeper_select: Pull, pull_keeper_enable: Enable, open_drain: Disable, speed: MHZ_100, drive_strength: R0_7, slew_rate: Fast}
  - {pin_num: J1, peripheral: USDHC1, signal: 'usdhc_data, 0', pin_signal: GPIO_SD_B0_02, software_input_on: Disable, hysteresis_enable: Disable, pull_up_down_config: Pull_Up_100K_Ohm,
    pull_keeper_select: Pull, pull_keeper_enable: Enable, open_drain: Disable, speed: MHZ_100, drive_strength: R0_7, slew_rate: Fast}
  - {pin_num: K1, peripheral: USDHC1, signal: 'usdhc_data, 1', pin_signal: GPIO_SD_B0_03, software_input_on: Disable, hysteresis_enable: Disable, pull_up_down_config: Pull_Up_100K_Ohm,
    pull_keeper_select: Pull, pull_keeper_enable: Enable, open_drain: Disable, speed: MHZ_100, drive_strength: R0_7, slew_rate: Fast}
  - {pin_num: H2, peripheral: USDHC1, signal: 'usdhc_data, 2', pin_signal: GPIO_SD_B0_04, software_input_on: Disable, hysteresis_enable: Disable, pull_up_down_config: Pull_Up_100K_Ohm,
    pull_keeper_select: Pull, pull_keeper_enable: Enable, open_drain: Disable, speed: MHZ_100, drive_strength: R0_7, slew_rate: Fast}
  - {pin_num: J2, peripheral: USDHC1, signal: 'usdhc_data, 3', pin_signal: GPIO_SD_B0_05, software_input_on: Disable, hysteresis_enable: Disable, pull_up_down_config: Pull_Up_100K_Ohm,
    pull_keeper_select: Pull, pull_keeper_enable: Enable, open_drain: Disable, speed: MHZ_100, drive_strength: R0_7, slew_rate: Fast}
  - {pin_num: C14, peripheral: GPIO2, signal: 'gpio_io, 30', pin_signal: GPIO_B1_14, direction: OUTPUT, gpio_init_state: 'true', software_input_on: Disable, hysteresis_enable: Enable,
    pull_up_down_config: Pull_Up_47K_Ohm, pull_keeper_select: Pull, pull_keeper_enable: Enable, open_drain: Disable, speed: MHZ_100, drive_strength: R0_4, slew_rate: Fast}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitPins(void) {
  CLOCK_EnableClock(kCLOCK_Iomuxc);           /* iomuxc clock (iomuxc_clk_enable): 0x03U */

  /* GPIO configuration of SD0_VSELECT on GPIO_B1_14 (pin C14) */
  gpio_pin_config_t SD0_VSELECT_config = {
      .direction = kGPIO_DigitalOutput,
      .outputLogic = 1U,
      .interruptMode = kGPIO_NoIntmode
  };
  /* Initialize GPIO functionality on GPIO_B1_14 (pin C14) */
  GPIO_PinInit(GPIO2, 30U, &SD0_VSELECT_config);

  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_B0_12_LPUART1_TX,        /* GPIO_AD_B0_12 is configured as LPUART1_TX */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_B0_13_LPUART1_RX,        /* GPIO_AD_B0_13 is configured as LPUART1_RX */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_B1_03_GPIO1_IO19,        /* GPIO_AD_B1_03 is configured as GPIO1_IO19 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_B1_12_GPIO2_IO28,           /* GPIO_B1_12 is configured as GPIO2_IO28 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_B1_14_GPIO2_IO30,           /* GPIO_B1_14 is configured as GPIO2_IO30 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_SD_B0_00_USDHC1_CMD,        /* GPIO_SD_B0_00 is configured as USDHC1_CMD */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_SD_B0_01_USDHC1_CLK,        /* GPIO_SD_B0_01 is configured as USDHC1_CLK */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_SD_B0_02_USDHC1_DATA0,      /* GPIO_SD_B0_02 is configured as USDHC1_DATA0 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_SD_B0_03_USDHC1_DATA1,      /* GPIO_SD_B0_03 is configured as USDHC1_DATA1 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_SD_B0_04_USDHC1_DATA2,      /* GPIO_SD_B0_04 is configured as USDHC1_DATA2 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_SD_B0_05_USDHC1_DATA3,      /* GPIO_SD_B0_05 is configured as USDHC1_DATA3 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_GPR->GPR26 = ((IOMUXC_GPR->GPR26 &
    (~(IOMUXC_GPR_GPR26_GPIO_MUX1_GPIO_SEL_MASK))) /* Mask bits to zero which are setting */
      | IOMUXC_GPR_GPR26_GPIO_MUX1_GPIO_SEL(0x00U) /* GPIO1 and GPIO6 share same IO MUX function, GPIO_MUX1 selects one GPIO function: 0x00U */
    );
  IOMUXC_GPR->GPR27 = ((IOMUXC_GPR->GPR27 &
    (~(IOMUXC_GPR_GPR27_GPIO_MUX2_GPIO_SEL_MASK))) /* Mask bits to zero which are setting */
      | IOMUXC_GPR_GPR27_GPIO_MUX2_GPIO_SEL(0x00U) /* GPIO2 and GPIO7 share same IO MUX function, GPIO_MUX2 selects one GPIO function: 0x00U */
    );
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AD_B0_12_LPUART1_TX,        /* GPIO_AD_B0_12 PAD functional properties : */
      0x10B0U);                               /* Slew Rate Field: Slow Slew Rate
                                                 Drive Strength Field: R0/6
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Keeper
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Down
                                                 Hyst. Enable Field: Hysteresis Disabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AD_B0_13_LPUART1_RX,        /* GPIO_AD_B0_13 PAD functional properties : */
      0x10B0U);                               /* Slew Rate Field: Slow Slew Rate
                                                 Drive Strength Field: R0/6
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Keeper
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Down
                                                 Hyst. Enable Field: Hysteresis Disabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AD_B1_03_GPIO1_IO19,        /* GPIO_AD_B1_03 PAD functional properties : */
      0x10B0U);                               /* Slew Rate Field: Slow Slew Rate
                                                 Drive Strength Field: R0/6
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Keeper
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Down
                                                 Hyst. Enable Field: Hysteresis Disabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_B1_12_GPIO2_IO28,           /* GPIO_B1_12 PAD functional properties : */
      0x017089U);                             /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0(150 Ohm @ 3.3V, 260 Ohm@1.8V)
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Pull
                                                 Pull Up / Down Config. Field: 47K Ohm Pull Up
                                                 Hyst. Enable Field: Hysteresis Enabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_B1_14_GPIO2_IO30,           /* GPIO_B1_14 PAD functional properties : */
      0x0170A1U);                             /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0/4
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Pull
                                                 Pull Up / Down Config. Field: 47K Ohm Pull Up
                                                 Hyst. Enable Field: Hysteresis Enabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_SD_B0_00_USDHC1_CMD,        /* GPIO_SD_B0_00 PAD functional properties : */
      0xB0B9U);                               /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0/7
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Pull
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Up
                                                 Hyst. Enable Field: Hysteresis Disabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_SD_B0_01_USDHC1_CLK,        /* GPIO_SD_B0_01 PAD functional properties : */
      0x80B9U);                               /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0/7
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Disabled
                                                 Pull / Keep Select Field: Keeper
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Up
                                                 Hyst. Enable Field: Hysteresis Disabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_SD_B0_02_USDHC1_DATA0,      /* GPIO_SD_B0_02 PAD functional properties : */
      0xB0B9U);                               /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0/7
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Pull
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Up
                                                 Hyst. Enable Field: Hysteresis Disabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_SD_B0_03_USDHC1_DATA1,      /* GPIO_SD_B0_03 PAD functional properties : */
      0xB0B9U);                               /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0/7
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Pull
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Up
                                                 Hyst. Enable Field: Hysteresis Disabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_SD_B0_04_USDHC1_DATA2,      /* GPIO_SD_B0_04 PAD functional properties : */
      0xB0B9U);                               /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0/7
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Pull
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Up
                                                 Hyst. Enable Field: Hysteresis Disabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_SD_B0_05_USDHC1_DATA3,      /* GPIO_SD_B0_05 PAD functional properties : */
      0xB0B9U);                               /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0/7
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Pull
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Up
                                                 Hyst. Enable Field: Hysteresis Disabled */
}

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
