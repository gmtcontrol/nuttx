/****************************************************************************
 * boards/arm/renesas/gmtplc/glc23x/include/board.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __BOARDS_ARM_RENESAS_GMTPLC_GLC23X_INCLUDE_BOARD_H
#define __BOARDS_ARM_RENESAS_GMTPLC_GLC23X_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <ra6m5_gpio.h>
#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The GLC23X board supports both MOSC and SOSC crystals.  
 *
 */

#define RA6M5_HOCO_FREQUENCY    16000000ul      /* High-speed on-chip oscillator */
#define RA6M5_MOCO_FREQUENCY    8000000ul       /* Middle-speed on-chip oscillator */
#define RA6M5_LOCO_FREQUENCY    32768           /* Low-speed on-chip oscillator */

#define RA6M5_MOSC_FREQUENCY    16000000ul      /* High-speed external oscillator */
#define RA6M5_SOSC_FREQUENCY    32768           /* Sub-clock oscillator */

/* PLL config; we use this to generate our system clock */

#define RA6M5_SYSCLK_FREQUENCY  200000000ul

#define RA6M5_CFG_ICLK_DIV      (RA6M5_CLOCKS_SYS_CLOCK_DIV_1)      /* ICLK  Div /1 */
#define RA6M5_CFG_PCLKA_DIV     (RA6M5_CLOCKS_SYS_CLOCK_DIV_2)      /* PCLKA Div /2 */
#define RA6M5_CFG_PCLKB_DIV     (RA6M5_CLOCKS_SYS_CLOCK_DIV_4)      /* PCLKB Div /4 */
#define RA6M5_CFG_PCLKC_DIV     (RA6M5_CLOCKS_SYS_CLOCK_DIV_4)      /* PCLKC Div /4 */
#define RA6M5_CFG_PCLKD_DIV     (RA6M5_CLOCKS_SYS_CLOCK_DIV_2)      /* PCLKD Div /2 */
#define RA6M5_CFG_BCLK_DIV      (RA6M5_CLOCKS_SYS_CLOCK_DIV_2)      /* BCLK  Div /2 */
#define RA6M5_CFG_FCLK_DIV      (RA6M5_CLOCKS_SYS_CLOCK_DIV_4)      /* FCLK  Div /4 */

#define RA6M5_CFG_PLL_SOURCE    (RA6M5_CLOCKS_SOURCE_MOSC)          /* PLL Src: MOSC */
#define RA6M5_CFG_PLL_DIV       (RA6M5_CLOCKS_PLL_DIV_2)            /* PLL Div /2 */
#define RA6M5_CFG_PLL_MUL       (RA6M5_CLOCKS_PLL_MUL_25_0)         /* PLL Mul x25.0 */

#define RA6M5_CFG_PLL2_SOURCE   (RA6M5_CLOCKS_SOURCE_MOSC)          /* PLL2 Src: MOSC */
#define RA6M5_CFG_PLL2_DIV      (RA6M5_CLOCKS_PLL_DIV_2)            /* PLL2 Div /2 */
#define RA6M5_CFG_PLL2_MUL      (RA6M5_CLOCKS_PLL_MUL_24_0)         /* PLL2 Mul x24.0 */

/* Alternate function pin selections ****************************************/

/* SCI0: Connected to RS485 #0 Port */

#define GPIO_SCI0_RX            GPIO_SCI0_RX_1                      /* P100   */
#define GPIO_SCI0_TX            GPIO_SCI0_TX_1                      /* P101   */
#define GPIO_SCI0_RS485_DIR     (GPIO_OUTPUT|GPIO_PORT1|GPIO_PIN2)  /* P102   */

/* SCI2: Connected to RS485 #1 Port */

#define GPIO_SCI2_RX            GPIO_SCI2_RX_2                      /* P301   */
#define GPIO_SCI2_TX            GPIO_SCI2_TX_2                      /* P302   */
#define GPIO_SCI2_RS485_DIR     (GPIO_OUTPUT|GPIO_PORT3|GPIO_PIN3)  /* P303   */

/* SCI8: Connected to RS232 Port */

#define GPIO_SCI8_RX            GPIO_SCI8_RX_1                      /* P104   */
#define GPIO_SCI8_TX            GPIO_SCI8_TX_1                      /* P105   */

/* SCI9: Connected to SPI Flash Port */

#define GPIO_SCI9_SDI           GPIO_SCI9_RX_2                      /* P601   */
#define GPIO_SCI9_SDO           GPIO_SCI9_TX_2                      /* P602   */
#define GPIO_SCI9_SCK           GPIO_SCI9_CK_2                      /* P600   */
#define GPIO_SCI9_SEL           (GPIO_OUTPUT|GPIO_OUTPUT_SET|GPIO_PORT1|GPIO_PIN7)  /* P107   */

/* IIC1: Connected to LED driver */

#define GPIO_IIC1_SDA           GPIO_IIC1_SDA_1                     /* P206   */
#define GPIO_IIC1_SCL           GPIO_IIC1_SCL_1                     /* P205   */

/* USBFS: VBUS Connected to P407 */

#define GPIO_OTGFS_VBUS         GPIO_USBFS_VBUS_1                   /* P407   */

/* EMAC: */

#define GPIO_ETH_MDC            GPIO_RMII_MDC                       /* P401   */
#define GPIO_ETH_MDIO           GPIO_RMII_MDIO                      /* P402   */
#define GPIO_ETH_REFCLKI        GPIO_RMII_REFCLKI                   /* P412   */
#define GPIO_ETH_TXD0           GPIO_RMII_TXD0                      /* P413   */
#define GPIO_ETH_TXD1           GPIO_RMII_TXD1                      /* P414   */
#define GPIO_ETH_TX_EN          GPIO_RMII_TX_EN                     /* P415   */
#define GPIO_ETH_RXD0           GPIO_RMII_RXD0                      /* P411   */
#define GPIO_ETH_RXD1           GPIO_RMII_RXD1                      /* P410   */
#define GPIO_ETH_RX_ER          GPIO_RMII_RX_ER                     /* P409   */
#define GPIO_ETH_CRS_DV         GPIO_RMII_CRS_DV                    /* P408   */

#define GPIO_ETH_PWR            (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_OUTPUT_SET|GPIO_PORT4|GPIO_PIN4)   /* P404   */
#define GPIO_ETH_RST            (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_OUTPUT_SET|GPIO_PORT4|GPIO_PIN6)   /* P406   */
#define GPIO_ETH_INT            (GPIO_INPUT |GPIO_FLOAT    |GPIO_PORT4|GPIO_PIN5)                   /* P405   */

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ra6m5_board_initialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices
 *   have been initialized.
 *
 ****************************************************************************/

void ra6m5_board_initialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __BOARDS_ARM_RENESAS_GMTPLC_GLC23X_INCLUDE_BOARD_H */
