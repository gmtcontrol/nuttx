/****************************************************************************
 * boards/arm/ra6m5/ek-ra6m5/include/board.h
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

#ifndef __BOARDS_ARM_RA6M5_EK_RA6M5_INCLUDE_BOARD_H
#define __BOARDS_ARM_RA6M5_EK_RA6M5_INCLUDE_BOARD_H

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

/* The EK_RA6M5 board supports both MOSC and SOSC crystals.  
 *
 */

#define RA6M5_HOCO_FREQUENCY    16000000ul      /* High-speed on-chip oscillator */
#define RA6M5_MOCO_FREQUENCY    8000000ul       /* Middle-speed on-chip oscillator */
#define RA6M5_LOCO_FREQUENCY    32768           /* Low-speed on-chip oscillator */

#define RA6M5_MOSC_FREQUENCY    24000000ul      /* High-speed external oscillator */
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
#define RA6M5_CFG_PLL_DIV       (RA6M5_CLOCKS_PLL_DIV_3)            /* PLL Div /3 */
#define RA6M5_CFG_PLL_MUL       (RA6M5_CLOCKS_PLL_MUL_25_0)         /* PLL Mul x25.0 */

#define RA6M5_CFG_PLL2_SOURCE   (RA6M5_CLOCKS_SOURCE_MOSC)          /* PLL2 Src: MOSC */
#define RA6M5_CFG_PLL2_DIV      (RA6M5_CLOCKS_PLL_DIV_3)            /* PLL2 Div /3 */
#define RA6M5_CFG_PLL2_MUL      (RA6M5_CLOCKS_PLL_MUL_24_0)         /* PLL2 Mul x24.0 */

/* Alternate function pin selections ****************************************/

/* SCI6: Connected to XXX */

#define GPIO_SCI6_SDA           GPIO_SCI6_SDA_2                     /* P506   */
#define GPIO_SCI6_SCL           GPIO_SCI6_SCL_2                     /* P505   */

/* SCI7: Connected to XXX */

#define GPIO_SCI7_RX            GPIO_SCI7_RX_2                      /* P614   */
#define GPIO_SCI7_TX            GPIO_SCI7_TX_2                      /* P613   */

/* IIC1: Connected to XXX */

#define GPIO_IIC1_SDA           GPIO_IIC1_SDA_2                     /* P511   */
#define GPIO_IIC1_SCL           GPIO_IIC1_SCL_2                     /* P512   */

/* IIC2: Connected to XXX */

#define GPIO_IIC2_SDA           GPIO_IIC2_SDA_2                     /* P414   */
#define GPIO_IIC2_SCL           GPIO_IIC2_SCL_2                     /* P415   */

/* SPI0: Connected to XXX */

#define GPIO_SPI0_SCK           GPIO_SPI0_SCK_2                     /* P204   */
#define GPIO_SPI0_MISO          GPIO_SPI0_MISO_2                    /* P202   */
#define GPIO_SPI0_MOSI          GPIO_SPI0_MOSI_2                    /* P203   */
#define GPIO_SPI0_SEL0          (GPIO_OUTPUT|GPIO_OUTPUT_SET|GPIO_PORT2|GPIO_PIN5)  /* P205 */
#define GPIO_SPI0_SEL1          (GPIO_OUTPUT|GPIO_OUTPUT_SET|GPIO_PORT2|GPIO_PIN6)  /* P206 */
#define GPIO_SPI0_SEL2          (GPIO_OUTPUT|GPIO_OUTPUT_SET|GPIO_PORT3|GPIO_PIN1)  /* P301 */
#define GPIO_SPI0_SEL3          (GPIO_OUTPUT|GPIO_OUTPUT_SET|GPIO_PORT3|GPIO_PIN2)  /* P302 */

/* SPI1: Connected to XXX */

#define GPIO_SPI1_SCK           GPIO_SPI1_SCK_2                     /* P412   */
#define GPIO_SPI1_MISO          GPIO_SPI1_MISO_2                    /* P410   */
#define GPIO_SPI1_MOSI          GPIO_SPI1_MOSI_2                    /* P411   */
#define GPIO_SPI1_SEL0          (GPIO_OUTPUT|GPIO_OUTPUT_SET|GPIO_PORT4|GPIO_PIN13) /* P413 */
#define GPIO_SPI1_SEL3          (GPIO_OUTPUT|GPIO_OUTPUT_SET|GPIO_PORT7|GPIO_PIN8)  /* P708 */

/* QSPI: Connected to XXX */

#define GPIO_QSPI_SCK           GPIO_QSPI_SCK_3                     /* P305   */
#define GPIO_QSPI_SSL           GPIO_QSPI_SSL_3                     /* P306   */
#define GPIO_QSPI_IO0           GPIO_QSPI_IO0_3                     /* P307   */
#define GPIO_QSPI_IO1           GPIO_QSPI_IO1_3                     /* P308   */
#define GPIO_QSPI_IO2           GPIO_QSPI_IO2_3                     /* P309   */
#define GPIO_QSPI_IO3           GPIO_QSPI_IO3_3                     /* P310   */

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
#endif  /* __BOARDS_ARM_RA6M5_EK_RA6M5_INCLUDE_BOARD_H */
