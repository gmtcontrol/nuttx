/****************************************************************************
 * boards/arm/stm32/glcx96/include/board.h
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

#ifndef __BOARD_ARM_STM32_GLCX96_INCLUDE_BOARD_H
#define __BOARD_ARM_STM32_GLCX96_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/* Logic in arch/arm/src and boards/ may need to include these file prior to
 * including board.h:  stm32_rcc.h, stm32_sdio.h, stm32.h.  They cannot be
 * included here because board.h is used in other contexts where the STM32
 * internal header files are not available.
 */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* Four clock sources are available on GLCX96 PLC for STM32F407VIGT6 
 * and RTC embedded:
 *
 * X1, 25 MHz crystal for Ethernet PHY with socket.
 * X2, 26 MHz crystal for USB OTG HS PHY
 * X3, 32 kHz crystal for embedded RTC
 * X4, 25 MHz crystal with socket for STM32F407IGH6 microcontroller
 *     (It can be removed from socket when internal RC clock is used.)
 *
 * This is the "standard" configuration as set up by
 * arch/arm/src/stm32f40xx_rcc.c:
 *   System Clock source           : PLL (HSE)
 *   SYSCLK(Hz)                    : 168000000    Determined by PLL
 *                                                 configuration
 *   HCLK(Hz)                      : 168000000    (STM32_RCC_CFGR_HPRE)
 *   AHB Prescaler                 : 1            (STM32_RCC_CFGR_HPRE)
 *   APB1 Prescaler                : 4            (STM32_RCC_CFGR_PPRE1)
 *   APB2 Prescaler                : 2            (STM32_RCC_CFGR_PPRE2)
 *   HSE Frequency(Hz)             : 16000000     (STM32_BOARD_XTAL)
 *   PLLM                          : 16           (STM32_PLLCFG_PLLM)
 *   PLLN                          : 336          (STM32_PLLCFG_PLLN)
 *   PLLP                          : 2            (STM32_PLLCFG_PLLP)
 *   PLLQ                          : 7            (STM32_PLLCFG_PLLQ)
 *   Main regulator output voltage : Scale1 mode  Needed for high speed
 *                                                SYSCLK
 *   Flash Latency(WS)             : 5
 *   Prefetch Buffer               : OFF
 *   Instruction cache             : ON
 *   Data cache                    : ON
 *   Require 48MHz for USB OTG FS, : Enabled
 *   SDIO and RNG clock
 */

/* HSI - 16 MHz RC factory-trimmed
 * LSI - 32 KHz RC
 * HSE - On-board crystal frequency is 16MHz
 * LSE - 32.768 kHz
 */

#define STM32_BOARD_XTAL        16000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768

/* Main PLL Configuration.
 *
 * PLL source is HSE
 * PLL_VCO = (STM32_HSE_FREQUENCY / PLLM) * PLLN
 *         = (16,000,000 / 16) * 336
 *         = 336,000,000
 * SYSCLK  = PLL_VCO / PLLP
 *         = 336,000,000 / 2 = 168,000,000
 * USB OTG FS, SDIO and RNG Clock
 *         =  PLL_VCO / PLLQ
 *         = 48,000,000
 */

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(16)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(336)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_2
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(7)

#define STM32_SYSCLK_FREQUENCY  168000000ul

/* AHB clock (HCLK) is SYSCLK (168MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK  /* HCLK  = SYSCLK / 1 */
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY

/* APB1 clock (PCLK1) is HCLK/4 (42MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd4     /* PCLK1 = HCLK / 4 */
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/4)

/* Timers driven from APB1 will be twice PCLK1 */

#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM12_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM13_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM14_CLKIN  (2*STM32_PCLK1_FREQUENCY)

/* APB2 clock (PCLK2) is HCLK/2 (84MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLKd2     /* PCLK2 = HCLK / 2 */
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* Timers driven from APB2 will be twice PCLK2 */

#define STM32_APB2_TIM1_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM9_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM10_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM11_CLKIN  (2*STM32_PCLK2_FREQUENCY)

/* Timer Frequencies, if APBx is set to 1, frequency is same as APBx
 * otherwise frequency is 2xAPBx.
 * Note: TIM1,8-11 are on APB2, others on APB1
 */

#define BOARD_TIM2_FREQUENCY    STM32_APB1_TIM2_CLKIN
#define BOARD_TIM3_FREQUENCY    STM32_APB1_TIM3_CLKIN
#define BOARD_TIM4_FREQUENCY    STM32_APB1_TIM4_CLKIN
#define BOARD_TIM5_FREQUENCY    STM32_APB1_TIM5_CLKIN
#define BOARD_TIM6_FREQUENCY    STM32_APB1_TIM6_CLKIN
#define BOARD_TIM7_FREQUENCY    STM32_APB1_TIM7_CLKIN
#define BOARD_TIM12_FREQUENCY   STM32_APB1_TIM12_CLKIN
#define BOARD_TIM13_FREQUENCY   STM32_APB1_TIM13_CLKIN
#define BOARD_TIM14_FREQUENCY   STM32_APB1_TIM14_CLKIN

#define BOARD_TIM1_FREQUENCY    STM32_APB2_TIM1_CLKIN
#define BOARD_TIM8_FREQUENCY    STM32_APB2_TIM8_CLKIN
#define BOARD_TIM9_FREQUENCY    STM32_APB2_TIM9_CLKIN
#define BOARD_TIM10_FREQUENCY   STM32_APB2_TIM10_CLKIN
#define BOARD_TIM11_FREQUENCY   STM32_APB2_TIM11_CLKIN


/* Ethernet *****************************************************************/

/* We need to provide clocking to the MII PHY via MCO1 (PA8) */

#if defined(CONFIG_NET) && defined(CONFIG_STM32_ETHMAC)

#  if !defined(CONFIG_STM32_RMII)
#    warning "CONFIG_STM32_RMII required for Ethernet"
#  endif
#endif

/* LED definitions **********************************************************/

/* If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
 * any way.  The following definitions are used to access individual LEDs.
 */

/* LED index values for use with board_userled() */

#define BOARD_RUN         0
#define BOARD_ERR         1
#define BOARD_COM         2
#define BOARD_NLEDS       4

/* LED bits for use with board_userled_all() */

#define BOARD_RUN_BIT     (1 << BOARD_RUN)
#define BOARD_ERR_BIT     (1 << BOARD_ERR)
#define BOARD_COM_BIT     (1 << BOARD_COM)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the (3) LEDs on
 * board the GLCX96.
 * The following definitions describe how NuttX controls the LEDs:
 */

#define LED_STARTED       0  /* LED1 */
#define LED_HEAPALLOCATE  1  /* LED2 */
#define LED_IRQSENABLED   2  /* LED1 + LED2 */
#define LED_STACKCREATED  3  /* LED3 */
#define LED_INIRQ         4  /* LED1 + LED3 */
#define LED_SIGNAL        5  /* LED2 + LED3 */
#define LED_ASSERTION     6  /* LED1 + LED2 + LED3 */
#define LED_PANIC         7  /* N/C  + N/C  + N/C + LED4 */

/* Button definitions *******************************************************/

/* The GLCX96 supports (1) buttons: */

#define BUTTON_RUN        0
#define NUM_BUTTONS       1

#define BUTTON_RUN_BIT    (1 << BUTTON_RUN)

/* Alternate function pin selections ****************************************/

/* Ethernet MAC */

#define GPIO_ETH_MDC          GPIO_ETH_MDC_0
#define GPIO_ETH_MDIO         GPIO_ETH_MDIO_0
#define GPIO_ETH_RMII_CRS_DV  GPIO_ETH_RMII_CRS_DV_0
#define GPIO_ETH_RMII_REF_CLK GPIO_ETH_RMII_REF_CLK_0
#define GPIO_ETH_RMII_TX_EN   GPIO_ETH_RMII_TX_EN_1
#define GPIO_ETH_RMII_TXD0    GPIO_ETH_RMII_TXD0_1
#define GPIO_ETH_RMII_TXD1    GPIO_ETH_RMII_TXD1_1
#define GPIO_ETH_RMII_RXD0    GPIO_ETH_RMII_RXD0_0
#define GPIO_ETH_RMII_RXD1    GPIO_ETH_RMII_RXD1_0

#define GPIO_KSZ8081_PWR      (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN9)
#define GPIO_KSZ8081_RST      (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN10)
#define GPIO_KSZ8081_INT      (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_50MHz|GPIO_PORTD|GPIO_PIN11)

/* OTG */

#define GPIO_OTGFS_DM   (GPIO_OTGFS_DM_0|GPIO_SPEED_100MHz)   /* PA11 */
#define GPIO_OTGFS_DP   (GPIO_OTGFS_DP_0|GPIO_SPEED_100MHz)   /* PA12 */
#define GPIO_OTGFS_ID   (GPIO_OTGFS_ID_0|GPIO_SPEED_100MHz)   /* PA10 */

/* USART1 : RS232 port */

#define GPIO_USART1_RX  GPIO_USART1_RX_2
#define GPIO_USART1_TX  GPIO_USART1_TX_2

/* USART3 : RS485 port */

#define GPIO_USART3_RX  GPIO_USART3_RX_3
#define GPIO_USART3_TX  GPIO_USART3_TX_3
#define GPIO_USART3_RS485_DIR (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN15)

/* SPI1 : NOR flash */

#define GPIO_SPI1_SCK   GPIO_SPI1_SCK_2
#define GPIO_SPI1_MOSI  GPIO_SPI1_MOSI_2
#define GPIO_SPI1_MISO  GPIO_SPI1_MISO_2
#define GPIO_SPI1_NSS   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN7)

/* SPI3 : EXP bus */

#define GPIO_SPI3_SCK   GPIO_SPI3_SCK_2
#define GPIO_SPI3_MOSI  GPIO_SPI3_MOSI_2
#define GPIO_SPI3_MISO  GPIO_SPI3_MISO_2

/* PWM(s) */

#define GPIO_TIM13_CH1OUT   GPIO_TIM13_CH1OUT_1     /* PA6 - HSO0 */
#define GPIO_TIM2_CH1OUT    GPIO_TIM2_CH1OUT_1      /* PA0 - HSO1 */
#define GPIO_TIM11_CH1OUT   GPIO_TIM11_CH1OUT_1     /* PB9 - HSO2 */
#define GPIO_TIM3_CH4OUT    GPIO_TIM3_CH4OUT_2      /* PC9 - AOUT */

/* DMA Channel/Stream Selections ********************************************/

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

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARD_ARM_STM32_GLCX96_INCLUDE_BOARD_H */
