/****************************************************************************
 * boards/arm/stm32/ais-tbcs/include/board.h
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

#ifndef __BOARDS_ARM_STM32_AIS_TBCS_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32_AIS_TBCS_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* HSI - Internal 8 MHz RC Oscillator
 * LSI - 32 KHz RC
 * HSE - 16 MHz XTAL
 * LSE - 32.768 kHz
 */

#define STM32_BOARD_XTAL        16000000ul          /* X1 on board */

#define STM32_HSI_FREQUENCY     8000000ul
#define STM32_LSI_FREQUENCY     40000               /* Between 30kHz and 60kHz */
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768               /* X2 on board */

/* PLL source is HSE/1, PLL multipler is 4:
 *   PLL frequency is 16MHz (XTAL) x 4 = 64MHz
 */

#define STM32_CFGR_PLLSRC       RCC_CFGR_PLLSRC
#define STM32_CFGR_PLLXTPRE     0
#define STM32_CFGR_PLLMUL       RCC_CFGR_PLLMUL_CLKx4
#define STM32_PLL_FREQUENCY     (4*STM32_BOARD_XTAL)

/* Use the PLL and set the SYSCLK source to be the PLL */

#define STM32_SYSCLK_SW         RCC_CFGR_SW_PLL
#define STM32_SYSCLK_SWS        RCC_CFGR_SWS_PLL
#define STM32_SYSCLK_FREQUENCY  STM32_PLL_FREQUENCY

/* AHB clock (HCLK) is SYSCLK (64MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY

/* APB2 clock (PCLK2) is HCLK (64MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLK
#define STM32_PCLK2_FREQUENCY   STM32_HCLK_FREQUENCY
#define STM32_APB2_CLKIN        (STM32_PCLK2_FREQUENCY)   /* Timers 1 and 8, 15-17 */

/* APB1 clock (PCLK1) is HCLK/2 (32MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd2
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* APB2 TIM 1 will receive PCLK2 (64MHz) */

#define STM32_APB2_TIM1_CLKIN   (STM32_PCLK2_FREQUENCY)

/* APB1 TIM 2-4 will be twice PCLK1 (64MHz) */

#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)

/* LED definitions **********************************************************/

/* The AIS-TBCS board supports 4 LEDs.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LED in
 * any way.  The following definition is used to access the LED.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED0       0  /* LED Alarm    */
#define BOARD_LED1       1  /* LED Logging  */
#define BOARD_LED2       2  /* LED Power    */
#define BOARD_LED3       3  /* LED SAR      */
#define BOARD_NLEDS      4

/* LED bits for use with board_userled_all() */

#define BOARD_LED0_BIT   (1 << BOARD_LED0)
#define BOARD_LED1_BIT   (1 << BOARD_LED1)
#define BOARD_LED2_BIT   (1 << BOARD_LED2)
#define BOARD_LED3_BIT   (1 << BOARD_LED3)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the LED on board
 * the AIS-TBCS.  The following definitions describe how NuttX controls
 * the LED:
 *
 *   SYMBOL              Meaning                  LED1 state
 *   ------------------  -----------------------  ----------
 *   LED_STARTED         NuttX has been started   OFF
 *   LED_HEAPALLOCATE    Heap has been allocated  OFF
 *   LED_IRQSENABLED     Interrupts enabled       OFF
 *   LED_STACKCREATED    Idle stack created       ON
 *   LED_INIRQ           In an interrupt          No change
 *   LED_SIGNAL          In a signal handler      No change
 *   LED_ASSERTION       An assertion failed      No change
 *   LED_PANIC           The system has crashed   Blinking
 *   LED_IDLE            STM32 is is sleep mode   Not used
 */

#define LED_STARTED      0
#define LED_HEAPALLOCATE 0
#define LED_IRQSENABLED  0
#define LED_STACKCREATED 1
#define LED_INIRQ        2
#define LED_SIGNAL       2
#define LED_ASSERTION    2
#define LED_PANIC        1

/* Button definitions *******************************************************/

/* 
 * The AIS-TBCS supports 4 buttons
 * 
 */

#define BUTTON_KEY0     0   /* KEY Accident */
#define BUTTON_KEY1     1   /* KEY Firet    */
#define BUTTON_KEY2     2   /* KEY Health   */
#define BUTTON_KEY3     3   /* KEY Security */
#define NUM_BUTTONS     4

#define BUTTON_KEY0_BIT (1 << BUTTON_KEY0)
#define BUTTON_KEY1_BIT (1 << BUTTON_KEY1)
#define BUTTON_KEY2_BIT (1 << BUTTON_KEY2)
#define BUTTON_KEY3_BIT (1 << BUTTON_KEY3)


/* Alternate function pin selections ****************************************/

/* 
 * UART(s)
 *
 */

/* USART1 - USER interface port */

#define GPIO_USART1_RX  GPIO_USART1_RX_0    /* PA10 */
#define GPIO_USART1_TX  GPIO_USART1_TX_0    /* PA9 */

/* USART2 - 1-wire sensor port */

#define GPIO_USART2_RX  GPIO_USART2_RX_0    /* PA3 */
#define GPIO_USART2_TX  GPIO_USART2_TX_0    /* PA2 */

/* USART3 - RF interface port */

#define GPIO_USART3_RX  GPIO_USART3_RX_0    /* PB11 */
#define GPIO_USART3_TX  GPIO_USART3_TX_0    /* PB10 */

/* UART4 - RS422(1) interface port */

#define GPIO_UART4_RX   GPIO_UART4_RX_0     /* PC11 */
#define GPIO_UART4_TX   GPIO_UART4_TX_0     /* PC10 */

/* UART5 - RS422(2) interface port */

#define GPIO_UART5_RX   GPIO_UART5_RX_0     /* PD2 */
#define GPIO_UART5_TX   GPIO_UART5_TX_0     /* PC12 */

/* 
 * SPI
 *
 * SPI1 is connected to the NOR.
 */

#define GPIO_SPI1_MISO  GPIO_SPI1_MISO_0
#define GPIO_SPI1_MOSI  GPIO_SPI1_MOSI_0
#define GPIO_SPI1_SCK   GPIO_SPI1_SCK_0

#endif /* __BOARDS_ARM_STM32_AIS_TBCS_INCLUDE_BOARD_H */
