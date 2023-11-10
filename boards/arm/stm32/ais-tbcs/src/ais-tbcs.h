/****************************************************************************
 * boards/arm/stm32/ais-tbcs/src/ais-tbcs.h
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

#ifndef __BOARDS_ARM_STM32_AIS_TBCS_SRC_AIS_TBCS_H
#define __BOARDS_ARM_STM32_AIS_TBCS_SRC_AIS_TBCS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LED definitions **********************************************************/

/* The AIS-TBCS board supports 4 LEDs. 
 *
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LED in
 * any way.  The following definition is used to access the LED.
 */

#define GPIO_LED0      (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                        GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN6)
#define GPIO_LED1      (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                        GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN7)
#define GPIO_LED2      (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                        GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN8)
#define GPIO_LED3      (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                        GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN9)

#define LED_DRIVER_PATH "/dev/userleds"

/* Button definitions *******************************************************/

/* The AIS-TBCS supports 4 buttons
 *
 *
 * NOTE that EXTI interrupts are configured.
 */

#define MIN_IRQBUTTON   GPIO_BTN_KEY0
#define MAX_IRQBUTTON   GPIO_BTN_KEY3
#define NUM_IRQBUTTONS  4

#define GPIO_BTN_KEY0   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_EXTI|GPIO_PORTB|GPIO_PIN15)
#define GPIO_BTN_KEY1   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_EXTI|GPIO_PORTB|GPIO_PIN14)
#define GPIO_BTN_KEY2   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_EXTI|GPIO_PORTB|GPIO_PIN13)
#define GPIO_BTN_KEY3   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_EXTI|GPIO_PORTB|GPIO_PIN12)


/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture specific initialization
 *
 *   CONFIG_BOARDCTL=y:
 *     If CONFIG_NSH_ARCHINITIALIZE=y:
 *       Called from the NSH library (or other application)
 *     Otherwise, assumed to be called from some other application.
 *
 *   Otherwise CONFIG_BOARD_LATE_INITIALIZE=y:
 *     Called from board_late_initialize().
 *
 *   Otherwise, bad news:  Never called
 *
 ****************************************************************************/

int stm32_bringup(void);

/****************************************************************************
 * Name: stm32_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

#ifdef CONFIG_ADC
int stm32_adc_setup(void);
#endif

#endif /* __BOARDS_ARM_STM32_AIS_TBCS_SRC_AIS_TBCS_H */
