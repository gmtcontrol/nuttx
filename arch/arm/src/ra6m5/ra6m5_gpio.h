/****************************************************************************
 * arch/arm/src/ra6m5/ra6m5_gpio.h
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

#ifndef __ARCH_ARM_SRC_RA6M5_RA6M5_GPIO_H
#define __ARCH_ARM_SRC_RA6M5_RA6M5_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

#include <nuttx/irq.h>
#include <arch/ra6m5/chip.h>

#include "chip.h"

#if defined(CONFIG_RA6M5_R7FA6M5BX)
#  include "hardware/ra6m5_gpio.h"
#else
#  error "Unsupported RA6M5 chip"
#endif

/****************************************************************************
 * Pre-Processor Declarations
 ****************************************************************************/

/* Bit-encoded input to stm32_configgpio() */

/* Each port bit of the general-purpose I/O (GPIO) ports can be individually
 * configured by software in several modes:
 *
 *  - Input floating
 *  - Input pull-up
 *  - Input-pull-down
 *  - Output open-drain with pull-up or pull-down capability
 *  - Output push-pull with pull-up or pull-down capability
 *  - Alternate function push-pull with pull-up or pull-down capability
 *  - Alternate function open-drain with pull-up or pull-down capability
 *  - Analog
 *
 * 32-bit Encoding:       0001 1111 0000 1111 0000 1111 1111 1111
 *                        ---- ---- ---- ---- ---- ---- ---- ----
 * Inputs:                .... .... .... MMUU .... ...X PPPP BBBB
 * Outputs:               .... .... .... MMUU .... FFOV PPPP BBBB
 * Alternate Functions:   ...A AAAA .... MMUU .... FFO. PPPP BBBB
 * Analog:                .... .... .... MM.. .... .... PPPP BBBB
 */

/* Mode:
 *
 * 0000 0000 0000 1100 0000 0000 0000 0000
 * ---- ---- ---- ---- ---- ---- ---- ----
 * .... .... .... MM.. .... .... .... ....
 */

#define GPIO_MODE_SHIFT             (18)                        /* Bits 18-19: GPIO port mode */
#define GPIO_MODE_MASK              (3 << GPIO_MODE_SHIFT)
#  define GPIO_INPUT                (0 << GPIO_MODE_SHIFT)      /* Input mode */
#  define GPIO_OUTPUT               (1 << GPIO_MODE_SHIFT)      /* General purpose output mode */
#  define GPIO_ALT                  (2 << GPIO_MODE_SHIFT)      /* Alternate function mode */
#  define GPIO_ANALOG               (3 << GPIO_MODE_SHIFT)      /* Analog mode */

/* Input/output pull-ups/downs (not used with analog):
 *
 * 0000 0000 0000 0011 0000 0000 0000 0000
 * ---- ---- ---- ---- ---- ---- ---- ----
 * .... .... .... ..UU .... .... .... ....
 */

#define GPIO_PUPD_SHIFT             (16)                        /* Bits 16-17: Pull-up/pull down */
#define GPIO_PUPD_MASK              (3 << GPIO_PUPD_SHIFT)
#  define GPIO_FLOAT                (0 << GPIO_PUPD_SHIFT)      /* No pull-up, pull-down */
#  define GPIO_PULLUP               (1 << GPIO_PUPD_SHIFT)      /* Pull-up */
#  define GPIO_PULLDOWN             (2 << GPIO_PUPD_SHIFT)      /* Pull-down */

/* Alternate Functions:
 *
 * 0001 1111 0000 0000 0000 0000 0000 0000
 * ---- ---- ---- ---- ---- ---- ---- ----
 * ...A AAAA .... .... .... .... .... ....
 */

#define GPIO_AF_SHIFT               (24)                        /* Bits 24-28: Alternate function */
#define GPIO_AF_MASK                (31 << GPIO_AF_SHIFT)
#  define GPIO_AF(n)                ((n) << GPIO_AF_SHIFT)
#  define GPIO_AF0                  (0 << GPIO_AF_SHIFT)        /* DEBUG pin */
#  define GPIO_AF1                  (1 << GPIO_AF_SHIFT)        /* AGT peripheral pin */
#  define GPIO_AF2                  (2 << GPIO_AF_SHIFT)        /* GPT0 peripheral pin */
#  define GPIO_AF3                  (3 << GPIO_AF_SHIFT)        /* GPT1 peripheral pin */
#  define GPIO_AF4                  (4 << GPIO_AF_SHIFT)        /* SCI0/2/4/6/8 peripheral pin */
#  define GPIO_AF5                  (5 << GPIO_AF_SHIFT)        /* SCI1/3/5/7/9 peripheral pin */
#  define GPIO_AF6                  (6 << GPIO_AF_SHIFT)        /* SPI peripheral pin */
#  define GPIO_AF7                  (7 << GPIO_AF_SHIFT)        /* IIC peripheral pin */
#  define GPIO_AF8                  (8 << GPIO_AF_SHIFT)        /* KEY peripheral pin */
#  define GPIO_AF9                  (9 << GPIO_AF_SHIFT)        /* clock/comparator/RTC peripheral pin */
#  define GPIO_AF10                 (10 << GPIO_AF_SHIFT)       /* CAC/ADC peripheral pin */
#  define GPIO_AF11                 (11 << GPIO_AF_SHIFT)       /* BUS peripheral pin */
#  define GPIO_AF12                 (12 << GPIO_AF_SHIFT)       /* CTSU/CMPHS peripheral pin */
#  define GPIO_AF13                 (13 << GPIO_AF_SHIFT)       /* LCD/SCI13579_DEn peripheral pin */
#  define GPIO_AF14                 (14 << GPIO_AF_SHIFT)       /* DALI/SCI02468_DEn peripheral pin */
#  define GPIO_AF16                 (16 << GPIO_AF_SHIFT)       /* CAN peripheral pin */
#  define GPIO_AF17                 (17 << GPIO_AF_SHIFT)       /* QSPI peripheral pin */
#  define GPIO_AF18                 (18 << GPIO_AF_SHIFT)       /* SSI peripheral pin */
#  define GPIO_AF19                 (19 << GPIO_AF_SHIFT)       /* USBFS peripheral pin */
#  define GPIO_AF20                 (20 << GPIO_AF_SHIFT)       /* USBHS/GPT2 peripheral pin */
#  define GPIO_AF21                 (21 << GPIO_AF_SHIFT)       /* SDMMC/GPT3 peripheral pin */
#  define GPIO_AF22                 (22 << GPIO_AF_SHIFT)       /* Ethernet MMI/GPT4 peripheral pin */
#  define GPIO_AF23                 (23 << GPIO_AF_SHIFT)       /* Ethernet RMII peripheral pin */
#  define GPIO_AF24                 (24 << GPIO_AF_SHIFT)       /* PDC peripheral pin */
#  define GPIO_AF25                 (25 << GPIO_AF_SHIFT)       /* Graphics LCD/CAC peripheral pin */
#  define GPIO_AF26                 (26 << GPIO_AF_SHIFT)       /* debug trace peripheral pin */
#  define GPIO_AF28                 (28 << GPIO_AF_SHIFT)       /* OSPI peripheral pin */
#  define GPIO_AF29                 (29 << GPIO_AF_SHIFT)       /* PGAOUT0 peripheral pin */
#  define GPIO_AF30                 (30 << GPIO_AF_SHIFT)       /* PGAOUT1 peripheral pin */

/* Output/Alt function frequency selection:
 *
 * 0000 0000 0000 0000 0000 1100 0000 0000
 * ---- ---- ---- ---- ---- ---- ---- ----
 * .... .... .... .... .... FF.. .... ....
 */

#define GPIO_SPEED_SHIFT            (10)                        /* Bits 10-11: GPIO frequency selection */
#define GPIO_SPEED_MASK             (3 << GPIO_SPEED_SHIFT)
#  define GPIO_SPEED_2MHZ           (0 << GPIO_SPEED_SHIFT)     /* 2 MHz Low speed output */
#  define GPIO_SPEED_25MHZ          (1 << GPIO_SPEED_SHIFT)     /* 25 MHz Medium speed output */
#  define GPIO_SPEED_50MHZ          (2 << GPIO_SPEED_SHIFT)     /* 50 MHz High speed output  */
#  define GPIO_SPEED_100MHZ         (3 << GPIO_SPEED_SHIFT)     /* 100 MHz Very High speed output */

/* Output/Alt function type selection:
 *
 * 0000 0000 0000 0000 0000 0010 0000 0000
 * ---- ---- ---- ---- ---- ---- ---- ----
 * .... .... .... .... .... ..O. .... ....
 */

#define GPIO_OPENDRAIN              (1 << 9)                    /* Bit9: 1=Open-drain output */
#define GPIO_PUSHPULL               (0)                         /* Bit9: 0=Push-pull output */

/* If the pin is a GPIO digital output, then this identifies the initial
 * output value.  If the pin is an input, this bit is overloaded to provide
 * the qualifier to distinguish input pull-up and -down:
 *
 * 0000 0000 0000 0000 0000 0001 0000 0000
 * ---- ---- ---- ---- ---- ---- ---- ----
 * .... .... .... .... .... ...V .... ....
 */

#define GPIO_OUTPUT_SET             (1 << 8)                    /* Bit 8: If output, initial value of output */
#define GPIO_OUTPUT_CLEAR           (0)

/* External interrupt selection (GPIO inputs only):
 *
 * 0000 0000 0000 0000 0000 0001 0000 0000
 * ---- ---- ---- ---- ---- ---- ---- ----
 * .... .... .... .... .... ...X .... ....
 */

#define GPIO_EXTI                   (1 << 8)                    /* Bit 8: Configure as EXTI interrupt */

/* This identifies the GPIO port:
 *
 * 0000 0000 0000 0000 0000 0000 1111 0000
 * ---- ---- ---- ---- ---- ---- ---- ----
 * .... .... .... .... .... .... PPPP ....
 */

#define GPIO_PORT_SHIFT             (4)                         /* Bit 4-7:  Port number */
#define GPIO_PORT_MASK              (15 << GPIO_PORT_SHIFT)
#  define GPIO_PORT0                (0 << GPIO_PORT_SHIFT)      /* PORT0 */
#  define GPIO_PORT1                (1 << GPIO_PORT_SHIFT)      /* PORT1 */
#  define GPIO_PORT2                (2 << GPIO_PORT_SHIFT)      /* PORT2 */
#  define GPIO_PORT3                (3 << GPIO_PORT_SHIFT)      /* PORT3 */
#  define GPIO_PORT4                (4 << GPIO_PORT_SHIFT)      /* PORT4 */
#  define GPIO_PORT5                (5 << GPIO_PORT_SHIFT)      /* PORT5 */
#  define GPIO_PORT6                (6 << GPIO_PORT_SHIFT)      /* PORT6 */
#  define GPIO_PORT7                (7 << GPIO_PORT_SHIFT)      /* PORT7 */
#  define GPIO_PORT8                (8 << GPIO_PORT_SHIFT)      /* PORT8 */
#  define GPIO_PORT9                (9 << GPIO_PORT_SHIFT)      /* PORT9 */
#  define GPIO_PORT10               (10 << GPIO_PORT_SHIFT)     /* PORT10 */
#  define GPIO_PORT11               (11 << GPIO_PORT_SHIFT)     /* PORT11 */
#  define GPIO_PORT12               (12 << GPIO_PORT_SHIFT)     /* PORT12 */
#  define GPIO_PORT13               (13 << GPIO_PORT_SHIFT)     /* PORT13 */
#  define GPIO_PORT14               (14 << GPIO_PORT_SHIFT)     /* PORT14 */

/* This identifies the bit in the port:
 *
 * 0000 0000 0000 0000 0000 0000 0000 1111
 * ---- ---- ---- ---- ---- ---- ---- ----
 * .... .... .... .... .... .... .... BBBB
 */

#define GPIO_PIN_SHIFT              (0)                         /* Bits 0-3: Pin number: 0-15 */
#define GPIO_PIN_MASK               (15 << GPIO_PIN_SHIFT)
#  define GPIO_PIN0                 (0 << GPIO_PIN_SHIFT)
#  define GPIO_PIN1                 (1 << GPIO_PIN_SHIFT)
#  define GPIO_PIN2                 (2 << GPIO_PIN_SHIFT)
#  define GPIO_PIN3                 (3 << GPIO_PIN_SHIFT)
#  define GPIO_PIN4                 (4 << GPIO_PIN_SHIFT)
#  define GPIO_PIN5                 (5 << GPIO_PIN_SHIFT)
#  define GPIO_PIN6                 (6 << GPIO_PIN_SHIFT)
#  define GPIO_PIN7                 (7 << GPIO_PIN_SHIFT)
#  define GPIO_PIN8                 (8 << GPIO_PIN_SHIFT)
#  define GPIO_PIN9                 (9 << GPIO_PIN_SHIFT)
#  define GPIO_PIN10                (10 << GPIO_PIN_SHIFT)
#  define GPIO_PIN11                (11 << GPIO_PIN_SHIFT)
#  define GPIO_PIN12                (12 << GPIO_PIN_SHIFT)
#  define GPIO_PIN13                (13 << GPIO_PIN_SHIFT)
#  define GPIO_PIN14                (14 << GPIO_PIN_SHIFT)
#  define GPIO_PIN15                (15 << GPIO_PIN_SHIFT)


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

/* Base addresses for each GPIO block */

EXTERN const uint32_t g_portbase[RA6M5_NPORTS];

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ra6m5_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *   Once it is configured as Alternative (GPIO_ALT|GPIO_CNF_AFPP|...)
 *   function, it must be unconfigured with ra6m5_unconfiggpio() with
 *   the same cfgset first before it can be set to non-alternative function.
 *
 * Returned Value:
 *   OK on success
 *   ERROR on invalid port, or when pin is locked as ALT function.
 *
 ****************************************************************************/

int ra6m5_configgpio(uint32_t cfgset);

/****************************************************************************
 * Name: ra6m5_unconfiggpio
 *
 * Description:
 *   Unconfigure a GPIO pin based on bit-encoded description of the pin, set
 *   it into default HiZ state (and possibly mark it's unused) and unlock it
 *   whether it was previously selected as alternative function
 *   (GPIO_ALT|GPIO_CNF_AFPP|...).
 *
 *   This is a safety function and prevents hardware from shocks, as
 *   unexpected write to the Timer Channel Output GPIO to fixed '1' or '0'
 *   while it should operate in PWM mode could produce excessive on-board
 *   currents and trigger over-current/alarm function.
 *
 * Returned Value:
 *  OK on success
 *  ERROR on invalid port
 *
 ****************************************************************************/

int ra6m5_unconfiggpio(uint32_t cfgset);

/****************************************************************************
 * Name: ra6m5_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void ra6m5_gpiowrite(uint32_t pinset, bool value);

/****************************************************************************
 * Name: ra6m5_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool ra6m5_gpioread(uint32_t pinset);

/****************************************************************************
 * Name: ra6m5_gpiosetevent
 *
 * Description:
 *   Sets/clears GPIO based event and interrupt triggers.
 *
 * Input Parameters:
 *  pinset      - GPIO pin configuration
 *  risingedge  - Enables interrupt on rising edges
 *  fallingedge - Enables interrupt on falling edges
 *  event       - Generate event when set
 *  func        - When non-NULL, generate interrupt
 *  arg         - Argument passed to the interrupt callback
 *
 * Returned Value:
 *  Zero (OK) is returned on success, otherwise a negated errno value is
 *  returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int ra6m5_gpiosetevent(uint32_t pinset, bool risingedge, bool fallingedge,
                         bool event, xcpt_t func, void *arg);

/****************************************************************************
 * Function:  ra6m5_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the provided base address
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
int ra6m5_dumpgpio(uint32_t pinset, const char *msg);
#else
#  define ra6m5_dumpgpio(p,m)
#endif

/****************************************************************************
 * Function:  ra6m5_gpioinit
 *
 * Description:
 *   Based on configuration within the .config file, it does:
 *    - Remaps positions of alternative functions.
 *
 *   Typically called from ra6m5_start().
 *
 ****************************************************************************/

void ra6m5_gpioinit(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RA6M5_RA6M5_GPIO_H */
