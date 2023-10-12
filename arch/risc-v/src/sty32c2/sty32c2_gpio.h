/****************************************************************************
 * arch/risc-v/src/sty32c2/sty32c2_gpio.h
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

#ifndef __ARCH_RISCV_SRC_STY32C2_STY32C2_GPIO_H
#define __ARCH_RISCV_SRC_STY32C2_STY32C2_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

#include "riscv_internal.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIO_PIN_SHIFT        (0)   /* Bits 0-4: Pin number */
#define GPIO_PIN_MASK         (31 << GPIO_PIN_SHIFT)
#define GPIO_PIN0             (0 << GPIO_PIN_SHIFT)
#define GPIO_PIN1             (1 << GPIO_PIN_SHIFT)
#define GPIO_PIN2             (2 << GPIO_PIN_SHIFT)
#define GPIO_PIN3             (3 << GPIO_PIN_SHIFT)
#define GPIO_PIN4             (4 << GPIO_PIN_SHIFT)
#define GPIO_PIN5             (5 << GPIO_PIN_SHIFT)
#define GPIO_PIN6             (6 << GPIO_PIN_SHIFT)
#define GPIO_PIN7             (7 << GPIO_PIN_SHIFT)
#define GPIO_PIN8             (8 << GPIO_PIN_SHIFT)
#define GPIO_PIN9             (9 << GPIO_PIN_SHIFT)
#define GPIO_PIN10            (10 << GPIO_PIN_SHIFT)
#define GPIO_PIN11            (11 << GPIO_PIN_SHIFT)
#define GPIO_PIN12            (12 << GPIO_PIN_SHIFT)
#define GPIO_PIN13            (13 << GPIO_PIN_SHIFT)
#define GPIO_PIN14            (14 << GPIO_PIN_SHIFT)
#define GPIO_PIN15            (15 << GPIO_PIN_SHIFT)
#define GPIO_PIN16            (16 << GPIO_PIN_SHIFT)
#define GPIO_PIN17            (17 << GPIO_PIN_SHIFT)
#define GPIO_PIN18            (18 << GPIO_PIN_SHIFT)
#define GPIO_PIN19            (19 << GPIO_PIN_SHIFT)
#define GPIO_PIN20            (20 << GPIO_PIN_SHIFT)
#define GPIO_PIN21            (21 << GPIO_PIN_SHIFT)
#define GPIO_PIN22            (22 << GPIO_PIN_SHIFT)
#define GPIO_PIN23            (23 << GPIO_PIN_SHIFT)

/****************************************************************************
 * Public Functions Prototypes
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
 * Name: sty32c2_gpio_config
 ****************************************************************************/

EXTERN int sty32c2_gpio_config(uint16_t gpiocfg);

/****************************************************************************
 * Name: sty32c2_gpio_write
 ****************************************************************************/

EXTERN void sty32c2_gpio_write(uint16_t gpiocfg, bool value);

/****************************************************************************
 * Name: sty32c2_gpio_read
 ****************************************************************************/

EXTERN bool sty32c2_gpio_read(uint16_t gpiocfg);

/****************************************************************************
 * Name: sty32c2_gpio_clearpending
 ****************************************************************************/

EXTERN void sty32c2_gpio_clearpending(uint32_t pin);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_STY32C2_STY32C2_GPIO_H */
