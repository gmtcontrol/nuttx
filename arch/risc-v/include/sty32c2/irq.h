/****************************************************************************
 * arch/risc-v/include/sty32c2/irq.h
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

#ifndef __ARCH_RISCV_INCLUDE_STY32C2_IRQ_H
#define __ARCH_RISCV_INCLUDE_STY32C2_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifdef CONFIG_STY32C2_USE_CUSTOM_IRQ_DEFINITIONS
#include CONFIG_STY32C2_CUSTOM_IRQ_DEFINITIONS_PATH
#else

#include  <arch/mode.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Map RISC-V exception code to NuttX IRQ */

#define STY32C2_IRQ_UART0    (RISCV_IRQ_EXT + 1)
#define STY32C2_IRQ_TIMER0   (RISCV_IRQ_EXT + 2)
#define STY32C2_IRQ_ETHMAC   (RISCV_IRQ_EXT + 3)
#define STY32C2_IRQ_SDCARD   (RISCV_IRQ_EXT + 4)
#define STY32C2_IRQ_GPIO     (RISCV_IRQ_EXT + 5)

/* The last hardware IRQ number */

#define STY32C2_IRQ_LAST     (STY32C2_IRQ_GPIO)

/* Second level GPIO interrupts.  GPIO interrupts are decoded and dispatched
 * as a second level of decoding:  The first level dispatches to the GPIO
 * interrupt handler.  The second to the decoded GPIO interrupt handler.
 */

#ifdef CONFIG_STY32C2_GPIO_IRQ
#  define STY32C2_NIRQ_GPIO           32
#  define STY32C2_FIRST_GPIOIRQ       (STY32C2_IRQ_LAST + 1)
#  define STY32C2_LAST_GPIOIRQ        (LITES_FIRST_GPIOIRQ + STY32C2_NIRQ_GPIO)
#else
#  define STY32C2_NIRQ_GPIO           0
#endif

/* Total number of IRQs */

#define NR_IRQS            (STY32C2_IRQ_LAST + STY32C2_NIRQ_GPIO + 1)

#endif /* CONFIG_STY32C2_USE_CUSTOM_IRQ_DEFINITIONS */
#endif /* __ARCH_RISCV_INCLUDE_STY32C2_IRQ_H */
