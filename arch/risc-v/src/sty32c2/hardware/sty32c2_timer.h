/****************************************************************************
 * arch/risc-v/src/sty32c2/hardware/sty32c2_timer.h
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

#ifndef __ARCH_RISCV_SRC_STY32C2_HARDWARE_STY32C2_TIMER_H
#define __ARCH_RISCV_SRC_STY32C2_HARDWARE_STY32C2_TIMER_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STY32C2_TIMER_LOAD        (STY32C2_TIMER0_BASE)
#define STY32C2_TIMER_RELOAD      (STY32C2_TIMER0_BASE + 0x04)
#define STY32C2_TIMER_ENABLE      (STY32C2_TIMER0_BASE + 0x08)
#define STY32C2_TIMER_UPDATE      (STY32C2_TIMER0_BASE + 0x0c)
#define STY32C2_TIMER_VALUE       (STY32C2_TIMER0_BASE + 0x10)
#define STY32C2_TIMER_EVSTAT      (STY32C2_TIMER0_BASE + 0x14)
#define STY32C2_TIMER_EVPEND      (STY32C2_TIMER0_BASE + 0x18)
#define STY32C2_TIMER_EVENAB      (STY32C2_TIMER0_BASE + 0x1c)

#endif /* __ARCH_RISCV_SRC_STY32C2_HARDWARE_STY32C2_TIMER_H */
