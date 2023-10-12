/****************************************************************************
 * arch/risc-v/src/sty32c2/hardware/sty32c2_plic.h
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

#ifndef __ARCH_RISCV_SRC_STY32C2_HARDWARE_STY32C2_PLIC_H
#define __ARCH_RISCV_SRC_STY32C2_HARDWARE_STY32C2_PLIC_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PLIC_PRIORITY_BASE      (0x4)
#define PLIC_PENDING_BASE       (0x1000)
#define PLIC_ENABLE_BASE        (0x2000)
#define PLIC_ENABLE_STRIDE      (0x80)
#define PLIC_CONTEXT_BASE       (0x200000)
#define PLIC_CONTEXT_STRIDE     (0x1000)
#define PLIC_MODE_MACHINE       (0x0)
#define PLIC_MODE_SUPERVISOR    (0x1)

#define STY32C2_PLIC_PRIORITY(interrupt) \
    (STY32C2_PLIC_BASE + PLIC_PRIORITY_BASE * interrupt)

#define STY32C2_PLIC_PENDING(hart) \
    (STY32C2_PLIC_BASE + PLIC_PENDING_BASE + (4 * hart))

#define STY32C2_PLIC_THRESHOLD(hart, mode) \
    (STY32C2_PLIC_BASE + PLIC_CONTEXT_BASE + PLIC_CONTEXT_STRIDE * (2 * hart + mode))

#define STY32C2_PLIC_CLAIM(hart, mode) \
    (STY32C2_PLIC_THRESHOLD(hart, mode) + 4)

#define STY32C2_PLIC_ENABLE(hart, mode) \
    (STY32C2_PLIC_BASE + PLIC_ENABLE_BASE + PLIC_ENABLE_STRIDE * (2 * hart + mode))

#endif /* __ARCH_RISCV_SRC_STY32C2_HARDWARE_STY32C2_PLIC_H */
