/****************************************************************************
 * arch/risc-v/src/sty32c2/hardware/sty32c2_clint.h
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

#ifndef __ARCH_RISCV_SRC_STY32C2_HARDWARE_STY32C2_CLINT_H
#define __ARCH_RISCV_SRC_STY32C2_HARDWARE_STY32C2_CLINT_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CLINT_MSIP_BASE         (0x0000)
#define CLINT_MTIMECMP_BASE     (0x4000)
#define CLINT_MTIME_BASE        (0xbff8)

#define STY32C2_CLINT_MSIP      (STY32C2_CLINT_BASE + CLINT_MSIP_BASE)
#define STY32C2_CLINT_MTIME     (STY32C2_CLINT_BASE + CLINT_MTIME_BASE)

#define STY32C2_CLINT_MTIMECMP(hart) \
    (STY32C2_CLINT_BASE + CLINT_MTIMECMP_BASE + (8 * hart))

#define RISCV_CLINT_MSIP        STY32C2_CLINT_MSIP

#endif /* __ARCH_RISCV_SRC_STY32C2_HARDWARE_STY32C2_CLINT_H */
