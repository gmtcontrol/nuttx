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

#define STY32C2_HART_MIE_OFFSET         (0x100)
#define STY32C2_HART_SIE_OFFSET         (0x80)

#define STY32C2_PLIC_NEXTHART_OFFSET    (0x2000)
#define STY32C2_PLIC_MTHRESHOLD_OFFSET  (0x0000)
#define STY32C2_PLIC_MCLAIM_OFFSET      (0x0004)
#define STY32C2_PLIC_STHRESHOLD_OFFSET  (0x1000)
#define STY32C2_PLIC_SCLAIM_OFFSET      (0x1004)    /* From hart base */
#define STY32C2_PLIC_CLAIM_S_OFFSET     (0x1000)    /* From mclaim to sclaim */
#define STY32C2_PLIC_THRESHOLD_S_OFFSET (0x1000)    /* From mthresh to sthresh */

#define STY32C2_PLIC_H0_MIE         (STY32C2_PLIC_BASE + 0x002000)
#define STY32C2_PLIC_H1_MIE         (STY32C2_PLIC_BASE + 0x002080)
#define STY32C2_PLIC_H1_SIE         (STY32C2_PLIC_BASE + 0x002100)
#define STY32C2_PLIC_H2_MIE         (STY32C2_PLIC_BASE + 0x002180)
#define STY32C2_PLIC_H2_SIE         (STY32C2_PLIC_BASE + 0x002200)
#define STY32C2_PLIC_H3_MIE         (STY32C2_PLIC_BASE + 0x002280)
#define STY32C2_PLIC_H3_SIE         (STY32C2_PLIC_BASE + 0x002300)

#define STY32C2_PLIC_H0_MTHRESHOLD  (STY32C2_PLIC_BASE + 0x200000)
#define STY32C2_PLIC_H0_MCLAIM      (STY32C2_PLIC_BASE + 0x200004)

#define STY32C2_PLIC_H1_MTHRESHOLD  (STY32C2_PLIC_BASE + 0x201000)
#define STY32C2_PLIC_H1_MCLAIM      (STY32C2_PLIC_BASE + 0x201004)
#define STY32C2_PLIC_H1_STHRESHOLD  (STY32C2_PLIC_BASE + 0x202000)
#define STY32C2_PLIC_H1_SCLAIM      (STY32C2_PLIC_BASE + 0x202004)

#define STY32C2_PLIC_H2_MTHRESHOLD  (STY32C2_PLIC_BASE + 0x203000)
#define STY32C2_PLIC_H2_MCLAIM      (STY32C2_PLIC_BASE + 0x203004)
#define STY32C2_PLIC_H2_STHRESHOLD  (STY32C2_PLIC_BASE + 0x204000)
#define STY32C2_PLIC_H2_SCLAIM      (STY32C2_PLIC_BASE + 0x204004)

#define STY32C2_PLIC_H3_MTHRESHOLD  (STY32C2_PLIC_BASE + 0x205000)
#define STY32C2_PLIC_H3_MCLAIM      (STY32C2_PLIC_BASE + 0x205004)
#define STY32C2_PLIC_H3_STHRESHOLD  (STY32C2_PLIC_BASE + 0x206000)
#define STY32C2_PLIC_H3_SCLAIM      (STY32C2_PLIC_BASE + 0x206004)

#define STY32C2_PLIC_PRIORITY(interrupt) \
    (STY32C2_PLIC_BASE + (4 * interrupt))

#define STY32C2_PLIC_PENDING(interrupt) \
    (STY32C2_PLIC_BASE + 0x1000 + (4 * interrupt))

#endif /* __ARCH_RISCV_SRC_STY32C2_HARDWARE_STY32C2_PLIC_H */
