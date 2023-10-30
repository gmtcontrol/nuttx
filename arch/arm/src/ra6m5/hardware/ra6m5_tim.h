/****************************************************************************
 * arch/arm/src/ra6m5/hardware/ra6m5_tim.h
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

#ifndef __ARCH_ARM_SRC_RA6M5_HARDWARE_RA6M5_TIM_H
#define __ARCH_ARM_SRC_RA6M5_HARDWARE_RA6M5_TIM_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* AGT Addresses */

#define RA6M5_AGT0_BASE                 (0x400E8000UL)  /* AGT0 base address */
#define RA6M5_AGT1_BASE                 (0x400E8100UL)  /* AGT1 base address */
#define RA6M5_AGT2_BASE                 (0x400E8200UL)  /* AGT2 base address */
#define RA6M5_AGT3_BASE                 (0x400E8300UL)  /* AGT3 base address */
#define RA6M5_AGT4_BASE                 (0x400E8400UL)  /* AGT4 base address */
#define RA6M5_AGT5_BASE                 (0x400E8500UL)  /* AGT5 base address */

/* Register Offsets *********************************************************/

#define RA6M5_AGT_AGT_OFFSET            0x0000          /* AGT Counter Register */
#define RA6M5_AGT_AGTCMA_OFFSET         0x0002          /* AGT Compare Match A Register */
#define RA6M5_AGT_AGTCMB_OFFSET         0x0004          /* AGT Compare Match B Register */
#define RA6M5_AGT_AGTCR_OFFSET          0x0008          /* AGT Control Register */
#define RA6M5_AGT_AGTMR1_OFFSET         0x0009          /* AGT Mode Register 1 */
#define RA6M5_AGT_AGTMR2_OFFSET         0x000A          /* AGT Mode Register 2 */
#define RA6M5_AGT_AGTIOSEL_ALT_OFFSET   0x000B          /* AGT Pin Select Register */
#define RA6M5_AGT_AGTIOC_OFFSET         0x000C          /* AGT I/O Control Register */
#define RA6M5_AGT_AGTISR_OFFSET         0x000D          /* AGT Event Pin Select Register */
#define RA6M5_AGT_AGTCMSR_OFFSET        0x000E          /* AGT Compare Match Function Select Register */
#define RA6M5_AGT_AGTIOSEL_OFFSET       0x000F          /* AGT Pin Select Register */

/* AGT0 Registers */

#define RA6M5_AGT0_AGT              (RA6M5_AGT0_BASE+RA6M5_AGT_AGT_OFFSET)
#define RA6M5_AGT0_AGTCMA           (RA6M5_AGT0_BASE+RA6M5_AGT_AGTCMA_OFFSET)
#define RA6M5_AGT0_AGTCMB           (RA6M5_AGT0_BASE+RA6M5_AGT_AGTCMB_OFFSET)
#define RA6M5_AGT0_AGTCR            (RA6M5_AGT0_BASE+RA6M5_AGT_AGTCR_OFFSET)
#define RA6M5_AGT0_AGTMR1           (RA6M5_AGT0_BASE+RA6M5_AGT_AGTMR1_OFFSET)
#define RA6M5_AGT0_AGTMR2           (RA6M5_AGT0_BASE+RA6M5_AGT_AGTMR2_OFFSET)
#define RA6M5_AGT0_AGTIOSEL_ALT     (RA6M5_AGT0_BASE+RA6M5_AGT_AGTIOSEL_ALT_OFFSET)
#define RA6M5_AGT0_AGTIOC           (RA6M5_AGT0_BASE+RA6M5_AGT_AGTIOC_OFFSET)
#define RA6M5_AGT0_AGTISR           (RA6M5_AGT0_BASE+RA6M5_AGT_AGTISR_OFFSET)
#define RA6M5_AGT0_AGTCMSR          (RA6M5_AGT0_BASE+RA6M5_AGT_AGTCMSR_OFFSET)
#define RA6M5_AGT0_AGTIOSEL         (RA6M5_AGT0_BASE+RA6M5_AGT_AGTIOSEL_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* AGT Control Register */

#define AGT_AGTCR_TSTART            (1 << 0)    /* Bit 0: AGT Count Start */
#define AGT_AGTCR_TCSTF             (1 << 1)    /* Bit 1: AGT Count Status Flag */
#define AGT_AGTCR_TSTOP             (1 << 2)    /* Bit 2: AGT Count Forced Stop */
#define AGT_AGTCR_TEDGF             (1 << 4)    /* Bit 4: Active Edge Judgment Flag */
#define AGT_AGTCR_TUNDF             (1 << 5)    /* Bit 5: Underflow Flag */
#define AGT_AGTCR_TCMAF             (1 << 6)    /* Bit 6: Compare Match A Flag */
#define AGT_AGTCR_TCMBF             (1 << 7)    /* Bit 7: Compare Match B Flag */

/* AGT Compare Match Function Select Register */

#define AGT_AGTCMSR_TCMEA           (1 << 0)    /* Bit 0: AGT Compare Match A Register Enable */
#define AGT_AGTCMSR_TCMEB           (1 << 4)    /* Bit 4: AGT Compare Match B Register Enable */

#endif /* __ARCH_ARM_SRC_RA6M5_HARDWARE_RA6M5_TIM_H */
