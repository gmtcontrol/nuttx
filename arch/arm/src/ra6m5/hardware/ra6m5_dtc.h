/****************************************************************************
 * arch/arm/src/ra6m5/hardware/ra6m5_dtc.h
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

#ifndef __ARCH_ARM_SRC_RA6M5_HARDWARE_RA6M5_DTC_H
#define __ARCH_ARM_SRC_RA6M5_HARDWARE_RA6M5_DTC_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define RA6M5_DTC_DTCCR_OFFSET          0x0000  /* DTC Control Register */
#define RA6M5_DTC_DTCVBR_OFFSET         0x0004  /* DTC Vector Base Register */
#define RA6M5_DTC_DTCST_OFFSET          0x000C  /* DTC Module Start Register */
#define RA6M5_DTC_DTCSTS_OFFSET         0x000E  /* DTC Status Register */
#define RA6M5_DTC_DTCCR_SEC_OFFSET      0x0010  /* DTC Control Register for secure Region */
#define RA6M5_DTC_DTCVBR_SEC_OFFSET     0x0014  /* DTC Vector Base Register for secure Region */
#define RA6M5_DTC_DTEVR_SEC_OFFSET      0x0020  /* DTC Error Vector Register */

/* These registers inaccessible directly from the CPU */
/* Base address: DTCVBR */

#define RA6M5_DTC_MRA_OFFSET            0x0003  /*  DTC Mode Register A (4 × Vector number) */
#define RA6M5_DTC_MRB_OFFSET            0x0002  /*  DTC Mode Register B (4 × Vector number) */
#define RA6M5_DTC_SAR_OFFSET            0x0004  /*  DTC Transfer Source Register (4 × Vector number) */
#define RA6M5_DTC_DAR_OFFSET            0x0008  /*  DTC Transfer Destination Register (4 × Vector number) */
#define RA6M5_DTC_CRA_OFFSET            0x000E  /*  DTC Transfer Count Register A (4 × Vector number) */
#define RA6M5_DTC_CRB_OFFSET            0x000C  /*  DTC Transfer Count Register B (4 × Vector number) */

#define RA6M5_DTC_MRA(vector)           (getreg32(RA6M5_DTC_BASE + RA6M5_DTC_DTCVBR_OFFSET) + (RA6M5_DTC_MRA_OFFSET + (4 * (vector))))
#define RA6M5_DTC_MRB(vector)           (getreg32(RA6M5_DTC_BASE + RA6M5_DTC_DTCVBR_OFFSET) + (RA6M5_DTC_MRB_OFFSET + (4 * (vector))))
#define RA6M5_DTC_SAR(vector)           (getreg32(RA6M5_DTC_BASE + RA6M5_DTC_DTCVBR_OFFSET) + (RA6M5_DTC_SAR_OFFSET + (4 * (vector))))
#define RA6M5_DTC_DAR(vector)           (getreg32(RA6M5_DTC_BASE + RA6M5_DTC_DTCVBR_OFFSET) + (RA6M5_DTC_DAR_OFFSET + (4 * (vector))))
#define RA6M5_DTC_CRA(vector)           (getreg32(RA6M5_DTC_BASE + RA6M5_DTC_DTCVBR_OFFSET) + (RA6M5_DTC_CRA_OFFSET + (4 * (vector))))
#define RA6M5_DTC_CRB(vector)           (getreg32(RA6M5_DTC_BASE + RA6M5_DTC_DTCVBR_OFFSET) + (RA6M5_DTC_CRB_OFFSET + (4 * (vector))))

/* Register Bitfield Definitions ********************************************/

/*  DTC Mode Register A */

#define DTC_MRA_SM_SHIFT                (2)         /* Bits [2:3]: Transfer Source Address Addressing Mode */
#define DTC_MRA_SM_MASK                 (3 << DTC_MRA_SM_SHIFT)
#  define DTC_MRA_SM(n)                 (((n) << DTC_MRA_SM_SHIFT) & DTC_MRA_SM_MASK)
#define DTC_MRA_SZ_SHIFT                (4)         /* Bits [4:5]: DTC Data Transfer Size */
#define DTC_MRA_SZ_MASK                 (3 << DTC_MRA_SZ_SHIFT)
#  define DTC_MRA_SZ(n)                 (((n) << DTC_MRA_SZ_SHIFT) & DTC_MRA_SZ_MASK)
#define DTC_MRA_MD_SHIFT                (6)         /* Bits [6:7]: DTC Transfer Mode Select */
#define DTC_MRA_MD_MASK                 (3 << DTC_MRA_MD_SHIFT)
#  define DTC_MRA_MD(n)                 (((n) << DTC_MRA_MD_SHIFT) & DTC_MRA_MD_MASK)

/*  DTC Mode Register B */

#define DTC_MRB_DM_SHIFT                (2)         /* Bits [2:3]: Transfer Destination Address Addressing Mode */
#define DTC_MRB_DM_MASK                 (3 << DTC_MRB_DM_SHIFT)
#  define DTC_MRB_DM(n)                 (((n) << DTC_MRB_DM_SHIFT) & DTC_MRB_DM_MASK)
#define DTC_MRB_DTS                     (1 << 4)    /* Bit 4: DTC Transfer Mode Select */
#define DTC_MRB_DISEL                   (1 << 5)    /* Bit 5: DTC Interrupt Select */
#define DTC_MRB_CHNS                    (1 << 6)    /* Bit 6: DTC Chain Transfer Select */
#define DTC_MRB_CHNE                    (1 << 7)    /* Bit 7: DTC Chain Transfer Enable */

/*  DTC Control Register */

#define DTC_DTCCR_RRS                   (1 << 4)    /* Bit 4: DTC Transfer Information Read Skip Enable */

/*  DTC Module Start Register */

#define DTC_DTCST_START                 (1 << 0)    /* Bit 0: DTC Module Start */

/*  DTC Module Start Register */

#define DTC_DTCSTS_VECN_SHIFT           (0)         /* Bits [0:7]: DTC-Activating Vector Number Monitoring */
#define DTC_DTCSTS_VECN_MASK            (255 << DTC_DTCSTS_VECN_SHIFT)
#  define DTC_DTCSTS_VECN(n)            (((n) << DTC_DTCSTS_VECN_SHIFT) & DTC_DTCSTS_VECN_MASK)
#define DTC_DTCSTS_ACT                  (1 << 15)   /* Bit 15: DTC Active Flag */

#endif /* __ARCH_ARM_SRC_RA6M5_HARDWARE_RA6M5_DTC_H */
