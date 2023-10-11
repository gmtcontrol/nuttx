/****************************************************************************
 * arch/arm/src/ra6m5/hardware/ra6m5_spi.h
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

#ifndef __ARCH_ARM_SRC_RA6M5_HARDWARE_RA6M5_QSPI_H
#define __ARCH_ARM_SRC_RA6M5_HARDWARE_RA6M5_QSPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define RA6M5_QSPI_SFMSMD_OFFSET    0x0000  /* Transfer Mode Control Regist */
#define RA6M5_QSPI_SFMSSC_OFFSET    0x0004  /* Chip Selection Control Register */
#define RA6M5_QSPI_SFMSKC_OFFSET    0x0008  /* Clock Control Register */
#define RA6M5_QSPI_SFMSST_OFFSET    0x000C  /* Status Register */
#define RA6M5_QSPI_SFMCOM_OFFSET    0x0010  /* Communication Port Register */
#define RA6M5_QSPI_SFMCMD_OFFSET    0x0014  /* Communication Mode Control Register */
#define RA6M5_QSPI_SFMCST_OFFSET    0x0018  /* Communication Status Register */
#define RA6M5_QSPI_SFMSIC_OFFSET    0x0020  /* Instruction Code Register */
#define RA6M5_QSPI_SFMSAC_OFFSET    0x0024  /* Address Mode Control Register */
#define RA6M5_QSPI_SFMSDC_OFFSET    0x0028  /* Dummy Cycle Control Register */
#define RA6M5_QSPI_SFMSPC_OFFSET    0x0030  /* SPI Protocol Control Register */
#define RA6M5_QSPI_SFMPMD_OFFSET    0x0034  /* Port Control Register */
#define RA6M5_QSPI_SFMCNT1_OFFSET   0x0804  /* External QSPI Address Register */

/* Register Bitfield Definitions ********************************************/

/* Transfer Mode Control Register */

#define QSPI_SFMSMD_SFMRM_SHIFT     (0)         /* Bits 0..2: Serial interface read mode select */
#define QSPI_SFMSMD_SFMRM_MASK      (7 << QSPI_SFMSMD_SFMRM_SHIFT)
#  define QSPI_SFMSMD_SFMRM(n)      (((n) << QSPI_SFMSMD_SFMRM_SHIFT) & QSPI_SFMSMD_SFMRM_MASK)

#define QSPI_SFMSMD_SFMSE_SHIFT     (4)         /* Bits 4..5: QSSL extension function select after SPI bus access */
#define QSPI_SFMSMD_SFMSE_MASK      (3 << QSPI_SFMSMD_SFMSE_SHIFT)
#  define QSPI_SFMSMD_SFMSE(n)      (((n) << QSPI_SFMSMD_SFMSE_SHIFT) & QSPI_SFMSMD_SFMSE_MASK)

#define QSPI_SFMSMD_SFMPFE          (1 << 6)    /* Bit 6: Prefetch function select */
#define QSPI_SFMSMD_SFMPAE          (1 << 7)    /* Bit 7: Function select for stopping prefetch at locations other than on byte boundaries */
#define QSPI_SFMSMD_SFMMD3          (1 << 8)    /* Bit 8: SPI mode select */
#define QSPI_SFMSMD_SFMOEX          (1 << 9)    /* Bit 9: Extension select for the I/O buffer output enable signal for the serial interface */
#define QSPI_SFMSMD_SFMOHW          (1 << 10)   /* Bit 10: Hold time adjustment for serial transmission */
#define QSPI_SFMSMD_SFMOSW          (1 << 11)   /* Bit 11: Setup time adjustment for serial transmission */
#define QSPI_SFMSMD_SFMCCE          (1 << 15)   /* Bit 15: Read instruction code select */

/* SPI Protocol Control Register */

#define QSPI_SFMSPC_SFMSPI_SHIFT    (0)         /* Bits 0..1: SPI protocol select */
#define QSPI_SFMSPC_SFMSPI_MASK     (3 << QSPI_SFMSPC_SFMSPI_SHIFT)
#  define QSPI_SFMSPC_SFMSPI(n)     (((n) << QSPI_SFMSPC_SFMSPI_SHIFT) & QSPI_SFMSPC_SFMSPI_MASK)

#define QSPI_SFMSPC_SFMSDE          (1 << 4)    /* Bit 4: QSPCLK extended selection bit when switching I/O of QIOn pin */

/* Address Mode Control Register */

#define QSPI_SFMSAC_SFMAS_SHIFT     (0)         /* Bits 0..1: Number of address bytes select for the serial interface */
#define QSPI_SFMSAC_SFMAS_MASK      (3 << QSPI_SFMSAC_SFMAS_SHIFT)
#  define QSPI_SFMSAC_SFMAS(n)      (((n) << QSPI_SFMSAC_SFMAS_SHIFT) & QSPI_SFMSAC_SFMAS_MASK)

#define QSPI_SFMSAC_SFM4BC          (1 << 4)    /* Bit 4: Selection of instruction code automatically generated when the serial interface  width is 4 bytes */


/* Chip Selection Control Register */

#define QSPI_SFMSSC_SFMSW_SHIFT     (0)         /* Bits 0..3: Minimum high-level width select for QSSL signal */
#define QSPI_SFMSSC_SFMSW_MASK      (15 << QSPI_SFMSSC_SFMSW_SHIFT)
#  define QSPI_SFMSSC_SFMSW(n)      (((n) << QSPI_SFMSSC_SFMSW_SHIFT) & QSPI_SFMSSC_SFMSW_MASK)

#define QSPI_SFMSAC_SFMSHD          (1 << 4)    /* Bit 4: QSSL Signal Hold Time */
#define QSPI_SFMSAC_SFMSLD          (1 << 5)    /* Bit 5: QSSL Signal Setup Time */

#endif /* __ARCH_ARM_SRC_RA6M5_HARDWARE_RA6M5_QSPI_H */
