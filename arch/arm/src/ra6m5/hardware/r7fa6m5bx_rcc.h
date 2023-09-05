/****************************************************************************
 * arch/arm/src/ra6m5/hardware/r7fa6m5bx_rcc.h
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

#ifndef __ARCH_ARM_SRC_RA6M5_HARDWARE_R7FA6M5BX_RCC_H
#define __ARCH_ARM_SRC_RA6M5_HARDWARE_R7FA6M5BX_RCC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_RA6M5_R7FA6M5BX)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The following definitions are macros instead of enums because the values are used in preprocessor conditionals. */
/* Must match SCKCR.CKSEL values. */
#define RA6M5_CLOCKS_SOURCE_HOCO        (0)     // The high speed on chip oscillator.
#define RA6M5_CLOCKS_SOURCE_MOCO        (1)     // The middle speed on chip oscillator.
#define RA6M5_CLOCKS_SOURCE_LOCO        (2)     // The low speed on chip oscillator.
#define RA6M5_CLOCKS_SOURCE_MOSC	    (3)     // The main oscillator.
#define RA6M5_CLOCKS_SOURCE_SOSC        (4)     // The subclock oscillator.
#define RA6M5_CLOCKS_SOURCE_PLL         (5)     // The PLL oscillator.
#define RA6M5_CLOCKS_SOURCE_PLL2        (6)     // The PLL2 oscillator.
#define RA6M5_CLOCKS_SOURCE_PCLKA       (7)     // The PCLKA oscillator
#define RA6M5_CLOCKS_SOURCE_PCLKB       (8)     // The PCLKB oscillator
#define RA6M5_CLOCKS_SOURCE_PCLKC       (9)     // The PCLKC oscillator
#define RA6M5_CLOCKS_SOURCE_PCLKD       (10)    // The PCLKD oscillator
#define RA6M5_CLOCKS_SOURCE_SYSYEM      (11)    // The System clock
#define RA6M5_CLOCKS_SOURCE_NUMBER      (12)

/* System clock divider options. */
#define RA6M5_CLOCKS_SYS_CLOCK_DIV_1    (0)     // System clock divided by 1.
#define RA6M5_CLOCKS_SYS_CLOCK_DIV_2    (1)     // System clock divided by 2.
#define RA6M5_CLOCKS_SYS_CLOCK_DIV_4    (2)     // System clock divided by 4.
#define RA6M5_CLOCKS_SYS_CLOCK_DIV_6    (3)     // System clock divided by 6. (for CANFD)
#define RA6M5_CLOCKS_SYS_CLOCK_DIV_8    (3)     // System clock divided by 8.
#define RA6M5_CLOCKS_SYS_CLOCK_DIV_16   (4)     // System clock divided by 16.
#define RA6M5_CLOCKS_SYS_CLOCK_DIV_32   (5)     // System clock divided by 32.
#define RA6M5_CLOCKS_SYS_CLOCK_DIV_64   (6)     // System clock divided by 64.
#define RA6M5_CLOCKS_SYS_CLOCK_DIV_128  (7)     // System clock divided by 128 (available for CLKOUT only).

/* PLL divider options. */
#define RA6M5_CLOCKS_PLL_DIV_1          (0)
#define RA6M5_CLOCKS_PLL_DIV_2          (1)
#define RA6M5_CLOCKS_PLL_DIV_3          (2)
#define RA6M5_CLOCKS_PLL_DIV_4          (2)

/* PLL multiplier options. */
#define RA6M5_CLOCKS_PLL_MUL_8_0        (0xF)
#define RA6M5_CLOCKS_PLL_MUL_9_0        (0x11)
#define RA6M5_CLOCKS_PLL_MUL_10_0       (0x13)
#define RA6M5_CLOCKS_PLL_MUL_10_5       (0x14)
#define RA6M5_CLOCKS_PLL_MUL_11_0       (0x15)
#define RA6M5_CLOCKS_PLL_MUL_11_5       (0x16)
#define RA6M5_CLOCKS_PLL_MUL_12_0       (0x17)
#define RA6M5_CLOCKS_PLL_MUL_12_5       (0x18)
#define RA6M5_CLOCKS_PLL_MUL_13_0       (0x19)
#define RA6M5_CLOCKS_PLL_MUL_13_5       (0x1A)
#define RA6M5_CLOCKS_PLL_MUL_14_0       (0x1B)
#define RA6M5_CLOCKS_PLL_MUL_14_5       (0x1c)
#define RA6M5_CLOCKS_PLL_MUL_15_0       (0x1d)
#define RA6M5_CLOCKS_PLL_MUL_15_5       (0x1e)
#define RA6M5_CLOCKS_PLL_MUL_16_0       (0x1f)
#define RA6M5_CLOCKS_PLL_MUL_16_5       (0x20)
#define RA6M5_CLOCKS_PLL_MUL_17_0       (0x21)
#define RA6M5_CLOCKS_PLL_MUL_17_5       (0x22)
#define RA6M5_CLOCKS_PLL_MUL_18_0       (0x23)
#define RA6M5_CLOCKS_PLL_MUL_18_5       (0x24)
#define RA6M5_CLOCKS_PLL_MUL_19_0       (0x25)
#define RA6M5_CLOCKS_PLL_MUL_19_5       (0x26)
#define RA6M5_CLOCKS_PLL_MUL_20_0       (0x27)
#define RA6M5_CLOCKS_PLL_MUL_20_5       (0x28)
#define RA6M5_CLOCKS_PLL_MUL_21_0       (0x29)
#define RA6M5_CLOCKS_PLL_MUL_21_5       (0x2A)
#define RA6M5_CLOCKS_PLL_MUL_22_0       (0x2B)
#define RA6M5_CLOCKS_PLL_MUL_22_5       (0x2c)
#define RA6M5_CLOCKS_PLL_MUL_23_0       (0x2d)
#define RA6M5_CLOCKS_PLL_MUL_23_5       (0x2e)
#define RA6M5_CLOCKS_PLL_MUL_24_0       (0x2f)
#define RA6M5_CLOCKS_PLL_MUL_24_5       (0x30)
#define RA6M5_CLOCKS_PLL_MUL_25_0       (0x31)
#define RA6M5_CLOCKS_PLL_MUL_25_5       (0x32)
#define RA6M5_CLOCKS_PLL_MUL_26_0       (0x33)
#define RA6M5_CLOCKS_PLL_MUL_26_5       (0x34)
#define RA6M5_CLOCKS_PLL_MUL_27_0       (0x35)
#define RA6M5_CLOCKS_PLL_MUL_27_5       (0x36)
#define RA6M5_CLOCKS_PLL_MUL_28_0       (0x37)
#define RA6M5_CLOCKS_PLL_MUL_28_5       (0x38)
#define RA6M5_CLOCKS_PLL_MUL_29_0       (0x39)
#define RA6M5_CLOCKS_PLL_MUL_29_5       (0x3A)
#define RA6M5_CLOCKS_PLL_MUL_30_0       (0x3B)
#define RA6M5_CLOCKS_PLL_MUL_31_0       (0x3D)

/* Register Offsets *********************************************************/

#define RA6M5_SYS_CGFSAR_OFFSET         0x03C0  /* Clock Generation Function Security Attribute Register */
#define RA6M5_SYS_SCKDIVCR_OFFSET       0x0020  /* System Clock Division Control Register */
#define RA6M5_SYS_SCKDIVCR2_OFFSET      0x0024  /* System Clock Division Control Register 2 */
#define RA6M5_SYS_SCKSCR_OFFSET         0x0026  /* System Clock Source Control Register */
#define RA6M5_SYS_PLLCCR_OFFSET         0x0028  /* PLL Clock Control Register */
#define RA6M5_SYS_PLLCR_OFFSET          0x002A  /* PLL Control Register */
#define RA6M5_SYS_PLLCCR2_OFFSET        0x002B  /* PLL Clock Control Register2 */
#define RA6M5_SYS_BCKCR_OFFSET          0x0030  /* External Bus Clock Control Register */
#define RA6M5_SYS_MEMWAIT_OFFSET        0x0031  /* Memory Wait Cycle Control Register */
#define RA6M5_SYS_MOSCCR_OFFSET         0x0032  /* Main Clock Oscillator Control Register */
#define RA6M5_SYS_HOCOCR_OFFSET         0x0036  /* High-Speed On-Chip Oscillator Control Register */
#define RA6M5_SYS_MOCOCR_OFFSET         0x0038  /* Middle-Speed On-Chip Oscillator Control Register */
#define RA6M5_SYS_FLLCR1_OFFSET         0x0039  /* FLL Control Register 1 */
#define RA6M5_SYS_FLLCR2_OFFSET         0x003A  /* FLL Control Register 2 */
#define RA6M5_SYS_OSCSF_OFFSET          0x003C  /* Oscillation Stabilization Flag Register */
#define RA6M5_SYS_CKOCR_OFFSET          0x003E  /* Clock Out Control Register */
#define RA6M5_SYS_TRCKCR_OFFSET         0x003F  /* Trace Clock Control Register */
#define RA6M5_SYS_OSTDCR_OFFSET         0x0040  /* Oscillation Stop Detection Control Register */
#define RA6M5_SYS_OSTDSR_OFFSET         0x0041  /* Oscillation Stop Detection Status Register */
#define RA6M5_SYS_PLL2CCR_OFFSET        0x0048  /* PLL2 Clock Control Register */
#define RA6M5_SYS_PLL2CR_OFFSET         0x004A  /* PLL2 Control Register */
#define RA6M5_SYS_USBCKDIVCR_OFFSET     0x006C  /* USB Clock Division Control Register */
#define RA6M5_SYS_OCTACKDIVCR_OFFSET    0x006D  /* Octal-SPI Clock Division Control Register */
#define RA6M5_SYS_CANFDCKDIVCR_OFFSET   0x006E  /* CANFD Clock Division Control Register */
#define RA6M5_SYS_USB60CKDIVCR_OFFSET   0x006F  /* USB60 Clock Division Control Register */
#define RA6M5_SYS_CECCKDIVCR_OFFSET     0x0070  /* CEC Clock Division Control Register */
#define RA6M5_SYS_USBCKCR_OFFSET        0x0074  /* USB Clock Control Register */
#define RA6M5_SYS_OCTACKCR_OFFSET       0x0075  /* Octal-SPI Clock Control Register */
#define RA6M5_SYS_CANFDCKCR_OFFSET      0x0076  /* CANFD Clock Control Register */
#define RA6M5_SYS_USB60CKCR_OFFSET      0x0077  /* USB60 Clock Control Register */
#define RA6M5_SYS_CECCKCR_OFFSET        0x0078  /* CEC Clock Control Register */
#define RA6M5_SYS_MOSCWTCR_OFFSET       0x00A2  /* Main Clock Oscillator Wait Control Register */
#define RA6M5_SYS_PRCR_OFFSET           0x03FE  /* Protect Register */
#define RA6M5_SYS_MOMCR_OFFSET          0x0413  /* Main Clock Oscillator Mode Oscillation Control Register */
#define RA6M5_SYS_SOSCCR_OFFSET         0x0480  /* Sub-Clock Oscillator Control Register */
#define RA6M5_SYS_SOMCR_OFFSET          0x0481  /* Sub Clock Oscillator Mode Control Register */
#define RA6M5_SYS_LOCOCR_OFFSET         0x0490  /* Low-Speed On-Chip Oscillator Control Register */
#define RA6M5_SYS_LOCOUTCR_OFFSET       0x0492  /* LOCO User Trimming Control Register */

#define RA6M5_FCACHE_FCACHEE_OFFSET     0x0100  /* Flash Cache Enable Register */
#define RA6M5_FCACHE_FCACHEIV_OFFSET    0x0104  /* Flash Cache Invalidate Register */
#define RA6M5_FCACHE_FLWT_OFFSET        0x011C  /* Flash Wait Cycle Register */
#define RA6M5_FCACHE_FSAR_OFFSET        0x0140  /* Flash Security Attribution Register */

#define RA6M5_CACHE_CCACTL_OFFSET       0x0000  /* C-Cache Control Register */
#define RA6M5_CACHE_CCAFCT_OFFSET       0x0004  /* C-Cache Flush Control Register */
#define RA6M5_CACHE_CCALCF_OFFSET       0x0008  /* C-Cache Line Configuration Register */
#define RA6M5_CACHE_SCACTL_OFFSET       0x0040  /* S-Cache Control Register */
#define RA6M5_CACHE_SCAFCT_OFFSET       0x0044  /* S-Cache Flush Control Register */
#define RA6M5_CACHE_SCALCF_OFFSET       0x0048  /* S-Cache Line Configuration Register */
#define RA6M5_CACHE_CAPOAD_OFFSET       0x0200  /* Cache Parity Error Operation After Detection Register */
#define RA6M5_CACHE_CAPRCR_OFFSET       0x0204  /* Cache Protection Register */

#define RA6M5_MSTP_MSTPCRA_OFFSET       0x0000  /* Module Stop Control Register A */
#define RA6M5_MSTP_MSTPCRB_OFFSET       0x0004  /* Module Stop Control Register B */
#define RA6M5_MSTP_MSTPCRC_OFFSET       0x0008  /* Module Stop Control Register C */
#define RA6M5_MSTP_MSTPCRD_OFFSET       0x000C  /* Module Stop Control Register D */
#define RA6M5_MSTP_MSTPCRE_OFFSET       0x0010  /* Module Stop Control Register E */

#define RA6M5_SYSTEM_REG(r)             (RA6M5_SYSTEM_BASE+(r))
#define RA6M5_CACHE_REG(r)              (RA6M5_CACHE_BASE+(r))
#define RA6M5_FCACHE_REG(r)             (RA6M5_FCACHE_BASE+(r))
#define RA6M5_MSTP_REG(r)               (RA6M5_MSTP_BASE+(r))

/* Register Bitfield Definitions ********************************************/

/* SCI SPI Clock Control Register */

#define CKCR_SCISPI_CKSEL_SHIFT     (0)         /* Bits 0..2: SCI SPI Clock (SCISPICLK) Source Select */
#define CKCR_SCISPI_CKSEL_MASK      (7 << CKCR_SCISPI_CKSEL_SHIFT)
#  define CKCR_SCISPI_CKSEL(n)      (((n) << CKCR_SCISPI_CKSEL_SHIFT) & CKCR_SCISPI_CKSEL_MASK)

#define CKCR_SCISPI_CKSREQ          (1 << 6)    /* Bit 6: SCI SPI Clock (SCISPICLK) Switching Request */
#define CKCR_SCISPI_CKSRDY          (1 << 7)    /* Bit 7: SCI SPI Clock (SCISPICLK) Switching Ready state flag */

/* Oscillation Stabilization Flag Register */

#define SYS_OSCSF_HOCOSF            (1 << 0)    /* Bit 0: HOCO Clock Oscillation Stabilization Flag */
#define SYS_OSCSF_MOSCSF            (1 << 3)    /* Bit 3: Main Clock Oscillation Stabilization Flag */
#define SYS_OSCSF_PLLSF             (1 << 5)    /* Bit 5: PLL Clock Oscillation Stabilization Flag */
#define SYS_OSCSF_PLL2SF            (1 << 6)    /* Bit 6: PLL2 Clock Oscillation Stabilization Flag */

/* Protect Register */

#define SYS_PRCR_PRC0               (1 << 0)    /* Bit 0: Protection bit for the clock generation circuit */
#define SYS_PRCR_PRC1               (1 << 1)    /* Bit 1: Protection bit for the low power modes, and the battery backup function */
#define SYS_PRCR_PRC3               (1 << 3)    /* Bit 3: Protection bit for the LVD */
#define SYS_PRCR_PRC4               (1 << 4)    /* Bit 3: Protection bit for the security function */

/* Module Stop Control Register A */

#define MSTP_MSTPCRA_DMADTC         (1 << 22)   /* Bit 22: DMA Controller/Data Transfer Controller Module Stop */

/* Module Stop Control Register B */

#define MSTP_MSTPCRB_CEC            (1 << 3)    /* Bit 3 : CEC Module Stop */
#define MSTP_MSTPCRB_QSPI           (1 << 6)    /* Bit 6 : Quad Serial Peripheral Interface Module Stop */
#define MSTP_MSTPCRB_USBFS          (1 << 11)   /* Bit 11: Universal Serial Bus 2.0 FS Interface 0 Module Stop */
#define MSTP_MSTPCRB_USBHS          (1 << 12)   /* Bit 12: Universal Serial Bus 2.0 HS Interface 0 Module Stop */
#define MSTP_MSTPCRB_EMAC           (1 << 15)   /* Bit 15: ETHERC0 and EDMAC0 Module Stop */

/* Module Stop Control Register D */

#define MSTP_MSTPCRD_AGT3           (1 << 0)    /* Bit 0 : Low Power Asynchronous General Purpose Timer 3 Module Stop */
#define MSTP_MSTPCRD_AGT2           (1 << 1)    /* Bit 1 : Low Power Asynchronous General Purpose Timer 2 Module Stop */
#define MSTP_MSTPCRD_AGT1           (1 << 2)    /* Bit 2 : Low Power Asynchronous General Purpose Timer 1 Module Stop */
#define MSTP_MSTPCRD_AGT0           (1 << 3)    /* Bit 3 : Low Power Asynchronous General Purpose Timer 0 Module Stop */

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern uint32_t g_clock_freq[];

/* Key code for writing PRCR register. */
#define BSP_PRV_PRCR_KEY            (0xA500U)

#endif /* CONFIG_RA6M5_R7FA6M5BX */
#endif /* __ARCH_ARM_SRC_RA6M5_HARDWARE_R7FA6M5BX_RCC_H */
