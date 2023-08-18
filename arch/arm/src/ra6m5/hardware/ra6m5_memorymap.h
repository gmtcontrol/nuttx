/****************************************************************************
 * arch/arm/src/ra6m5/hardware/ra6m5_memorymap.h
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

#ifndef __ARCH_ARM_SRC_RA6M5_HARDWARE_RA6M5_MEMORYMAP_H
#define __ARCH_ARM_SRC_RA6M5_HARDWARE_RA6M5_MEMORYMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RA6M5 Address Blocks *****************************************************/

#define RA6M5_CODE_BASE         0x00000000  /* 0x00000000-0x002fffff: 3072k code block */
#define RA6M5_SRAM_BASE         0x20000000  /* 0x20000000-0x207fffff: 512k  sram block */

/* Code Base Addresses ******************************************************/

#define RA6M5_BOOT_BASE         0x00000000  /* 0x00000000-0x000fffff: Aliased boot memory */
#define RA6M5_FLASH_BASE        0x00000000  /* 0x00000000-0x002fffff: FLASH memory */
#define RA6M5_SRAM0_BASE        0x20000000  /* 0x20000000-0x2007ffff: 512k SRAM0 */

/* System Memory Addresses **************************************************/

#define RA6M5_SYSMEM_UID        0x01008190  /* The 128-bit unique device identifier (16-byte) */
#define RA6M5_SYSMEM_PNR        0x010080F0  /* Part Numbering (16-byte) */
#define RA6M5_SYSMEM_VER        0x010081B0  /* MCU Version (1-byte) */

/* Peripheral Base Addresses ************************************************/

/* APB1 Base Addresses ******************************************************/

#define RA6M5_AGT0_BASE         0x400E8000UL
#define RA6M5_AGT1_BASE         0x400E8100UL
#define RA6M5_AGT2_BASE         0x400E8200UL
#define RA6M5_AGT3_BASE         0x400E8300UL
#define RA6M5_AGT4_BASE         0x400E8400UL
#define RA6M5_AGT5_BASE         0x400E8500UL

#define RA6M5_GPT0_BASE         0x40169000UL
#define RA6M5_GPT1_BASE         0x40169100UL
#define RA6M5_GPT2_BASE         0x40169200UL
#define RA6M5_GPT3_BASE         0x40169300UL
#define RA6M5_GPT4_BASE         0x40169400UL
#define RA6M5_GPT5_BASE         0x40169500UL
#define RA6M5_GPT6_BASE         0x40169600UL
#define RA6M5_GPT7_BASE         0x40169700UL
#define RA6M5_GPT8_BASE         0x40169800UL
#define RA6M5_GPT9_BASE         0x40169900UL
#define RA6M5_GPT10_BASE        0x40169A00UL
#define RA6M5_GPT11_BASE        0x40169B00UL
#define RA6M5_GPT12_BASE        0x40169C00UL
#define RA6M5_GPT13_BASE        0x40169D00UL
#define RA6M5_GPT_OPS_BASE      0x40169A00UL
#define RA6M5_GPT_POEG0_BASE    0x4008A000UL
#define RA6M5_GPT_POEG1_BASE    0x4008A100UL
#define RA6M5_GPT_POEG2_BASE    0x4008A200UL
#define RA6M5_GPT_POEG3_BASE    0x4008A300UL

#define RA6M5_WDT_BASE          0x40083400UL
#define RA6M5_IWDT_BASE         0x40083200UL

#define RA6M5_SPI0_BASE         0x4011A000UL
#define RA6M5_SPI1_BASE         0x4011A100UL

#define RA6M5_SCI0_BASE         0x40118000UL
#define RA6M5_SCI1_BASE         0x40118100UL
#define RA6M5_SCI2_BASE         0x40118200UL
#define RA6M5_SCI3_BASE         0x40118300UL
#define RA6M5_SCI4_BASE         0x40118400UL
#define RA6M5_SCI5_BASE         0x40118500UL
#define RA6M5_SCI6_BASE         0x40118600UL
#define RA6M5_SCI7_BASE         0x40118700UL
#define RA6M5_SCI8_BASE         0x40118800UL
#define RA6M5_SCI9_BASE         0x40118900UL

#define RA6M5_I2C0_BASE         0x4009F000UL
#define RA6M5_I2C1_BASE         0x4009F100UL
#define RA6M5_I2C2_BASE         0x4009F200UL

#define RA6M5_CAN0_BASE         0x400A8000UL
#define RA6M5_CAN1_BASE         0x400A9000UL
#define RA6M5_CANFD_BASE        0x400B0000UL

#define RA6M5_DMA_BASE          0x40005200UL
#define RA6M5_DMAC0_BASE        0x40005000UL
#define RA6M5_DMAC1_BASE        0x40005040UL
#define RA6M5_DMAC2_BASE        0x40005080UL
#define RA6M5_DMAC3_BASE        0x400050C0UL
#define RA6M5_DMAC4_BASE        0x40005100UL
#define RA6M5_DMAC5_BASE        0x40005140UL
#define RA6M5_DMAC6_BASE        0x40005180UL
#define RA6M5_DMAC7_BASE        0x400051C0UL

#define RA6M5_DTC_BASE          0x40005400UL

#define RA6M5_CRC_BASE          0x40108000UL
#define RA6M5_CTSU_BASE         0x400D0000UL

#define RA6M5_CACHE_BASE        0x40007000UL
#define RA6M5_FCACHE_BASE       0x4001C000UL
 
#define RA6M5_BKPSRAM_BASE      0x40002000UL

#define RA6M5_PORT0_BASE        0x40080000UL
#define RA6M5_PORT1_BASE        0x40080020UL
#define RA6M5_PORT2_BASE        0x40080040UL
#define RA6M5_PORT3_BASE        0x40080060UL
#define RA6M5_PORT4_BASE        0x40080080UL
#define RA6M5_PORT5_BASE        0x400800A0UL
#define RA6M5_PORT6_BASE        0x400800C0UL
#define RA6M5_PORT7_BASE        0x400800E0UL
#define RA6M5_PORT8_BASE        0x40080100UL
#define RA6M5_PORT9_BASE        0x40080120UL
#define RA6M5_PORT10_BASE       0x40080140UL
#define RA6M5_PORT11_BASE       0x40080160UL
#define RA6M5_PORT12_BASE       0x40080180UL
#define RA6M5_PORT13_BASE       0x400801A0UL
#define RA6M5_PORT14_BASE       0x400801C0UL

#define RA6M5_PFS0_BASE         0x40080800UL
#define RA6M5_PFS1_BASE         0x40080840UL
#define RA6M5_PFS2_BASE         0x40080880UL
#define RA6M5_PFS3_BASE         0x400808C0UL
#define RA6M5_PFS4_BASE         0x40080900UL
#define RA6M5_PFS5_BASE         0x40080940UL
#define RA6M5_PFS6_BASE         0x40080980UL
#define RA6M5_PFS7_BASE         0x400809C0UL
#define RA6M5_PFS8_BASE         0x40080A00UL
#define RA6M5_PFS9_BASE         0x40080A40UL
#define RA6M5_PFS10_BASE        0x40080A80UL
#define RA6M5_PFS11_BASE        0x40080AC0UL
#define RA6M5_PFS12_BASE        0x40080B00UL
#define RA6M5_PFS13_BASE        0x40080B40UL
#define RA6M5_PFS14_BASE        0x40080B80UL
#define RA6M5_PMISC_BASE        0x40080D00UL

#define RA6M5_RTC_BASE          0x40083000UL
 
#define RA6M5_ADC0_BASE         0x40170000UL
#define RA6M5_ADC1_BASE         0x40170200UL

#define RA6M5_USBFS_BASE        0x40090000UL
#define RA6M5_USBHS_BASE        0x40111000UL

#define RA6M5_OSPI_BASE         0x400A6000UL

#define RA6M5_SDHI0_BASE        0x40092000UL
#define RA6M5_SDHI1_BASE        0x40092400UL

#define RA6M5_SYSTEM_BASE       0x4001E000UL
 
#define RA6M5_CACHE_BASE        0x40007000UL
#define RA6M5_FCACHE_BASE       0x4001C000UL

#define RA6M5_MSTP_BASE         0x40084000UL

#define RA6M5_ICU_BASE          0x40006000UL
#define RA6M5_CPSCU_BASE        0x40008000UL

#endif /* __ARCH_ARM_SRC_RA6M5_HARDWARE_RA6M5_MEMORYMAP_H */
