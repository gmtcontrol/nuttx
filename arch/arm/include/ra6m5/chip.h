/****************************************************************************
 * arch/arm/include/ra6m5/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_RA6M5_CHIP_H
#define __ARCH_ARM_INCLUDE_RA6M5_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

#if defined(CONFIG_RA6M5_R7FA6M5BX)
#  define RA6M5_SRAM0_SIZE       (512*1024)  /* 512Kb SRAM0 */
#else
#  error "Unsupported RA6M5 chip"
#endif

#if defined(CONFIG_RA6M5_R7FA6M5BX)
#  define RA6M5_NAGT                     6   /* Low Power Asynchronous General Purpose Timer */
#  define RA6M5_NGPT                     10  /* General PWM Timer */
#  define RA6M5_NSCI                     10  /* Serial Communications Interface */
#  define RA6M5_QSPI                     1   /* Quad SPI */
#  define RA6M5_OSPI                     1   /* Octa SPI */
#  define RA6M5_NSPI                     2   /* Serial Peripheral Interface */
#  define RA6M5_NI2C                     3   /* I2C Bus Interface */
#  define RA6M5_NUSBFS                   1   /* USB 2.0 Full-Speed Module */
#  define RA6M5_NUSBHS                   0   /* USB 2.0 High-Speed Module */
#  define RA6M5_NCAN                     2   /* CAN */
#  define RA6M5_NSDHI                    1   /* SD/MMC Host Interface */
#  define RA6M5_NDMA                     1   /* DMA Controller */
#  define RA6M5_NPORTS                   10  /* 10 I/O ports */
#  define RA6M5_NADC                     1   /* 12-bit ADC1, up to 20 channels */
#  define RA6M5_NCRC                     1   /* CRC */
#endif /* CONFIG_RA6M5_R7FA6M5BX */

/* NVIC priority levels *****************************************************/

/* 16 Programmable interrupt levels */

#define NVIC_SYSH_PRIORITY_MIN     0xf0 /* All bits set in minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x10 /* Four bits of interrupt priority used */

#endif /* __ARCH_ARM_INCLUDE_RA6M5_CHIP_H */
