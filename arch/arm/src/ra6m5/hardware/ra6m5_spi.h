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

#ifndef __ARCH_ARM_SRC_RA6M5_HARDWARE_RA6M5_SPI_H
#define __ARCH_ARM_SRC_RA6M5_HARDWARE_RA6M5_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define RA6M5_SPI_SPCR_OFFSET   0x0000  /* SPI Control Register */
#define RA6M5_SPI_SSLP_OFFSET   0x0001  /* SPI Slave Select Polarity Register */
#define RA6M5_SPI_SPPCR_OFFSET  0x0002  /* SPI Pin Control Register */
#define RA6M5_SPI_SPSR_OFFSET   0x0003  /* SPI Status Register */
#define RA6M5_SPI_SPDR_OFFSET   0x0004  /* SPI Data Register */
#define RA6M5_SPI_SPSCR_OFFSET  0x0008  /* SPI Sequence Control Register */
#define RA6M5_SPI_SPSSR_OFFSET  0x0009  /* SPI Sequence Status Register */
#define RA6M5_SPI_SPBR_OFFSET   0x000A  /* SPI Bit Rate Register */
#define RA6M5_SPI_SPDCR_OFFSET  0x000B  /* SPI Data Control Register */
#define RA6M5_SPI_SPCKD_OFFSET  0x000C  /* SPI Clock Delay Register */
#define RA6M5_SPI_SSLND_OFFSET  0x000D  /* SPI Slave Select Negation Delay Register */
#define RA6M5_SPI_SPND_OFFSET   0x000E  /* SPI Next-Access Delay Register */
#define RA6M5_SPI_SPCR2_OFFSET  0x000F  /* SPI Control Register 2 */
#define RA6M5_SPI_SPCMD0_OFFSET 0x0010  /* SPI Command Register 0 */
#define RA6M5_SPI_SPCMD1_OFFSET 0x0012  /* SPI Command Register 1 */
#define RA6M5_SPI_SPCMD2_OFFSET 0x0014  /* SPI Command Register 2 */
#define RA6M5_SPI_SPCMD3_OFFSET 0x0016  /* SPI Command Register 3 */
#define RA6M5_SPI_SPCMD4_OFFSET 0x0018  /* SPI Command Register 4 */
#define RA6M5_SPI_SPCMD5_OFFSET 0x001A  /* SPI Command Register 5 */
#define RA6M5_SPI_SPCMD6_OFFSET 0x001C  /* SPI Command Register 6 */
#define RA6M5_SPI_SPCMD7_OFFSET 0x001E  /* SPI Command Register 7 */
#define RA6M5_SPI_SPDCR2_OFFSET 0x0020  /* SPI Data Control Register 2 */
#define RA6M5_SPI_SPCR3_OFFSET  0x0021  /* SPI Control Register 3 */

#endif /* __ARCH_ARM_SRC_RA6M5_HARDWARE_RA6M5_SPI_H */
