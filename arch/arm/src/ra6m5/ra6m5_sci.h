/****************************************************************************
 * arch/arm/src/ra6m5/ra6m5_sci.h
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

#ifndef __ARCH_ARM_STC_RA6M5_RA6M5_SCI_H
#define __ARCH_ARM_STC_RA6M5_RA6M5_SCI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/serial/serial.h>

#include "chip.h"

#if defined(CONFIG_RA6M5_R7FA6M5BX)
#  include "hardware/ra6m5_sci.h"
#else
#  error "Unsupported RA6M5 chip"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Make sure that we have not enabled more U[S]ARTs than are supported by the
 * device.
 */

#if !defined(CONFIG_RA6M5_HAVE_SCI9)
#  undef CONFIG_RA6M5_SCI9
#endif
#if !defined(CONFIG_RA6M5_HAVE_SCI8)
#  undef CONFIG_RA6M5_SCI8
#endif
#if !defined(CONFIG_RA6M5_HAVE_SCI7)
#  undef CONFIG_RA6M5_SCI7
#endif
#if !defined(CONFIG_RA6M5_HAVE_SCI6)
#  undef CONFIG_RA6M5_SCI6
#endif
#if !defined(CONFIG_RA6M5_HAVE_SCI5)
#  undef CONFIG_RA6M5_SCI5
#endif
#if !defined(CONFIG_RA6M5_HAVE_SCI4)
#  undef CONFIG_RA6M5_SCI4
#endif
#if !defined(CONFIG_RA6M5_HAVE_SCI3)
#  undef CONFIG_RA6M5_SCI3
#endif
#if !defined(CONFIG_RA6M5_HAVE_SCI2)
#  undef CONFIG_RA6M5_SCI2
#endif
#if !defined(CONFIG_RA6M5_HAVE_SCI1)
#  undef CONFIG_RA6M5_SCI1
#endif
#if !defined(CONFIG_RA6M5_HAVE_SCI0)
#  undef CONFIG_RA6M5_SCI0
#endif

/* Sanity checks */

#if !defined(CONFIG_RA6M5_SCI0)
#  undef CONFIG_RA6M5_SCI0_UART
#  undef CONFIG_RA6M5_SCI0_1WIRE
#  undef CONFIG_RA6M5_SCI0_SPI
#  undef CONFIG_RA6M5_SCI0_IIC
#endif
#if !defined(CONFIG_RA6M5_SCI1)
#  undef CONFIG_RA6M5_SCI1_UART
#  undef CONFIG_RA6M5_SCI1_1WIRE
#  undef CONFIG_RA6M5_SCI1_SPI
#  undef CONFIG_RA6M5_SCI1_IIC
#endif
#if !defined(CONFIG_RA6M5_SCI2)
#  undef CONFIG_RA6M5_SCI2_UART
#  undef CONFIG_RA6M5_SCI2_1WIRE
#  undef CONFIG_RA6M5_SCI2_SPI
#  undef CONFIG_RA6M5_SCI2_IIC
#endif
#if !defined(CONFIG_RA6M5_SCI3)
#  undef CONFIG_RA6M5_SCI3_UART
#  undef CONFIG_RA6M5_SCI3_1WIRE
#  undef CONFIG_RA6M5_SCI3_SPI
#  undef CONFIG_RA6M5_SCI3_IIC
#endif
#if !defined(CONFIG_RA6M5_SCI4)
#  undef CONFIG_RA6M5_SCI4_UART
#  undef CONFIG_RA6M5_SCI4_1WIRE
#  undef CONFIG_RA6M5_SCI4_SPI
#  undef CONFIG_RA6M5_SCI4_IIC
#endif
#if !defined(CONFIG_RA6M5_SCI5)
#  undef CONFIG_RA6M5_SCI5_UART
#  undef CONFIG_RA6M5_SCI5_1WIRE
#  undef CONFIG_RA6M5_SCI5_SPI
#  undef CONFIG_RA6M5_SCI5_IIC
#endif
#if !defined(CONFIG_RA6M5_SCI6)
#  undef CONFIG_RA6M5_SCI6_UART
#  undef CONFIG_RA6M5_SCI6_1WIRE
#  undef CONFIG_RA6M5_SCI6_SPI
#  undef CONFIG_RA6M5_SCI6_IIC
#endif
#if !defined(CONFIG_RA6M5_SCI7)
#  undef CONFIG_RA6M5_SCI7_UART
#  undef CONFIG_RA6M5_SCI7_1WIRE
#  undef CONFIG_RA6M5_SCI7_SPI
#  undef CONFIG_RA6M5_SCI7_IIC
#endif
#if !defined(CONFIG_RA6M5_SCI8)
#  undef CONFIG_RA6M5_SCI8_UART
#  undef CONFIG_RA6M5_SCI8_1WIRE
#  undef CONFIG_RA6M5_SCI8_SPI
#  undef CONFIG_RA6M5_SCI8_IIC
#endif
#if !defined(CONFIG_RA6M5_SCI9)
#  undef CONFIG_RA6M5_SCI9_UART
#  undef CONFIG_RA6M5_SCI9_1WIRE
#  undef CONFIG_RA6M5_SCI9_SPI
#  undef CONFIG_RA6M5_SCI9_IIC
#endif

/* Is there a SCI enabled? */

#if defined(CONFIG_RA6M5_SCI0_UART) || defined(CONFIG_RA6M5_SCI1_UART) || \
    defined(CONFIG_RA6M5_SCI2_UART) || defined(CONFIG_RA6M5_SCI3_UART) || \
    defined(CONFIG_RA6M5_SCI4_UART) || defined(CONFIG_RA6M5_SCI5_UART) || \
    defined(CONFIG_RA6M5_SCI6_UART) || defined(CONFIG_RA6M5_SCI7_UART) || \
    defined(CONFIG_RA6M5_SCI8_UART) || defined(CONFIG_RA6M5_SCI9_UART)
#  define HAVE_UART 1
#endif

/* Is there a serial console? */

#if defined(CONFIG_SCI0_SERIAL_CONSOLE) && defined(CONFIG_RA6M5_SCI0_UART)
#  undef CONFIG_SCI1_SERIAL_CONSOLE
#  undef CONFIG_SCI2_SERIAL_CONSOLE
#  undef CONFIG_SCI3_SERIAL_CONSOLE
#  undef CONFIG_SCI4_SERIAL_CONSOLE
#  undef CONFIG_SCI5_SERIAL_CONSOLE
#  undef CONFIG_SCI6_SERIAL_CONSOLE
#  undef CONFIG_SCI7_SERIAL_CONSOLE
#  undef CONFIG_SCI8_SERIAL_CONSOLE
#  undef CONFIG_SCI9_SERIAL_CONSOLE
#  define CONSOLE_UART 0
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_SCI1_SERIAL_CONSOLE) && defined(CONFIG_RA6M5_SCI1_UART)
#  undef CONFIG_SCI0_SERIAL_CONSOLE
#  undef CONFIG_SCI2_SERIAL_CONSOLE
#  undef CONFIG_SCI3_SERIAL_CONSOLE
#  undef CONFIG_SCI4_SERIAL_CONSOLE
#  undef CONFIG_SCI5_SERIAL_CONSOLE
#  undef CONFIG_SCI6_SERIAL_CONSOLE
#  undef CONFIG_SCI7_SERIAL_CONSOLE
#  undef CONFIG_SCI8_SERIAL_CONSOLE
#  undef CONFIG_SCI9_SERIAL_CONSOLE
#  define CONSOLE_UART 1
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_SCI2_SERIAL_CONSOLE) && defined(CONFIG_RA6M5_SCI2_UART)
#  undef CONFIG_SCI0_SERIAL_CONSOLE
#  undef CONFIG_SCI1_SERIAL_CONSOLE
#  undef CONFIG_SCI3_SERIAL_CONSOLE
#  undef CONFIG_SCI4_SERIAL_CONSOLE
#  undef CONFIG_SCI5_SERIAL_CONSOLE
#  undef CONFIG_SCI6_SERIAL_CONSOLE
#  undef CONFIG_SCI7_SERIAL_CONSOLE
#  undef CONFIG_SCI8_SERIAL_CONSOLE
#  undef CONFIG_SCI9_SERIAL_CONSOLE
#  define CONSOLE_UART 2
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_SCI3_SERIAL_CONSOLE) && defined(CONFIG_RA6M5_SCI3_UART)
#  undef CONFIG_SCI0_SERIAL_CONSOLE
#  undef CONFIG_SCI1_SERIAL_CONSOLE
#  undef CONFIG_SCI2_SERIAL_CONSOLE
#  undef CONFIG_SCI4_SERIAL_CONSOLE
#  undef CONFIG_SCI5_SERIAL_CONSOLE
#  undef CONFIG_SCI6_SERIAL_CONSOLE
#  undef CONFIG_SCI7_SERIAL_CONSOLE
#  undef CONFIG_SCI8_SERIAL_CONSOLE
#  undef CONFIG_SCI9_SERIAL_CONSOLE
#  define CONSOLE_UART 3
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_SCI4_SERIAL_CONSOLE) && defined(CONFIG_RA6M5_SCI4_UART)
#  undef CONFIG_SCI0_SERIAL_CONSOLE
#  undef CONFIG_SCI1_SERIAL_CONSOLE
#  undef CONFIG_SCI2_SERIAL_CONSOLE
#  undef CONFIG_SCI3_SERIAL_CONSOLE
#  undef CONFIG_SCI5_SERIAL_CONSOLE
#  undef CONFIG_SCI6_SERIAL_CONSOLE
#  undef CONFIG_SCI7_SERIAL_CONSOLE
#  undef CONFIG_SCI8_SERIAL_CONSOLE
#  undef CONFIG_SCI9_SERIAL_CONSOLE
#  define CONSOLE_UART 4
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_SCI5_SERIAL_CONSOLE) && defined(CONFIG_RA6M5_SCI5_UART)
#  undef CONFIG_SCI0_SERIAL_CONSOLE
#  undef CONFIG_SCI1_SERIAL_CONSOLE
#  undef CONFIG_SCI2_SERIAL_CONSOLE
#  undef CONFIG_SCI3_SERIAL_CONSOLE
#  undef CONFIG_SCI4_SERIAL_CONSOLE
#  undef CONFIG_SCI6_SERIAL_CONSOLE
#  undef CONFIG_SCI7_SERIAL_CONSOLE
#  undef CONFIG_SCI8_SERIAL_CONSOLE
#  undef CONFIG_SCI9_SERIAL_CONSOLE
#  define CONSOLE_UART 5
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_SCI6_SERIAL_CONSOLE) && defined(CONFIG_RA6M5_SCI6_UART)
#  undef CONFIG_SCI0_SERIAL_CONSOLE
#  undef CONFIG_SCI1_SERIAL_CONSOLE
#  undef CONFIG_SCI2_SERIAL_CONSOLE
#  undef CONFIG_SCI3_SERIAL_CONSOLE
#  undef CONFIG_SCI4_SERIAL_CONSOLE
#  undef CONFIG_SCI5_SERIAL_CONSOLE
#  undef CONFIG_SCI7_SERIAL_CONSOLE
#  undef CONFIG_SCI8_SERIAL_CONSOLE
#  undef CONFIG_SCI9_SERIAL_CONSOLE
#  define CONSOLE_UART 6
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_SCI7_SERIAL_CONSOLE) && defined(CONFIG_RA6M5_SCI7_UART)
#  undef CONFIG_SCI0_SERIAL_CONSOLE
#  undef CONFIG_SCI1_SERIAL_CONSOLE
#  undef CONFIG_SCI2_SERIAL_CONSOLE
#  undef CONFIG_SCI3_SERIAL_CONSOLE
#  undef CONFIG_SCI4_SERIAL_CONSOLE
#  undef CONFIG_SCI5_SERIAL_CONSOLE
#  undef CONFIG_SCI6_SERIAL_CONSOLE
#  undef CONFIG_SCI8_SERIAL_CONSOLE
#  undef CONFIG_SCI9_SERIAL_CONSOLE
#  define CONSOLE_UART 7
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_SCI8_SERIAL_CONSOLE) && defined(CONFIG_RA6M5_SCI8_UART)
#  undef CONFIG_SCI0_SERIAL_CONSOLE
#  undef CONFIG_SCI1_SERIAL_CONSOLE
#  undef CONFIG_SCI2_SERIAL_CONSOLE
#  undef CONFIG_SCI3_SERIAL_CONSOLE
#  undef CONFIG_SCI4_SERIAL_CONSOLE
#  undef CONFIG_SCI5_SERIAL_CONSOLE
#  undef CONFIG_SCI6_SERIAL_CONSOLE
#  undef CONFIG_SCI7_SERIAL_CONSOLE
#  undef CONFIG_SCI9_SERIAL_CONSOLE
#  define CONSOLE_UART 8
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_SCI9_SERIAL_CONSOLE) && defined(CONFIG_RA6M5_SCI9_UART)
#  undef CONFIG_SCI0_SERIAL_CONSOLE
#  undef CONFIG_SCI1_SERIAL_CONSOLE
#  undef CONFIG_SCI2_SERIAL_CONSOLE
#  undef CONFIG_SCI3_SERIAL_CONSOLE
#  undef CONFIG_SCI4_SERIAL_CONSOLE
#  undef CONFIG_SCI5_SERIAL_CONSOLE
#  undef CONFIG_SCI6_SERIAL_CONSOLE
#  undef CONFIG_SCI7_SERIAL_CONSOLE
#  undef CONFIG_SCI8_SERIAL_CONSOLE
#  define CONSOLE_UART 9
#  define HAVE_CONSOLE 1
#else
#  undef CONFIG_SCI0_SERIAL_CONSOLE
#  undef CONFIG_SCI1_SERIAL_CONSOLE
#  undef CONFIG_SCI2_SERIAL_CONSOLE
#  undef CONFIG_SCI3_SERIAL_CONSOLE
#  undef CONFIG_SCI4_SERIAL_CONSOLE
#  undef CONFIG_SCI5_SERIAL_CONSOLE
#  undef CONFIG_SCI6_SERIAL_CONSOLE
#  undef CONFIG_SCI7_SERIAL_CONSOLE
#  undef CONFIG_SCI8_SERIAL_CONSOLE
#  undef CONFIG_SCI9_SERIAL_CONSOLE
#  define CONSOLE_UART 0
#  undef HAVE_CONSOLE
#endif

/* DMA support is only provided if CONFIG_ARCH_DMA is in the NuttX
 * configuration
 */

#if !defined(HAVE_UART) || !defined(CONFIG_ARCH_DMA)
#  undef CONFIG_SCI0_RXDMA
#  undef CONFIG_SCI1_RXDMA
#  undef CONFIG_SCI2_RXDMA
#  undef CONFIG_SCI3_RXDMA
#  undef CONFIG_SCI4_RXDMA
#  undef CONFIG_SCI5_RXDMA
#  undef CONFIG_SCI6_RXDMA
#  undef CONFIG_SCI7_RXDMA
#  undef CONFIG_SCI8_RXDMA
#  undef CONFIG_SCI9_RXDMA
#endif

/* Disable the DMA/DTC configuration on all unused SCIs */

#ifndef CONFIG_RA6M5_SCI0_UART
#  undef CONFIG_SCI0_RXDMA
#  undef CONFIG_SCI0_RXDTC
#  undef CONFIG_SCI0_TXDTC
#endif

#ifndef CONFIG_RA6M5_SCI1_UART
#  undef CONFIG_SCI1_RXDMA
#  undef CONFIG_SCI1_RXDTC
#  undef CONFIG_SCI1_TXDTC
#endif

#ifndef CONFIG_RA6M5_SCI2_UART
#  undef CONFIG_SCI2_RXDMA
#  undef CONFIG_SCI2_RXDTC
#  undef CONFIG_SCI2_TXDTC
#endif

#ifndef CONFIG_RA6M5_SCI3_UART
#  undef CONFIG_SCI3_RXDMA
#  undef CONFIG_SCI3_RXDTC
#  undef CONFIG_SCI3_TXDTC
#endif

#ifndef CONFIG_RA6M5_SCI4_UART
#  undef CONFIG_SCI4_RXDMA
#  undef CONFIG_SCI4_RXDTC
#  undef CONFIG_SCI4_TXDTC
#endif

#ifndef CONFIG_RA6M5_SCI5_UART
#  undef CONFIG_SCI5_RXDMA
#  undef CONFIG_SCI5_RXDTC
#  undef CONFIG_SCI5_TXDTC
#endif

#ifndef CONFIG_RA6M5_SCI6_UART
#  undef CONFIG_SCI6_RXDMA
#  undef CONFIG_SCI6_RXDTC
#  undef CONFIG_SCI6_TXDTC
#endif

#ifndef CONFIG_RA6M5_SCI7_UART
#  undef CONFIG_SCI7_RXDMA
#  undef CONFIG_SCI7_RXDTC
#  undef CONFIG_SCI7_TXDTC
#endif

#ifndef CONFIG_RA6M5_SCI8_UART
#  undef CONFIG_SCI8_RXDMA
#  undef CONFIG_SCI8_RXDTC
#  undef CONFIG_SCI8_TXDTC
#endif

#ifndef CONFIG_RA6M5_SCI9_UART
#  undef CONFIG_SCI9_RXDMA
#  undef CONFIG_SCI9_RXDTC
#  undef CONFIG_SCI9_TXDTC
#endif


/* Is DMA available on any (enabled) SCI? */

#undef SERIAL_HAVE_DMA
#if defined(CONFIG_SCI0_RXDMA) || defined(CONFIG_SCI1_RXDMA) || \
    defined(CONFIG_SCI2_RXDMA) || defined(CONFIG_SCI3_RXDMA) || \
    defined(CONFIG_SCI4_RXDMA) || defined(CONFIG_SCI5_RXDMA) || \
    defined(CONFIG_SCI6_RXDMA) || defined(CONFIG_SCI7_RXDMA) || \
    defined(CONFIG_SCI8_RXDMA) || defined(CONFIG_SCI9_RXDMA)
#  define SERIAL_HAVE_DMA 1
#endif

/* Is DMA used on the console UART? */

#undef SERIAL_HAVE_CONSOLE_DMA
#if defined(CONFIG_SCI0_SERIAL_CONSOLE) && defined(CONFIG_SCI0_RXDMA)
#  define SERIAL_HAVE_CONSOLE_DMA 1
#elif defined(CONFIG_SCI1_SERIAL_CONSOLE) && defined(CONFIG_SCI1_RXDMA)
#  define SERIAL_HAVE_CONSOLE_DMA 1
#elif defined(CONFIG_SCI2_SERIAL_CONSOLE) && defined(CONFIG_SCI2_RXDMA)
#  define SERIAL_HAVE_CONSOLE_DMA 1
#elif defined(CONFIG_SCI3_SERIAL_CONSOLE) && defined(CONFIG_SCI3_RXDMA)
#  define SERIAL_HAVE_CONSOLE_DMA 1
#elif defined(CONFIG_SCI4_SERIAL_CONSOLE) && defined(CONFIG_SCI4_RXDMA)
#  define SERIAL_HAVE_CONSOLE_DMA 1
#elif defined(CONFIG_SCI5_SERIAL_CONSOLE) && defined(CONFIG_SCI5_RXDMA)
#  define SERIAL_HAVE_CONSOLE_DMA 1
#elif defined(CONFIG_SCI6_SERIAL_CONSOLE) && defined(CONFIG_SCI6_RXDMA)
#  define SERIAL_HAVE_CONSOLE_DMA 1
#elif defined(CONFIG_SCI7_SERIAL_CONSOLE) && defined(CONFIG_SCI7_RXDMA)
#  define SERIAL_HAVE_CONSOLE_DMA 1
#elif defined(CONFIG_SCI8_SERIAL_CONSOLE) && defined(CONFIG_SCI8_RXDMA)
#  define SERIAL_HAVE_CONSOLE_DMA 1
#elif defined(CONFIG_SCI9_SERIAL_CONSOLE) && defined(CONFIG_SCI9_RXDMA)
#  define SERIAL_HAVE_CONSOLE_DMA 1
#endif

/* Is DMA used on all (enabled) SCIs */

#define SERIAL_HAVE_ONLY_DMA 1
#if defined(CONFIG_RA6M5_SCI0_UART) && !defined(CONFIG_SCI0_RXDMA)
#  undef SERIAL_HAVE_ONLY_DMA
#elif defined(CONFIG_RA6M5_SCI1_UART) && !defined(CONFIG_SCI1_RXDMA)
#  undef SERIAL_HAVE_ONLY_DMA
#elif defined(CONFIG_RA6M5_SCI2_UART) && !defined(CONFIG_SCI2_RXDMA)
#  undef SERIAL_HAVE_ONLY_DMA
#elif defined(CONFIG_RA6M5_SCI3_UART) && !defined(CONFIG_SCI3_RXDMA)
#  undef SERIAL_HAVE_ONLY_DMA
#elif defined(CONFIG_RA6M5_SCI4_UART) && !defined(CONFIG_SCI4_RXDMA)
#  undef SERIAL_HAVE_ONLY_DMA
#elif defined(CONFIG_RA6M5_SCI5_UART) && !defined(CONFIG_SCI5_RXDMA)
#  undef SERIAL_HAVE_ONLY_DMA
#elif defined(CONFIG_RA6M5_SCI6_UART) && !defined(CONFIG_SCI6_RXDMA)
#  undef SERIAL_HAVE_ONLY_DMA
#elif defined(CONFIG_RA6M5_SCI7_UART) && !defined(CONFIG_SCI7_RXDMA)
#  undef SERIAL_HAVE_ONLY_DMA
#elif defined(CONFIG_RA6M5_SCI8_UART) && !defined(CONFIG_SCI8_RXDMA)
#  undef SERIAL_HAVE_ONLY_DMA
#elif defined(CONFIG_RA6M5_SCI9_UART) && !defined(CONFIG_SCI9_RXDMA)
#  undef SERIAL_HAVE_ONLY_DMA
#endif

/* DTC support is only provided if CONFIG_ARCH_DTC is in the NuttX
 * configuration
 */

#if !defined(HAVE_UART) || !defined(CONFIG_ARCH_DTC)
#  undef CONFIG_SCI0_RXDTC
#  undef CONFIG_SCI1_RXDTC
#  undef CONFIG_SCI2_RXDTC
#  undef CONFIG_SCI3_RXDTC
#  undef CONFIG_SCI4_RXDTC
#  undef CONFIG_SCI5_RXDTC
#  undef CONFIG_SCI6_RXDTC
#  undef CONFIG_SCI7_RXDTC
#  undef CONFIG_SCI8_RXDTC
#  undef CONFIG_SCI9_RXDTC
#  undef CONFIG_SCI0_TXDTC
#  undef CONFIG_SCI1_TXDTC
#  undef CONFIG_SCI2_TXDTC
#  undef CONFIG_SCI3_TXDTC
#  undef CONFIG_SCI4_TXDTC
#  undef CONFIG_SCI5_TXDTC
#  undef CONFIG_SCI6_TXDTC
#  undef CONFIG_SCI7_TXDTC
#  undef CONFIG_SCI8_TXDTC
#  undef CONFIG_SCI9_TXDTC
#endif

/* Is DTC available on any (enabled) SCI? */

#undef SERIAL_HAVE_DTC
#if defined(CONFIG_SCI0_RXDTC) || defined(CONFIG_SCI1_RXDTC) || \
    defined(CONFIG_SCI2_RXDTC) || defined(CONFIG_SCI3_RXDTC) || \
    defined(CONFIG_SCI4_RXDTC) || defined(CONFIG_SCI5_RXDTC) || \
    defined(CONFIG_SCI6_RXDTC) || defined(CONFIG_SCI7_RXDTC) || \
    defined(CONFIG_SCI8_RXDTC) || defined(CONFIG_SCI9_RXDTC) || \
	defined(CONFIG_SCI0_TXDTC) || defined(CONFIG_SCI1_TXDTC) || \
    defined(CONFIG_SCI2_TXDTC) || defined(CONFIG_SCI3_TXDTC) || \
    defined(CONFIG_SCI4_TXDTC) || defined(CONFIG_SCI5_TXDTC) || \
    defined(CONFIG_SCI6_TXDTC) || defined(CONFIG_SCI7_TXDTC) || \
    defined(CONFIG_SCI8_TXDTC) || defined(CONFIG_SCI9_TXDTC)
#  define SERIAL_HAVE_DTC 1
#endif

/* Is DTC used on the console UART? */

#undef SERIAL_HAVE_CONSOLE_DTC
#if defined(CONFIG_SCI0_SERIAL_CONSOLE) && (defined(CONFIG_SCI0_RXDTC) || defined(CONFIG_SCI0_TXDTC))
#  define SERIAL_HAVE_CONSOLE_DTC 1
#elif defined(CONFIG_SCI1_SERIAL_CONSOLE) && (defined(CONFIG_SCI1_RXDTC) || defined(CONFIG_SCI1_TXDTC))
#  define SERIAL_HAVE_CONSOLE_DTC 1
#elif defined(CONFIG_SCI2_SERIAL_CONSOLE) && (defined(CONFIG_SCI2_RXDTC) || defined(CONFIG_SCI2_TXDTC))
#  define SERIAL_HAVE_CONSOLE_DTC 1
#elif defined(CONFIG_SCI3_SERIAL_CONSOLE) && (defined(CONFIG_SCI3_RXDTC) || defined(CONFIG_SCI3_TXDTC))
#  define SERIAL_HAVE_CONSOLE_DTC 1
#elif defined(CONFIG_SCI4_SERIAL_CONSOLE) && (defined(CONFIG_SCI4_RXDTC) || defined(CONFIG_SCI4_TXDTC))
#  define SERIAL_HAVE_CONSOLE_DTC 1
#elif defined(CONFIG_SCI5_SERIAL_CONSOLE) && (defined(CONFIG_SCI5_RXDTC) || defined(CONFIG_SCI5_TXDTC))
#  define SERIAL_HAVE_CONSOLE_DTC 1
#elif defined(CONFIG_SCI6_SERIAL_CONSOLE) && (defined(CONFIG_SCI6_RXDTC) || defined(CONFIG_SCI6_TXDTC))
#  define SERIAL_HAVE_CONSOLE_DTC 1
#elif defined(CONFIG_SCI7_SERIAL_CONSOLE) && (defined(CONFIG_SCI7_RXDTC) || defined(CONFIG_SCI7_TXDTC))
#  define SERIAL_HAVE_CONSOLE_DTC 1
#elif defined(CONFIG_SCI8_SERIAL_CONSOLE) && (defined(CONFIG_SCI8_RXDTC) || defined(CONFIG_SCI8_TXDTC))
#  define SERIAL_HAVE_CONSOLE_DTC 1
#elif defined(CONFIG_SCI9_SERIAL_CONSOLE) && (defined(CONFIG_SCI9_RXDTC) || defined(CONFIG_SCI9_TXDTC))
#  define SERIAL_HAVE_CONSOLE_DTC 1
#endif


/* Is DTC used on all (enabled) SCIs */

#define SERIAL_HAVE_ONLY_DTC 1
#if defined(CONFIG_RA6M5_SCI0_UART) && (!defined(CONFIG_SCI0_RXDTC) && !defined(CONFIG_SCI0_TXDTC))
#  undef SERIAL_HAVE_ONLY_DTC
#elif defined(CONFIG_RA6M5_SCI1_UART) && (!defined(CONFIG_SCI1_RXDTC) && !defined(CONFIG_SCI1_TXDTC))
#  undef SERIAL_HAVE_ONLY_DTC
#elif defined(CONFIG_RA6M5_SCI2_UART) && (!defined(CONFIG_SCI2_RXDTC) && !defined(CONFIG_SCI2_TXDTC))
#  undef SERIAL_HAVE_ONLY_DTC
#elif defined(CONFIG_RA6M5_SCI3_UART) && (!defined(CONFIG_SCI3_RXDTC) && !defined(CONFIG_SCI3_TXDTC))
#  undef SERIAL_HAVE_ONLY_DTC
#elif defined(CONFIG_RA6M5_SCI4_UART) && (!defined(CONFIG_SCI4_RXDTC) && !defined(CONFIG_SCI4_TXDTC))
#  undef SERIAL_HAVE_ONLY_DTC
#elif defined(CONFIG_RA6M5_SCI5_UART) && (!defined(CONFIG_SCI5_RXDTC) && !defined(CONFIG_SCI5_TXDTC))
#  undef SERIAL_HAVE_ONLY_DTC
#elif defined(CONFIG_RA6M5_SCI6_UART) && (!defined(CONFIG_SCI6_RXDTC) && !defined(CONFIG_SCI6_TXDTC))
#  undef SERIAL_HAVE_ONLY_DTC
#elif defined(CONFIG_RA6M5_SCI7_UART) && (!defined(CONFIG_SCI7_RXDTC) && !defined(CONFIG_SCI7_TXDTC))
#  undef SERIAL_HAVE_ONLY_DTC
#elif defined(CONFIG_RA6M5_SCI8_UART) && (!defined(CONFIG_SCI8_RXDTC) && !defined(CONFIG_SCI8_TXDTC))
#  undef SERIAL_HAVE_ONLY_DTC
#elif defined(CONFIG_RA6M5_SCI9_UART) && (!defined(CONFIG_SCI9_RXDTC) && !defined(CONFIG_SCI9_TXDTC))
#  undef SERIAL_HAVE_ONLY_DTC
#endif

/* Is RS-485 used? */

#if defined(CONFIG_SCI0_RS485) || defined(CONFIG_SCI1_RS485) || \
    defined(CONFIG_SCI2_RS485) || defined(CONFIG_SCI3_RS485) || \
    defined(CONFIG_SCI4_RS485) || defined(CONFIG_SCI5_RS485) || \
    defined(CONFIG_SCI6_RS485) || defined(CONFIG_SCI7_RS485) || \
    defined(CONFIG_SCI8_RS485) || defined(CONFIG_SCI8_RS485)
#  define HAVE_RS485 1
#endif

#ifdef HAVE_RS485
#  define SCI_SCR_USED_INTS    (SCI_SCR_RIE | SCI_SCR_TIE | SCI_SCR_TEIE)
#else
#  define SCI_SCR_USED_INTS    (SCI_SCR_RIE | SCI_SCR_TIE)
#endif

/* Number of divisors in the data table used for baud rate calculation. */
#define SCI_UART_NUM_DIVISORS_ASYNC     (13U)

/* Valid range of values for the modulation duty register is 128 - 256 (256 = modulation disabled). */
#define SCI_UART_MDDR_MIN               (128U)
#define SCI_UART_MDDR_MAX               (256U)

/* The bit rate register is 8-bits, so the maximum value is 255. */
#define SCI_UART_BRR_MAX                (255U)


/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef struct st_baud_setting_const_t
{
    uint8_t bgdm  : 1;                 /**< BGDM value to get divisor */
    uint8_t abcs  : 1;                 /**< ABCS value to get divisor */
    uint8_t abcse : 1;                 /**< ABCSE value to get divisor */
    uint8_t cks   : 2;                 /**< CKS  value to get divisor (CKS = N) */
} baud_setting_const_t;


/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ra6m5_serial_dma_poll
 *
 * Description:
 *   Must be called periodically if any RA6M5 UART is configured for DMA.
 *   The DMA callback is triggered for each fifo size/2 bytes, but this can
 *   result in some bytes being transferred but not collected if the incoming
 *   data is not a whole multiple of half the FIFO size.
 *
 *   May be safely called from either interrupt or thread context.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_DMA
void ra6m5_serial_dma_poll(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_STC_RA6M5_RA6M5_SCI_H */
