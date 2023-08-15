/****************************************************************************
 * arch/arm/src/ra6m5/hardware/r7fa6m5bx_dbgmcu.h
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

#ifndef __ARCH_ARM_SRC_RA6M5_HARDWARE_R7FA6M5BX_DBGMCU_H
#define __ARCH_ARM_SRC_RA6M5_HARDWARE_R7FA6M5BX_DBGMCU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Addresses *******************************************************/

#define RA6M5_DBGMCU_IDCODE       0xe0044000  /* MCU identifier */
#define RA6M5_DBGMCU_CR           0xe0044004  /* MCU debug */
#define RA6M5_DBGMCU_APB1_FZ      0xe0044008  /* Debug MCU APB1 freeze register */
#define RA6M5_DBGMCU_APB1_FZ2     0xe004400c  /* Debug MCU APB1 freeze register 2 */
#define RA6M5_DBGMCU_APB2_FZ      0xe0044010  /* Debug MCU APB2 freeze register */

/* Register Bitfield Definitions ********************************************/

/* MCU identifier */

#define DBGMCU_IDCODE_DEVID_SHIFT (0)       /* Bits 11-0: Device Identifier */
#define DBGMCU_IDCODE_DEVID_MASK  (0x0fff << DBGMCU_IDCODE_DEVID_SHIFT)
#define DBGMCU_IDCODE_REVID_SHIFT (16)      /* Bits 31-16:  Revision Identifier */
#define DBGMCU_IDCODE_REVID_MASK  (0xffff << DBGMCU_IDCODE_REVID_SHIFT)

/* MCU debug */

#define DBGMCU_CR_STOP            (1 << 1)  /* Bit 1: Allows debug in Stop mode */
#define DBGMCU_CR_STANDBY         (1 << 2)  /* Bit 2: Allows debug in Standby mode */
#define DBGMCU_CR_TRACEIOEN       (1 << 4)  /* Bit 4: Trace pin enable */
#define DBGMCU_CR_TRACEEN         (1 << 5)  /* Bit 5: Trace port and clock enable */
#define DBGMCU_CR_TRACEMODE_SHIFT (6)       /* Bits 7-6: Trace mode pin assignment */
#define DBGMCU_CR_TRACEMODE_MASK  (3 << DBGMCU_CR_TRACEMODE_SHIFT)
#define DBGMCU_CR_ASYNCH          (0 << DBGMCU_CR_TRACEMODE_SHIFT) /* Asynchronous Mode */
#define DBGMCU_CR_SYNCH1          (1 << DBGMCU_CR_TRACEMODE_SHIFT) /* Synchronous Mode, TRACEDATA size=1 */
#define DBGMCU_CR_SYNCH2          (2 << DBGMCU_CR_TRACEMODE_SHIFT) /* Synchronous Mode, TRACEDATA size=2 */
#define DBGMCU_CR_SYNCH4          (3 << DBGMCU_CR_TRACEMODE_SHIFT) /* Synchronous Mode, TRACEDATA size=4 */

#endif /* __ARCH_ARM_SRC_RA6M5_HARDWARE_R7FA6M5BXDBGMCU_H */
