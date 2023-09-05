/****************************************************************************
 * arch/arm/src/ra6m5/ra6m5_eth.h
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

#ifndef __ARCH_ARM_SRC_RA6M5_RA6M5_ETH_H
#define __ARCH_ARM_SRC_RA6M5_RA6M5_ETH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/wdog.h>

#include "arm_internal.h"
#include "chip.h"

#include "hardware/ra6m5_eth.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Understood PHY types */

/* Definitions for use with ra6m5_phy_boardinitialize */

#define RA6M5_NETHERNET 1

#define EMAC0_INTF 0

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
 * Function: ra6m5_ethinitialize
 *
 * Description:
 *   Initialize the EMAC driver.
 *
 * Input Parameters:
 *   intf - If multiple EMAC peripherals are supported, this identifies the
 *     the EMAC peripheral being initialized.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *   Called very early in the initialization sequence.
 *
 ****************************************************************************/

#ifdef CONFIG_RA6M5_EMAC0
int ra6m5_ethinitialize(int intf);

/****************************************************************************
 * Function: ra6m5_txtimeout_expiry
 *
 * Description:
 *   txtimeout timer
 *
 * Input Parameters:
 *   arg  - Input argument
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

void ra6m5_txtimeout_expiry(wdparm_t arg);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RA6M5_RA6M5_ETH_H */
