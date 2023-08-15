/****************************************************************************
 * arch/arm/src/ra6m5/ra6m5_pwr.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include "arm_internal.h"
#include "ra6m5_pwr.h"
#include "ra6m5_rcc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: enableclk
 *
 * Description:
 *   Enable/disable the clock to the power control peripheral.  Enabling must
 *   be done after the APB1 clock is validly configured, and prior to using
 *   any functionality controlled by the PWR block (i.e. much of anything
 *   else provided by this module).
 *
 * Input Parameters:
 *   enable - True: enable the clock to the Power control (PWR) block.
 *
 * Returned Value:
 *   True:  the PWR block was previously enabled.
 *
 ****************************************************************************/

bool ra6m5_pwr_enableclk(bool enable)
{
  return false;
}

/****************************************************************************
 * Name: ra6m5_pwr_enablebkp
 *
 * Description:
 *   Enables access to the backup domain (RTC registers, RTC backup data
 *   registers and backup SRAM).
 *
 * Input Parameters:
 *   writable  True: enable ability to write to backup domain registers
 *
 * Returned Value:
 *   True: The backup domain was previously writable.
 *
 ****************************************************************************/

bool ra6m5_pwr_enablebkp(bool writable)
{
  return false;
}

/****************************************************************************
 * Name ra6m5_pwr_adjustvcore
 *
 * Description:
 *   Adjusts the voltage used for digital peripherals (V_CORE) before
 *   raising or after decreasing the system clock frequency.  Compare
 *   [RM0456], section 10.5.4 Dynamic voltage scaling management.
 *
 * Input Parameters:
 *   sysclock - The frequency in Hertz the system clock will or has been set
 *              to.
 *
 ****************************************************************************/

void ra6m5_pwr_adjustvcore(unsigned sysclock)
{
}

/****************************************************************************
 * Name ra6m5_pwr_enable_smps
 *
 * Description:
 *   Select between the Low-Drop Out (LDO) or Switched Mode Power Suppy
 *   (SMPS) regulator.  Compare [RM0456], section 10.5.1 SMPS and LDO
 *   embedded regulators.
 *
 * Input Parameters:
 *   enable - If true, the SMPS regulator will be enabled, otherwise the LDO,
 *
 ****************************************************************************/

void ra6m5_pwr_enablesmps(bool enable)
{
}
