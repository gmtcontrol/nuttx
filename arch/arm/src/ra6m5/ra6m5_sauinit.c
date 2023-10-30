/****************************************************************************
 * arch/arm/src/ra6m5/ra6m5_mpuinit.c
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

#include <assert.h>
#include <sys/param.h>

#include <nuttx/userspace.h>

#include "arm_internal.h"
#include "sau.h"
#include "ra6m5_sauinit.h"

#if defined(CONFIG_BUILD_PROTECTED)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra6m5_sauinitialize
 *
 * Description:
 *   Configure the MPU to permit user-space access to only restricted SAM3U
 *   resources.
 *
 ****************************************************************************/

void ra6m5_sauinitialize(void)
{
  uintptr_t datastart = MIN(USERSPACE->us_datastart, USERSPACE->us_bssstart);
  uintptr_t dataend   = MAX(USERSPACE->us_dataend,   USERSPACE->us_bssend);

  DEBUGASSERT(USERSPACE->us_textend >= USERSPACE->us_textstart &&
              dataend >= datastart);

  /* Show MPU information */

  sau_showtype();

  /* Configure user flash and SRAM space */

  sau_non_secure_callable(USERSPACE->us_textstart,
                          USERSPACE->us_textend - USERSPACE->us_textstart);

  sau_non_secure(datastart, dataend - datastart);

  /* Then enable the SAU */

  sau_control(false, false);
}

/****************************************************************************
 * Name: ra6m5_sau_uheap
 *
 * Description:
 *  Map the user-heap region.
 *
 *  This logic may need an extension to handle external SDRAM).
 *
 ****************************************************************************/

void ra6m5_sau_uheap(uintptr_t start, size_t size)
{
  sau_non_secure(start, size);
}

#endif /* CONFIG_BUILD_PROTECTED */
