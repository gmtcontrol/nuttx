/****************************************************************************
 * arch/arm/src/ra6m5/ra6m5_dumpgpio.c
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

/* Output debug info even if debug output is not selected. */

#undef  CONFIG_DEBUG_INFO
#define CONFIG_DEBUG_INFO 1

#include <sys/types.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>

#include "arm_internal.h"
#include "chip.h"
#include "ra6m5_gpio.h"
#include "ra6m5_rcc.h"

#ifdef CONFIG_DEBUG_FEATURES

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  ra6m5_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the provided base address
 *
 ****************************************************************************/

int ra6m5_dumpgpio(uint32_t pinset, const char *msg)
{
  return OK;
}

#endif /* CONFIG_DEBUG_FEATURES */
