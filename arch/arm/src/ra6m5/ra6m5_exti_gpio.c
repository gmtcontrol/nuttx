/****************************************************************************
 * arch/arm/src/ra6m5/ra6m5_exti_gpio.c
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
#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>

#include "arm_internal.h"
#include "chip.h"
#include "ra6m5_gpio.h"
#include "ra6m5_exti.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct gpio_callback_s
{
  xcpt_t callback;   /* Callback entry point */
  void  *arg;        /* The argument that accompanies the callback */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Interrupt handlers attached to each EXTI */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Interrupt Service Routine - Dispatcher
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra6m5_gpiosetevent
 *
 * Description:
 *   Sets/clears GPIO based event and interrupt triggers.
 *
 * Description:
 *   Sets/clears GPIO based event and interrupt triggers.
 *
 * Input Parameters:
 *  pinset      - GPIO pin configuration
 *  risingedge  - Enables interrupt on rising edges
 *  fallingedge - Enables interrupt on falling edges
 *  event       - Generate event when set
 *  func        - When non-NULL, generate interrupt
 *  arg         - Argument passed to the interrupt callback
 *
 * Returned Value:
 *  Zero (OK) is returned on success, otherwise a negated errno value is
 *  returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int ra6m5_gpiosetevent(uint32_t pinset, bool risingedge, bool fallingedge,
                         bool event, xcpt_t func, void *arg)
{
  return OK;
}
