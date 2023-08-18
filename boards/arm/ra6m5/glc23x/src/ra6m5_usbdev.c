/****************************************************************************
 * boards/arm/stm32f7/nucleo-144/src/ra6m5_reset.c
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

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/usb/usbdev.h>

#include "arm_internal.h"
#include "chip.h"

#include "glc23x.h"
#include <arch/board/board.h>

#ifdef CONFIG_RA6M5_USBFS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra6m5_usbinitialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins.
 *
 ****************************************************************************/

void ra6m5_usbinitialize(void)
{
  uinfo("called\n");

  /* USB VBUS pin configuration */

#ifdef GPIO_OTGFS_VBUS
  ra6m5_configgpio(GPIO_OTGFS_VBUS);
#endif
}

/****************************************************************************
 * Name:  ra6m5_usbsuspend
 *
 * Description:
 *   Board logic must provide the ra6m5_usbsuspend logic if the USBDEV
 *   driver is used.  This function is called whenever the USB enters or
 *   leaves suspend mode.
 *   This is an opportunity for the board logic to shutdown clocks, power,
 *   etc. while the USB is suspended.
 *
 ****************************************************************************/

void ra6m5_usbsuspend(struct usbdev_s *dev, bool resume)
{
  uinfo("resume: %d\n", resume);
}

#endif /* CONFIG_RA6M5_USBFS */
