/****************************************************************************
 * boards/risc-v/sty32c2/ulx3s/src/ulx3s_pwm.c
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

#include <errno.h>
#include <debug.h>
#include <stddef.h>
#include <stdio.h>

#include <nuttx/timers/pwm.h>
#include <arch/board/board.h>

#include "ulx3s_pwm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ulx3s_pwm_setup
 *
 * Description:
 *   Initialise all PWM channels enabled in gateware and map them to
 *   /dev/pwmX. Where X is the PMW channel number. From 0 ... STY32C2_PWM_MAX.
 *
 * Returned Value:
 *   OK is returned on success.
 *   -ENODEV is return on the first PWM device initialise failure.
 *
 ****************************************************************************/

int ulx3s_pwm_setup(void)
{
  struct pwm_lowerhalf_s *pwm = NULL;
  int ret = OK;
  int channel;
  char devpath[12] =
    {
        0
    };

  for (channel = 0; channel < STY32C2_PWM_MAX; channel++)
    {
      pwm = ulx3s_pwminitialize(channel);
      if (!pwm)
        {
          pwmerr("Failed fetching PWM channel %d lower half\n", channel);
          return -ENODEV;
        }

      /* Register the PWM driver at "/dev/pwmX" */

      snprintf(devpath, 12, "/dev/pwm%d", channel);
      ret = pwm_register(devpath, pwm);
      if (ret < 0)
        {
          pwmerr("pwm_register channel %d failed: %d\n", channel, ret);
          return ret;
        }
    }

  return ret;
}
