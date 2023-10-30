/****************************************************************************
 * arch/risc-v/src/sty32c2/sty32c2_rtc.c
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

#include <sys/types.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/timers/rtc.h>

#include "riscv_internal.h"
#include "chip.h"
#include "sty32c2_rtc.h"


/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_RTC_DRIVER
volatile bool g_rtc_enabled = false;
#endif

/****************************************************************************
 * Name: sty32c2_set_rtc_timer
 *
 * Description:
 *   HBN set RTC timer configuration
 *
 * Input Parameters:
 *   delay: RTC interrupt delay 32 clocks
 *   compval_low: RTC interrupt commpare value low 32 bits
 *   compval_high: RTC interrupt commpare value high 32 bits
 *   comp_mode: RTC interrupt commpare
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void sty32c2_set_rtc_timer(uint8_t delay_type, uint32_t compval_low,
                             uint32_t compval_high,  uint8_t comp_mode)
{
}

/****************************************************************************
 * Name: sty32c2_clear_rtc_counter
 *
 * Description:
 *   HBN set RTC timer configuration
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void sty32c2_clear_rtc_counter(void)
{
}

/****************************************************************************
 * Name: sty32c2_enable_rtc_counter
 *
 * Description:
 *   HBN clear RTC timer counter
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void sty32c2_enable_rtc_counter(void)
{
}

/****************************************************************************
 * Name: sty32c2_get_rtc_timer_val
 *
 * Description:
 *   HBN get RTC timer count value
 *
 * Input Parameters:
 *   val_low: RTC count value pointer for low 32 bits
 *   val_high: RTC count value pointer for high 8 bits
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void sty32c2_get_rtc_timer_val(uint32_t *val_low, uint32_t *val_high)
{
}

#ifdef CONFIG_RTC_DRIVER

/****************************************************************************
 * Name: up_rtc_time
 *
 * Description:
 *   Get the current time in seconds.  This is similar to the standard time()
 *   function.  This interface is only required if the low-resolution
 *   RTC/counter hardware implementation is selected.  It is only used by the
 *   RTOS during initialization to set up the system time when CONFIG_RTC is
 *   set but CONFIG_RTC_HIRES is not set.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current time in seconds
 *
 ****************************************************************************/

#ifndef CONFIG_RTC_HIRES
time_t up_rtc_time(void)
{
  uint64_t time_us;
  irqstate_t flags;

  flags = spin_lock_irqsave(NULL);

  /* NOTE: RT-Timer starts to work after the board is initialized, and the
   * RTC controller starts works after up_rtc_initialize is initialized.
   * Since the system clock starts to work before the board is initialized,
   * if CONFIG_RTC is enabled, the system time must be matched by the time
   * of the RTC controller (up_rtc_initialize has already been initialized,
   * and RT-Timer cannot work).
   */

  /* Determine if RT-Timer is started */

  if (g_rt_timer_enabled == true)
    {
      /* Get the time from RT-Timer, the time interval between RTC
       * controller and RT-Timer is stored in g_rtc_save->offset.
       */

      time_us = rt_timer_time_us() + g_rtc_save->offset +
                              esp32c3_rtc_get_boot_time();
    }
  else
    {
      /* Get the time from RTC controller. */

      time_us = esp32c3_rtc_get_time_us() +
                  esp32c3_rtc_get_boot_time();
    }

  spin_unlock_irqrestore(NULL, flags);

  return (time_t)(time_us / USEC_PER_SEC);
}
#endif /* !CONFIG_RTC_HIRES */

/****************************************************************************
 * Name: up_rtc_settime
 *
 * Description:
 *   Set the RTC to the provided time. All RTC implementations must be
 *   able to set their time based on a standard timespec.
 *
 * Input Parameters:
 *   ts - the time to use
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_settime(const struct timespec *ts)
{
  irqstate_t flags;
  uint64_t now_us;
  uint64_t rtc_offset_us;

  DEBUGASSERT(ts != NULL && ts->tv_nsec < NSEC_PER_SEC);
  flags = spin_lock_irqsave(NULL);

  now_us = ((uint64_t) ts->tv_sec) * USEC_PER_SEC +
          ts->tv_nsec / NSEC_PER_USEC;
  if (g_rt_timer_enabled == true)
    {
      /* Set based on RT-Timer offset value. */

      rtc_offset_us = now_us - rt_timer_time_us();
    }
  else
    {
      /* Set based on the offset value of the RT controller. */

      rtc_offset_us = now_us - esp32c3_rtc_get_time_us();
    }

  g_rtc_save->offset = 0;
  esp32c3_rtc_set_boot_time(rtc_offset_us);

  spin_unlock_irqrestore(NULL, flags);

  return OK;
}

/****************************************************************************
 * Name: up_rtc_initialize
 *
 * Description:
 *   Initialize the hardware RTC per the selected configuration.
 *   This function is called once during the OS initialization sequence
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_initialize(void)
{
  g_rtc_save = &rtc_saved_data;

  /* If saved data is invalid, clear offset information */

  if (g_rtc_save->magic != MAGIC_RTC_SAVE)
    {
      g_rtc_save->magic = MAGIC_RTC_SAVE;
      g_rtc_save->offset = 0;
      esp32c3_rtc_set_boot_time(0);
    }

#ifdef CONFIG_RTC_HIRES
  /* Synchronize the base time to the RTC time */

  up_rtc_gettime(&g_basetime);
#endif

  g_rtc_enabled = true;

  return OK;
}

/****************************************************************************
 * Name: up_rtc_gettime
 *
 * Description:
 *   Get the current time from the high resolution RTC time or RT-Timer. This
 *   interface is only supported by the high-resolution RTC/counter hardware
 *   implementation. It is used to replace the system timer.
 *
 * Input Parameters:
 *   tp - The location to return the RTC time or RT-Timer value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_HIRES
int up_rtc_gettime(struct timespec *tp)
{
  irqstate_t flags;
  uint64_t time_us;

  flags = spin_lock_irqsave(NULL);

  if (g_rt_timer_enabled == true)
    {
      time_us = rt_timer_time_us() + g_rtc_save->offset +
                              esp32c3_rtc_get_boot_time();
    }
  else
    {
      time_us = esp32c3_rtc_get_time_us() + esp32c3_rtc_get_boot_time();
    }

  tp->tv_sec  = time_us / USEC_PER_SEC;
  tp->tv_nsec = (time_us % USEC_PER_SEC) * NSEC_PER_USEC;

  spin_unlock_irqrestore(NULL, flags);

  return OK;
}
#endif /* CONFIG_RTC_HIRES */

#ifdef CONFIG_RTC_ALARM

/****************************************************************************
 * Name: up_rtc_setalarm
 *
 * Description:
 *   Set up an alarm.
 *
 * Input Parameters:
 *   alminfo - Information about the alarm configuration.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_setalarm(struct alm_setalarm_s *alminfo)
{
  struct rt_timer_args_s rt_timer_args;
  struct alm_cbinfo_s *cbinfo;
  irqstate_t flags;
  int ret = -EBUSY;
  int id;

  DEBUGASSERT(alminfo != NULL);
  DEBUGASSERT((RTC_ALARM0 <= alminfo->as_id) &&
              (alminfo->as_id < RTC_ALARM_LAST));

  /* Set the alarm in RT-Timer */

  id = alminfo->as_id;
  cbinfo = &g_alarmcb[id];

  if (cbinfo->ac_cb == NULL)
    {
      /* Create the RT-Timer alarm */

      flags = spin_lock_irqsave(NULL);

      if (cbinfo->alarm_hdl == NULL)
        {
          cbinfo->index = id;
          rt_timer_args.arg = cbinfo;
          rt_timer_args.callback = esp32c3_rt_cb_handler;
          ret = rt_timer_create(&rt_timer_args, &cbinfo->alarm_hdl);
          if (ret < 0)
            {
              rtcerr("ERROR: Failed to create rt_timer error=%d\n", ret);
              spin_unlock_irqrestore(NULL, flags);
              return ret;
            }
        }

      cbinfo->ac_cb  = alminfo->as_cb;
      cbinfo->ac_arg = alminfo->as_arg;
      cbinfo->deadline_us = alminfo->as_time.tv_sec * USEC_PER_SEC +
                            alminfo->as_time.tv_nsec / NSEC_PER_USEC;

      if (cbinfo->alarm_hdl == NULL)
        {
          rtcerr("ERROR: failed to create alarm timer\n");
        }
      else
        {
          rtcinfo("Start RTC alarm.\n");
          rt_timer_start(cbinfo->alarm_hdl, cbinfo->deadline_us, false);
          ret = OK;
        }

      spin_unlock_irqrestore(NULL, flags);
    }

  return ret;
}

/****************************************************************************
 * Name: up_rtc_cancelalarm
 *
 * Description:
 *   Cancel an alarm.
 *
 * Input Parameters:
 *   alarmid - Identifies the alarm to be cancelled
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_cancelalarm(enum alm_id_e alarmid)
{
  struct alm_cbinfo_s *cbinfo;
  irqstate_t flags;
  int ret = -ENODATA;

  DEBUGASSERT((RTC_ALARM0 <= alarmid) &&
              (alarmid < RTC_ALARM_LAST));

  /* Set the alarm in hardware and enable interrupts */

  cbinfo = &g_alarmcb[alarmid];

  if (cbinfo->ac_cb != NULL)
    {
      flags = spin_lock_irqsave(NULL);

      /* Stop and delete the alarm */

      rtcinfo("Cancel RTC alarm.\n");
      rt_timer_stop(cbinfo->alarm_hdl);
      rt_timer_delete(cbinfo->alarm_hdl);
      cbinfo->ac_cb = NULL;
      cbinfo->deadline_us = 0;
      cbinfo->alarm_hdl = NULL;

      spin_unlock_irqrestore(NULL, flags);

      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: up_rtc_rdalarm
 *
 * Description:
 *   Query an alarm configured in hardware.
 *
 * Input Parameters:
 *   tp      - Location to return the timer match register.
 *   alarmid - Identifies the alarm to get.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_rdalarm(struct timespec *tp, uint32_t alarmid)
{
  irqstate_t flags;
  struct alm_cbinfo_s *cbinfo;
  DEBUGASSERT(tp != NULL);
  DEBUGASSERT((RTC_ALARM0 <= alarmid) &&
              (alarmid < RTC_ALARM_LAST));

  flags = spin_lock_irqsave(NULL);

  /* Get the alarm according to the alarmid */

  cbinfo = &g_alarmcb[alarmid];

  tp->tv_sec = (rt_timer_time_us() + g_rtc_save->offset +
              cbinfo->deadline_us) / USEC_PER_SEC;
  tp->tv_nsec = ((rt_timer_time_us() + g_rtc_save->offset +
              cbinfo->deadline_us) % USEC_PER_SEC) * NSEC_PER_USEC;

  spin_unlock_irqrestore(NULL, flags);

  return OK;
}

#endif /* CONFIG_RTC_ALARM */

/****************************************************************************
 * Name: up_rtc_timer_init
 *
 * Description:
 *   Init RTC timer.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_timer_init(void)
{
  /* RT-Timer enabled */

  g_rt_timer_enabled = true;

  /* Get the time difference between rt_timer and RTC timer */

  g_rtc_save->offset = esp32c3_rtc_get_time_us() - rt_timer_time_us();

  return OK;
}

#endif /* CONFIG_RTC_DRIVER */
