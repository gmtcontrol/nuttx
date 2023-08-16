/****************************************************************************
 * arch/ARM/src/ra6m5/ra6m5_rtc.c
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

#include <time.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/wqueue.h>
#include <nuttx/compiler.h>
#include <arch/board/board.h>
#include <ra6m5_rtc.h>

#include "nuttx/compiler.h"

#ifdef CONFIG_RA6M5_RTC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifdef CONFIG_RTC_HIRES
#  ifndef CONFIG_RTC_FREQUENCY
#    error "CONFIG_RTC_FREQUENCY is required for CONFIG_RTC_HIRES"
#  endif
#else
#  ifndef CONFIG_RTC_FREQUENCY
#    define CONFIG_RTC_FREQUENCY 1
#  endif
#  if CONFIG_RTC_FREQUENCY != 1
#    error "Only lo-res CONFIG_RTC_FREQUENCY of 1Hz is supported"
#  endif
#  endif

/* Constant values used in RTC */

#define RTC_WAIT_PERIOD               184
#define RTC_DUMMY_READ                (3)
#define RTC_RADJ_INITVALUE            (0x0)

/****************************************************************************
 * Private Macros
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

void up_enable_irq(int irq);
void up_disable_irq(int irq);
void up_clear_irq(int irq);
bool up_status_irq(int irq);

static uint32_t rtc_dec2bcd(uint8_t value);
#if defined (CONFIG_RTC_HIRES) || defined (CONFIG_RTC_ALARM) || defined (CONFIG_RTC_DATETIME)
static int rtc_bcd2dec(uint32_t value);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Callback to use when the alarm expires */

#ifdef CONFIG_RTC_ALARM
struct alm_cbinfo_s
{
  volatile alm_callback_t ac_cb; /* Client callback function */
  volatile void *ac_arg;         /* Argument to pass with the callback function */
};
#endif

/* Callback to use when the periodic interrupt expires */

#ifdef CONFIG_RTC_PERIODIC
struct prd_cbinfo_s
{
  volatile periodiccb_t prd_cb; /* Client callback function */
  volatile void *prd_arg;       /* Argument to pass with the callback function */
};
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static struct alm_cbinfo_s g_alarmcb;
#endif

#ifdef CONFIG_RTC_PERIODIC
static struct prd_cbinfo_s g_periodiccb;
#endif

/* Callback to use when the cary interrupt expires */

#ifdef CONFIG_RA6M5_RTC_CARRY
static carrycb_t g_carrycb;
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_rtc_enabled is set true after the RTC has successfully initialized */

volatile bool g_rtc_enabled = false;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra6m5_rtc_getreg
 *
 * Description:
 *   Get a 8-bit register value by offset
 *
 ****************************************************************************/

static inline uint8_t ra6m5_rtc_getreg(uint32_t offset)
{
  return getreg8(RA6M5_RTC_BASE + offset);
}

/****************************************************************************
 * Name: ra6m5_rtc_putreg
 *
 * Description:
 *  Put a 8-bit register value by offset
 *
 ****************************************************************************/

static inline void ra6m5_rtc_putreg(uint32_t offset, uint8_t value)
{
  putreg8(value, RA6M5_RTC_BASE + offset);
}

/****************************************************************************
 * Name: ra6m5_rtc_getreg16
 *
 * Description:
 *   Get a 16-bit register value by offset
 *
 ****************************************************************************/

static inline uint16_t ra6m5_rtc_getreg16(uint32_t offset)
{
  return getreg16(RA6M5_RTC_BASE + offset);
}

/****************************************************************************
 * Name: ra6m5_rtc_putreg16
 *
 * Description:
 *  Put a 16-bit register value by offset
 *
 ****************************************************************************/

static inline void ra6m5_rtc_putreg16(uint32_t offset, uint16_t value)
{
  putreg16(value, RA6M5_RTC_BASE + offset);
}

/****************************************************************************
 * Name: rtc_dumpregs
 *
 * Description:
 *    Disable RTC write protection
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_RTC_INFO
static void rtc_dumpregs(const char *msg)
{
  rtcinfo("%s:\n", msg);
  rtcinfo("  64-Hz Counter: %08x\n", ra6m5_rtc_getreg(RA6M5_RTC_R64CNT));
  rtcinfo("  Second Counter: %08x\n", ra6m5_rtc_getreg(RA6M5_RTC_RSECCNT_OFFSET));
  rtcinfo("  Minute Counter: %08x\n", ra6m5_rtc_getreg(RA6M5_RTC_RMINCNT_OFFSET));
  rtcinfo("  Hour Counter: %08x\n", ra6m5_rtc_getreg(RA6M5_RTC_RHRCNT_OFFSET));
  rtcinfo("  Day-of-Week Counter: %08x\n", ra6m5_rtc_getreg(RA6M5_RTC_RWKCNT_OFFSET));
  rtcinfo("  Date Counter: %08x\n", ra6m5_rtc_getreg(RA6M5_RTC_RDAYCNT_OFFSET));
  rtcinfo("  Month Counter: %08x\n", ra6m5_rtc_getreg(RA6M5_RTC_RMONCNT_OFFSET));
  rtcinfo("  Year Counter: %08x\n", ra6m5_rtc_getreg(RA6M5_RTC_RYRCNT_OFFSET));
  rtcinfo(" Second Alarm Register: %08x\n", ra6m5_rtc_getreg(RA6M5_RTC_RSECAR_OFFSET));
  rtcinfo(" Minute Alarm Register: %08x\n", ra6m5_rtc_getreg(RA6M5_RTC_RMINAR_OFFSET));
  rtcinfo(" Hour Alarm Register: %08x\n", ra6m5_rtc_getreg(RA6M5_RTC_RHRAR_OFFSET));
  rtcinfo(" Day-of-Week Alarm Register: %08x\n", ra6m5_rtc_getreg(RA6M5_RTC_RWKAR_OFFSET));
  rtcinfo(" Date Alarm Register: %08x\n", ra6m5_rtc_getreg(RA6M5_RTC_RDAYAR_OFFSET));
  rtcinfo(" Month Alarm Register: %08x\n", ra6m5_rtc_getreg(RA6M5_RTC_RMONAR_OFFSET));
  rtcinfo(" Year Alarm Register: %08x\n", ra6m5_rtc_getreg(RA6M5_RTC_RYRAR_OFFSET));
  rtcinfo(" Year Alarm Enable Register: %08x\n", ra6m5_rtc_getreg(RA6M5_RTC_RYRAREN_OFFSET));
  rtcinfo(" RTC Control Register 1: %08x\n", ra6m5_rtc_getreg(RA6M5_RTC_RCR1_OFFSET));
  rtcinfo(" RTC Control Register 2: %08x\n", ra6m5_rtc_getreg(RA6M5_RTC_RCR2_OFFSET));
  rtcinfo(" RTC Control Register 4: %08x\n", ra6m5_rtc_getreg(RA6M5_RTC_RCR4_OFFSET));
}
#else
#  define rtc_dumpregs(msg)
#endif

/****************************************************************************
 * Name: rtc_dumptime
 *
 * Description:
 *    Disable RTC write protection
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_RTC_INFO
static void rtc_dumptime(struct tm *tp, const char *msg)
{
  rtcinfo("%s:\n", msg);
  rtcinfo("  tm_sec: %08x\n", tp->tm_sec);
  rtcinfo("  tm_min: %08x\n", tp->tm_min);
  rtcinfo(" tm_hour: %08x\n", tp->tm_hour);
  rtcinfo(" tm_mday: %08x\n", tp->tm_mday);
  rtcinfo("  tm_mon: %08x\n", tp->tm_mon);
  rtcinfo(" tm_year: %08x\n", tp->tm_year);
}
#else
#  define rtc_dumptime(tp, msg)
#endif

/****************************************************************************
 * Name: rtc_dec2bcd
 *
 * Description:
 *   Converts decimal value to BCD format
 *
 * Input Parameters:
 *   value - The byte to be converted.
 *
 * Returned Value:
 *   The value in BCD representation
 *
 ****************************************************************************/

static uint32_t rtc_dec2bcd(uint8_t value)
{
  return (uint8_t) ((((value / 10) << 4) & 0xf0) | (value % 10));
}

/****************************************************************************
 * Name: rtc_bcd2dec
 *
 * Description:
 *   Convert from 2 digit BCD to decimal.
 *
 * Input Parameters:
 *   value - The BCD value to be converted.
 *
 * Returned Value:
 *   The value in binary representation
 *
 ****************************************************************************/

#if defined (CONFIG_RTC_HIRES) || defined (CONFIG_RTC_ALARM) || defined (CONFIG_RTC_DATETIME)
static int rtc_bcd2dec(uint32_t value)
{
  return (int) ((((value & 0xf0) >> 4) * 10) + (value & 0x0f));
}
#endif

/****************************************************************************
 * Name: rtc_interrupt
 *
 * Description:
 *    RTC interrupt service routine
 *
 * Input Parameters:
 *   irq - The IRQ number that generated the interrupt
 *   context - Architecture specific register save information.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static int rtc_alm_interrupt(int irq, void *context, void *arg)
{
  struct alm_cbinfo_s *cbinfo;
  alm_callback_t cb;
  uint8_t source = ra6m5_rtc_getreg(RA6M5_RTC_RCR1_OFFSET);
  if ((source & RTC_RCR1_AIE) != 0)
    {
      /* Alarm callback */

      cbinfo = &g_alarmcb;
      cb = cbinfo->ac_cb;
      arg = (void *)cbinfo->ac_arg;
      cbinfo->ac_cb  = NULL;
      cbinfo->ac_arg = NULL;
      cb(arg, 0);
    }

  up_disable_irq(RA6M5_IRQ_RTC_ALARM);
  return 0;
}
#endif

#ifdef CONFIG_RTC_PERIODIC
static int rtc_periodic_interrupt(int irq, void *context, void *arg)
{
  struct prd_cbinfo_s *cbinfo;
  periodiccb_t cb;
  uint8_t source = ra6m5_rtc_getreg(RA6M5_RTC_RCR1_OFFSET);
  if ((source & RTC_RCR1_PIE) != 0)
    {
      /* Periodic callback */

      cbinfo = &g_periodiccb;
      cb = cbinfo->prd_cb;
      arg = (void *)cbinfo->prd_arg;
      cb(arg, 0);
    }

  return 0;
}
#endif

#ifdef CONFIG_RA6M5_RTC_CARRY
static int rtc_carry_interrupt(int irq, void *context, void *arg)
{
  uint8_t source = ra6m5_rtc_getreg(RA6M5_RTC_RCR1_OFFSET);
  if ((source & RTC_RCR1_CIE) != 0)
    {
      /* Carry callback */

      g_carrycb();
    }

  return 0;
}

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_rtc_initialize
 *
 * Description:
 *   Initialize the hardware RTC per the selected configuration.
 *   This function is
 *   called once during the OS initialization sequence
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
  uint8_t regval;
  uint32_t temp_byte;
  rtc_dumpregs("On reset");

  /* Disable alarm, periodic and carry interrupts */

  up_disable_irq(RA6M5_IRQ_RTC_ALARM);
  up_disable_irq(RA6M5_IRQ_RTC_PERIOD);
  up_disable_irq(RA6M5_IRQ_RTC_CARRY);

  /* Set RTC clock source as sub clock */

  regval = ra6m5_rtc_getreg(RA6M5_RTC_RCR4_OFFSET);
  regval &= ~RTC_RCR4_RCKSEL;
  ra6m5_rtc_putreg(RA6M5_RTC_RCR4_OFFSET, regval);

  /* Stop all counters */

  ra6m5_rtc_putreg(RA6M5_RTC_RCR2_OFFSET, 0);
  while ((regval = ra6m5_rtc_getreg(RA6M5_RTC_RCR2_OFFSET)) & RTC_RCR2_START)
    {
      /* Ensure the clock is stopped while configuring it. */
    }

  /* Select count mode */

  regval &= ~RTC_RCR2_CNTMD;
  ra6m5_rtc_putreg(RA6M5_RTC_RCR2_OFFSET, regval);

  while ((regval = ra6m5_rtc_getreg(RA6M5_RTC_RCR2_OFFSET)) & RTC_RCR2_CNTMD)
    {
      /* Wait for the calendar count mode complete setting */
    }

  /* Execute RTC software reset */

  regval |= RTC_RCR2_RESET;
  ra6m5_rtc_putreg(RA6M5_RTC_RCR2_OFFSET, regval);

  while ((regval = ra6m5_rtc_getreg(RA6M5_RTC_RCR2_OFFSET)) & RTC_RCR2_RESET)
    {
      /* Wait for the reset to complete */
    }

  /* Stop RTC counter */

  regval &= ~(RTC_RCR2_START);
  ra6m5_rtc_putreg(RA6M5_RTC_RCR2_OFFSET, regval);

  while (ra6m5_rtc_getreg(RA6M5_RTC_RCR2_OFFSET) & RTC_RCR2_START)
    {
      /* Ensure the clock is stopped while configuring it. */
    }

  /* Clear ALM, PRD, CUP IRQ */

  up_clear_irq(RA6M5_IRQ_RTC_ALARM);
  up_clear_irq(RA6M5_IRQ_RTC_PERIOD);
  up_clear_irq(RA6M5_IRQ_RTC_CARRY);

  /* After a reset is generated, write to the RTC register
   * when six cycles of the count source have elapsed
   */

  up_udelay(RTC_WAIT_PERIOD);

  /* Start the counter and set 24hr mode */

  regval = ra6m5_rtc_getreg(RA6M5_RTC_RCR2_OFFSET);
  regval |= (RTC_RCR2_HR24);
  ra6m5_rtc_putreg(RA6M5_RTC_RCR2_OFFSET, regval);

  /* Setting RADJ register */

  regval = ra6m5_rtc_getreg(RA6M5_RTC_RADJ_OFFSET);
  regval |= (RTC_RADJ_INITVALUE);
  ra6m5_rtc_putreg(RA6M5_RTC_RADJ_OFFSET, regval);

  /* Setting AADJE and AADJP register */

  regval = ra6m5_rtc_getreg(RA6M5_RTC_RCR2_OFFSET);
  regval |= (RTC_RCR2_AADJE | RTC_RCR2_AADJP);
  ra6m5_rtc_putreg(RA6M5_RTC_RCR2_OFFSET, regval);
  g_rtc_enabled = true;
  UNUSED(temp_byte);
  return OK;
}

/****************************************************************************
 * Name: up_rtc_gettime
 *
 * Description:
 *   Get the current date and time from the date/time RTC.
 * Input Parameters:
 *   tp - The location to return the high resolution time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#if defined(CONFIG_RTC_HIRES)
int up_rtc_gettime(struct timespec *tp)
{
  uint8_t weekcnt;
  uint8_t daycnt;
  uint8_t monthcnt;
  uint8_t yearcnt;
  uint8_t seccnt;
  uint8_t mincnt;
  uint8_t hrcnt;
  uint8_t tmp_week;
  uint8_t tmp_day;
  uint8_t tmp_month;
  uint8_t tmp_year;
  uint16_t bcd_years;
  uint8_t regval;
  struct tm t;

  regval = ra6m5_rtc_getreg(RA6M5_RTC_RCR2_OFFSET);
  if (!(regval & RTC_RCR2_START))
    {
      regval |= RTC_RCR2_START;
      ra6m5_rtc_putreg(RA6M5_RTC_RCR2_OFFSET, regval);
    }

  do
    {
      weekcnt  = ra6m5_rtc_getreg(RA6M5_RTC_RWKCNT_OFFSET);
      daycnt   = ra6m5_rtc_getreg(RA6M5_RTC_RDAYCNT_OFFSET);
      monthcnt = ra6m5_rtc_getreg(RA6M5_RTC_RMONCNT_OFFSET);
      yearcnt  = ra6m5_rtc_getreg(RA6M5_RTC_RYRCNT_OFFSET);
      seccnt   = ra6m5_rtc_getreg(RA6M5_RTC_RSECCNT_OFFSET);
      mincnt   = ra6m5_rtc_getreg(RA6M5_RTC_RMINCNT_OFFSET);
      hrcnt    = ra6m5_rtc_getreg(RA6M5_RTC_RHRCNT_OFFSET);
      tmp_week = ra6m5_rtc_getreg(RA6M5_RTC_RWKCNT_OFFSET);
      tmp_day  = ra6m5_rtc_getreg(RA6M5_RTC_RDAYCNT_OFFSET);
      tmp_month = ra6m5_rtc_getreg(RA6M5_RTC_RMONCNT_OFFSET);
      tmp_year  = ra6m5_rtc_getreg(RA6M5_RTC_RYRCNT_OFFSET);
    }

  while (tmp_week != weekcnt && tmp_day != daycnt &&
         tmp_month != monthcnt && tmp_year != yearcnt);

  /* Disable Carry interrupt */

  up_disable_irq(RA6M5_IRQ_RTC_CARRY);

  /* Enable RTC carry interrupt */

  regval = ra6m5_rtc_getreg(RA6M5_RTC_RCR1_OFFSET);
  regval |= (RTC_RCR1_CIE);
  ra6m5_rtc_putreg(RA6M5_RTC_RCR1_OFFSET, regval);

  do
    {
      /* Clear carry flag in ICU */

      up_clear_irq(RA6M5_IRQ_RTC_CARRY);

      /* Read and convert RTC registers;
       * mask off unknown bits and hour am/pm.
       */

      /* Seconds. (0-59) */

      t.tm_sec  = rtc_bcd2dec((uint8_t) (ra6m5_rtc_getreg(RA6M5_RTC_RSECCNT_OFFSET) & 0x7fu));
      t.tm_min  = rtc_bcd2dec((uint8_t) (ra6m5_rtc_getreg(RA6M5_RTC_RMINCNT_OFFSET) & 0x7fu));
      t.tm_hour = rtc_bcd2dec((uint8_t) (ra6m5_rtc_getreg(RA6M5_RTC_RHRCNT_OFFSET) & 0x3fu));
      t.tm_mday = rtc_bcd2dec(ra6m5_rtc_getreg(RA6M5_RTC_RDAYCNT_OFFSET));
      t.tm_mon  = rtc_bcd2dec(ra6m5_rtc_getreg(RA6M5_RTC_RMONCNT_OFFSET)) - 1;

      /* Years since 2000 */

      bcd_years = (uint16_t) ra6m5_rtc_getreg16(RA6M5_RTC_RYRCNT_OFFSET);

      t.tm_year = rtc_bcd2dec((uint8_t) (bcd_years & 0xff)) + 100;

      tp->tv_sec = timegm(&t);
      tp->tv_nsec = 0;
    }

  while (up_status_irq(RA6M5_IRQ_RTC_CARRY));

  UNUSED(hrcnt);
  UNUSED(mincnt);
  UNUSED(seccnt);
  return OK;
}
#endif

int ra6m5_rtc_setdatetime(const struct tm *tp)
{
  volatile uint8_t dummy_byte;
  volatile uint16_t dummy_word;
  uint8_t regval;
  int i;

  /* Break out the time values (note that the time is set only to units of
   * seconds)
   */

  /* gmtime_r(&tp->tv_sec, &tp); */

  rtc_dumptime(&tp, "Setting time");

  /* Then write the broken out values to the RTC */

  /* Convert the struct tm format to RTC time register fields.
   *
   *   struct tm       TIMR register
   *   tm_sec    0-61* SEC    (0-59)
   *   tm_min    0-59  MIN    (0-59)
   *   tm_hour   0-23  HOUR   (0-23)
   *
   *  *To allow for leap seconds.  But these never actuall happen.
   */

  /* Stop all counters */

  regval = ra6m5_rtc_getreg(RA6M5_RTC_RCR2_OFFSET);
  regval &= ~(RTC_RCR2_START);
  ra6m5_rtc_putreg(RA6M5_RTC_RCR2_OFFSET, regval);

  while ((regval = ra6m5_rtc_getreg(RA6M5_RTC_RCR2_OFFSET)) & RTC_RCR2_START)
    {
      /* Ensure the clock is stopped while configuring it. */
    }

  /* Execute RTC software reset */

  regval |= RTC_RCR2_RESET;
  ra6m5_rtc_putreg(RA6M5_RTC_RCR2_OFFSET, regval);

  while ((regval = ra6m5_rtc_getreg(RA6M5_RTC_RCR2_OFFSET)) & RTC_RCR2_RESET)
    {
      /* Wait for the reset to complete */
    }

  regval |= RTC_RCR2_HR24;
  ra6m5_rtc_putreg(RA6M5_RTC_RCR2_OFFSET, regval);

  /* Set time */

  /* Set seconds. (0-59) */

  ra6m5_rtc_putreg(RA6M5_RTC_RSECCNT_OFFSET, rtc_dec2bcd((uint8_t)tp->tm_sec));

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
      dummy_byte = ra6m5_rtc_getreg(RA6M5_RTC_RSECCNT_OFFSET);
    }

  /* Set minutes (0-59) */

  ra6m5_rtc_putreg(RA6M5_RTC_RMINCNT_OFFSET, rtc_dec2bcd((uint8_t) tp->tm_min));

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
      dummy_byte = ra6m5_rtc_getreg(RA6M5_RTC_RMINCNT_OFFSET);
    }

  /* Set hours. (0-23) */

  ra6m5_rtc_putreg(RA6M5_RTC_RHRCNT_OFFSET, rtc_dec2bcd((uint8_t) tp->tm_hour));

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
      dummy_byte = ra6m5_rtc_getreg(RA6M5_RTC_RHRCNT_OFFSET);
    }

  /* Set the date */

  /* Day of the week (0-6, 0=Sunday) */

#if defined(CONFIG_LIBC_LOCALTIME) || defined(CONFIG_TIME_EXTENDED)
  ra6m5_rtc_putreg(RA6M5_RTC_RWKCNT_OFFSET, rtc_dec2bcd((uint8_t) tp->tm_wday));

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
      dummy_byte = ra6m5_rtc_getreg(RA6M5_RTC_RWKCNT_OFFSET);
    }
#endif

  /* Day of the month (1-31) */

  ra6m5_rtc_putreg(RA6M5_RTC_RDAYCNT_OFFSET, rtc_dec2bcd((uint8_t) tp->tm_mday));

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
      dummy_byte = ra6m5_rtc_getreg(RA6M5_RTC_RDAYCNT_OFFSET);
    }

  /* Month. (1-12, 1=January) */

  ra6m5_rtc_putreg(RA6M5_RTC_RMONCNT_OFFSET, rtc_dec2bcd((uint8_t) (tp->tm_mon + 1)));

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
      dummy_byte = ra6m5_rtc_getreg(RA6M5_RTC_RMONCNT_OFFSET);
    }

  /* Year. (00-99) */

  ra6m5_rtc_putreg16(RA6M5_RTC_RYRCNT_OFFSET, (uint16_t) (rtc_dec2bcd((uint8_t)((tp->tm_year + 1900) % 100))));

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
      dummy_word = ra6m5_rtc_getreg16(RA6M5_RTC_RYRCNT_OFFSET);
    }

  regval = ra6m5_rtc_getreg(RA6M5_RTC_RCR2_OFFSET);
  regval |= RTC_RCR2_START;
  ra6m5_rtc_putreg(RA6M5_RTC_RCR2_OFFSET, regval);

  rtc_dumpregs("New time setting");
  UNUSED(dummy_word);
  UNUSED(dummy_byte);
  return OK;
}

/****************************************************************************
 * Name: up_rtc_settime
 *
 * Description:
 *   Set the RTC to the provided time.  All RTC implementations
 *   must be able to
 *   set their time based on a standard timespec.
 *
 * Input Parameters:
 *   tp - the time to use
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_settime(const struct timespec *tp)
{
  struct tm newtime;
  int i;
  volatile uint8_t dummy_byte;
  volatile uint16_t dummy_word;
  uint8_t regval;

  /* Break out the time values (note that the time is set only to units of
   * seconds)
   */

  gmtime_r(&tp->tv_sec, &newtime);
  rtc_dumptime(&newtime, "Setting time");

  /* Then write the broken out values to the RTC */

  /* Convert the struct tm format to RTC time register fields.
   *
   *   struct tm       TIMR register
   *   tm_sec    0-61* SEC    (0-59)
   *   tm_min    0-59  MIN    (0-59)
   *   tm_hour   0-23  HOUR   (0-23)
   *
   *  *To allow for leap seconds.  But these never actuall happen.
   */

  /* Stop all counters */

  regval = ra6m5_rtc_getreg(RA6M5_RTC_RCR2_OFFSET);
  regval &= ~RTC_RCR2_START;
  ra6m5_rtc_putreg(RA6M5_RTC_RCR2_OFFSET, regval);

  while ((regval = ra6m5_rtc_getreg(RA6M5_RTC_RCR2_OFFSET)) & RTC_RCR2_START)
    {
      /* Ensure the clock is stopped while configuring it. */
    }

  /* Execute RTC software reset */

  regval |= RTC_RCR2_RESET;
  ra6m5_rtc_putreg(RA6M5_RTC_RCR2_OFFSET, regval);

  while ((regval = ra6m5_rtc_getreg(RA6M5_RTC_RCR2_OFFSET)) & RTC_RCR2_RESET)
    {
      /* Wait for the reset to complete */
    }

  regval |= RTC_RCR2_HR24;
  ra6m5_rtc_putreg(RA6M5_RTC_RCR2_OFFSET, regval);

  /* Set time */

  /* Set seconds. (0-59) */

  ra6m5_rtc_putreg(RA6M5_RTC_RSECCNT_OFFSET, rtc_dec2bcd((uint8_t)newtime.tm_sec));

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
      dummy_byte = ra6m5_rtc_getreg(RA6M5_RTC_RSECCNT_OFFSET);
    }

  /* Set minutes (0-59) */

  ra6m5_rtc_putreg(RA6M5_RTC_RMINCNT_OFFSET, rtc_dec2bcd((uint8_t) newtime.tm_min));

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
      dummy_byte = ra6m5_rtc_getreg(RA6M5_RTC_RMINCNT_OFFSET);
    }

  /* Set hours. (0-23) */

  ra6m5_rtc_putreg(RA6M5_RTC_RHRCNT_OFFSET, rtc_dec2bcd((uint8_t) newtime.tm_hour));

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
      dummy_byte = ra6m5_rtc_getreg(RA6M5_RTC_RHRCNT_OFFSET);
    }

  /* Set the date */

  /* Day of the week (0-6, 0=Sunday) */

  ra6m5_rtc_putreg(RA6M5_RTC_RWKCNT_OFFSET, rtc_dec2bcd((uint8_t) newtime.tm_wday));

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
      dummy_byte = ra6m5_rtc_getreg(RA6M5_RTC_RWKCNT_OFFSET);
    }

  /* Day of the month (1-31) */

  ra6m5_rtc_putreg(RA6M5_RTC_RDAYCNT_OFFSET, rtc_dec2bcd((uint8_t) newtime.tm_mday));

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
      dummy_byte = ra6m5_rtc_getreg(RA6M5_RTC_RDAYCNT_OFFSET);
    }

  /* Month. (1-12, 1=January) */

  ra6m5_rtc_putreg(RA6M5_RTC_RMONCNT_OFFSET, rtc_dec2bcd((uint8_t) (newtime.tm_mon + 1)));

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
      dummy_byte = ra6m5_rtc_getreg(RA6M5_RTC_RMONCNT_OFFSET);
    }

  /* Year. (00-99) */

  ra6m5_rtc_putreg16(RA6M5_RTC_RYRCNT_OFFSET, (uint16_t) (rtc_dec2bcd((uint8_t)((newtime.tm_year + 1900) % 100))));

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
      dummy_word = ra6m5_rtc_getreg16(RA6M5_RTC_RYRCNT_OFFSET);
    }

  regval = ra6m5_rtc_getreg(RA6M5_RTC_RCR2_OFFSET);
  regval |= RTC_RCR2_START;
  ra6m5_rtc_putreg(RA6M5_RTC_RCR2_OFFSET, regval);

  rtc_dumpregs("New time setting");
  UNUSED(dummy_word);
  UNUSED(dummy_byte);
  return OK;
}

/****************************************************************************
 * Name: ra6m5_rtc_getalarmdatetime
 *
 * Description:
 *   Get the current date and time for a RTC alarm.
 *
 * Input Parameters:
 *   reg - RTC alarm register
 *   tp - The location to return the high resolution time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static int ra6m5_rtc_getalarmdatetime(struct tm *tp)
{
  uint8_t bcd_years;
  DEBUGASSERT(tp != NULL);

  tp->tm_sec  = rtc_bcd2dec((uint8_t) (ra6m5_rtc_getreg(RA6M5_RTC_RSECAR_OFFSET) & 0x7fu));
  tp->tm_min  = rtc_bcd2dec((uint8_t) (ra6m5_rtc_getreg(RA6M5_RTC_RMINAR_OFFSET) & 0x7fu));
  tp->tm_hour = rtc_bcd2dec((uint8_t) (ra6m5_rtc_getreg(RA6M5_RTC_RHRAR_OFFSET) & 0x3fu));
  tp->tm_mday = rtc_bcd2dec(ra6m5_rtc_getreg(RA6M5_RTC_RDAYAR_OFFSET) & 0x3fu);
  tp->tm_mon  = rtc_bcd2dec(ra6m5_rtc_getreg(RA6M5_RTC_RMONAR_OFFSET) & 0x1fu) - 1;

  /* Years since 2000 */

  bcd_years = (uint8_t) ra6m5_rtc_getreg16(RA6M5_RTC_RYRAR_OFFSET);

  tp->tm_year = rtc_bcd2dec((uint8_t) (bcd_years & 0xff)) + 100;
  return 0;
}

#endif
/****************************************************************************
 * Name: ra6m5_rtc_rdalarm
 *
 * Description:
 *   Query an alarm configured in hardware.
 *
 * Input Parameters:
 *  alminfo - Information about the alarm configuration.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
int ra6m5_rtc_rdalarm(struct alm_rdalarm_s *alminfo)
{
  int ret = -EINVAL;
  DEBUGASSERT(alminfo != NULL);
  ret = ra6m5_rtc_getalarmdatetime((struct tm *)alminfo->ar_time);
  return ret;
}
#endif

/****************************************************************************
 * Name: ra6m5_rtc_setalarm
 *
 * Description:
 *   Set up an alarm.
 *
 * Input Parameters:
 *   tp - the time to set the alarm
 *   callback - the function to call when the alarm expires.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
int ra6m5_rtc_setalarm(struct alm_setalarm_s *alminfo)
{
  irqstate_t flags;
  uint8_t dummy_byte;
  uint8_t dummy_word;
  uint8_t i, regval;

  /* Is there already something waiting on the ALARM? */

  flags = enter_critical_section();

  /* Save the callback info */

  g_alarmcb.ac_cb  = alminfo->as_cb;
  g_alarmcb.ac_arg = alminfo->as_arg;

  /* Disable ICU alarm interrupt */

  up_disable_irq(RA6M5_IRQ_RTC_ALARM);

  /* Attach the Alarm Interrupt */

  irq_attach(RA6M5_IRQ_RTC_ALARM, rtc_alm_interrupt, NULL);

  /* Start RTC counter */

  regval = ra6m5_rtc_getreg(RA6M5_RTC_RCR2_OFFSET);
  regval |= RTC_RCR2_START;
  ra6m5_rtc_putreg(RA6M5_RTC_RCR2_OFFSET, regval);

  while (!(ra6m5_rtc_getreg(RA6M5_RTC_RCR2_OFFSET) & RTC_RCR2_START))
    {
      /* Wait for the register modification to complete */
    }

  /* Set time */

  /* Set seconds. (0-59) */

  ra6m5_rtc_putreg(RA6M5_RTC_RSECAR_OFFSET, 0x80u);

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
       dummy_byte = ra6m5_rtc_getreg(RA6M5_RTC_RSECAR_OFFSET);
    }

  regval = ra6m5_rtc_getreg(RA6M5_RTC_RSECAR_OFFSET);
  regval |= rtc_dec2bcd((uint8_t)alminfo->as_time.tm_sec);
  ra6m5_rtc_putreg(RA6M5_RTC_RSECAR_OFFSET, regval);

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
      dummy_byte = ra6m5_rtc_getreg(RA6M5_RTC_RSECAR_OFFSET);
    }

  /* Set minutes (0-59) */

  ra6m5_rtc_putreg(RA6M5_RTC_RMINAR_OFFSET, 0x80u);

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
       dummy_byte = ra6m5_rtc_getreg(RA6M5_RTC_RMINAR_OFFSET);
    }

  regval = ra6m5_rtc_getreg(RA6M5_RTC_RMINAR_OFFSET);
  regval |= rtc_dec2bcd((uint8_t) alminfo->as_time.tm_min);
  ra6m5_rtc_putreg(RA6M5_RTC_RMINAR_OFFSET, regval);

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
       dummy_byte = ra6m5_rtc_getreg(RA6M5_RTC_RMINAR_OFFSET);
    }

  /* Set hours. (0-23) */

  ra6m5_rtc_putreg(RA6M5_RTC_RHRAR_OFFSET, 0x80u);

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
      dummy_byte = ra6m5_rtc_getreg(RA6M5_RTC_RHRAR_OFFSET);
    }

  regval = ra6m5_rtc_getreg(RA6M5_RTC_RHRAR_OFFSET);
  regval |= rtc_dec2bcd((uint8_t) alminfo->as_time.tm_hour);
  ra6m5_rtc_putreg(RA6M5_RTC_RHRAR_OFFSET, regval);

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
      dummy_byte = ra6m5_rtc_getreg(RA6M5_RTC_RHRAR_OFFSET);
    }

  /* Day of the month (1-31) */

  ra6m5_rtc_putreg(RA6M5_RTC_RDAYAR_OFFSET, 0x80u);

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
       dummy_byte = ra6m5_rtc_getreg(RA6M5_RTC_RDAYAR_OFFSET);
    }

  regval = ra6m5_rtc_getreg(RA6M5_RTC_RDAYAR_OFFSET);
  regval |= rtc_dec2bcd((uint8_t) alminfo->as_time.tm_mday);
  ra6m5_rtc_putreg(RA6M5_RTC_RDAYAR_OFFSET, regval);

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
       dummy_byte = ra6m5_rtc_getreg(RA6M5_RTC_RDAYAR_OFFSET);
    }

  /* Month. (1-12, 1=January) */

  ra6m5_rtc_putreg(RA6M5_RTC_RMONAR_OFFSET, 0x80u);

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
      dummy_byte = ra6m5_rtc_getreg(RA6M5_RTC_RMONAR_OFFSET);
    }

  regval = ra6m5_rtc_getreg(RA6M5_RTC_RMONAR_OFFSET);
  regval |= rtc_dec2bcd((uint8_t) ((alminfo->as_time.tm_mon) + 1));
  ra6m5_rtc_putreg(RA6M5_RTC_RMONAR_OFFSET, regval);

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
      dummy_byte = ra6m5_rtc_getreg(RA6M5_RTC_RMONAR_OFFSET);
    }

  /* Year. (00-99) */

  //ra6m5_rtc_putreg(RA6M5_RTC_RYRAREN_OFFSET, 0x80u);

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
      dummy_byte = ra6m5_rtc_getreg(RA6M5_RTC_RYRAREN_OFFSET);
    }

  ra6m5_rtc_putreg16(RA6M5_RTC_RYRAR_OFFSET, 
                      (uint16_t) (rtc_dec2bcd((uint8_t)((alminfo->as_time.tm_year) + 1900))));

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
      dummy_word = ra6m5_rtc_getreg16(RA6M5_RTC_RYRAR_OFFSET);
    }

  rtc_dumpregs("New alarm setting");

  /* Enable RTC ALARM interrupt */

  regval = ra6m5_rtc_getreg(RA6M5_RTC_RCR1_OFFSET);
  regval |= RTC_RCR1_AIE;
  ra6m5_rtc_putreg(RA6M5_RTC_RCR1_OFFSET, regval);

  /* Clear IR flag of ICU ALARM interrupt */

  up_clear_irq(RA6M5_IRQ_RTC_ALARM);

  /* Enable ICU alarm interrupts */

  up_enable_irq(RA6M5_IRQ_RTC_ALARM);

  /* Set Priority of alarm interrupt */

  leave_critical_section(flags);
  UNUSED(dummy_byte);
  UNUSED(dummy_word);
  return OK;
}
#endif

#ifdef CONFIG_RTC_PERIODIC
int ra6m5_rtc_setperiodic(const struct timespec *period,
                          periodiccb_t callback)
{
  irqstate_t flags;
  volatile uint8_t regval;
  uint8_t prd;
  flags = enter_critical_section();

  /* No.. Save the callback function pointer */

  g_periodiccb.prd_cb = callback;
  prd = period->tv_sec;

  /* Disable ICU periodic interrupt */

  up_disable_irq(RA6M5_IRQ_RTC_PERIOD);

  /* Clear IRQ flag of periodic interrupt */

  up_clear_irq(RA6M5_IRQ_RTC_PERIOD);

  /* Set RTC control register 1 */

  regval = ra6m5_rtc_getreg(RA6M5_RTC_RCR1_OFFSET);
  regval &= ~RTC_RCR1_PES_MASK;
  regval |= (RTC_RCR1_PIE | RTC_RCR1_PES(prd));
  ra6m5_rtc_putreg(RA6M5_RTC_RCR1_OFFSET, regval);

  /* Attach the isr */

  irq_attach(RA6M5_IRQ_RTC_PERIOD, rtc_periodic_interrupt, NULL);

  /* Start RTC counter */

  regval = ra6m5_rtc_getreg(RA6M5_RTC_RCR2_OFFSET);
  regval |= RTC_RCR2_START;
  ra6m5_rtc_putreg(RA6M5_RTC_RCR2_OFFSET, regval);

  /* Enable ICU periodic interrupt */

  up_enable_irq(RA6M5_IRQ_RTC_PERIOD);

  /* Set periodic interrupt priority level */

  leave_critical_section(flags);
  return OK;
}
#endif

#ifdef CONFIG_RA6M5_RTC_CARRY
void ra6m5_rtc_set_carry(carrycb_t callback)
{
  uint8_t regval;
  irqstate_t flags;
  flags = enter_critical_section();

  /* No.. Save the callback function pointer */

  g_carrycb = callback;

  /* Clear IRQ flag of carry interrupt */

  up_clear_irq(RA6M5_IRQ_RTC_CARRY);

  /* Attach the isr */

  irq_attach(RA6M5_IRQ_RTC_CARRY, rtc_carry_interrupt, NULL);

  /* Start the timer */

  regval = ra6m5_rtc_getreg(RA6M5_RTC_RCR2_OFFSET);
  regval |= RTC_RCR2_START;
  ra6m5_rtc_putreg(RA6M5_RTC_RCR2_OFFSET, regval);

  /* Enable ICU carry interrupt */

  up_enable_irq(RA6M5_IRQ_RTC_CARRY);

  /* Set carry interrupt priority level */

  leave_critical_section(flags);
}

#endif
#ifdef CONFIG_RTC_ALARM
int ra6m5_rtc_cancelalarm(void)
{
  irqstate_t flags;
  int ret = -ENODATA;

  flags = enter_critical_section();

  /* Cancel the global callback function */

  g_alarmcb.ac_cb = NULL;
  g_alarmcb.ac_arg = NULL;

  /* Unset the alarm */

  ra6m5_rtc_putreg(RA6M5_RTC_RSECAR_OFFSET, 0x0);
  ra6m5_rtc_putreg(RA6M5_RTC_RMINAR_OFFSET, 0x0);
  ra6m5_rtc_putreg(RA6M5_RTC_RHRAR_OFFSET, 0x0);
  ra6m5_rtc_putreg(RA6M5_RTC_RWKAR_OFFSET, 0x0);
  ra6m5_rtc_putreg(RA6M5_RTC_RDAYAR_OFFSET, 0x0);
  ra6m5_rtc_putreg(RA6M5_RTC_RMONAR_OFFSET, 0x0);
  ra6m5_rtc_putreg16(RA6M5_RTC_RYRAR_OFFSET, 0x0);
  ret = OK;

  leave_critical_section(flags);

  return ret;
}

#endif

#ifdef CONFIG_RTC_PERIODIC
int ra6m5_rtc_cancelperiodic(void)
{
  uint8_t regval;

  /* Disable ICU periodic interrupt */

  up_disable_irq(RA6M5_IRQ_RTC_PERIOD);

  /* Clear IRQ flag of periodic interrupt */

  up_clear_irq(RA6M5_IRQ_RTC_PERIOD);

  /* Disable RTC periodic interrupt */

  regval = ra6m5_rtc_getreg(RA6M5_RTC_RCR1_OFFSET);
  regval &= ~RTC_RCR1_PIE;
  ra6m5_rtc_putreg(RA6M5_RTC_RCR1_OFFSET, regval);

  while (ra6m5_rtc_getreg(RA6M5_RTC_RCR1_OFFSET) & RTC_RCR1_PIE)
    {
      /* Wait for this write to complete. */
    }

  return OK;
}

#endif

#if defined(CONFIG_RA6M5_RTC_CARRY)

int ra6m5_rtc_cancelcarry(void)
{
  uint8_t regval;

  /* Clear IRQ flag of carry interrupt */

  up_clear_irq(RA6M5_IRQ_RTC_CARRY);

  /* Disable ICU carry interrupt */

  up_disable_irq(RA6M5_IRQ_RTC_CARRY);

  /* Disable RTC carry interrupt */

  regval = ra6m5_rtc_getreg(RA6M5_RTC_RCR1_OFFSET);
  regval &= ~RTC_RCR1_CIE;
  ra6m5_rtc_putreg(RA6M5_RTC_RCR1_OFFSET, regval);

  return OK;
}

#endif
/****************************************************************************
 * Name: up_rtc_getdatetime
 *
 * Description:
 *   Get the current date and time from the date/time RTC.  This interface
 *   is only supported by the date/time RTC hardware implementation.
 *   It is used to replace the system timer.  It is only used
 *   by the RTOS during initialization to set up the system time
 *   when CONFIG_RTC  and CONFIG_RTC_DATETIME
 *   are selected (and CONFIG_RTC_HIRES is not).
 *
 *   NOTE: Some date/time RTC hardware is capability of sub-second accuracy.
 *   That sub-second accuracy is lost in this interface.  However,
 *   since the system time is reinitialized on each power-up/reset,
 *   there will be no timing inaccuracy in the long run.
 *
 * Input Parameters:
 *   tp - The location to return the high resolution time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_DATETIME
int up_rtc_getdatetime(struct tm *tp)
{
  uint8_t weekcnt;
  uint8_t daycnt;
  uint8_t monthcnt;
  uint8_t yearcnt;
  uint8_t seccnt;
  uint8_t mincnt;
  uint8_t hrcnt;
  uint8_t tmp_week;
  uint8_t tmp_day;
  uint8_t tmp_month;
  uint8_t tmp_year;
  uint16_t bcd_years;
  uint8_t regval;

  /* Sample the data time registers.  There is a race condition here...
   * If we sample the time just before midnight on December 31,
   * the date could be wrong because the day rolled over while were sampling.
   */

  regval = ra6m5_rtc_getreg(RA6M5_RTC_RCR2_OFFSET);
  if (!(regval & RTC_RCR2_START))
    {
      regval |= RTC_RCR2_START;
      ra6m5_rtc_putreg(RA6M5_RTC_RCR2_OFFSET, regval);
    }

  do
    {
      weekcnt  = ra6m5_rtc_getreg(RA6M5_RTC_RWKCNT_OFFSET);
      daycnt   = ra6m5_rtc_getreg(RA6M5_RTC_RDAYCNT_OFFSET);
      monthcnt = ra6m5_rtc_getreg(RA6M5_RTC_RMONCNT_OFFSET);
      yearcnt  = ra6m5_rtc_getreg(RA6M5_RTC_RYRCNT_OFFSET);
      seccnt   = ra6m5_rtc_getreg(RA6M5_RTC_RSECCNT_OFFSET);
      mincnt   = ra6m5_rtc_getreg(RA6M5_RTC_RMINCNT_OFFSET);
      hrcnt    = ra6m5_rtc_getreg(RA6M5_RTC_RHRCNT_OFFSET);
      tmp_week = ra6m5_rtc_getreg(RA6M5_RTC_RWKCNT_OFFSET);
      tmp_day  = ra6m5_rtc_getreg(RA6M5_RTC_RDAYCNT_OFFSET);
      tmp_month = ra6m5_rtc_getreg(RA6M5_RTC_RMONCNT_OFFSET);
      tmp_year  = ra6m5_rtc_getreg(RA6M5_RTC_RYRCNT_OFFSET);
    }

  while (tmp_week != weekcnt && tmp_day != daycnt &&
         tmp_month != monthcnt && tmp_year != yearcnt);

  rtc_dumpregs("Reading Time");

  /* Convert the RTC time register fields to struct tm format.
   *
   *   struct tm       TIMR register
   *   tm_sec    0-61* SEC    (0-59)
   *   tm_min    0-59  MIN    (0-59)
   *   tm_hour   0-23  HOUR   (0-23)
   *
   *  *To allow for leap seconds.  But these never actuall happen.
   */

  /* Disable carry interrupt */

  up_disable_irq(RA6M5_IRQ_RTC_CARRY);

  /* Enable RTC carry interrupt */

  regval = ra6m5_rtc_getreg(RA6M5_RTC_RCR1_OFFSET);
  regval |= (RTC_RCR1_CIE);
  ra6m5_rtc_putreg(RA6M5_RTC_RCR1_OFFSET, regval);

  do
    {
      /* Clear carry flag in ICU */

      up_clear_irq(RA6M5_IRQ_RTC_CARRY);

      /* Read and convert RTC registers;
       * mask off unknown bits and hour am/pm.
       */

      /* Seconds. (0-59) */

      tp->tm_sec = rtc_bcd2dec((uint8_t) (ra6m5_rtc_getreg(RA6M5_RTC_RSECCNT_OFFSET) & 0x7fu));

      /* Minutes. (0-59) */

      tp->tm_min = rtc_bcd2dec((uint8_t) (ra6m5_rtc_getreg(RA6M5_RTC_RMINCNT_OFFSET) & 0x7fu));

      /* Hours. (0-23) */

      tp->tm_hour = rtc_bcd2dec((uint8_t) (ra6m5_rtc_getreg(RA6M5_RTC_RHRCNT_OFFSET) & 0x3fu));

      /* Day of the month (1-31) */

      tp->tm_mday = rtc_bcd2dec(ra6m5_rtc_getreg(RA6M5_RTC_RDAYCNT_OFFSET));

      /* Months since January (0-11) */

      tp->tm_mon =  rtc_bcd2dec(ra6m5_rtc_getreg(RA6M5_RTC_RMONCNT_OFFSET)) - 1;

      /* Years since 2000 */

      bcd_years = (uint16_t) ra6m5_rtc_getreg16(RA6M5_RTC_RYRCNT_OFFSET);

      /* years years since 1900 (100-199) */

      tp->tm_year = rtc_bcd2dec((uint8_t) (bcd_years & 0xff)) + 100;

      /* Days since Sunday (0-6) */

      tp->tm_wday = (int) (ra6m5_rtc_getreg(RA6M5_RTC_RWKCNT_OFFSET) & 0x07u);
      rtc_dumptime(tp, "Returning");
    }

  while (up_status_irq(RA6M5_IRQ_RTC_CARRY));

  UNUSED(hrcnt);
  UNUSED(mincnt);
  UNUSED(seccnt);

  return OK;
}

#endif
#endif /* CONFIG_RA6M5_RTC */
