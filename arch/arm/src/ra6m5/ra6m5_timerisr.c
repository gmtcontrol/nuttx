/****************************************************************************
 * arch/arm/src/ra6m5/ra6m5_timerisr.c
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
#include <time.h>
#include <debug.h>
#include <nuttx/arch.h>
#include <nuttx/timers/arch_timer.h>

#include "nvic.h"
#include "systick.h"

#include "chip.h"
#include "ra6m5.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  ra6m5_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

#if !defined(CONFIG_ARMV8M_SYSTICK) || !defined(CONFIG_TIMER_ARCH)
static int ra6m5_timerisr(int irq, uint32_t *regs, void *arg)
{
  /* Process timer interrupt */

  nxsched_process_timer();

  return 0;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  up_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize
 *   the timer interrupt.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  uint32_t regval;

  /* Set the SysTick interrupt to the default priority */

  regval = getreg32(NVIC_SYSH12_15_PRIORITY);
  regval &= ~NVIC_SYSH_PRIORITY_PR15_MASK;
  regval |= (NVIC_SYSH_PRIORITY_DEFAULT << NVIC_SYSH_PRIORITY_PR15_SHIFT);
  putreg32(regval, NVIC_SYSH12_15_PRIORITY);

  /* Use SysTick to drive system timer  */

#if defined(CONFIG_ARMV8M_SYSTICK) && defined(CONFIG_TIMER_ARCH)
  up_timer_set_lowerhalf(systick_initialize(true, g_clock_freq[RA6M5_CLOCKS_SOURCE_SYSYEM], -1));
#else

  /* Make sure that the SYSTICK clock source is set correctly */

  regval = getreg32(NVIC_SYSTICK_CTRL);
  regval |= NVIC_SYSTICK_CTRL_CLKSOURCE;
  putreg32(regval, NVIC_SYSTICK_CTRL);

  /* 
  * The desired timer interrupt frequency is provided by the definition
  * CLK_TCK (see include/time.h).  CLK_TCK defines the desired number of
  * system clock ticks per second.  That value is a user configurable setting
  * that defaults to 100 (100 ticks per second = 10 MS interval). 
  */

  uint32_t systick = ((g_clock_freq[RA6M5_CLOCKS_SOURCE_SYSYEM] / CLK_TCK) - 1);

  /* The size of the reload field is 24 bits.  Verify that the reload value
  * will fit in the reload register.
  */

  if (systick > 0x00ffffff)  {
      return;
  }

  /* Configure SysTick to interrupt at the requested rate */

  putreg32(systick, NVIC_SYSTICK_RELOAD);

  /* Attach the timer interrupt vector */

  irq_attach(RA6M5_IRQ_SYSTICK, (xcpt_t)ra6m5_timerisr, NULL);

  /* Enable SysTick interrupts */

  putreg32((NVIC_SYSTICK_CTRL_CLKSOURCE | NVIC_SYSTICK_CTRL_TICKINT |
          NVIC_SYSTICK_CTRL_ENABLE), NVIC_SYSTICK_CTRL);

  /* And enable the timer interrupt */

  up_enable_irq(RA6M5_IRQ_SYSTICK);
#endif
}
