/****************************************************************************
 * boards/arm/stm32/glcx96/src/stm32_autoleds.c
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
#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "stm32.h"
#include "glcx96.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The following definitions map the encoded LED setting to GPIO settings */

#define GLCX96_LEDR       (1 << 0)
#define GLCX96_LEDE       (1 << 1)
#define GLCX96_LEDC       (1 << 2)

#define ON_SETBITS_SHIFT  (0)
#define ON_CLRBITS_SHIFT  (4)
#define OFF_SETBITS_SHIFT (8)
#define OFF_CLRBITS_SHIFT (12)

#define ON_BITS(v)        ((v) & 0xff)
#define OFF_BITS(v)       (((v) >> 8) & 0x0ff)
#define SETBITS(b)        ((b) & 0x0f)
#define CLRBITS(b)        (((b) >> 4) & 0x0f)

#define ON_SETBITS(v)     (SETBITS(ON_BITS(v))
#define ON_CLRBITS(v)     (CLRBITS(ON_BITS(v))
#define OFF_SETBITS(v)    (SETBITS(OFF_BITS(v))
#define OFF_CLRBITS(v)    (CLRBITS(OFF_BITS(v))

#define LED_STARTED_ON_SETBITS       ((GLCX96_LEDR) << ON_SETBITS_SHIFT)
#define LED_STARTED_ON_CLRBITS       ((GLCX96_LEDE|GLCX96_LEDC) << ON_CLRBITS_SHIFT)
#define LED_STARTED_OFF_SETBITS      (0 << OFF_SETBITS_SHIFT)
#define LED_STARTED_OFF_CLRBITS      ((GLCX96_LEDR|GLCX96_LEDE|GLCX96_LEDC) << OFF_CLRBITS_SHIFT)

#define LED_HEAPALLOCATE_ON_SETBITS  ((GLCX96_LEDE) << ON_SETBITS_SHIFT)
#define LED_HEAPALLOCATE_ON_CLRBITS  ((GLCX96_LEDR|GLCX96_LEDC) << ON_CLRBITS_SHIFT)
#define LED_HEAPALLOCATE_OFF_SETBITS ((GLCX96_LEDR) << OFF_SETBITS_SHIFT)
#define LED_HEAPALLOCATE_OFF_CLRBITS ((GLCX96_LEDE|GLCX96_LEDC) << OFF_CLRBITS_SHIFT)

#define LED_IRQSENABLED_ON_SETBITS   ((GLCX96_LEDR|GLCX96_LEDE) << ON_SETBITS_SHIFT)
#define LED_IRQSENABLED_ON_CLRBITS   ((GLCX96_LEDC) << ON_CLRBITS_SHIFT)
#define LED_IRQSENABLED_OFF_SETBITS  ((GLCX96_LEDE) << OFF_SETBITS_SHIFT)
#define LED_IRQSENABLED_OFF_CLRBITS  ((GLCX96_LEDR|GLCX96_LEDC) << OFF_CLRBITS_SHIFT)

#define LED_STACKCREATED_ON_SETBITS  ((GLCX96_LEDC) << ON_SETBITS_SHIFT)
#define LED_STACKCREATED_ON_CLRBITS  ((GLCX96_LEDR|GLCX96_LEDE) << ON_CLRBITS_SHIFT)
#define LED_STACKCREATED_OFF_SETBITS ((GLCX96_LEDR|GLCX96_LEDE) << OFF_SETBITS_SHIFT)
#define LED_STACKCREATED_OFF_CLRBITS ((GLCX96_LEDC) << OFF_CLRBITS_SHIFT)

#define LED_INIRQ_ON_SETBITS         ((GLCX96_LEDR) << ON_SETBITS_SHIFT)
#define LED_INIRQ_ON_CLRBITS         ((0) << ON_CLRBITS_SHIFT)
#define LED_INIRQ_OFF_SETBITS        ((0) << OFF_SETBITS_SHIFT)
#define LED_INIRQ_OFF_CLRBITS        ((GLCX96_LEDR) << OFF_CLRBITS_SHIFT)

#define LED_SIGNAL_ON_SETBITS        ((GLCX96_LEDE) << ON_SETBITS_SHIFT)
#define LED_SIGNAL_ON_CLRBITS        ((0) << ON_CLRBITS_SHIFT)
#define LED_SIGNAL_OFF_SETBITS       ((0) << OFF_SETBITS_SHIFT)
#define LED_SIGNAL_OFF_CLRBITS       ((GLCX96_LEDE) << OFF_CLRBITS_SHIFT)

#define LED_ASSERTION_ON_SETBITS     ((GLCX96_LEDE) << ON_SETBITS_SHIFT)
#define LED_ASSERTION_ON_CLRBITS     ((0) << ON_CLRBITS_SHIFT)
#define LED_ASSERTION_OFF_SETBITS    ((0) << OFF_SETBITS_SHIFT)
#define LED_ASSERTION_OFF_CLRBITS    ((GLCX96_LEDE) << OFF_CLRBITS_SHIFT)

#define LED_PANIC_ON_SETBITS         ((GLCX96_LEDE) << ON_SETBITS_SHIFT)
#define LED_PANIC_ON_CLRBITS         ((0) << ON_CLRBITS_SHIFT)
#define LED_PANIC_OFF_SETBITS        ((0) << OFF_SETBITS_SHIFT)
#define LED_PANIC_OFF_CLRBITS        ((GLCX96_LEDE) << OFF_CLRBITS_SHIFT)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint16_t g_ledbits[8] =
{
  (LED_STARTED_ON_SETBITS       | LED_STARTED_ON_CLRBITS |
  LED_STARTED_OFF_SETBITS      | LED_STARTED_OFF_CLRBITS),

  (LED_HEAPALLOCATE_ON_SETBITS  | LED_HEAPALLOCATE_ON_CLRBITS |
  LED_HEAPALLOCATE_OFF_SETBITS | LED_HEAPALLOCATE_OFF_CLRBITS),

  (LED_IRQSENABLED_ON_SETBITS   | LED_IRQSENABLED_ON_CLRBITS |
  LED_IRQSENABLED_OFF_SETBITS  | LED_IRQSENABLED_OFF_CLRBITS),

  (LED_STACKCREATED_ON_SETBITS  | LED_STACKCREATED_ON_CLRBITS |
  LED_STACKCREATED_OFF_SETBITS | LED_STACKCREATED_OFF_CLRBITS),

  (LED_INIRQ_ON_SETBITS         | LED_INIRQ_ON_CLRBITS |
  LED_INIRQ_OFF_SETBITS        | LED_INIRQ_OFF_CLRBITS),

  (LED_SIGNAL_ON_SETBITS        | LED_SIGNAL_ON_CLRBITS |
  LED_SIGNAL_OFF_SETBITS       | LED_SIGNAL_OFF_CLRBITS),

  (LED_ASSERTION_ON_SETBITS     | LED_ASSERTION_ON_CLRBITS |
  LED_ASSERTION_OFF_SETBITS    | LED_ASSERTION_OFF_CLRBITS),

  (LED_PANIC_ON_SETBITS         | LED_PANIC_ON_CLRBITS |
  LED_PANIC_OFF_SETBITS        | LED_PANIC_OFF_CLRBITS)
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void led_clrbits(unsigned int clrbits)
{
  if ((clrbits & GLCX96_LEDR) != 0)
    {
      stm32_gpiowrite(GPIO_LEDRUN, false);
    }

  if ((clrbits & GLCX96_LEDE) != 0)
    {
      stm32_gpiowrite(GPIO_LEDERR, false);
    }

  if ((clrbits & GLCX96_LEDC) != 0)
    {
      stm32_gpiowrite(GPIO_LEDCOM, false);
    }
}

static inline void led_setbits(unsigned int setbits)
{
  if ((setbits & GLCX96_LEDR) != 0)
    {
      stm32_gpiowrite(GPIO_LEDRUN, true);
    }

  if ((setbits & GLCX96_LEDE) != 0)
    {
      stm32_gpiowrite(GPIO_LEDERR, true);
    }

  if ((setbits & GLCX96_LEDC) != 0)
    {
      stm32_gpiowrite(GPIO_LEDCOM, true);
    }
}

static void led_setonoff(unsigned int bits)
{
  led_clrbits(CLRBITS(bits));
  led_setbits(SETBITS(bits));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_led_initialize
 ****************************************************************************/

void stm32_led_initialize(void)
{
  /* Configure LED1-4 GPIOs for output */

  stm32_configgpio(GPIO_LEDRUN);
  stm32_configgpio(GPIO_LEDERR);
  stm32_configgpio(GPIO_LEDCOM);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  led_setonoff(ON_BITS(g_ledbits[led]));
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  led_setonoff(OFF_BITS(g_ledbits[led]));
}

#endif /* CONFIG_ARCH_LEDS */
