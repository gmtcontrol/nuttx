/****************************************************************************
 * arch/risc-v/src/sty32c2/sty32c2_gpio.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/spinlock.h>
#include <arch/board/board.h>

#include "riscv_internal.h"
#include "sty32c2_gpio.h"
#include "sty32c2_memorymap.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sty32c2_gpio_config
 ****************************************************************************/

int sty32c2_gpio_config(uint16_t gpiocfg)
{
  return 0;
}

/****************************************************************************
 * Name: sty32c2_gpio_write
 ****************************************************************************/

void sty32c2_gpio_write(uint16_t gpiocfg, bool value)
{
  uint32_t pin  = sty32c2_gpio_getpin(gpiocfg);

  if (value)
    {
      modifyreg32(STY32C2_GPIO_OUTPUT_REG, 0, 0x1 << pin);
    }
  else
    {
      modifyreg32(STY32C2_GPIO_OUTPUT_REG, 0x1 << pin, 0);
    }
}

/****************************************************************************
 * Name: sty32c2_gpio_read
 ****************************************************************************/

bool sty32c2_gpio_read(uint16_t gpiocfg)
{
  uint32_t pin  = sty32c2_gpio_getpin(gpiocfg);
  return (getreg32(STY32C2_GPIO_INPUT_REG) & (0x1 << pin)) != 0;
}

/****************************************************************************
 * Name: sty32c2_gpio_clearpending
 ****************************************************************************/

void sty32c2_gpio_clearpending(uint32_t pin)
{
  ASSERT(0 <= pin && pin <= 31);
}
