/****************************************************************************
 * arch/risc-v/src/sty32c2/sty32c2_clockconfig.c
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
#include <arch/board/board.h>

#include "riscv_internal.h"
#include "sty32c2_clockconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sty32c2_get_cpuclk
 ****************************************************************************/

uint32_t sty32c2_get_cpuclk(void)
{
  /* fpga fabric default sys frequency */
#if defined(CONFIG_ARCH_BOARD_T20F256DK)
  return 50000000UL;
#elif defined(CONFIG_ARCH_BOARD_ULX3S)
  return 50000000UL;
#else
  return 100000000UL;
#endif
}

/****************************************************************************
 * Name: sty32c2_get_spiclk
 ****************************************************************************/

uint32_t sty32c2_get_spiclk(void)
{
  /* fpga fabric default spi frequency */
  return 25000000UL;
}

/****************************************************************************
 * Name: sty32c2_clockconfig
 ****************************************************************************/

void sty32c2_clockconfig(void)
{
  /* pll is set by fpga fabric */
}
