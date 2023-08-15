/****************************************************************************
 * arch/arm/src/ra6m5/ra6m5_rcc.h
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

#ifndef __ARCH_ARM_SRC_RA6M5_RA6M5_RCC_H
#define __ARCH_ARM_SRC_RA6M5_RA6M5_RCC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "arm_internal.h"
#include "chip.h"

#if defined(CONFIG_RA6M5_R7FA6M5BX)
#  include "hardware/r7fa6m5bx_rcc.h"
#else
#  error "Unsupported RA6M5 chip"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ra6m5_clockconfig
 *
 * Description:
 *   Called to establish the clock settings based on the values in board.h.
 *   This function (by default) will reset most everything, enable the PLL,
 *   and enable peripheral clocking for all periperipherals enabled in the
 *   NuttX configuration file.
 *
 *   If CONFIG_ARCH_BOARD_RA6M5_CUSTOM_CLOCKCONFIG is defined, then
 *   clocking will be enabled by an externally provided, board-specific
 *   function called ra6m5_board_clockconfig().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void ra6m5_clockconfig(void);

/****************************************************************************
 * Name: ra6m5_board_clockconfig
 *
 * Description:
 *   Any RA6M5 board may replace the "standard" board clock configuration
 *   logic with its own, custom clock configuration logic.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_BOARD_RA6M5_CUSTOM_CLOCKCONFIG
void ra6m5_board_clockconfig(void);
#endif

/****************************************************************************
 * Name: ra6m5_stdclockconfig
 *
 * Description:
 *   The standard logic to configure the clocks based on settings in board.h.
 *   Applicable if no custom clock config is provided.  This function is
 *   chip type specific and implemented in corresponding modules such as e.g.
 *   stm3262xx_rcc.c
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_BOARD_RA6M5_CUSTOM_CLOCKCONFIG
void ra6m5_stdclockconfig(void);
#endif

/****************************************************************************
 * Name: ra6m5_clockenable
 *
 * Description:
 *   Re-enable the clock and restore the clock settings based on settings in
 *   board.h.  This function is only available to support low-power modes of
 *   operation:  When re-awakening from deep-sleep modes, it is necessary to
 *   re-enable/re-start the PLL
 *
 *   This function performs a subset of the operations performed by
 *   ra6m5_clockconfig():  It does not reset any devices, and it does not
 *   reset the currently enabled peripheral clocks.
 *
 *   If CONFIG_ARCH_BOARD_RA6M5_CUSTOM_CLOCKCONFIG is defined, then
 *   clocking will be enabled by an externally provided, board-specific
 *   function called ra6m5_board_clockconfig().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_PM
void ra6m5_clockenable(void);
#endif

/****************************************************************************
 * Name: ra6m5_enable_peripherals_clock
 *
 * Description:
 *   Enable all the chip peripherals according to configuration.  This is
 *   chip type specific and thus implemented in corresponding modules such as
 *   e.g. stm3262xx_rcc.c
 *
 ****************************************************************************/

void ra6m5_enable_peripherals_clock(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RA6M5_RA6M5_RCC_H */
