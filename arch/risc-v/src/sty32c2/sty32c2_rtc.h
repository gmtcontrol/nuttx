/****************************************************************************
 * arch/risc-v/src/sty32c2/sty32c2_rtc.h
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

#ifndef __ARCH_RISCV_SRC_STY32C2_STY32C2_RTC_H
#define __ARCH_RISCV_SRC_STY32C2_STY32C2_RTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

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
                                 uint32_t compval_high,  uint8_t comp_mode);

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

void sty32c2_clear_rtc_counter(void);

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

void sty32c2_enable_rtc_counter(void);

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

void sty32c2_get_rtc_timer_val(uint32_t *val_low, uint32_t *val_high);

/****************************************************************************
 * Name: sty32c2_rtc_lowerhalf_initialize
 *
 * Description:
 *   None.
 *
 * Input Parameters:
 *   pwm - A number identifying the pwm instance.
 *
 * Returned Value:
 *   On success, a pointer to the BL602 lower half PWM driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

struct rtc_lowerhalf_s *sty32c2_rtc_lowerhalf_initialize(void);

#endif /* __ARCH_RISCV_SRC_STY32C2_STY32C2_RTC_H */
