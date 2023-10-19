/****************************************************************************
 * arch/risc-v/src/sty32c2/hardware/sty32c2_pwm.h
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

#ifndef __ARCH_RISCV_SRC_STY32C2_HARDWARE_STY32C2_PWM_H
#define __ARCH_RISCV_SRC_STY32C2_HARDWARE_STY32C2_PWM_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PWM_PERIOD_OFFSET       (0x00)
#define PWM_ENABLE_OFFSET       (0x04)
#define PWM_WIDTH_OFFSET        (0x08)

#define STY32C2_PWM_NCHANNELS   (7)

/* Channel 0,1,2,3,4,5,6 are the same so create this helper for register offsets */

#define STY32C2_PWM_CHAN_OFFSET(n)    (STY32C2_PWM0_BASE + ((n) * 0x0c))

#define STY32C2_PWM_ENABLE_REG(chan) \
    (STY32C2_PWM_CHAN_OFFSET(chan) + PWM_ENABLE_OFFSET)

#define STY32C2_PWM_WIDTH_REG(chan) \
    (STY32C2_PWM_CHAN_OFFSET(chan) + PWM_WIDTH_OFFSET)

#define STY32C2_PWM_PERIOD_REG(chan) \
    (STY32C2_PWM_CHAN_OFFSET(chan) + PWM_PERIOD_OFFSET)

#endif /* _ARCH_RISCV_SRC_STY32C2_CHIP_STY32C2_PWM_H */
