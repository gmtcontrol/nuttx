/****************************************************************************
 * arch/risc-v/src/sty32c2/hardware/sty32c2_plic.h
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

#ifndef __ARCH_RISCV_SRC_STY32C2_HARDWARE_STY32C2_GPIO_H
#define __ARCH_RISCV_SRC_STY32C2_HARDWARE_STY32C2_GPIO_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIO_INPUT_OFFSET       (0)
#define GPIO_OUTPUT_OFFSET      (4)

#define STY32C2_GPIO_INPUT_REG \
    (STY32C2_GPIO_BASE + GPIO_INPUT_OFFSET)

#define STY32C2_GPIO_OUTPUT_REG \
    (STY32C2_GPIO_BASE + GPIO_OUTPUT_OFFSET)

#endif /* __ARCH_RISCV_SRC_STY32C2_HARDWARE_STY32C2_GPIO_H */
