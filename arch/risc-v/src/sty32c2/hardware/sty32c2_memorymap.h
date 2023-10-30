/****************************************************************************
 * arch/risc-v/src/sty32c2/hardware/sty32c2_memorymap.h
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

#ifndef __ARCH_RISCV_SRC_STY32C2_HARDWARE_STY32C2_MEMORYMAP_H
#define __ARCH_RISCV_SRC_STY32C2_HARDWARE_STY32C2_MEMORYMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Base Address ****************************************************/

#define STY32C2_REG_STRIDE          0x800
#define STY32C2_CTRL_BASE           0xf0000000
#define STY32C2_CLINT_BASE          0xf0010000
#define STY32C2_PLIC_BASE           0xf0c00000

#if defined(CONFIG_ARCH_BOARD_ULX3S)

#define STY32C2_GPIO_BASE           0xf0000800
#define STY32C2_TIMER0_BASE         0xf0009000
#define STY32C2_UART0_BASE          0xf0009800
#define STY32C2_UART1_BASE          0xf000a000
#define STY32C2_UART2_BASE          0xf000a800
#define STY32C2_I2C0_BASE           0xf0001000
#define STY32C2_I2C1_BASE           0xf0001800
#define STY32C2_SPI0_BASE           0xf0006800
#define STY32C2_SPI1_BASE           0xf0007000
#define STY32C2_SPI2_BASE           0xf0007800
#define STY32C2_LEDS_BASE           0xf0002800
#define STY32C2_PWM0_BASE           0xf0003000

#define STY32C2_SDBLOCK2MEM_BASE    0xf0003800
#define STY32C2_SDCORE_BASE         0xf0004000
#define STY32C2_SDIRQ_BASE          0xf0004800
#define STY32C2_SDMEM2BLOCK_BASE    0xf0005000
#define STY32C2_SDPHY_BASE          0xf0005800

#elif defined(CONFIG_ARCH_BOARD_TI60DEV)

#define STY32C2_TIMER0_BASE         0xf0004800
#define STY32C2_UART0_BASE          0xf0005000

#define STY32C2_SDBLOCK2MEM_BASE    0xf0001000
#define STY32C2_SDCORE_BASE         0xf0001800
#define STY32C2_SDIRQ_BASE          0xf0002000
#define STY32C2_SDMEM2BLOCK_BASE    0xf0002800
#define STY32C2_SDPHY_BASE          0xf0003000

#else
#error "Invalid board definition!"
#endif

#endif /* __ARCH_RISCV_SRC_STY32C2_HARDWARE_STY32C2_MEMORYMAP_H */
