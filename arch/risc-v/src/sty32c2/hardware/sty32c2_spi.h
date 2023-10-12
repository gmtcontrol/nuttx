/****************************************************************************
 * arch/risc-v/src/sty32c2/hardware/sty32c2_spi.h
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

#ifndef __ARCH_RISCV_SRC_STY32C2_HARDWARE_STY32C2_SPI_H
#define __ARCH_RISCV_SRC_STY32C2_HARDWARE_STY32C2_SPI_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SPI_CONTROL_OFFSET      (0x00)
#define SPI_STATUS_OFFSET       (0x04)
#define SPI_MOSI_OFFSET         (0x08)
#define SPI_MISO_OFFSET         (0x0c)
#define SPI_CS_OFFSET           (0x10)
#define SPI_LOOPBACK_OFFSET     (0x14)
#define SPI_CLKDIV_OFFSET       (0x18)

#define SPI_CONTROL_XFER        (0x01)
#define SPI_STATUS_DONE         (0x01)
#define SPI_CSMODE_SW           (0x10000)
#define SPI_CSMODE_HW           (0x00000)

#define STY32C2_SPI_CTRL_REG(bus) \
    (STY32C2_SPI0_BASE + (STY32C2_REG_STRIDE * (bus)) + SPI_CONTROL_OFFSET)

#define STY32C2_SPI_STAT_REG(bus) \
    (STY32C2_SPI0_BASE + (STY32C2_REG_STRIDE * (bus)) + SPI_STATUS_OFFSET)

#define STY32C2_SPI_MOSI_REG(bus) \
    (STY32C2_SPI0_BASE + (STY32C2_REG_STRIDE * (bus)) + SPI_MOSI_OFFSET)

#define STY32C2_SPI_MISO_REG(bus) \
    (STY32C2_SPI0_BASE + (STY32C2_REG_STRIDE * (bus)) + SPI_MISO_OFFSET)

#define STY32C2_SPI_CS_REG(bus) \
    (STY32C2_SPI0_BASE + (STY32C2_REG_STRIDE * (bus)) + SPI_CS_OFFSET)

#define STY32C2_SPI_LOOP_REG(bus) \
    (STY32C2_SPI0_BASE + (STY32C2_REG_STRIDE * (bus)) + SPI_LOOPBACK_OFFSET)

#define STY32C2_SPI_CDIV_REG(bus) \
    (STY32C2_SPI0_BASE + (STY32C2_REG_STRIDE * (bus)) + SPI_CLKDIV_OFFSET)

#endif /* __ARCH_RISCV_SRC_STY32C2_HARDWARE_STY32C2_SPI_H */
