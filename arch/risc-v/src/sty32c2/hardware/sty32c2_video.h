/****************************************************************************
 * arch/risc-v/src/sty32c2/hardware/sty32c2_i2c.h
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

#ifndef __ARCH_RISCV_SRC_STY32C2_HARDWARE_STY32C2_VIDEO_H
#define __ARCH_RISCV_SRC_STY32C2_HARDWARE_STY32C2_VIDEO_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VIDEO_DMA_BASE_ADDR     (0x00)
#define VIDEO_DMA_LENG_ADDR     (0x04)
#define VIDEO_DMA_ENAB_ADDR     (0x08)
#define VIDEO_DMA_DONE_ADDR     (0x0c)
#define VIDEO_DMA_LOOP_ADDR     (0x10)
#define VIDEO_DMA_OFFS_ADDR     (0x14)

#define VIDEO_VTG_ENAB_ADDR     (0x00)
#define VIDEO_VTG_HRES_ADDR     (0x04)
#define VIDEO_VTG_VRES_ADDR     (0x14)

#define STY32C2_FBDMA_BASE_REG \
    (STY32C2_FBDEV_BASE + VIDEO_DMA_BASE_ADDR)

#define STY32C2_FBDMA_LENG_REG \
    (STY32C2_FBDEV_BASE + VIDEO_DMA_LENG_ADDR)

#define STY32C2_FBDMA_ENAB_REG \
    (STY32C2_FBDEV_BASE + VIDEO_DMA_ENAB_ADDR)

#define STY32C2_FBDMA_DONE_REG \
    (STY32C2_FBDEV_BASE + VIDEO_DMA_DONE_ADDR)

#define STY32C2_FBDMA_LOOP_REG \
    (STY32C2_FBDEV_BASE + VIDEO_DMA_LOOP_ADDR)

#define STY32C2_FBDMA_OFFS_REG \
    (STY32C2_FBDEV_BASE + VIDEO_DMA_OFFS_ADDR)


#define STY32C2_VTG_ENAB_REG \
    (STY32C2_VTGDEV_BASE + VIDEO_VTG_ENAB_ADDR)

#define STY32C2_VTG_HRES_REG \
    (STY32C2_VTGDEV_BASE + VIDEO_VTG_HRES_ADDR)

#define STY32C2_VTG_VRES_REG \
    (STY32C2_VTGDEV_BASE + VIDEO_VTG_VRES_ADDR)

#endif /* __ARCH_RISCV_SRC_STY32C2_HARDWARE_STY32C2_VIDEO_H */
