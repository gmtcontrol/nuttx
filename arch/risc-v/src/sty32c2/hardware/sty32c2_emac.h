/****************************************************************************
 * arch/risc-v/src/sty32c2/hardware/sty32c2_emac.h
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

#ifndef __ARCH_RISCV_SRC_STY32C2_HARDWARE_STY32C2_EMAC_H
#define __ARCH_RISCV_SRC_STY32C2_HARDWARE_STY32C2_EMAC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/sty32c2_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ETHMAC_RX_SLOTS   2
#define ETHMAC_TX_SLOTS   2
#define ETHMAC_SLOT_SIZE  2048

/* EMAC Register Offsets ****************************************************/

#define STY32C2_ETHMAC_SRAM_WRITER_SLOT_OFFSET            0x0000
#define STY32C2_ETHMAC_SRAM_WRITER_LENGTH_OFFSET          0x0004
#define STY32C2_ETHMAC_SRAM_WRITER_ERRORS_OFFSET          0x0008
#define STY32C2_ETHMAC_SRAM_WRITER_EV_STATUS_OFFSET       0x000c
#define STY32C2_ETHMAC_SRAM_WRITER_EV_PENDING_OFFSET      0x0010
#define STY32C2_ETHMAC_SRAM_WRITER_EV_ENABLE_OFFSET       0x0014
#define STY32C2_ETHMAC_SRAM_READER_START_OFFSET           0x0018
#define STY32C2_ETHMAC_SRAM_READER_READY_OFFSET           0x001c
#define STY32C2_ETHMAC_SRAM_READER_LEVEL_OFFSET           0x0020
#define STY32C2_ETHMAC_SRAM_READER_SLOT_OFFSET            0x0024
#define STY32C2_ETHMAC_SRAM_READER_LENGTH_OFFSET          0x0028
#define STY32C2_ETHMAC_SRAM_READER_EV_STATUS_OFFSET       0x002c
#define STY32C2_ETHMAC_SRAM_READER_EV_PENDING_OFFSET      0x0030
#define STY32C2_ETHMAC_SRAM_READER_EV_ENABLE_OFFSET       0x0034
#define STY32C2_ETHMAC_PREAMBLE_CRC_OFFSET                0x0038
#define STY32C2_ETHMAC_RX_DATAPATH_PREAMBLE_ERRORS_OFFSET 0x003c
#define STY32C2_ETHMAC_RX_DATAPATH_CRC_ERRORS_OFFSET      0x0040

#define STY32C2_ETHPHY_CRG_RESET_OFFSET                   0x0000
#define STY32C2_ETHPHY_MDIO_W_OFFSET                      0x0004
#define STY32C2_ETHPHY_MDIO_R_OFFSET                      0x0008

/* STY32C2_EMAC register addresses ********************************************/

#define STY32C2_ETHMAC_SRAM_WRITER_SLOT                   (STY32C2_ETHMAC_BASE + STY32C2_ETHMAC_SRAM_WRITER_SLOT_OFFSET)
#define STY32C2_ETHMAC_SRAM_WRITER_LENGTH                 (STY32C2_ETHMAC_BASE + STY32C2_ETHMAC_SRAM_WRITER_LENGTH_OFFSET)
#define STY32C2_ETHMAC_SRAM_WRITER_ERRORS                 (STY32C2_ETHMAC_BASE + STY32C2_ETHMAC_SRAM_WRITER_ERRORS_OFFSET)
#define STY32C2_ETHMAC_SRAM_WRITER_EV_STATUS              (STY32C2_ETHMAC_BASE + STY32C2_ETHMAC_SRAM_WRITER_EV_STATUS_OFFSET)
#define STY32C2_ETHMAC_SRAM_WRITER_EV_PENDING             (STY32C2_ETHMAC_BASE + STY32C2_ETHMAC_SRAM_WRITER_EV_PENDING_OFFSET)
#define STY32C2_ETHMAC_SRAM_WRITER_EV_ENABLE              (STY32C2_ETHMAC_BASE + STY32C2_ETHMAC_SRAM_WRITER_EV_ENABLE_OFFSET)
#define STY32C2_ETHMAC_SRAM_READER_START                  (STY32C2_ETHMAC_BASE + STY32C2_ETHMAC_SRAM_READER_START_OFFSET)
#define STY32C2_ETHMAC_SRAM_READER_READY                  (STY32C2_ETHMAC_BASE + STY32C2_ETHMAC_SRAM_READER_READY_OFFSET)
#define STY32C2_ETHMAC_SRAM_READER_LEVEL                  (STY32C2_ETHMAC_BASE + STY32C2_ETHMAC_SRAM_READER_LEVEL_OFFSET)
#define STY32C2_ETHMAC_SRAM_READER_SLOT                   (STY32C2_ETHMAC_BASE + STY32C2_ETHMAC_SRAM_READER_SLOT_OFFSET)
#define STY32C2_ETHMAC_SRAM_READER_LENGTH                 (STY32C2_ETHMAC_BASE + STY32C2_ETHMAC_SRAM_READER_LENGTH_OFFSET)
#define STY32C2_ETHMAC_SRAM_READER_EV_STATUS              (STY32C2_ETHMAC_BASE + STY32C2_ETHMAC_SRAM_READER_EV_STATUS_OFFSET)
#define STY32C2_ETHMAC_SRAM_READER_EV_PENDING             (STY32C2_ETHMAC_BASE + STY32C2_ETHMAC_SRAM_READER_EV_PENDING_OFFSET)
#define STY32C2_ETHMAC_SRAM_READER_EV_ENABLE              (STY32C2_ETHMAC_BASE + STY32C2_ETHMAC_SRAM_READER_EV_ENABLE_OFFSET)
#define STY32C2_ETHMAC_PREAMBLE_CRC                       (STY32C2_ETHMAC_BASE + STY32C2_ETHMAC_PREAMBLE_CRC_OFFSET)
#define STY32C2_ETHMAC_RX_DATAPATH_PREAMBLE_ERRORS        (STY32C2_ETHMAC_BASE + STY32C2_ETHMAC_RX_DATAPATH_PREAMBLE_ERRORS_OFFSET)
#define STY32C2_ETHMAC_RX_DATAPATH_CRC_ERRORS             (STY32C2_ETHMAC_BASE + STY32C2_ETHMAC_RX_DATAPATH_CRC_ERRORS_OFFSET)

#define STY32C2_ETHPHY_CRG_RESET                          (STY32C2_ETHPHY_BASE + STY32C2_ETHPHY_CRG_RESET_OFFSET)
#define STY32C2_ETHPHY_MDIO_W                             (STY32C2_ETHPHY_BASE + STY32C2_ETHPHY_MDIO_W_OFFSET)
#define STY32C2_ETHPHY_MDIO_R                             (STY32C2_ETHPHY_BASE + STY32C2_ETHPHY_MDIO_R_OFFSET)

#endif /* __ARCH_RISCV_SRC_STY32C2_HARDWARE_STY32C2_EMAC_H */
