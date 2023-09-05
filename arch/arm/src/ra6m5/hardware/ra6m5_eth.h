/****************************************************************************
 * arch/arm/src/ra6m5/hardware/ra6m5_eth.h
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

#ifndef __ARCH_ARM_SRC_RA6M5_HARDWARE_RA6M5_ETH_H
#define __ARCH_ARM_SRC_RA6M5_HARDWARE_RA6M5_ETH_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ETHERC and EDMAC base Addresses */

#define RA6M5_EDMAC0_BASE               (0x40114000UL)     /* EDMAC base address */
#define RA6M5_ETHERC0_BASE              (0x40114100UL)     /* Ethernet MAC base address */

/* Ethernet Addresses */

/* Register Offsets  */

/* MAC Registers */

#define RA6M5_ETH_ECMR_OFFSET          (0x0000) /* ETHERC Mode register */
#define RA6M5_ETH_RFLR_OFFSET          (0x0008) /* Receive Frame Maximum Length Register */
#define RA6M5_ETH_ECSR_OFFSET          (0x0010) /* ETHERC Status Register */
#define RA6M5_ETH_ECSIPR_OFFSET        (0x0018) /* ETHERC Interrupt Enable Register */
#define RA6M5_ETH_PIR_OFFSET           (0x0020) /* PHY Interface Register */
#define RA6M5_ETH_PSR_OFFSET           (0x0028) /* PHY Status Register */
#define RA6M5_ETH_RDMLR_OFFSET         (0x0040) /* Random Number Generation Counter Limit Setting Register */
#define RA6M5_ETH_IPGR_OFFSET          (0x0050) /* Interpacket Gap Register */
#define RA6M5_ETH_APR_OFFSET           (0x0054) /* Automatic PAUSE Frame Register */
#define RA6M5_ETH_MPR_OFFSET           (0x0058) /* Manual PAUSE Frame Register */
#define RA6M5_ETH_RFCF_OFFSET          (0x0060) /* Received PAUSE Frame Counter */
#define RA6M5_ETH_TPAUSER_OFFSET       (0x0064) /* PAUSE Frame Retransmit Count Setting Register */
#define RA6M5_ETH_TPAUSECR_OFFSET      (0x0068) /* PAUSE Frame Retransmit Counter */
#define RA6M5_ETH_BCFRR_OFFSET         (0x006c) /* Broadcast Frame Receive Count Setting Register */
#define RA6M5_ETH_MAHR_OFFSET          (0x00c0) /* MAC Address Upper Bit Register */
#define RA6M5_ETH_MALR_OFFSET          (0x00c8) /* MAC Address Lower Bit Register */
#define RA6M5_ETH_TROCR_OFFSET         (0x00d0) /* Transmit Retry Over Counter Register */
#define RA6M5_ETH_CDCR_OFFSET          (0x00d4) /* Late Collision Detect Counter Register */
#define RA6M5_ETH_LCCR_OFFSET          (0x00d8) /* Lost Carrier Counter Register */
#define RA6M5_ETH_CNDCR_OFFSET         (0x00dc) /* Carrier Not Detect Counter Register */
#define RA6M5_ETH_CEFCR_OFFSET         (0x00e4) /* CRC Error Frame Receive Counter Register */
#define RA6M5_ETH_FRECR_OFFSET         (0x00e8) /* Frame Receive Error Counter Register */
#define RA6M5_ETH_TSFRCR_OFFSET        (0x00ec) /* Too-Short Frame Receive Counter Register */
#define RA6M5_ETH_TLFRCR_OFFSET        (0x00f0) /* Too-Long Frame Receive Counter Register */
#define RA6M5_ETH_RFCR_OFFSET          (0x00f4) /* Received Alignment Error Frame Counter Register */
#define RA6M5_ETH_MAFCR_OFFSET         (0x00f8) /* Multicast Address Frame Receive Counter Register */

/* DMA Registers */

#define RA6M5_ETHD_EDMR_OFFSET         (0x0000) /* EDMAC Mode Register  */
#define RA6M5_ETHD_EDTRR_OFFSET        (0x0008) /* EDMAC Transmit Request Register  */
#define RA6M5_ETHD_EDRRR_OFFSET        (0x0010) /* EDMAC Receive Request Register  */
#define RA6M5_ETHD_TDLAR_OFFSET        (0x0018) /* Transmit Descriptor List Start Address Register  */
#define RA6M5_ETHD_RDLAR_OFFSET        (0x0020) /* Receive Descriptor List Start Address Register  */
#define RA6M5_ETHD_EESR_OFFSET         (0x0028) /* ETHERC/EDMAC Status Register  */
#define RA6M5_ETHD_EESIPR_OFFSET       (0x0030) /* ETHERC/EDMAC Status Interrupt Enable Register  */
#define RA6M5_ETHD_TRSCER_OFFSET       (0x0038) /* ETHERC/EDMAC Transmit/Receive Status Copy Enable Register  */
#define RA6M5_ETHD_RMFCR_OFFSET        (0x0040) /* Missed-Frame Counter Register  */
#define RA6M5_ETHD_TFTR_OFFSET         (0x0048) /* Transmit FIFO Threshold Register  */
#define RA6M5_ETHD_FDR_OFFSET          (0x0050) /* FIFO Depth Register  */
#define RA6M5_ETHD_RMCR_OFFSET         (0x0058) /* Receive Method Control Register  */
#define RA6M5_ETHD_TFUCR_OFFSET        (0x0064) /* Transmit FIFO Underflow Counter  */
#define RA6M5_ETHD_RFOCR_OFFSET        (0x0068) /* Receive FIFO Overflow Counter  */
#define RA6M5_ETHD_IOSR_OFFSET         (0x006c) /* Independent Output Signal Setting Register  */
#define RA6M5_ETHD_FCFTR_OFFSET        (0x0070) /* Flow Control Start FIFO Threshold Setting Register  */
#define RA6M5_ETHD_RPADIR_OFFSET       (0x0078) /* Receive Data Padding Insert Register  */
#define RA6M5_ETHD_TRIMD_OFFSET        (0x007c) /* Transmit Interrupt Setting Register  */
#define RA6M5_ETHD_RBWAR_OFFSET        (0x00c8) /* Receive Buffer Write Address Register  */
#define RA6M5_ETHD_RDFAR_OFFSET        (0x00cc) /* Receive Descriptor Fetch Address Register  */
#define RA6M5_ETHD_TBRAR_OFFSET        (0x00d4) /* Transmit Buffer Read Address Register  */
#define RA6M5_ETHD_TDFAR_OFFSET        (0x00d8) /* Transmit Descriptor Fetch Address Register  */

/* Register Base Addresses */

/* MAC Registers */

#define RA6M5_ETH_ECMR     (RA6M5_ETHERC0_BASE+RA6M5_ETH_ECMR_OFFSET)
#define RA6M5_ETH_RFLR     (RA6M5_ETHERC0_BASE+RA6M5_ETH_RFLR_OFFSET)
#define RA6M5_ETH_ECSR     (RA6M5_ETHERC0_BASE+RA6M5_ETH_ECSR_OFFSET)
#define RA6M5_ETH_ECSIPR   (RA6M5_ETHERC0_BASE+RA6M5_ETH_ECSIPR_OFFSET)
#define RA6M5_ETH_PIR      (RA6M5_ETHERC0_BASE+RA6M5_ETH_PIR_OFFSET)
#define RA6M5_ETH_PSR      (RA6M5_ETHERC0_BASE+RA6M5_ETH_PSR_OFFSET)
#define RA6M5_ETH_RDMLR    (RA6M5_ETHERC0_BASE+RA6M5_ETH_RDMLR_OFFSET)
#define RA6M5_ETH_IPGR     (RA6M5_ETHERC0_BASE+RA6M5_ETH_IPGR_OFFSET)
#define RA6M5_ETH_APR      (RA6M5_ETHERC0_BASE+RA6M5_ETH_APR_OFFSET)
#define RA6M5_ETH_MPR      (RA6M5_ETHERC0_BASE+RA6M5_ETH_MPR_OFFSET)
#define RA6M5_ETH_RFCF     (RA6M5_ETHERC0_BASE+RA6M5_ETH_RFCF_OFFSET)
#define RA6M5_ETH_TPAUSER  (RA6M5_ETHERC0_BASE+RA6M5_ETH_TPAUSER_OFFSET)
#define RA6M5_ETH_TPAUSECR (RA6M5_ETHERC0_BASE+RA6M5_ETH_TPAUSECR_OFFSET)
#define RA6M5_ETH_BCFRR    (RA6M5_ETHERC0_BASE+RA6M5_ETH_BCFRR_OFFSET)
#define RA6M5_ETH_MAHR     (RA6M5_ETHERC0_BASE+RA6M5_ETH_MAHR_OFFSET)
#define RA6M5_ETH_MALR     (RA6M5_ETHERC0_BASE+RA6M5_ETH_MALR_OFFSET)
#define RA6M5_ETH_TROCR    (RA6M5_ETHERC0_BASE+RA6M5_ETH_TROCR_OFFSET)
#define RA6M5_ETH_CDCR     (RA6M5_ETHERC0_BASE+RA6M5_ETH_CDCR_OFFSET)
#define RA6M5_ETH_LCCR     (RA6M5_ETHERC0_BASE+RA6M5_ETH_LCCR_OFFSET)
#define RA6M5_ETH_CNDCR    (RA6M5_ETHERC0_BASE+RA6M5_ETH_CNDCR_OFFSET)
#define RA6M5_ETH_CEFCR    (RA6M5_ETHERC0_BASE+RA6M5_ETH_CEFCR_OFFSET)
#define RA6M5_ETH_FRECR    (RA6M5_ETHERC0_BASE+RA6M5_ETH_FRECR_OFFSET)
#define RA6M5_ETH_TSFRCR   (RA6M5_ETHERC0_BASE+RA6M5_ETH_TSFRCR_OFFSET)
#define RA6M5_ETH_TLFRCR   (RA6M5_ETHERC0_BASE+RA6M5_ETH_TLFRCR_OFFSET)
#define RA6M5_ETH_RFCR     (RA6M5_ETHERC0_BASE+RA6M5_ETH_RFCR_OFFSET)
#define RA6M5_ETH_MAFCR    (RA6M5_ETHERC0_BASE+RA6M5_ETH_MAFCR_OFFSET)

/* DMA Registers */

#define RA6M5_ETHD_EDMR          (RA6M5_EDMAC0_BASE+RA6M5_ETHD_EDMR_OFFSET)
#define RA6M5_ETHD_EDTRR         (RA6M5_EDMAC0_BASE+RA6M5_ETHD_EDTRR_OFFSET)
#define RA6M5_ETHD_EDRRR         (RA6M5_EDMAC0_BASE+RA6M5_ETHD_EDRRR_OFFSET)
#define RA6M5_ETHD_TDLAR         (RA6M5_EDMAC0_BASE+RA6M5_ETHD_TDLAR_OFFSET)
#define RA6M5_ETHD_RDLAR         (RA6M5_EDMAC0_BASE+RA6M5_ETHD_RDLAR_OFFSET)
#define RA6M5_ETHD_EESR          (RA6M5_EDMAC0_BASE+RA6M5_ETHD_EESR_OFFSET)
#define RA6M5_ETHD_EESIPR        (RA6M5_EDMAC0_BASE+RA6M5_ETHD_EESIPR_OFFSET)
#define RA6M5_ETHD_TRSCER        (RA6M5_EDMAC0_BASE+RA6M5_ETHD_TRSCER_OFFSET)
#define RA6M5_ETHD_RMFCR         (RA6M5_EDMAC0_BASE+RA6M5_ETHD_RMFCR_OFFSET)
#define RA6M5_ETHD_TFTR          (RA6M5_EDMAC0_BASE+RA6M5_ETHD_TFTR_OFFSET)
#define RA6M5_ETHD_FDR           (RA6M5_EDMAC0_BASE+RA6M5_ETHD_FDR_OFFSET)
#define RA6M5_ETHD_RMCR          (RA6M5_EDMAC0_BASE+RA6M5_ETHD_RMCR_OFFSET)
#define RA6M5_ETHD_TFUCR         (RA6M5_EDMAC0_BASE+RA6M5_ETHD_TFUCR_OFFSET)
#define RA6M5_ETHD_RFOCR         (RA6M5_EDMAC0_BASE+RA6M5_ETHD_RFOCR_OFFSET)
#define RA6M5_ETHD_IOSR          (RA6M5_EDMAC0_BASE+RA6M5_ETHD_IOSR_OFFSET)
#define RA6M5_ETHD_FCFTR         (RA6M5_EDMAC0_BASE+RA6M5_ETHD_FCFTR_OFFSET)
#define RA6M5_ETHD_RPADIR        (RA6M5_EDMAC0_BASE+RA6M5_ETHD_RPADIR_OFFSET)
#define RA6M5_ETHD_TRIMD         (RA6M5_EDMAC0_BASE+RA6M5_ETHD_TRIMD_OFFSET)
#define RA6M5_ETHD_RBWAR         (RA6M5_EDMAC0_BASE+RA6M5_ETHD_RBWAR_OFFSET)
#define RA6M5_ETHD_RDFAR         (RA6M5_EDMAC0_BASE+RA6M5_ETHD_RDFAR_OFFSET)
#define RA6M5_ETHD_TBRAR         (RA6M5_EDMAC0_BASE+RA6M5_ETHD_TBRAR_OFFSET)
#define RA6M5_ETHD_TDFAR         (RA6M5_EDMAC0_BASE+RA6M5_ETHD_TDFAR_OFFSET)

/* MPC (Multifunction pin controller) Registers for Ethernet */

#define RA6M5_MPC_PFENET         (0x40080D00)

/* Register Bit-Field Definitions */

/* MAC Registers */

#define ETH_ECSR_LCHNG     (1 << 2)     /* Bit 2: Link Signal Change Flag */

/* Bit 2: LINK Signal Change Interrupt enable/disable */

#define ETH_ECSIPR_LCHNGIP (1 << 2)
#define ETH_ECMR_CLR       (0x00000000)

/* Clear all ETHERC status BFR, PSRTO, LCHNG, MPD, ICD */

#define ETH_ECSR_CLR       (0x00000037)
#define ETH_ECMR_RE        (1 << 6) /* Transmit function is enabled */
#define ETH_ECMR_TE        (1 << 5) /* Receive function is enabled */
#define ETH_ECMR_DM        (1 << 1) /* Duplex Mode */
#define ETH_ECMR_RTM       (1 << 2) /* Bit Rate */

/* Bit 4:0:Interpacket Gap 96 bit time (initial value) */

#define ETH_IPGR_IPG_INITIAL     (0x00000014)

/* Receive Frame Maximum Length */

#define ETH_RFLR_RFL             (1518)

/* EDMA Registers */

/* Bit 22: ETHERC status interrupt request is enable/disabled. */

#define ETHD_EDMR_SWR            (1 << 0)

/* Bit 6: Big Endian Mode/Little Endian Mode */

#define ETHD_EDMR_DE             (1 << 6)

/* Clear all EDMAC status bits */

#define ETHD_EESR_EDMAC          (0x47ff0f9f)

/* Frame transfer Complete status Flag check */

#define ETHD_EESR_TC         (1 << 21)

/* ETHERC/EDMAC Status Register Source Flag */

#define ETHD_EESR_ECI        (1 << 22)
#define ETHD_EESR_FR         (1 << 18)   /* Frame Receive Flag */

/* Frame Receive Interrupt Request Enable */

#define ETHD_EESIPR_FRIP     (1 << 18)

/* Frame Transfer Complete Interrupt Request Enable */

#define ETHD_EESIPR_TCIP     (1 << 21)

/* ETHERC/EDMAC Status Register Source Interrupt Request Enable */

#define ETHD_EESIPR_ECIIP    (1 << 22)

/* ETHERC/EDMAC Write-Back Complete Interrupt Request Enable */

#define ETHD_EESIPR_TWBIP    (1 << 30)

/* Bit 0:10: Transmit FIFO Threshold */

#define ETHD_TFTR_TFT        (0x00000000)

/* Bit: 20: Transmit Descriptor Empty Flag */

#define ETHD_EESR_TDE        (1 << 20)

/* Ether PSR register */

#define ETH_PSR_LMON         (1)

/* EDMAC Transmit Request Register's bit */

#define ETHD_EDRRR_TR        (1) /* Transmit Request */

/* EDMAC Receive Request Register's bit */

#define ETHD_EDRRR_RR        (1) /* Receive descriptor read,
                               * and receive function is enabled
                               */

/* Transmit Interrupt Setting Register's bit */

#define ETHD_TRIMD_TIS        (1)      /* Transmit Interrupt is enabled */
#define ETHD_TRIMD_TIM        (1 << 4) /* Write-back complete interrupt mode */

/* Receive Method Control Register's bit */

/* Receive Method Control Register's bit */

#define ETHD_RMCR_RNR        (1) /* EDRRR.RR bit (receive request bit) is not
                                  * set to 0 when one frame has been received
                                                                  */

/* FIFO Depth Register's bit */

#define ETHD_FDR_RFD         (7)       /* Receive FIFO Depth */
#define ETHD_FDR_TFD         (7 << 8)  /* Transmit FIFO Depth */

/* ETHERC/EDMAC Transmit/Receive Status Copy Enable Register's bit */

#define ETHD_TRSCER_RRFCE     (1 << 4)  /* RRF Flag Copy Enable */
#define ETHD_TRSCER_RMAFCE    (1 << 7)  /* RMAF Flag Copy Enable */

/* Broadcast Frame Receive Count Setting Register's field */

#define ETH_BCFRR_BCF    (0x0000)  /* Broadcast Frame Continuous Receive Count Setting */

/* PHY Interface Register's bit and values */

#define ETH_PIR_MDC              (1)          /* MII/RMII Management Data Clock */
#define ETH_PIR_MMD              (1 << 1)     /* MII/RMII Management Mode */
#define ETH_PIR_MDO              (1 << 2)     /* MII/RMII Management Data-Out */
#define ETH_PIR_MDI              (1 << 3)     /* MII/RMII Management Data-In */

#define ETH_PIR_RESET_ALL        (0x00000000) /* Reset All Flags of PIR */
#define ETH_PIR_SET_MDC          (0x00000001) /* Setting MDC of PIR */
#define ETH_PIR_SET_MMD          (0x00000002) /* Setting MMD of PIR */
#define ETH_PIR_SET_MMD_MDC      (0x00000003) /* Setting MMD and MDC */
#define ETH_PIR_SET_MDO_MMD      (0x00000006) /* Setting MDO and MMD */
#define ETH_PIR_SET_MDO_MMD_MDC  (0x00000007) /* Setting MDO, MMD and MDC */

/* Ethernet Control Register's bit and value */

#define ETH_PFENET_MII_MODE      (0x10)
#define ETH_PFENET_RMII_MODE     (0x00)

#endif /* __ARCH_ARM_SRC_RA6M5_HARDWARE_RA6M5_ETH_H */
