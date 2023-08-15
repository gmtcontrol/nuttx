/****************************************************************************
 * arch/arm/src/ra6m5/hardware/ra6m5_sci.h
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

#ifndef __ARCH_ARM_SRC_RA6M5_HARDWARE_RA6M5_SCI_H
#define __ARCH_ARM_SRC_RA6M5_HARDWARE_RA6M5_SCI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define RA6M5_SCI_SMR_OFFSET    0x0000  /* Serial Mode Register */
#define RA6M5_SCI_BRR_OFFSET    0x0001  /* Bit Rate Register */
#define RA6M5_SCI_SCR_OFFSET    0x0002  /* Serial Control Register */
#define RA6M5_SCI_TDR_OFFSET    0x0003  /* Transmit Data Register */
#define RA6M5_SCI_SSR_OFFSET    0x0004  /* Serial Status Register */
#define RA6M5_SCI_RDR_OFFSET    0x0005  /* Receive Data Register */
#define RA6M5_SCI_SCMR_OFFSET   0x0006  /* Smart Card Mode Register */
#define RA6M5_SCI_SEMR_OFFSET   0x0007  /* Serial Extended Mode Register */
#define RA6M5_SCI_SNFR_OFFSET   0x0008  /* Noise Filter Setting Register */
#define RA6M5_SCI_SIMR1_OFFSET  0x0009  /* I2C Mode Register 1 */
#define RA6M5_SCI_SIMR2_OFFSET  0x000A  /* I2C Mode Register 2 */
#define RA6M5_SCI_SIMR3_OFFSET  0x000B  /* I2C Mode Register 3 */
#define RA6M5_SCI_SISR_OFFSET   0x000C  /* I2C Status Register */
#define RA6M5_SCI_SPMR_OFFSET   0x000D  /* SPI Mode Register */
#define RA6M5_SCI_FTDRH_OFFSET  0x000E  /* Transmit FIFO Data Register H */
#define RA6M5_SCI_FTDRL_OFFSET  0x000F  /* Transmit FIFO Data Register L */
#define RA6M5_SCI_FRDRH_OFFSET  0x0010  /* Receive FIFO Data Register H */
#define RA6M5_SCI_FRDRL_OFFSET  0x0011  /* Receive FIFO Data Register L */
#define RA6M5_SCI_MDDR_OFFSET   0x0012  /* Modulation Duty Register */
#define RA6M5_SCI_DCCR_OFFSET   0x0013  /* Data Compare Match Control Register */
#define RA6M5_SCI_FCR_OFFSET    0x0014  /* FIFO Control Register */
#define RA6M5_SCI_FDR_OFFSET    0x0016  /* FIFO Data Count Register */
#define RA6M5_SCI_LSR_OFFSET    0x0018  /* Line Status Register */
#define RA6M5_SCI_CDR_OFFSET    0x001A  /* Compare Match Data Register */
#define RA6M5_SCI_SPTR_OFFSET   0x001C  /* Serial Port Register */
#define RA6M5_SCI_ACTR_OFFSET   0x001D  /* Adjustment Communication Timing Register */
#define RA6M5_SCI_ESMER_OFFSET  0x0020  /* Extended Serial Module Enable Register */
#define RA6M5_SCI_CR0_OFFSET    0x0021  /* Control Register 0 */
#define RA6M5_SCI_CR1_OFFSET    0x0022  /* Control Register 1 */
#define RA6M5_SCI_CR2_OFFSET    0x0023  /* Control Register 2 */
#define RA6M5_SCI_CR3_OFFSET    0x0024  /* Control Register 3 */
#define RA6M5_SCI_PCR_OFFSET    0x0025  /* Port Control Register */
#define RA6M5_SCI_ICR_OFFSET    0x0026  /* Interrupt Control Register */
#define RA6M5_SCI_STR_OFFSET    0x0027  /* Status Register */
#define RA6M5_SCI_STCR_OFFSET   0x0028  /* Status Clear Register */
#define RA6M5_SCI_CF0DR_OFFSET  0x0029  /* Control Field 0 Data Register */
#define RA6M5_SCI_CF0CR_OFFSET  0x002A  /* Control Field 0 Compare Enable Register */
#define RA6M5_SCI_CF0RR_OFFSET  0x002B  /* Control Field 0 Receive Data Register */
#define RA6M5_SCI_PCF1DR_OFFSET 0x002C  /* Primary Control Field 1 Data Register */
#define RA6M5_SCI_SCF1DR_OFFSET 0x002D  /* Secondary Control Field 1 Data Register */
#define RA6M5_SCI_CF1CR_OFFSET  0x002E  /* Control Field 1 Compare Enable Register */
#define RA6M5_SCI_CF1RR_OFFSET  0x002F  /* Control Field 1 Receive Data Register */
#define RA6M5_SCI_TCR_OFFSET    0x0030  /* Timer Control Register */
#define RA6M5_SCI_TMR_OFFSET    0x0031  /* Timer Mode Register */
#define RA6M5_SCI_TPRE_OFFSET   0x0032  /* Timer Prescaler Register */
#define RA6M5_SCI_TCNT_OFFSET   0x0033  /* Timer Count Register */


/* Register Addresses *******************************************************/

#if RA6M5_NSCI > 0
#  define RA6M5_SCI0_SMR        (RA6M5_SCI0_BASE + RA6M5_SCI_SMR_OFFSET)
#  define RA6M5_SCI0_BRR        (RA6M5_SCI0_BASE + RA6M5_SCI_BRR_OFFSET)
#  define RA6M5_SCI0_SCR        (RA6M5_SCI0_BASE + RA6M5_SCI_SCR_OFFSET)
#  define RA6M5_SCI0_TDR        (RA6M5_SCI0_BASE + RA6M5_SCI_TDR_OFFSET)
#  define RA6M5_SCI0_SSR        (RA6M5_SCI0_BASE + RA6M5_SCI_SSR_OFFSET)
#  define RA6M5_SCI0_RDR        (RA6M5_SCI0_BASE + RA6M5_SCI_RDR_OFFSET)
#  define RA6M5_SCI0_SNFR       (RA6M5_SCI0_BASE + RA6M5_SCI_SNFR_OFFSET)
#  define RA6M5_SCI0_FTDRH      (RA6M5_SCI0_BASE + RA6M5_SCI_FTDRH_OFFSET)
#  define RA6M5_SCI0_FTDRL      (RA6M5_SCI0_BASE + RA6M5_SCI_FTDRL_OFFSET)
#  define RA6M5_SCI0_FRDRH      (RA6M5_SCI0_BASE + RA6M5_SCI_FRDRH_OFFSET)
#  define RA6M5_SCI0_FRDRL      (RA6M5_SCI0_BASE + RA6M5_SCI_FRDRL_OFFSET)
#  define RA6M5_SCI0_FCR        (RA6M5_SCI0_BASE + RA6M5_SCI_FCR_OFFSET)
#  define RA6M5_SCI0_FDR        (RA6M5_SCI0_BASE + RA6M5_SCI_FDR_OFFSET)
#  define RA6M5_SCI0_LSR        (RA6M5_SCI0_BASE + RA6M5_SCI_LSR_OFFSET)
#  define RA6M5_SCI0_SPTR       (RA6M5_SCI0_BASE + RA6M5_SCI_SPTR_OFFSET)
#  define RA6M5_SCI0_CR0        (RA6M5_SCI0_BASE + RA6M5_SCI_CR0_OFFSET)
#  define RA6M5_SCI0_CR1        (RA6M5_SCI0_BASE + RA6M5_SCI_CR1_OFFSET)
#  define RA6M5_SCI0_CR2        (RA6M5_SCI0_BASE + RA6M5_SCI_CR2_OFFSET)
#  define RA6M5_SCI0_CR3        (RA6M5_SCI0_BASE + RA6M5_SCI_CR3_OFFSET)
#  define RA6M5_SCI0_PCR        (RA6M5_SCI0_BASE + RA6M5_SCI_PCR_OFFSET)
#  define RA6M5_SCI0_ICR        (RA6M5_SCI0_BASE + RA6M5_SCI_ICR_OFFSET)
#  define RA6M5_SCI0_STR        (RA6M5_SCI0_BASE + RA6M5_SCI_STR_OFFSET)
#  define RA6M5_SCI0_STCR       (RA6M5_SCI0_BASE + RA6M5_SCI_STCR_OFFSET)
#endif

#if RA6M5_NSCI > 1
#  define RA6M5_SCI1_SMR        (RA6M5_SCI1_BASE + RA6M5_SCI_SMR_OFFSET)
#  define RA6M5_SCI1_BRR        (RA6M5_SCI1_BASE + RA6M5_SCI_BRR_OFFSET)
#  define RA6M5_SCI1_SCR        (RA6M5_SCI1_BASE + RA6M5_SCI_SCR_OFFSET)
#  define RA6M5_SCI1_TDR        (RA6M5_SCI1_BASE + RA6M5_SCI_TDR_OFFSET)
#  define RA6M5_SCI1_SSR        (RA6M5_SCI1_BASE + RA6M5_SCI_SSR_OFFSET)
#  define RA6M5_SCI1_RDR        (RA6M5_SCI1_BASE + RA6M5_SCI_RDR_OFFSET)
#  define RA6M5_SCI1_SNFR       (RA6M5_SCI1_BASE + RA6M5_SCI_SNFR_OFFSET)
#  define RA6M5_SCI1_FTDRH      (RA6M5_SCI1_BASE + RA6M5_SCI_FTDRH_OFFSET)
#  define RA6M5_SCI1_FTDRL      (RA6M5_SCI1_BASE + RA6M5_SCI_FTDRL_OFFSET)
#  define RA6M5_SCI1_FRDRH      (RA6M5_SCI1_BASE + RA6M5_SCI_FRDRH_OFFSET)
#  define RA6M5_SCI1_FRDRL      (RA6M5_SCI1_BASE + RA6M5_SCI_FRDRL_OFFSET)
#  define RA6M5_SCI1_FCR        (RA6M5_SCI1_BASE + RA6M5_SCI_FCR_OFFSET)
#  define RA6M5_SCI1_FDR        (RA6M5_SCI1_BASE + RA6M5_SCI_FDR_OFFSET)
#  define RA6M5_SCI1_LSR        (RA6M5_SCI1_BASE + RA6M5_SCI_LSR_OFFSET)
#  define RA6M5_SCI1_SPTR       (RA6M5_SCI1_BASE + RA6M5_SCI_SPTR_OFFSET)
#  define RA6M5_SCI1_CR0        (RA6M5_SCI1_BASE + RA6M5_SCI_CR0_OFFSET)
#  define RA6M5_SCI1_CR1        (RA6M5_SCI1_BASE + RA6M5_SCI_CR1_OFFSET)
#  define RA6M5_SCI1_CR2        (RA6M5_SCI1_BASE + RA6M5_SCI_CR2_OFFSET)
#  define RA6M5_SCI1_CR3        (RA6M5_SCI1_BASE + RA6M5_SCI_CR3_OFFSET)
#  define RA6M5_SCI1_PCR        (RA6M5_SCI1_BASE + RA6M5_SCI_PCR_OFFSET)
#  define RA6M5_SCI1_ICR        (RA6M5_SCI1_BASE + RA6M5_SCI_ICR_OFFSET)
#  define RA6M5_SCI1_STR        (RA6M5_SCI1_BASE + RA6M5_SCI_STR_OFFSET)
#  define RA6M5_SCI1_STCR       (RA6M5_SCI1_BASE + RA6M5_SCI_STCR_OFFSET)
#endif

#if RA6M5_NSCI > 2
#  define RA6M5_SCI2_SMR        (RA6M5_SCI2_BASE + RA6M5_SCI_SMR_OFFSET)
#  define RA6M5_SCI2_BRR        (RA6M5_SCI2_BASE + RA6M5_SCI_BRR_OFFSET)
#  define RA6M5_SCI2_SCR        (RA6M5_SCI2_BASE + RA6M5_SCI_SCR_OFFSET)
#  define RA6M5_SCI2_TDR        (RA6M5_SCI2_BASE + RA6M5_SCI_TDR_OFFSET)
#  define RA6M5_SCI2_SSR        (RA6M5_SCI2_BASE + RA6M5_SCI_SSR_OFFSET)
#  define RA6M5_SCI2_RDR        (RA6M5_SCI2_BASE + RA6M5_SCI_RDR_OFFSET)
#  define RA6M5_SCI2_SNFR       (RA6M5_SCI2_BASE + RA6M5_SCI_SNFR_OFFSET)
#  define RA6M5_SCI2_FTDRH      (RA6M5_SCI2_BASE + RA6M5_SCI_FTDRH_OFFSET)
#  define RA6M5_SCI2_FTDRL      (RA6M5_SCI2_BASE + RA6M5_SCI_FTDRL_OFFSET)
#  define RA6M5_SCI2_FRDRH      (RA6M5_SCI2_BASE + RA6M5_SCI_FRDRH_OFFSET)
#  define RA6M5_SCI2_FRDRL      (RA6M5_SCI2_BASE + RA6M5_SCI_FRDRL_OFFSET)
#  define RA6M5_SCI2_FCR        (RA6M5_SCI2_BASE + RA6M5_SCI_FCR_OFFSET)
#  define RA6M5_SCI2_FDR        (RA6M5_SCI2_BASE + RA6M5_SCI_FDR_OFFSET)
#  define RA6M5_SCI2_LSR        (RA6M5_SCI2_BASE + RA6M5_SCI_LSR_OFFSET)
#  define RA6M5_SCI2_SPTR       (RA6M5_SCI2_BASE + RA6M5_SCI_SPTR_OFFSET)
#  define RA6M5_SCI2_CR0        (RA6M5_SCI2_BASE + RA6M5_SCI_CR0_OFFSET)
#  define RA6M5_SCI2_CR1        (RA6M5_SCI2_BASE + RA6M5_SCI_CR1_OFFSET)
#  define RA6M5_SCI2_CR2        (RA6M5_SCI2_BASE + RA6M5_SCI_CR2_OFFSET)
#  define RA6M5_SCI2_CR3        (RA6M5_SCI2_BASE + RA6M5_SCI_CR3_OFFSET)
#  define RA6M5_SCI2_PCR        (RA6M5_SCI2_BASE + RA6M5_SCI_PCR_OFFSET)
#  define RA6M5_SCI2_ICR        (RA6M5_SCI2_BASE + RA6M5_SCI_ICR_OFFSET)
#  define RA6M5_SCI2_STR        (RA6M5_SCI2_BASE + RA6M5_SCI_STR_OFFSET)
#  define RA6M5_SCI2_STCR       (RA6M5_SCI2_BASE + RA6M5_SCI_STCR_OFFSET)
#endif

#if RA6M5_NSCI > 3
#  define RA6M5_SCI3_SMR        (RA6M5_SCI3_BASE + RA6M5_SCI_SMR_OFFSET)
#  define RA6M5_SCI3_BRR        (RA6M5_SCI3_BASE + RA6M5_SCI_BRR_OFFSET)
#  define RA6M5_SCI3_SCR        (RA6M5_SCI3_BASE + RA6M5_SCI_SCR_OFFSET)
#  define RA6M5_SCI3_TDR        (RA6M5_SCI3_BASE + RA6M5_SCI_TDR_OFFSET)
#  define RA6M5_SCI3_SSR        (RA6M5_SCI3_BASE + RA6M5_SCI_SSR_OFFSET)
#  define RA6M5_SCI3_RDR        (RA6M5_SCI3_BASE + RA6M5_SCI_RDR_OFFSET)
#  define RA6M5_SCI3_SNFR       (RA6M5_SCI3_BASE + RA6M5_SCI_SNFR_OFFSET)
#  define RA6M5_SCI3_FTDRH      (RA6M5_SCI3_BASE + RA6M5_SCI_FTDRH_OFFSET)
#  define RA6M5_SCI3_FTDRL      (RA6M5_SCI3_BASE + RA6M5_SCI_FTDRL_OFFSET)
#  define RA6M5_SCI3_FRDRH      (RA6M5_SCI3_BASE + RA6M5_SCI_FRDRH_OFFSET)
#  define RA6M5_SCI3_FRDRL      (RA6M5_SCI3_BASE + RA6M5_SCI_FRDRL_OFFSET)
#  define RA6M5_SCI3_FCR        (RA6M5_SCI3_BASE + RA6M5_SCI_FCR_OFFSET)
#  define RA6M5_SCI3_FDR        (RA6M5_SCI3_BASE + RA6M5_SCI_FDR_OFFSET)
#  define RA6M5_SCI3_LSR        (RA6M5_SCI3_BASE + RA6M5_SCI_LSR_OFFSET)
#  define RA6M5_SCI3_SPTR       (RA6M5_SCI3_BASE + RA6M5_SCI_SPTR_OFFSET)
#  define RA6M5_SCI3_CR0        (RA6M5_SCI3_BASE + RA6M5_SCI_CR0_OFFSET)
#  define RA6M5_SCI3_CR1        (RA6M5_SCI3_BASE + RA6M5_SCI_CR1_OFFSET)
#  define RA6M5_SCI3_CR2        (RA6M5_SCI3_BASE + RA6M5_SCI_CR2_OFFSET)
#  define RA6M5_SCI3_CR3        (RA6M5_SCI3_BASE + RA6M5_SCI_CR3_OFFSET)
#  define RA6M5_SCI3_PCR        (RA6M5_SCI3_BASE + RA6M5_SCI_PCR_OFFSET)
#  define RA6M5_SCI3_ICR        (RA6M5_SCI3_BASE + RA6M5_SCI_ICR_OFFSET)
#  define RA6M5_SCI3_STR        (RA6M5_SCI3_BASE + RA6M5_SCI_STR_OFFSET)
#  define RA6M5_SCI3_STCR       (RA6M5_SCI3_BASE + RA6M5_SCI_STCR_OFFSET)
#endif

#if RA6M5_NSCI > 4
#  define RA6M5_SCI4_SMR        (RA6M5_SCI4_BASE + RA6M5_SCI_SMR_OFFSET)
#  define RA6M5_SCI4_BRR        (RA6M5_SCI4_BASE + RA6M5_SCI_BRR_OFFSET)
#  define RA6M5_SCI4_SCR        (RA6M5_SCI4_BASE + RA6M5_SCI_SCR_OFFSET)
#  define RA6M5_SCI4_TDR        (RA6M5_SCI4_BASE + RA6M5_SCI_TDR_OFFSET)
#  define RA6M5_SCI4_SSR        (RA6M5_SCI4_BASE + RA6M5_SCI_SSR_OFFSET)
#  define RA6M5_SCI4_RDR        (RA6M5_SCI4_BASE + RA6M5_SCI_RDR_OFFSET)
#  define RA6M5_SCI4_SNFR       (RA6M5_SCI4_BASE + RA6M5_SCI_SNFR_OFFSET)
#  define RA6M5_SCI4_FTDRH      (RA6M5_SCI4_BASE + RA6M5_SCI_FTDRH_OFFSET)
#  define RA6M5_SCI4_FTDRL      (RA6M5_SCI4_BASE + RA6M5_SCI_FTDRL_OFFSET)
#  define RA6M5_SCI4_FRDRH      (RA6M5_SCI4_BASE + RA6M5_SCI_FRDRH_OFFSET)
#  define RA6M5_SCI4_FRDRL      (RA6M5_SCI4_BASE + RA6M5_SCI_FRDRL_OFFSET)
#  define RA6M5_SCI4_FCR        (RA6M5_SCI4_BASE + RA6M5_SCI_FCR_OFFSET)
#  define RA6M5_SCI4_FDR        (RA6M5_SCI4_BASE + RA6M5_SCI_FDR_OFFSET)
#  define RA6M5_SCI4_LSR        (RA6M5_SCI4_BASE + RA6M5_SCI_LSR_OFFSET)
#  define RA6M5_SCI4_SPTR       (RA6M5_SCI4_BASE + RA6M5_SCI_SPTR_OFFSET)
#  define RA6M5_SCI4_CR0        (RA6M5_SCI4_BASE + RA6M5_SCI_CR0_OFFSET)
#  define RA6M5_SCI4_CR1        (RA6M5_SCI4_BASE + RA6M5_SCI_CR1_OFFSET)
#  define RA6M5_SCI4_CR2        (RA6M5_SCI4_BASE + RA6M5_SCI_CR2_OFFSET)
#  define RA6M5_SCI4_CR3        (RA6M5_SCI4_BASE + RA6M5_SCI_CR3_OFFSET)
#  define RA6M5_SCI4_PCR        (RA6M5_SCI4_BASE + RA6M5_SCI_PCR_OFFSET)
#  define RA6M5_SCI4_ICR        (RA6M5_SCI4_BASE + RA6M5_SCI_ICR_OFFSET)
#  define RA6M5_SCI4_STR        (RA6M5_SCI4_BASE + RA6M5_SCI_STR_OFFSET)
#  define RA6M5_SCI4_STCR       (RA6M5_SCI4_BASE + RA6M5_SCI_STCR_OFFSET)
#endif

#if RA6M5_NSCI > 5
#  define RA6M5_SCI5_SMR        (RA6M5_SCI5_BASE + RA6M5_SCI_SMR_OFFSET)
#  define RA6M5_SCI5_BRR        (RA6M5_SCI5_BASE + RA6M5_SCI_BRR_OFFSET)
#  define RA6M5_SCI5_SCR        (RA6M5_SCI5_BASE + RA6M5_SCI_SCR_OFFSET)
#  define RA6M5_SCI5_TDR        (RA6M5_SCI5_BASE + RA6M5_SCI_TDR_OFFSET)
#  define RA6M5_SCI5_SSR        (RA6M5_SCI5_BASE + RA6M5_SCI_SSR_OFFSET)
#  define RA6M5_SCI5_RDR        (RA6M5_SCI5_BASE + RA6M5_SCI_RDR_OFFSET)
#  define RA6M5_SCI5_SNFR       (RA6M5_SCI5_BASE + RA6M5_SCI_SNFR_OFFSET)
#  define RA6M5_SCI5_FTDRH      (RA6M5_SCI5_BASE + RA6M5_SCI_FTDRH_OFFSET)
#  define RA6M5_SCI5_FTDRL      (RA6M5_SCI5_BASE + RA6M5_SCI_FTDRL_OFFSET)
#  define RA6M5_SCI5_FRDRH      (RA6M5_SCI5_BASE + RA6M5_SCI_FRDRH_OFFSET)
#  define RA6M5_SCI5_FRDRL      (RA6M5_SCI5_BASE + RA6M5_SCI_FRDRL_OFFSET)
#  define RA6M5_SCI5_FCR        (RA6M5_SCI5_BASE + RA6M5_SCI_FCR_OFFSET)
#  define RA6M5_SCI5_FDR        (RA6M5_SCI5_BASE + RA6M5_SCI_FDR_OFFSET)
#  define RA6M5_SCI5_LSR        (RA6M5_SCI5_BASE + RA6M5_SCI_LSR_OFFSET)
#  define RA6M5_SCI5_SPTR       (RA6M5_SCI5_BASE + RA6M5_SCI_SPTR_OFFSET)
#  define RA6M5_SCI5_CR0        (RA6M5_SCI5_BASE + RA6M5_SCI_CR0_OFFSET)
#  define RA6M5_SCI5_CR1        (RA6M5_SCI5_BASE + RA6M5_SCI_CR1_OFFSET)
#  define RA6M5_SCI5_CR2        (RA6M5_SCI5_BASE + RA6M5_SCI_CR2_OFFSET)
#  define RA6M5_SCI5_CR3        (RA6M5_SCI5_BASE + RA6M5_SCI_CR3_OFFSET)
#  define RA6M5_SCI5_PCR        (RA6M5_SCI5_BASE + RA6M5_SCI_PCR_OFFSET)
#  define RA6M5_SCI5_ICR        (RA6M5_SCI5_BASE + RA6M5_SCI_ICR_OFFSET)
#  define RA6M5_SCI5_STR        (RA6M5_SCI5_BASE + RA6M5_SCI_STR_OFFSET)
#  define RA6M5_SCI5_STCR       (RA6M5_SCI5_BASE + RA6M5_SCI_STCR_OFFSET)
#endif

#if RA6M5_NSCI > 6
#  define RA6M5_SCI6_SMR        (RA6M5_SCI6_BASE + RA6M5_SCI_SMR_OFFSET)
#  define RA6M5_SCI6_BRR        (RA6M5_SCI6_BASE + RA6M5_SCI_BRR_OFFSET)
#  define RA6M5_SCI6_SCR        (RA6M5_SCI6_BASE + RA6M5_SCI_SCR_OFFSET)
#  define RA6M5_SCI6_TDR        (RA6M5_SCI6_BASE + RA6M5_SCI_TDR_OFFSET)
#  define RA6M5_SCI6_SSR        (RA6M5_SCI6_BASE + RA6M5_SCI_SSR_OFFSET)
#  define RA6M5_SCI6_RDR        (RA6M5_SCI6_BASE + RA6M5_SCI_RDR_OFFSET)
#  define RA6M5_SCI6_SNFR       (RA6M5_SCI6_BASE + RA6M5_SCI_SNFR_OFFSET)
#  define RA6M5_SCI6_FTDRH      (RA6M5_SCI6_BASE + RA6M5_SCI_FTDRH_OFFSET)
#  define RA6M5_SCI6_FTDRL      (RA6M5_SCI6_BASE + RA6M5_SCI_FTDRL_OFFSET)
#  define RA6M5_SCI6_FRDRH      (RA6M5_SCI6_BASE + RA6M5_SCI_FRDRH_OFFSET)
#  define RA6M5_SCI6_FRDRL      (RA6M5_SCI6_BASE + RA6M5_SCI_FRDRL_OFFSET)
#  define RA6M5_SCI6_FCR        (RA6M5_SCI6_BASE + RA6M5_SCI_FCR_OFFSET)
#  define RA6M5_SCI6_FDR        (RA6M5_SCI6_BASE + RA6M5_SCI_FDR_OFFSET)
#  define RA6M5_SCI6_LSR        (RA6M5_SCI6_BASE + RA6M5_SCI_LSR_OFFSET)
#  define RA6M5_SCI6_SPTR       (RA6M5_SCI6_BASE + RA6M5_SCI_SPTR_OFFSET)
#  define RA6M5_SCI6_CR0        (RA6M5_SCI6_BASE + RA6M5_SCI_CR0_OFFSET)
#  define RA6M5_SCI6_CR1        (RA6M5_SCI6_BASE + RA6M5_SCI_CR1_OFFSET)
#  define RA6M5_SCI6_CR2        (RA6M5_SCI6_BASE + RA6M5_SCI_CR2_OFFSET)
#  define RA6M5_SCI6_CR3        (RA6M5_SCI6_BASE + RA6M5_SCI_CR3_OFFSET)
#  define RA6M5_SCI6_PCR        (RA6M5_SCI6_BASE + RA6M5_SCI_PCR_OFFSET)
#  define RA6M5_SCI6_ICR        (RA6M5_SCI6_BASE + RA6M5_SCI_ICR_OFFSET)
#  define RA6M5_SCI6_STR        (RA6M5_SCI6_BASE + RA6M5_SCI_STR_OFFSET)
#  define RA6M5_SCI6_STCR       (RA6M5_SCI6_BASE + RA6M5_SCI_STCR_OFFSET)
#endif

#if RA6M5_NSCI > 7
#  define RA6M5_SCI7_SMR        (RA6M5_SCI7_BASE + RA6M5_SCI_SMR_OFFSET)
#  define RA6M5_SCI7_BRR        (RA6M5_SCI7_BASE + RA6M5_SCI_BRR_OFFSET)
#  define RA6M5_SCI7_SCR        (RA6M5_SCI7_BASE + RA6M5_SCI_SCR_OFFSET)
#  define RA6M5_SCI7_TDR        (RA6M5_SCI7_BASE + RA6M5_SCI_TDR_OFFSET)
#  define RA6M5_SCI7_SSR        (RA6M5_SCI7_BASE + RA6M5_SCI_SSR_OFFSET)
#  define RA6M5_SCI7_RDR        (RA6M5_SCI7_BASE + RA6M5_SCI_RDR_OFFSET)
#  define RA6M5_SCI7_SNFR       (RA6M5_SCI7_BASE + RA6M5_SCI_SNFR_OFFSET)
#  define RA6M5_SCI7_FTDRH      (RA6M5_SCI7_BASE + RA6M5_SCI_FTDRH_OFFSET)
#  define RA6M5_SCI7_FTDRL      (RA6M5_SCI7_BASE + RA6M5_SCI_FTDRL_OFFSET)
#  define RA6M5_SCI7_FRDRH      (RA6M5_SCI7_BASE + RA6M5_SCI_FRDRH_OFFSET)
#  define RA6M5_SCI7_FRDRL      (RA6M5_SCI7_BASE + RA6M5_SCI_FRDRL_OFFSET)
#  define RA6M5_SCI7_FCR        (RA6M5_SCI7_BASE + RA6M5_SCI_FCR_OFFSET)
#  define RA6M5_SCI7_FDR        (RA6M5_SCI7_BASE + RA6M5_SCI_FDR_OFFSET)
#  define RA6M5_SCI7_LSR        (RA6M5_SCI7_BASE + RA6M5_SCI_LSR_OFFSET)
#  define RA6M5_SCI7_SPTR       (RA6M5_SCI7_BASE + RA6M5_SCI_SPTR_OFFSET)
#  define RA6M5_SCI7_CR0        (RA6M5_SCI7_BASE + RA6M5_SCI_CR0_OFFSET)
#  define RA6M5_SCI7_CR1        (RA6M5_SCI7_BASE + RA6M5_SCI_CR1_OFFSET)
#  define RA6M5_SCI7_CR2        (RA6M5_SCI7_BASE + RA6M5_SCI_CR2_OFFSET)
#  define RA6M5_SCI7_CR3        (RA6M5_SCI7_BASE + RA6M5_SCI_CR3_OFFSET)
#  define RA6M5_SCI7_PCR        (RA6M5_SCI7_BASE + RA6M5_SCI_PCR_OFFSET)
#  define RA6M5_SCI7_ICR        (RA6M5_SCI7_BASE + RA6M5_SCI_ICR_OFFSET)
#  define RA6M5_SCI7_STR        (RA6M5_SCI7_BASE + RA6M5_SCI_STR_OFFSET)
#  define RA6M5_SCI7_STCR       (RA6M5_SCI7_BASE + RA6M5_SCI_STCR_OFFSET)
#endif

#if RA6M5_NSCI > 8
#  define RA6M5_SCI8_SMR        (RA6M5_SCI8_BASE + RA6M5_SCI_SMR_OFFSET)
#  define RA6M5_SCI8_BRR        (RA6M5_SCI8_BASE + RA6M5_SCI_BRR_OFFSET)
#  define RA6M5_SCI8_SCR        (RA6M5_SCI8_BASE + RA6M5_SCI_SCR_OFFSET)
#  define RA6M5_SCI8_TDR        (RA6M5_SCI8_BASE + RA6M5_SCI_TDR_OFFSET)
#  define RA6M5_SCI8_SSR        (RA6M5_SCI8_BASE + RA6M5_SCI_SSR_OFFSET)
#  define RA6M5_SCI8_RDR        (RA6M5_SCI8_BASE + RA6M5_SCI_RDR_OFFSET)
#  define RA6M5_SCI8_SNFR       (RA6M5_SCI8_BASE + RA6M5_SCI_SNFR_OFFSET)
#  define RA6M5_SCI8_FTDRH      (RA6M5_SCI8_BASE + RA6M5_SCI_FTDRH_OFFSET)
#  define RA6M5_SCI8_FTDRL      (RA6M5_SCI8_BASE + RA6M5_SCI_FTDRL_OFFSET)
#  define RA6M5_SCI8_FRDRH      (RA6M5_SCI8_BASE + RA6M5_SCI_FRDRH_OFFSET)
#  define RA6M5_SCI8_FRDRL      (RA6M5_SCI8_BASE + RA6M5_SCI_FRDRL_OFFSET)
#  define RA6M5_SCI8_FCR        (RA6M5_SCI8_BASE + RA6M5_SCI_FCR_OFFSET)
#  define RA6M5_SCI8_FDR        (RA6M5_SCI8_BASE + RA6M5_SCI_FDR_OFFSET)
#  define RA6M5_SCI8_LSR        (RA6M5_SCI8_BASE + RA6M5_SCI_LSR_OFFSET)
#  define RA6M5_SCI8_SPTR       (RA6M5_SCI8_BASE + RA6M5_SCI_SPTR_OFFSET)
#  define RA6M5_SCI8_CR0        (RA6M5_SCI8_BASE + RA6M5_SCI_CR0_OFFSET)
#  define RA6M5_SCI8_CR1        (RA6M5_SCI8_BASE + RA6M5_SCI_CR1_OFFSET)
#  define RA6M5_SCI8_CR2        (RA6M5_SCI8_BASE + RA6M5_SCI_CR2_OFFSET)
#  define RA6M5_SCI8_CR3        (RA6M5_SCI8_BASE + RA6M5_SCI_CR3_OFFSET)
#  define RA6M5_SCI8_PCR        (RA6M5_SCI8_BASE + RA6M5_SCI_PCR_OFFSET)
#  define RA6M5_SCI8_ICR        (RA6M5_SCI8_BASE + RA6M5_SCI_ICR_OFFSET)
#  define RA6M5_SCI8_STR        (RA6M5_SCI8_BASE + RA6M5_SCI_STR_OFFSET)
#  define RA6M5_SCI8_STCR       (RA6M5_SCI8_BASE + RA6M5_SCI_STCR_OFFSET)
#endif

#if RA6M5_NSCI > 9
#  define RA6M5_SCI9_SMR        (RA6M5_SCI9_BASE + RA6M5_SCI_SMR_OFFSET)
#  define RA6M5_SCI9_BRR        (RA6M5_SCI9_BASE + RA6M5_SCI_BRR_OFFSET)
#  define RA6M5_SCI9_SCR        (RA6M5_SCI9_BASE + RA6M5_SCI_SCR_OFFSET)
#  define RA6M5_SCI9_TDR        (RA6M5_SCI9_BASE + RA6M5_SCI_TDR_OFFSET)
#  define RA6M5_SCI9_SSR        (RA6M5_SCI9_BASE + RA6M5_SCI_SSR_OFFSET)
#  define RA6M5_SCI9_RDR        (RA6M5_SCI9_BASE + RA6M5_SCI_RDR_OFFSET)
#  define RA6M5_SCI9_SNFR       (RA6M5_SCI9_BASE + RA6M5_SCI_SNFR_OFFSET)
#  define RA6M5_SCI9_FTDRH      (RA6M5_SCI9_BASE + RA6M5_SCI_FTDRH_OFFSET)
#  define RA6M5_SCI9_FTDRL      (RA6M5_SCI9_BASE + RA6M5_SCI_FTDRL_OFFSET)
#  define RA6M5_SCI9_FRDRH      (RA6M5_SCI9_BASE + RA6M5_SCI_FRDRH_OFFSET)
#  define RA6M5_SCI9_FRDRL      (RA6M5_SCI9_BASE + RA6M5_SCI_FRDRL_OFFSET)
#  define RA6M5_SCI9_FCR        (RA6M5_SCI9_BASE + RA6M5_SCI_FCR_OFFSET)
#  define RA6M5_SCI9_FDR        (RA6M5_SCI9_BASE + RA6M5_SCI_FDR_OFFSET)
#  define RA6M5_SCI9_LSR        (RA6M5_SCI9_BASE + RA6M5_SCI_LSR_OFFSET)
#  define RA6M5_SCI9_SPTR       (RA6M5_SCI9_BASE + RA6M5_SCI_SPTR_OFFSET)
#  define RA6M5_SCI9_CR0        (RA6M5_SCI9_BASE + RA6M5_SCI_CR0_OFFSET)
#  define RA6M5_SCI9_CR1        (RA6M5_SCI9_BASE + RA6M5_SCI_CR1_OFFSET)
#  define RA6M5_SCI9_CR2        (RA6M5_SCI9_BASE + RA6M5_SCI_CR2_OFFSET)
#  define RA6M5_SCI9_CR3        (RA6M5_SCI9_BASE + RA6M5_SCI_CR3_OFFSET)
#  define RA6M5_SCI9_PCR        (RA6M5_SCI9_BASE + RA6M5_SCI_PCR_OFFSET)
#  define RA6M5_SCI9_ICR        (RA6M5_SCI9_BASE + RA6M5_SCI_ICR_OFFSET)
#  define RA6M5_SCI9_STR        (RA6M5_SCI9_BASE + RA6M5_SCI_STR_OFFSET)
#  define RA6M5_SCI9_STCR       (RA6M5_SCI9_BASE + RA6M5_SCI_STCR_OFFSET)
#endif

/* Register Bitfield Definitions ********************************************/

/* Serial Mode Register */

#define SCI_SMR_CKS_SHIFT       (0)         /* Bits 1: Clock Select */
#define SCI_SMR_CKS_MASK        (3 << SCI_SMR_CKS_SHIFT)
#  define SCI_SMR_CKS(n)        (((n) << SCI_SMR_CKS_SHIFT) & SCI_SMR_CKS_MASK)

#define SCI_SMR_MP              (1 << 2)    /* Bit 2: Multi-Processor Mode */
#define SCI_SMR_STOP            (1 << 3)    /* Bit 3: Stop Bit Length */
#define SCI_SMR_PM              (1 << 4)    /* Bit 4: Parity Mode */
#define SCI_SMR_PE              (1 << 5)    /* Bit 5: Parity Enable */
#define SCI_SMR_CHR             (1 << 6)    /* Bit 6: Character Length */
#define SCI_SMR_CM              (1 << 7)    /* Bit 7: Communication Mode */

/* Serial Control Register */

#define SCI_SCR_CKE_SHIFT       (0)         /* Bits 1: Clock Enable */
#define SCI_SCR_CKE_MASK        (3 << SCI_SCR_CKE_SHIFT)
#  define SCI_SCR_CKE(n)        (((n) << SCI_SCR_CKE_SHIFT) & SCI_SCR_CKE_MASK)

#define SCI_SCR_TEIE            (1 << 2)    /* Bit 2: Transmit End Interrupt Enable */
#define SCI_SCR_MPIE            (1 << 3)    /* Bit 3: Multi-Processor Interrupt Enable */
#define SCI_SCR_RE              (1 << 4)    /* Bit 4: Receive Enable */
#define SCI_SCR_TE              (1 << 5)    /* Bit 5: Transmit Enable */
#define SCI_SCR_RIE             (1 << 6)    /* Bit 6: Receive Interrupt Enable */
#define SCI_SCR_TIE             (1 << 7)    /* Bit 7: Transmit Interrupt Enable */

/* Serial Status Register */

#define SCI_SSR_MPBT            (1 << 0)    /* Bit 0: Multi-Processor Bit Transfer */
#define SCI_SSR_MPB             (1 << 1)    /* Bit 1: Multi-Processor */
#define SCI_SSR_TEND            (1 << 2)    /* Bit 2: Transmit End Flag */
#define SCI_SSR_PER             (1 << 3)    /* Bit 3: Parity Error Flag */
#define SCI_SSR_FER             (1 << 4)    /* Bit 4: Framing Error Flag */
#define SCI_SSR_ORER            (1 << 5)    /* Bit 5: Overrun Error Flag */
#define SCI_SSR_RDRF            (1 << 6)    /* Bit 6: Receive Data Full Flag */
#define SCI_SSR_TDRE            (1 << 7)    /* Bit 7: Transmit Data Empty Flag */

/* Smart Card Mode Register */

#define SCI_SCMR_RESET          (0xf2)      /* Reset value */

#define SCI_SCMR_SMIF           (1 << 0)    /* Bit 0: Smart Card Interface Mode Select */
#define SCI_SCMR_SINV           (1 << 2)    /* Bit 2: Transmitted/Received Data Invert */
#define SCI_SCMR_SDIR           (1 << 3)    /* Bit 3: Transmitted/Received Data Transfer Direction */
#define SCI_SCMR_CHR1           (1 << 4)    /* Bit 4: Character Length 1 */
#define SCI_SCMR_BCP2           (1 << 7)    /* Bit 7: Base Clock Pulse 2 */

/* Serial Extended Mode Register */

#define SCI_SEMR_BRME           (1 << 2)    /* Bit 2: Bit Rate Modulation Enable */

/* SPI Mode Register */

#define SCI_SPMR_CTSE           (1 << 1)    /* Bit 1: CTS Enable */
#define SCI_SPMR_CSTPEN         (1 << 3)    /* Bit 3: CTS external pin Enable */
#define SCI_SPMR_CKPOL          (1 << 6)    /* Bit 6: Clock Polarity Select */
#define SCI_SPMR_CKPH           (1 << 7)    /* Bit 7: Clock Phase Select */

/* Serial Port Register */

#define SCI_SPTR_RINV           (1 << 4)    /* Bit 4: RXD invert bit */
#define SCI_SPTR_TINV           (1 << 5)    /* Bit 5: TXD invert bit */

/* FIFO Control Register */

#define SCI_FCR_TTRG_SHIFT      (4)         /* Bits[7:4]: Transmit FIFO Data Trigger Number */
#define SCI_FCR_TTRG_MASK       (15 << SCI_FCR_TTRG_SHIFT)
#  define SCI_FCR_TTRG(n)       (((n) << SCI_FCR_TTRG_SHIFT) & SCI_FCR_TTRG_MASK)

#define SCI_FCR_RTRG_SHIFT      (8)         /* Bits[11:8]: Receive FIFO Data Trigger Number */
#define SCI_FCR_RTRG_MASK       (15 << SCI_FCR_RTRG_SHIFT)
#  define SCI_FCR_RTRG(n)       (((n) << SCI_FCR_RTRG_SHIFT) & SCI_FCR_RTRG_MASK)

#define SCI_FCR_RSTRG_SHIFT     (12)        /* Bits[15:12]: RTS Output Active Trigger Number Select */
#define SCI_FCR_RSTRG_MASK      (15 << SCI_FCR_RSTRG_SHIFT)
#  define SCI_FCR_RSTRG(n)      (((n) << SCI_FCR_RSTRG_SHIFT) & SCI_FCR_RSTRG_MASK)

/* Interrupt Control Register */

#define SCI_ICR_BFDIE           (1 << 0)    /* Bit 0: Break Field Low Width Detected Interrupt Enable */
#define SCI_ICR_CF0MIE          (1 << 1)    /* Bit 1: Control Field 0 Match Detected Interrupt Enable */


/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Macros
 ****************************************************************************/

#define RA6M5_SCI_STOP(n)       (1 << (22 + (9-(n))))

#endif /* __ARCH_ARM_SRC_RA6M5_HARDWARE_RA6M5_SCI_H */
