/****************************************************************************
 * arch/arm/src/ra6m5/hardware/r7fa6m5bx_iic.h
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

#ifndef __ARCH_ARM_SRC_RA6M5_HARDWARE_R7FA6M5BX_IIC_H
#define __ARCH_ARM_SRC_RA6M5_HARDWARE_R7FA6M5BX_IIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_RA6M5_R7FA6M5BX)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define RA6M5_IIC_ICCR1_OFFSET          0x0000  /* I2C Bus Control Register 1 */
#define RA6M5_IIC_ICCR2_OFFSET          0x0001  /* I2C Bus Control Register 2 */
#define RA6M5_IIC_ICMR1_OFFSET          0x0002  /* I2C Bus Mode Register 1 */
#define RA6M5_IIC_ICMR2_OFFSET          0x0003  /* I2C Bus Mode Register 2 */
#define RA6M5_IIC_ICMR3_OFFSET          0x0004  /* I2C Bus Mode Register 3 */
#define RA6M5_IIC_ICFER_OFFSET          0x0005  /* I2C Bus Function Enable Register */
#define RA6M5_IIC_ICSER_OFFSET          0x0006  /* I2C Bus Bus Status Enable Register */
#define RA6M5_IIC_ICIER_OFFSET          0x0007  /* I2C Bus Interrupt Enable Register */
#define RA6M5_IIC_ICSR1_OFFSET          0x0008  /* I2C Bus Status Register 1 */
#define RA6M5_IIC_ICSR2_OFFSET          0x0009  /* I2C Bus Status Register 2 */
#define RA6M5_IIC_SARL0_OFFSET          0x000A  /* I2C Slave Address Register L0 */
#define RA6M5_IIC_SARU0_OFFSET          0x000B  /* I2C Slave Address Register U0 */
#define RA6M5_IIC_SARL1_OFFSET          0x000C  /* I2C Slave Address Register L1 */
#define RA6M5_IIC_SARU1_OFFSET          0x000D  /* I2C Slave Address Register U1 */
#define RA6M5_IIC_SARL2_OFFSET          0x000E  /* I2C Slave Address Register L2 */
#define RA6M5_IIC_SARU2_OFFSET          0x000F  /* I2C Slave Address Register U2 */
#define RA6M5_IIC_ICBRL_OFFSET          0x0010  /* I2C Bus Bit Rate Low-Level Register */
#define RA6M5_IIC_ICBRH_OFFSET          0x0011  /* I2C Bus Bit Rate High-Level Register */
#define RA6M5_IIC_ICDRT_OFFSET          0x0012  /* I2C Bus Transmit Data Register */
#define RA6M5_IIC_ICDRR_OFFSET          0x0013  /* I2C Bus Receive Data Register */
#define RA6M5_IIC_ICWUR_OFFSET          0x0016  /* I2C Bus Wake Up Unit Register */
#define RA6M5_IIC_ICWUR1_OFFSET         0x0017  /* I2C Bus Wake Up Unit Register 2 */

/* Register Bitfield Definitions ********************************************/


/*  I2C Bus Control Register 1 */

#define IIC_ICCR1_SDAI              (1 << 0)    /* Bit 0: SDA Line Monitor */
#define IIC_ICCR1_SCLI              (1 << 1)    /* Bit 1: SCL Line Monitor */
#define IIC_ICCR1_SDAO              (1 << 2)    /* Bit 2: SDA Output Control/Monitor */
#define IIC_ICCR1_SCLO              (1 << 3)    /* Bit 3: SCL Output Control/Monitor */
#define IIC_ICCR1_SOWP              (1 << 4)    /* Bit 4: SCLO/SDAO Write Protect */
#define IIC_ICCR1_CLO               (1 << 5)    /* Bit 5: Extra SCL Clock Cycle Output */
#define IIC_ICCR1_IICRST            (1 << 6)    /* Bit 6: I2C Bus Interface Internal Reset */
#define IIC_ICCR1_ICE               (1 << 7)    /* Bit 7: I2C Bus Interface Enable */

/*  I2C Bus Control Register 2 */

#define IIC_ICCR2_ST                (1 << 1)    /* Bit 1: Start Condition Issuance Request */
#define IIC_ICCR2_RS                (1 << 2)    /* Bit 2: Restart Condition Issuance Request */
#define IIC_ICCR2_SP                (1 << 3)    /* Bit 3: Stop Condition Issuance Request */
#define IIC_ICCR2_TRS               (1 << 5)    /* Bit 5: Transmit/Receive Mode */
#define IIC_ICCR2_MST               (1 << 6)    /* Bit 6: Master/Slave Mode */
#define IIC_ICCR2_BBSY              (1 << 7)    /* Bit 7: Bus Busy Detection Flag */

/*   I2C Bus Mode Register 1 */

#define IIC_ICMR1_BC_SHIFT          (0)         /* Bits 0..2: Bit Counter */
#define IIC_ICMR1_BC_MASK           (7 << IIC_ICMR1_BC_SHIFT)
#  define IIC_ICMR1_BC(n)           (((n) << IIC_ICMR1_BC_SHIFT) & IIC_ICMR1_BC_MASK)
#define IIC_ICMR1_BCWP              (1 << 3)    /* Bit 3: BC Write Protect */
#define IIC_ICMR1_CKS_SHIFT         (4)         /* Bits 4..6: Internal Reference Clock Select */
#define IIC_ICMR1_CKS_MASK          (7 << IIC_ICMR1_CKS_SHIFT)
#  define IIC_ICMR1_CKS(n)          (((n) << IIC_ICMR1_CKS_SHIFT) & IIC_ICMR1_CKS_MASK)
#define IIC_ICMR1_MTWP              (1 << 7)    /* Bit 7: MST/TRS Write Protect */

/*   I2C Bus Mode Register 2 */

#define IIC_ICMR2_TMOS              (1 << 0)    /* Bit 0: Timeout Detection Time Select */
#define IIC_ICMR2_TMOL              (1 << 1)    /* Bit 1: Timeout L Count Control */
#define IIC_ICMR2_TMOH              (1 << 2)    /* Bit 2: Timeout H Count Control */
#define IIC_ICMR2_SDDL_SHIFT        (4)         /* Bits 4..6: SDA Output Delay Counter */
#define IIC_ICMR2_SDDL_MASK         (7 << IIC_ICMR2_SDDL_SHIFT)
#  define IIC_ICMR2_SDDL(n)         (((n) << IIC_ICMR2_SDDL_SHIFT) & IIC_ICMR2_SDDL_MASK)
#define IIC_ICMR2_DLCS              (1 << 7)    /* Bit 7: SDA Output Delay Clock Source Select */

/*   I2C Bus Mode Register 3 */

#define IIC_ICMR3_NF_SHIFT          (0)         /* Bits 0..2: Bit Counter */
#define IIC_ICMR3_NF_MASK           (3 << IIC_ICMR3_NF_SHIFT)
#  define IIC_ICMR3_NF(n)           (((n) << IIC_ICMR3_NF_SHIFT) & IIC_ICMR3_NF_MASK)
#define IIC_ICMR3_ACKBR             (1 << 2)    /* Bit 2: Receive Acknowledge */
#define IIC_ICMR3_ACKBT             (1 << 3)    /* Bit 3: Transmit Acknowledge */
#define IIC_ICMR3_ACKWP             (1 << 4)    /* Bit 4: ACKBT Write Protect */
#define IIC_ICMR3_RDRFS             (1 << 5)    /* Bit 5: RDRF Flag Set Timing Select */
#define IIC_ICMR3_WAIT              (1 << 6)    /* Bit 6: Low-hold is released by reading ICDRR */
#define IIC_ICMR3_SMBS              (1 << 7)    /* Bit 7: SMBus/I2C Bus Select */

/*   I2C Bus Function Enable Register */

#define IIC_ICFER_TMOE              (1 << 0)    /* Bit 0: Timeout Function Enable */
#define IIC_ICFER_MALE              (1 << 1)    /* Bit 1: Master Arbitration-Lost Detection Enable */
#define IIC_ICFER_NALE              (1 << 2)    /* Bit 2: NACK Transmission Arbitration-Lost Detection Enable */
#define IIC_ICFER_SALE              (1 << 3)    /* Bit 3: Slave Arbitration-Lost Detection Enable */
#define IIC_ICFER_NACKE             (1 << 4)    /* Bit 4: NACK Reception Transfer Suspension Enable */
#define IIC_ICFER_NFE               (1 << 5)    /* Bit 5: Digital Noise Filter Circuit Enable */
#define IIC_ICFER_SCLE              (1 << 6)    /* Bit 6: SCL Synchronous Circuit Enable */
#define IIC_ICFER_FMPE              (1 << 7)    /* Bit 7: Fast-Mode Plus Enable */

/*   I2C Bus Status Enable Register */

#define IIC_ICSER_SAR0E             (1 << 0)    /* Bit 0: Slave Address Register 0 Enable */
#define IIC_ICSER_SAR1E             (1 << 1)    /* Bit 1: Slave Address Register 1 Enable */
#define IIC_ICSER_SAR2E             (1 << 2)    /* Bit 2: Slave Address Register 2 Enable */
#define IIC_ICSER_GCAE              (1 << 3)    /* Bit 3: General Call Address Enable */
#define IIC_ICSER_DIDE              (1 << 5)    /* Bit 5: Device-ID Address Detection Enable */
#define IIC_ICSER_HOAE              (1 << 7)    /* Bit 7: Host Address Enable */

/*   I2C Bus Interrupt Enable Register */

#define IIC_ICIER_TMOIE             (1 << 0)    /* Bit 0: Timeout Interrupt Request Enable */
#define IIC_ICIER_ALIE              (1 << 1)    /* Bit 1: Arbitration-Lost Interrupt Request Enable */
#define IIC_ICIER_STIE              (1 << 2)    /* Bit 2: Start Condition Detection Interrupt Request Enable */
#define IIC_ICIER_SPIE              (1 << 3)    /* Bit 3: Stop Condition Detection Interrupt Request Enable */
#define IIC_ICIER_NAKIE             (1 << 4)    /* Bit 4: NACK Reception Interrupt Request Enable */
#define IIC_ICIER_RIE               (1 << 5)    /* Bit 5: Receive Data Full Interrupt Request Enable */
#define IIC_ICIER_TEIE              (1 << 6)    /* Bit 6: Transmit End Interrupt Request Enable */
#define IIC_ICIER_TIE               (1 << 7)    /* Bit 7: Transmit Data Empty Interrupt Request Enable */


/*  I2C Bus Status Register 1 */

#define IIC_ICSR1_AAS0              (1 << 0)    /* Bit 0: Slave Address 0 Detection Flag */
#define IIC_ICSR1_AAS1              (1 << 1)    /* Bit 1: Slave Address 1 Detection Flag */
#define IIC_ICSR1_AAS2              (1 << 2)    /* Bit 2: Slave Address 2 Detection Flag */
#define IIC_ICSR1_GCA               (1 << 3)    /* Bit 3: General Call Address Detection Flag */
#define IIC_ICSR1_DID               (1 << 5)    /* Bit 5: Device-ID Address Detection Flag */
#define IIC_ICSR1_HOA               (1 << 7)    /* Bit 7: Host Address Detection Flag */

/*  I2C Bus Status Register 2 */

#define IIC_ICSR2_TMOF              (1 << 0)    /* Bit 0: Timeout Detection Flag */
#define IIC_ICSR2_AL                (1 << 1)    /* Bit 1: Arbitration-Lost Flag */
#define IIC_ICSR2_START             (1 << 2)    /* Bit 2: Start Condition Detection Flag */
#define IIC_ICSR2_STOP              (1 << 3)    /* Bit 3: Stop Condition Detection Flag */
#define IIC_ICSR2_NACKF             (1 << 4)    /* Bit 4: NACK Detection Flag */
#define IIC_ICSR2_RDRF              (1 << 5)    /* Bit 5: Receive Data Full Flag */
#define IIC_ICSR2_TEND              (1 << 6)    /* Bit 6: Transmit End Flag */
#define IIC_ICSR2_TDRE              (1 << 7)    /* Bit 7: Transmit Data Empty Flag */

#define RIIC_ICSR2_ERRORMASK        (IIC_ICSR2_AL | IIC_ICSR2_NACKF | IIC_ICSR2_TMOF)

/*   I2C Bus Bit Rate Low-Level Register */

#define IIC_ICBRL_BRL_SHIFT         (0)         /* Bits 0..4: Bit Rate Low-Level Period */
#define IIC_ICBRL_BRL_MASK          (31 << IIC_ICBRL_BRL_SHIFT)
#  define IIC_ICBRL_BRL(n)          (((n) << IIC_ICBRL_BRL_SHIFT) & IIC_ICBRL_BRL_MASK)
#define RIIC_ICBRL_MASK             (0xE0)

/*   I2C Bus Bit Rate High-Level Register */

#define IIC_ICBRH_BRL_SHIFT         (0)         /* Bits 0..4: Bit Rate High-Level Period */
#define IIC_ICBRH_BRL_MASK          (31 << IIC_ICBRH_BRL_SHIFT)
#  define IIC_ICBRH_BRL(n)          (((n) << IIC_ICBRH_BRL_SHIFT) & IIC_ICBRH_BRL_MASK)
#define RIIC_ICBRH_MASK             (0xE0)

#endif /* CONFIG_RA6M5_R7FA6M5BX */
#endif /* __ARCH_ARM_SRC_RA6M5_HARDWARE_R7FA6M5BX_IIC_H */
