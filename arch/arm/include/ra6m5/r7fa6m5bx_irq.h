/****************************************************************************
 * arch/arm/include/ra6m5/r7fa6m5bx_irq.h
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

#ifndef __ARCH_ARM_INCLUDE_RA6M5_R7FA6M5BX_IRQ_H
#define __ARCH_ARM_INCLUDE_RA6M5_R7FA6M5BX_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/ra6m5/ra6m5_irq.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* IRQ numbers.  The IRQ number corresponds vector number and hence map
 * directly to bits in the NVIC.  This does, however, waste several words of
 * memory in the IRQ to handle mapping tables.
 *
 * Processor Exceptions (vectors 0-15). These common definitions can be found
 * in the file nuttx/arch/arm/include/ra6m5/ra6m5_irq.h, which is
 * included above.
 *
 * External interrupts (vectors >= 16)
 *
 * These interrupt vectors were implemented based on RM0456 Table 153
 * (RA6M575/585 vector table) and should work for RA6M575xx and
 * STL32U585xx.
 */

/* SCI */

#define RA6M5_RXI_IRQ_OFFSET    (0)   /* RxIn */
#define RA6M5_TXI_IRQ_OFFSET    (1)   /* TxIn */
#define RA6M5_TEI_IRQ_OFFSET    (2)   /* TEIn */
#define RA6M5_ERI_IRQ_OFFSET    (3)   /* ERIn */
#define RA6M5_SCI_NIRQS         (4)

#define RA6M5_IRQ_ICU_IELSR0    (RA6M5_IRQ_FIRST + 0)   /* 0:   Event selected in the ICU.IELSR0  register */
#define RA6M5_IRQ_ICU_IELSR1    (RA6M5_IRQ_FIRST + 1)   /* 1:   Event selected in the ICU.IELSR1  register */
#define RA6M5_IRQ_ICU_IELSR2    (RA6M5_IRQ_FIRST + 2)   /* 2:   Event selected in the ICU.IELSR2  register */
#define RA6M5_IRQ_ICU_IELSR3    (RA6M5_IRQ_FIRST + 3)   /* 3:   Event selected in the ICU.IELSR3  register */
#define RA6M5_IRQ_ICU_IELSR4    (RA6M5_IRQ_FIRST + 4)   /* 4:   Event selected in the ICU.IELSR4  register */
#define RA6M5_IRQ_ICU_IELSR5    (RA6M5_IRQ_FIRST + 5)   /* 5:   Event selected in the ICU.IELSR5  register */
#define RA6M5_IRQ_ICU_IELSR6    (RA6M5_IRQ_FIRST + 6)   /* 6:   Event selected in the ICU.IELSR6  register */
#define RA6M5_IRQ_ICU_IELSR7    (RA6M5_IRQ_FIRST + 7)   /* 7:   Event selected in the ICU.IELSR7  register */
#define RA6M5_IRQ_ICU_IELSR8    (RA6M5_IRQ_FIRST + 8)   /* 8:   Event selected in the ICU.IELSR8  register */
#define RA6M5_IRQ_ICU_IELSR9    (RA6M5_IRQ_FIRST + 9)   /* 9:   Event selected in the ICU.IELSR9  register */
#define RA6M5_IRQ_ICU_IELSR10   (RA6M5_IRQ_FIRST + 10)  /* 10:  Event selected in the ICU.IELSR10 register */
#define RA6M5_IRQ_ICU_IELSR11   (RA6M5_IRQ_FIRST + 11)  /* 11:  Event selected in the ICU.IELSR11 register */
#define RA6M5_IRQ_ICU_IELSR12   (RA6M5_IRQ_FIRST + 12)  /* 12:  Event selected in the ICU.IELSR12 register */
#define RA6M5_IRQ_ICU_IELSR13   (RA6M5_IRQ_FIRST + 13)  /* 13:  Event selected in the ICU.IELSR13 register */
#define RA6M5_IRQ_ICU_IELSR14   (RA6M5_IRQ_FIRST + 14)  /* 14:  Event selected in the ICU.IELSR14 register */
#define RA6M5_IRQ_ICU_IELSR15   (RA6M5_IRQ_FIRST + 15)  /* 15:  Event selected in the ICU.IELSR15 register */
#define RA6M5_IRQ_ICU_IELSR16   (RA6M5_IRQ_FIRST + 16)  /* 16:  Event selected in the ICU.IELSR16 register */
#define RA6M5_IRQ_ICU_IELSR17   (RA6M5_IRQ_FIRST + 17)  /* 17:  Event selected in the ICU.IELSR17 register */
#define RA6M5_IRQ_ICU_IELSR18   (RA6M5_IRQ_FIRST + 18)  /* 18:  Event selected in the ICU.IELSR18 register */
#define RA6M5_IRQ_ICU_IELSR19   (RA6M5_IRQ_FIRST + 19)  /* 19:  Event selected in the ICU.IELSR19 register */
#define RA6M5_IRQ_ICU_IELSR20   (RA6M5_IRQ_FIRST + 20)  /* 20:  Event selected in the ICU.IELSR20 register */
#define RA6M5_IRQ_ICU_IELSR21   (RA6M5_IRQ_FIRST + 21)  /* 21:  Event selected in the ICU.IELSR21 register */
#define RA6M5_IRQ_ICU_IELSR22   (RA6M5_IRQ_FIRST + 22)  /* 22:  Event selected in the ICU.IELSR22 register */
#define RA6M5_IRQ_ICU_IELSR23   (RA6M5_IRQ_FIRST + 23)  /* 23:  Event selected in the ICU.IELSR23 register */
#define RA6M5_IRQ_ICU_IELSR24   (RA6M5_IRQ_FIRST + 24)  /* 24:  Event selected in the ICU.IELSR24 register */
#define RA6M5_IRQ_ICU_IELSR25   (RA6M5_IRQ_FIRST + 25)  /* 25:  Event selected in the ICU.IELSR25 register */
#define RA6M5_IRQ_ICU_IELSR26   (RA6M5_IRQ_FIRST + 26)  /* 26:  Event selected in the ICU.IELSR26 register */
#define RA6M5_IRQ_ICU_IELSR27   (RA6M5_IRQ_FIRST + 27)  /* 27:  Event selected in the ICU.IELSR27 register */
#define RA6M5_IRQ_ICU_IELSR28   (RA6M5_IRQ_FIRST + 28)  /* 28:  Event selected in the ICU.IELSR28 register */
#define RA6M5_IRQ_ICU_IELSR29   (RA6M5_IRQ_FIRST + 29)  /* 29:  Event selected in the ICU.IELSR29 register */
#define RA6M5_IRQ_ICU_IELSR30   (RA6M5_IRQ_FIRST + 30)  /* 30:  Event selected in the ICU.IELSR30 register */
#define RA6M5_IRQ_ICU_IELSR31   (RA6M5_IRQ_FIRST + 31)  /* 31:  Event selected in the ICU.IELSR31 register */
#define RA6M5_IRQ_ICU_IELSR32   (RA6M5_IRQ_FIRST + 32)  /* 32:  Event selected in the ICU.IELSR32 register */
#define RA6M5_IRQ_ICU_IELSR33   (RA6M5_IRQ_FIRST + 33)  /* 33:  Event selected in the ICU.IELSR33 register */
#define RA6M5_IRQ_ICU_IELSR34   (RA6M5_IRQ_FIRST + 34)  /* 34:  Event selected in the ICU.IELSR34 register */
#define RA6M5_IRQ_ICU_IELSR35   (RA6M5_IRQ_FIRST + 35)  /* 35:  Event selected in the ICU.IELSR35 register */
#define RA6M5_IRQ_ICU_IELSR36   (RA6M5_IRQ_FIRST + 36)  /* 36:  Event selected in the ICU.IELSR36 register */
#define RA6M5_IRQ_ICU_IELSR37   (RA6M5_IRQ_FIRST + 37)  /* 37:  Event selected in the ICU.IELSR37 register */
#define RA6M5_IRQ_ICU_IELSR38   (RA6M5_IRQ_FIRST + 38)  /* 38:  Event selected in the ICU.IELSR38 register */
#define RA6M5_IRQ_ICU_IELSR39   (RA6M5_IRQ_FIRST + 39)  /* 39:  Event selected in the ICU.IELSR39 register */
#define RA6M5_IRQ_ICU_IELSR40   (RA6M5_IRQ_FIRST + 40)  /* 40:  Event selected in the ICU.IELSR40 register */
#define RA6M5_IRQ_ICU_IELSR41   (RA6M5_IRQ_FIRST + 41)  /* 41:  Event selected in the ICU.IELSR41 register */
#define RA6M5_IRQ_ICU_IELSR42   (RA6M5_IRQ_FIRST + 42)  /* 42:  Event selected in the ICU.IELSR42 register */
#define RA6M5_IRQ_ICU_IELSR43   (RA6M5_IRQ_FIRST + 43)  /* 43:  Event selected in the ICU.IELSR43 register */
#define RA6M5_IRQ_ICU_IELSR44   (RA6M5_IRQ_FIRST + 44)  /* 44:  Event selected in the ICU.IELSR44 register */
#define RA6M5_IRQ_ICU_IELSR45   (RA6M5_IRQ_FIRST + 45)  /* 45:  Event selected in the ICU.IELSR45 register */
#define RA6M5_IRQ_ICU_IELSR46   (RA6M5_IRQ_FIRST + 46)  /* 46:  Event selected in the ICU.IELSR46 register */
#define RA6M5_IRQ_ICU_IELSR47   (RA6M5_IRQ_FIRST + 47)  /* 47:  Event selected in the ICU.IELSR47 register */
#define RA6M5_IRQ_ICU_IELSR48   (RA6M5_IRQ_FIRST + 48)  /* 48:  Event selected in the ICU.IELSR48 register */
#define RA6M5_IRQ_ICU_IELSR49   (RA6M5_IRQ_FIRST + 49)  /* 49:  Event selected in the ICU.IELSR49 register */
#define RA6M5_IRQ_ICU_IELSR50   (RA6M5_IRQ_FIRST + 50)  /* 50:  Event selected in the ICU.IELSR50 register */
#define RA6M5_IRQ_ICU_IELSR51   (RA6M5_IRQ_FIRST + 51)  /* 51:  Event selected in the ICU.IELSR51 register */
#define RA6M5_IRQ_ICU_IELSR52   (RA6M5_IRQ_FIRST + 52)  /* 52:  Event selected in the ICU.IELSR52 register */
#define RA6M5_IRQ_ICU_IELSR53   (RA6M5_IRQ_FIRST + 53)  /* 53:  Event selected in the ICU.IELSR53 register */
#define RA6M5_IRQ_ICU_IELSR54   (RA6M5_IRQ_FIRST + 54)  /* 54:  Event selected in the ICU.IELSR54 register */
#define RA6M5_IRQ_ICU_IELSR55   (RA6M5_IRQ_FIRST + 55)  /* 55:  Event selected in the ICU.IELSR55 register */
#define RA6M5_IRQ_ICU_IELSR56   (RA6M5_IRQ_FIRST + 56)  /* 56:  Event selected in the ICU.IELSR56 register */
#define RA6M5_IRQ_ICU_IELSR57   (RA6M5_IRQ_FIRST + 57)  /* 57:  Event selected in the ICU.IELSR57 register */
#define RA6M5_IRQ_ICU_IELSR58   (RA6M5_IRQ_FIRST + 58)  /* 58:  Event selected in the ICU.IELSR58 register */
#define RA6M5_IRQ_ICU_IELSR59   (RA6M5_IRQ_FIRST + 59)  /* 59:  Event selected in the ICU.IELSR59 register */
#define RA6M5_IRQ_ICU_IELSR60   (RA6M5_IRQ_FIRST + 60)  /* 60:  Event selected in the ICU.IELSR60 register */
#define RA6M5_IRQ_ICU_IELSR61   (RA6M5_IRQ_FIRST + 61)  /* 61:  Event selected in the ICU.IELSR61 register */
#define RA6M5_IRQ_ICU_IELSR62   (RA6M5_IRQ_FIRST + 62)  /* 62:  Event selected in the ICU.IELSR62 register */
#define RA6M5_IRQ_ICU_IELSR63   (RA6M5_IRQ_FIRST + 63)  /* 63:  Event selected in the ICU.IELSR63 register */
#define RA6M5_IRQ_ICU_IELSR64   (RA6M5_IRQ_FIRST + 64)  /* 64:  Event selected in the ICU.IELSR64 register */
#define RA6M5_IRQ_ICU_IELSR65   (RA6M5_IRQ_FIRST + 65)  /* 65:  Event selected in the ICU.IELSR65 register */
#define RA6M5_IRQ_ICU_IELSR66   (RA6M5_IRQ_FIRST + 66)  /* 66:  Event selected in the ICU.IELSR66 register */
#define RA6M5_IRQ_ICU_IELSR67   (RA6M5_IRQ_FIRST + 67)  /* 67:  Event selected in the ICU.IELSR67 register */
#define RA6M5_IRQ_ICU_IELSR68   (RA6M5_IRQ_FIRST + 68)  /* 68:  Event selected in the ICU.IELSR68 register */
#define RA6M5_IRQ_ICU_IELSR69   (RA6M5_IRQ_FIRST + 69)  /* 69:  Event selected in the ICU.IELSR69 register */
#define RA6M5_IRQ_ICU_IELSR70   (RA6M5_IRQ_FIRST + 70)  /* 70:  Event selected in the ICU.IELSR70 register */
#define RA6M5_IRQ_ICU_IELSR71   (RA6M5_IRQ_FIRST + 71)  /* 71:  Event selected in the ICU.IELSR71 register */
#define RA6M5_IRQ_ICU_IELSR72   (RA6M5_IRQ_FIRST + 72)  /* 72:  Event selected in the ICU.IELSR72 register */
#define RA6M5_IRQ_ICU_IELSR73   (RA6M5_IRQ_FIRST + 73)  /* 73:  Event selected in the ICU.IELSR73 register */
#define RA6M5_IRQ_ICU_IELSR74   (RA6M5_IRQ_FIRST + 74)  /* 74:  Event selected in the ICU.IELSR74 register */
#define RA6M5_IRQ_ICU_IELSR75   (RA6M5_IRQ_FIRST + 75)  /* 75:  Event selected in the ICU.IELSR75 register */
#define RA6M5_IRQ_ICU_IELSR76   (RA6M5_IRQ_FIRST + 76)  /* 76:  Event selected in the ICU.IELSR76 register */
#define RA6M5_IRQ_ICU_IELSR77   (RA6M5_IRQ_FIRST + 77)  /* 77:  Event selected in the ICU.IELSR77 register */
#define RA6M5_IRQ_ICU_IELSR78   (RA6M5_IRQ_FIRST + 78)  /* 78:  Event selected in the ICU.IELSR78 register */
#define RA6M5_IRQ_ICU_IELSR79   (RA6M5_IRQ_FIRST + 79)  /* 79:  Event selected in the ICU.IELSR79 register */
#define RA6M5_IRQ_ICU_IELSR80   (RA6M5_IRQ_FIRST + 80)  /* 80:  Event selected in the ICU.IELSR80 register */
#define RA6M5_IRQ_ICU_IELSR81   (RA6M5_IRQ_FIRST + 81)  /* 81:  Event selected in the ICU.IELSR81 register */
#define RA6M5_IRQ_ICU_IELSR82   (RA6M5_IRQ_FIRST + 82)  /* 82:  Event selected in the ICU.IELSR82 register */
#define RA6M5_IRQ_ICU_IELSR83   (RA6M5_IRQ_FIRST + 83)  /* 83:  Event selected in the ICU.IELSR83 register */
#define RA6M5_IRQ_ICU_IELSR84   (RA6M5_IRQ_FIRST + 84)  /* 84:  Event selected in the ICU.IELSR84 register */
#define RA6M5_IRQ_ICU_IELSR85   (RA6M5_IRQ_FIRST + 85)  /* 85:  Event selected in the ICU.IELSR85 register */
#define RA6M5_IRQ_ICU_IELSR86   (RA6M5_IRQ_FIRST + 86)  /* 86:  Event selected in the ICU.IELSR86 register */
#define RA6M5_IRQ_ICU_IELSR87   (RA6M5_IRQ_FIRST + 87)  /* 87:  Event selected in the ICU.IELSR87 register */
#define RA6M5_IRQ_ICU_IELSR88   (RA6M5_IRQ_FIRST + 88)  /* 88:  Event selected in the ICU.IELSR88 register */
#define RA6M5_IRQ_ICU_IELSR89   (RA6M5_IRQ_FIRST + 89)  /* 89:  Event selected in the ICU.IELSR89 register */
#define RA6M5_IRQ_ICU_IELSR90   (RA6M5_IRQ_FIRST + 90)  /* 90:  Event selected in the ICU.IELSR90 register */
#define RA6M5_IRQ_ICU_IELSR91   (RA6M5_IRQ_FIRST + 91)  /* 91:  Event selected in the ICU.IELSR91 register */
#define RA6M5_IRQ_ICU_IELSR92   (RA6M5_IRQ_FIRST + 92)  /* 92:  Event selected in the ICU.IELSR92 register */
#define RA6M5_IRQ_ICU_IELSR93   (RA6M5_IRQ_FIRST + 93)  /* 93:  Event selected in the ICU.IELSR93 register */
#define RA6M5_IRQ_ICU_IELSR94   (RA6M5_IRQ_FIRST + 94)  /* 94:  Event selected in the ICU.IELSR94 register */
#define RA6M5_IRQ_ICU_IELSR95   (RA6M5_IRQ_FIRST + 95)  /* 95:  Event selected in the ICU.IELSR95 register */

#ifdef CONFIG_RA6M5_SCI0
#  define RA6M5_SCI0_IRQBASE    (RA6M5_IRQ_ICU_IELSR0)
#  define RA6M5_IRQ_SCI0_RXI    (RA6M5_SCI0_IRQBASE + RA6M5_RXI_IRQ_OFFSET) /* SCI0 Receive data full interrupt */
#  define RA6M5_IRQ_SCI0_TXI    (RA6M5_SCI0_IRQBASE + RA6M5_TXI_IRQ_OFFSET) /* SCI0 Transmit data empty interrupt */
#  define RA6M5_IRQ_SCI0_TEI    (RA6M5_SCI0_IRQBASE + RA6M5_TEI_IRQ_OFFSET) /* SCI0 Transmit end interrupt */
#  define RA6M5_IRQ_SCI0_ERI    (RA6M5_SCI0_IRQBASE + RA6M5_ERI_IRQ_OFFSET) /* SCI0 Receive error interrupt */
#  define RA6M5_SCI1_IRQBASE    (RA6M5_SCI0_IRQBASE + RA6M5_SCI_NIRQS)
#else 
#  define RA6M5_SCI1_IRQBASE    (RA6M5_IRQ_ICU_IELSR0)
#endif

#ifdef CONFIG_RA6M5_SCI1
#  define RA6M5_IRQ_SCI1_RXI    (RA6M5_SCI1_IRQBASE + RA6M5_RXI_IRQ_OFFSET) /* SCI1 Receive data full interrupt */
#  define RA6M5_IRQ_SCI1_TXI    (RA6M5_SCI1_IRQBASE + RA6M5_TXI_IRQ_OFFSET) /* SCI1 Transmit data empty interrupt */
#  define RA6M5_IRQ_SCI1_TEI    (RA6M5_SCI1_IRQBASE + RA6M5_TEI_IRQ_OFFSET) /* SCI1 Transmit end interrupt */
#  define RA6M5_IRQ_SCI1_ERI    (RA6M5_SCI1_IRQBASE + RA6M5_ERI_IRQ_OFFSET) /* SCI1 Receive error interrupt */
#  define RA6M5_SCI2_IRQBASE    (RA6M5_SCI1_IRQBASE + RA6M5_SCI_NIRQS)
#else 
#  define RA6M5_SCI2_IRQBASE    (RA6M5_SCI1_IRQBASE)
#endif

#ifdef CONFIG_RA6M5_SCI2
#  define RA6M5_IRQ_SCI2_RXI    (RA6M5_SCI2_IRQBASE + RA6M5_RXI_IRQ_OFFSET) /* SCI2 Receive data full interrupt */
#  define RA6M5_IRQ_SCI2_TXI    (RA6M5_SCI2_IRQBASE + RA6M5_TXI_IRQ_OFFSET) /* SCI2 Transmit data empty interrupt */
#  define RA6M5_IRQ_SCI2_TEI    (RA6M5_SCI2_IRQBASE + RA6M5_TEI_IRQ_OFFSET) /* SCI2 Transmit end interrupt */
#  define RA6M5_IRQ_SCI2_ERI    (RA6M5_SCI2_IRQBASE + RA6M5_ERI_IRQ_OFFSET) /* SCI2 Receive error interrupt */
#  define RA6M5_SCI3_IRQBASE    (RA6M5_SCI2_IRQBASE + RA6M5_SCI_NIRQS)
#else 
#  define RA6M5_SCI3_IRQBASE    (RA6M5_SCI2_IRQBASE)
#endif

#ifdef CONFIG_RA6M5_SCI3
#  define RA6M5_IRQ_SCI3_RXI    (RA6M5_SCI3_IRQBASE + RA6M5_RXI_IRQ_OFFSET) /* SCI3 Receive data full interrupt */
#  define RA6M5_IRQ_SCI3_TXI    (RA6M5_SCI3_IRQBASE + RA6M5_TXI_IRQ_OFFSET) /* SCI3 Transmit data empty interrupt */
#  define RA6M5_IRQ_SCI3_TEI    (RA6M5_SCI3_IRQBASE + RA6M5_TEI_IRQ_OFFSET) /* SCI3 Transmit end interrupt */
#  define RA6M5_IRQ_SCI3_ERI    (RA6M5_SCI3_IRQBASE + RA6M5_ERI_IRQ_OFFSET) /* SCI3 Receive error interrupt */
#  define RA6M5_SCI4_IRQBASE    (RA6M5_SCI3_IRQBASE + RA6M5_SCI_NIRQS)
#else 
#  define RA6M5_SCI4_IRQBASE    (RA6M5_SCI3_IRQBASE)
#endif

#ifdef CONFIG_RA6M5_SCI4
#  define RA6M5_IRQ_SCI4_RXI    (RA6M5_SCI4_IRQBASE + RA6M5_RXI_IRQ_OFFSET) /* SCI4 Receive data full interrupt */
#  define RA6M5_IRQ_SCI4_TXI    (RA6M5_SCI4_IRQBASE + RA6M5_TXI_IRQ_OFFSET) /* SCI4 Transmit data empty interrupt */
#  define RA6M5_IRQ_SCI4_TEI    (RA6M5_SCI4_IRQBASE + RA6M5_TEI_IRQ_OFFSET) /* SCI4 Transmit end interrupt */
#  define RA6M5_IRQ_SCI4_ERI    (RA6M5_SCI4_IRQBASE + RA6M5_ERI_IRQ_OFFSET) /* SCI4 Receive error interrupt */
#  define RA6M5_SCI5_IRQBASE    (RA6M5_SCI4_IRQBASE + RA6M5_SCI_NIRQS)
#else 
#  define RA6M5_SCI5_IRQBASE    (RA6M5_SCI4_IRQBASE)
#endif

#ifdef CONFIG_RA6M5_SCI5
#  define RA6M5_IRQ_SCI5_RXI    (RA6M5_SCI5_IRQBASE + RA6M5_RXI_IRQ_OFFSET) /* SCI5 Receive data full interrupt */
#  define RA6M5_IRQ_SCI5_TXI    (RA6M5_SCI5_IRQBASE + RA6M5_TXI_IRQ_OFFSET) /* SCI5 Transmit data empty interrupt */
#  define RA6M5_IRQ_SCI5_TEI    (RA6M5_SCI5_IRQBASE + RA6M5_TEI_IRQ_OFFSET) /* SCI5 Transmit end interrupt */
#  define RA6M5_IRQ_SCI5_ERI    (RA6M5_SCI5_IRQBASE + RA6M5_ERI_IRQ_OFFSET) /* SCI5 Receive error interrupt */
#  define RA6M5_SCI6_IRQBASE    (RA6M5_SCI5_IRQBASE + RA6M5_SCI_NIRQS)
#else 
#  define RA6M5_SCI6_IRQBASE    (RA6M5_SCI5_IRQBASE)
#endif

#ifdef CONFIG_RA6M5_SCI6
#  define RA6M5_IRQ_SCI6_RXI    (RA6M5_SCI6_IRQBASE + RA6M5_RXI_IRQ_OFFSET) /* SCI6 Receive data full interrupt */
#  define RA6M5_IRQ_SCI6_TXI    (RA6M5_SCI6_IRQBASE + RA6M5_TXI_IRQ_OFFSET) /* SCI6 Transmit data empty interrupt */
#  define RA6M5_IRQ_SCI6_TEI    (RA6M5_SCI6_IRQBASE + RA6M5_TEI_IRQ_OFFSET) /* SCI6 Transmit end interrupt */
#  define RA6M5_IRQ_SCI6_ERI    (RA6M5_SCI6_IRQBASE + RA6M5_ERI_IRQ_OFFSET) /* SCI6 Receive error interrupt */
#  define RA6M5_SCI7_IRQBASE    (RA6M5_SCI6_IRQBASE + RA6M5_SCI_NIRQS)
#else 
#  define RA6M5_SCI7_IRQBASE    (RA6M5_SCI6_IRQBASE)
#endif

#ifdef CONFIG_RA6M5_SCI7
#  define RA6M5_IRQ_SCI7_RXI    (RA6M5_SCI7_IRQBASE + RA6M5_RXI_IRQ_OFFSET) /* SCI7 Receive data full interrupt */
#  define RA6M5_IRQ_SCI7_TXI    (RA6M5_SCI7_IRQBASE + RA6M5_TXI_IRQ_OFFSET) /* SCI7 Transmit data empty interrupt */
#  define RA6M5_IRQ_SCI7_TEI    (RA6M5_SCI7_IRQBASE + RA6M5_TEI_IRQ_OFFSET) /* SCI7 Transmit end interrupt */
#  define RA6M5_IRQ_SCI7_ERI    (RA6M5_SCI7_IRQBASE + RA6M5_ERI_IRQ_OFFSET) /* SCI7 Receive error interrupt */
#  define RA6M5_SCI8_IRQBASE    (RA6M5_SCI7_IRQBASE + RA6M5_SCI_NIRQS)
#else 
#  define RA6M5_SCI8_IRQBASE    (RA6M5_SCI7_IRQBASE)
#endif

#ifdef CONFIG_RA6M5_SCI8
#  define RA6M5_IRQ_SCI8_RXI    (RA6M5_SCI8_IRQBASE + RA6M5_RXI_IRQ_OFFSET) /* SCI8 Receive data full interrupt */
#  define RA6M5_IRQ_SCI8_TXI    (RA6M5_SCI8_IRQBASE + RA6M5_TXI_IRQ_OFFSET) /* SCI8 Transmit data empty interrupt */
#  define RA6M5_IRQ_SCI8_TEI    (RA6M5_SCI8_IRQBASE + RA6M5_TEI_IRQ_OFFSET) /* SCI8 Transmit end interrupt */
#  define RA6M5_IRQ_SCI8_ERI    (RA6M5_SCI8_IRQBASE + RA6M5_ERI_IRQ_OFFSET) /* SCI8 Receive error interrupt */
#  define RA6M5_SCI9_IRQBASE    (RA6M5_SCI8_IRQBASE + RA6M5_SCI_NIRQS)
#else 
#  define RA6M5_SCI9_IRQBASE    (RA6M5_SCI8_IRQBASE)
#endif

#ifdef CONFIG_RA6M5_SCI9
#  define RA6M5_IRQ_SCI9_RXI    (RA6M5_SCI9_IRQBASE + RA6M5_RXI_IRQ_OFFSET) /* SCI9 Receive data full interrupt */
#  define RA6M5_IRQ_SCI9_TXI    (RA6M5_SCI9_IRQBASE + RA6M5_TXI_IRQ_OFFSET) /* SCI9 Transmit data empty interrupt */
#  define RA6M5_IRQ_SCI9_TEI    (RA6M5_SCI9_IRQBASE + RA6M5_TEI_IRQ_OFFSET) /* SCI9 Transmit end interrupt */
#  define RA6M5_IRQ_SCI9_ERI    (RA6M5_SCI9_IRQBASE + RA6M5_ERI_IRQ_OFFSET) /* SCI9 Receive error interrupt */
#  define RA6M5_SPI0_IRQBASE    (RA6M5_SCI9_IRQBASE + RA6M5_SCI_NIRQS)
#else 
#  define RA6M5_SPI0_IRQBASE    (RA6M5_SCI9_IRQBASE)
#endif

#ifdef CONFIG_RA6M5_SPI0
#  define RA6M5_IRQ_SPI0_RXI    (RA6M5_SPI0_IRQBASE + RA6M5_RXI_IRQ_OFFSET) /* SPI0 Receive data full interrupt */
#  define RA6M5_IRQ_SPI0_TXI    (RA6M5_SPI0_IRQBASE + RA6M5_TXI_IRQ_OFFSET) /* SPI0 Transmit data empty interrupt */
#  define RA6M5_IRQ_SPI0_TEI    (RA6M5_SPI0_IRQBASE + RA6M5_TEI_IRQ_OFFSET) /* SPI0 Transmit end interrupt */
#  define RA6M5_IRQ_SPI0_ERI    (RA6M5_SPI0_IRQBASE + RA6M5_ERI_IRQ_OFFSET) /* SPI0 Receive error interrupt */
#  define RA6M5_SPI1_IRQBASE    (RA6M5_SPI0_IRQBASE + RA6M5_SCI_NIRQS)
#else 
#  define RA6M5_SPI1_IRQBASE    (RA6M5_SPI0_IRQBASE)
#endif

#ifdef CONFIG_RA6M5_SPI1
#  define RA6M5_IRQ_SPI1_RXI    (RA6M5_SPI1_IRQBASE + RA6M5_RXI_IRQ_OFFSET) /* SPI1 Receive data full interrupt */
#  define RA6M5_IRQ_SPI1_TXI    (RA6M5_SPI1_IRQBASE + RA6M5_TXI_IRQ_OFFSET) /* SPI1 Transmit data empty interrupt */
#  define RA6M5_IRQ_SPI1_TEI    (RA6M5_SPI1_IRQBASE + RA6M5_TEI_IRQ_OFFSET) /* SPI1 Transmit end interrupt */
#  define RA6M5_IRQ_SPI1_ERI    (RA6M5_SPI1_IRQBASE + RA6M5_ERI_IRQ_OFFSET) /* SPI1 Receive error interrupt */
#  define RA6M5_QSPI_IRQBASE    (RA6M5_SPI1_IRQBASE + RA6M5_SCI_NIRQS)
#else 
#  define RA6M5_QSPI_IRQBASE    (RA6M5_SPI1_IRQBASE)
#endif

#ifdef CONFIG_RA6M5_QSPI
#  define RA6M5_IRQ_QSPI_INT    (RA6M5_QSPI_IRQBASE + 0) /* SPI1 Receive data full interrupt */
#  define RA6M5_OSPI_IRQBASE    (RA6M5_QSPI_IRQBASE + 1)
#else 
#  define RA6M5_OSPI_IRQBASE    (RA6M5_QSPI_IRQBASE)
#endif

#ifdef CONFIG_RA6M5_OSPI
#  define RA6M5_IRQ_QSPI_INT    (RA6M5_OSPI_IRQBASE + 0) /* SPI1 Receive data full interrupt */
#  define RA6M5_IIC0_IRQBASE    (RA6M5_OSPI_IRQBASE + 1)
#else 
#  define RA6M5_IIC0_IRQBASE    (RA6M5_OSPI_IRQBASE)
#endif

#ifdef CONFIG_RA6M5_IIC0
#  define RA6M5_IRQ_IIC0_RXI    (RA6M5_IIC0_IRQBASE + RA6M5_RXI_IRQ_OFFSET) /* IIC0 Receive data full interrupt */
#  define RA6M5_IRQ_IIC0_TXI    (RA6M5_IIC0_IRQBASE + RA6M5_TXI_IRQ_OFFSET) /* IIC0 Transmit data empty interrupt */
#  define RA6M5_IRQ_IIC0_TEI    (RA6M5_IIC0_IRQBASE + RA6M5_TEI_IRQ_OFFSET) /* IIC0 Transmit end interrupt */
#  define RA6M5_IRQ_IIC0_ERI    (RA6M5_IIC0_IRQBASE + RA6M5_ERI_IRQ_OFFSET) /* IIC0 Receive error interrupt */
#  define RA6M5_IIC1_IRQBASE    (RA6M5_IIC0_IRQBASE + RA6M5_SCI_NIRQS)
#else 
#  define RA6M5_IIC1_IRQBASE    (RA6M5_IIC0_IRQBASE)
#endif

#ifdef CONFIG_RA6M5_IIC1
#  define RA6M5_IRQ_IIC1_RXI    (RA6M5_IIC1_IRQBASE + RA6M5_RXI_IRQ_OFFSET) /* IIC1 Receive data full interrupt */
#  define RA6M5_IRQ_IIC1_TXI    (RA6M5_IIC1_IRQBASE + RA6M5_TXI_IRQ_OFFSET) /* IIC1 Transmit data empty interrupt */
#  define RA6M5_IRQ_IIC1_TEI    (RA6M5_IIC1_IRQBASE + RA6M5_TEI_IRQ_OFFSET) /* IIC1 Transmit end interrupt */
#  define RA6M5_IRQ_IIC1_ERI    (RA6M5_IIC1_IRQBASE + RA6M5_ERI_IRQ_OFFSET) /* IIC1 Receive error interrupt */
#  define RA6M5_IIC2_IRQBASE    (RA6M5_IIC1_IRQBASE + RA6M5_SCI_NIRQS)
#else 
#  define RA6M5_IIC2_IRQBASE    (RA6M5_IIC1_IRQBASE)
#endif

#ifdef CONFIG_RA6M5_IIC2
#  define RA6M5_IRQ_IIC2_RXI    (RA6M5_IIC2_IRQBASE + RA6M5_RXI_IRQ_OFFSET) /* IIC2 Receive data full interrupt */
#  define RA6M5_IRQ_IIC2_TXI    (RA6M5_IIC2_IRQBASE + RA6M5_TXI_IRQ_OFFSET) /* IIC2 Transmit data empty interrupt */
#  define RA6M5_IRQ_IIC2_TEI    (RA6M5_IIC2_IRQBASE + RA6M5_TEI_IRQ_OFFSET) /* IIC2 Transmit end interrupt */
#  define RA6M5_IRQ_IIC2_ERI    (RA6M5_IIC2_IRQBASE + RA6M5_ERI_IRQ_OFFSET) /* IIC2 Receive error interrupt */
#  define RA6M5_DTC_IRQBASE     (RA6M5_IIC2_IRQBASE + RA6M5_SCI_NIRQS)
#else 
#  define RA6M5_DTC_IRQBASE     (RA6M5_IIC2_IRQBASE)
#endif

#ifdef CONFIG_RA6M5_DTC
#  define RA6M5_IRQ_DTC_COMP    (RA6M5_DTC_IRQBASE + 0) /* DTC transfer complete interrupt */
#  define RA6M5_IRQ_DTC_END     (RA6M5_DTC_IRQBASE + 1) /* DTC transfer end interrupt */
#  define RA6M5_RTC_IRQBASE     (RA6M5_DTC_IRQBASE + 2)
#else 
#  define RA6M5_RTC_IRQBASE     (RA6M5_DTC_IRQBASE)
#endif

#ifdef CONFIG_RA6M5_RTC
#  define RA6M5_IRQ_RTC_ALARM   (RA6M5_RTC_IRQBASE + 0) /* RTC Alarm interrupt */
#  define RA6M5_IRQ_RTC_PERIOD  (RA6M5_RTC_IRQBASE + 1) /* RTC Periodic interrupt */
#  define RA6M5_IRQ_RTC_CARRY   (RA6M5_RTC_IRQBASE + 2) /* RTC Carry interrupt */
#  define RA6M5_USBFS_IRQBASE   (RA6M5_RTC_IRQBASE + 3)
#else 
#  define RA6M5_USBFS_IRQBASE   (RA6M5_RTC_IRQBASE)
#endif

#ifdef CONFIG_RA6M5_USBFS
#  ifdef CONFIG_USBDEV_DMA
#    define RA6M5_IRQ_USBFS_FIFO0   (RA6M5_USBFS_IRQBASE + 0)   /* DMA transfer request 0 */
#    define RA6M5_IRQ_USBFS_FIFO1   (RA6M5_USBFS_IRQBASE + 1)   /* DMA transfer request 1 */
#    define RA6M5_IRQ_USBFS_INT     (RA6M5_USBFS_IRQBASE + 2)   /* USBFS interrupt */
#    define RA6M5_IRQ_USBFS_RSM     (RA6M5_USBFS_IRQBASE + 3)   /* USBFS resume interrupt */
#    define RA6M5_USBHS_IRQBASE     (RA6M5_USBFS_IRQBASE + 4)
#  else
#    define RA6M5_IRQ_USBFS_INT     (RA6M5_USBFS_IRQBASE + 0)   /* USBFS interrupt */
#    define RA6M5_IRQ_USBFS_RSM     (RA6M5_USBFS_IRQBASE + 1)   /* USBFS resume interrupt */
#    define RA6M5_USBHS_IRQBASE     (RA6M5_USBFS_IRQBASE + 2)
#  endif
#else 
#  define RA6M5_USBHS_IRQBASE       (RA6M5_USBFS_IRQBASE)
#endif

#ifdef CONFIG_RA6M5_USBHS
#  ifdef CONFIG_USBDEV_DMA
#    define RA6M5_IRQ_USBHS_FIFO0   (RA6M5_USBHS_IRQBASE + 0)   /* DMA transfer request 0 */
#    define RA6M5_IRQ_USBHS_FIFO1   (RA6M5_USBHS_IRQBASE + 1)   /* DMA transfer request 1 */
#    define RA6M5_IRQ_USBHS_INT_RSM (RA6M5_USBHS_IRQBASE + 2)   /* USBHS interruptrupt */
#    define RA6M5_AGT0_IRQBASE      (RA6M5_USBHS_IRQBASE + 3)
#  else
#    define RA6M5_IRQ_USBHS_INT_RSM (RA6M5_USBHS_IRQBASE + 0)   /* USBFS interrupt */
#    define RA6M5_AGT0_IRQBASE      (RA6M5_USBHS_IRQBASE + 1)
#  endif
#else 
#  define RA6M5_AGT0_IRQBASE        (RA6M5_USBHS_IRQBASE)
#endif

#ifdef CONFIG_RA6M5_AGT0
#  define RA6M5_IRQ_AGT0_INT        (RA6M5_AGT0_IRQBASE + 0)    /* AGT0 interrupt */
#  define RA6M5_IRQ_AGT0_CMPA       (RA6M5_AGT0_IRQBASE + 1)    /* AGT0 Compare match A */
#  define RA6M5_IRQ_AGT0_CMPB       (RA6M5_AGT0_IRQBASE + 2)    /* AGT0 Compare match B */
#  define RA6M5_AGT1_IRQBASE        (RA6M5_AGT0_IRQBASE + 3)
#else 
#  define RA6M5_AGT1_IRQBASE        (RA6M5_AGT0_IRQBASE)
#endif

#ifdef CONFIG_RA6M5_AGT1
#  define RA6M5_IRQ_AGT1_INT        (RA6M5_AGT1_IRQBASE + 0)    /* AGT1 interrupt */
#  define RA6M5_IRQ_AGT1_CMPA       (RA6M5_AGT1_IRQBASE + 1)    /* AGT1 Compare match A */
#  define RA6M5_IRQ_AGT1_CMPB       (RA6M5_AGT1_IRQBASE + 2)    /* AGT1 Compare match B */
#  define RA6M5_AGT2_IRQBASE        (RA6M5_AGT1_IRQBASE + 3)
#else 
#  define RA6M5_AGT2_IRQBASE        (RA6M5_AGT1_IRQBASE)
#endif

#ifdef CONFIG_RA6M5_AGT2
#  define RA6M5_IRQ_AGT2_INT        (RA6M5_AGT2_IRQBASE + 0)    /* AGT2 interrupt */
#  define RA6M5_IRQ_AGT2_CMPA       (RA6M5_AGT2_IRQBASE + 1)    /* AGT2 Compare match A */
#  define RA6M5_IRQ_AGT2_CMPB       (RA6M5_AGT2_IRQBASE + 2)    /* AGT2 Compare match B */
#  define RA6M5_AGT3_IRQBASE        (RA6M5_AGT2_IRQBASE + 3)
#else 
#  define RA6M5_AGT3_IRQBASE        (RA6M5_AGT2_IRQBASE)
#endif

#ifdef CONFIG_RA6M5_AGT3
#  define RA6M5_IRQ_AGT3_INT        (RA6M5_AGT3_IRQBASE + 0)    /* AGT3 interrupt */
#  define RA6M5_IRQ_AGT3_CMPA       (RA6M5_AGT3_IRQBASE + 1)    /* AGT3 Compare match A */
#  define RA6M5_IRQ_AGT3_CMPB       (RA6M5_AGT3_IRQBASE + 2)    /* AGT3 Compare match B */
#  define RA6M5_AGT4_IRQBASE        (RA6M5_AGT3_IRQBASE + 3)
#else 
#  define RA6M5_AGT4_IRQBASE        (RA6M5_AGT3_IRQBASE)
#endif

#ifdef CONFIG_RA6M5_AGT4
#  define RA6M5_IRQ_AGT4_INT        (RA6M5_AGT4_IRQBASE + 0)    /* AGT4 interrupt */
#  define RA6M5_IRQ_AGT4_CMPA       (RA6M5_AGT4_IRQBASE + 1)    /* AGT4 Compare match A */
#  define RA6M5_IRQ_AGT4_CMPB       (RA6M5_AGT4_IRQBASE + 2)    /* AGT4 Compare match B */
#  define RA6M5_AGT5_IRQBASE        (RA6M5_AGT4_IRQBASE + 3)
#else 
#  define RA6M5_AGT5_IRQBASE        (RA6M5_AGT4_IRQBASE)
#endif

#ifdef CONFIG_RA6M5_AGT5
#  define RA6M5_IRQ_AGT5_INT        (RA6M5_AGT5_IRQBASE + 0)    /* AGT5 interrupt */
#  define RA6M5_IRQ_AGT5_CMPA       (RA6M5_AGT5_IRQBASE + 1)    /* AGT5 Compare match A */
#  define RA6M5_IRQ_AGT5_CMPB       (RA6M5_AGT5_IRQBASE + 2)    /* AGT5 Compare match B */
#  define RA6M5_ETH_IRQBASE         (RA6M5_AGT5_IRQBASE + 3)
#else 
#  define RA6M5_ETH_IRQBASE         (RA6M5_AGT5_IRQBASE)
#endif

#ifdef CONFIG_RA6M5_EMAC
#  define RA6M5_IRQ_ETH             (RA6M5_ETH_IRQBASE + 0)     /* EDMAC 0 interrupt */
#  define RA6M5_NEXT_IRQBASE        (RA6M5_ETH_IRQBASE + 1)
#else 
#  define RA6M5_NEXT_IRQBASE        (RA6M5_ETH_IRQBASE)
#endif

#if defined(CONFIG_RA6M5_R7FA6M5BX)
#  define RA6M5_IRQ_NEXTINTS  96
#else
#  error "Unsupported RA6M5 chip"
#endif

/* (EXTI interrupts do not use IRQ numbers) */

#define NR_IRQS                     (RA6M5_IRQ_FIRST + RA6M5_IRQ_NEXTINTS)

#endif /* __ARCH_ARM_INCLUDE_RA6M5_R7FA6M5BX_IRQ_H */
