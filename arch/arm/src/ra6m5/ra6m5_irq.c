/****************************************************************************
 * arch/arm/src/ra6m5/ra6m5_irq.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/irq.h>
#include <arch/armv8-m/nvicpri.h>

#include "nvic.h"
#include "ram_vectors.h"
#include "arm_internal.h"
#include "ra6m5.h"
#include "ra6m5_rcc.h"
#include "hardware/ra6m5_icu.h"


/****************************************************************************
 * Public Data
 ****************************************************************************/
static const uint32_t g_interrupt_event_link_select[RA6M5_IRQ_NEXTINTS] =
{
#ifdef RA6M5_IRQ_SCI0_RXI
  ELC_EVENT_SCI0_RXI,
#endif
#ifdef RA6M5_IRQ_SCI0_TXI
  ELC_EVENT_SCI0_TXI,
#endif
#ifdef RA6M5_IRQ_SCI0_TEI
  ELC_EVENT_SCI0_TEI,
#endif
#ifdef RA6M5_IRQ_SCI0_ERI
  ELC_EVENT_SCI0_ERI,
#endif
#ifdef RA6M5_IRQ_SCI1_RXI
  ELC_EVENT_SCI1_RXI,
#endif
#ifdef RA6M5_IRQ_SCI1_TXI
  ELC_EVENT_SCI1_TXI,
#endif
#ifdef RA6M5_IRQ_SCI1_TEI
  ELC_EVENT_SCI1_TEI,
#endif
#ifdef RA6M5_IRQ_SCI1_ERI
  ELC_EVENT_SCI1_ERI,
#endif
#ifdef RA6M5_IRQ_SCI2_RXI
  ELC_EVENT_SCI2_RXI,
#endif
#ifdef RA6M5_IRQ_SCI2_TXI
  ELC_EVENT_SCI2_TXI,
#endif
#ifdef RA6M5_IRQ_SCI2_TEI
  ELC_EVENT_SCI2_TEI,
#endif
#ifdef RA6M5_IRQ_SCI2_ERI
  ELC_EVENT_SCI2_ERI,
#endif
#ifdef RA6M5_IRQ_SCI3_RXI
  ELC_EVENT_SCI3_RXI,
#endif
#ifdef RA6M5_IRQ_SCI3_TXI
  ELC_EVENT_SCI3_TXI,
#endif
#ifdef RA6M5_IRQ_SCI3_TEI
  ELC_EVENT_SCI3_TEI,
#endif
#ifdef RA6M5_IRQ_SCI3_ERI
  ELC_EVENT_SCI3_ERI,
#endif
#ifdef RA6M5_IRQ_SCI4_RXI
  ELC_EVENT_SCI4_RXI,
#endif
#ifdef RA6M5_IRQ_SCI4_TXI
  ELC_EVENT_SCI4_TXI,
#endif
#ifdef RA6M5_IRQ_SCI4_TEI
  ELC_EVENT_SCI4_TEI,
#endif
#ifdef RA6M5_IRQ_SCI4_ERI
  ELC_EVENT_SCI4_ERI,
#endif
#ifdef RA6M5_IRQ_SCI5_RXI
  ELC_EVENT_SCI5_RXI,
#endif
#ifdef RA6M5_IRQ_SCI5_TXI
  ELC_EVENT_SCI5_TXI,
#endif
#ifdef RA6M5_IRQ_SCI5_TEI
  ELC_EVENT_SCI5_TEI,
#endif
#ifdef RA6M5_IRQ_SCI5_ERI
  ELC_EVENT_SCI5_ERI,
#endif
#ifdef RA6M5_IRQ_SCI6_RXI
  ELC_EVENT_SCI6_RXI,
#endif
#ifdef RA6M5_IRQ_SCI6_TXI
  ELC_EVENT_SCI6_TXI,
#endif
#ifdef RA6M5_IRQ_SCI6_TEI
  ELC_EVENT_SCI6_TEI,
#endif
#ifdef RA6M5_IRQ_SCI6_ERI
  ELC_EVENT_SCI6_ERI,
#endif
#ifdef RA6M5_IRQ_SCI7_RXI
  ELC_EVENT_SCI7_RXI,
#endif
#ifdef RA6M5_IRQ_SCI7_TXI
  ELC_EVENT_SCI7_TXI,
#endif
#ifdef RA6M5_IRQ_SCI7_TEI
  ELC_EVENT_SCI7_TEI,
#endif
#ifdef RA6M5_IRQ_SCI7_ERI
  ELC_EVENT_SCI7_ERI,
#endif
#ifdef RA6M5_IRQ_SCI8_RXI
  ELC_EVENT_SCI8_RXI,
#endif
#ifdef RA6M5_IRQ_SCI8_TXI
  ELC_EVENT_SCI8_TXI,
#endif
#ifdef RA6M5_IRQ_SCI8_TEI
  ELC_EVENT_SCI8_TEI,
#endif
#ifdef RA6M5_IRQ_SCI8_ERI
  ELC_EVENT_SCI8_ERI,
#endif
#ifdef RA6M5_IRQ_SCI9_RXI
  ELC_EVENT_SCI9_RXI,
#endif
#ifdef RA6M5_IRQ_SCI9_TXI
  ELC_EVENT_SCI9_TXI,
#endif
#ifdef RA6M5_IRQ_SCI9_TEI
  ELC_EVENT_SCI9_TEI,
#endif
#ifdef RA6M5_IRQ_SCI9_ERI
  ELC_EVENT_SCI9_ERI,
#endif
#ifdef RA6M5_IRQ_SPI0_RXI
  ELC_EVENT_SPI0_RXI,
#endif
#ifdef RA6M5_IRQ_SPI0_TXI
  ELC_EVENT_SPI0_TXI,
#endif
#ifdef RA6M5_IRQ_SPI0_TEI
  ELC_EVENT_SPI0_TEI,
#endif
#ifdef RA6M5_IRQ_SPI0_ERI
  ELC_EVENT_SPI0_ERI,
#endif
#ifdef RA6M5_IRQ_SPI1_RXI
  ELC_EVENT_SPI1_RXI,
#endif
#ifdef RA6M5_IRQ_SPI1_TXI
  ELC_EVENT_SPI1_TXI,
#endif
#ifdef RA6M5_IRQ_SPI1_TEI
  ELC_EVENT_SPI1_TEI,
#endif
#ifdef RA6M5_IRQ_SPI1_ERI
  ELC_EVENT_SPI1_ERI,
#endif
#ifndef CONFIG_I2C_POLLED
#ifdef RA6M5_IRQ_IIC0_RXI
  ELC_EVENT_IIC0_RXI,
#endif
#ifdef RA6M5_IRQ_IIC0_TXI
  ELC_EVENT_IIC0_TXI,
#endif
#ifdef RA6M5_IRQ_IIC0_TEI
  ELC_EVENT_IIC0_TEI,
#endif
#ifdef RA6M5_IRQ_IIC0_ERI
  ELC_EVENT_IIC0_ERI,
#endif
#ifdef RA6M5_IRQ_IIC1_RXI
  ELC_EVENT_IIC1_RXI,
#endif
#ifdef RA6M5_IRQ_IIC1_TXI
  ELC_EVENT_IIC1_TXI,
#endif
#ifdef RA6M5_IRQ_IIC1_TEI
  ELC_EVENT_IIC1_TEI,
#endif
#ifdef RA6M5_IRQ_IIC1_ERI
  ELC_EVENT_IIC1_ERI,
#endif
#ifdef RA6M5_IRQ_IIC2_RXI
  ELC_EVENT_IIC2_RXI,
#endif
#ifdef RA6M5_IRQ_IIC2_TXI
  ELC_EVENT_IIC2_TXI,
#endif
#ifdef RA6M5_IRQ_IIC2_TEI
  ELC_EVENT_IIC2_TEI,
#endif
#ifdef RA6M5_IRQ_IIC2_ERI
  ELC_EVENT_IIC2_ERI,
#endif
#endif /* CONFIG_I2C_POLLED */
#ifdef RA6M5_IRQ_DTC_COMP
  ELC_EVENT_DTC_COMPLETE,
#endif
#ifdef RA6M5_IRQ_DTC_END
  ELC_EVENT_DTC_END,
#endif
};

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Get a 32-bit version of the default priority */

#define DEFPRIORITY32 \
  (NVIC_SYSH_PRIORITY_DEFAULT << 24 | \
   NVIC_SYSH_PRIORITY_DEFAULT << 16 | \
   NVIC_SYSH_PRIORITY_DEFAULT << 8  | \
   NVIC_SYSH_PRIORITY_DEFAULT)

/* Given the address of a NVIC ENABLE register, this is the offset to
 * the corresponding CLEAR ENABLE register.
 */

#define NVIC_ENA_OFFSET    (0)
#define NVIC_CLRENA_OFFSET (NVIC_IRQ0_31_CLEAR - NVIC_IRQ0_31_ENABLE)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra6m5_dumpnvic
 *
 * Description:
 *   Dump some interesting NVIC registers
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_IRQ_INFO)
static void ra6m5_dumpnvic(const char *msg, int irq)
{
  irqstate_t flags;

  flags = enter_critical_section();

  irqinfo("NVIC (%s, irq=%d):\n", msg, irq);
  irqinfo("  INTCTRL:    %08x VECTAB:  %08x\n",
          getreg32(NVIC_INTCTRL), getreg32(NVIC_VECTAB));
  irqinfo("  IRQ ENABLE: %08x %08x %08x\n",
          getreg32(NVIC_IRQ0_31_ENABLE), getreg32(NVIC_IRQ32_63_ENABLE),
          getreg32(NVIC_IRQ64_95_ENABLE));
  irqinfo("  SYSH_PRIO:  %08x %08x %08x\n",
          getreg32(NVIC_SYSH4_7_PRIORITY), getreg32(NVIC_SYSH8_11_PRIORITY),
          getreg32(NVIC_SYSH12_15_PRIORITY));
  irqinfo("  IRQ PRIO:   %08x %08x %08x %08x\n",
          getreg32(NVIC_IRQ0_3_PRIORITY), getreg32(NVIC_IRQ4_7_PRIORITY),
          getreg32(NVIC_IRQ8_11_PRIORITY), getreg32(NVIC_IRQ12_15_PRIORITY));
  irqinfo("              %08x %08x %08x %08x\n",
          getreg32(NVIC_IRQ16_19_PRIORITY), getreg32(NVIC_IRQ20_23_PRIORITY),
          getreg32(NVIC_IRQ24_27_PRIORITY),
          getreg32(NVIC_IRQ28_31_PRIORITY));
  irqinfo("              %08x %08x %08x %08x\n",
          getreg32(NVIC_IRQ32_35_PRIORITY), getreg32(NVIC_IRQ36_39_PRIORITY),
          getreg32(NVIC_IRQ40_43_PRIORITY),
          getreg32(NVIC_IRQ44_47_PRIORITY));
  irqinfo("              %08x %08x %08x %08x\n",
          getreg32(NVIC_IRQ48_51_PRIORITY), getreg32(NVIC_IRQ52_55_PRIORITY),
          getreg32(NVIC_IRQ56_59_PRIORITY),
          getreg32(NVIC_IRQ60_63_PRIORITY));
  irqinfo("              %08x\n",
          getreg32(NVIC_IRQ64_67_PRIORITY));

  leave_critical_section(flags);
}
#else
#  define ra6m5_dumpnvic(msg, irq)
#endif

/****************************************************************************
 * Name: ra6m5_nmi, ra6m5_pendsv,
 *       ra6m5_dbgmonitor, ra6m5_pendsv, ra6m5_reserved
 *
 * Description:
 *   Handlers for various exceptions.  None are handled and all are fatal
 *   error conditions.  The only advantage these provide over the default
 *   unexpected interrupt handler is that they provide a diagnostic output.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
static int ra6m5_nmi(int irq, void *context, void *arg)
{
  up_irq_save();
  _err("PANIC!!! NMI received\n");
  PANIC();
  return 0;
}

static int ra6m5_pendsv(int irq, void *context, void *arg)
{
  up_irq_save();
  _err("PANIC!!! PendSV received\n");
  PANIC();
  return 0;
}

static int ra6m5_dbgmonitor(int irq, void *context, void *arg)
{
  up_irq_save();
  _err("PANIC!!! Debug Monitor received\n");
  PANIC();
  return 0;
}

static int ra6m5_reserved(int irq, void *context, void *arg)
{
  up_irq_save();
  _err("PANIC!!! Reserved interrupt\n");
  PANIC();
  return 0;
}
#endif

/****************************************************************************
 * Name: ra6m5_prioritize_syscall
 *
 * Description:
 *   Set the priority of an exception.  This function may be needed
 *   internally even if support for prioritized interrupts is not enabled.
 *
 ****************************************************************************/

#ifdef CONFIG_ARMV8M_USEBASEPRI
static inline void ra6m5_prioritize_syscall(int priority)
{
  uint32_t regval;

  /* SVCALL is system handler 11 */

  regval  = getreg32(NVIC_SYSH8_11_PRIORITY);
  regval &= ~NVIC_SYSH_PRIORITY_PR11_MASK;
  regval |= (priority << NVIC_SYSH_PRIORITY_PR11_SHIFT);
  putreg32(regval, NVIC_SYSH8_11_PRIORITY);
}
#endif

/****************************************************************************
 * Name: ra6m5_irqinfo
 *
 * Description:
 *   Given an IRQ number, provide the register and bit setting to enable or
 *   disable the irq.
 *
 ****************************************************************************/

static int ra6m5_irqinfo(int irq, uintptr_t *regaddr, uint32_t *bit,
                         uintptr_t offset)
{
  int n;

  DEBUGASSERT(irq >= RA6M5_IRQ_NMI && irq < NR_IRQS);

  /* Check for external interrupt */

  if (irq >= RA6M5_IRQ_FIRST)
    {
      n        = irq - RA6M5_IRQ_FIRST;
      *regaddr = NVIC_IRQ_ENABLE(n) + offset;
      *bit     = (uint32_t)1 << (n & 0x1f);
    }

  /* Handle processor exceptions.  Only a few can be disabled */

  else
    {
      *regaddr = NVIC_SYSHCON;
      if (irq == RA6M5_IRQ_MEMFAULT)
        {
          *bit = NVIC_SYSHCON_MEMFAULTENA;
        }
      else if (irq == RA6M5_IRQ_BUSFAULT)
        {
          *bit = NVIC_SYSHCON_BUSFAULTENA;
        }
      else if (irq == RA6M5_IRQ_USAGEFAULT)
        {
          *bit = NVIC_SYSHCON_USGFAULTENA;
        }
      else if (irq == RA6M5_IRQ_SYSTICK)
        {
          *regaddr = NVIC_SYSTICK_CTRL;
          *bit = NVIC_SYSTICK_CTRL_ENABLE;
        }
      else
        {
          return ERROR; /* Invalid or unsupported exception */
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  uint32_t regaddr;
  uint32_t regval;
  uint16_t prcr;
  int num_priority_registers;
  int i;

  /* Unprotect security registers. */
  prcr = getreg16(RA6M5_SYSTEM_REG(RA6M5_SYS_PRCR_OFFSET));
  putreg16((prcr | BSP_PRV_PRCR_KEY) | SYS_PRCR_PRC4, RA6M5_SYSTEM_REG(RA6M5_SYS_PRCR_OFFSET));

  /* Set the DMAC channels to secure access. */
  regval  = getreg32(RA6M5_CPSCU_REG(RA6M5_CPSCU_ICUSARC_OFFSET));
  regval &= ~0xFFU; 
  putreg32(regval, RA6M5_CPSCU_REG(RA6M5_CPSCU_ICUSARC_OFFSET));

  /* The Secure Attribute managed within the ARM CPU NVIC must match the security attribution of IELSEn
  * (Reference section 13.2.9 in the RA6M4 manual R01UH0890EJ0050). */
  for (i = 0; i < RA6M5_IRQ_NEXTINTS / 32; i++)
    {
        /* Place all vectors in secure state */
        putreg32(0, RA6M5_CPSCU_ICUSAR_REG(i));
        putreg32(0, NVIC_IRQ_TARGET(i));
    }

  /* Protect security registers. */
  putreg16((prcr | BSP_PRV_PRCR_KEY) | ~SYS_PRCR_PRC4, RA6M5_SYSTEM_REG(RA6M5_SYS_PRCR_OFFSET));

  for (i = 0; i < RA6M5_IRQ_NEXTINTS; i++)
    {
        putreg32(g_interrupt_event_link_select[i], RA6M5_ICU_IELSR_REG(i));
    }

  for (i = 0; i < RA6M5_IRQ_NEXTINTS; i += 32)
    {
      putreg32(0xffffffff, NVIC_IRQ_CLEAR(i));
    }

  /* The standard location for the vector table is at the beginning of FLASH
   * at address 0x0000:0000.  If we are using the STMicro DFU bootloader,
   * then the vector table will be offset to a different location in FLASH
   * and we will need to set the NVIC vector location to this alternative
   * location.
   */

  putreg32((uint32_t)_vectors, NVIC_VECTAB);

#ifdef CONFIG_ARCH_RAMVECTORS
  /* If CONFIG_ARCH_RAMVECTORS is defined, then we are using a RAM-based
   * vector table that requires special initialization.
   */

  arm_ramvec_initialize();
#endif

  /* Set all interrupts (and exceptions) to the default priority */

  putreg32(DEFPRIORITY32, NVIC_SYSH4_7_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_SYSH8_11_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_SYSH12_15_PRIORITY);

  /* The NVIC ICTR register (bits 0-4) holds the number of of interrupt
   * lines that the NVIC supports:
   *
   *  0 -> 32 interrupt lines,  8 priority registers
   *  1 -> 64 "       " "   ", 16 priority registers
   *  2 -> 96 "       " "   ", 32 priority registers
   *  ...
   */

  num_priority_registers = (getreg32(NVIC_ICTR) + 1) * 8;

  /* Now set all of the interrupt lines to the default priority */

  regaddr = NVIC_IRQ0_3_PRIORITY;
  while (num_priority_registers--)
    {
      putreg32(DEFPRIORITY32, regaddr);
      regaddr += 4;
    }

  /* Attach the SVCall and Hard Fault exception handlers.  The SVCall
   * exception is used for performing context switches; The Hard Fault
   * must also be caught because a SVCall may show up as a Hard Fault
   * under certain conditions.
   */

  irq_attach(RA6M5_IRQ_SVCALL, arm_svcall, NULL);
  irq_attach(RA6M5_IRQ_HARDFAULT, arm_hardfault, NULL);

  /* Set the priority of the SVCall interrupt */

#ifdef CONFIG_ARCH_IRQPRIO

  /* up_prioritize_irq(RA6M5_IRQ_PENDSV, NVIC_SYSH_PRIORITY_MIN); */

#endif
#ifdef CONFIG_ARMV8M_USEBASEPRI
  ra6m5_prioritize_syscall(NVIC_SYSH_SVCALL_PRIORITY);
#endif

  /* If the MPU is enabled, then attach and enable the Memory Management
   * Fault handler.
   */

#ifdef CONFIG_ARM_MPU
  irq_attach(RA6M5_IRQ_MEMFAULT, arm_memfault, NULL);
  up_enable_irq(RA6M5_IRQ_MEMFAULT);
#endif

  /* Attach all other processor exceptions (except reset and sys tick) */

#ifdef CONFIG_DEBUG_FEATURES
  irq_attach(RA6M5_IRQ_NMI, ra6m5_nmi, NULL);
#ifndef CONFIG_ARM_MPU
  irq_attach(RA6M5_IRQ_MEMFAULT, arm_memfault, NULL);
#endif
  irq_attach(RA6M5_IRQ_BUSFAULT, arm_busfault, NULL);
  irq_attach(RA6M5_IRQ_USAGEFAULT, arm_usagefault, NULL);
  irq_attach(RA6M5_IRQ_PENDSV, ra6m5_pendsv, NULL);
  irq_attach(RA6M5_IRQ_DBGMONITOR, ra6m5_dbgmonitor, NULL);
  irq_attach(RA6M5_IRQ_RESERVED, ra6m5_reserved, NULL);
#endif

  ra6m5_dumpnvic("initial", NR_IRQS);

#ifndef CONFIG_SUPPRESS_INTERRUPTS

  /* And finally, enable interrupts */

  up_irq_enable();
#endif
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
  uintptr_t regaddr;
  uint32_t regval;
  uint32_t bit;

  if (ra6m5_irqinfo(irq, &regaddr, &bit, NVIC_CLRENA_OFFSET) == 0)
    {
      /* Modify the appropriate bit in the register to disable the interrupt.
       * For normal interrupts, we need to set the bit in the associated
       * Interrupt Clear Enable register.  For other exceptions, we need to
       * clear the bit in the System Handler Control and State Register.
       */

      if (irq >= RA6M5_IRQ_FIRST)
        {
          putreg32(bit, regaddr);
        }
      else
        {
          regval  = getreg32(regaddr);
          regval &= ~bit;
          putreg32(regval, regaddr);
        }
    }
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
  uintptr_t regaddr;
  uint32_t regval;
  uint32_t bit;

  if (ra6m5_irqinfo(irq, &regaddr, &bit, NVIC_ENA_OFFSET) == 0)
    {
      /* Modify the appropriate bit in the register to enable the interrupt.
       * For normal interrupts, we need to set the bit in the associated
       * Interrupt Set Enable register.  For other exceptions, we need to
       * set the bit in the System Handler Control and State Register.
       */

      if (irq >= RA6M5_IRQ_FIRST)
        {
          putreg32(bit, regaddr);
        }
      else
        {
          regval  = getreg32(regaddr);
          regval |= bit;
          putreg32(regval, regaddr);
        }
    }
}

/****************************************************************************
 * Name: arm_ack_irq
 *
 * Description:
 *   Acknowledge the IRQ
 *
 ****************************************************************************/

void arm_ack_irq(int irq)
{
  int n;

  /* Check for external interrupt */

  if (irq >= RA6M5_IRQ_FIRST) 
    {
      n = irq - RA6M5_IRQ_FIRST;

      /* Clear the IR bit in the selected IELSR register */
      modreg32(0, ICU_IELSR_IR, RA6M5_ICU_IELSR_REG(n));

      /* Read back the IELSR register to ensure that the IR bit is cleared */
     __asm__ volatile ("" : : "r" (RA6M5_ICU_IELSR_REG(n)));

      /* Flush memory transactions to ensure that the IR bit is cleared before clearing the pending bit in the NVIC */
      __asm__ volatile ("dmb 0xF":::"memory");
    }
}

/****************************************************************************
 * Name: up_prioritize_irq
 *
 * Description:
 *   Set the priority of an IRQ.
 *
 *   Since this API is not supported on all architectures, it should be
 *   avoided in common implementations where possible.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQPRIO
int up_prioritize_irq(int irq, int priority)
{
  uint32_t regaddr;
  uint32_t regval;
  int shift;

  DEBUGASSERT(irq >= RA6M5_IRQ_MEMFAULT && irq < NR_IRQS &&
              (unsigned)priority <= NVIC_SYSH_PRIORITY_MIN);

  if (irq < RA6M5_IRQ_FIRST)
    {
      /* NVIC_SYSH_PRIORITY() maps {0..15} to one of three priority
       * registers (0-3 are invalid)
       */

      regaddr = NVIC_SYSH_PRIORITY(irq);
      irq    -= 4;
    }
  else
    {
      /* NVIC_IRQ_PRIORITY() maps {0..} to one of many priority registers */

      irq    -= RA6M5_IRQ_FIRST;
      regaddr = NVIC_IRQ_PRIORITY(irq);
    }

  regval      = getreg32(regaddr);
  shift       = ((irq & 3) << 3);
  regval     &= ~(0xff << shift);
  regval     |= (priority << shift);
  putreg32(regval, regaddr);

  ra6m5_dumpnvic("prioritize", irq);
  return OK;
}
#endif
