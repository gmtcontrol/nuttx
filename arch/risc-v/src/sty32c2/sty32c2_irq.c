/****************************************************************************
 * arch/risc-v/src/sty32c2/sty32c2_irq.c
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
#include <stdio.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "riscv_internal.h"
#include "sty32c2.h"
#include "sty32c2_plic.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

//volatile uintptr_t *g_current_regs[CONFIG_SMP_NCPUS];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  /* Disable Machine interrupts */

  up_irq_save();

  /* Disable all global interrupts for current hart */

  uintptr_t iebase = sty32c2_plic_get_iebase();
  putreg32(0, iebase);

  /* Clear pendings in PLIC (for current hart) */

  uintptr_t claim_address = sty32c2_plic_get_claimbase();
  uint32_t val = getreg32(claim_address);
  putreg32(val, claim_address);

  /* Colorize the interrupt stack for debug purposes */

#if defined(CONFIG_STACK_COLORATION) && CONFIG_ARCH_INTERRUPTSTACK > 15
  size_t intstack_size = (CONFIG_ARCH_INTERRUPTSTACK & ~15);
  riscv_stack_color((void *)&g_intstackalloc, intstack_size);
#endif

   /* Set priority for all global interrupts to 1 (lowest) */

  int id;

  for (id = 1; id <= NR_IRQS; id++)
    {
      putreg32(1, (uintptr_t)STY32C2_PLIC_PRIORITY(id));
    }

  /* Set irq threshold to 0 (permits all global interrupts) */

  uintptr_t threshold_address = sty32c2_plic_get_thresholdbase();
  putreg32(0, threshold_address);

  /* currents_regs is non-NULL only while processing an interrupt */

  CURRENT_REGS = NULL;

  /* Attach the common interrupt handler */

  riscv_exception_attach();

#ifdef CONFIG_SMP
  /* Clear MSOFT for CPU0 */

  putreg32(0, RISCV_CLINT_MSIP);

  up_enable_irq(RISCV_IRQ_MSOFT);
#endif

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
  int extirq;

  if (irq == RISCV_IRQ_MSOFT)
    {
      /* Read mstatus & clear machine software interrupt enable in mie */

      CLEAR_CSR(mie, MIE_MSIE);
    }
  else if (irq == RISCV_IRQ_MTIMER)
    {
      /* Read mstatus & clear machine timer interrupt enable in mie */

      CLEAR_CSR(mie, MIE_MTIE);
    }
  else if (irq > RISCV_IRQ_MEXT)
    {
      extirq = irq - RISCV_IRQ_MEXT;

      /* Clear enable bit for the irq */

      uintptr_t iebase = sty32c2_plic_get_iebase();

      if (0 <= extirq && extirq <= 31)
        {
          modifyreg32(iebase + (4 * (extirq / 32)), 1 << (extirq % 32), 0);
        }
      else
        {
          ASSERT(false);
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
  int extirq;

  if (irq == RISCV_IRQ_MSOFT)
    {
      /* Read mstatus & set machine software interrupt enable in mie */

      SET_CSR(mie, MIE_MSIE);
    }
  else if (irq == RISCV_IRQ_MTIMER)
    {
      /* Read mstatus & set machine timer interrupt enable in mie */

      SET_CSR(mie, MIE_MTIE);
    }
  else if (irq > RISCV_IRQ_MEXT)
    {
      extirq = irq - RISCV_IRQ_MEXT;

      /* Set enable bit for the irq */

      uintptr_t iebase = sty32c2_plic_get_iebase();

      if (0 <= extirq && extirq <= 31)
        {
          modifyreg32(iebase + (4 * (extirq / 32)), 0, 1 << (extirq % 32));
        }
      else
        {
          ASSERT(false);
        }
    }
}

/****************************************************************************
 * Name: riscv_ack_irq
 *
 * Description:
 *   Acknowledge the IRQ
 *
 ****************************************************************************/

void riscv_ack_irq(int irq)
{
}

/****************************************************************************
 * Name: up_irq_enable
 *
 * Description:
 *   Return the current interrupt state and enable interrupts
 *
 ****************************************************************************/

irqstate_t up_irq_enable(void)
{
  irqstate_t oldstat;

  /* Enable MEIE (machine external interrupt enable) */

  SET_CSR(mie, MIE_MEIE);

  /* Read mstatus & set machine interrupt enable (MIE) in mstatus */

  oldstat = READ_AND_SET_CSR(mstatus, MSTATUS_MIE);
  return oldstat;
}
