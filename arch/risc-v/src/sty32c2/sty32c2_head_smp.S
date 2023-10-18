/****************************************************************************
 * arch/risc-v/src/sty32c2/sty32c2_head_smp.S
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
#include <arch/irq.h>

#include "chip.h"
#include "sty32c2_memorymap.h"
#include "riscv_internal.h"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

  /* Imported symbols */

  .extern __trap_vec

  .section .text
  .global __start

__start:

  /* Disable all interrupts (i.e. timer, external) in mie */

  csrw mie, zero
  csrw mip, zero

  /* Initialize the Machine Trap Vector */

  la   t0, __trap_vec
  csrw mtvec, t0

  /* Make sure that mtvec is updated before continuing */

1:
  csrr t1, mtvec
  bne  t0, t1, 1b

  /* mscratch must be init to zero- we are not using scratch memory */

  csrw mscratch, zero
  csrw mepc, zero
  li   x1,  0
  li   x2,  0
  li   x3,  0
  li   x4,  0
  li   x5,  0
  li   x6,  0
  li   x7,  0
  li   x8,  0
  li   x9,  0
  li   x10, 0
  li   x11, 0
  li   x12, 0
  li   x13, 0
  li   x14, 0
  li   x15, 0
  li   x16, 0
  li   x17, 0
  li   x18, 0
  li   x19, 0
  li   x20, 0
  li   x21, 0
  li   x22, 0
  li   x23, 0
  li   x24, 0
  li   x25, 0
  li   x26, 0
  li   x27, 0
  li   x28, 0
  li   x29, 0
  li   x30, 0
  li   x31, 0

  /* Delegation register and mmu initializations */

  csrr a0, mhartid

  /* Delegation registers must be explicitly reset */

  csrw mideleg, 0
  csrw medeleg, 0

  /* Remove MMU mappings (if any) */

  csrw satp, zero
  fence

  /* Flush TLB (does not make a difference really) */

  sfence.vma x0, x0

  /* To get g_cpu_basestack[mhartid], must get g_cpu_basestack first */

  la   t0, g_cpu_basestack

  /* Offset = pointer width (4-byte) * hart id */

  slli t1, a0, 2
  add  t0, t0, t1

  /* Load idle stack base to sp */

  REGLOAD  sp, 0(t0)

  /*
   * sp (stack top) = sp + idle stack size - XCPTCONTEXT_SIZE
   * 
   * Note: Reserve some space used by up_initial_state since we are already
   * running and using the per CPU idle stack.
   */

  li   t0, STACK_ALIGN_UP(CONFIG_IDLETHREAD_STACKSIZE - XCPTCONTEXT_SIZE)
  add  sp, sp, t0

  /* Jump to __sty32c2_start with mhartid in a0 */

  jal  __sty32c2_start

  /* We shouldn't return from __sty32c2_start
   * in case of return, loop forever. nop's added so can be seen in debugger
   */

1:
  nop
  nop
  j 1b

  .global _init
  .global _fini

_init:
_fini:

  /* These don't have to do anything since we use init_array/fini_array. */

  ret
