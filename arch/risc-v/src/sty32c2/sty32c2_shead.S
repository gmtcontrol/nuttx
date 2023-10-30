/****************************************************************************
 * arch/risc-v/src/sty32c2/sty32c2_head.S
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

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

  /* Imported symbols */

  .extern __trap_vec

  .section .text
  .global __start

/****************************************************************************
 * Name: __start
 *
 * Description:
 *   Supervisor mode start function.
 *
 * Input Parameters:
 *    a0 - hartid
 *
 ****************************************************************************/

__start:

  /* Disable all interrupts in sie */

  csrw sie, zero
  csrw sip, zero

  /* Set the S-mode trap vector */

  la   t0, __trap_vec
  csrw stvec, t0

  /* Clear sscratch */

  csrw sscratch, zero
  csrw scause, zero
  csrw sepc, zero

  /* initialize global pointer, global data */

.option push
.option norelax
  la  gp, __global_pointer$
.option pop

  /* Make sure the writes to CSR stick before continuing */

  fence

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

  j __sty32c2_start
