/****************************************************************************
 * arch/risc-v/src/sty32c2/sty32c2_start.c
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

#include <nuttx/init.h>
#include <arch/board/board.h>

#include "chip.h"
#include "sty32c2.h"
#include "sty32c2_clockconfig.h"
#include "sty32c2_mm_init.h"
#include "sty32c2_userspace.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#  define showprogress(c) riscv_lowputc(c)
#else
#  define showprogress(c)
#endif

#if defined (CONFIG_BUILD_KERNEL) && !defined (CONFIG_ARCH_USE_S_MODE)
#  error "Target requires kernel in S-mode, enable CONFIG_ARCH_USE_S_MODE"
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_idle_topstack: _sbss is the start of the BSS region as defined by the
 * linker script. _ebss lies at the end of the BSS region. The idle task
 * stack starts at the end of BSS and is of size CONFIG_IDLETHREAD_STACKSIZE.
 * The IDLE thread is the thread that the system boots on and, eventually,
 * becomes the IDLE, do nothing task that runs only when there is nothing
 * else to run.  The heap continues from there until the end of memory.
 * g_idle_topstack is a read-only variable the provides this computed
 * address.
 */

uintptr_t g_idle_topstack = STY32C2_IDLESTACK_TOP;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __sty32c2_start
 ****************************************************************************/
#if defined(CONFIG_SMP)
void __sty32c2_start(uint32_t mhartid)
#else
void __sty32c2_start(void)
#endif
{
  const uint32_t *src;
  uint32_t *dest;

  /* Configure FPU */

  riscv_fpuconfig();

#if defined(CONFIG_SMP) && (CONFIG_SMP_NCPUS > 1)
  if (0 < mhartid)
    {
      goto cpux;
    }
#endif

  /* Clear .bss.  We'll do this inline (vs. calling memset) just to be
   * certain that there are no issues with the state of global variables.
   */

  for (dest = (uint32_t*)_sbss; dest < (uint32_t*)_ebss; )
    {
      *dest++ = 0;
    }

  /* Move the initialized data section from his temporary holding spot in
   * FLASH into the correct place in SRAM.  The correct place in SRAM is
   * give by _sdata and _edata.  The temporary location is in FLASH at the
   * end of all of the other read-only data (.text, .rodata) at _eronly.
   */

  /* for vexriscv the full image is loaded in ddr ram */

  for (src = (const uint32_t*)_eronly, dest = (uint32_t*)_sdata; dest < (uint32_t*)_edata; )
    {
      *dest++ = *src++;
    }

  /* Setup PLL */

  sty32c2_clockconfig();

  /* Configure the UART so we can get debug output */

  sty32c2_lowsetup();

  showprogress('A');

#ifdef USE_EARLYSERIALINIT
  riscv_earlyserialinit();
#endif

  showprogress('B');

  /* Do board initialization */

  sty32c2_boardinitialize();

  showprogress('C');

#ifdef CONFIG_ARCH_USE_S_MODE
  /* Initialize the per CPU areas */

  if (mhartid != 0)
    {
      riscv_percpu_add_hart(mhartid);
    }

  showprogress('D');
#endif /* CONFIG_ARCH_USE_S_MODE */

  /* For the case of the separate user-/kernel-space build, perform whatever
   * platform specific initialization of the user memory is required.
   * Normally this just means initializing the user space .data and .bss
   * segments.
   */

#ifdef CONFIG_BUILD_PROTECTED
  sty32c2_userspace();
  showprogress('E');
#endif

#ifdef CONFIG_BUILD_KERNEL
  sty32c2_mm_init();
  showprogress('F');
#endif

  /* Call nx_start() */

  nx_start();

#if defined(CONFIG_SMP) && (CONFIG_SMP_NCPUS > 1)
cpux:

  showprogress('a' + mhartid);

  riscv_cpu_boot(mhartid);
#endif

  while (true)
    {
      asm("WFI");
    }
}
