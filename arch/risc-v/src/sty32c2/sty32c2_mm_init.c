/****************************************************************************
 * arch/risc-v/src/sty32c2/sty32c2_mm_init.c
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
#include <nuttx/arch.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <arch/board/board_mmap.h>

#include "sty32c2_memorymap.h"

#include "riscv_internal.h"
#include "riscv_mmu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Map the whole I/O memory with vaddr = paddr mappings */

#define MMU_IO_BASE     (0x80000000)
#define MMU_IO_SIZE     (0x80000000)

/* Physical and virtual addresses to page tables (vaddr = paddr mapping) */

#define PGT_L1_PBASE    (uintptr_t)&m_l1_pgtable
#define PGT_L2_PBASE    (uintptr_t)&m_l2_pgtable
#define PGT_L1_VBASE    PGT_L1_PBASE
#define PGT_L2_VBASE    PGT_L2_PBASE

#define PGT_L1_SIZE     (2048) /* Enough to map 8 GiB */
#define PGT_L2_SIZE     (2048) /* Enough to map 8 MiB */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Kernel mappings simply here, mapping is vaddr=paddr */

static uint32_t         m_l1_pgtable[PGT_L1_SIZE] locate_data(".pgtables");
static uint32_t         m_l2_pgtable[PGT_L2_SIZE] locate_data(".pgtables");

/* Kernel mappings (L1 base) */

uintptr_t               g_kernel_mappings  = PGT_L1_VBASE;
uintptr_t               g_kernel_pgt_pbase = PGT_L1_PBASE;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sty32c2_kernel_mappings
 *
 * Description:
 *  Setup kernel mappings when usinc CONFIG_BUILD_KERNEL. Sets up the kernel
 *  MMU mappings.
 *
 ****************************************************************************/

void sty32c2_kernel_mappings(void)
{
  /* Begin mapping memory to MMU; note that at this point the MMU is not yet
   * active, so the page table virtual addresses are actually physical
   * addresses and so forth. M-mode does not perform translations anyhow, so
   * this mapping is quite simple to do
   */

  /* Map I/O region, use 2 L1 entries (i.e. 2 * 1GB address space) */

  mmu_ln_map_region(1, PGT_L1_VBASE, MMU_IO_BASE, MMU_IO_BASE, MMU_IO_SIZE, MMU_IO_FLAGS);

  /* Map the kernel text and data */

  mmu_ln_map_region(2, PGT_L2_VBASE, KFLASH_START, KFLASH_START, KFLASH_SIZE, MMU_KTEXT_FLAGS);
  mmu_ln_map_region(2, PGT_L2_VBASE, KSRAM_START,  KSRAM_START,  KSRAM_SIZE,  MMU_KDATA_FLAGS);

  /* Connect the L1 and L2 page tables */

  mmu_ln_setentry(1, PGT_L1_VBASE, PGT_L2_PBASE, KFLASH_START, PTE_G);

  /* Map the page pool */

  mmu_ln_map_region(2, PGT_L2_VBASE, PGPOOL_START, PGPOOL_START, PGPOOL_SIZE, MMU_KDATA_FLAGS);
}
