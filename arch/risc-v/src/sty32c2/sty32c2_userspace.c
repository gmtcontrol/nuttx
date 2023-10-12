/****************************************************************************
 * arch/risc-v/src/sty32c2/sty32c2_userspace.c
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

#include <nuttx/userspace.h>

#include <arch/board/board_mmap.h>

#include "sty32c2_userspace.h"
#include "riscv_internal.h"
#ifdef CONFIG_ARCH_HAVE_MMU
#include "riscv_mmu.h"
#endif

#ifdef CONFIG_BUILD_PROTECTED

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Physical and virtual addresses to page tables (vaddr = paddr mapping) */

#define PGT_L1_PBASE    (uint32_t)&m_l1_pgtable
#define PGT_L2_PBASE    (uint32_t)&m_l2_pgtable
#define PGT_L3_ROMPBASE (uint32_t)&m_l3_romtbl
#define PGT_L3_RAMPBASE (uint32_t)&m_l3_ramtbl
#define PGT_L1_VBASE    PGT_L1_PBASE
#define PGT_L2_VBASE    PGT_L2_PBASE
#define PGT_L3_ROMVBASE PGT_L3_ROMPBASE
#define PGT_L3_RAMVBASE PGT_L3_RAMPBASE

#define PGT_L1_SIZE     (1024)  /* Enough to map 4 GiB */
#define PGT_L2_SIZE     (1024)  /* Enough to map 4 MiB */
#define PGT_L3_SIZE     (1024)  /* Enough to map 4 MiB */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: configure_mpu
 *
 * Description:
 *   This function configures the MPU for for kernel- / userspace separation.
 *   It will also grant access to the page table memory for the supervisor.
 *
 ****************************************************************************/

static void configure_mpu(void);

/****************************************************************************
 * Name: configure_mmu
 *
 * Description:
 *   This function configures the MMU and page tables for kernel- / userspace
 *   separation.
 *
 ****************************************************************************/

static void configure_mmu(void);

/****************************************************************************
 * Name: map_region
 *
 * Description:
 *   Map a region of physical memory to the L2 page table
 *
 * Input Parameters:
 *   l2base - L2 page table physical base address
 *   paddr - Beginning of the physical address mapping
 *   vaddr - Beginning of the virtual address mapping
 *   size - Size of the region in bytes
 *   mmuflags - The MMU flags to use in the mapping
 *
 ****************************************************************************/

static void map_region(uintptr_t l2base, uintptr_t paddr, uintptr_t vaddr,
                       size_t size, uint32_t mmuflags);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* With a 2 level page table setup the total available memory is 512GB.
 * However, this is overkill. A single L3 page table can map 2MB of memory,
 * and for MPFS, this user space is plenty enough. If more memory is needed,
 * simply increase the size of the L3 page table (n * 512), where each 'n'
 * provides 2MB of memory.
 */

/* L1 table must be in memory always for this to work */

static uint32_t         m_l1_pgtable[PGT_L1_SIZE] locate_data(".pgtables");
static uint32_t         m_l2_pgtable[PGT_L2_SIZE] locate_data(".pgtables");

/* Allocate separate tables for ROM/RAM mappings */

static uint32_t         m_l3_romtbl[PGT_L3_SIZE]  locate_data(".pgtables");
static uint32_t         m_l3_ramtbl[PGT_L3_SIZE]  locate_data(".pgtables");

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sty32c2_userspace
 *
 * Description:
 *   For the case of the separate user-/kernel-space build, perform whatever
 *   platform specific initialization of the user memory is required.
 *   Normally this just means initializing the user space .data and .bss
 *   segments.
 *
 ****************************************************************************/

void sty32c2_userspace(void)
{
  uint8_t *src;
  uint8_t *dest;
  uint8_t *end;

  /* Clear all of user-space .bss */

  DEBUGASSERT(USERSPACE->us_bssstart != 0 && USERSPACE->us_bssend != 0 &&
              USERSPACE->us_bssstart <= USERSPACE->us_bssend);

  dest = (uint8_t *)USERSPACE->us_bssstart;
  end  = (uint8_t *)USERSPACE->us_bssend;

  while (dest != end)
    {
      *dest++ = 0;
    }

  /* Initialize all of user-space .data */

  DEBUGASSERT(USERSPACE->us_datasource != 0 &&
              USERSPACE->us_datastart != 0 && USERSPACE->us_dataend != 0 &&
              USERSPACE->us_datastart <= USERSPACE->us_dataend);

  src  = (uint8_t *)USERSPACE->us_datasource;
  dest = (uint8_t *)USERSPACE->us_datastart;
  end  = (uint8_t *)USERSPACE->us_dataend;

  while (dest != end)
    {
      *dest++ = *src++;
    }

  /* Configure MPU / PMP to grant access to the userspace */

  configure_mpu();
  configure_mmu();
}

/****************************************************************************
 * Name: configure_mpu
 *
 * Description:
 *   This function configures the MPU for for kernel- / userspace separation.
 *   It will also grant access to the page table memory for the supervisor.
 *
 ****************************************************************************/

static void configure_mpu(void)
{
}

/****************************************************************************
 * Name: configure_mmu
 *
 * Description:
 *   This function configures the MMU and page tables for kernel- / userspace
 *   separation.
 *
 ****************************************************************************/

static void configure_mmu(void)
{
}

/****************************************************************************
 * Name: map_region
 *
 * Description:
 *   Map a region of physical memory to the L3 page table
 *
 * Input Parameters:
 *   l2base - L2 page table physical base address
 *   paddr - Beginning of the physical address mapping
 *   vaddr - Beginning of the virtual address mapping
 *   size - Size of the region in bytes
 *   mmuflags - The MMU flags to use in the mapping
 *
 ****************************************************************************/

static void map_region(uintptr_t l2base, uintptr_t paddr, uintptr_t vaddr,
                       size_t size, uint32_t mmuflags)
{
  uintptr_t end_vaddr;

  /* Map the region to the L2 table as a whole */

  mmu_ln_map_region(2, l2base, paddr, vaddr, size, mmuflags);

  /* Connect to L1 table */

  end_vaddr = vaddr + size;
  while (vaddr < end_vaddr)
    {
      mmu_ln_setentry(1, PGT_L1_VBASE, l2base, vaddr, PTE_G);
      l2base += RV_MMU_L2_PAGE_SIZE;
      vaddr += RV_MMU_L1_PAGE_SIZE;
    }
}

#endif /* CONFIG_BUILD_PROTECTED */
