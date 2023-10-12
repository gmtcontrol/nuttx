/****************************************************************************
 * arch/risc-v/src/sty32c2/sty32c2_plic.c
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
#include <arch/irq.h>

#include "sty32c2.h"
#include "sty32c2_plic.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sty32c2_plic_get_iebase
 *
 * Description:
 *   Context aware way to query PLIC interrupt enable base address
 *
 * Returned Value:
 *   Interrupt enable base address
 *
 ****************************************************************************/

uintptr_t sty32c2_plic_get_iebase(void)
{
  uintptr_t hart_id = riscv_mhartid();
  uintptr_t iebase = STY32C2_PLIC_ENABLE(hart_id, PLIC_MODE_MACHINE);
 
  return iebase;
}

/****************************************************************************
 * Name: sty32c2_plic_get_claimbase
 *
 * Description:
 *   Context aware way to query PLIC interrupt claim base address
 *
 * Returned Value:
 *   Interrupt enable claim address
 *
 ****************************************************************************/

uintptr_t sty32c2_plic_get_claimbase(void)
{
  uintptr_t hart_id = riscv_mhartid();
  uintptr_t claim_address = STY32C2_PLIC_CLAIM(hart_id, PLIC_MODE_MACHINE);

  return claim_address;
}

/****************************************************************************
 * Name: sty32c2_plic_get_thresholdbase
 *
 * Description:
 *   Context aware way to query PLIC interrupt threshold base address
 *
 * Returned Value:
 *   Interrupt enable threshold address
 *
 ****************************************************************************/

uintptr_t sty32c2_plic_get_thresholdbase(void)
{
  uintptr_t hart_id = riscv_mhartid();
  uintptr_t threshold_address = STY32C2_PLIC_THRESHOLD(hart_id, PLIC_MODE_MACHINE);

  return threshold_address;
}
