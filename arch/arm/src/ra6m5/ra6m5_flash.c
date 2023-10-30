/****************************************************************************
 * arch/arm/src/ra6m5/ra6m5_flash.c
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

/* Provides standard flash access functions, to be used by the flash mtd
 * driver.  The interface is defined in the include/nuttx/progmem.h
 *
 * Notes about this implementation:
 *  - HSI16 is automatically turned ON by MCU, if not enabled beforehand
 *  - Only Standard Programming is supported, no Fast Programming.
 *  - Low Power Modes are not permitted during write/erase
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/progmem.h>

#include <semaphore.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <string.h>
#include <sys/param.h>
#include <nuttx/mutex.h>

#include "ra6m5_rcc.h"
#include "ra6m5_waste.h"
#include "ra6m5_flash.h"
#include "arm_internal.h"

#if !defined(CONFIG_RA6M5_R7FA6M5BX)
#  error "Unrecognized RA6M5 chip"
#endif

#if !defined(CONFIG_RA6M5_FLASH_OVERRIDE_DEFAULT)
#  warning "Flash Configuration has been overridden - make sure it is correct"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void ra6m5_flash_unlock(void)
{
}

void ra6m5_flash_lock(void)
{
}

/****************************************************************************
 * Name: ra6m5_flash_user_optbytes
 *
 * Description:
 *   Modify the contents of the user option bytes (USR OPT) on the flash.
 *   This does not set OBL_LAUNCH so new options take effect only after
 *   next power reset.
 *
 * Input Parameters:
 *   clrbits - Bits in the option bytes to be cleared
 *   setbits - Bits in the option bytes to be set
 *
 * Returned Value:
 *   Option bytes after operation is completed
 *
 ****************************************************************************/

uint32_t ra6m5_flash_user_optbytes(uint32_t clrbits, uint32_t setbits)
{
  return 0;
}

size_t up_progmem_pagesize(size_t page)
{
  return 0;
}

size_t up_progmem_erasesize(size_t block)
{
  return 0;
}

ssize_t up_progmem_getpage(size_t addr)
{
  return 0;
}

size_t up_progmem_getaddress(size_t page)
{
  return 0;
}

size_t up_progmem_neraseblocks(void)
{
  return 0;
}

bool up_progmem_isuniform(void)
{
  return true;
}

ssize_t up_progmem_eraseblock(size_t block)
{
  return 0;
}

ssize_t up_progmem_ispageerased(size_t page)
{
  return 0;
}

ssize_t up_progmem_write(size_t addr, const void *buf, size_t buflen)
{
  return 0;
}

/****************************************************************************
 * Name: up_progmem_erasestate
 *
 * Description:
 *   Return value of erase state.
 *
 ****************************************************************************/

uint8_t up_progmem_erasestate(void)
{
  return 0;
}
