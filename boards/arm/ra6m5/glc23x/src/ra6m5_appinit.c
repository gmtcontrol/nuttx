/****************************************************************************
 * boards/arm/ra6m5/glc23x/src/ra6m5_appinit.c
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

#include <errno.h>
#include <stdint.h>

#include <nuttx/board.h>

#include "ra6m5_uid.h"
#include "glc23x.h"


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application specific initialization.  This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initialization logic and the
 *         matching application logic.  The value cold be such things as a
 *         mode enumeration value, a set of DIP switch switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int board_app_initialize(uintptr_t arg)
{
  /* Did we already initialize via board_late_initialize()? */

#ifndef CONFIG_BOARD_LATE_INITIALIZE
  return ra6m5_bringup();
#else
  return OK;
#endif
}

/****************************************************************************
 * Name: board_ioctl
 *
 * Description:
 *   If CONFIG_BOARDCTL=y, boards may also select CONFIG_BOARDCTL_IOCTL=y
 *   enable board specific commands.  In this case, all commands not
 *   recognized by boardctl() will be forwarded to the board-provided
 *   board_ioctl() function.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARDCTL_IOCTL
int board_ioctl(unsigned int cmd, uintptr_t arg)
{
  switch (cmd)
  {
    case BOARDIOC_MKRD:
      break;
  }

  return -ENOTTY;
}
#endif

/****************************************************************************
 * Name: board_uniqueid
 *
 * Description:
 *   Return a unique ID associated with the board.  The meaning of this
 *   unique ID is not specified.  It may be a chip identifying number, a
 *   serial number, a MAC address, etc.  It may be in binary or it may be
 *   ASCII.  The only only requirement is that the length of the unique
 *   ID be exactly CONFIG_BOARDCTL_UNIQUEID_SIZE in length.
 *
 * Input Parameters:
 *   uniqueid - A reference to a writable memory location provided by the
 *     caller to receive the board unique ID.  The memory memory referenced
 *     by this pointer must be at least CONFIG_BOARDCTL_UNIQUEID_SIZE in
 *     length.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned indicating the nature of the failure.
 *
 ****************************************************************************/
#if defined(CONFIG_BOARDCTL_UNIQUEID)
int board_uniqueid(uint8_t *uniqueid)
{
  if (uniqueid == NULL)
    {
      return -EINVAL;
    }

  ra6m5_get_uniqueid(uniqueid);
  
  return OK;
}
#endif
