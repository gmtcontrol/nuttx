/****************************************************************************
 * boards/arm/ra6m5/ek-ra6m5/src/ra6m5_w25.c
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

#include <stdbool.h>
#include <stdio.h>
#include <errno.h>
#include <debug.h>

#if defined(CONFIG_RA6M5_QSPI0)
#  include <nuttx/spi/spi.h>
#  include <nuttx/mtd/mtd.h>
#  include <nuttx/fs/fs.h>
#  include <nuttx/fs/nxffs.h>
#endif

#include "ra6m5_qspi.h"
#include "ek-ra6m5.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Can't support the MX25 device if it SPI1 or MX25 support is not enabled */

#undef HAVE_MX25

#if defined(CONFIG_RA6M5_QSPI0)
#define HAVE_MX25  1
#endif

/* Can't support MX25 features if mountpoints are disabled */

#if defined(CONFIG_DISABLE_MOUNTPOINT)
#  undef HAVE_MX25
#endif

/* Can't support both FAT and NXFFS */

#if defined(CONFIG_FS_FAT) && defined(CONFIG_FS_NXFFS)
#  warning "Can't support both FAT and NXFFS -- using FAT"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra6m5_mx25_initialize
 *
 * Description:
 *   Initialize and register the MX25R FLASH file system.
 *
 ****************************************************************************/

int ra6m5_mx25_initialize(int minor)
{
#ifdef HAVE_MX25
  struct qspi_dev_s *qspi;
  struct mtd_dev_s *mtd;
#ifdef CONFIG_FS_NXFFS
  char devname[12];
#endif
  int ret;

  /* Create an instance of the QSPI device driver */

  qspi = ra6m5_qspi_initialize(minor);
  if (!qspi)
    {
      ferr("ERROR: Failed to initialize QSPI port\n");
      return -ENODEV;
    }

  /* Now bind the SPI interface to the MX25 SPI FLASH driver */

  mtd = mx25rxx_initialize(qspi, true);
  if (!mtd)
    {
      ferr("ERROR: Failed to bind QSPI port to the MX25R FLASH driver\n");
      return -ENODEV;
    }

#ifndef CONFIG_FS_NXFFS
  /* And use the FTL layer to wrap the MTD driver as a block driver */

  ret = ftl_initialize(minor, mtd);
  if (ret < 0)
    {
      ferr("ERROR: Initialize the FTL layer\n");
      return ret;
    }
#else
  /* Initialize to provide NXFFS on the MTD interface */

  ret = nxffs_initialize(mtd);
  if (ret < 0)
    {
      ferr("ERROR: NXFFS initialization failed: %d\n", -ret);
      return ret;
    }

  /* Mount the file system at /mnt/sdx */

  snprintf(devname, 12, "/mnt/sd%c", 'a' + minor);
  ret = nx_mount(NULL, devname, "nxffs", 0, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Failed to mount the NXFFS volume: %d\n", ret);
      return ret;
    }
#endif
#endif

  return OK;
}
