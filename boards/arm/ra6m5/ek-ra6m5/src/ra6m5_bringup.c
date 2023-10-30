/****************************************************************************
 * boards/arm/ra6m5/ek-ra6m5/src/ra6m5_bringup.c
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

#include <sys/mount.h>
#include <sys/types.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/fs/fs.h>

#ifdef CONFIG_CDCACM
#  include <nuttx/usb/cdcacm.h>
#endif

#ifdef CONFIG_RA6M5_SPI
#  include <nuttx/spi/spi.h>
#  include <nuttx/mtd/mtd.h>
#  include "ra6m5_spi.h"
#endif

#ifdef CONFIG_RA6M5_IIC
#  include <nuttx/i2c/i2c_master.h>
#  include "ra6m5_iic.h"
#endif

#ifdef CONFIG_RA6M5_RTC
#  include <nuttx/timers/rtc.h>
#  include "ra6m5_rtc.h"
#endif

#ifdef CONFIG_RA6M5_RTC
#  include "ra6m5_dtc.h"
#endif

#include "ek-ra6m5.h"

/* Configuration ************************************************************/

/* Assume that we support everything until convinced otherwise */

#if (defined(CONFIG_RA6M5_SPI0)     || defined(CONFIG_RA6M5_SPI1)      || \
     defined(CONFIG_RA6M5_SCI0_SPI) || defined(CONFIG_RA6M5_SCI1_SPI)  || \
     defined(CONFIG_RA6M5_SCI2_SPI) || defined(CONFIG_RA6M5_SCI3_SPI)  || \
     defined(CONFIG_RA6M5_SCI4_SPI) || defined(CONFIG_RA6M5_SCI5_SPI)  || \
     defined(CONFIG_RA6M5_SCI6_SPI) || defined(CONFIG_RA6M5_SCI7_SPI)  || \
     defined(CONFIG_RA6M5_SCI8_SPI) || defined(CONFIG_RA6M5_SCI9_SPI)) && defined(CONFIG_MTD_W25)
#define HAVE_W25  1
#endif

/* Can't support W25 features if mountpoints are disabled */

#ifdef CONFIG_DISABLE_MOUNTPOINT
#  undef HAVE_W25
#endif

/* Default W25 minor number */

#if defined(HAVE_W25) && !defined(CONFIG_NSH_W25MINOR)
#  define CONFIG_NSH_W25MINOR 0
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rtc_driver_initialize
 *
 * Description:
 *   Initialize and register the RTC driver.
 *
 ****************************************************************************/

#ifdef CONFIG_RA6M5_RTC
static int rtc_driver_initialize(void)
{
  struct rtc_lowerhalf_s *lower;
  int ret;

  /* Instantiate the ra6m5 lower-half RTC driver */

  lower = ra6m5_rtc_lowerhalf();
  if (lower == NULL)
    {
      serr("ERROR: Failed to instantiate the RTC lower-half driver\n");
      ret = -ENOMEM;
    }
  else
    {
      /* Bind the lower half driver and register the combined RTC driver
       * as /dev/rtc0
       */

      ret = rtc_initialize(0, lower);
      if (ret < 0)
        {
          serr("ERROR: Failed to bind/register the RTC driver: %d\n", ret);
        }
    }

  return ret;
}
#endif /* CONFIG_RA6M5_RTC */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra6m5_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int ra6m5_bringup(void)
{
  int ret;

#ifdef HAVE_W25
  /* Initialize and register the W25 FLASH file system. */

  ret = ra6m5_w25initialize(CONFIG_NSH_W25MINOR);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize W25 minor %d: %d\n",
             CONFIG_NSH_W25MINOR, ret);
      return ret;
    }
#endif

#ifdef CONFIG_MTD_MX25RXX
  /* Initialize the MX25R QSPI memory */

  ret = ra6m5_mx25_initialize(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize MX25R: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_RA6M5_IIC
  struct i2c_master_s *i2c = NULL;

  /* Get the I2C lower half instance */
#ifdef CONFIG_RA6M5_IIC0
  i2c = ra6m5_i2cbus_initialize(0);

  if (i2c == NULL)
    {
      i2cerr("ERROR: Initialization of IIC Channel 0 failed: %d\n", ret);
    }

  else
    {
      /* Register the I2C character driver */

      ret = i2c_register(i2c, 0);
      if (ret < 0)
        {
          i2cerr("ERROR: Failed to register IIC device: %d\n", ret);
        }
    }
#endif

#ifdef CONFIG_RA6M5_IIC1
  i2c = ra6m5_i2cbus_initialize(1);

  if (i2c == NULL)
    {
      i2cerr("ERROR: Initialization of IIC Channel 0 failed: %d\n", ret);
    }

  else
    {
      /* Register the I2C character driver */

      ret = i2c_register(i2c, 0);
      if (ret < 0)
        {
          i2cerr("ERROR: Failed to register IIC device: %d\n", ret);
        }
    }
#endif

#ifdef CONFIG_RA6M5_IIC2
  i2c = ra6m5_i2cbus_initialize(2);

  if (i2c == NULL)
    {
      i2cerr("ERROR: Initialization of IIC Channel 0 failed: %d\n", ret);
    }

  else
    {
      /* Register the I2C character driver */

      ret = i2c_register(i2c, 0);
      if (ret < 0)
        {
          i2cerr("ERROR: Failed to register IIC device: %d\n", ret);
        }
    }
#endif
#endif /* CONFIG_RA6M5_IIC */

#ifdef CONFIG_RA6M5_RTC
  ret = rtc_driver_initialize();
  if (ret < 0)
    {
      rtcerr("ERROR: rtc_driver_initialize failed: %d\n", ret);
    }
#endif /* CONFIG_RA6M5_RTC */

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#ifdef CONFIG_RA6M5_DTC
  /* Initialize DTC */

  ra6m5_dtc_initialize();
#endif /* CONFIG_RA6M5_DTC */

#if defined(CONFIG_USBHOST)
  ret = nsh_usbhostinitialize();
#endif

#if defined(CONFIG_CDCACM) && !defined(CONFIG_CDCACM_CONSOLE)
  /* Initialize CDCACM */

  syslog(LOG_INFO, "Initialize CDCACM device\n");

  ret = cdcacm_initialize(0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: cdcacm_initialize failed: %d\n", ret);
    }
#endif /* CONFIG_CDCACM & !CONFIG_CDCACM_CONSOLE */

  UNUSED(ret);
  return OK;
}
