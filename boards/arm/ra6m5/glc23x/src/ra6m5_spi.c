/****************************************************************************
 * boards/arm/ra6m5/glc23x/src/ra6m5_spi.c
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
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/spi/spi.h>

#include "arm_internal.h"
#include "chip.h"
#include "ra6m5_gpio.h"
#include "ra6m5_spi.h"

#include "glc23x.h"
#include <arch/board/board.h>

#ifdef CONFIG_RA6M5_SPI

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra6m5_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the B-U585I-IOT02A
 *    board.
 *
 ****************************************************************************/

void ra6m5_spidev_initialize(void)
{
  /* NOTE: Clocking for SPI1 and/or SPI3 was already provided in ra6m5_rcc.c.
   *       Configurations of SPI pins is performed in ra6m5_spi.c.
   *       Here, we only initialize chip select pins unique to the board
   *       architecture.
   */

#ifdef CONFIG_RA6M5_SCI9_SPI
  ra6m5_configgpio(GPIO_SCI9_SEL);
#endif

}

/****************************************************************************
 * Name:  ra6m5_spi0/1select and ra6m5_spi0/1status
 *
 * Description:
 *   The external functions, ra6m5_spi1/2/3select and ra6m5_spi1/2/3status
 *   must be provided by board-specific logic.  They are implementations of
 *   the select and status methods of the SPI interface defined by struct
 *   spi_ops_s (see include/nuttx/spi/spi.h). All other methods (including
 *   ra6m5_spibus_initialize()) are provided by common STM32 logic.  To use
 *   this common SPI logic on your board:
 *
 *   1. Provide logic in ra6m5_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide ra6m5_spi1/2/3select() and ra6m5_spi1/2/3status() functions
 *      in your board-specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board is
 *      configured.
 *   3. Add a calls to ra6m5_spibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by ra6m5_spibus_initialize() may then be used
 *      to bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_RA6M5_SPI0
void ra6m5_spi0select(struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  spiinfo("devid: %08lx CS: %s\n",
          (unsigned long)devid, selected ? "assert" : "de-assert");

  ra6m5_gpiowrite(GPIO_SPI0_SEL, !selected);
}

uint8_t ra6m5_spi0status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_RA6M5_SPI1
void ra6m5_spi1select(struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  spiinfo("devid: %08lx CS: %s\n",
          (unsigned long)devid, selected ? "assert" : "de-assert");

  ra6m5_gpiowrite(GPIO_SPI1_SEL, !selected);
}

uint8_t ra6m5_spi1status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_RA6M5_SCI9_SPI
void ra6m5_scispi9select(struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  spiinfo("devid: %08lx CS: %s\n",
          (unsigned long)devid, selected ? "assert" : "de-assert");

  ra6m5_gpiowrite(GPIO_SCI9_SEL, !selected);
}

uint8_t ra6m5_scispi9status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

/****************************************************************************
 * Name: ra6m5_spi1cmddata
 *
 * Description:
 *   This function must be provided by platform-specific logic. This is an
 *   implementation of the cmddata method of the SPI interface defined by
 *   struct spi_ops_s (see include/nuttx/spi/spi.h).
 *
 * Input Parameters:
 *
 *   spi - SPI device that controls the bus the device that requires the CMD/
 *         DATA selection.
 *   devid - If there are multiple devices on the bus, this selects which one
 *         to select cmd or data.  NOTE:  This design restricts, for example,
 *         one one SPI display per SPI bus.
 *   cmd - true: select command; false: select data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
#ifdef CONFIG_RA6M5_SPI0
int ra6m5_spi0cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#ifdef CONFIG_RA6M5_SPI1
int ra6m5_spi1cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#ifdef CONFIG_RA6M5_SCI9_SPI
int ra6m5_scispi9cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#endif /* CONFIG_SPI_CMDDATA */
#endif /* CONFIG_RA6M5_SPI */
