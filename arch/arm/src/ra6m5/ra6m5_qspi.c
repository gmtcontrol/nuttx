/****************************************************************************
 * arch/arm/src/ra6m5/ra6m5_qspi.c
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
 * The external functions, ra6m5_spi1/2/3/4/5/6select and
 * ra6m5_spi1/2/3/4/5/6status must be provided by board-specific logic.  They
 * are implementations of the select and status methods of the SPI interface
 * defined by struct spi_ops_s (see include/nuttx/spi/spi.h).  All other
 * methods (including ra6m5_spibus_initialize()) are provided by common STM32
 * logic.  To use this common SPI logic on your board:
 *
 *   1. Provide logic in ra6m5_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide ra6m5_spi1/2/3/4/5/6select() and ra6m5_spi1/2/3/4/5/6status()
 *      functions in your board-specific logic.  These functions will perform
 *      chip selection and status operations using GPIOs in the way your
 *      board is configured.
 *   3. Add a calls to ra6m5_spibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by ra6m5_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/compiler.h>
#include <nuttx/mutex.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "ra6m5_rcc.h"
#include "ra6m5_gpio.h"
#include "ra6m5_qspi.h"
#include "hardware/ra6m5_qspi.h"

#if defined(CONFIG_RA6M5_QSPI0)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* QSPI frequency limits */

#define RA6M5_QSPI_FREQMAX  (g_clock_freq[RA6M5_CLOCKS_SOURCE_PCLKA] / 2)
#define RA6M5_QSPI_FREQMIN  (g_clock_freq[RA6M5_CLOCKS_SOURCE_PCLKA] / 48)

/* Instruction handled by the RA6M5 QSPI peripheral */

#define QSPI_SECTOR_ERASE 0x20
#define QSPI_BLOCK_ERASE  0xd8
#define QSPI_ALL_ERASE    0xc7
#define QSPI_PAGE_PROGRAM 0x02
#define QSPI_READ_STATUS  0x05
#define QSPI_WRITE_ENABLE 0x06
#define QSPI_WRITE_ENABLE 0x06

/* QSPI memory synchronization */

#define MEMORY_SYNC()     do { ARM_DSB(); ARM_ISB(); } while (0)

/* Ensure that the DMA buffers are word-aligned. */

#define ALIGN_SHIFT       2
#define ALIGN_MASK        3
#define ALIGN_UP(n)       (((n)+ALIGN_MASK) & ~ALIGN_MASK)
#define IS_ALIGNED(n)     (((uint32_t)(n) & ALIGN_MASK) == 0)


/* Configuration ************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ra6m5_qspidev_s
{
  struct qspi_dev_s qspi;           /* Externally visible part of the QSPI interface */
  mutex_t           lock;           /* Assures mutually exclusive access to QSPI */
  uint32_t          base;           /* QSPI base address */
  uint32_t          actual;         /* Actual clock frequency */
  uint32_t          frequency;      /* Requested clock frequency */
  bool              initialized;    /* TRUE: Controller has been initialized */
  uint8_t           intf;           /* QSPI controller number (0) */
  uint8_t           mode;           /* Mode 0,3 */
  sem_t             op_sem;         /* Block until complete */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

static inline void ra6m5_qspi_putreg(struct ra6m5_qspidev_s *priv,
                                     uint32_t offset,
                                     uint32_t value);
static inline uint32_t ra6m5_qspi_getreg(struct ra6m5_qspidev_s *priv,
                                         uint32_t offset);

/* QSPI operations */

static int ra6m5_qspi_lock(struct qspi_dev_s *dev, bool lock);

static int ra6m5_qspi_lock(struct qspi_dev_s *dev, bool lock);
static uint32_t ra6m5_qspi_setfrequency(struct qspi_dev_s *dev,
                                        uint32_t frequency);
static void ra6m5_qspi_setmode(struct qspi_dev_s *dev,
                               enum qspi_mode_e mode);
static void ra6m5_qspi_setbits(struct qspi_dev_s *dev, int nbits);
static int ra6m5_qspi_command(struct qspi_dev_s *dev,
                             struct qspi_cmdinfo_s *cmdinfo);
static int ra6m5_qspi_memory(struct qspi_dev_s *dev,
                             struct qspi_meminfo_s *meminfo);
static void *ra6m5_qspi_alloc(struct qspi_dev_s *dev, size_t buflen);
static void ra6m5_qspi_free(struct qspi_dev_s *dev, void *buffer);

static int ra6m5_qspi_interrupt(int irq, void *context, void *arg);
static int ra6m5_qspi_hw_initialize(struct ra6m5_qspidev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct qspi_ops_s g_qspi_ops =
{
  .lock              = ra6m5_qspi_lock,
  .setfrequency      = ra6m5_qspi_setfrequency,
  .setmode           = ra6m5_qspi_setmode,
  .setbits           = ra6m5_qspi_setbits,
  .command           = ra6m5_qspi_command,
  .memory            = ra6m5_qspi_memory,
  .alloc             = ra6m5_qspi_alloc,
  .free              = ra6m5_qspi_free,
};

static struct ra6m5_qspidev_s g_qspi0_dev =
{
  .qspi      =
  {
    .ops     = &g_qspi_ops,
  },
  .lock      = NXMUTEX_INITIALIZER,
  .op_sem    = SEM_INITIALIZER(0),
  .base      = RA6M5_QSPI_BASE,
  .intf      = 0,
  .mode      = 255
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra6m5_qspi_putreg
 *
 * Description:
 *   Put a 32-bit register value by offset
 *
 ****************************************************************************/

static inline void ra6m5_qspi_putreg(struct ra6m5_qspidev_s *priv,
                                     uint32_t offset,
                                     uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: ra6m5_qspi_getreg
 *
 * Description:
 *   Get a 32-bit register value by offset
 *
 ****************************************************************************/

static inline uint32_t ra6m5_qspi_getreg(struct ra6m5_qspidev_s *priv,
                                        uint32_t offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: qspi_d0_byte_write_standard
 *
 * Description:
 *   Writes a byte in the SPI mode configured in the QSPI peripheral.
 *
 ****************************************************************************/

static void qspi_d0_byte_write_standard(struct ra6m5_qspidev_s *priv, uint8_t byte)
{
  ra6m5_qspi_putreg(priv, RA6M5_QSPI_SFMCOM_OFFSET, byte);
}

/****************************************************************************
 * Name: qspi_d0_byte_write_quad_mode
 *
 * Description:
 *   Writes a byte in extended SPI mode when the QSPI peripheral is 
 *   configured for quad mode.
 *
 ****************************************************************************/

static void qspi_d0_byte_write_quad_mode(struct ra6m5_qspidev_s *priv, uint8_t byte)
{
  /* The LSB of each nibble ends up on D0. */
  uint32_t value = 0xEEEEEEEE;
  for (uint32_t i = 0U; i < 8U; i++)
    {
      uint32_t bit = ((uint32_t) (byte >> i) & 1U);

      /* Place bits in every 4th bit (bit 0, 4, 8, ... 28). */
      uint32_t bit_mask = bit << (i * 4U);

      value |= bit_mask;
    }

  ra6m5_qspi_putreg(priv, RA6M5_QSPI_SFMCOM_OFFSET, (uint8_t) (value >> 24U));
  ra6m5_qspi_putreg(priv, RA6M5_QSPI_SFMCOM_OFFSET, (uint8_t) (value >> 16U));
  ra6m5_qspi_putreg(priv, RA6M5_QSPI_SFMCOM_OFFSET, (uint8_t) (value >>  8U));
  ra6m5_qspi_putreg(priv, RA6M5_QSPI_SFMCOM_OFFSET, (uint8_t) (value       ));
}

/****************************************************************************
 * Name: ra6m5_qspi_lock
 *
 * Description:
 *   On QSPI buses where there are multiple devices, it will be necessary to
 *   lock QSPI to have exclusive access to the buses for a sequence of
 *   transfers.  The bus should be locked before the chip is selected. After
 *   locking the QSPI bus, the caller should then also call the setfrequency,
 *   setbits, and setmode methods to make sure that the QSPI is properly
 *   configured for the device.  If the QSPI bus is being shared, then it
 *   may have been left in an incompatible state.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   lock - true: Lock QSPI bus, false: unlock QSPI bus
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int ra6m5_qspi_lock(struct qspi_dev_s *dev, bool lock)
{
  struct ra6m5_qspidev_s *priv = (struct ra6m5_qspidev_s *)dev;
  int ret;

  spiinfo("lock=%d\n", lock);
  if (lock)
    {
      ret = nxmutex_lock(&priv->lock);
    }
  else
    {
      ret = nxmutex_unlock(&priv->lock);
    }

  return ret;
}

/****************************************************************************
 * Name: ra6m5_qspi_setfrequency
 *
 * Description:
 *   Set the QSPI frequency.
 *
 * Input Parameters:
 *   dev -       Device-specific state data
 *   frequency - The QSPI frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static uint32_t ra6m5_qspi_setfrequency(struct qspi_dev_s *dev,
                                        uint32_t frequency)
{
  struct ra6m5_qspidev_s *priv = (struct ra6m5_qspidev_s *)dev;
  uint32_t sckfreq = 0;
  uint32_t actual  = 0;

  spiinfo("frequency=%ld\n", frequency);
  DEBUGASSERT(priv);

  /* Check if the requested frequency is the same as the frequency
   * selection
   */

  if (priv->frequency == frequency)
    {
      /* We are already at this frequency.  Return the actual. */

      return priv->actual;
    }

  /* Get prescaler */

  if (frequency <= RA6M5_QSPI_FREQMIN)
    {
      sckfreq = 2;
    }
  else if (frequency <= RA6M5_QSPI_FREQMAX)
    {
      sckfreq = (g_clock_freq[RA6M5_CLOCKS_SOURCE_PCLKA] / 2) / frequency;
    }
  else
    {
      sckfreq = 0;
    }

  /* Modify register */

  ra6m5_qspi_putreg(priv, RA6M5_QSPI_SFMSKC_OFFSET, sckfreq);

  /* Calculate the new actual frequency */

  actual = g_clock_freq[RA6M5_CLOCKS_SOURCE_PCLKA] / (2 * sckfreq);

  /* Save the frequency setting */

  priv->frequency = frequency;
  priv->actual    = actual;

  spiinfo("Frequency %ld->%ld\n", frequency, actual);

  return actual;
}

/****************************************************************************
 * Name: ra6m5_qspi_setmode
 *
 * Description:
 *   Set the QSPI mode. Optional.  See enum qspi_mode_e for mode definitions.
 *   NOTE:  the RA6M5 QSPI supports only modes 0 and 3.
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   mode - The QSPI mode requested
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void ra6m5_qspi_setmode(struct qspi_dev_s *dev, enum qspi_mode_e mode)
{
  struct ra6m5_qspidev_s *priv = (struct ra6m5_qspidev_s *)dev;
  uint32_t regval = 0;

  spiinfo("mode=%d\n", mode);

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      regval = ra6m5_qspi_getreg(priv, RA6M5_QSPI_SFMSMD_OFFSET);

      switch (mode)
        {
          case QSPIDEV_MODE0:
            {
              regval &= ~(QSPI_SFMSMD_SFMMD3);
              break;
            }

          case QSPIDEV_MODE3:
            {
              regval |= (QSPI_SFMSMD_SFMMD3);
              break;
            }

          case QSPIDEV_MODE1:
          case QSPIDEV_MODE2:
            {
              spiinfo("unsupported mode=%d\n", mode);

              /* No break here */
            }
          default:
            {
              DEBUGASSERT(0);
              return;
            }
        }

      /* Write new mode */

      ra6m5_qspi_putreg(priv, RA6M5_QSPI_SFMSMD_OFFSET, regval);

      /* Save the mode so that subsequent re-configurations will be faster */

      priv->mode = mode;
    }
}

/****************************************************************************
 * Name: ra6m5_qspi_setbits
 *
 * Description:
 *   Set the number of bits per word.
 *   NOTE:  the RA6M5 QSPI only supports 8 bits, so this does nothing.
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   nbits - The number of bits requests
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void ra6m5_qspi_setbits(struct qspi_dev_s *dev, int nbits)
{
  if (nbits != 8)
    {
      spiinfo("unsupported nbits=%d\n", nbits);
      DEBUGASSERT(FALSE);
    }
}


/****************************************************************************
 * Name: ra6m5_qspi_command
 *
 * Description:
 *   Perform one QSPI data transfer
 *
 *   TODO: long frame mode not supported
 *
 * Input Parameters:
 *   dev     - Device-specific state data
 *   cmdinfo - Describes the command transfer to be performed.
 *
 * Returned Value:
 *   Zero (OK) on SUCCESS, a negated errno on value of failure
 *
 ****************************************************************************/

static int ra6m5_qspi_command(struct qspi_dev_s *dev,
                              struct qspi_cmdinfo_s *cmdinfo)
{
  struct ra6m5_qspidev_s *priv = (struct ra6m5_qspidev_s *)dev;
  void (*write_command)(struct ra6m5_qspidev_s *priv, uint8_t byte) = qspi_d0_byte_write_standard;
  void (*write_address)(struct ra6m5_qspidev_s *priv, uint8_t byte) = qspi_d0_byte_write_standard;

  DEBUGASSERT(cmdinfo->cmd < 256);

  if (QSPICMD_ISADDRESS(cmdinfo->flags))
    {
      /* Enter direct communication mode */
      ra6m5_qspi_putreg(priv, RA6M5_QSPI_SFMCMD_OFFSET, 1);

      /* Send erase command. */
      write_command(priv, cmdinfo->cmd);

      /* Only ERASE commands supported */
      switch (cmdinfo->cmd)
        {
          case QSPI_SECTOR_ERASE:
          case QSPI_BLOCK_ERASE:
            {
              /* Send address if this is not a chip erase command. */
              if ((cmdinfo->addr & 3) == 3)
              {
                write_address(priv, (uint8_t) (cmdinfo->addr >> 24));
              }
              write_address(priv, (uint8_t) (cmdinfo->addr >> 16));
              write_address(priv, (uint8_t) (cmdinfo->addr >> 8));
              write_address(priv, (uint8_t) (cmdinfo->addr));
              break;
            }

          case QSPI_ALL_ERASE:
            {
              break;
            }

          default:
            {
              /* Not supported addressed command */

              DEBUGASSERT(0);
              return -EINVAL;
            }
        }

      /* Close the SPI bus cycle. Reference section 39.10.3 "Generating the SPI Bus Cycle during Direct Communication"
      * in the RA6M3 manual R01UH0886EJ0100. */
      ra6m5_qspi_putreg(priv, RA6M5_QSPI_SFMCMD_OFFSET, 1);

      /* Exit direct communication mode */
      ra6m5_qspi_putreg(priv, RA6M5_QSPI_SFMCMD_OFFSET, 0);
    }

  else if (QSPICMD_ISWRITE(cmdinfo->flags))
    {
      /* Enter direct communication mode */
      ra6m5_qspi_putreg(priv, RA6M5_QSPI_SFMCMD_OFFSET, 1);

      /* Send command to write data */
      write_command(priv, cmdinfo->cmd);

      /* Write data to QSPI. */
      uint8_t *buffer = cmdinfo->buffer;
      for (uint16_t i = 0; i < cmdinfo->buflen; i++)
      {
        ra6m5_qspi_putreg(priv, RA6M5_QSPI_SFMCOM_OFFSET, buffer[i]);
      }

      /* Close the SPI bus cycle. Reference section 39.10.3 "Generating the SPI Bus Cycle during Direct Communication"
      * in the RA6M3 manual R01UH0886EJ0100. */
      ra6m5_qspi_putreg(priv, RA6M5_QSPI_SFMCMD_OFFSET, 1);
    
      /* Exit direct communication mode */
      ra6m5_qspi_putreg(priv, RA6M5_QSPI_SFMCMD_OFFSET, 0);
    }

  else if (QSPICMD_ISREAD(cmdinfo->flags))
    {
      /* Enter direct communication mode */
      ra6m5_qspi_putreg(priv, RA6M5_QSPI_SFMCMD_OFFSET, 1);

      /* Send command to start read */
      write_command(priv, cmdinfo->cmd);

      /* Read data from QSPI */
      uint8_t *buffer = cmdinfo->buffer;
      for (uint16_t i = 0; i < cmdinfo->buflen; i++)
      {
        buffer[i] = (uint8_t) ra6m5_qspi_getreg(priv, RA6M5_QSPI_SFMCOM_OFFSET);
      }

      /* Close the SPI bus cycle. Reference section 39.10.3 "Generating the SPI Bus Cycle during Direct
        * Communication" in the RA6M3 manual R01UH0886EJ0100. */
      ra6m5_qspi_putreg(priv, RA6M5_QSPI_SFMCMD_OFFSET, 1);

      /* Return to ROM access mode */
      ra6m5_qspi_putreg(priv, RA6M5_QSPI_SFMCMD_OFFSET, 0);
    }
  else
    {
      /* Enter direct communication mode */
      ra6m5_qspi_putreg(priv, RA6M5_QSPI_SFMCMD_OFFSET, 1);

      /* Send command to start read */
      write_command(priv, cmdinfo->cmd);

      /* Close the SPI bus cycle. Reference section 39.10.3 "Generating the SPI Bus Cycle during Direct
        * Communication" in the RA6M3 manual R01UH0886EJ0100. */
      ra6m5_qspi_putreg(priv, RA6M5_QSPI_SFMCMD_OFFSET, 1);

      /* Return to ROM access mode */
      ra6m5_qspi_putreg(priv, RA6M5_QSPI_SFMCMD_OFFSET, 0);
    }
 
  return OK;
}

/****************************************************************************
 * Name: ra6m5_qspi_memory
 *
 * Description:
 *   Perform one QSPI memory transfer
 *
 * Input Parameters:
 *   dev     - Device-specific state data
 *   meminfo - Describes the memory transfer to be performed.
 *
 * Returned Value:
 *   Zero (OK) on SUCCESS, a negated errno on value of failure
 *
 ****************************************************************************/

static int ra6m5_qspi_memory(struct qspi_dev_s *dev,
                             struct qspi_meminfo_s *meminfo)
{
  struct ra6m5_qspidev_s *priv = (struct ra6m5_qspidev_s *)dev;
  void (*write_command)(struct ra6m5_qspidev_s *priv, uint8_t byte) = qspi_d0_byte_write_quad_mode;
  void (*write_address)(struct ra6m5_qspidev_s *priv, uint8_t byte) = qspi_d0_byte_write_standard;

  /* Check the quad request */
  if (QSPIMEM_ISQUADIO(meminfo->flags))
    {
      /* Store the current configuration */
      uint32_t spc = ra6m5_qspi_getreg(priv, RA6M5_QSPI_SFMSPC_OFFSET);
      uint32_t smd = ra6m5_qspi_getreg(priv, RA6M5_QSPI_SFMSMD_OFFSET);

      /* Set the SPI protocol "Extended SPI protocol" */
      ra6m5_qspi_putreg(priv, RA6M5_QSPI_SFMSPC_OFFSET,  (spc & ~QSPI_SFMSPC_SFMSPI_MASK) | QSPI_SFMSPC_SFMSPI(0));

      /* Set the read mode to "Fast Read Quad I/O" */      
      ra6m5_qspi_putreg(priv, RA6M5_QSPI_SFMSMD_OFFSET,  (smd & ~QSPI_SFMSMD_SFMRM_MASK) | QSPI_SFMSMD_SFMRM(5));

      if (QSPIMEM_ISWRITE(meminfo->flags))
        {
          /* Enter direct communication mode */
          ra6m5_qspi_putreg(priv, RA6M5_QSPI_SFMCMD_OFFSET, 1);

          /* Send command to write data */
          write_command(priv, meminfo->cmd);

          /* Send address */
          if ((meminfo->addr & 3) == 3)
          {
            write_address(priv, (uint8_t) (meminfo->addr >> 24));
          }
          write_address(priv, (uint8_t) (meminfo->addr >> 16));
          write_address(priv, (uint8_t) (meminfo->addr >> 8));
          write_address(priv, (uint8_t) (meminfo->addr));

          /* Write data to QSPI. */
          uint8_t *buffer = meminfo->buffer;
          for (uint16_t i = 0; i < meminfo->buflen; i++)
          {
            ra6m5_qspi_putreg(priv, RA6M5_QSPI_SFMCOM_OFFSET, buffer[i]);
          }

          /* Close the SPI bus cycle. Reference section 39.10.3 "Generating the SPI Bus Cycle during Direct Communication"
          * in the RA6M3 manual R01UH0886EJ0100. */
          ra6m5_qspi_putreg(priv, RA6M5_QSPI_SFMCMD_OFFSET, 1);
        
          /* Exit direct communication mode */
          ra6m5_qspi_putreg(priv, RA6M5_QSPI_SFMCMD_OFFSET, 0);
        }

      else if (QSPIMEM_ISREAD(meminfo->flags))
        {
          /* Enter direct communication mode */
          ra6m5_qspi_putreg(priv, RA6M5_QSPI_SFMCMD_OFFSET, 1);

          /* Send command to start read */
          write_command(priv, meminfo->cmd);

          /* Send address */
          if ((meminfo->addr & 3) == 3)
          {
            write_address(priv, (uint8_t) (meminfo->addr >> 24));
          }
          write_address(priv, (uint8_t) (meminfo->addr >> 16));
          write_address(priv, (uint8_t) (meminfo->addr >> 8));
          write_address(priv, (uint8_t) (meminfo->addr));

          /* Read data from QSPI */
          uint8_t *buffer = meminfo->buffer;
          for (uint16_t i = 0; i < meminfo->buflen; i++)
          {
            buffer[i] = (uint8_t) ra6m5_qspi_getreg(priv, RA6M5_QSPI_SFMCOM_OFFSET);
          }

          /* Close the SPI bus cycle. Reference section 39.10.3 "Generating the SPI Bus Cycle during Direct
            * Communication" in the RA6M3 manual R01UH0886EJ0100. */
          ra6m5_qspi_putreg(priv, RA6M5_QSPI_SFMCMD_OFFSET, 1);

          /* Return to ROM access mode */
          ra6m5_qspi_putreg(priv, RA6M5_QSPI_SFMCMD_OFFSET, 0);
        }

      /* Restore the saved configuration" */
      ra6m5_qspi_putreg(priv, RA6M5_QSPI_SFMSPC_OFFSET, spc);
      ra6m5_qspi_putreg(priv, RA6M5_QSPI_SFMSMD_OFFSET, smd);
    }

  return OK;
}

/****************************************************************************
 * Name: ra6m5_qspi_alloc
 *
 * Description:
 *   Allocate a buffer suitable for DMA data transfer
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   buflen - Buffer length to allocate in bytes
 *
 * Returned Value:
 *   Address of the allocated memory on success; NULL is returned on any
 *   failure.
 *
 ****************************************************************************/

static void *ra6m5_qspi_alloc(struct qspi_dev_s *dev, size_t buflen)
{
  return kmm_malloc(ALIGN_UP(buflen));
}

/****************************************************************************
 * Name: ra6m5_qspi_free
 *
 * Description:
 *   Free memory returned by QSPI_ALLOC
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   buffer - Buffer previously allocated via QSPI_ALLOC
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void ra6m5_qspi_free(struct qspi_dev_s *dev, void *buffer)
{
  if (buffer)
    {
      kmm_free(buffer);
    }
}

/****************************************************************************
 * Name: ra6m5_qspi_interrupt
 *
 * Description:
 *   Interrupt handler; we handle all QSPI cases -- reads, writes,
 *   automatic status polling, etc.
 *
 * Input Parameters:
 *   irq  -
 *   context  -
 *   qrg  -
 *
 * Returned Value:
 *   OK means we handled it
 *
 ****************************************************************************/

static int ra6m5_qspi_interrupt(int irq, void *context, void *arg)
{
  return OK;
}

/****************************************************************************
 * Name: ra6m5_qspi_hw_initialize
 *
 * Description:
 *   Initialize the QSPI peripheral from hardware reset.
 *
 * Input Parameters:
 *   priv - Device state structure.
 *
 * Returned Value:
 *   Zero (OK) on SUCCESS, a negated errno on value of failure
 *
 ****************************************************************************/

static int ra6m5_qspi_hw_initialize(struct ra6m5_qspidev_s *priv)
{
  int ret = 0;

  /* Only for QSPI0 */

  DEBUGASSERT(priv->intf == 0);

  /* Release from the module-stop state */

  modifyreg32(RA6M5_MSTP_REG(RA6M5_MSTP_MSTPCRB_OFFSET), MSTP_MSTPCRB_QSPI, 0);

  /* Configure QSPI pins */

  ra6m5_configgpio(GPIO_QSPI_SCK);
  ra6m5_configgpio(GPIO_QSPI_SSL);
  ra6m5_configgpio(GPIO_QSPI_IO0);
  ra6m5_configgpio(GPIO_QSPI_IO1);
  ra6m5_configgpio(GPIO_QSPI_IO2);
  ra6m5_configgpio(GPIO_QSPI_IO3);

  /* Initialized unused registers. */

  ra6m5_qspi_putreg(priv, RA6M5_QSPI_SFMCST_OFFSET, 0);
  ra6m5_qspi_putreg(priv, RA6M5_QSPI_SFMSIC_OFFSET, 0);
  ra6m5_qspi_putreg(priv, RA6M5_QSPI_SFMPMD_OFFSET, 0);
  ra6m5_qspi_putreg(priv, RA6M5_QSPI_SFMCNT1_OFFSET, 0);

  /* Set the initial SPI protocol. */

  ra6m5_qspi_putreg(priv, RA6M5_QSPI_SFMSPC_OFFSET, QSPI_SFMSPC_SFMSPI(0) | QSPI_SFMSPC_SFMSDE);

  /* Set the SPI clock rate */

  ra6m5_qspi_putreg(priv, RA6M5_QSPI_SFMSKC_OFFSET, 4);

  /* Set the address mode. */

  ra6m5_qspi_putreg(priv, RA6M5_QSPI_SFMSAC_OFFSET, QSPI_SFMSAC_SFMAS(2));

  /* Set the number of dummy cycles in QSPI peripheral */

  ra6m5_qspi_putreg(priv, RA6M5_QSPI_SFMSDC_OFFSET, 0xff00);

  /* Set configured minimum high level width for QSSL signal. */

  ra6m5_qspi_putreg(priv, RA6M5_QSPI_SFMSSC_OFFSET, QSPI_SFMSSC_SFMSW(7) | QSPI_SFMSAC_SFMSHD | QSPI_SFMSAC_SFMSLD);

  /* Set the read mode based on user configuration. */

  ra6m5_qspi_putreg(priv, RA6M5_QSPI_SFMSMD_OFFSET, QSPI_SFMSMD_SFMRM(0) | QSPI_SFMSMD_SFMPFE);

  /* Attach the interrupt handler */

  ret = irq_attach(RA6M5_IRQ_QSPI_INT, ra6m5_qspi_interrupt, priv);
  if (ret < 0)
    {
      spierr("ERROR: Failed to attach QSPI irq\n");
      return ret;
    }

  /* Enable QSPI interrupts */

  up_enable_irq(RA6M5_IRQ_QSPI_INT);

  /* Wait for READY event.
   * TODO: add timeout.
   */

  //nxsem_wait(&priv->op_sem);

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra6m5_qspi_initialize
 *
 * Description:
 *   Initialize the selected QSPI port in master mode
 *
 * Input Parameters:
 *   intf - Interface number(must be zero)
 *
 * Returned Value:
 *   Valid QSPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct qspi_dev_s *ra6m5_qspi_initialize(int intf)
{
  struct ra6m5_qspidev_s *priv = NULL;
  int                     ret  = OK;

  /* The RA6M5 has only a single QSPI port */

  spiinfo("intf: %d\n", intf);
  DEBUGASSERT(intf == 0);

  /* Select the QSPI interface */

  if (intf == 0)
    {
      /* Select QSPI0 */

      priv = &g_qspi0_dev;
    }
  else
    {
      spierr("ERROR: QSPI%d not supported\n", intf);
      return NULL;
    }

  /* Has the QSPI hardware been initialized? */

  if (!priv->initialized)
    {
      /* Perform hardware initialization.  Puts the QSPI into an active
       * state.
       */

      ret = ra6m5_qspi_hw_initialize(priv);
      if (ret < 0)
        {
          spierr("ERROR: Failed to initialize QSPI hardware\n");
          irq_detach(RA6M5_IRQ_QSPI_INT);
          return NULL;
        }

      /* Enable interrupts at the NVIC */

      priv->initialized = true;
    }

  return &priv->qspi;
}

#endif /* CONFIG_RA6M5_QSPI0 */
