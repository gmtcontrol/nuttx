/****************************************************************************
 * arch/risc-v/src/sty32c2/sty32c2_i2c_bitbang.c
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

#include <debug.h>
#include <errno.h>
#include <assert.h>

#include <arch/board/board.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>

#include "riscv_internal.h"
#include "hardware/sty32c2_spi.h"
#include "sty32c2.h"
#include "sty32c2_spi.h"
#include "sty32c2_clockconfig.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sty32c2_spidev_s
{
  struct spi_dev_s dev;         /* Externally visible part of the SPI interface */
  sem_t            exclsem;     /* Held while chip is selected for mutual exclusion */
  uint32_t         frequency;   /* Requested clock frequency */
  uint32_t         actual;      /* Actual clock frequency */
  uint8_t          nbits;       /* Width of word in bits (4 to 16) */
  uint8_t          dma_chan;    /* DMA channel */
  uint8_t          mode;        /* Mode 0,1,2,3 */
  uint8_t          bus;         /* Bus number */
  int              refs;        /* Referernce count */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SPI methods */

static int spi_lock(struct spi_dev_s *dev, bool lock);
static void spi_select(struct spi_dev_s *dev, uint32_t devid, 
                       bool selected);
static uint32_t spi_setfrequency(struct spi_dev_s *dev,
                                 uint32_t frequency);
static void spi_setmode(struct spi_dev_s *dev, 
                        enum spi_mode_e mode);
static void spi_setbits(struct spi_dev_s *dev, int nbits);
#ifdef CONFIG_SPI_HWFEATURES
static int spi_hwfeatures(struct spi_dev_s *dev,
                          spi_hwfeatures_t features);
#endif
static uint8_t spi_status(struct spi_dev_s *dev,
                          uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
static int spi_cmddata(struct spi_dev_s *dev,
                       uint32_t devid, bool cmd);
#endif
static uint32_t spi_send(struct spi_dev_s *dev, uint32_t wd);
static void spi_exchange(struct spi_dev_s *dev,
                         const void *txbuffer,
                         void *rxbuffer, size_t nwords);
#ifndef CONFIG_SPI_EXCHANGE
static void spi_sndblock(struct spi_dev_s *dev,
                         const void *txbuffer, size_t nwords);
static void spi_recvblock(struct spi_dev_s *dev,
                          void *rxbuffer, size_t nwords);
#endif
#ifdef CONFIG_SPI_TRIGGER
static int spi_trigger(struct spi_dev_s *dev);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_STY32C2_SPI0
static const struct spi_ops_s g_spi0_ops =
{
  .lock             = spi_lock,
  .select           = spi_select,
  .setfrequency     = spi_setfrequency,
  .setmode          = spi_setmode,
  .setbits          = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures       = spi_hwfeatures,
#endif
  .status           = spi_status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata          = spi_cmddata,
#endif
  .send             = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange         = spi_exchange,
#else
  .sndblock         = spi_sndblock,
  .recvblock        = spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger          = spi_trigger,
#endif
  .registercallback = NULL,
};

static struct sty32c2_spidev_s g_spi0_dev =
{
  .dev    = 
  { 
    .ops  = &g_spi0_ops
  },
  .bus    = 0,
};
#endif

#ifdef CONFIG_STY32C2_SPI1
static const struct spi_ops_s g_spi1_ops =
{
  .lock             = spi_lock,
  .select           = spi_select,
  .setfrequency     = spi_setfrequency,
  .setmode          = spi_setmode,
  .setbits          = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures       = spi_hwfeatures,
#endif
  .status           = spi_status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata          = spi_cmddata,
#endif
  .send             = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange         = spi_exchange,
#else
  .sndblock         = spi_sndblock,
  .recvblock        = spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger          = spi_trigger,
#endif
  .registercallback = NULL,
};

static struct sty32c2_spidev_s g_spi1_dev =
{
  .dev    = 
  { 
    .ops  = &g_spi1_ops
  },
  .bus    = 1,
};
#endif

#ifdef CONFIG_STY32C2_SPI2
static const struct spi_ops_s g_spi2_ops =
{
  .lock             = spi_lock,
  .select           = spi_select,
  .setfrequency     = spi_setfrequency,
  .setmode          = spi_setmode,
  .setbits          = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures       = spi_hwfeatures,
#endif
  .status           = spi_status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata          = spi_cmddata,
#endif
  .send             = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange         = spi_exchange,
#else
  .sndblock         = spi_sndblock,
  .recvblock        = spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger          = spi_trigger,
#endif
  .registercallback = NULL,
};

static struct sty32c2_spidev_s g_spi2_dev =
{
  .dev    = 
  { 
    .ops  = &g_spi2_ops
  },
  .bus    = 2,
};
#endif

#ifdef CONFIG_STY32C2_SPI3
static const struct spi_ops_s g_spi3_ops =
{
  .lock             = spi_lock,
  .select           = spi_select,
  .setfrequency     = spi_setfrequency,
  .setmode          = spi_setmode,
  .setbits          = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures       = spi_hwfeatures,
#endif
  .status           = spi_status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata          = spi_cmddata,
#endif
  .send             = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange         = spi_exchange,
#else
  .sndblock         = spi_sndblock,
  .recvblock        = spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger          = spi_trigger,
#endif
  .registercallback = NULL,
};

static struct sty32c2_spidev_s g_spi3_dev =
{
  .dev    = 
  { 
    .ops  = &g_spi3_ops
  },
  .bus    = 3,
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_lock
 *
 * Description:
 *   Lock or unlock the SPI device
 *
 * Input Parameters:
 *   priv   - Private SPI device structure
 *   lock   - true: Lock spi bus, false: unlock SPI bus
 *
 * Returned Value:
 *   The result of lock or unlock the SPI device
 *
 ****************************************************************************/

static int spi_lock(struct spi_dev_s *dev, bool lock)
{
  struct sty32c2_spidev_s *priv = (struct sty32c2_spidev_s *)dev;
  int ret;

  if (lock)
    {
      /* Take the semaphore (perhaps waiting) */

      ret = nxsem_wait_uninterruptible(&priv->exclsem);
    }
  else
    {
      ret = nxsem_post(&priv->exclsem);
    }

  return ret;
}

/****************************************************************************
 * Name: spi_select
 *
 * Description:
 *   Enable/disable the SPI chip select.  The implementation of this method
 *   must include handshaking:  If a device is selected, it must hold off
 *   all other attempts to select the device until the device is deselected.
 *
 *   If disable SPI_SWCS, driver will use hardware CS so that when
 *   once transmission is started, hardware select the device and when this
 *   transmission is done, hardware deselect the device automatically. And
 *   the function will do nothing.
 *
 * Input Parameters:
 *   priv     - Private SPI device structure
 *   devid    - Identifies the device to select
 *   selected - true: slave selected, false: slave de-selected
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_select(struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  struct sty32c2_spidev_s *priv = (struct sty32c2_spidev_s *)dev;

  spiinfo("devid: %lu, bus: %u, CS: %s\n", devid, priv->bus, selected ? "select" : "free");

  /* User controlled chip select. */

  putreg32(SPI_CSMODE_HW | selected, STY32C2_SPI_CS_REG(priv->bus));
  
}

/****************************************************************************
 * Name: spi_setfrequency
 *
 * Description:
 *   Set the SPI frequency.
 *
 * Input Parameters:
 *   dev -       Device-specific state data
 *   frequency - The SPI frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static uint32_t spi_setfrequency(struct spi_dev_s *dev,
                                 uint32_t frequency)
{
  struct sty32c2_spidev_s *priv = (struct sty32c2_spidev_s *)dev;
  uint32_t divisor;

  if (priv->frequency == frequency)
    {
      /* We are already at this frequency. Return the actual. */

      return priv->actual;
    }

  /* frequency = SPI_CLOCK / divisor, or divisor = SPI_CLOCK / frequency */

  divisor = sty32c2_get_spiclk() / frequency;

  /* "In master mode, CPSDVSRmin = 2 or larger (even numbers only)" */

  if (divisor < 1)
    {
      divisor = 1;
    }
  else if (divisor > 65535)
    {
      divisor = 65535;
    }

  /* Save the new divisor value */

  putreg32(divisor << 1, STY32C2_SPI_CDIV_REG(priv->bus));

  /* Save the frequency setting */

  priv->frequency = frequency;
  priv->actual    = sty32c2_get_spiclk() / divisor;

  spiinfo("divisor=%lu, frequency=%lu, actual=%lu\n", divisor, priv->frequency, priv->actual);

  return priv->actual;
}

/****************************************************************************
 * Name: spi_setdelay
 *
 * Description:
 *   Set the SPI Delays in nanoseconds. Optional.
 *
 * Input Parameters:
 *   dev        - Device-specific state data
 *   startdelay - The delay between CS active and first CLK
 *   stopdelay  - The delay between last CLK and CS inactive
 *   csdelay    - The delay between CS inactive and CS active again
 *   ifdelay    - The delay between frames
 *
 * Returned Value:
 *   Returns zero (OK) on success; a negated errno value is return on any
 *   failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_DELAY_CONTROL
static int spi_setdelay(struct spi_dev_s *dev, uint32_t startdelay,
                        uint32_t stopdelay, uint32_t csdelay,
                        uint32_t ifdelay)
{
  spierr("SPI CS delay control not supported\n");
  DEBUGPANIC();

  return -1;
}
#endif

/****************************************************************************
 * Name: spi_setmode
 *
 * Description:
 *   Set the SPI mode.
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   mode - The SPI mode requested
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode)
{
  struct sty32c2_spidev_s *priv = (struct sty32c2_spidev_s *)dev;

  spiinfo("mode=%d\n", mode);

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      switch (mode)
        {
        case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
          break;

        case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
          break;

        case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
          break;

        case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
          break;

        default:
          return;
        }

      priv->mode = mode;
    }
}

/****************************************************************************
 * Name: sty32c2_spi_setbits
 *
 * Description:
 *   Set the number if bits per word.
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   nbits - The number of bits in an SPI word.
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void spi_setbits(struct spi_dev_s *dev, int nbits)
{
  struct sty32c2_spidev_s *priv = (struct sty32c2_spidev_s *)dev;

  spiinfo("nbits=%d\n", nbits);

  /* Has the number of bits changed? */

  if (nbits != priv->nbits)
    {
      /* Save the selection so that subsequent re-configurations
       * will be faster.
       */

      priv->nbits = nbits;
    }
}

/****************************************************************************
 * Name: spi_status
 *
 * Description:
 *   Get SPI/MMC status.  Optional.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   devid - Identifies the device to report status on
 *
 * Returned Value:
 *   Returns a bitset of status values (see SPI_STATUS_* defines)
 *
 ****************************************************************************/

static uint8_t spi_status(struct spi_dev_s *dev, uint32_t devid)
{
  struct sty32c2_spidev_s *priv = (struct sty32c2_spidev_s *)dev;
  uint8_t status = 0;

  spiinfo("status=%u, devid=%lu, bus=%u\n", status, devid, priv->bus);

  return status;
}

/****************************************************************************
 * Name: spi_cmddata
 *
 * Description:
 *   Some devices require an additional out-of-band bit to specify if the
 *   next word sent to the device is a command or data. This is typical, for
 *   example, in "9-bit" displays where the 9th bit is the CMD/DATA bit.
 *   This function provides selection of command or data.
 *
 *   This "latches" the CMD/DATA state.  It does not have to be called before
 *   every word is transferred; only when the CMD/DATA state changes.  This
 *   method is required if CONFIG_SPI_CMDDATA is selected in the NuttX
 *   configuration
 *
 *   This function reconfigures MISO from SPI Pin to GPIO Pin, and sets
 *   MISO to high (data) or low (command). spi_select() will revert
 *   MISO back from GPIO Pin to SPI Pin.  We must revert because the SPI Bus
 *   may be used by other drivers.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   cmd - TRUE: The following word is a command; FALSE: the following words
 *         are data.
 *
 * Returned Value:
 *   OK unless an error occurs.  Then a negated errno value is returned
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
static int spi_cmddata(struct spi_dev_s *dev,
                       uint32_t devid, bool cmd)
{
  spierr("SPI cmddata not supported\n");
  DEBUGPANIC();

  return -ENODEV;
}
#endif

/****************************************************************************
 * Name: spi_hwfeatures
 *
 * Description:
 *   Set hardware-specific feature flags.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   features - H/W feature flags
 *
 * Returned Value:
 *   Zero (OK) if the selected H/W features are enabled; A negated errno
 *   value if any H/W feature is not supportable.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_HWFEATURES
static int spi_hwfeatures(struct spi_dev_s *dev,
                          spi_hwfeatures_t features)
{
  /* Other H/W features are not supported */

  spierr("SPI hardware specific feature not supported\n");
  DEBUGPANIC();

  return -1;
}
#endif

/****************************************************************************
 * Name: spi_dma_exchange
 *
 * Description:
 *   Exchange a block of data from SPI by DMA.
 *
 * Input Parameters:
 *   priv     - SPI private state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to the buffer in which to receive data
 *   nwords   - the length of data that to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_dma_exchange(struct sty32c2_spidev_s *priv,
                             const void *txbuffer,
                             void *rxbuffer, uint32_t nwords)
{
  spierr("SPI dma not supported\n");
  DEBUGPANIC();
}

/****************************************************************************
 * Name: spi_poll_send
 *
 * Description:
 *   Exchange one word on SPI by polling mode.
 *
 * Input Parameters:
 *   priv - SPI private state data
 *   wd  - The word to send.  the size of the data is determined by the
 *         number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   Received value
 *
 ****************************************************************************/

static uint32_t spi_poll_send(struct sty32c2_spidev_s *priv, uint32_t wd)
{
  uint32_t val;
 
  /* Write data to TX register */

  putreg32(wd, STY32C2_SPI_MOSI_REG(priv->bus));

  /* Start the data transfer */

  putreg32(((priv->nbits & 0xff) << 8) | SPI_CONTROL_XFER, STY32C2_SPI_CTRL_REG(priv->bus));

  /* Wait for the done  */

  while ((getreg32(STY32C2_SPI_STAT_REG(priv->bus)) & SPI_STATUS_DONE) == 0) {

  }

  /* Get the value from the RX register and return it */

  val = getreg32(STY32C2_SPI_MISO_REG(priv->bus));

  spiinfo("send=%lx, recv=%lx\n", wd, val);

  return val;
}

/****************************************************************************
 * Name: spi_dma_send
 *
 * Description:
 *   Exchange one word on SPI by SPI DMA mode.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   wd  - The word to send.  the size of the data is determined by the
 *         number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   Received value
 *
 ****************************************************************************/

static uint32_t spi_dma_send(struct sty32c2_spidev_s *priv, uint32_t wd)
{
  uint32_t rd = 0;

  spi_dma_exchange(priv, &wd, &rd, 1);

  return rd;
}

/****************************************************************************
 * Name: spi_send
 *
 * Description:
 *   Exchange one word on SPI.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   wd  - The word to send.  the size of the data is determined by the
 *         number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   Received value
 *
 ****************************************************************************/

static uint32_t spi_send(struct spi_dev_s *dev, uint32_t wd)
{
  struct sty32c2_spidev_s *priv = (struct sty32c2_spidev_s *)dev;
  uint32_t rd;

  if (priv->dma_chan)
    {
      rd = spi_dma_send(priv, wd);
    }
  else
    {
      rd = spi_poll_send(priv, wd);
    }

  return rd;
}

/****************************************************************************
 * Name: spi_poll_exchange
 *
 * Description:
 *   Exchange a block of data from SPI.
 *
 * Input Parameters:
 *   priv     - SPI private state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to the buffer in which to receive data
 *   nwords   - the length of data that to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_poll_exchange(struct sty32c2_spidev_s *priv,
                              const void *txbuffer,
                              void *rxbuffer, size_t nwords)
{
  int i;
  uint32_t w_wd = 0xffff;
  uint32_t r_wd;

  for (i = 0; i < nwords; i++)
    {
      if (txbuffer)
        {
          if (priv->nbits == 8)
            {
              w_wd = ((uint8_t *)txbuffer)[i];
            }
          else
            {
              w_wd = ((uint16_t *)txbuffer)[i];
            }
        }

      r_wd = spi_poll_send(priv, w_wd);

      if (rxbuffer)
        {
          if (priv->nbits == 8)
            {
              ((uint8_t *)rxbuffer)[i] = r_wd;
            }
          else
            {
              ((uint16_t *)rxbuffer)[i] = r_wd;
            }
        }
    }
}

/****************************************************************************
 * Name: spi_exchange
 *
 * Description:
 *   Exchange a block of data from SPI.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to the buffer in which to receive data
 *   nwords   - the length of data that to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_exchange(struct spi_dev_s *dev,
                         const void *txbuffer, void *rxbuffer,
                         size_t nwords)
{
  struct sty32c2_spidev_s *priv = (struct sty32c2_spidev_s *)dev;

  if (priv->dma_chan)
    {
      spi_dma_exchange(priv, txbuffer, rxbuffer, nwords);
    }
  else
    {
      spi_poll_exchange(priv, txbuffer, rxbuffer, nwords);
    }
}

#ifndef CONFIG_SPI_EXCHANGE

/****************************************************************************
 * Name: spi_sndblock
 *
 * Description:
 *   Send a block of data on SPI.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   buffer - A pointer to the buffer of data to be sent
 *   nwords - the length of data to send from the buffer in number of words.
 *            The wordsize is determined by the number of bits-per-word
 *            selected for the SPI interface.  If nbits <= 8, the data is
 *            packed into uint8_t's; if nbits >8, the data is packed into
 *            uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_sndblock(struct spi_dev_s *dev,
                         const void *txbuffer, size_t nwords)
{
  spiinfo("txbuffer=%p nwords=%d\n", txbuffer, nwords);

  spi_exchange(dev, txbuffer, NULL, nwords);
}

/****************************************************************************
 * Name: sty32c2_spi_recvblock
 *
 * Description:
 *   Receive a block of data from SPI.
 *
 * Input Parameters:
 *   dev -    Device-specific state data
 *   buffer - A pointer to the buffer in which to receive data
 *   nwords - the length of data that can be received in the buffer in number
 *            of words.  The wordsize is determined by the number of bits-
 *            per-word selected for the SPI interface.  If nbits <= 8, the
 *            data is packed into uint8_t's; if nbits >8, the data is packed
 *            into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_recvblock(struct spi_dev_s *dev,
                          void *rxbuffer, size_t nwords)
{
  spiinfo("rxbuffer=%p nwords=%d\n", rxbuffer, nwords);

  spi_exchange(dev, NULL, rxbuffer, nwords);
}
#endif

/****************************************************************************
 * Name: spi_trigger
 *
 * Description:
 *   Trigger a previously configured DMA transfer.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *
 * Returned Value:
 *   OK       - Trigger was fired
 *   -ENOSYS  - Trigger not fired due to lack of DMA or low level support
 *   -EIO     - Trigger not fired because not previously primed
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_TRIGGER
static int spi_trigger(struct spi_dev_s *dev)
{
  spierr("SPI trigger not supported\n");
  DEBUGPANIC();

  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: spi_init
 *
 * Description:
 *   Initialize STY32C2 SPI hardware interface
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_init(struct sty32c2_spidev_s *priv)
{
  /* Initialize the SPI semaphore that enforces mutually exclusive access */

  nxsem_init(&priv->exclsem, 0, 1);

}

/****************************************************************************
 * Name: spi_deinit
 *
 * Description:
 *   Deinitialize bl602 SPI hardware interface
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_deinit(struct sty32c2_spidev_s *priv)
{
  priv->frequency = 0;
  priv->actual = 0;
  priv->mode = SPIDEV_MODE0;
  priv->nbits = 0;
  priv->dma_chan = 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sty32c2_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI bus
 *
 * Input Parameter:
 *   bus - Bus number
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct spi_dev_s *sty32c2_spibus_initialize(int bus)
{
  struct sty32c2_spidev_s *priv = NULL;
  irqstate_t flags;

  switch (bus)
    {
#ifdef CONFIG_STY32C2_SPI0
      case 0:
        priv = &g_spi0_dev;
        break;
#endif

#ifdef CONFIG_STY32C2_SPI1
      case 1:
        priv = &g_spi1_dev;
        break;
#endif

#ifdef CONFIG_STY32C2_SPI2
      case 2:
        priv = &g_spi2_dev;
        break;
#endif

#ifdef CONFIG_STY32C2_SPI3
      case 3:
        priv = &g_spi3_dev;
        break;
#endif
      default:
        return ((struct spi_dev_s*)priv);
    }

  /* enter the critical section */

  flags = enter_critical_section();

  /* If already initialized */

  if (priv->refs != 0)
    {
      leave_critical_section(flags);

      return ((struct spi_dev_s*)&priv->dev);
    }

  spi_init(priv);

  priv->refs++;

  /* exit the critical section */

  leave_critical_section(flags);

  return ((struct spi_dev_s*)&priv->dev);
}

/****************************************************************************
 * Name: sty32c2_spibus_uninitialize
 *
 * Description:
 *   Uninitialize an SPI bus
 *
 ****************************************************************************/

int sty32c2_spibus_uninitialize(struct spi_dev_s *dev)
{
  struct sty32c2_spidev_s *priv = (struct sty32c2_spidev_s *)dev;
  irqstate_t flags;

  DEBUGASSERT(dev);

  if (priv->refs == 0)
    {
      return ERROR;
    }

  flags = enter_critical_section();

  if (--priv->refs)
    {
      leave_critical_section(flags);
      return OK;
    }

  leave_critical_section(flags);

  spi_deinit(priv);

  nxsem_destroy(&priv->exclsem);

  return OK;
}