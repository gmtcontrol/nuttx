/****************************************************************************
 * arch/risc-v/src/sty32c2/sty32c2_sdio.c
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

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/cache.h>
#include <nuttx/clock.h>
#include <nuttx/mmcsd.h>
#include <nuttx/sdio.h>
#include <nuttx/semaphore.h>
#include <nuttx/signal.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "chip.h"
#include "riscv_internal.h"

#include "sty32c2_sdio.h"
#include "sty32c2_clockconfig.h"
#include "hardware/sty32c2_sdio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SD_CTL_DATA_XFER_NONE           0
#define SD_CTL_DATA_XFER_READ           1
#define SD_CTL_DATA_XFER_WRITE          2

#define SDCARD_CTRL_RESPONSE_NONE       0
#define SDCARD_CTRL_RESPONSE_SHORT      1
#define SDCARD_CTRL_RESPONSE_LONG       2

#define STY32C2_INT_CARDDETECT            (1 << 0)
#define STY32C2_INT_BLOCK2MEM             (1 << 1)
#define STY32C2_INT_MEM2BLOCK             (1 << 2)
#define STY32C2_INT_CMDDONE               (1 << 3)

#define STY32C2_EV_CMDDONE                (1 << 0)
#define STY32C2_EV_WRERROR                (1 << 1)
#define STY32C2_EV_TIMEOUT                (1 << 2)
#define STY32C2_EV_CRCERROR               (1 << 3)

#define MAX_DIVIDER                     256

#ifndef CONFIG_STY32C2_IDMODE_FREQ
#  define CONFIG_STY32C2_IDMODE_FREQ 400000    /* 400 KHz, ID mode */
#endif

#ifndef CONFIG_STY32C2_MMCXFR_FREQ
#  define CONFIG_STY32C2_MMCXFR_FREQ 25000000  /* 25MHz MMC, normal clocking */
#endif

#ifndef CONFIG_STY32C2_SD4BIT_FREQ
#  define CONFIG_STY32C2_SD4BIT_FREQ 50000000  /* 25MHz SD 4-bit, normal clocking */
#endif

#define max(x, y) (((x) > (y)) ? (x) : (y))
#define min(x, y) (((x) < (y)) ? (x) : (y))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sty32c2_dev_s
{
  struct sdio_dev_s  dev;             /* Standard, base SDIO interface */

  /* Event support */

  sem_t              waitsem;         /* Implements event waiting */
  sdio_eventset_t    waitevents;      /* Set of events to be waited for */
  uint32_t           waitints;        /* Interrupt enables for event waiting */
  volatile sdio_eventset_t wkupevent; /* The event that caused the wakeup */
  struct wdog_s      waitwdog;        /* Watchdog that handles event timeouts */

  /* Callback support */

  sdio_statset_t     cdstatus;        /* Card status */
  sdio_eventset_t    cbevents;        /* Set of events to be cause callbacks */
  worker_t           callback;        /* Registered callback function */
  void              *cbarg;           /* Registered callback argument */
  struct work_s      cbwork;          /* Callback work queue structure */

  /* Interrupt mode data transfer support */

  uint32_t           xfrints;         /* Interrupt enables for data transfer */

  /* Card interrupt support for SDIO */

  uint32_t           cintints;                /* Interrupt enables for card ints */
  void              (*do_sdio_card)(void *);  /* SDIO card ISR */
  void               *do_sdio_arg;            /* arg for SDIO card ISR */

  /* Fixed transfer block size support */

  uint8_t            block_size;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  sty32c2_interrupt(int irq, void *context, void *arg);

static void sty32c2_reset(struct sdio_dev_s *dev);
static sdio_capset_t sty32c2_capabilities(struct sdio_dev_s *dev);
static sdio_statset_t sty32c2_status(struct sdio_dev_s *dev);
static void sty32c2_widebus(struct sdio_dev_s *dev,
                          bool enable);
static void sty32c2_clock(struct sdio_dev_s *dev,
                        enum sdio_clock_e rate);
static int  sty32c2_attach(struct sdio_dev_s *dev);
static int  sty32c2_sendcmd(struct sdio_dev_s *dev,
                          uint32_t cmd, uint32_t arg);
static void sty32c2_blocksetup(struct sdio_dev_s *dev,
                             unsigned int blocklen, unsigned int nblocks);
static int  sty32c2_cancel(struct sdio_dev_s *dev);
static int  sty32c2_recvsetup(struct sdio_dev_s *dev,
                            uint8_t *buffer, size_t nbytes);
static int  sty32c2_sendsetup(struct sdio_dev_s *dev,
                            const uint8_t *buffer, size_t buflen);
static int  sty32c2_waitresponse(struct sdio_dev_s *dev,
                               uint32_t cmd);
static void sty32c2_callbackenable(struct sdio_dev_s *dev,
                                 sdio_eventset_t eventset);
static int  sty32c2_registercallback(struct sdio_dev_s *dev,
                                   worker_t callback, void *arg);
static int  sty32c2_recvlong(struct sdio_dev_s *dev,
                           uint32_t cmd, uint32_t rlong[4]);
static int  sty32c2_recvshort(struct sdio_dev_s *dev,
                            uint32_t cmd, uint32_t *rshort);
static void sty32c2_waitenable(struct sdio_dev_s *dev,
                             sdio_eventset_t eventset, uint32_t timeout);
static sdio_eventset_t sty32c2_eventwait(struct sdio_dev_s *dev);
static void sty32c2_configwaitints(struct sty32c2_dev_s *priv, uint32_t waitmask,
                sdio_eventset_t waitevents, sdio_eventset_t wkupevents);
static void sty32c2_configxfrints(struct sty32c2_dev_s *priv,
                                uint32_t xfrints);

/* Data Transfer Helpers ****************************************************/

static void sty32c2_eventtimeout(wdparm_t arg);
static void sty32c2_endwait(struct sty32c2_dev_s *priv,
                sdio_eventset_t wkupevent);
static void sty32c2_callback(void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct sty32c2_dev_s g_sdiodev =
{
  .dev =
  {
    .reset            = sty32c2_reset,
    .capabilities     = sty32c2_capabilities,
    .status           = sty32c2_status,
    .widebus          = sty32c2_widebus,
    .clock            = sty32c2_clock,
    .attach           = sty32c2_attach,
    .sendcmd          = sty32c2_sendcmd,
    .blocksetup       = sty32c2_blocksetup,
    .recvsetup        = sty32c2_recvsetup,
    .sendsetup        = sty32c2_sendsetup,
    .cancel           = sty32c2_cancel,
    .dmarecvsetup     = sty32c2_recvsetup,
    .dmasendsetup     = sty32c2_sendsetup,
    .waitresponse     = sty32c2_waitresponse,
    .recv_r1          = sty32c2_recvshort,
    .recv_r2          = sty32c2_recvlong,
    .recv_r3          = sty32c2_recvshort,
    .recv_r4          = sty32c2_recvshort,
    .recv_r5          = sty32c2_recvshort,
    .recv_r6          = sty32c2_recvshort,
    .recv_r7          = sty32c2_recvshort,
    .waitenable       = sty32c2_waitenable,
    .eventwait        = sty32c2_eventwait,
    .callbackenable   = sty32c2_callbackenable,
    .registercallback = sty32c2_registercallback,
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sty32c2_pow2roundup
 *
 * Description:
 *
 ****************************************************************************/

static inline uint32_t sty32c2_pow2roundup(uint32_t r)
{
  r--;
  r |= r >>  1;
  r |= r >>  2;
  r |= r >>  4;
  r |= r >>  8;
  r |= r >> 16;
  r++;
  return r;
}

/****************************************************************************
 * Name: sty32c2_configwaitints
 *
 * Description:
 *   Enable/disable SDIO interrupts needed to support the wait function
 *
 * Input Parameters:
 *   priv       - A reference to the SDIO device state structure
 *   waitmask   - The set of bits in the SDIO MASK register to set
 *   waitevents - Waited for events
 *   wkupevent  - Wake-up events
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sty32c2_configwaitints(struct sty32c2_dev_s *priv, uint32_t waitints,
                                 sdio_eventset_t waitevents,
                                 sdio_eventset_t wkupevent)
{
  irqstate_t flags;

  flags            = enter_critical_section();
  priv->waitevents = waitevents;
  priv->wkupevent  = wkupevent;
  priv->waitints   = waitints;

  putreg32(priv->xfrints | priv->waitints | priv->cintints,
           STY32C2_SDIRQ_ENABLE);

  mcinfo("pending irq: %08" PRIx32 " enabled irq: %08" PRIx32 "\n",
         getreg32(STY32C2_SDIRQ_PENDING),
         getreg32(STY32C2_SDIRQ_ENABLE));

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: sty32c2_configxfrints
 *
 * Description:
 *   Enable SDIO interrupts needed to support the data transfer event
 *
 * Input Parameters:
 *   priv    - A reference to the SDIO device state structure
 *   xfrints - The set of bits in the SDIO MASK register to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sty32c2_configxfrints(struct sty32c2_dev_s *priv, uint32_t xfrints)
{
  irqstate_t flags;

  flags = enter_critical_section();
  priv->xfrints = xfrints;

  putreg32(priv->xfrints | priv->waitints | priv->cintints,
           STY32C2_SDIRQ_ENABLE);

  mcinfo("pending irq: %08" PRIx32 " enabled irq: %08" PRIx32 "\n",
         getreg32(STY32C2_SDIRQ_PENDING),
         getreg32(STY32C2_SDIRQ_ENABLE));

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: sty32c2_interrupt
 *
 * Description:
 *   SDIO interrupt handler
 *
 * Input Parameters:
 *   dev - An instance of the SDIO device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int sty32c2_interrupt(int irq, void *context, void *arg)
{
  struct sty32c2_dev_s *priv = &g_sdiodev;
  uint32_t pending;

  mcinfo("pending irq: %08" PRIx32 " enabled irq: %08" PRIx32 "\n",
         getreg32(STY32C2_SDIRQ_PENDING),
         getreg32(STY32C2_SDIRQ_ENABLE));

  pending = getreg32(STY32C2_SDIRQ_PENDING) & getreg32(STY32C2_SDIRQ_ENABLE);
  putreg32(pending, STY32C2_SDIRQ_PENDING);

  mcinfo("pending irq: %08" PRIx32 " enabled irq: %08" PRIx32 "\n",
         getreg32(STY32C2_SDIRQ_PENDING),
         getreg32(STY32C2_SDIRQ_ENABLE));

  /* check for card change interrupt */

  if ((pending & STY32C2_INT_CARDDETECT) != 0)
    {
      mcinfo("Card Detect State: %lu\n", getreg32(STY32C2_SDPHY_CARD_DETECT));

      /* Perform callback */

      if (priv->do_sdio_card != NULL)
        {
          priv->do_sdio_card(priv->do_sdio_arg);
        }
    }

  /* check for DMA write interrupt */

  if ((pending & STY32C2_INT_BLOCK2MEM) != 0)
    {
      if ((priv->waitevents & SDIOWAIT_TRANSFERDONE) != 0)
        {
          sty32c2_configxfrints(priv, 0);

          /* Yes.. wake up any waiting threads */

          sty32c2_endwait(priv, SDIOWAIT_TRANSFERDONE);
        }
    }

  /* check for DMA read interrupt */

  if ((pending & STY32C2_INT_MEM2BLOCK) != 0)
    {
      if ((priv->waitevents & SDIOWAIT_TRANSFERDONE) != 0)
        {
          sty32c2_configxfrints(priv, 0);

          /* Yes.. wake up any waiting threads */

          sty32c2_endwait(priv, SDIOWAIT_TRANSFERDONE);
        }
    }

#if 0 /* Not used */
  /* check for command complete interrupt */

  if ((pending & STY32C2_INT_CMDDONE) != 0)
    {
      if ((priv->waitevents &
            (SDIOWAIT_CMDDONE | SDIOWAIT_RESPONSEDONE)) != 0)
        {
          /* Yes.. wake the thread up */

          sty32c2_endwait(priv, SDIOWAIT_CMDDONE);
        }
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: sty32c2_eventtimeout
 *
 * Description:
 *   The watchdog timeout setup when the event wait start has expired without
 *   any other waited-for event occurring.
 *
 * Input Parameters:
 *   arg    - The argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

static void sty32c2_eventtimeout(wdparm_t arg)
{
  struct sty32c2_dev_s *priv = (struct sty32c2_dev_s *)arg;

  DEBUGASSERT((priv->waitevents & SDIOWAIT_TIMEOUT) != 0 ||
              priv->wkupevent != 0);

  mcinfo("pending irq: %08" PRIx32 " enabled irq: %08" PRIx32 "\n",
         getreg32(STY32C2_SDIRQ_PENDING),
         getreg32(STY32C2_SDIRQ_ENABLE));

  if ((priv->waitevents & SDIOWAIT_TIMEOUT) != 0)
    {
      sty32c2_endwait(priv, SDIOWAIT_TIMEOUT);
      mcerr("Timeout\n");
    }
}

/****************************************************************************
 * Name: sty32c2_endwait
 *
 * Description:
 *   Wake up a waiting thread if the waited-for event has occurred.
 *
 * Input Parameters:
 *   priv      - An instance of the SDIO device interface
 *   wkupevent - The event that caused the wait to end
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

static void sty32c2_endwait(struct sty32c2_dev_s *priv,
                          sdio_eventset_t wkupevent)
{
  /* Cancel the watchdog timeout */

  wd_cancel(&priv->waitwdog);

  /* Disable event-related interrupts */

  sty32c2_configwaitints(priv, 0, 0, wkupevent);

  /* Wake up the waiting thread */

  nxsem_post(&priv->waitsem);
}

/****************************************************************************
 * Name: sty32c2_reset
 *
 * Description:
 *   Reset the SDIO controller.  Undo all setup and initialization.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sty32c2_reset(struct sdio_dev_s *dev)
{
  struct sty32c2_dev_s *priv = (struct sty32c2_dev_s *)dev;
  irqstate_t flags;

  flags = enter_critical_section();

  priv->waitevents  = 0;
  priv->waitints    = 0;
  priv->wkupevent   = 0;

  wd_cancel(&priv->waitwdog);

  priv->xfrints     = 0;

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: sty32c2_capabilities
 *
 * Description:
 *   Get capabilities (and limitations) of the SDIO driver (optional)
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Returns a bitset of status values (see SDIO_CAPS_* defines)
 *
 ****************************************************************************/

static sdio_capset_t sty32c2_capabilities(struct sdio_dev_s *dev)
{
  sdio_capset_t caps = 0;

  /* LiteSDCard only supports 4-bit bus width */

  caps |= SDIO_CAPS_4BIT_ONLY;
  caps |= SDIO_CAPS_DMASUPPORTED;
  caps |= SDIO_CAPS_DMABEFOREWRITE;

  return caps;
}

/****************************************************************************
 * Name: sty32c2_status
 *
 * Description:
 *   Get SDIO status.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Returns a bitset of status values (see sty32c2_dev_s* defines)
 *
 ****************************************************************************/

static sdio_statset_t sty32c2_status(struct sdio_dev_s *dev)
{
  struct sty32c2_dev_s *priv = (struct sty32c2_dev_s *)dev;

  sdio_statset_t cd = priv->cdstatus;
  mcinfo("CD Status: %u\n", cd);

  return cd;
}

/****************************************************************************
 * Name: sty32c2_widebus
 *
 * Description:
 *   Called after change in Bus width has been selected (via ACMD6).  Most
 *   controllers will need to perform some special operations to work
 *   correctly in the new bus mode.
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *   wide - true: wide bus (4-bit) bus mode enabled
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sty32c2_widebus(struct sdio_dev_s *dev, bool wide)
{
  /* LiteSDCard only supports 4-bit bus width.
   * Nothing to do here.
   */
}

/****************************************************************************
 * Name: sty32c2_clock
 *
 * Description:
 *   Enable/disable SDIO clocking
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *   rate - Specifies the clocking to use (see enum sdio_clock_e)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sty32c2_clock(struct sdio_dev_s *dev, enum sdio_clock_e rate)
{
  uint32_t clk_freq;

  switch (rate)
    {
      /* Return early - SDPHY doesnt support clock disabling */

      default:
      case CLOCK_SDIO_DISABLED:
        return;

      /* Enable in initial ID mode clocking (<400KHz) */

      case CLOCK_IDMODE:
        clk_freq = CONFIG_STY32C2_IDMODE_FREQ;
        break;

      /* Enable in MMC normal operation clocking */

      case CLOCK_MMC_TRANSFER:
        clk_freq = CONFIG_STY32C2_MMCXFR_FREQ;
        break;

      /* SD normal operation clocking (wide 4-bit mode) */

      case CLOCK_SD_TRANSFER_4BIT:
        clk_freq = CONFIG_STY32C2_SD4BIT_FREQ;
        break;
    }

  /* Set the new clock frequency along with the clock enable/disable bit */

  uint32_t divider;
  divider = clk_freq ? sty32c2_get_cpuclk() / clk_freq : MAX_DIVIDER;
  divider = sty32c2_pow2roundup(divider);
  divider = min(max(divider, 2), MAX_DIVIDER);

  /* this is the *effective* new clk_freq */

  clk_freq = sty32c2_get_cpuclk() / divider;
  if (clk_freq > 1000000)
    {
      mcinfo("Setting SDCard clk freq to %ld MHz\n", clk_freq / 1000000);
    }
  else
    {
      mcinfo("Setting SDCard clk freq to %ld KHz\n", clk_freq / 1000);
    }

  putreg32(divider, STY32C2_SDPHY_CLOCKER_DIVIDER);
}

/****************************************************************************
 * Name: sty32c2_attach
 *
 * Description:
 *   Attach and prepare interrupts
 *
 * Input Parameters:
 *   dev - An instance of the SDIO device interface
 *
 * Returned Value:
 *   OK on success; A negated errno on failure.
 *
 ****************************************************************************/

static int sty32c2_attach(struct sdio_dev_s *dev)
{
  int ret;

  /* Attach the SDIO interrupt handler */

  ret = irq_attach(STY32C2_IRQ_SDCARD, sty32c2_interrupt, NULL);
  if (ret == OK)
    {
      putreg32(0xffffffff, STY32C2_SDIRQ_PENDING);
      putreg32(0, STY32C2_SDIRQ_ENABLE);
      up_enable_irq(STY32C2_IRQ_SDCARD);
    }

  return ret;
}

/****************************************************************************
 * Name: sty32c2_sendcmd
 *
 * Description:
 *   Send the SDIO command
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *   cmd  - The command to send (32-bits, encoded)
 *   arg  - 32-bit argument required with some commands
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int sty32c2_sendcmd(struct sdio_dev_s *dev, uint32_t cmd,
                         uint32_t arg)
{
  uint32_t transfer = 0;
  uint32_t resplen = 0;
  uint32_t regval = 0;
  uint32_t cmdidx = 0;

  irqstate_t flags;
  flags = enter_critical_section();

  /* Set WAITRESP bits */

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
    case MMCSD_NO_RESPONSE:
      resplen = SDCARD_CTRL_RESPONSE_NONE;
      break;

    case MMCSD_R1_RESPONSE:
    case MMCSD_R1B_RESPONSE:
    case MMCSD_R3_RESPONSE:
    case MMCSD_R4_RESPONSE:
    case MMCSD_R5_RESPONSE:
    case MMCSD_R6_RESPONSE:
    case MMCSD_R7_RESPONSE:
      resplen = SDCARD_CTRL_RESPONSE_SHORT;
      break;

    case MMCSD_R2_RESPONSE:
      resplen = SDCARD_CTRL_RESPONSE_LONG;
      break;
    }

  switch (cmd & MMCSD_DATAXFR_MASK)
    {
    case MMCSD_NODATAXFR:
      transfer = SD_CTL_DATA_XFER_NONE;
      break;

    case MMCSD_RDSTREAM:
      transfer = SD_CTL_DATA_XFER_READ;
      break;

    case MMCSD_WRSTREAM:
      transfer = SD_CTL_DATA_XFER_WRITE;
      break;

    case MMCSD_RDDATAXFR:
      transfer = SD_CTL_DATA_XFER_READ;
      break;

    case MMCSD_WRDATAXFR:
      transfer = SD_CTL_DATA_XFER_WRITE;
      break;
    }

  /* Write the SDIO CMD */

  cmdidx = (cmd & MMCSD_CMDIDX_MASK) >> MMCSD_CMDIDX_SHIFT;
  regval = (cmdidx << 8) | (transfer << 5) | resplen;
  putreg32(arg, STY32C2_SDCORE_CMD_ARGUMENT);
  putreg32(regval, STY32C2_SDCORE_CMD_COMMAND);
  putreg32(1, STY32C2_SDCORE_CMD_SEND);

  mcinfo("cmd: %" PRIu32 " cmdid: %" PRIu32 " arg: %08" PRIx32
         " regval: %08" PRIx32 "\n",
         cmd, cmdidx, arg, regval);

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: sty32c2_blocksetup
 *
 * Description:
 *   Configure block size and the number of blocks for next transfer
 *
 * Input Parameters:
 *   dev       - An instance of the SDIO device interface
 *   blocklen  - The selected block size.
 *   nblocklen - The number of blocks to transfer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sty32c2_blocksetup(struct sdio_dev_s *dev,
                             unsigned int blocklen, unsigned int nblocks)
{
  struct sty32c2_dev_s *priv = (struct sty32c2_dev_s *)dev;

  mcinfo("blocklen=%d, total transfer=%d (%d blocks)\n", blocklen,
         blocklen * nblocks, nblocks);

  /* Configure block size for next transfer */

  priv->block_size = blocklen;

  putreg32(blocklen, STY32C2_SDCORE_BLOCK_LENGTH);
  putreg32(nblocks, STY32C2_SDCORE_BLOCK_COUNT);
}

/****************************************************************************
 * Name: sty32c2_cancel
 *
 * Description:
 *   Cancel the data transfer setup of SDIO_RECVSETUP, SDIO_SENDSETUP,
 *   SDIO_DMARECVSETUP or SDIO_DMASENDSETUP.  This must be called to cancel
 *   the data transfer setup if, for some reason, you cannot perform the
 *   transfer.
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *
 * Returned Value:
 *   OK is success; a negated errno on failure
 *
 ****************************************************************************/

static int sty32c2_cancel(struct sdio_dev_s *dev)
{
  struct sty32c2_dev_s *priv = (struct sty32c2_dev_s *)dev;

  /* Cancel any watchdog timeout */

  wd_cancel(&priv->waitwdog);

  /* Stop DMA transfers */

  putreg32(0, STY32C2_SDBLOCK2MEM_DMA_ENABLE);
  putreg32(0, STY32C2_SDMEM2BLOCK_DMA_ENABLE);

  return OK;
}

/****************************************************************************
 * Name: sty32c2_dmarecvsetup
 *
 * Description:
 *   Setup to perform a read DMA.  If the processor supports a data cache,
 *   then this method will also make sure that the contents of the DMA memory
 *   and the data cache are coherent.  For read transfers this may mean
 *   invalidating the data cache.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   buffer - The memory to DMA from
 *   buflen - The size of the DMA transfer in bytes
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

static int sty32c2_recvsetup(struct sdio_dev_s *dev, uint8_t *buffer,
                           size_t nbytes)
{
  struct sty32c2_dev_s *priv = (struct sty32c2_dev_s *)dev;
  DEBUGASSERT(priv != NULL && buffer != NULL && nbytes > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

  mcinfo("buffer: %p bytes: %u\n", buffer, nbytes);

  sty32c2_configxfrints(priv, STY32C2_INT_BLOCK2MEM);

  /* flush CPU d-cache */

  up_invalidate_dcache_all();

  putreg32(0, STY32C2_SDBLOCK2MEM_DMA_ENABLE);
  putreg32((uintptr_t)buffer >> 32, STY32C2_SDBLOCK2MEM_DMA_BASE);
  putreg32((uintptr_t)buffer, STY32C2_SDBLOCK2MEM_DMA_BASE + 0x04);
  putreg32(nbytes, STY32C2_SDBLOCK2MEM_DMA_LENGTH);
  putreg32(1, STY32C2_SDBLOCK2MEM_DMA_ENABLE);

  return OK;
}

/****************************************************************************
 * Name: sty32c2_sendsetup
 *
 * Description:
 *   Setup to perform a write DMA.  If the processor supports a data cache,
 *   then this method will also make sure that the contents of the DMA memory
 *   and the data cache are coherent.  For write transfers, this may mean
 *   flushing the data cache.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   buffer - The memory to DMA into
 *   nbytes - The size of the DMA transfer in bytes
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

static int sty32c2_sendsetup(struct sdio_dev_s *dev,
                           const uint8_t *buffer, size_t nbytes)
{
  struct sty32c2_dev_s *priv = (struct sty32c2_dev_s *)dev;
  DEBUGASSERT(priv != NULL && buffer != NULL && nbytes > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

  mcinfo("buffer: %p bytes: %u\n", buffer, nbytes);

  sty32c2_configxfrints(priv, STY32C2_INT_MEM2BLOCK);

  /* flush CPU d-cache */

  up_invalidate_dcache_all();

  putreg32(0, STY32C2_SDMEM2BLOCK_DMA_ENABLE);
  putreg32((uintptr_t)buffer >> 32, STY32C2_SDMEM2BLOCK_DMA_BASE);
  putreg32((uintptr_t)buffer, STY32C2_SDMEM2BLOCK_DMA_BASE + 0x04);
  putreg32(nbytes, STY32C2_SDMEM2BLOCK_DMA_LENGTH);
  putreg32(1, STY32C2_SDMEM2BLOCK_DMA_ENABLE);

  return OK;
}

/****************************************************************************
 * Name: sty32c2_waitresponse
 *
 * Description:
 *   Poll-wait for the response to the last command to be ready.
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *   cmd  - The command that was sent.  See 32-bit command definitions above.
 *
 * Returned Value:
 *   OK is success; a negated errno on failure
 *
 ****************************************************************************/

static int sty32c2_waitresponse(struct sdio_dev_s *dev, uint32_t cmd)
{
  uint32_t ev;

  for (; ; )
    {
      if ((cmd & MMCSD_DATAXFR_MASK) == 0)
        {
          ev = getreg32(STY32C2_SDCORE_CMD_EVENT);
        }
      else
        {
          ev = getreg32(STY32C2_SDCORE_DATA_EVENT);
        }

      mcinfo("%lu\n", ev);

      if (ev & STY32C2_EV_CMDDONE)
        break;

      nxsig_usleep(10);
    }

  if (ev & STY32C2_EV_WRERROR)
    {
      return -EIO;
    }

  if (ev & STY32C2_EV_TIMEOUT)
    {
      return -ETIMEDOUT;
    }

  if (ev & STY32C2_EV_CRCERROR)
    {
      return -EILSEQ;
    }

  return OK;
}

static int sty32c2_recvlong(struct sdio_dev_s *dev, uint32_t cmd,
                          uint32_t rlong[4])
{
  uint32_t regval;
  int ret = OK;

  /* R3  OCR (48-bit)
   *     47        0               Start bit
   *     46        0               Transmission bit (0=from card)
   *     45:40     bit5   - bit0   Reserved
   *     39:8      bit31  - bit0   32-bit OCR register
   *     7:1       bit6   - bit0   Reserved
   *     0         1               End bit
   */

  /* Check that this is the correct response to this command */

#ifdef CONFIG_DEBUG_MEMCARD_INFO
  if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R2_RESPONSE)
    {
      mcerr("ERROR: Wrong response CMD=%08x\n", cmd);
      ret = -EINVAL;
    }
  else
#endif
    {
      /* Check if a timeout or CRC error occurred */

      regval = getreg32(STY32C2_SDCORE_CMD_EVENT);
      if ((regval & STY32C2_EV_TIMEOUT) != 0)
        {
          mcerr("ERROR: Command timeout: %08" PRIx32 "\n", regval);
          ret = -ETIMEDOUT;
        }
      else if ((regval & STY32C2_EV_CRCERROR) != 0)
        {
          mcerr("ERROR: CRC failure: %08" PRIx32 "\n", regval);
          ret = -EIO;
        }
    }

  if (rlong != NULL)
    {
      rlong[0] = getreg32(STY32C2_SDCORE_CMD_RESPONSE + 0x00);
      rlong[1] = getreg32(STY32C2_SDCORE_CMD_RESPONSE + 0x04);
      rlong[2] = getreg32(STY32C2_SDCORE_CMD_RESPONSE + 0x08);
      rlong[3] = getreg32(STY32C2_SDCORE_CMD_RESPONSE + 0x0c);
    }

  return ret;
}

static int sty32c2_recvshort(struct sdio_dev_s *dev, uint32_t cmd,
                           uint32_t *rshort)
{
  uint32_t regval;
  int ret = OK;

  /* R3  OCR (48-bit)
   *     47        0               Start bit
   *     46        0               Transmission bit (0=from card)
   *     45:40     bit5   - bit0   Reserved
   *     39:8      bit31  - bit0   32-bit OCR register
   *     7:1       bit6   - bit0   Reserved
   *     0         1               End bit
   */

  /* Check that this is the correct response to this command */

#ifdef CONFIG_DEBUG_MEMCARD_INFO
  if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R1_RESPONSE &&
      (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R1B_RESPONSE &&
      (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R3_RESPONSE &&
      (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R4_RESPONSE &&
      (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R5_RESPONSE &&
      (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R6_RESPONSE &&
      (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R7_RESPONSE)
    {
      mcerr("ERROR: Wrong response CMD=%08x\n", cmd);
      ret = -EINVAL;
    }
  else
#endif
    {
      /* Check if a timeout or CRC error occurred */

      regval = getreg32(STY32C2_SDCORE_CMD_EVENT);
      if ((regval & STY32C2_EV_TIMEOUT) != 0)
        {
          mcerr("ERROR: Command timeout: %08" PRIx32 "\n", regval);
          ret = -ETIMEDOUT;
        }
      else if ((regval & STY32C2_EV_CRCERROR) != 0)
        {
          mcerr("ERROR: CRC failure: %08" PRIx32 "\n", regval);
          ret = -EIO;
        }
    }

  if (rshort != NULL)
    {
      *rshort = getreg32(STY32C2_SDCORE_CMD_RESPONSE + 0x0c);
    }

  return ret;
}

/****************************************************************************
 * Name: sty32c2_waitenable
 *
 * Description:
 *   Enable/disable of a set of SDIO wait events.  This is part of the
 *   the SDIO_WAITEVENT sequence.  The set of to-be-waited-for events is
 *   configured before calling sty32c2_eventwait.  This is done in this way
 *   to help the driver to eliminate race conditions between the command
 *   setup and the subsequent events.
 *
 *   The enabled events persist until either (1) SDIO_WAITENABLE is called
 *   again specifying a different set of wait events, or (2) SDIO_EVENTWAIT
 *   returns.
 *
 * Input Parameters:
 *   dev      - An instance of the SDIO device interface
 *   eventset - A bitset of events to enable or disable (see SDIOWAIT_*
 *              definitions). 0=disable; 1=enable.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sty32c2_waitenable(struct sdio_dev_s *dev,
                             sdio_eventset_t eventset, uint32_t timeout)
{
  struct sty32c2_dev_s *priv = (struct sty32c2_dev_s *)dev;

  DEBUGASSERT(priv != NULL);

  sty32c2_configwaitints(priv, 0, eventset, 0);

  /* Check if the timeout event is specified in the event set */

  if ((priv->waitevents & SDIOWAIT_TIMEOUT) != 0)
    {
      int delay;
      int ret;

      /* Yes.. Handle a cornercase: The user requested a timeout event but
       * with timeout == 0?
       */

      if (timeout == 0)
        {
          priv->wkupevent = SDIOWAIT_TIMEOUT;
          return;
        }

      /* Start the watchdog timer */

      delay = MSEC2TICK(timeout);
      ret   = wd_start(&priv->waitwdog, delay,
                       sty32c2_eventtimeout, (wdparm_t)priv);
      if (ret < 0)
        {
          mcerr("ERROR: wd_start failed: %d\n", ret);
        }
    }
}

/****************************************************************************
 * Name: sty32c2_eventwait
 *
 * Description:
 *   Wait for one of the enabled events to occur (or a timeout).  Note that
 *   all events enabled by SDIO_WAITEVENTS are disabled when sty32c2_eventwait
 *   returns.  SDIO_WAITEVENTS must be called again before sty32c2_eventwait
 *   can be used again.
 *
 * Input Parameters:
 *   dev     - An instance of the SDIO device interface
 *   timeout - Maximum time in milliseconds to wait.  Zero means immediate
 *             timeout with no wait.  The timeout value is ignored if
 *             SDIOWAIT_TIMEOUT is not included in the waited-for eventset.
 *
 * Returned Value:
 *   Event set containing the event(s) that ended the wait.  Should always
 *   be non-zero.  All events are disabled after the wait concludes.
 *
 ****************************************************************************/

static sdio_eventset_t sty32c2_eventwait(struct sdio_dev_s *dev)
{
  struct sty32c2_dev_s *priv = (struct sty32c2_dev_s *)dev;
  sdio_eventset_t wkupevent = 0;
  int ret;

  DEBUGASSERT((priv->waitevents != 0 && priv->wkupevent == 0) ||
              (priv->waitevents == 0 && priv->wkupevent != 0));
  for (; ; )
    {
      ret = nxsem_wait_uninterruptible(&priv->waitsem);
      if (ret < 0)
        {
          sty32c2_configwaitints(priv, 0, 0, 0);
          wd_cancel(&priv->waitwdog);
          return SDIOWAIT_ERROR;
        }

      wkupevent = priv->wkupevent;
      if (wkupevent != 0)
        {
          break;
        }
    }

  sty32c2_configwaitints(priv, 0, 0, 0);
  return wkupevent;
}

/****************************************************************************
 * Name: sty32c2_registercallback
 *
 * Description:
 *   Register a callback that that will be invoked on any media status
 *   change.  Callbacks should not be made from interrupt handlers, rather
 *   interrupt level events should be handled by calling back on the work
 *   thread.
 *
 *   When this method is called, all callbacks should be disabled until they
 *   are enabled via a call to SDIO_CALLBACKENABLE
 *
 * Input Parameters:
 *   dev -      Device-specific state data
 *   callback - The function to call on the media change
 *   arg -      A caller provided value to return with the callback
 *
 * Returned Value:
 *   0 on success; negated errno on failure.
 *
 ****************************************************************************/

static int sty32c2_registercallback(struct sdio_dev_s *dev,
                                  worker_t callback, void *arg)
{
  struct sty32c2_dev_s *priv = (struct sty32c2_dev_s *)dev;

  /* Disable callbacks and register this callback and is argument */

  mcinfo("Register %p(%p)\n", callback, arg);
  DEBUGASSERT(priv != NULL);

  priv->cbevents = 0;
  priv->cbarg    = arg;
  priv->callback = callback;

  return OK;
}

/****************************************************************************
 * Name: sty32c2_callbackenable
 *
 * Description:
 *   Enable/disable of a set of SDIO callback events.  This is part of the
 *   the SDIO callback sequence.  The set of events is configured to enabled
 *   callbacks to the function provided in sty32c2_registercallback.
 *
 *   Events are automatically disabled once the callback is performed and no
 *   further callback events will occur until they are again enabled by
 *   calling this method.
 *
 * Input Parameters:
 *   dev      - An instance of the SDIO device interface
 *   eventset - A bitset of events to enable or disable (see SDIOMEDIA_*
 *              definitions). 0=disable; 1=enable.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sty32c2_callbackenable(struct sdio_dev_s *dev,
                                 sdio_eventset_t eventset)
{
  struct sty32c2_dev_s *priv = (struct sty32c2_dev_s *)dev;

  mcinfo("eventset: %02x\n", eventset);
  DEBUGASSERT(priv != NULL);

  priv->cbevents = eventset;
  sty32c2_callback(priv);
}

/****************************************************************************
 * Name: sty32c2_callback
 *
 * Description:
 *   Perform callback.
 *
 * Assumptions:
 *   This function does not execute in the context of an interrupt handler.
 *   It may be invoked on any user thread or scheduled on the work thread
 *   from an interrupt handler.
 *
 ****************************************************************************/

static void sty32c2_callback(void *arg)
{
  struct sty32c2_dev_s *priv = (struct sty32c2_dev_s *)arg;

  /* Is a callback registered? */

  DEBUGASSERT(priv != NULL);
  mcinfo("Callback %p(%p) cbevents: %02x cdstatus: %02x\n",
         priv->callback, priv->cbarg, priv->cbevents, priv->cdstatus);

  if (priv->callback)
    {
      /* Yes.. Check for enabled callback events */

      if ((priv->cdstatus & SDIO_STATUS_PRESENT) != 0)
        {
          /* Media is present.  Is the media inserted event enabled? */

          if ((priv->cbevents & SDIOMEDIA_INSERTED) == 0)
            {
              /* No... return without performing the callback */

              return;
            }
        }
      else
        {
          /* Media is not present.  Is the media eject event enabled? */

          if ((priv->cbevents & SDIOMEDIA_EJECTED) == 0)
            {
              /* No... return without performing the callback */

              return;
            }
        }

      /* Perform the callback, disabling further callbacks.  Of course, the
       * the callback can (and probably should) re-enable callbacks.
       */

      priv->cbevents = 0;

      /* Callbacks cannot be performed in the context of an interrupt
       * handler.  If we are in an interrupt handler, then queue the
       * callback to be performed later on the work thread.
       */

      if (up_interrupt_context())
        {
          /* Yes.. queue it */

           mcinfo("Queuing callback to %p(%p)\n",
                  priv->callback, priv->cbarg);
           work_queue(HPWORK, &priv->cbwork, priv->callback,
                      priv->cbarg, 0);
        }
      else
        {
          /* No.. then just call the callback here */

          mcinfo("Callback to %p(%p)\n", priv->callback, priv->cbarg);
          priv->callback(priv->cbarg);
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sty32c2_sdio_get_card_detect
 *
 * Description:
 *   Get the card detection state determined by the peripheral.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Card detected state.
 *   True when a card is present.
 *
 ****************************************************************************/

bool sty32c2_sdio_get_card_detect(void)
{
  return getreg32(STY32C2_SDPHY_CARD_DETECT) == 0;
}

/****************************************************************************
 * Name: sty32c2_sdio_set_card_isr
 *
 * Description:
 *   SDIO card generates interrupt via SDIO_DATA_1 pin.
 *   Called by board-specific logic to register an ISR for SDIO card.
 *
 * Input Parameters:
 *   func  - callback function.
 *   arg   - arg to be passed to the function.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sty32c2_sdio_set_card_isr(struct sdio_dev_s *dev,
                             void (*func)(void *), void *arg)
{
  irqstate_t flags;
  uint32_t regval;
  struct sty32c2_dev_s *priv = (struct sty32c2_dev_s *)dev;

  mcinfo("Enable Card Detect ISR\n");

  mcinfo("pending irq: %08" PRIx32 " enabled irq: %08" PRIx32 "\n",
         getreg32(STY32C2_SDIRQ_PENDING),
         getreg32(STY32C2_SDIRQ_ENABLE));

  priv->do_sdio_card = func;
  priv->do_sdio_arg = arg;

  if (priv->do_sdio_card != NULL)
    {
      priv->cintints = STY32C2_INT_CARDDETECT;
    }
  else
    {
      priv->cintints = 0;
    }

  flags  = enter_critical_section();
  regval = getreg32(STY32C2_SDIRQ_ENABLE);
  regval = (regval & ~STY32C2_INT_CARDDETECT) | priv->cintints;
  putreg32(regval, STY32C2_SDIRQ_ENABLE);

  mcinfo("pending irq: %08" PRIx32 " enabled irq: %08" PRIx32 "\n",
         getreg32(STY32C2_SDIRQ_PENDING),
         getreg32(STY32C2_SDIRQ_ENABLE));

  mcinfo("Card Detect State: %lu\n", getreg32(STY32C2_SDPHY_CARD_DETECT));

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: sdio_initialize
 *
 * Description:
 *   Initialize SDIO for operation.
 *
 * Input Parameters:
 *   slotno - Not used.
 *
 * Returned Value:
 *   A reference to an SDIO interface structure.  NULL is returned on
 *   failures.
 *
 ****************************************************************************/

struct sdio_dev_s *sdio_initialize(int slotno)
{
  struct sty32c2_dev_s *priv = &g_sdiodev;

  mcinfo("slotno: %d\n", slotno);

  nxsem_init(&priv->waitsem, 0, 0);
  nxsem_set_protocol(&priv->waitsem, SEM_PRIO_NONE);

  sty32c2_reset(&priv->dev);
  return &g_sdiodev.dev;
}

/****************************************************************************
 * Name: sdio_mediachange
 *
 * Description:
 *   Called by board-specific logic -- possibly from an interrupt handler --
 *   in order to signal to the driver that a card has been inserted or
 *   removed from the slot
 *
 * Input Parameters:
 *   dev        - An instance of the SDIO driver device state structure.
 *   cardinslot - true is a card has been detected in the slot; false if a
 *                card has been removed from the slot.  Only transitions
 *                (inserted->removed or removed->inserted should be reported)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sdio_mediachange(struct sdio_dev_s *dev, bool cardinslot)
{
  struct sty32c2_dev_s *priv = (struct sty32c2_dev_s *)dev;
  sdio_statset_t cdstatus;
  irqstate_t flags;

  flags = enter_critical_section();

  cdstatus = priv->cdstatus;
  if (cardinslot)
    {
      priv->cdstatus |= SDIO_STATUS_PRESENT;
    }
  else
    {
      priv->cdstatus &= ~SDIO_STATUS_PRESENT;
    }

  leave_critical_section(flags);

  mcinfo("cdstatus OLD: %02x NEW: %02x\n", cdstatus, priv->cdstatus);

  if (cdstatus != priv->cdstatus)
    {
      sty32c2_callback(priv);
    }
}
