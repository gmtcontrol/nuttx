/****************************************************************************
 * arch/arm/src/ra6m5/ra6m5_serial.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <semaphore.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/serial.h>
#include <nuttx/power/pm.h>

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <arch/board/board.h>

#include "chip.h"
#include "ra6m5_gpio.h"
#include "ra6m5_sci.h"
#ifdef SERIAL_HAVE_DMA
#  include "ra6m5_dma.h"
#endif
#ifdef SERIAL_HAVE_DTC
#  include "ra6m5_dtc.h"
#endif
#include "ra6m5_rcc.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef USE_SERIALDRIVER
#ifdef HAVE_UART

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ra6m5_serial_s
{
  struct uart_dev_s dev;       /* Generic UART device */
  uint8_t           ie;        /* Saved interrupt mask bits value */
  uint8_t           sr;        /* Saved status bits */

  /* Has been initialized and HW is setup. */

  bool              initialized;

#ifdef CONFIG_PM
  bool              suspended; /* UART device has been suspended. */

  /* Interrupt mask value stored before suspending for stop mode. */

  uint8_t           suspended_ie;
#endif

  /* If termios are supported, then the following fields may vary at
   * runtime.
   */

#ifdef CONFIG_SERIAL_TERMIOS
  uint8_t           parity;    /* 0=none, 1=odd, 2=even */
  uint8_t           bits;      /* Number of bits (7 or 8) */
  bool              stopbits2; /* True: Configure with 2 stop bits instead of 1 */
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  bool              iflow;     /* input flow control (RTS) enabled */
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  bool              oflow;     /* output flow control (CTS) enabled */
#endif
  uint32_t          baud;      /* Configured baud */
#else
  const uint8_t     parity;    /* 0=none, 1=odd, 2=even */
  const uint8_t     bits;      /* Number of bits (7 or 8) */
  const bool        stopbits2; /* True: Configure with 2 stop bits instead of 1 */
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  const bool        iflow;     /* input flow control (RTS) enabled */
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  const bool        oflow;     /* output flow control (CTS) enabled */
#endif
  const uint32_t    baud;      /* Configured baud */
#endif
  const uint8_t     irqrxi;    /* Receive data full IRQ */
  const uint8_t     irqtxi;    /* Transmit data empty IRQ */
  const uint8_t     irqtei;    /* Transmit end IRQ */
  const uint8_t     irqeri;    /* Receive error IRQ */
  uint32_t          clock;     /* Peripheral frequency */
  const uint32_t    base;      /* Base address of SCI registers */
  const uint32_t    tx_gpio;   /* U[S]ART TX GPIO pin configuration */
  const uint32_t    rx_gpio;   /* U[S]ART RX GPIO pin configuration */
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  const uint32_t    rts_gpio;  /* U[S]ART RTS GPIO pin configuration */
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  const uint32_t    cts_gpio;  /* U[S]ART CTS GPIO pin configuration */
#endif

#ifdef SERIAL_HAVE_DMA
  const unsigned int rxdma_channel; /* DMA channel assigned */
#endif

#ifdef SERIAL_HAVE_DTC
  const dtc_static_transfer_data_cfg_t *txdtc_config;
  const dtc_static_transfer_data_cfg_t *rxdtc_config;
#endif

  /* RX DMA state */

#ifdef SERIAL_HAVE_DMA
  DMA_HANDLE        rxdma;     /* currently-open receive DMA stream */
  bool              rxenable;  /* DMA-based reception en/disable */
#ifdef CONFIG_PM
  bool              rxdmasusp; /* Rx DMA suspended */
#endif
  uint32_t          rxdmanext; /* Next byte in the DMA buffer to be read */
  char       *const rxfifo;    /* Receive DMA buffer */
#endif

#ifdef HAVE_RS485
  const uint32_t    rs485_dir_gpio;     /* U[S]ART RS-485 DIR GPIO pin configuration */
  const bool        rs485_dir_polarity; /* U[S]ART RS-485 DIR pin state for TX enabled */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifndef CONFIG_SUPPRESS_UART_CONFIG
static void ra6m5serial_setformat(struct uart_dev_s *dev);
#endif
static int  ra6m5serial_setup(struct uart_dev_s *dev);
static void ra6m5serial_shutdown(struct uart_dev_s *dev);
static int  ra6m5serial_attach(struct uart_dev_s *dev);
static void ra6m5serial_detach(struct uart_dev_s *dev);
static int  ra6m5serial_interrupt(int irq, void *context,
                                    void *arg);
static int  ra6m5serial_ioctl(struct file *filep, int cmd,
                              unsigned long arg);
#ifndef SERIAL_HAVE_ONLY_DMA
static int  ra6m5serial_receive(struct uart_dev_s *dev,
                                unsigned int *status);
static void ra6m5serial_rxint(struct uart_dev_s *dev, bool enable);
static bool ra6m5serial_rxavailable(struct uart_dev_s *dev);
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool ra6m5serial_rxflowcontrol(struct uart_dev_s *dev,
                                      unsigned int nbuffered, bool upper);
#endif
static void ra6m5serial_send(struct uart_dev_s *dev, int ch);
static void ra6m5serial_txint(struct uart_dev_s *dev, bool enable);
static bool ra6m5serial_txready(struct uart_dev_s *dev);

#ifdef SERIAL_HAVE_DMA
static int  ra6m5serial_dmasetup(struct uart_dev_s *dev);
static void ra6m5serial_dmashutdown(struct uart_dev_s *dev);
static int  ra6m5serial_dmareceive(struct uart_dev_s *dev,
                                   unsigned int *status);
static void ra6m5serial_dmareenable(struct ra6m5_serial_s *priv);
#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool ra6m5serial_dmaiflowrestart(struct ra6m5_serial_s *priv);
#endif
static void ra6m5serial_dmarxint(struct uart_dev_s *dev, bool enable);
static bool ra6m5serial_dmarxavailable(struct uart_dev_s *dev);

static void ra6m5serial_dmarxcallback(DMA_HANDLE handle, uint8_t status,
                                      void *arg);
#endif

#ifdef CONFIG_PM
static void ra6m5serial_setsuspend(struct uart_dev_s *dev, bool suspend);
static void ra6m5serial_pm_setsuspend(bool suspend);
static void ra6m5serial_pmnotify(struct pm_callback_s *cb, int domain,
                                 enum pm_state_e pmstate);
static int  ra6m5serial_pmprepare(struct pm_callback_s *cb, int domain,
                                  enum pm_state_e pmstate);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Baud rate divisor information (UART mode) */
static const baud_setting_const_t g_async_baud[SCI_UART_NUM_DIVISORS_ASYNC] =
{
    {0U, 0U, 1U, 0U},                  /* BGDM, ABCS, ABCSE, n */
    {1U, 1U, 0U, 0U},
    {1U, 0U, 0U, 0U},
    {0U, 0U, 1U, 1U},
    {0U, 0U, 0U, 0U},
    {1U, 0U, 0U, 1U},
    {0U, 0U, 1U, 2U},
    {0U, 0U, 0U, 1U},
    {1U, 0U, 0U, 2U},
    {0U, 0U, 1U, 3U},
    {0U, 0U, 0U, 2U},
    {1U, 0U, 0U, 3U},
    {0U, 0U, 0U, 3U}
};

static const uint16_t g_div_coefficient[SCI_UART_NUM_DIVISORS_ASYNC] =
{
    6U,
    8U,
    16U,
    24U,
    32U,
    64U,
    96U,
    128U,
    256U,
    384U,
    512U,
    1024U,
    2048U,
};

#ifndef SERIAL_HAVE_ONLY_DMA
static const struct uart_ops_s g_sci_ops =
{
  .setup          = ra6m5serial_setup,
  .shutdown       = ra6m5serial_shutdown,
  .attach         = ra6m5serial_attach,
  .detach         = ra6m5serial_detach,
  .ioctl          = ra6m5serial_ioctl,
  .receive        = ra6m5serial_receive,
  .rxint          = ra6m5serial_rxint,
  .rxavailable    = ra6m5serial_rxavailable,
#  ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = ra6m5serial_rxflowcontrol,
#  endif
  .send           = ra6m5serial_send,
  .txint          = ra6m5serial_txint,
  .txready        = ra6m5serial_txready,
  .txempty        = ra6m5serial_txready,
};
#endif

#ifdef SERIAL_HAVE_DMA
static const struct uart_ops_s g_sci_dma_ops =
{
  .setup          = ra6m5serial_dmasetup,
  .shutdown       = ra6m5serial_dmashutdown,
  .attach         = ra6m5serial_attach,
  .detach         = ra6m5serial_detach,
  .ioctl          = ra6m5serial_ioctl,
  .receive        = ra6m5serial_dmareceive,
  .rxint          = ra6m5serial_dmarxint,
  .rxavailable    = ra6m5serial_dmarxavailable,
#  ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = ra6m5serial_rxflowcontrol,
#  endif
  .send           = ra6m5serial_send,
  .txint          = ra6m5serial_txint,
  .txready        = ra6m5serial_txready,
  .txempty        = ra6m5serial_txready,
};
#endif

/* I/O buffers */

#ifdef CONFIG_RA6M5_SCI0_UART
static char g_sci0rxbuffer[CONFIG_SCI0_RXBUFSIZE];
static char g_sci0txbuffer[CONFIG_SCI0_TXBUFSIZE];
#  ifdef CONFIG_SCI0_RXDMA
static char g_sci0rxfifo[RXDMA_BUFFER_SIZE];
#  endif
#endif

#ifdef CONFIG_RA6M5_SCI1_UART
static char g_sci1rxbuffer[CONFIG_SCI1_RXBUFSIZE];
static char g_sci1txbuffer[CONFIG_SCI1_TXBUFSIZE];
#  ifdef CONFIG_SCI1_RXDMA
static char g_sci1rxfifo[RXDMA_BUFFER_SIZE];
#  endif
#endif

#ifdef CONFIG_RA6M5_SCI2_UART
static char g_sci2rxbuffer[CONFIG_SCI2_RXBUFSIZE];
static char g_sci2txbuffer[CONFIG_SCI2_TXBUFSIZE];
#  ifdef CONFIG_SCI2_RXDMA
static char g_sci2rxfifo[RXDMA_BUFFER_SIZE];
#  endif
#endif

#ifdef CONFIG_RA6M5_SCI3_UART
static char g_sci3rxbuffer[CONFIG_SCI3_RXBUFSIZE];
static char g_sci3txbuffer[CONFIG_SCI3_TXBUFSIZE];
#  ifdef CONFIG_SCI3_RXDMA
static char g_sci3rxfifo[RXDMA_BUFFER_SIZE];
#  endif
#endif

#ifdef CONFIG_RA6M5_SCI4_UART
static char g_sci4rxbuffer[CONFIG_SCI4_RXBUFSIZE];
static char g_sci4txbuffer[CONFIG_SCI4_TXBUFSIZE];
#  ifdef CONFIG_SCI4_RXDMA
static char g_sci4rxfifo[RXDMA_BUFFER_SIZE];
#  endif
#endif

#ifdef CONFIG_RA6M5_SCI5_UART
static char g_sci5rxbuffer[CONFIG_SCI5_RXBUFSIZE];
static char g_sci5txbuffer[CONFIG_SCI5_TXBUFSIZE];
#  ifdef CONFIG_SCI5_RXDMA
static char g_sci5rxfifo[RXDMA_BUFFER_SIZE];
#  endif
#endif

#ifdef CONFIG_RA6M5_SCI6_UART
static char g_sci6rxbuffer[CONFIG_SCI6_RXBUFSIZE];
static char g_sci6txbuffer[CONFIG_SCI6_TXBUFSIZE];
#  ifdef CONFIG_SCI6_RXDMA
static char g_sci6rxfifo[RXDMA_BUFFER_SIZE];
#  endif
#endif

#ifdef CONFIG_RA6M5_SCI7_UART
static char g_sci7rxbuffer[CONFIG_SCI7_RXBUFSIZE];
static char g_sci7txbuffer[CONFIG_SCI7_TXBUFSIZE];
#  ifdef CONFIG_SCI7_RXDMA
static char g_sci7rxfifo[RXDMA_BUFFER_SIZE];
#  endif
#endif

#ifdef CONFIG_RA6M5_SCI8_UART
static char g_sci8rxbuffer[CONFIG_SCI8_RXBUFSIZE];
static char g_sci8txbuffer[CONFIG_SCI8_TXBUFSIZE];
#  ifdef CONFIG_SCI8_RXDMA
static char g_sci8rxfifo[RXDMA_BUFFER_SIZE];
#  endif
#endif

#ifdef CONFIG_RA6M5_SCI9_UART
static char g_sci9rxbuffer[CONFIG_SCI9_RXBUFSIZE];
static char g_sci9txbuffer[CONFIG_SCI9_TXBUFSIZE];
#  ifdef CONFIG_SCI9_RXDMA
static char g_sci9rxfifo[RXDMA_BUFFER_SIZE];
#  endif
#endif

/* This describes the state of the RA6M5 SCI0 port. */

#ifdef CONFIG_RA6M5_SCI0_UART
static struct ra6m5_serial_s g_sci0priv =
{
  .dev =
    {
#  if CONSOLE_UART == 0
      .isconsole = true,
#  endif
      .recv      =
      {
        .size    = CONFIG_SCI0_RXBUFSIZE,
        .buffer  = g_sci0rxbuffer,
      },
      .xmit      =
      {
        .size    = CONFIG_SCI0_TXBUFSIZE,
        .buffer  = g_sci0txbuffer,
      },
#  ifdef CONFIG_SCI0_RXDMA
      .ops       = &g_sci_dma_ops,
#  else
      .ops       = &g_sci_ops,
#  endif
      .priv      = &g_sci0priv,
    },

  .irqrxi        = RA6M5_IRQ_SCI0_RXI,
  .irqtxi        = RA6M5_IRQ_SCI0_TXI,
  .irqtei        = RA6M5_IRQ_SCI0_TEI,
  .irqeri        = RA6M5_IRQ_SCI0_ERI,
  .parity        = CONFIG_SCI0_PARITY,
  .bits          = CONFIG_SCI0_BITS,
  .stopbits2     = CONFIG_SCI0_2STOP,
  .baud          = CONFIG_SCI0_BAUD,
  .clock         = 0,
  .base          = RA6M5_SCI0_BASE,
  .tx_gpio       = GPIO_SCI0_TX,
  .rx_gpio       = GPIO_SCI0_RX,
#  if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_SCI0_OFLOWCONTROL)
  .oflow         = true,
  .cts_gpio      = GPIO_SCI0_CTS,
#  endif
#  if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_SCI0_IFLOWCONTROL)
  .iflow         = true,
  .rts_gpio      = GPIO_SCI0_RTS,
#  endif
#  ifdef CONFIG_SCI0_RXDMA
  .rxdma_channel = DMAMAP_SCI0_RX,
  .rxfifo        = g_sci0rxfifo,
#  endif

#  ifdef CONFIG_SCI0_RS485
  .rs485_dir_gpio = GPIO_SCI0_RS485_DIR,
#    if (CONFIG_SCI0_RS485_DIR_POLARITY == 0)
  .rs485_dir_polarity = false,
#    else
  .rs485_dir_polarity = true,
#    endif
#  endif
};
#endif

/* This describes the state of the RA6M5 SCI1 port. */

#ifdef CONFIG_RA6M5_SCI1_UART
static struct ra6m5_serial_s g_sci1priv =
{
  .dev =
    {
#  if CONSOLE_UART == 1
      .isconsole = true,
#  endif
      .recv      =
      {
        .size    = CONFIG_SCI1_RXBUFSIZE,
        .buffer  = g_sci1rxbuffer,
      },
      .xmit      =
      {
        .size    = CONFIG_SCI1_TXBUFSIZE,
        .buffer  = g_sci1txbuffer,
      },
#  ifdef CONFIG_SCI1_RXDMA
      .ops       = &g_sci_dma_ops,
#  else
      .ops       = &g_sci_ops,
#  endif
      .priv      = &g_sci1priv,
    },

  .irqrxi        = RA6M5_IRQ_SCI1_RXI,
  .irqtxi        = RA6M5_IRQ_SCI1_TXI,
  .irqtei        = RA6M5_IRQ_SCI1_TEI,
  .irqeri        = RA6M5_IRQ_SCI1_ERI,
  .parity        = CONFIG_SCI1_PARITY,
  .bits          = CONFIG_SCI1_BITS,
  .stopbits2     = CONFIG_SCI1_2STOP,
  .baud          = CONFIG_SCI1_BAUD,
  .clock         = 0,
  .base          = RA6M5_SCI1_BASE,
  .tx_gpio       = GPIO_SCI1_TX,
  .rx_gpio       = GPIO_SCI1_RX,
#  if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_SCI1_OFLOWCONTROL)
  .oflow         = true,
  .cts_gpio      = GPIO_SCI1_CTS,
#  endif
#  if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_SCI1_IFLOWCONTROL)
  .iflow         = true,
  .rts_gpio      = GPIO_SCI1_RTS,
#  endif
#  ifdef CONFIG_SCI1_RXDMA
  .rxdma_channel = DMAMAP_SCI1_RX,
  .rxfifo        = g_sci1rxfifo,
#  endif

#  ifdef CONFIG_SCI1_RS485
  .rs485_dir_gpio = GPIO_SCI1_RS485_DIR,
#    if (CONFIG_SCI1_RS485_DIR_POLARITY == 0)
  .rs485_dir_polarity = false,
#    else
  .rs485_dir_polarity = true,
#    endif
#  endif
};
#endif

/* This describes the state of the RA6M5 SCI2 port. */

#ifdef CONFIG_RA6M5_SCI2_UART
static struct ra6m5_serial_s g_sci2priv =
{
  .dev =
    {
#  if CONSOLE_UART == 2
      .isconsole = true,
#  endif
      .recv      =
      {
        .size    = CONFIG_SCI2_RXBUFSIZE,
        .buffer  = g_sci2rxbuffer,
      },
      .xmit      =
      {
        .size    = CONFIG_SCI2_TXBUFSIZE,
        .buffer  = g_sci2txbuffer,
      },
#  ifdef CONFIG_SCI2_RXDMA
      .ops       = &g_sci_dma_ops,
#  else
      .ops       = &g_sci_ops,
#  endif
      .priv      = &g_sci2priv,
    },

  .irqrxi        = RA6M5_IRQ_SCI2_RXI,
  .irqtxi        = RA6M5_IRQ_SCI2_TXI,
  .irqtei        = RA6M5_IRQ_SCI2_TEI,
  .irqeri        = RA6M5_IRQ_SCI2_ERI,
  .parity        = CONFIG_SCI2_PARITY,
  .bits          = CONFIG_SCI2_BITS,
  .stopbits2     = CONFIG_SCI2_2STOP,
  .baud          = CONFIG_SCI2_BAUD,
  .clock         = 0,
  .base          = RA6M5_SCI2_BASE,
  .tx_gpio       = GPIO_SCI2_TX,
  .rx_gpio       = GPIO_SCI2_RX,
#  if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_SCI2_OFLOWCONTROL)
  .oflow         = true,
  .cts_gpio      = GPIO_SCI2_CTS,
#  endif
#  if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_SCI2_IFLOWCONTROL)
  .iflow         = true,
  .rts_gpio      = GPIO_SCI2_RTS,
#  endif
#  ifdef CONFIG_SCI2_RXDMA
  .rxdma_channel = DMAMAP_SCI2_RX,
  .rxfifo        = g_sci2rxfifo,
#  endif

#  ifdef CONFIG_SCI2_RS485
  .rs485_dir_gpio = GPIO_SCI2_RS485_DIR,
#    if (CONFIG_SCI2_RS485_DIR_POLARITY == 0)
  .rs485_dir_polarity = false,
#    else
  .rs485_dir_polarity = true,
#    endif
#  endif
};
#endif

/* This describes the state of the RA6M5 SCI3 port. */

#ifdef CONFIG_RA6M5_SCI3_UART
static struct ra6m5_serial_s g_sci3priv =
{
  .dev =
    {
#  if CONSOLE_UART == 3
      .isconsole = true,
#  endif
      .recv      =
      {
        .size    = CONFIG_SCI3_RXBUFSIZE,
        .buffer  = g_sci3rxbuffer,
      },
      .xmit      =
      {
        .size    = CONFIG_SCI3_TXBUFSIZE,
        .buffer  = g_sci3txbuffer,
      },
#  ifdef CONFIG_SCI3_RXDMA
      .ops       = &g_sci_dma_ops,
#  else
      .ops       = &g_sci_ops,
#  endif
      .priv      = &g_sci3priv,
    },

  .irqrxi        = RA6M5_IRQ_SCI3_RXI,
  .irqtxi        = RA6M5_IRQ_SCI3_TXI,
  .irqtei        = RA6M5_IRQ_SCI3_TEI,
  .irqeri        = RA6M5_IRQ_SCI3_ERI,
  .parity        = CONFIG_SCI3_PARITY,
  .bits          = CONFIG_SCI3_BITS,
  .stopbits2     = CONFIG_SCI3_2STOP,
  .baud          = CONFIG_SCI3_BAUD,
  .clock         = 0,
  .base          = RA6M5_SCI3_BASE,
  .tx_gpio       = GPIO_SCI3_TX,
  .rx_gpio       = GPIO_SCI3_RX,
#  if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_SCI3_OFLOWCONTROL)
  .oflow         = true,
  .cts_gpio      = GPIO_SCI3_CTS,
#  endif
#  if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_SCI3_IFLOWCONTROL)
  .iflow         = true,
  .rts_gpio      = GPIO_SCI3_RTS,
#  endif
#  ifdef CONFIG_SCI3_RXDMA
  .rxdma_channel = DMAMAP_SCI3_RX,
  .rxfifo        = g_sci3rxfifo,
#  endif

#  ifdef CONFIG_SCI3_RS485
  .rs485_dir_gpio = GPIO_SCI3_RS485_DIR,
#    if (CONFIG_SCI3_RS485_DIR_POLARITY == 0)
  .rs485_dir_polarity = false,
#    else
  .rs485_dir_polarity = true,
#    endif
#  endif
};
#endif

/* This describes the state of the RA6M5 SCI4 port. */

#ifdef CONFIG_RA6M5_SCI4_UART
static struct ra6m5_serial_s g_sci4priv =
{
  .dev =
    {
#  if CONSOLE_UART == 4
      .isconsole = true,
#  endif
      .recv      =
      {
        .size    = CONFIG_SCI4_RXBUFSIZE,
        .buffer  = g_sci4rxbuffer,
      },
      .xmit      =
      {
        .size    = CONFIG_SCI4_TXBUFSIZE,
        .buffer  = g_sci4txbuffer,
      },
#  ifdef CONFIG_SCI4_RXDMA
      .ops       = &g_sci_dma_ops,
#  else
      .ops       = &g_sci_ops,
#  endif
      .priv      = &g_sci4priv,
    },

  .irqrxi        = RA6M5_IRQ_SCI4_RXI,
  .irqtxi        = RA6M5_IRQ_SCI4_TXI,
  .irqtei        = RA6M5_IRQ_SCI4_TEI,
  .irqeri        = RA6M5_IRQ_SCI4_ERI,
  .parity        = CONFIG_SCI4_PARITY,
  .bits          = CONFIG_SCI4_BITS,
  .stopbits2     = CONFIG_SCI4_2STOP,
  .baud          = CONFIG_SCI4_BAUD,
  .clock         = 0,
  .base          = RA6M5_SCI4_BASE,
  .tx_gpio       = GPIO_SCI4_TX,
  .rx_gpio       = GPIO_SCI4_RX,
#  if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_SCI4_OFLOWCONTROL)
  .oflow         = true,
  .cts_gpio      = GPIO_SCI4_CTS,
#  endif
#  if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_SCI4_IFLOWCONTROL)
  .iflow         = true,
  .rts_gpio      = GPIO_SCI4_RTS,
#  endif
#  ifdef CONFIG_SCI4_RXDMA
  .rxdma_channel = DMAMAP_SCI4_RX,
  .rxfifo        = g_sci4rxfifo,
#  endif

#  ifdef CONFIG_SCI4_RS485
  .rs485_dir_gpio = GPIO_SCI4_RS485_DIR,
#    if (CONFIG_SCI4_RS485_DIR_POLARITY == 0)
  .rs485_dir_polarity = false,
#    else
  .rs485_dir_polarity = true,
#    endif
#  endif
};
#endif

/* This describes the state of the RA6M5 SCI5 port. */

#ifdef CONFIG_RA6M5_SCI5_UART
static struct ra6m5_serial_s g_sci5priv =
{
  .dev =
    {
#  if CONSOLE_UART == 5
      .isconsole = true,
#  endif
      .recv      =
      {
        .size    = CONFIG_SCI5_RXBUFSIZE,
        .buffer  = g_sci5rxbuffer,
      },
      .xmit      =
      {
        .size    = CONFIG_SCI5_TXBUFSIZE,
        .buffer  = g_sci5txbuffer,
      },
#  ifdef CONFIG_SCI5_RXDMA
      .ops       = &g_sci_dma_ops,
#  else
      .ops       = &g_sci_ops,
#  endif
      .priv      = &g_sci5priv,
    },

  .irqrxi        = RA6M5_IRQ_SCI5_RXI,
  .irqtxi        = RA6M5_IRQ_SCI5_TXI,
  .irqtei        = RA6M5_IRQ_SCI5_TEI,
  .irqeri        = RA6M5_IRQ_SCI5_ERI,
  .parity        = CONFIG_SCI5_PARITY,
  .bits          = CONFIG_SCI5_BITS,
  .stopbits2     = CONFIG_SCI5_2STOP,
  .baud          = CONFIG_SCI5_BAUD,
  .clock         = 0,
  .base          = RA6M5_SCI5_BASE,
  .tx_gpio       = GPIO_SCI5_TX,
  .rx_gpio       = GPIO_SCI5_RX,
#  if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_SCI5_OFLOWCONTROL)
  .oflow         = true,
  .cts_gpio      = GPIO_SCI5_CTS,
#  endif
#  if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_SCI5_IFLOWCONTROL)
  .iflow         = true,
  .rts_gpio      = GPIO_SCI5_RTS,
#  endif
#  ifdef CONFIG_SCI5_RXDMA
  .rxdma_channel = DMAMAP_SCI5_RX,
  .rxfifo        = g_sci5rxfifo,
#  endif

#  ifdef CONFIG_SCI5_RS485
  .rs485_dir_gpio = GPIO_SCI5_RS485_DIR,
#    if (CONFIG_SCI5_RS485_DIR_POLARITY == 0)
  .rs485_dir_polarity = false,
#    else
  .rs485_dir_polarity = true,
#    endif
#  endif
};
#endif

/* This describes the state of the RA6M5 SCI6 port. */

#ifdef CONFIG_RA6M5_SCI6_UART
static struct ra6m5_serial_s g_sci6priv =
{
  .dev =
    {
#  if CONSOLE_UART == 6
      .isconsole = true,
#  endif
      .recv      =
      {
        .size    = CONFIG_SCI6_RXBUFSIZE,
        .buffer  = g_sci6rxbuffer,
      },
      .xmit      =
      {
        .size    = CONFIG_SCI6_TXBUFSIZE,
        .buffer  = g_sci6txbuffer,
      },
#  ifdef CONFIG_SCI6_RXDMA
      .ops       = &g_sci_dma_ops,
#  else
      .ops       = &g_sci_ops,
#  endif
      .priv      = &g_sci6priv,
    },

  .irqrxi        = RA6M5_IRQ_SCI6_RXI,
  .irqtxi        = RA6M5_IRQ_SCI6_TXI,
  .irqtei        = RA6M5_IRQ_SCI6_TEI,
  .irqeri        = RA6M5_IRQ_SCI6_ERI,
  .parity        = CONFIG_SCI6_PARITY,
  .bits          = CONFIG_SCI6_BITS,
  .stopbits2     = CONFIG_SCI6_2STOP,
  .baud          = CONFIG_SCI6_BAUD,
  .clock         = 0,
  .base          = RA6M5_SCI6_BASE,
  .tx_gpio       = GPIO_SCI6_TX,
  .rx_gpio       = GPIO_SCI6_RX,
#  if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_SCI6_OFLOWCONTROL)
  .oflow         = true,
  .cts_gpio      = GPIO_SCI6_CTS,
#  endif
#  if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_SCI6_IFLOWCONTROL)
  .iflow         = true,
  .rts_gpio      = GPIO_SCI6_RTS,
#  endif
#  ifdef CONFIG_SCI6_RXDMA
  .rxdma_channel = DMAMAP_SCI6_RX,
  .rxfifo        = g_sci6rxfifo,
#  endif

#  ifdef CONFIG_SCI6_RS485
  .rs485_dir_gpio = GPIO_SCI6_RS485_DIR,
#    if (CONFIG_SCI6_RS485_DIR_POLARITY == 0)
  .rs485_dir_polarity = false,
#    else
  .rs485_dir_polarity = true,
#    endif
#  endif
};
#endif

/* This describes the state of the RA6M5 SCI7 port. */

#ifdef CONFIG_RA6M5_SCI7_UART
static struct ra6m5_serial_s g_sci7priv =
{
  .dev =
    {
#  if CONSOLE_UART == 7
      .isconsole = true,
#  endif
      .recv      =
      {
        .size    = CONFIG_SCI7_RXBUFSIZE,
        .buffer  = g_sci7rxbuffer,
      },
      .xmit      =
      {
        .size    = CONFIG_SCI7_TXBUFSIZE,
        .buffer  = g_sci7txbuffer,
      },
#  ifdef CONFIG_SCI7_RXDMA
      .ops       = &g_sci_dma_ops,
#  else
      .ops       = &g_sci_ops,
#  endif
      .priv      = &g_sci7priv,
    },

  .irqrxi        = RA6M5_IRQ_SCI7_RXI,
  .irqtxi        = RA6M5_IRQ_SCI7_TXI,
  .irqtei        = RA6M5_IRQ_SCI7_TEI,
  .irqeri        = RA6M5_IRQ_SCI7_ERI,
  .parity        = CONFIG_SCI7_PARITY,
  .bits          = CONFIG_SCI7_BITS,
  .stopbits2     = CONFIG_SCI7_2STOP,
  .baud          = CONFIG_SCI7_BAUD,
  .clock         = 0,
  .base          = RA6M5_SCI7_BASE,
  .tx_gpio       = GPIO_SCI7_TX,
  .rx_gpio       = GPIO_SCI7_RX,
#  if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_SCI7_OFLOWCONTROL)
  .oflow         = true,
  .cts_gpio      = GPIO_SCI7_CTS,
#  endif
#  if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_SCI7_IFLOWCONTROL)
  .iflow         = true,
  .rts_gpio      = GPIO_SCI7_RTS,
#  endif
#  ifdef CONFIG_SCI7_RXDMA
  .rxdma_channel = DMAMAP_SCI7_RX,
  .rxfifo        = g_sci7rxfifo,
#  endif

#  ifdef CONFIG_SCI7_RS485
  .rs485_dir_gpio = GPIO_SCI7_RS485_DIR,
#    if (CONFIG_SCI7_RS485_DIR_POLARITY == 0)
  .rs485_dir_polarity = false,
#    else
  .rs485_dir_polarity = true,
#    endif
#  endif
};
#endif

/* This describes the state of the RA6M5 SCI8 port. */

#ifdef CONFIG_RA6M5_SCI8_UART

#if defined(CONFIG_SCI8_TXDTC)
dtc_static_transfer_data_cfg_t g_sci8_txcfg =
{
#if CONFIG_RA6M5_SCI8_BUF_SIZE > 1
  .transfer_mode          = DTC_TRANSFER_MODE_BLOCK,
#else
  .transfer_mode          = DTC_TRANSFER_MODE_NORMAL,
#endif
  .data_size              = DTC_DATA_SIZE_BYTE,
  .src_addr_mode          = DTC_SRC_ADDR_INCR,
  .chain_transfer_enable  = DTC_CHAIN_TRANSFER_DISABLE,
  .chain_transfer_mode    = DTC_CHAIN_TRANSFER_CONTINUOUSLY,
  .response_interrupt     = DTC_INTERRUPT_AFTER_ALL_COMPLETE,
  .repeat_block_side      = DTC_REPEAT_BLOCK_DESTINATION,
  .dest_addr_mode         = DTC_DES_ADDR_FIXED,
  .source_addr            = 0,                          /* This will set dynamically */
  .dest_addr              = 0,                          /* Set data register address */
  .transfer_count         = 0,                          /* This will set dynamically */
#if CONFIG_RA6M5_SCI8_BUF_SIZE > 1
  .block_size             = CONFIG_RA6M5_RSPI_BUF_SIZE, /* Looks like tx fifo size */
#else
  .block_size             = 0,
#endif
};
#endif

static struct ra6m5_serial_s g_sci8priv =
{
  .dev =
    {
#  if CONSOLE_UART == 8
      .isconsole = true,
#  endif
      .recv      =
      {
        .size    = CONFIG_SCI8_RXBUFSIZE,
        .buffer  = g_sci8rxbuffer,
      },
      .xmit      =
      {
        .size    = CONFIG_SCI8_TXBUFSIZE,
        .buffer  = g_sci8txbuffer,
      },
#  ifdef CONFIG_SCI8_RXDMA
      .ops       = &g_sci_dma_ops,
#  else
      .ops       = &g_sci_ops,
#  endif
      .priv      = &g_sci8priv,
    },

  .irqrxi        = RA6M5_IRQ_SCI8_RXI,
  .irqtxi        = RA6M5_IRQ_SCI8_TXI,
  .irqtei        = RA6M5_IRQ_SCI8_TEI,
  .irqeri        = RA6M5_IRQ_SCI8_ERI,
  .parity        = CONFIG_SCI8_PARITY,
  .bits          = CONFIG_SCI8_BITS,
  .stopbits2     = CONFIG_SCI8_2STOP,
  .baud          = CONFIG_SCI8_BAUD,
  .clock         = 0,
  .base          = RA6M5_SCI8_BASE,
  .tx_gpio       = GPIO_SCI8_TX,
  .rx_gpio       = GPIO_SCI8_RX,
#  if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_SCI8_OFLOWCONTROL)
  .oflow         = true,
  .cts_gpio      = GPIO_SCI8_CTS,
#  endif
#  if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_SCI8_IFLOWCONTROL)
  .iflow         = true,
  .rts_gpio      = GPIO_SCI8_RTS,
#  endif
#  ifdef CONFIG_SCI8_RXDMA
  .rxdma_channel = DMAMAP_SCI8_RX,
  .rxfifo        = g_sci8rxfifo,
#  endif

#  ifdef CONFIG_SCI8_RS485
  .rs485_dir_gpio = GPIO_SCI8_RS485_DIR,
#    if (CONFIG_SCI8_RS485_DIR_POLARITY == 0)
  .rs485_dir_polarity = false,
#    else
  .rs485_dir_polarity = true,
#    endif
#  endif
};
#endif

/* This describes the state of the RA6M5 SCI9 port. */

#ifdef CONFIG_RA6M5_SCI9_UART
static struct ra6m5_serial_s g_sci9priv =
{
  .dev =
    {
#  if CONSOLE_UART == 9
      .isconsole = true,
#  endif
      .recv      =
      {
        .size    = CONFIG_SCI9_RXBUFSIZE,
        .buffer  = g_sci9rxbuffer,
      },
      .xmit      =
      {
        .size    = CONFIG_SCI9_TXBUFSIZE,
        .buffer  = g_sci9txbuffer,
      },
#  ifdef CONFIG_SCI9_RXDMA
      .ops       = &g_sci_dma_ops,
#  else
      .ops       = &g_sci_ops,
#  endif
      .priv      = &g_sci9priv,
    },

  .irqrxi        = RA6M5_IRQ_SCI9_RXI,
  .irqtxi        = RA6M5_IRQ_SCI9_TXI,
  .irqtei        = RA6M5_IRQ_SCI9_TEI,
  .irqeri        = RA6M5_IRQ_SCI9_ERI,
  .parity        = CONFIG_SCI9_PARITY,
  .bits          = CONFIG_SCI9_BITS,
  .stopbits2     = CONFIG_SCI9_2STOP,
  .baud          = CONFIG_SCI9_BAUD,
  .clock         = 0,
  .base          = RA6M5_SCI9_BASE,
  .tx_gpio       = GPIO_SCI9_TX,
  .rx_gpio       = GPIO_SCI9_RX,
#  if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_SCI9_OFLOWCONTROL)
  .oflow         = true,
  .cts_gpio      = GPIO_SCI9_CTS,
#  endif
#  if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_SCI9_IFLOWCONTROL)
  .iflow         = true,
  .rts_gpio      = GPIO_SCI9_RTS,
#  endif
#  ifdef CONFIG_SCI9_RXDMA
  .rxdma_channel = DMAMAP_SCI9_RX,
  .rxfifo        = g_sci9rxfifo,
#  endif

#  ifdef CONFIG_SCI9_RS485
  .rs485_dir_gpio = GPIO_SCI9_RS485_DIR,
#    if (CONFIG_SCI9_RS485_DIR_POLARITY == 0)
  .rs485_dir_polarity = false,
#    else
  .rs485_dir_polarity = true,
#    endif
#  endif
};
#endif

/* This table lets us iterate over the configured SCIs */

static struct ra6m5_serial_s * const
  g_uart_devs[RA6M5_NSCI] =
{
#ifdef CONFIG_RA6M5_SCI0_UART
  [0] = &g_sci0priv,
#endif
#ifdef CONFIG_RA6M5_SCI1_UART
  [1] = &g_sci1priv,
#endif
#ifdef CONFIG_RA6M5_SCI2_UART
  [2] = &g_sci2priv,
#endif
#ifdef CONFIG_RA6M5_SCI3_UART
  [3] = &g_sci3priv,
#endif
#ifdef CONFIG_RA6M5_SCI4_UART
  [4] = &g_sci4priv,
#endif
#ifdef CONFIG_RA6M5_SCI5_UART
  [5] = &g_sci5priv,
#endif
#ifdef CONFIG_RA6M5_SCI6_UART
  [6] = &g_sci6priv,
#endif
#ifdef CONFIG_RA6M5_SCI7_UART
  [7] = &g_sci7priv,
#endif
#ifdef CONFIG_RA6M5_SCI8_UART
  [8] = &g_sci8priv,
#endif
#ifdef CONFIG_RA6M5_SCI9_UART
  [9] = &g_sci9priv,
#endif
};

#ifdef CONFIG_PM
struct serialpm_s
{
  struct pm_callback_s pm_cb;
  bool serial_suspended;
};

static struct serialpm_s g_serialpm =
{
  .pm_cb.notify  = ra6m5serial_pmnotify,
  .pm_cb.prepare = ra6m5serial_pmprepare,
  .serial_suspended = false
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra6m5serial_getreg8
 ****************************************************************************/

static inline 
uint8_t ra6m5serial_getreg8(struct ra6m5_serial_s *priv, int offset) {
  return getreg8(priv->base + offset);
}

static inline 
uint16_t ra6m5serial_getreg16(struct ra6m5_serial_s *priv, int offset) {
  return getreg16(priv->base + offset);
}

static inline 
uint32_t ra6m5serial_getreg32(struct ra6m5_serial_s *priv, int offset) {
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: ra6m5serial_putreg8
 ****************************************************************************/

static inline
void ra6m5serial_putreg8(struct ra6m5_serial_s *priv,
                         int offset, uint8_t value)
{
  putreg8(value, priv->base + offset);
}

static inline
void ra6m5serial_putreg16(struct ra6m5_serial_s *priv,
                          int offset, uint16_t value)
{
  putreg16(value, priv->base + offset);
}

static inline
void ra6m5serial_putreg32(struct ra6m5_serial_s *priv,
                          int offset, uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: ra6m5serial_setusartint
 ****************************************************************************/

static inline
void ra6m5serial_setusartint(struct ra6m5_serial_s *priv,
                             uint8_t ie)
{
  uint8_t regval;

  /* Save the interrupt mask */

  priv->ie = ie;

  /* And restore the interrupt state (see the interrupt enable/usage table
   * above)
   */

  regval = ra6m5serial_getreg8(priv, RA6M5_SCI_SCR_OFFSET);
  regval &= ~(SCI_SCR_USED_INTS);
  regval |= (ie & (SCI_SCR_USED_INTS));
  ra6m5serial_putreg8(priv, RA6M5_SCI_SCR_OFFSET, regval);
}

/****************************************************************************
 * Name: ra6m5serial_restoreusartint
 ****************************************************************************/

static void ra6m5serial_restoreusartint(struct ra6m5_serial_s *priv,
                                        uint8_t ie)
{
  irqstate_t flags;

  flags = enter_critical_section();

  ra6m5serial_setusartint(priv, ie);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: ra6m5serial_disableusartint
 ****************************************************************************/

static void ra6m5serial_disableusartint(struct ra6m5_serial_s *priv,
                                        uint8_t *ie)
{
  irqstate_t flags;

  flags = enter_critical_section();

  if (ie)
    {
      uint8_t scr;

      /* SCI interrupts:
       *
       * Enable           Status          Meaning
       * ---------------- -------------- ----------------------
       * SCI_SCR_RIE      SCI_SSR_RDRF    Received Data Full
       * SCI_SCR_TIE      SCI_SSR_TDRE    Transmit Data Empty
       * SCI_SCR_TEIE     SCI_SSR_TEND    Transmission End  (only RS-485)
       */

      scr = ra6m5serial_getreg8(priv, RA6M5_SCI_SCR_OFFSET);

      /* Return the current interrupt mask value for the used interrupts.
       * Notice that this depends on the fact that none of the used interrupt
       * enable bits overlap.  This logic would fail if we needed the break
       * interrupt!
       */

      *ie = scr & (SCI_SCR_USED_INTS);
    }

  /* Disable all interrupts */

  ra6m5serial_setusartint(priv, 0);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: ra6m5serial_dmanextrx
 *
 * Description:
 *   Returns the index into the RX FIFO where the DMA will place the next
 *   byte that it receives.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_DMA
static int ra6m5serial_dmanextrx(struct ra6m5_serial_s *priv)
{
  size_t dmaresidual;

  dmaresidual = ra6m5_dmaresidual(priv->rxdma);

  return (RXDMA_BUFFER_SIZE - (int)dmaresidual);
}
#endif

/****************************************************************************
 * Name: ra6m5serial_setformat
 *
 * Description:
 *   Set the serial line format and speed.
 *
 ****************************************************************************/

#ifndef CONFIG_SUPPRESS_UART_CONFIG
static void ra6m5serial_setformat(struct uart_dev_s *dev)
{
  struct ra6m5_serial_s *priv =
    (struct ra6m5_serial_s *)dev->priv;
  uint8_t regval;

  /* Find the best BRR (bit rate register) value.
    *  In table g_async_baud, divisor values are stored for BGDM, ABCS, ABCSE and N values.  Each set of divisors
    *  is tried, and the settings with the lowest bit rate error are stored. The formula to calculate BRR is as
    *  follows and it must be 255 or less:
    *  BRR = (PCLK / (div_coefficient * baud)) - 1
    */
  static const int32_t  SCI_UART_100_PERCENT_X_1000 = 100000;
  static const int32_t  SCI_UART_MDDR_DIVISOR       = 256;
  static const uint32_t baud_rate_error_x_1000      = 5000;
  static const bool     bitrate_modulation          = false;

  int32_t  hit_bit_err = SCI_UART_100_PERCENT_X_1000;
  uint32_t hit_mddr    = 0U;
  uint32_t divisor     = 0U;
  uint32_t freq_hz     = priv->clock;

  uint8_t reg_brr  = SCI_UART_BRR_MAX;
  uint8_t reg_mddr = SCI_UART_MDDR_MIN;
  uint8_t reg_semr = ra6m5serial_getreg8(priv, RA6M5_SCI_SEMR_OFFSET) & 0xA3;
  uint8_t reg_scr  = ra6m5serial_getreg8(priv, RA6M5_SCI_SCR_OFFSET)  & 0x03;

  for (uint32_t select_16_base_clk_cycles = 0U;
        select_16_base_clk_cycles <= 1U && (hit_bit_err > ((int32_t) baud_rate_error_x_1000));
        select_16_base_clk_cycles++)
  {
      for (uint32_t i = 0U; i < SCI_UART_NUM_DIVISORS_ASYNC; i++)
      {
          /* if select_16_base_clk_cycles == true:  Skip this calculation for divisors that are not acheivable with 16 base clk cycles per bit.
            *  if select_16_base_clk_cycles == false: Skip this calculation for divisors that are only acheivable without 16 base clk cycles per bit.
            */
          if (((uint8_t) select_16_base_clk_cycles) ^ (g_async_baud[i].abcs | g_async_baud[i].abcse))
          {
              continue;
          }

          divisor = (uint32_t) g_div_coefficient[i] * priv->baud;
          uint32_t temp_brr = freq_hz / divisor;

          if (temp_brr <= (SCI_UART_BRR_MAX + 1U))
          {
              while (temp_brr > 0U)
              {
                  temp_brr -= 1U;

                  /* Calculate the bit rate error. The formula is as follows:
                    *  bit rate error[%] = {(PCLK / (baud * div_coefficient * (BRR + 1)) - 1} x 100
                    *  calculates bit rate error[%] to three decimal places
                    */
                  int32_t err_divisor = (int32_t) (divisor * (temp_brr + 1U));

                  /* Promoting to 64 bits for calculation, but the final value can never be more than 32 bits, as
                    * described below, so this cast is safe.
                    *    1. (temp_brr + 1) can be off by an upper limit of 1 due to rounding from the calculation:
                    *       freq_hz / divisor, or:
                    *       freq_hz / divisor <= (temp_brr + 1) < (freq_hz / divisor) + 1
                    *    2. Solving for err_divisor:
                    *       freq_hz <= err_divisor < freq_hz + divisor
                    *    3. Solving for bit_err:
                    *       0 >= bit_err >= (freq_hz * 100000 / (freq_hz + divisor)) - 100000
                    *    4. freq_hz >= divisor (or temp_brr would be -1 and we would never enter this while loop), so:
                    *       0 >= bit_err >= 100000 / freq_hz - 100000
                    *    5. Larger frequencies yield larger bit errors (absolute value).  As the frequency grows,
                    *       the bit_err approaches -100000, so:
                    *       0 >= bit_err >= -100000
                    *    6. bit_err is between -100000 and 0.  This entire range fits in an int32_t type, so the cast
                    *       to (int32_t) is safe.
                    */
                  int32_t bit_err = (int32_t) (((((int64_t) freq_hz) * SCI_UART_100_PERCENT_X_1000) /
                                                err_divisor) - SCI_UART_100_PERCENT_X_1000);

                  reg_mddr = 0U;
                  if (bitrate_modulation)
                  {
                      /* Calculate the MDDR (M) value if bit rate modulation is enabled,
                        * The formula to calculate MBBR (from the M and N relationship given in the hardware manual) is as follows
                        * and it must be between 128 and 256.
                        * MDDR = ((div_coefficient * baud * 256) * (BRR + 1)) / PCLK */
                      reg_mddr = (uint32_t) err_divisor / (freq_hz / 128);

                      /* The maximum value that could result from the calculation above is 256, which is a valid MDDR
                        * value, so only the lower bound is checked. */
                      if (reg_mddr < SCI_UART_MDDR_MIN)
                      {
                          break;
                      }

                      /* Adjust bit rate error for bit rate modulation. The following formula is used:
                        *  bit rate error [%] = ((bit rate error [%, no modulation] + 100) * MDDR / 256) - 100
                        */
                      bit_err = (((bit_err + SCI_UART_100_PERCENT_X_1000) * (int32_t) reg_mddr) /
                                  SCI_UART_MDDR_DIVISOR) - SCI_UART_100_PERCENT_X_1000;
                  }

                  /* Take the absolute value of the bit rate error. */
                  if (bit_err < 0)
                  {
                      bit_err = -bit_err;
                  }

                  /* If the absolute value of the bit rate error is less than the previous lowest absolute value of
                    *  bit rate error, then store these settings as the best value.
                    */
                  if (bit_err < hit_bit_err)
                  {
                      reg_semr |= g_async_baud[i].bgdm  << 6;
                      reg_semr |= g_async_baud[i].abcs  << 4;
                      reg_semr |= g_async_baud[i].abcse << 3;
                      reg_scr  |= g_async_baud[i].cks;
                      reg_brr   = (uint8_t) temp_brr;
                      hit_bit_err = bit_err;
                      hit_mddr    = reg_mddr;
                  }

                  if (bitrate_modulation)
                  {
                      reg_semr |= 1 << 2;
                      reg_mddr = (uint8_t) hit_mddr;
                  }
                  else
                  {
                      break;
                  }
              }
          }
      }
  }

  ra6m5serial_putreg8(priv, RA6M5_SCI_MDDR_OFFSET, reg_mddr);
  ra6m5serial_putreg8(priv, RA6M5_SCI_SEMR_OFFSET, reg_semr);
  ra6m5serial_putreg8(priv, RA6M5_SCI_SCR_OFFSET,  reg_scr);
  ra6m5serial_putreg8(priv, RA6M5_SCI_BRR_OFFSET,  reg_brr);

  /* Configure parity mode */

  regval  = ra6m5serial_getreg8(priv, RA6M5_SCI_SMR_OFFSET);
  regval &= ~(SCI_SMR_PM | SCI_SMR_PE | SCI_SMR_STOP | SCI_SMR_CHR);

  if (priv->parity == 1)       /* Odd parity */
    {
      regval |= (SCI_SMR_PE | SCI_SMR_PM);
    }
  else if (priv->parity == 2)  /* Even parity */
    {
      regval |= SCI_SMR_PE;
    }

  /* Configure word length (parity uses one of configured bits)
   *
   * Default: 1 start, 8 data (no parity), n stop, OR
   *          1 start, 7 data + parity, n stop
   */

  if (priv->bits == 9 || (priv->bits == 8 && priv->parity != 0))
    {
      /* Select: 1 start, 8 data + parity, n stop, OR
       *         1 start, 9 data (no parity), n stop.
       */

      regval |= SCI_SMR_CHR;
    }
  else if (priv->bits == 7 && priv->parity == 0)
    {
      /* Select: 1 start, 7 data (no parity), n stop, OR
       */
      uint32_t scmr;

      scmr  = ra6m5serial_getreg8(priv, RA6M5_SCI_SCMR_OFFSET);
      scmr |= SCI_SCMR_CHR1;
      ra6m5serial_putreg8(priv, RA6M5_SCI_SCMR_OFFSET, scmr);
    }

  /* Configure STOP bits */

  if (priv->stopbits2)
    {
      regval |= SCI_SMR_STOP;
    }

  ra6m5serial_putreg8(priv, RA6M5_SCI_SMR_OFFSET, regval);

  /* Configure hardware flow control */

  regval  = ra6m5serial_getreg8(priv, RA6M5_SCI_SPMR_OFFSET);
  regval &= ~(SCI_SPMR_CTSE | SCI_SPMR_CSTPEN);

#if defined(CONFIG_SERIAL_IFLOWCONTROL) && !defined(CONFIG_RA6M5_FLOWCONTROL_BROKEN)
  if (priv->iflow && (priv->rts_gpio != 0))
    {

    }
#endif

#ifdef CONFIG_SERIAL_OFLOWCONTROL
  if (priv->oflow && (priv->cts_gpio != 0))
    {
      regval |= SCI_SPMR_CTSE;
    }
#endif

  ra6m5serial_putreg8(priv, RA6M5_SCI_SPMR_OFFSET, regval);
}
#endif /* CONFIG_SUPPRESS_UART_CONFIG */

/****************************************************************************
 * Name: ra6m5serial_setsuspend
 *
 * Description:
 *   Suspend or resume serial peripheral.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static void ra6m5serial_setsuspend(struct uart_dev_s *dev, bool suspend)
{
  struct ra6m5_serial_s *priv = (struct ra6m5_serial_s *)dev->priv;
#ifdef SERIAL_HAVE_DMA
  bool dmarestored = false;
#endif

  if (priv->suspended == suspend)
    {
      return;
    }

  priv->suspended = suspend;

  if (suspend)
    {
#ifdef CONFIG_SERIAL_IFLOWCONTROL
      if (priv->iflow)
        {
          /* Force RTS high to prevent further Rx. */

          ra6m5_configgpio((priv->rts_gpio & ~GPIO_MODE_MASK)
                             | (GPIO_OUTPUT | GPIO_OUTPUT_SET));
        }
#endif

      /* Disable interrupts to prevent Tx. */

      ra6m5serial_disableusartint(priv, &priv->suspended_ie);

      /* Wait last Tx to complete. */

      while ((ra6m5serial_getreg8(priv, RA6M5_SCI_SSR_OFFSET) &
              SCI_SSR_TEND) == 0);

#ifdef SERIAL_HAVE_DMA
      if (priv->dev.ops == &g_sci_dma_ops && !priv->rxdmasusp)
        {
#ifdef CONFIG_SERIAL_IFLOWCONTROL
          if (priv->iflow && priv->rxdmanext == RXDMA_BUFFER_SIZE)
            {
              /* Rx DMA in non-circular iflow mode and already stopped
               * at end of DMA buffer. No need to suspend.
               */
            }
          else
#endif
            {
              /* Suspend Rx DMA. */

              ra6m5_dmastop(priv->rxdma);
              priv->rxdmasusp = true;
            }
        }
#endif
    }
  else
    {
#ifdef SERIAL_HAVE_DMA
      if (priv->dev.ops == &g_sci_dma_ops && priv->rxdmasusp)
        {
#ifdef CONFIG_SERIAL_IFLOWCONTROL
          if (priv->iflow)
            {
              ra6m5serial_dmaiflowrestart(priv);
            }
          else
#endif
            {
              /* This SCI does not have HW flow-control. Unconditionally
               * re-enable DMA (might loss unprocessed bytes received
               * to DMA buffer before suspending).
               */

              ra6m5serial_dmareenable(priv);
              priv->rxdmasusp = false;
            }

          dmarestored = true;
        }
#endif

      /* Re-enable interrupts to resume Tx. */

      ra6m5serial_restoreusartint(priv, priv->suspended_ie);

#ifdef CONFIG_SERIAL_IFLOWCONTROL
      if (priv->iflow)
        {
          /* Restore peripheral RTS control. */

          ra6m5_configgpio(priv->rts_gpio);
        }
#endif
    }

#ifdef SERIAL_HAVE_DMA
  if (dmarestored)
    {
      irqstate_t flags;

      flags = enter_critical_section();

      /* Perform initial Rx DMA buffer fetch to wake-up serial device
       * activity.
       */

      if (priv->rxdma != NULL)
        {
          ra6m5serial_dmarxcallback(priv->rxdma, 0, priv);
        }

      leave_critical_section(flags);
    }
#endif
}
#endif

/****************************************************************************
 * Name: ra6m5serial_pm_setsuspend
 *
 * Description:
 *   Suspend or resume serial peripherals for/from deep-sleep/stop modes.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static void ra6m5serial_pm_setsuspend(bool suspend)
{
  int n;

  /* Already in desired state? */

  if (suspend == g_serialpm.serial_suspended)
    return;

  g_serialpm.serial_suspended = suspend;

  for (n = 0; n < RA6M5_NLPUART + RA6M5_NSCI + RA6M5_NUART; n++)
    {
      struct ra6m5_serial_s *priv = g_uart_devs[n];

      if (!priv || !priv->initialized)
        {
          continue;
        }

      ra6m5serial_setsuspend(&priv->dev, suspend);
    }
}
#endif

/****************************************************************************
 * Name: ra6m5serial_setclock
 *
 * Description:
 *   Enable or disable clock for the SCI peripheral
 *
 * Input Parameters:
 *   dev - A reference to the UART driver state structure
 *   on  - Enable clock if 'on' is 'true' and disable if 'false'
 *
 ****************************************************************************/

static void ra6m5serial_setclock(struct uart_dev_s *dev, bool on)
{
  struct ra6m5_serial_s *priv =
    (struct ra6m5_serial_s *)dev->priv;
  uint32_t addr = RA6M5_MSTP_REG(RA6M5_MSTP_MSTPCRB_OFFSET);
  uint32_t mask;

  /* Update the peripheral clock frequency */

  priv->clock = g_clock_freq[RA6M5_CLOCKS_SOURCE_PCLKA];

  /* Determine which SCI to configure */

  switch (priv->base)
    {
    default:
      return;
#ifdef CONFIG_RA6M5_SCI0_UART
    case RA6M5_SCI0_BASE:
      mask = RA6M5_SCI_STOP(0);
      break;
#endif
#ifdef CONFIG_RA6M5_SCI1_UART
    case RA6M5_SCI1_BASE:
      mask = RA6M5_SCI_STOP(1);
      break;
#endif
#ifdef CONFIG_RA6M5_SCI2_UART
    case RA6M5_SCI2_BASE:
      mask = RA6M5_SCI_STOP(2);
      break;
#endif
#ifdef CONFIG_RA6M5_SCI3_UART
    case RA6M5_SCI3_BASE:
      mask = RA6M5_SCI_STOP(3);
      break;
#endif
#ifdef CONFIG_RA6M5_SCI4_UART
    case RA6M5_SCI4_BASE:
      mask = RA6M5_SCI_STOP(4);
      break;
#endif
#ifdef CONFIG_RA6M5_SCI5_UART
    case RA6M5_SCI5_BASE:
      mask = RA6M5_SCI_STOP(5);
      break;
#endif
#ifdef CONFIG_RA6M5_SCI8_UART
    case RA6M5_SCI8_BASE:
      mask = RA6M5_SCI_STOP(8);
      break;
#endif
    }

  /* Enable/disable clock for SCI */

  if (on)
    {
      modifyreg32(addr, mask, 0);
    }
  else
    {
      modifyreg32(addr, 0, mask);
    }
}

/****************************************************************************
 * Name: ra6m5serial_setup
 *
 * Description:
 *   Configure the SCI baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int ra6m5serial_setup(struct uart_dev_s *dev)
{
  struct ra6m5_serial_s *priv =
    (struct ra6m5_serial_s *)dev->priv;

#ifndef CONFIG_SUPPRESS_UART_CONFIG
  uint8_t scr;
  uint8_t smr;

  /* Note: The logic here depends on the fact that that the SCI module
   * was enabled in ra6m5_lowsetup().
   */

  /* Enable SCI clock */

  ra6m5serial_setclock(dev, true);

  /* Configure pins for SCI use */

  ra6m5_configgpio(priv->tx_gpio);
  ra6m5_configgpio(priv->rx_gpio);

#ifdef CONFIG_SERIAL_OFLOWCONTROL
  if (priv->cts_gpio != 0)
    {
      ra6m5_configgpio(priv->cts_gpio);
    }
#endif

#ifdef CONFIG_SERIAL_IFLOWCONTROL
  if (priv->rts_gpio != 0)
    {
      uint32_t config = priv->rts_gpio;

#ifdef CONFIG_RA6M5_FLOWCONTROL_BROKEN
      /* Instead of letting hw manage this pin, we will bitbang */

      config = (config & ~GPIO_MODE_MASK) | GPIO_OUTPUT;

#endif
      ra6m5_configgpio(config);
    }
#endif

#ifdef HAVE_RS485
  if (priv->rs485_dir_gpio != 0)
    {
      ra6m5_configgpio(priv->rs485_dir_gpio);
      ra6m5_gpiowrite(priv->rs485_dir_gpio, !priv->rs485_dir_polarity);
    }
#endif

  /* Configure SCR */

  scr  = ra6m5serial_getreg8(priv, RA6M5_SCI_SCR_OFFSET);
  scr &= ~(SCI_SCR_USED_INTS | SCI_SCR_RE | SCI_SCR_TE);
  ra6m5serial_putreg8(priv, RA6M5_SCI_SCR_OFFSET, scr);

  /* Configure SMR */

  smr  = ra6m5serial_getreg8(priv, RA6M5_SCI_SMR_OFFSET);
  smr &= ~(SCI_SMR_MP | SCI_SMR_STOP | SCI_SMR_PM | SCI_SMR_PE | 
           SCI_SMR_CHR | SCI_SMR_CM);

  /* Configure STOP bits */

  if (priv->stopbits2)
    {
      smr |= SCI_SMR_STOP;
    }

  ra6m5serial_putreg8(priv, RA6M5_SCI_SMR_OFFSET, smr);

  /* Configure the SCI line format and speed. */

  ra6m5serial_setformat(dev);

  /* Enable Rx, Tx, and the SCI */

  scr |= (SCI_SCR_RE | SCI_SCR_TE);
  ra6m5serial_putreg8(priv, RA6M5_SCI_SCR_OFFSET, scr);

#endif /* CONFIG_SUPPRESS_UART_CONFIG */

  /* Set up the cached interrupt enables value */

  priv->ie = 0;

  /* Mark device as initialized. */

  priv->initialized = true;

  return OK;
}

/****************************************************************************
 * Name: ra6m5serial_dmasetup
 *
 * Description:
 *   Configure the SCI baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_DMA
static int ra6m5serial_dmasetup(struct uart_dev_s *dev)
{
  struct ra6m5_serial_s *priv =
    (struct ra6m5_serial_s *)dev->priv;
  int result;
  uint32_t regval;

  /* Do the basic UART setup first, unless we are the console */

  if (!dev->isconsole)
    {
      result = ra6m5serial_setup(dev);
      if (result != OK)
        {
          return result;
        }
    }

  /* Acquire the DMA channel.  This should always succeed. */

  priv->rxdma = ra6m5_dmachannel(priv->rxdma_channel);

#ifdef CONFIG_SERIAL_IFLOWCONTROL
  if (priv->iflow)
    {
      /* Configure for non-circular DMA reception into the RX FIFO */

      ra6m5_dmasetup(priv->rxdma,
                     priv->base + RA6M5_SCI_RDR_OFFSET,
                     (uint32_t)priv->rxfifo,
                     RXDMA_BUFFER_SIZE,
                     SERIAL_DMA_IFLOW_CONTROL_WORD);
    }
  else
#endif
    {
      /* Configure for circular DMA reception into the RX FIFO */

      ra6m5_dmasetup(priv->rxdma,
                     priv->base + RA6M5_SCI_RDR_OFFSET,
                     (uint32_t)priv->rxfifo,
                     RXDMA_BUFFER_SIZE,
                     SERIAL_DMA_CONTROL_WORD);
    }

  /* Reset our DMA shadow pointer to match the address just
   * programmed above.
   */

  priv->rxdmanext = 0;

  /* Enable receive DMA for the UART */

  regval  = ra6m5serial_getreg8(priv, RA6M5_SCI_CR3_OFFSET);
  regval |= SCI_CR3_DMAR;
  ra6m5serial_putreg8(priv, RA6M5_SCI_CR3_OFFSET, regval);

#ifdef CONFIG_SERIAL_IFLOWCONTROL
  if (priv->iflow)
    {
      /* Start the DMA channel, and arrange for callbacks at the full point
       * in the FIFO. After buffer gets full, hardware flow-control kicks
       * in and DMA transfer is stopped.
       */

      ra6m5_dmastart(priv->rxdma, ra6m5serial_dmarxcallback,
                       (void *)priv, false);
    }
  else
#endif
    {
      /* Start the DMA channel, and arrange for callbacks at the half and
       * full points in the FIFO.  This ensures that we have half a FIFO
       * worth of time to claim bytes before they are overwritten.
       */

      ra6m5_dmastart(priv->rxdma, ra6m5serial_dmarxcallback,
                       (void *)priv, true);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: ra6m5serial_shutdown
 *
 * Description:
 *   Disable the SCI.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void ra6m5serial_shutdown(struct uart_dev_s *dev)
{
  struct ra6m5_serial_s *priv =
    (struct ra6m5_serial_s *)dev->priv;
  uint8_t regval;

  /* Mark device as uninitialized. */

  priv->initialized = false;

  /* Disable all interrupts */

  ra6m5serial_disableusartint(priv, NULL);

  /* Disable SCI clock */

  ra6m5serial_setclock(dev, false);

  /* Disable Rx, Tx, and the UART */

  regval  = ra6m5serial_getreg8(priv, RA6M5_SCI_SCR_OFFSET);
  regval &= ~(SCI_SCR_RE | SCI_SCR_TE);
  ra6m5serial_putreg8(priv, RA6M5_SCI_SCR_OFFSET, regval);

  /* Release pins. "If the serial-attached device is powered down, the TX
   * pin causes back-powering, potentially confusing the device to the point
   * of complete lock-up."
   *
   * REVISIT:  Is unconfiguring the pins appropriate for all device?  If not,
   * then this may need to be a configuration option.
   */

  ra6m5_unconfiggpio(priv->tx_gpio);
  ra6m5_unconfiggpio(priv->rx_gpio);

#ifdef CONFIG_SERIAL_OFLOWCONTROL
  if (priv->cts_gpio != 0)
    {
      ra6m5_unconfiggpio(priv->cts_gpio);
    }
#endif

#ifdef CONFIG_SERIAL_IFLOWCONTROL
  if (priv->rts_gpio != 0)
    {
      ra6m5_unconfiggpio(priv->rts_gpio);
    }
#endif

#ifdef HAVE_RS485
  if (priv->rs485_dir_gpio != 0)
    {
      ra6m5_unconfiggpio(priv->rs485_dir_gpio);
    }
#endif
}

/****************************************************************************
 * Name: ra6m5serial_dmashutdown
 *
 * Description:
 *   Disable the SCI.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_DMA
static void ra6m5serial_dmashutdown(struct uart_dev_s *dev)
{
  struct ra6m5_serial_s *priv =
    (struct ra6m5_serial_s *)dev->priv;

  /* Perform the normal UART shutdown */

  ra6m5serial_shutdown(dev);

  /* Stop the DMA channel */

  ra6m5_dmastop(priv->rxdma);

  /* Release the DMA channel */

  ra6m5_dmafree(priv->rxdma);
  priv->rxdma = NULL;
}
#endif

/****************************************************************************
 * Name: ra6m5serial_attach
 *
 * Description:
 *   Configure the SCI to operation in interrupt driven mode.  This method
 *   is called when the serial port is opened.  Normally, this is just after
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless
 *   the hardware supports multiple levels of interrupt enabling).  The RX
 *   and TX interrupts are not enabled until the txint() and rxint() methods
 *   are called.
 *
 ****************************************************************************/

static int ra6m5serial_attach(struct uart_dev_s *dev)
{
  struct ra6m5_serial_s *priv =
    (struct ra6m5_serial_s *)dev->priv;
  int ret;

  /* Attach the RDR full IRQ (RXI) that is enabled by the RIE SCR bit */

  ret = irq_attach(priv->irqrxi, ra6m5serial_interrupt, dev);
  if (OK != ret)
    {
      return ret;
    }

  /* Attach the TDR empty IRQ (TXI) enabled by the TIE SCR bit */

  ret = irq_attach(priv->irqtxi, ra6m5serial_interrupt, dev);
  if (OK != ret)
    {
      return ret;
    }

  /* Attach the ERI IRQ */

  ret = irq_attach(priv->irqeri, ra6m5serial_interrupt, dev);
  if (OK != ret)
    {
      return ret;
    }

  /* Attach the TEI IRQ */

  ret = irq_attach(priv->irqtei, ra6m5serial_interrupt, dev);
  if (OK == ret)
    {
#ifdef CONFIG_ARCH_IRQPRIO
      /* All SCI0 interrupts share the same prioritization */

      up_prioritize_irq(priv->irqrxi, 7);  /* Set SCI priority midway */
      up_prioritize_irq(priv->irqtxi, 7);
#endif

      /* Enable the interrupts */

      up_enable_irq(priv->irqrxi);
      up_enable_irq(priv->irqtxi);
      up_enable_irq(priv->irqtei);
      up_enable_irq(priv->irqeri);

      /* Return OK on success */

      return OK;
    }

  irq_detach(priv->irqrxi);
  irq_detach(priv->irqtxi);
  irq_detach(priv->irqeri);

  return ret;
}

/****************************************************************************
 * Name: ra6m5serial_detach
 *
 * Description:
 *   Detach SCI interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void ra6m5serial_detach(struct uart_dev_s *dev)
{
  struct ra6m5_serial_s *priv =
    (struct ra6m5_serial_s *)dev->priv;
  
  up_disable_irq(priv->irqrxi);
  up_disable_irq(priv->irqtxi);
  up_disable_irq(priv->irqtei);
  up_disable_irq(priv->irqeri);

  irq_detach(priv->irqrxi);
  irq_detach(priv->irqtxi);
  irq_detach(priv->irqtei);
  irq_detach(priv->irqeri);
}

/****************************************************************************
 * Name: ra6m5serial_interrupt
 *
 * Description:
 *   This is the SCI interrupt handler.  It will be invoked when an
 *   interrupt is received on the 'irq'.  It should call uart_xmitchars or
 *   uart_recvchars to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'arg' to the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int ra6m5serial_interrupt(int irq, void *context, void *arg)
{
  struct ra6m5_serial_s *priv = (struct ra6m5_serial_s *)arg;
  int  passes;
  bool handled;

  DEBUGASSERT(priv != NULL);

  /* Report serial activity to the power management logic */

#if defined(CONFIG_PM) && CONFIG_RA6M5_PM_SERIAL_ACTIVITY > 0
  pm_activity(PM_IDLE_DOMAIN, CONFIG_RA6M5_PM_SERIAL_ACTIVITY);
#endif

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  handled = true;
  for (passes = 0; passes < 256 && handled; passes++)
    {
      handled = false;

      /* Get the masked SCI status word. */

      priv->sr = ra6m5serial_getreg8(priv, RA6M5_SCI_SSR_OFFSET);

      /* SCI interrupts:
       *
       * Enable           Status          Meaning
       * ---------------- -------------- ----------------------
       * SCI_SCR_RIE      SCI_SSR_RDRF    Received Data Full
       * SCI_SCR_TIE      SCI_SSR_TDRE    Transmit Data Empty
       * SCI_SCR_TEIE     SCI_SSR_TEND    Transmission End  (only RS-485)
       * 
       */

#ifdef HAVE_RS485
      /* Transmission of whole buffer is over - TC is set, TXEIE is cleared.
       * Note - this should be first, to have the most recent TC bit value
       * from SR register - sending data affects TC, but without refresh we
       * will not know that...
       */

      if ((priv->sr & SCI_SSR_TEND) != 0 &&
          (priv->ie & SCI_SCR_TEIE) != 0 &&
          (priv->ie & SCI_SCR_TIE) == 0)
        {
          ra6m5_gpiowrite(priv->rs485_dir_gpio, !priv->rs485_dir_polarity);
          ra6m5serial_restoreusartint(priv, priv->ie & ~SCI_SCR_TEIE);
        }
#endif

      /* Handle incoming, receive bytes. */

      if ((priv->sr & SCI_SSR_RDRF) != 0 &&
          (priv->ie & SCI_SCR_RIE) != 0)
        {
          /* Received data ready... process incoming bytes.  NOTE the check
           * for RDRF:  We cannot call uart_recvchards of RX interrupts are
           * disabled.
           */

          uart_recvchars(&priv->dev);
          handled = true;
        }

      /* We may still have to read from the DR register to clear any pending
       * error conditions.
       */

      else if ((priv->sr & (SCI_SSR_ORER | SCI_SSR_PER | SCI_SSR_FER))
               != 0)
        {
          /* These errors are cleared by writing "0" the corresponding 
           * bit to the Serial Status Register (SSR).
           */

          ra6m5serial_putreg8(priv, RA6M5_SCI_SSR_OFFSET,
                              ~(SCI_SSR_ORER | SCI_SSR_PER | SCI_SSR_FER));
        }

      /* Handle outgoing, transmit bytes */

      if ((priv->sr & SCI_SSR_TDRE) != 0 &&
          (priv->ie & SCI_SCR_TIE) != 0)
        {
          /* Transmit data register empty ... process outgoing bytes */

          uart_xmitchars(&priv->dev);
          handled = true;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: ra6m5serial_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int ra6m5serial_ioctl(struct file *filep, int cmd,
                             unsigned long arg)
{
#if defined(CONFIG_SERIAL_TERMIOS) || defined(CONFIG_SERIAL_TIOCSERGSTRUCT)
  struct inode      *inode = filep->f_inode;
  struct uart_dev_s *dev   = inode->i_private;
#endif
#if defined(CONFIG_SERIAL_TERMIOS)
  struct ra6m5_serial_s *priv =
    (struct ra6m5_serial_s *)dev->priv;
#endif
  int                ret    = OK;

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TIOCSERGSTRUCT
    case TIOCSERGSTRUCT:
      {
        struct ra6m5_serial_s *user;

        user = (struct ra6m5_serial_s *)arg;

        if (!user)
          {
            ret = -EINVAL;
          }
        else
          {
            memcpy(user, dev, sizeof(struct ra6m5_serial_s));
          }
      }
      break;
#endif

#ifdef CONFIG_RA6M5_SCI_SINGLEWIRE
    case TIOCSSINGLEWIRE:
      {
        uint32_t cr1;
        uint32_t cr1_ue;
        irqstate_t flags;

        flags = enter_critical_section();

        /* Get the original state of UE */

        cr1    = ra6m5serial_getreg8(priv, STM32_RA6M5_CR1_OFFSET);
        cr1_ue = cr1 & RA6M5_CR1_UE;
        cr1   &= ~RA6M5_CR1_UE;

        /* Disable UE, HDSEL can only be written when UE=0 */

        ra6m5serial_putreg8(priv, STM32_RA6M5_CR1_OFFSET, cr1);

        /* Change the TX port to be open-drain/push-pull and enable/disable
         * half-duplex mode.
         */

        uint32_t cr = ra6m5serial_getreg8(priv, STM32_RA6M5_CR3_OFFSET);

        if ((arg & SER_SINGLEWIRE_ENABLED) != 0)
          {
            uint32_t gpio_val = GPIO_OPENDRAIN;

            if ((arg & SER_SINGLEWIRE_PULL_MASK) == SER_SINGLEWIRE_PULLUP)
              {
                gpio_val |= GPIO_PULLUP;
              }
            else
              {
                gpio_val |= GPIO_FLOAT;
              }

            if ((arg & SER_SINGLEWIRE_PULL_MASK) == SER_SINGLEWIRE_PULLDOWN)
              {
                gpio_val |= GPIO_PULLDOWN;
              }
            else
              {
                gpio_val |= GPIO_FLOAT;
              }

            ra6m5_configgpio((priv->tx_gpio &
                                ~(GPIO_PUPD_MASK | GPIO_OPENDRAIN)) |
                               gpio_val);

            cr |= RA6M5_CR3_HDSEL;
          }
        else
          {
            ra6m5_configgpio((priv->tx_gpio &
                                ~(GPIO_PUPD_MASK | GPIO_OPENDRAIN)) |
                               GPIO_PUSHPULL);
            cr &= ~RA6M5_CR3_HDSEL;
          }

        ra6m5serial_putreg8(priv, STM32_RA6M5_CR3_OFFSET, cr);

        /* Re-enable UE if appropriate */

        ra6m5serial_putreg8(priv, STM32_RA6M5_CR1_OFFSET, cr1 | cr1_ue);
        leave_critical_section(flags);
      }
     break;
#endif

#ifdef CONFIG_RA6M5_RA6M5_INVERT
    case TIOCSINVERT:
      {
        uint8_t scr;
        uint8_t scr_en;
        irqstate_t flags;

        flags = enter_critical_section();

        /* Get the original state of TE/RE */

        scr    = ra6m5serial_getreg8(priv, RA6M5_SCI_SCR_OFFSET);
        scr_en = scr & 0x30;
        scr   &= ~0x30;

        /* Disable TE,RE, {R,T}XINV can only be written when TE=RE=0 */

        ra6m5serial_putreg8(priv, RA6M5_SCI_SCR_OFFSET, scr);

        /* Enable/disable signal inversion. */

        uint8_t sptr = ra6m5serial_getreg8(priv, RA6M5_SCI_SPTR_OFFSET);

        if (arg & SER_INVERT_ENABLED_RX)
          {
            sptr |= SCI_SPTR_RINV;
          }
        else
          {
            sptr &= ~SCI_SPTR_RINV;
          }

        if (arg & SER_INVERT_ENABLED_TX)
          {
            sptr |= SCI_SPTR_TINV;
          }
        else
          {
            sptr &= ~SCI_SPTR_TINV;
          }

        ra6m5serial_putreg8(priv, RA6M5_SCI_SPTR_OFFSET, sptr);

        /* Re-enable TE/RE if appropriate */

        ra6m5serial_putreg8(priv, RA6M5_SCI_SCR_OFFSET, scr | scr_en);
        leave_critical_section(flags);
      }
     break;
#endif

#ifdef CONFIG_RA6M5_RA6M5_SWAP
    case TIOCSSWAP:
      {
        uint8_t scr;
        uint8_t scr_en;
        irqstate_t flags;

        flags = enter_critical_section();

        /* Get the original state of TE/RE */

        scr    = ra6m5serial_getreg8(priv, RA6M5_SCI_SCR_OFFSET);
        scr_en = scr & 0x30;
        scr   &= ~0x30;

        /* Disable TE/RE, SWAP can only be written when TE=RE=0 */

        ra6m5serial_putreg8(priv, RA6M5_SCI_SCR_OFFSET, scr);

        /* Enable/disable Swap mode. */

        //uint32_t cr = ra6m5serial_getreg8(priv, STM32_RA6M5_CR2_OFFSET);

        if (arg == SER_SWAP_ENABLED)
          {

          }
        else
          {

          }

        //ra6m5serial_putreg8(priv, STM32_RA6M5_CR2_OFFSET, cr);

        /* Re-enable TE/RE if appropriate */

        ra6m5serial_putreg8(priv, RA6M5_SCI_SCR_OFFSET, scr | scr_en);
        leave_critical_section(flags);
      }
     break;
#endif

#ifdef CONFIG_SERIAL_TERMIOS
    case TCGETS:
      {
        struct termios *termiosp = (struct termios *)arg;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        cfsetispeed(termiosp, priv->baud);

        /* Note that since we only support 8/9 bit modes and
         * there is no way to report 9-bit mode, we always claim 8.
         */

        termiosp->c_cflag =
          ((priv->parity != 0) ? PARENB : 0) |
          ((priv->parity == 1) ? PARODD : 0) |
          ((priv->stopbits2) ? CSTOPB : 0) |
#ifdef CONFIG_SERIAL_OFLOWCONTROL
          ((priv->oflow) ? CCTS_OFLOW : 0) |
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
          ((priv->iflow) ? CRTS_IFLOW : 0) |
#endif
          CS8;

        /* TODO: CRTS_IFLOW, CCTS_OFLOW */
      }
      break;

    case TCSETS:
      {
        struct termios *termiosp = (struct termios *)arg;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        /* Perform some sanity checks before accepting any changes */

        if (((termiosp->c_cflag & CSIZE) != CS8)
#ifdef CONFIG_SERIAL_OFLOWCONTROL
            || ((termiosp->c_cflag & CCTS_OFLOW) && (priv->cts_gpio == 0))
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
            || ((termiosp->c_cflag & CRTS_IFLOW) && (priv->rts_gpio == 0))
#endif
           )
          {
            ret = -EINVAL;
            break;
          }

        if (termiosp->c_cflag & PARENB)
          {
            priv->parity = (termiosp->c_cflag & PARODD) ? 1 : 2;
          }
        else
          {
            priv->parity = 0;
          }

        priv->stopbits2 = (termiosp->c_cflag & CSTOPB) != 0;
#ifdef CONFIG_SERIAL_OFLOWCONTROL
        priv->oflow = (termiosp->c_cflag & CCTS_OFLOW) != 0;
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
        priv->iflow = (termiosp->c_cflag & CRTS_IFLOW) != 0;
#endif

        /* Note that since there is no way to request 9-bit mode
         * and no way to support 5/6/7-bit modes, we ignore them
         * all here.
         */

        /* Note that only cfgetispeed is used because we have knowledge
         * that only one speed is supported.
         */

        priv->baud = cfgetispeed(termiosp);

        /* Effect the changes immediately - note that we do not implement
         * TCSADRAIN / TCSAFLUSH
         */

        ra6m5serial_setformat(dev);
      }
      break;
#endif /* CONFIG_SERIAL_TERMIOS */

#ifdef CONFIG_RA6M5_RA6M5_BREAKS
#  ifdef CONFIG_RA6M5_SERIALBRK_BSDCOMPAT
    case TIOCSBRK:  /* BSD compatibility: Turn break on, unconditionally */
      {
        irqstate_t flags;
        uint32_t tx_break;

        flags = enter_critical_section();

        /* Disable any further tx activity */

        priv->ie |= RA6M5_CR1_IE_BREAK_INPROGRESS;

        ra6m5serial_txint(dev, false);

        /* Configure TX as a GPIO output pin and Send a break signal */

        tx_break = GPIO_OUTPUT |
                   (~(GPIO_MODE_MASK | GPIO_OUTPUT_SET) & priv->tx_gpio);
        ra6m5_configgpio(tx_break);

        leave_critical_section(flags);
      }
      break;

    case TIOCCBRK:  /* BSD compatibility: Turn break off, unconditionally */
      {
        irqstate_t flags;

        flags = enter_critical_section();

        /* Configure TX back to U(S)ART */

        ra6m5_configgpio(priv->tx_gpio);

        priv->ie &= ~RA6M5_CR1_IE_BREAK_INPROGRESS;

        /* Enable further tx activity */

        ra6m5serial_txint(dev, true);

        leave_critical_section(flags);
      }
      break;
#  else
    case TIOCSBRK:  /* No BSD compatibility: Turn break on for M bit times */
      {
        uint32_t cr1;
        irqstate_t flags;

        flags = enter_critical_section();
        cr1   = ra6m5serial_getreg8(priv, STM32_RA6M5_CR1_OFFSET);
        ra6m5serial_putreg8(priv, STM32_RA6M5_CR1_OFFSET,
                             cr1 | RA6M5_CR1_SBK);
        leave_critical_section(flags);
      }
      break;

    case TIOCCBRK:  /* No BSD compatibility: May turn off break too soon */
      {
        uint32_t cr1;
        irqstate_t flags;

        flags = enter_critical_section();
        cr1   = ra6m5serial_getreg8(priv, STM32_RA6M5_CR1_OFFSET);
        ra6m5serial_putreg8(priv, STM32_RA6M5_CR1_OFFSET,
                             cr1 & ~RA6M5_CR1_SBK);
        leave_critical_section(flags);
      }
      break;
#  endif
#endif

    default:
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: ra6m5serial_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the SCI.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

#ifndef SERIAL_HAVE_ONLY_DMA
static int ra6m5serial_receive(struct uart_dev_s *dev,
                               unsigned int *status)
{
  struct ra6m5_serial_s *priv =
    (struct ra6m5_serial_s *)dev->priv;
  uint8_t rdr;

  /* Get the Rx byte */

  rdr      = ra6m5serial_getreg8(priv, RA6M5_SCI_RDR_OFFSET);

  /* Get the Rx byte plux error information.  Return those in status */

  *status  = priv->sr << 16 | rdr;
  priv->sr = 0;

  /* Then return the actual received byte */

  return rdr & 0xff;
}
#endif

/****************************************************************************
 * Name: ra6m5serial_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

#ifndef SERIAL_HAVE_ONLY_DMA
static void ra6m5serial_rxint(struct uart_dev_s *dev, bool enable)
{
  struct ra6m5_serial_s *priv =
    (struct ra6m5_serial_s *)dev->priv;
  irqstate_t flags;
  uint8_t ie;

  /* SCI interrupts:
    *
    * Enable           Status          Meaning
    * ---------------- -------------- ----------------------
    * SCI_SCR_RIE      SCI_SSR_RDRF    Received Data Full
    * SCI_SCR_TIE      SCI_SSR_TDRE    Transmit Data Empty
    * SCI_SCR_TEIE     SCI_SSR_TEND    Transmission End  (only RS-485)
    */

  flags = enter_critical_section();
  ie = priv->ie;
  if (enable)
    {
      /* Receive an interrupt when their is anything in the Rx data
       * register (or an Rx timeout occurs).
       */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
#ifdef CONFIG_SCI_ERRINTS
      ie |= (SCI_SCR_RIE);
#else
      ie |= SCI_SCR_RIE;
#endif
#endif
    }
  else
    {
      ie &= ~(SCI_SCR_RIE);
    }

  /* Then set the new interrupt state */

  ra6m5serial_restoreusartint(priv, ie);
  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: ra6m5serial_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

#ifndef SERIAL_HAVE_ONLY_DMA
static bool ra6m5serial_rxavailable(struct uart_dev_s *dev)
{
  struct ra6m5_serial_s *priv =
    (struct ra6m5_serial_s *)dev->priv;

  return ((ra6m5serial_getreg8(priv, RA6M5_SCI_SSR_OFFSET) &
           SCI_SSR_RDRF) != 0);
}
#endif

/****************************************************************************
 * Name: ra6m5serial_rxflowcontrol
 *
 * Description:
 *   Called when Rx buffer is full (or exceeds configured watermark levels
 *   if CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS is defined).
 *   Return true if UART activated RX flow control to block more incoming
 *   data
 *
 * Input Parameters:
 *   dev       - UART device instance
 *   nbuffered - the number of characters currently buffered
 *               (if CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS is
 *               not defined the value will be 0 for an empty buffer or the
 *               defined buffer size for a full buffer)
 *   upper     - true indicates the upper watermark was crossed where
 *               false indicates the lower watermark has been crossed
 *
 * Returned Value:
 *   true if RX flow control activated.
 *
 ****************************************************************************/

#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool ra6m5serial_rxflowcontrol(struct uart_dev_s *dev,
                                      unsigned int nbuffered, bool upper)
{
  struct ra6m5_serial_s *priv =
    (struct ra6m5_serial_s *)dev->priv;

#if defined(CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS) && \
    defined(CONFIG_RA6M5_FLOWCONTROL_BROKEN)
  if (priv->iflow && (priv->rts_gpio != 0))
    {
      /* Assert/de-assert nRTS set it high resume/stop sending */

      ra6m5_gpiowrite(priv->rts_gpio, upper);

      if (upper)
        {
          /* With heavy Rx traffic, RXNE might be set and data pending.
           * Returning 'true' in such case would cause RXNE left unhandled
           * and causing interrupt storm. Sending end might be also be slow
           * to react on nRTS, and returning 'true' here would prevent
           * processing that data.
           *
           * Therefore, return 'false' so input data is still being processed
           * until sending end reacts on nRTS signal and stops sending more.
           */

          return false;
        }

      return upper;
    }

#else
  if (priv->iflow)
    {
      /* Is the RX buffer full? */

      if (upper)
        {
          /* Disable Rx interrupt to prevent more data being from
           * peripheral.  When hardware RTS is enabled, this will
           * prevent more data from coming in.
           *
           * This function is only called when UART recv buffer is full,
           * that is: "dev->recv.head + 1 == dev->recv.tail".
           *
           * Logic in "uart_read" will automatically toggle Rx interrupts
           * when buffer is read empty and thus we do not have to re-
           * enable Rx interrupts.
           */

          uart_disablerxint(dev);
          return true;
        }

      /* No.. The RX buffer is empty */

      else
        {
          /* We might leave Rx interrupt disabled if full recv buffer was
           * read empty.  Enable Rx interrupt to make sure that more input is
           * received.
           */

          uart_enablerxint(dev);
        }
    }
#endif

  return false;
}
#endif

/****************************************************************************
 * Name: ra6m5serial_dmareceive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the SCI.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_DMA
static int ra6m5serial_dmareceive(struct uart_dev_s *dev,
                                  unsigned int *status)
{
  struct ra6m5_serial_s *priv =
    (struct ra6m5_serial_s *)dev->priv;
  int c = 0;

  if (ra6m5serial_dmanextrx(priv) != priv->rxdmanext)
    {
      c = priv->rxfifo[priv->rxdmanext];

      priv->rxdmanext++;
      if (priv->rxdmanext == RXDMA_BUFFER_SIZE)
        {
#ifdef CONFIG_SERIAL_IFLOWCONTROL
          if (priv->iflow)
            {
              /* RX DMA buffer full. RX paused, RTS line pulled up to prevent
               * more input data from other end.
               */
            }
          else
#endif
            {
              priv->rxdmanext = 0;
            }
        }
    }

  return c;
}
#endif

/****************************************************************************
 * Name: ra6m5serial_dmareenable
 *
 * Description:
 *   Call to re-enable RX DMA.
 *
 ****************************************************************************/

#if defined(SERIAL_HAVE_DMA)
static void ra6m5serial_dmareenable(struct ra6m5_serial_s *priv)
{
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  if (priv->iflow)
    {
      /* Configure for non-circular DMA reception into the RX FIFO */

      ra6m5_dmasetup(priv->rxdma,
                       priv->base + RA6M5_SCI_RDR_OFFSET,
                       (uint32_t)priv->rxfifo,
                       RXDMA_BUFFER_SIZE,
                       SERIAL_DMA_IFLOW_CONTROL_WORD);
    }
  else
#endif
    {
      /* Configure for circular DMA reception into the RX FIFO */

      ra6m5_dmasetup(priv->rxdma,
                       priv->base + RA6M5_SCI_RDR_OFFSET,
                       (uint32_t)priv->rxfifo,
                       RXDMA_BUFFER_SIZE,
                       SERIAL_DMA_CONTROL_WORD);
    }

  /* Reset our DMA shadow pointer to match the address just
   * programmed above.
   */

  priv->rxdmanext = 0;

#ifdef CONFIG_SERIAL_IFLOWCONTROL
  if (priv->iflow)
    {
      /* Start the DMA channel, and arrange for callbacks at the full point
       * in the FIFO. After buffer gets full, hardware flow-control kicks
       * in and DMA transfer is stopped.
       */

      ra6m5_dmastart(priv->rxdma, ra6m5serial_dmarxcallback,
                      (void *)priv, false);
    }
  else
#endif
    {
      /* Start the DMA channel, and arrange for callbacks at the half and
       * full points in the FIFO.  This ensures that we have half a FIFO
       * worth of time to claim bytes before they are overwritten.
       */

      ra6m5_dmastart(priv->rxdma, ra6m5serial_dmarxcallback,
                      (void *)priv, true);
    }

#ifdef CONFIG_PM
  /* Clear DMA suspended flag. */

  priv->rxdmasusp = false;
#endif
}
#endif

/****************************************************************************
 * Name: ra6m5serial_dmaiflowrestart
 *
 * Description:
 *   Call to restart RX DMA for input flow-controlled SCI
 *
 ****************************************************************************/

#if defined(SERIAL_HAVE_DMA) && defined(CONFIG_SERIAL_IFLOWCONTROL)
static bool ra6m5serial_dmaiflowrestart(struct ra6m5_serial_s *priv)
{
  if (!priv->rxenable)
    {
      /* Rx not enabled by upper layer. */

      return false;
    }

  if (priv->rxdmanext != RXDMA_BUFFER_SIZE)
    {
#ifdef CONFIG_PM
      if (priv->rxdmasusp)
        {
          /* Rx DMA in suspended state. */

          if (ra6m5serial_dmarxavailable(&priv->dev))
            {
              /* DMA buffer has unprocessed data, do not re-enable yet. */

              return false;
            }
        }
      else
#endif
        {
          return false;
        }
    }

  /* DMA is stopped or suspended and DMA buffer does not have pending data,
   * re-enabling without data loss is now safe.
   */

  ra6m5serial_dmareenable(priv);

  return true;
}
#endif

/****************************************************************************
 * Name: ra6m5serial_dmarxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_DMA
static void ra6m5serial_dmarxint(struct uart_dev_s *dev, bool enable)
{
  struct ra6m5_serial_s *priv =
    (struct ra6m5_serial_s *)dev->priv;

  /* En/disable DMA reception.
   *
   * Note that it is not safe to check for available bytes and immediately
   * pass them to uart_recvchars as that could potentially recurse back
   * to us again.  Instead, bytes must wait until the next up_dma_poll or
   * DMA event.
   */

  priv->rxenable = enable;

#ifdef CONFIG_SERIAL_IFLOWCONTROL
  if (priv->iflow)
    {
      /* Re-enable RX DMA. */

      ra6m5serial_dmaiflowrestart(priv);
    }
#endif
}
#endif

/****************************************************************************
 * Name: ra6m5serial_dmarxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_DMA
static bool ra6m5serial_dmarxavailable(struct uart_dev_s *dev)
{
  struct ra6m5_serial_s *priv =
    (struct ra6m5_serial_s *)dev->priv;

  /* Compare our receive pointer to the current DMA pointer, if they
   * do not match, then there are bytes to be received.
   */

  return (ra6m5serial_dmanextrx(priv) != priv->rxdmanext);
}
#endif

/****************************************************************************
 * Name: ra6m5serial_send
 *
 * Description:
 *   This method will send one byte on the SCI
 *
 ****************************************************************************/

static void ra6m5serial_send(struct uart_dev_s *dev, int ch)
{
  struct ra6m5_serial_s *priv =
    (struct ra6m5_serial_s *)dev->priv;

#ifdef HAVE_RS485
  if (priv->rs485_dir_gpio != 0)
    {
      ra6m5_gpiowrite(priv->rs485_dir_gpio, priv->rs485_dir_polarity);
    }
#endif

  ra6m5serial_putreg8(priv, RA6M5_SCI_TDR_OFFSET, (uint8_t)ch);
}

/****************************************************************************
 * Name: ra6m5serial_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void ra6m5serial_txint(struct uart_dev_s *dev, bool enable)
{
  struct ra6m5_serial_s *priv =
    (struct ra6m5_serial_s *)dev->priv;
  irqstate_t flags;

  /* SCI transmit interrupts:
   *
   * Enable           Status          Meaning
   * ---------------- -------------- ----------------------
   * SCI_SCR_TIE      SCI_SSR_TDRE    Transmit Data Empty
   * SCI_SCR_TEIE     SCI_SSR_TEND    Transmission End  (only RS-485)
   */

  flags = enter_critical_section();
  if (enable)
    {
      /* Set to receive an interrupt when the TX data register is empty */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      uint8_t ie = priv->ie | SCI_SCR_TIE;

      /* If RS-485 is supported on this SCI, then also enable the
       * transmission complete interrupt.
       */

#  ifdef HAVE_RS485
      if (priv->rs485_dir_gpio != 0)
        {
          ie |= SCI_SCR_TEIE;
        }
#  endif

#  ifdef CONFIG_RA6M5_SERIALBRK_BSDCOMPAT
      if (priv->ie & USART_CR1_IE_BREAK_INPROGRESS)
        {
          leave_critical_section(flags);
          return;
        }
#  endif

      ra6m5serial_restoreusartint(priv, ie);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
#endif
    }
  else
    {
      /* Disable the TX interrupt */

      ra6m5serial_restoreusartint(priv, priv->ie & ~SCI_SCR_TIE);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: ra6m5serial_txready
 *
 * Description:
 *   Return true if the transmit data register is empty
 *
 ****************************************************************************/

static bool ra6m5serial_txready(struct uart_dev_s *dev)
{
  struct ra6m5_serial_s *priv =
    (struct ra6m5_serial_s *)dev->priv;

  return ((ra6m5serial_getreg8(priv, RA6M5_SCI_SSR_OFFSET) &
           SCI_SSR_TDRE) == SCI_SSR_TDRE);
}

/****************************************************************************
 * Name: ra6m5serial_dmarxcallback
 *
 * Description:
 *   This function checks the current DMA state and calls the generic
 *   serial stack when bytes appear to be available.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_DMA
static void ra6m5serial_dmarxcallback(DMA_HANDLE handle, uint8_t status,
                                      void *arg)
{
  struct ra6m5_serial_s *priv = (struct ra6m5_serial_s *)arg;

  if (priv->rxenable && ra6m5serial_dmarxavailable(&priv->dev))
    {
      uart_recvchars(&priv->dev);

#ifdef CONFIG_SERIAL_IFLOWCONTROL
      if (priv->iflow)
        {
          /* Re-enable RX DMA. */

          ra6m5serial_dmaiflowrestart(priv);
        }
#endif
    }

  /* Get the masked SCI status word to check and clear error flags.
   *
   * When wake-up from low power mode was not fast enough, UART is resumed
   * too late and sometimes exactly when character was coming over UART,
   * resulting to frame error.
   * If error flag is not cleared, Rx DMA will be stuck. Clearing errors
   * will release Rx DMA.
   */

  priv->sr = ra6m5serial_getreg8(priv, RA6M5_SCI_ISR_OFFSET);

  if ((priv->sr & (SCI_ISR_ORE | SCI_ISR_NF | SCI_ISR_FE)) != 0)
    {
      ra6m5serial_putreg8(priv, RA6M5_SCI_ICR_OFFSET,
                           (SCI_ICR_NCF | SCI_ICR_ORECF |
                            SCI_ICR_FECF));
    }
}
#endif

/****************************************************************************
 * Name: ra6m5serial_pmnotify
 *
 * Description:
 *   Notify the driver of new power state. This callback is  called after
 *   all drivers have had the opportunity to prepare for the new power state.
 *
 * Input Parameters:
 *
 *    cb - Returned to the driver. The driver version of the callback
 *         structure may include additional, driver-specific state data at
 *         the end of the structure.
 *
 *    pmstate - Identifies the new PM state
 *
 * Returned Value:
 *   None - The driver already agreed to transition to the low power
 *   consumption state when when it returned OK to the prepare() call.
 *
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static void ra6m5serial_pmnotify(struct pm_callback_s *cb, int domain,
                                 enum pm_state_e pmstate)
{
  switch (pmstate)
    {
      case PM_NORMAL:
        {
          ra6m5serial_pm_setsuspend(false);
        }
        break;

      case PM_IDLE:
        {
          ra6m5serial_pm_setsuspend(false);
        }
        break;

      case PM_STANDBY:
        {
          /* TODO: Alternative configuration and logic for enabling serial in
           *       Stop 1 mode with HSI16 missing. Current logic allows
           *       suspending serial peripherals for Stop 0/1/2 when serial
           *       Rx/Tx buffers are empty (checked in pmprepare).
           */

          ra6m5serial_pm_setsuspend(true);
        }
        break;

      case PM_SLEEP:
        {
          ra6m5serial_pm_setsuspend(true);
        }
        break;

      default:

        /* Should not get here */

        break;
    }
}
#endif

/****************************************************************************
 * Name: ra6m5serial_pmprepare
 *
 * Description:
 *   Request the driver to prepare for a new power state. This is a warning
 *   that the system is about to enter into a new power state. The driver
 *   should begin whatever operations that may be required to enter power
 *   state. The driver may abort the state change mode by returning a
 *   non-zero value from the callback function.
 *
 * Input Parameters:
 *
 *    cb - Returned to the driver. The driver version of the callback
 *         structure may include additional, driver-specific state data at
 *         the end of the structure.
 *
 *    pmstate - Identifies the new PM state
 *
 * Returned Value:
 *   Zero - (OK) means the event was successfully processed and that the
 *          driver is prepared for the PM state change.
 *
 *   Non-zero - means that the driver is not prepared to perform the tasks
 *              needed achieve this power setting and will cause the state
 *              change to be aborted. NOTE: The prepare() method will also
 *              be called when reverting from lower back to higher power
 *              consumption modes (say because another driver refused a
 *              lower power state change). Drivers are not permitted to
 *              return non-zero values when reverting back to higher power
 *              consumption modes!
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static int ra6m5serial_pmprepare(struct pm_callback_s *cb, int domain,
                                 enum pm_state_e pmstate)
{
  int n;

  /* Logic to prepare for a reduced power state goes here. */

  switch (pmstate)
    {
    case PM_NORMAL:
    case PM_IDLE:
      break;

    case PM_STANDBY:
    case PM_SLEEP:

#ifdef SERIAL_HAVE_DMA
      /* Flush Rx DMA buffers before checking state of serial device
       * buffers.
       */

      ra6m5_serial_dma_poll();
#endif

      /* Check if any of the active ports have data pending on Tx/Rx
       * buffers.
       */

      for (n = 0; n < RA6M5_NLPUART + RA6M5_NSCI + RA6M5_NUART; n++)
        {
          struct ra6m5_serial_s *priv = g_uart_devs[n];

          if (!priv || !priv->initialized)
            {
              /* Not active, skip. */

              continue;
            }

          if (priv->suspended)
            {
              /* Port already suspended, skip. */

              continue;
            }

          /* Check if port has data pending (Rx & Tx). */

          if (priv->dev.xmit.head != priv->dev.xmit.tail)
            {
              return ERROR;
            }

          if (priv->dev.recv.head != priv->dev.recv.tail)
            {
              return ERROR;
            }
        }
      break;

    default:

      /* Should not get here */

      break;
    }

  return OK;
}
#endif

#endif /* HAVE_UART */
#endif /* USE_SERIALDRIVER */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef USE_SERIALDRIVER

/****************************************************************************
 * Name: arm_earlyserialinit
 *
 * Description:
 *   Performs the low level SCI initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before arm_serialinit.
 *
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT
void arm_earlyserialinit(void)
{
#ifdef HAVE_UART
  unsigned i;

  /* Disable all SCI interrupts */

  for (i = 0; i < RA6M5_NSCI; i++)
    {
      if (g_uart_devs[i])
        {
          ra6m5serial_disableusartint(g_uart_devs[i], NULL);
        }
    }

  /* Configure whichever one is the console */

#ifdef CONSOLE_UART
  ra6m5serial_setup(&g_uart_devs[CONSOLE_UART]->dev);
#endif
#endif /* HAVE UART */
}
#endif /* USE_EARLYSERIALINIT */

/****************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that arm_earlyserialinit was called previously.
 *
 ****************************************************************************/

void arm_serialinit(void)
{
#ifdef HAVE_UART
  char devname[16];
  unsigned i;
  unsigned minor = 0;
#ifdef CONFIG_PM
  int ret;
#endif

  /* Register to receive power management callbacks */

#ifdef CONFIG_PM
  ret = pm_register(&g_serialpm.pm_cb);
  DEBUGASSERT(ret == OK);
  UNUSED(ret);
#endif

  /* Register the console */

#ifdef CONSOLE_UART
  uart_register("/dev/console", &g_uart_devs[CONSOLE_UART]->dev);

#ifndef CONFIG_RA6M5_SERIAL_DISABLE_REORDERING
  /* If not disabled, register the console UART to ttyS0 and exclude
   * it from initializing it further down
   */

  uart_register("/dev/ttyS0", &g_uart_devs[CONSOLE_UART]->dev);
  minor = 1;
#endif

#ifdef SERIAL_HAVE_CONSOLE_DMA
  /* If we need to re-initialise the console to enable DMA do that here. */

  ra6m5serial_dmasetup(&g_uart_devs[CONSOLE_UART]->dev);
#endif
#endif /* CONSOLE_UART */

  /* Register all remaining SCIs */

  strlcpy(devname, "/dev/ttySx", sizeof(devname));

  for (i = 0; i < RA6M5_NSCI; i++)
    {
      /* Don't create a device for non-configured ports. */

      if (g_uart_devs[i] == 0)
        {
          continue;
        }

#ifndef CONFIG_RA6M5_SERIAL_DISABLE_REORDERING
      /* Don't create a device for the console - we did that above */

      if (g_uart_devs[i]->dev.isconsole)
        {
          continue;
        }
#endif

      /* Register SCIs as devices in increasing order */

      devname[9] = '0' + minor++;
      uart_register(devname, &g_uart_devs[i]->dev);
    }
#endif /* HAVE UART */
}

/****************************************************************************
 * Name: ra6m5_serial_dma_poll
 *
 * Description:
 *   Checks receive DMA buffers for received bytes that have not accumulated
 *   to the point where the DMA half/full interrupt has triggered.
 *
 *   This function should be called from a timer or other periodic context.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_DMA
void ra6m5_serial_dma_poll(void)
{
    irqstate_t flags;

    flags = enter_critical_section();

#ifdef CONFIG_LPUART1_RXDMA
  if (g_lpuart1priv.rxdma != NULL)
    {
      ra6m5serial_dmarxcallback(g_lpuart1priv.rxdma, 0, &g_lpuart1priv);
    }
#endif

#ifdef CONFIG_SCI0_RXDMA
  if (g_sci0priv.rxdma != NULL)
    {
      ra6m5serial_dmarxcallback(g_sci0priv.rxdma, 0, &g_sci0priv);
    }
#endif

#ifdef CONFIG_SCI1_RXDMA
  if (g_sci1priv.rxdma != NULL)
    {
      ra6m5serial_dmarxcallback(g_sci1priv.rxdma, 0, &g_sci1priv);
    }
#endif

#ifdef CONFIG_SCI2_RXDMA
  if (g_sci2priv.rxdma != NULL)
    {
      ra6m5serial_dmarxcallback(g_sci2priv.rxdma, 0, &g_sci2priv);
    }
#endif

#ifdef CONFIG_SCI3_RXDMA
  if (g_sci3priv.rxdma != NULL)
    {
      ra6m5serial_dmarxcallback(g_sci3priv.rxdma, 0, &g_sci3priv);
    }
#endif

#ifdef CONFIG_SCI4_RXDMA
  if (g_sci4priv.rxdma != NULL)
    {
      ra6m5serial_dmarxcallback(g_sci4priv.rxdma, 0, &g_sci4priv);
    }
#endif

  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#ifdef CONSOLE_UART
  struct ra6m5_serial_s *priv = g_uart_devs[CONSOLE_UART];
  uint8_t ie;

  ra6m5serial_disableusartint(priv, &ie);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      arm_lowputc('\r');
    }

  arm_lowputc(ch);
  ra6m5serial_restoreusartint(priv, ie);
#endif
  return ch;
}

#else /* USE_SERIALDRIVER */

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#ifdef CONSOLE_UART
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      arm_lowputc('\r');
    }

  arm_lowputc(ch);
#endif
  return ch;
}

#endif /* USE_SERIALDRIVER */
