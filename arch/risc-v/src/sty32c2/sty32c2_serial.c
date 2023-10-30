/****************************************************************************
 * arch/risc-v/src/sty32c2/sty32c2_serial.c
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
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>

#include <arch/board/board.h>

#include "riscv_internal.h"
#include "sty32c2_config.h"
#include "chip.h"
#include "sty32c2.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sty32c2_tty_s
{
  uart_dev_t * dev; /* TTY Device Reference */
  const int    idx; /* TTY Index */
};

/* If we are not using the serial driver for the console, then we still must
 * provide some minimal implementation of up_putc.
 */

#ifdef USE_SERIALDRIVER

/* Which UART with be tty0/console and which tty1?  The console will always
 * be ttyS0.  If there is no console then will use the lowest numbered UART.
 */

#ifdef HAVE_SERIAL_CONSOLE
#  if defined(CONFIG_UART0_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart0port     /* UART0 is console */
#    define SERIAL_CONSOLE  0
#  elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart1port     /* UART1 is console */
#    define SERIAL_CONSOLE  1
#  elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart2port     /* UART2 is console */
#    define SERIAL_CONSOLE  2
#  elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart3port     /* UART3 is console */
#    define SERIAL_CONSOLE  3
#  elif defined(CONFIG_UART4_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart4port     /* UART4 is console */
#    define SERIAL_CONSOLE  4
#  elif defined(CONFIG_UART5_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart5port     /* UART5 is console */
#    define SERIAL_CONSOLE  5
#  elif defined(CONFIG_UART6_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart6port     /* UART6 is console */
#    define SERIAL_CONSOLE  6
#  else
#    error "I'm confused... Do we have a serial console or not?"
#  endif
#else
#  undef  CONSOLE_DEV                        /* No console */
#  undef  CONFIG_UART0_SERIAL_CONSOLE
#  undef  CONFIG_UART1_SERIAL_CONSOLE
#  undef  CONFIG_UART2_SERIAL_CONSOLE
#  undef  CONFIG_UART3_SERIAL_CONSOLE
#  undef  CONFIG_UART4_SERIAL_CONSOLE
#  undef  CONFIG_UART5_SERIAL_CONSOLE
#  undef  CONFIG_UART6_SERIAL_CONSOLE
#  if defined(CONFIG_STY32C2_UART0)
#    define SERIAL_CONSOLE  0
#  endif
#endif

/* Common initialization logic will not not know that the all of the UARTs
 * have been disabled.  So, as a result, we may still have to provide
 * stub implementations of riscv_earlyserialinit(), riscv_serialinit(), and
 * up_putc().
 */

#ifdef HAVE_UART_DEVICE

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  uintptr_t uartbase; /* Base address of UART registers */
  uint32_t  baud;     /* Configured baud */
  uint8_t   irq;      /* IRQ associated with this UART */
  uint8_t   im;       /* Interrupt mask state */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level helpers */

static uint32_t up_serialin(struct up_dev_s *priv, int offset);
static void up_serialout(struct up_dev_s *priv, int offset, uint32_t value);
static void up_restoreuartint(struct up_dev_s *priv, uint8_t im);
static void up_disableuartint(struct up_dev_s *priv, uint8_t *im);

/* Serial driver methods */

static int  up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int  up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
static int  up_interrupt(int irq, void *context, void *arg);
static int  up_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  up_receive(struct uart_dev_s *dev, unsigned int *status);
static void up_rxint(struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(struct uart_dev_s *dev);
static void up_send(struct uart_dev_s *dev, int ch);
static void up_txint(struct uart_dev_s *dev, bool enable);
static bool up_txready(struct uart_dev_s *dev);
static bool up_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops =
{
  .setup          = up_setup,
  .shutdown       = up_shutdown,
  .attach         = up_attach,
  .detach         = up_detach,
  .ioctl          = up_ioctl,
  .receive        = up_receive,
  .rxint          = up_rxint,
  .rxavailable    = up_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = up_send,
  .txint          = up_txint,
  .txready        = up_txready,
  .txempty        = up_txempty,
};

/* UART Instances */

#ifdef CONFIG_STY32C2_UART0
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];

static struct up_dev_s g_uart0priv =
{
  .uartbase  = STY32C2_UART0_BASE,
  .baud      = CONFIG_UART0_BAUD,
  .irq       = STY32C2_IRQ_UART0,
};

static uart_dev_t g_uart0port =
{
#ifdef SERIAL_CONSOLE
  .isconsole = SERIAL_CONSOLE == 0,
#endif
  .recv      =
  {
    .size    = CONFIG_UART0_RXBUFSIZE,
    .buffer  = g_uart0rxbuffer,
  },
  .xmit      =
  {
    .size    = CONFIG_UART0_TXBUFSIZE,
    .buffer  = g_uart0txbuffer,
  },
  .ops       = &g_uart_ops,
  .priv      = &g_uart0priv,
};
#endif

#ifdef CONFIG_STY32C2_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];

static struct up_dev_s g_uart1priv =
{
  .uartbase  = STY32C2_UART1_BASE,
  .baud      = CONFIG_UART1_BAUD,
  .irq       = STY32C2_IRQ_UART1,
};

static uart_dev_t g_uart1port =
{
#ifdef SERIAL_CONSOLE
  .isconsole = SERIAL_CONSOLE == 1,
#endif
  .recv      =
  {
    .size    = CONFIG_UART1_RXBUFSIZE,
    .buffer  = g_uart1rxbuffer,
  },
  .xmit      =
  {
    .size    = CONFIG_UART1_TXBUFSIZE,
    .buffer  = g_uart1txbuffer,
  },
  .ops       = &g_uart_ops,
  .priv      = &g_uart1priv,
};
#endif

#ifdef CONFIG_STY32C2_UART2
static char g_uart2rxbuffer[CONFIG_UART2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_UART2_TXBUFSIZE];

static struct up_dev_s g_uart2priv =
{
  .uartbase  = STY32C2_UART2_BASE,
  .baud      = CONFIG_UART2_BAUD,
  .irq       = STY32C2_IRQ_UART2,
};

static uart_dev_t g_uart2port =
{
#ifdef SERIAL_CONSOLE
  .isconsole = SERIAL_CONSOLE == 2,
#endif
  .recv      =
  {
    .size    = CONFIG_UART2_RXBUFSIZE,
    .buffer  = g_uart2rxbuffer,
  },
  .xmit      =
  {
    .size    = CONFIG_UART2_TXBUFSIZE,
    .buffer  = g_uart2txbuffer,
  },
  .ops       = &g_uart_ops,
  .priv      = &g_uart2priv,
};
#endif

#ifdef CONFIG_STY32C2_UART3
static char g_uart3rxbuffer[CONFIG_UART3_RXBUFSIZE];
static char g_uart3txbuffer[CONFIG_UART3_TXBUFSIZE];

static struct up_dev_s g_uart3priv =
{
  .uartbase  = STY32C2_UART3_BASE,
  .baud      = CONFIG_UART3_BAUD,
  .irq       = STY32C2_IRQ_UART3,
};

static uart_dev_t g_uart3port =
{
#ifdef SERIAL_CONSOLE
  .isconsole = SERIAL_CONSOLE == 3,
#endif
  .recv      =
  {
    .size    = CONFIG_UART3_RXBUFSIZE,
    .buffer  = g_uart3rxbuffer,
  },
  .xmit      =
  {
    .size    = CONFIG_UART3_TXBUFSIZE,
    .buffer  = g_uart3txbuffer,
  },
  .ops       = &g_uart_ops,
  .priv      = &g_uart3priv,
};
#endif

#ifdef CONFIG_STY32C2_UART4
static char g_uart4rxbuffer[CONFIG_UART4_RXBUFSIZE];
static char g_uart4txbuffer[CONFIG_UART4_TXBUFSIZE];

static struct up_dev_s g_uart4priv =
{
  .uartbase  = STY32C2_UART4_BASE,
  .baud      = CONFIG_UART4_BAUD,
  .irq       = STY32C2_IRQ_UART4,
};

static uart_dev_t g_uart4port =
{
#ifdef SERIAL_CONSOLE
  .isconsole = SERIAL_CONSOLE == 4,
#endif
  .recv      =
  {
    .size    = CONFIG_UART4_RXBUFSIZE,
    .buffer  = g_uart4rxbuffer,
  },
  .xmit      =
  {
    .size    = CONFIG_UART4_TXBUFSIZE,
    .buffer  = g_uart4txbuffer,
  },
  .ops       = &g_uart_ops,
  .priv      = &g_uart4priv,
};
#endif

#ifdef CONFIG_STY32C2_UART5
static char g_uart5rxbuffer[CONFIG_UART5_RXBUFSIZE];
static char g_uart5txbuffer[CONFIG_UART5_TXBUFSIZE];

static struct up_dev_s g_uart5priv =
{
  .uartbase  = STY32C2_UART5_BASE,
  .baud      = CONFIG_UART5_BAUD,
  .irq       = STY32C2_IRQ_UART5,
};

static uart_dev_t g_uart5port =
{
#ifdef SERIAL_CONSOLE
  .isconsole = SERIAL_CONSOLE == 4,
#endif
  .recv      =
  {
    .size    = CONFIG_UART5_RXBUFSIZE,
    .buffer  = g_uart5rxbuffer,
  },
  .xmit      =
  {
    .size    = CONFIG_UART5_TXBUFSIZE,
    .buffer  = g_uart5txbuffer,
  },
  .ops       = &g_uart_ops,
  .priv      = &g_uart5priv,
};
#endif

#ifdef CONFIG_STY32C2_UART6
static char g_uart6rxbuffer[CONFIG_UART6_RXBUFSIZE];
static char g_uart6txbuffer[CONFIG_UART6_TXBUFSIZE];

static struct up_dev_s g_uart6priv =
{
  .uartbase  = STY32C2_UART6_BASE,
  .baud      = CONFIG_UART6_BAUD,
  .irq       = STY32C2_IRQ_UART6,
};

static uart_dev_t g_uart6port =
{
#ifdef SERIAL_CONSOLE
  .isconsole = SERIAL_CONSOLE == 4,
#endif
  .recv      =
  {
    .size    = CONFIG_UART6_RXBUFSIZE,
    .buffer  = g_uart6rxbuffer,
  },
  .xmit      =
  {
    .size    = CONFIG_UART6_TXBUFSIZE,
    .buffer  = g_uart6txbuffer,
  },
  .ops       = &g_uart_ops,
  .priv      = &g_uart6priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_serialin
 ****************************************************************************/

static uint32_t up_serialin(struct up_dev_s *priv, int offset)
{
  return getreg32(priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_serialout
 ****************************************************************************/

static void up_serialout(struct up_dev_s *priv, int offset, uint32_t value)
{
  putreg32(value, priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_restoreuartint
 ****************************************************************************/

static void up_restoreuartint(struct up_dev_s *priv, uint8_t im)
{
  irqstate_t flags = enter_critical_section();

  priv->im = im;

  putreg32(getreg32(priv->uartbase + UART_EV_PENDING_OFFSET), \
                    priv->uartbase + UART_EV_PENDING_OFFSET);
  up_serialout(priv, UART_EV_ENABLE_OFFSET, im);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_disableuartint
 ****************************************************************************/

static void up_disableuartint(struct up_dev_s *priv, uint8_t *im)
{
  irqstate_t flags = enter_critical_section();

  /* Return the current interrupt mask value */

  if (im)
    {
     *im = priv->im;
    }

  /* Disable all interrupts */

  priv->im = 0;

  putreg32(getreg32(priv->uartbase + UART_EV_PENDING_OFFSET), \
                    priv->uartbase + UART_EV_PENDING_OFFSET);
  up_serialout(priv, UART_EV_ENABLE_OFFSET, 0);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int up_setup(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* default baudrate set by fpga fabric is 115200 */

  /* Clear the pending flags */

  putreg32(getreg32(priv->uartbase + UART_EV_PENDING_OFFSET), \
                    priv->uartbase + UART_EV_PENDING_OFFSET);

  /* Enable RX & TX */

  putreg32(UART_EV_RX | UART_EV_TX, priv->uartbase + UART_EV_ENABLE_OFFSET);

  return OK;
}

/****************************************************************************
 * Name: up_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void up_shutdown(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Disable interrupts */

  up_disableuartint(priv, NULL);
}

/****************************************************************************
 * Name: up_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method
 *   is called when the serial port is opened.  Normally, this is just after
 *   the the setup() method is called, however, the serial console may
 *   operate in a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled by the attach method (unless the
 *   hardware supports multiple levels of interrupt enabling).  The RX and TX
 *   interrupts are not enabled until the txint() and rxint() are called.
 *
 ****************************************************************************/

static int up_attach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  int ret;

  /* Initialize interrupt generation on the peripheral */

  putreg32(getreg32(priv->uartbase + UART_EV_PENDING_OFFSET), \
                    priv->uartbase + UART_EV_PENDING_OFFSET);
  up_serialout(priv, UART_EV_ENABLE_OFFSET, UART_EV_TX | UART_EV_RX);

  ret = irq_attach(priv->irq, up_interrupt, dev);

  if (ret == OK)
    {
      /* Enable the interrupt (RX and TX interrupts are still disabled
       * in the UART
       */

      up_enable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: up_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void up_detach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Disable interrupts */

  up_disable_irq(priv->irq);

  /* Detach from the interrupt */

  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: up_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq'  It should call uart_transmitchars or
 *   uart_receivechar to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'irq' number into the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int up_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct up_dev_s   *priv;
  uint32_t           status;
  int                passes;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (struct up_dev_s *)dev->priv;

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  for (passes = 0; passes < 256; passes++)
    {
      /* Retrieve interrupt pending status */

      status = up_serialin(priv, UART_EV_PENDING_OFFSET);

      if (status == 0)
        {
          break;
        }

      if (status & UART_EV_RX)
        {
          /* Process incoming bytes */

          uart_recvchars(dev);
        }

      if (status & UART_EV_TX)
        {
          /* Process outgoing bytes */

          putreg32(UART_EV_TX, priv->uartbase + UART_EV_PENDING_OFFSET);
          uart_xmitchars(dev);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: up_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int up_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: up_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int up_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  int rxdata;

  /* Return status information */

  if (status)
    {
      *status = 0; /* We are not yet tracking serial errors */
    }

  rxdata = (int)(up_serialin(priv, UART_RXTX_OFFSET));
  putreg32(UART_EV_RX, priv->uartbase + UART_EV_PENDING_OFFSET);

  return rxdata;
}

/****************************************************************************
 * Name: up_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void up_rxint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  irqstate_t flags = enter_critical_section();

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->im |= UART_EV_RX;
#endif
    }
  else
    {
      priv->im &= ~UART_EV_RX;
    }

  putreg32(getreg32(priv->uartbase + UART_EV_PENDING_OFFSET), \
                    priv->uartbase + UART_EV_PENDING_OFFSET);
  up_serialout(priv, UART_EV_ENABLE_OFFSET, priv->im);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

static bool up_rxavailable(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Return true is data is available in the receive data buffer */

  uint32_t rxempty = up_serialin(priv, UART_RXEMPTY_OFFSET);

  return rxempty == 0;
}

/****************************************************************************
 * Name: up_send
 *
 * Description:
 *   This method will send one byte on the UART.
 *
 ****************************************************************************/

static void up_send(struct uart_dev_s *dev, int ch)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  up_serialout(priv, UART_RXTX_OFFSET, (uint32_t)ch);
}

/****************************************************************************
 * Name: up_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void up_txint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();
  if (enable)
    {
      /* Enable the TX interrupt */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->im |= UART_EV_TX;

      putreg32(getreg32(priv->uartbase + UART_EV_PENDING_OFFSET), \
                        priv->uartbase + UART_EV_PENDING_OFFSET);
      up_serialout(priv, UART_EV_ENABLE_OFFSET, priv->im);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
#endif
    }
  else
    {
      /* Disable the TX interrupt */

      priv->im &= ~UART_EV_TX;

      putreg32(getreg32(priv->uartbase + UART_EV_PENDING_OFFSET), \
                        priv->uartbase + UART_EV_PENDING_OFFSET);
      up_serialout(priv, UART_EV_ENABLE_OFFSET, priv->im);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return true if the tranmsit data register is not full
 *
 ****************************************************************************/

static bool up_txready(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Return TRUE if the TX FIFO is not full */

  return (up_serialin(priv, UART_TXFULL_OFFSET)) == 0;
}

/****************************************************************************
 * Name: up_txempty
 *
 * Description:
 *   Return true if the tranmsit data register is empty
 *
 ****************************************************************************/

static bool up_txempty(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Return TRUE if the TX is pending */

  return (up_serialin(priv, UART_EV_PENDING_OFFSET) & UART_EV_TX) == 1;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT

/****************************************************************************
 * Name: riscv_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before riscv_serialinit.  NOTE:  This function depends on GPIO pin
 *   configuration performed in up_consoleinit() and main clock
 *   initialization performed in up_clkinitialize().
 *
 ****************************************************************************/

void riscv_earlyserialinit(void)
{
  /* Disable interrupts from all UARTS.  The console is enabled in
   * sty32c2_consoleinit().
   */

#ifdef CONFIG_STY32C2_UART0
  up_disableuartint(g_uart0port.priv, NULL);
#endif
#ifdef CONFIG_STY32C2_UART1
  up_disableuartint(g_uart1port.priv, NULL);
#endif
#ifdef CONFIG_STY32C2_UART2
  up_disableuartint(g_uart2port.priv, NULL);
#endif
#ifdef CONFIG_STY32C2_UART3
  up_disableuartint(g_uart3port.priv, NULL);
#endif
#ifdef CONFIG_STY32C2_UART4
  up_disableuartint(g_uart4port.priv, NULL);
#endif
#ifdef CONFIG_STY32C2_UART5
  up_disableuartint(g_uart5port.priv, NULL);
#endif
#ifdef CONFIG_STY32C2_UART6
  up_disableuartint(g_uart6port.priv, NULL);
#endif

  /* Configuration whichever one is the console */

#ifdef HAVE_SERIAL_CONSOLE
  CONSOLE_DEV.isconsole = true;
  up_setup(&CONSOLE_DEV);
#endif
}
#endif

/****************************************************************************
 * Name: riscv_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that riscv_earlyserialinit was called previously.
 *
 ****************************************************************************/

void riscv_serialinit(void)
{
  int i;
  int nuart;

  char devpath[16] = "/dev/ttyS";

  /* Device NO. */

  int devno = 1;

  const struct sty32c2_tty_s  ttydevs[] =
  {
#ifdef CONFIG_STY32C2_UART0
    {
      .dev = &g_uart0port,
      .idx =  g_uart0port.isconsole ? 0 : devno ++,
    },
#endif
#ifdef CONFIG_STY32C2_UART1
    {
      .dev = &g_uart1port,
      .idx =  g_uart1port.isconsole ? 0 : devno ++,
    },
#endif
#ifdef CONFIG_STY32C2_UART2
    {
      .dev = &g_uart2port,
      .idx =  g_uart2port.isconsole ? 0 : devno ++,
    },
#endif
#ifdef CONFIG_STY32C2_UART3
    {
      .dev = &g_uart3port,
      .idx =  g_uart3port.isconsole ? 0 : devno ++,
    },
#endif
#ifdef CONFIG_STY32C2_UART4
    {
      .dev = &g_uart4port,
      .idx =  g_uart4port.isconsole ? 0 : devno ++,
    },
#endif
#ifdef CONFIG_STY32C2_UART5
    {
      .dev = &g_uart5port,
      .idx =  g_uart5port.isconsole ? 0 : devno ++,
    },
#endif
#ifdef CONFIG_STY32C2_UART6
    {
      .dev = &g_uart6port,
      .idx =  g_uart6port.isconsole ? 0 : devno ++,
    },
#endif

  /* Place a dummy One as a Place holder to avoid uartdevs
   * to be empty when All above uart devices are undefined,
   * in which case a complier error will raise.
   */

    {
      .dev = NULL,
      .idx = -1,
    },
  };

  nuart = (int)(sizeof(ttydevs) / sizeof(ttydevs[0]));

  /* Register the console */

#ifdef HAVE_SERIAL_CONSOLE
  uart_register("/dev/console", &CONSOLE_DEV);
#endif

  /* Register All Uarts */

  for (i = 0; i < nuart; ++i)
    {
      const struct sty32c2_tty_s * tty = &ttydevs[i];

      if (!tty->dev)
        {
          continue;
        }

      /* OS (NuttX) is primordial and so many resources are uninitialized
       * while we are in 'riscv_serialinit'. The high level C lib functions
       * may not work well. Codes such as the following
       * 'snprintf(devpath, "/dev/ttyS%d\n", devno)...'
       * would not work as expected.
       *
       * It is ok to complete the device path manually.
       */

      devno = tty->idx;

      if (devno < 10)
        {
          devpath[9] = devno + '0';

          /* Terminate the String */

          devpath[10] = '\0';
        }
      else
        {
          /* There is one pre-condition that devno doesn't exceed 100 */

          int d = devno / 10;
          devpath[9] = d + '0';

          d = devno - d * 10;
          devpath[10] = d + '0';

          /* Terminate the String */

          devpath[11] = '\0';
        }

      uart_register(devpath, tty->dev);
    }
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug  writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  struct up_dev_s *priv = (struct up_dev_s *)CONSOLE_DEV.priv;
  uint8_t imr;

  up_disableuartint(priv, &imr);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      riscv_lowputc('\r');
    }

  riscv_lowputc(ch);
  up_restoreuartint(priv, imr);
#endif
  return ch;
}

#ifdef HAVE_SERIAL_CONSOLE
/****************************************************************************
 * Name: sty32c2_console_uart_setup
 ****************************************************************************/

void sty32c2_console_uart_setup(void)
{
#ifdef CONSOLE_DEV
  up_setup(&CONSOLE_DEV);
#endif
}

/****************************************************************************
 * Name: sty32c2_console_uart_putc
 ****************************************************************************/

void sty32c2_console_uart_putc(char ch)
{
#ifdef CONSOLE_DEV
  struct up_dev_s *priv = (struct up_dev_s *)CONSOLE_DEV.priv;

  /* Wait until the TX data register is empty */
  while (getreg8(priv->uartbase + UART_TXFULL_OFFSET));

  /* Then send the character */

  putreg8(ch, priv->uartbase + UART_RXTX_OFFSET);
  putreg8(UART_EV_TX, priv->uartbase + UART_EV_PENDING_OFFSET);
#endif
}
#endif /* HAVE_SERIAL_CONSOLE */

/****************************************************************************
 * Name: riscv_earlyserialinit, riscv_serialinit, and up_putc
 *
 * Description:
 *   stubs that may be needed.  These stubs would be used if all UARTs are
 *   disabled.  In that case, the logic in common/up_initialize() is not
 *   smart enough to know that there are not UARTs and will still expect
 *   these interfaces to be provided.
 *
 ****************************************************************************/

#else /* HAVE_UART_DEVICE */
void riscv_earlyserialinit(void)
{
}

void riscv_serialinit(void)
{
}

int up_putc(int ch)
{
  return ch;
}

#endif /* HAVE_UART_DEVICE */
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
#ifdef HAVE_SERIAL_CONSOLE
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      riscv_lowputc('\r');
    }

  riscv_lowputc(ch);
#endif
  return ch;
}

#endif /* USE_SERIALDRIVER */
