/****************************************************************************
 * arch/arm/src/ra6m5/ra6m5_lowputc.c
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

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"

#include "ra6m5.h"
#include "ra6m5_rcc.h"
#include "ra6m5_sci.h"
#include "ra6m5_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Select USART parameters for the selected console */

#ifdef HAVE_CONSOLE
#  if defined(CONFIG_SCI8_SERIAL_CONSOLE)
#    define RA6M5_CONSOLE_BASE      RA6M5_SCI8_BASE
#    define RA6M5_CONSOLE_BAUD      CONFIG_SCI8_BAUD
#    define RA6M5_CONSOLE_BITS      CONFIG_SCI8_BITS
#    define RA6M5_CONSOLE_PARITY    CONFIG_SCI8_PARITY
#    define RA6M5_CONSOLE_2STOP     CONFIG_SCI8_2STOP
#    define RA6M5_CONSOLE_TX        GPIO_SCI8_TX
#    define RA6M5_CONSOLE_RX        GPIO_SCI8_RX
#    ifdef CONFIG_SCI8_RS485
#      define RA6M5_CONSOLE_RS485_DIR GPIO_SCI8_RS485_DIR
#      if (CONFIG_SCI8_RS485_DIR_POLARITY == 0)
#        define RA6M5_CONSOLE_RS485_DIR_POLARITY false
#      else
#        define RA6M5_CONSOLE_RS485_DIR_POLARITY true
#      endif
#    endif
#  endif

/* Find the best BRR (bit rate register) value.
  *  In table g_async_baud, divisor values are stored for BGDM, ABCS, ABCSE and N values.  Each set of divisors
  *  is tried, and the settings with the lowest bit rate error are stored. The formula to calculate BRR is as
  *  follows and it must be 255 or less:
  *  BRR = (PCLK / (div_coefficient * baud)) - 1
  */

#define RA6M5_BRR_VALUE   (107)

#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/

void arm_lowputc(char ch)
{
#ifdef HAVE_CONSOLE
  uint8_t regval;

  /* Wait until the TX data register is empty */
  do {
    regval = getreg8(RA6M5_CONSOLE_BASE + RA6M5_SCI_SSR_OFFSET);
  } while (!(regval & SCI_SSR_TDRE));
  
#ifdef RA6M5_CONSOLE_RS485_DIR
  ra6m5_gpiowrite(RA6M5_CONSOLE_RS485_DIR,
                    RA6M5_CONSOLE_RS485_DIR_POLARITY);
#endif

  /* Then send the character */
  putreg8(ch, RA6M5_CONSOLE_BASE + RA6M5_SCI_TDR_OFFSET);

#ifdef RA6M5_CONSOLE_RS485_DIR
  /* Wait until the TX data complete */
  do {
    regval = getreg8(RA6M5_CONSOLE_BASE + RA6M5_SCI_SSR_OFFSET);
  } while (!(regval & SCI_SSR_TEND));

  ra6m5_gpiowrite(RA6M5_CONSOLE_RS485_DIR,
                    !RA6M5_CONSOLE_RS485_DIR_POLARITY);
#endif

#endif /* HAVE_CONSOLE */
}

/****************************************************************************
 * Name: ra6m5_lowsetup
 *
 * Description:
 *   This performs basic initialization of the USART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/

void ra6m5_lowsetup(void)
{
#if defined(HAVE_UART)
#if defined(HAVE_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)
  uint32_t regval;
#endif

#if defined(HAVE_CONSOLE)
  /* Enable SCI Peripheral */
  modifyreg32(RA6M5_MSTP_REG(RA6M5_MSTP_MSTPCRB_OFFSET), RA6M5_SCI_STOP(CONSOLE_UART), 0);
#endif

  /* Enable the console SCI and configure GPIO pins needed for rx/tx.
   *
   * NOTE: Clocking for selected SCIs was already provided in
   * ra6m5_rcc.c
   */

#ifdef RA6M5_CONSOLE_TX
  ra6m5_configgpio(RA6M5_CONSOLE_TX);
#endif
#ifdef RA6M5_CONSOLE_RX
  ra6m5_configgpio(RA6M5_CONSOLE_RX);
#endif

#ifdef RA6M5_CONSOLE_RS485_DIR
  ra6m5_configgpio(RA6M5_CONSOLE_RS485_DIR);
  ra6m5_gpiowrite(RA6M5_CONSOLE_RS485_DIR,
                    !RA6M5_CONSOLE_RS485_DIR_POLARITY);
#endif

  /* Enable and configure the selected console device */

#if defined(HAVE_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)

  /* Configure SEMR 
   * BGDM  = 1
   * ABCS  = 1
   * ABCSE = 0
  */
  regval = 0x50;
  putreg8((uint8_t)regval, RA6M5_CONSOLE_BASE + RA6M5_SCI_SEMR_OFFSET);

  /* Configure SCMR */
  regval = 0xf2;
  putreg8((uint8_t)regval, RA6M5_CONSOLE_BASE + RA6M5_SCI_SCMR_OFFSET);

  /* Configure SMR */
  regval = 0x00;
  putreg8((uint8_t)regval, RA6M5_CONSOLE_BASE + RA6M5_SCI_SMR_OFFSET);

  /* Configure the SCI Baud Rate */
  putreg8(RA6M5_BRR_VALUE,
           RA6M5_CONSOLE_BASE + RA6M5_SCI_BRR_OFFSET);

  /* Enable Rx, Tx, and the USART */
  regval = SCI_SCR_RE | SCI_SCR_TE;
  putreg8(regval, RA6M5_CONSOLE_BASE + RA6M5_SCI_SCR_OFFSET);

#endif /* HAVE_CONSOLE && !CONFIG_SUPPRESS_UART_CONFIG */
#endif /* HAVE_UART */
}
