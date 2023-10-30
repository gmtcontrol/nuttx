/****************************************************************************
 * arch/arm/src/ra6m5/ra6m5_gpio.c
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
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>
#include <arch/ra6m5/chip.h>

#include "arm_internal.h"
#include "chip.h"
#include "ra6m5_gpio.h"

#include "hardware/ra6m5_syscfg.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Base addresses for each GPIO port */

const uint32_t g_portbase[RA6M5_NPORTS] =
{
#if RA6M5_NPORTS > 0
  RA6M5_PFS0_BASE,
#endif
#if RA6M5_NPORTS > 1
  RA6M5_PFS1_BASE,
#endif
#if RA6M5_NPORTS > 2
  RA6M5_PFS2_BASE,
#endif
#if RA6M5_NPORTS > 3
  RA6M5_PFS3_BASE,
#endif
#if RA6M5_NPORTS > 4
  RA6M5_PFS4_BASE,
#endif
#if RA6M5_NPORTS > 5
  RA6M5_PFS5_BASE,
#endif
#if RA6M5_NPORTS > 6
  RA6M5_PFS6_BASE,
#endif
#if RA6M5_NPORTS > 7
  RA6M5_PFS7_BASE,
#endif
#if RA6M5_NPORTS > 8
  RA6M5_PFS8_BASE,
#endif
#if RA6M5_NPORTS > 9
  RA6M5_PFS9_BASE,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  ra6m5_gpioinit
 *
 * Description:
 *   Based on configuration within the .config file, it does:
 *    - Remaps positions of alternative functions.
 *
 *   Typically called from ra6m5_start().
 *
 * Assumptions:
 *   This function is called early in the initialization sequence so that
 *   no mutual exclusion is necessary.
 *
 ****************************************************************************/

void ra6m5_gpioinit(void)
{
  /* If this is first entry then allow writing of PFS. */

  /* writing to PFSWE bit enabled */
  putreg8(0, RA6M5_PMISC_REG(RA6M5_PMISC_PWPR_OFFSET));

  /* writing to PFS register enabled */
  putreg8(1 << 6, RA6M5_PMISC_REG(RA6M5_PMISC_PWPR_OFFSET));
}

/****************************************************************************
 * Name: ra6m5_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *   Once it is configured as Alternative (GPIO_ALT|GPIO_CNF_AFPP|...)
 *   function, it must be unconfigured with ra6m5_unconfiggpio() with
 *   the same cfgset first before it can be set to non-alternative function.
 *
 * Returned Value:
 *   OK on success
 *   A negated errno value on invalid port, or when pin is locked as ALT
 *   function.
 *
 * To-Do: Auto Power Enable
 ****************************************************************************/

int ra6m5_configgpio(uint32_t cfgset)
{
  uintptr_t base;
  uint32_t regval;
  uint32_t setting;
  unsigned int port;
  unsigned int pin;
  unsigned int pinmode;
  irqstate_t flags;

  /* Verify that this hardware supports the select GPIO port */

  port = (cfgset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port >= RA6M5_NPORTS)
    {
      return -EINVAL;
    }

  /* Get the port base address */

  base = g_portbase[port];

  /* Get the pin number and select the port configuration register for that
   * pin
   */

  pin = (cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  /* Set up the mode register (and remember whether the pin mode) */

  switch (cfgset & GPIO_MODE_MASK)
    {
      default:
      case GPIO_INPUT:      /* Input mode */
        pinmode = IOPORT_CFG_PORT_DIRECTION_INPUT;
        break;

      case GPIO_OUTPUT:     /* General purpose output mode */
        pinmode = IOPORT_CFG_PORT_DIRECTION_OUTPUT;

        /* Set the initial output value */

        if ((cfgset & GPIO_OUTPUT_SET) != 0) {
          pinmode |= IOPORT_CFG_PORT_OUTPUT_HIGH;
        }
        break;

      case GPIO_ALT:        /* Alternate function mode */
        pinmode = IOPORT_CFG_PERIPHERAL_PIN;
        break;

      case GPIO_ANALOG:     /* Analog mode */
        pinmode = IOPORT_CFG_ANALOG_ENABLE;
        break;
    }

  /* Interrupts must be disabled from here on out so that we have mutually
   * exclusive access to all of the GPIO configuration registers.
   */

  flags = enter_critical_section();

  /* Get the current register value */

  regval  = getreg32(base + RA6M5_PFS_LOC(pin));

  /* Now apply the configuration to the mode register */

  regval &= GPIO_PRV_PIN_WRITE_MASK;
  regval |= (uint32_t)pinmode;

  /* Set up the pull-up/pull-down configuration (all but analog pins) */

  setting = 0;
  if (pinmode != IOPORT_CFG_ANALOG_ENABLE)
    {
      switch (cfgset & GPIO_PUPD_MASK)
        {
          default:
          case GPIO_FLOAT:      /* No pull-up, pull-down */
          case GPIO_PULLDOWN:   /* Pull-down */
            break;

          case GPIO_PULLUP:     /* Pull-up */
            setting = IOPORT_CFG_PULLUP_ENABLE;
            break;
        }
    }

  regval |= setting;

  /* Set the alternate function (Only alternate function pins) */
  
  setting = 0;
  if (pinmode == IOPORT_CFG_PERIPHERAL_PIN)
    {
      setting = (cfgset & GPIO_AF_MASK);
    }

  regval |= setting;

  /* Set speed (Only outputs and alternate function pins) */

  setting = 0;
  if (pinmode == IOPORT_CFG_PORT_DIRECTION_OUTPUT || pinmode == IOPORT_CFG_PERIPHERAL_PIN)
    {
      switch (cfgset & GPIO_SPEED_MASK)
        {
          default:
          case GPIO_SPEED_2MHZ:    /* 2 MHz Low speed output */
          case GPIO_SPEED_25MHZ:   /* 25 MHz Medium speed output */
          case GPIO_SPEED_50MHZ:   /* 50 MHz High speed output  */
            break;

          case GPIO_SPEED_100MHZ:   /* 100 MHz Very High speed output */
            setting = IOPORT_CFG_DRIVE_HS_HIGH;
            break;
        }
    }

  regval |= setting;

  /* Set push-pull/open-drain (Only outputs and alternate function pins) */

  setting = 0;
  if ((pinmode == IOPORT_CFG_PORT_DIRECTION_OUTPUT || pinmode == IOPORT_CFG_PERIPHERAL_PIN) &&
      (cfgset & GPIO_OPENDRAIN) != 0)
    {
      setting = IOPORT_CFG_NMOS_ENABLE;
    }

  regval |= setting;

  /* Save the modified register value */

  putreg32(regval, base + RA6M5_PFS_LOC(pin));

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: ra6m5_unconfiggpio
 *
 * Description:
 *   Unconfigure a GPIO pin based on bit-encoded description of the pin, set
 *   it into default HiZ state (and possibly mark it's unused) and unlock it
 *   whether it was previously selected as alternative function
 *   (GPIO_ALT|GPIO_CNF_AFPP|...).
 *
 *   This is a safety function and prevents hardware from shocks, as
 *   unexpected write to the Timer Channel Output GPIO to fixed '1' or '0'
 *   while it should operate in PWM mode could produce excessive on-board
 *   currents and trigger over-current/alarm function.
 *
 * Returned Value:
 *  OK on success
 *  A negated errno value on invalid port
 *
 * To-Do: Auto Power Disable
 ****************************************************************************/

int ra6m5_unconfiggpio(uint32_t cfgset)
{
  /* Reuse port and pin number and set it to default HiZ INPUT */

  cfgset &= GPIO_PORT_MASK | GPIO_PIN_MASK;
  cfgset |= GPIO_INPUT | GPIO_FLOAT;

  return ra6m5_configgpio(cfgset);
}

/****************************************************************************
 * Name: ra6m5_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void ra6m5_gpiowrite(uint32_t pinset, bool value)
{
  uint32_t pfs_bits;
  uint32_t base;
  uint32_t bit;
  unsigned int port;
  unsigned int pin;

  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port < RA6M5_NPORTS)
    {
      /* Get the port base address */

      base = g_portbase[port];

      /* Get the pin number  */

      pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

      /* Clear PMR, ASEL, ISEL and PODR bits. */

      pfs_bits  = getreg32(base + RA6M5_PFS_LOC(pin));
      pfs_bits &= GPIO_PRV_PIN_WRITE_MASK;

      /* Set or clear the output on the pin */

      bit = pfs_bits | (uint32_t)value;

      putreg32(bit | GPIO_PFS_PDR_OUTPUT, base + RA6M5_PFS_LOC(pin));
    }
}

/****************************************************************************
 * Name: ra6m5_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool ra6m5_gpioread(uint32_t pinset)
{
  uint32_t base;
  unsigned int port;
  unsigned int pin;

  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port < RA6M5_NPORTS)
    {
      /* Get the port base address */

      base = g_portbase[port];

      /* Get the pin number and return the input state of that pin */

      pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
      return ((getreg32(base + RA6M5_PFS_LOC(pin)) & 2) != 0);
    }

  return 0;
}
