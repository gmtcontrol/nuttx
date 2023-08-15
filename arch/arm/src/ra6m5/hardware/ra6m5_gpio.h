/****************************************************************************
 * arch/arm/src/ra6m5/hardware/ra6m5_gpio.h
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

#ifndef __ARCH_ARM_SRC_RA6M5_HARDWARE_RA6M5_GPIO_H
#define __ARCH_ARM_SRC_RA6M5_HARDWARE_RA6M5_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/** Options to configure pin functions  */
enum
{
    IOPORT_CFG_PORT_DIRECTION_INPUT  = 0x00000000, ///< Sets the pin direction to input (default)
    IOPORT_CFG_PORT_DIRECTION_OUTPUT = 0x00000004, ///< Sets the pin direction to output
    IOPORT_CFG_PORT_OUTPUT_LOW       = 0x00000000, ///< Sets the pin level to low
    IOPORT_CFG_PORT_OUTPUT_HIGH      = 0x00000001, ///< Sets the pin level to high
    IOPORT_CFG_PULLUP_ENABLE         = 0x00000010, ///< Enables the pin's internal pull-up
    IOPORT_CFG_PIM_TTL               = 0x00000020, ///< Enables the pin's input mode
    IOPORT_CFG_NMOS_ENABLE           = 0x00000040, ///< Enables the pin's NMOS open-drain output
    IOPORT_CFG_PMOS_ENABLE           = 0x00000080, ///< Enables the pin's PMOS open-drain ouput
    IOPORT_CFG_DRIVE_MID             = 0x00000400, ///< Sets pin drive output to medium
    IOPORT_CFG_DRIVE_HS_HIGH         = 0x00000800, ///< Sets pin drive output to high along with supporting high speed
    IOPORT_CFG_DRIVE_MID_IIC         = 0x00000C00, ///< Sets pin to drive output needed for IIC on a 20mA port
    IOPORT_CFG_DRIVE_HIGH            = 0x00000C00, ///< Sets pin drive output to high
    IOPORT_CFG_EVENT_RISING_EDGE     = 0x00001000, ///< Sets pin event trigger to rising edge
    IOPORT_CFG_EVENT_FALLING_EDGE    = 0x00002000, ///< Sets pin event trigger to falling edge
    IOPORT_CFG_EVENT_BOTH_EDGES      = 0x00003000, ///< Sets pin event trigger to both edges
    IOPORT_CFG_IRQ_ENABLE            = 0x00004000, ///< Sets pin as an IRQ pin
    IOPORT_CFG_ANALOG_ENABLE         = 0x00008000, ///< Enables pin to operate as an analog pin
    IOPORT_CFG_PERIPHERAL_PIN        = 0x00010000  ///< Enables pin to operate as a peripheral pin
};

/* Helper Macros ************************************************************/

#define RA6M5_PFS_LOC(pin)              ((pin) * 4)  /* Pin Function Control Register */
#define RA6M5_PMISC_REG(r)              (RA6M5_PMISC_BASE+(r))

/* Register Offsets *********************************************************/

#define RA6M5_PMISC_PWPR_OFFSET         0x0003      /* Write-Protect Register */
#define RA6M5_PMISC_PWPRS_OFFSET        0x0005      /* Write-Protect Register for Secure */

/* Register Bitfield Definitions ********************************************/

/* GPIO port mode register */

#define GPIO_PFS_PDR_OUTPUT         (4U)
#define GPIO_PRV_PIN_WRITE_MASK     (0xFFFE3FFE)

#endif /* __ARCH_ARM_SRC_RA6M5_HARDWARE_RA6M5_GPIO_H */
