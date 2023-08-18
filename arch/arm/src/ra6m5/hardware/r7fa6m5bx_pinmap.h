/****************************************************************************
 * arch/arm/src/ra6m5/hardware/r7fa6m5bx_pinmap.h
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

#ifndef __ARCH_ARM_SRC_RA6M5_HARDWARE_R7FA6M5BX_PINMAP_H
#define __ARCH_ARM_SRC_RA6M5_HARDWARE_R7FA6M5BX_PINMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SCI0 - Serial Communications Interface */

#define GPIO_SCI0_TX_1      (GPIO_ALT|GPIO_AF4|GPIO_PORT1|GPIO_PIN1)
#define GPIO_SCI0_TX_2      (GPIO_ALT|GPIO_AF4|GPIO_PORT4|GPIO_PIN11)
#define GPIO_SCI0_RX_1      (GPIO_ALT|GPIO_AF4|GPIO_PORT1|GPIO_PIN0)
#define GPIO_SCI0_RX_2      (GPIO_ALT|GPIO_AF4|GPIO_PORT4|GPIO_PIN10)

/* SCI1 - Serial Communications Interface */

#define GPIO_SCI1_TX_1      (GPIO_ALT|GPIO_AF5|GPIO_PORT2|GPIO_PIN13)
#define GPIO_SCI1_RX_1      (GPIO_ALT|GPIO_AF5|GPIO_PORT2|GPIO_PIN12)
#define GPIO_SCI1_RX_2      (GPIO_ALT|GPIO_AF5|GPIO_PORT7|GPIO_PIN8)

/* SCI2 - Serial Communications Interface */

#define GPIO_SCI2_TX_1      (GPIO_ALT|GPIO_AF4|GPIO_PORT1|GPIO_PIN12)
#define GPIO_SCI2_TX_2      (GPIO_ALT|GPIO_AF4|GPIO_PORT3|GPIO_PIN2)
#define GPIO_SCI2_RX_1      (GPIO_ALT|GPIO_AF4|GPIO_PORT1|GPIO_PIN13)
#define GPIO_SCI2_RX_2      (GPIO_ALT|GPIO_AF4|GPIO_PORT3|GPIO_PIN1)

/* SCI3 - Serial Communications Interface */

#define GPIO_SCI3_TX_1      (GPIO_ALT|GPIO_AF5|GPIO_PORT4|GPIO_PIN9)
#define GPIO_SCI3_RX_1      (GPIO_ALT|GPIO_AF5|GPIO_PORT4|GPIO_PIN8)

/* SCI4 - Serial Communications Interface */

#define GPIO_SCI4_TX_1      (GPIO_ALT|GPIO_AF4|GPIO_PORT2|GPIO_PIN5)
#define GPIO_SCI4_TX_2      (GPIO_ALT|GPIO_AF4|GPIO_PORT2|GPIO_PIN7)
#define GPIO_SCI4_RX_1      (GPIO_ALT|GPIO_AF4|GPIO_PORT2|GPIO_PIN6)

/* SCI5 - Serial Communications Interface */

#define GPIO_SCI5_TX_1      (GPIO_ALT|GPIO_AF5|GPIO_PORT5|GPIO_PIN1)
#define GPIO_SCI5_RX_1      (GPIO_ALT|GPIO_AF5|GPIO_PORT5|GPIO_PIN2)

/* SCI6 - Serial Communications Interface */

#define GPIO_SCI6_SDA_1     (GPIO_ALT|GPIO_AF4|GPIO_PORT3|GPIO_PIN5)
#define GPIO_SCI6_SCL_1     (GPIO_ALT|GPIO_AF4|GPIO_PORT3|GPIO_PIN4)

/* SCI7 - Serial Communications Interface */

#define GPIO_SCI7_TX_1      (GPIO_ALT|GPIO_AF5|GPIO_PORT4|GPIO_PIN1)
#define GPIO_SCI7_RX_1      (GPIO_ALT|GPIO_AF5|GPIO_PORT4|GPIO_PIN2)

/* SCI8 - Serial Communications Interface */

#define GPIO_SCI8_TX_1      (GPIO_ALT|GPIO_AF4|GPIO_PORT1|GPIO_PIN5)
#define GPIO_SCI8_RX_1      (GPIO_ALT|GPIO_AF4|GPIO_PORT1|GPIO_PIN4)

/* SCI9 - Serial Communications Interface */

#define GPIO_SCI9_TX_1      (GPIO_ALT|GPIO_AF5|GPIO_PORT1|GPIO_PIN9)
#define GPIO_SCI9_TX_2      (GPIO_ALT|GPIO_AF5|GPIO_PORT6|GPIO_PIN2)
#define GPIO_SCI9_RX_1      (GPIO_ALT|GPIO_AF5|GPIO_PORT1|GPIO_PIN10)
#define GPIO_SCI9_RX_2      (GPIO_ALT|GPIO_AF5|GPIO_PORT6|GPIO_PIN1)
#define GPIO_SCI9_CK_1      (GPIO_ALT|GPIO_AF5|GPIO_PORT1|GPIO_PIN11)
#define GPIO_SCI9_CK_2      (GPIO_ALT|GPIO_AF5|GPIO_PORT6|GPIO_PIN0)

/* IIC1 - Serial Communications Interface */

#define GPIO_IIC1_SDA_1     (GPIO_ALT|GPIO_AF7|GPIO_PORT2|GPIO_PIN6)
#define GPIO_IIC1_SCL_1     (GPIO_ALT|GPIO_AF7|GPIO_PORT2|GPIO_PIN5)

/* USBFS -  Full-Speed Module */

#define GPIO_USBFS_VBUS_1   (GPIO_ALT|GPIO_AF19|GPIO_PORT4|GPIO_PIN7)

#endif /* __ARCH_ARM_SRC_RA6M5_HARDWARE_R7FA6M5BX_PINMAP_H */
