/****************************************************************************
 * arch/renesas/src/ra6m5/ra6m5_iic.h
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

#ifndef __ARCH_RENESAS_SRC_RA6M5_RA6M5_IIC_H
#define __ARCH_RENESAS_SRC_RA6M5_RA6M5_IIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>

#include "arm_internal.h"
#include "chip.h"

#if defined(CONFIG_RA6M5_R7FA6M5BX)
#  include "hardware/r7fa6m5bx_iic.h"
#else
#  error "Unsupported RA6M5 chip"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SCL_RISE_TIME_FASTPLUS  120E-9
#define SCL_FALL_TIME_FASTPLUS  120E-9
#define SCL_RISE_TIME_FAST      300E-9
#define SCL_FALL_TIME_FAST      300E-9
#define SCL_RISE_TIME_STANDARD  1000E-9
#define SCL_FALL_TIME_STANDARD  300E-9
#define RIIC_MAX_DIV            0x08
#define RIIC_CALC_MAX           32
#define RIIC_INTERRUPT_PRIO     (0x0f)
#define RIIC_RATE_CALC          (0xff)

#define RIIC_SUCCESS            0 /* Successful operation */
#define RIIC_ERR_NO_INIT        1 /* Uninitialized state */
#define RIIC_ERR_BUS_BUSY       2 /* Channel is on communication. */
#define RIIC_ERR_AL             3 /* Arbitration lost error */
#define RIIC_ERR_TMO            4 /* Time Out error */
#define RIIC_ERR_NACK           5 /* NACK reception */
#define RIIC_ERR_OTHER          6 /* Other error */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ra6m5_i2cbus_initialize
 *
 * Description:
 *   Initialize the selected RIIC channel. And return a unique instance of
 *   struct i2c_master_s.  This function may be called to obtain multiple
 *   instances of the interface, each of which may be set up with a
 *   different frequency and slave address.
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple I2C interfaces)
 *
 * Returned Value:
 *   Valid I2C device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct i2c_master_s *ra6m5_i2cbus_initialize(int channel);

/****************************************************************************
 * Name: ra6m5_i2cbus_uninitialize
 *
 * Description:
 *   De-initialize the selected I2C port, and power down the device.
 *
 * Input Parameters:
 *   Device structure as returned by the ra6m5_i2cbus_initialize()
 *
 * Returned Value:
 *   OK on success, ERROR when internal reference count mismatch or dev
 *   points to invalid hardware device.
 *
 ****************************************************************************/

int ra6m5_i2cbus_uninitialize(struct i2c_master_s *dev);

#endif /* __ARCH_RENESAS_SRC_RA6M5_RA6M5_RIIC_H */
