/****************************************************************************
 * arch/arm/src/ra6m5/ra6m5_agt0.h
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

#ifndef __ARCH_ARM_SRC_RA6M5_RA6M5_AGT0_H
#define __ARCH_ARM_SRC_RA6M5_RA6M5_AGT0_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include "arm_internal.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ra6m5_agt0_txpoll   1
#define ra6m5_agt0_timeout  2


/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ra6m5_agt0_create
 *
 * Description:
 *   Initializes AGT0 Timer
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void ra6m5_agt0_create(uint32_t txpoll_time, uint32_t txtimeout_time);

/****************************************************************************
 * Name: ra6m5_agt0_start
 *
 * Description:
 *   Start CMTW0 Timer
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void ra6m5_agt0_start(uint8_t type, uint32_t timeout);

/****************************************************************************
 * Name: ra6m5_agt0_stop
 *
 * Description:
 *   Stop AGT0 Timer
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void ra6m5_agt0_stop(uint8_t type);

#endif /* __ARCH_ARM_SRC_RA6M5_RA6M5_AGT0_H */
