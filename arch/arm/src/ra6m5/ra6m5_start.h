/****************************************************************************
 * arch/arm/src/ra6m5/ra6m5_start.h
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

#ifndef __ARCH_ARM_SRC_RA6M5_RA6M5_START_H
#define __ARCH_ARM_SRC_RA6M5_RA6M5_START_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ra6m5_board_initialize
 *
 * Description:
 *   All RA6M5 architectures must provide the following entry point.  This
 *   entry point is called early in the initialization -- after all memory
 *   has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void ra6m5_board_initialize(void);

#endif /* __ARCH_ARM_SRC_RA6M5_RA6M5_START_H */
