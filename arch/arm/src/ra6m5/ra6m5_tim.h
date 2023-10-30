/****************************************************************************
 * arch/arm/src/ra6m5/ra6m5_tim.h
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

#ifndef __ARCH_ARM_SRC_RA6M5_RA6M5_TIM_H
#define __ARCH_ARM_SRC_RA6M5_RA6M5_TIM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/ra6m5_tim.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Helpers ******************************************************************/

#define RA6M5_TIM_SETMODE(d,mode)       ((d)->ops->setmode(d,mode))
#define RA6M5_TIM_SETCLOCK(d,freq)      ((d)->ops->setclock(d,freq))
#define RA6M5_TIM_GETCLOCK(d)           ((d)->ops->getclock(d))
#define RA6M5_TIM_SETPERIOD(d,period)   ((d)->ops->setperiod(d,period))
#define RA6M5_TIM_GETPERIOD(d)          ((d)->ops->getperiod(d))
#define RA6M5_TIM_GETCOUNTER(d)         ((d)->ops->getcounter(d))
#define RA6M5_TIM_SETCHANNEL(d,ch,mode) ((d)->ops->setchannel(d,ch,mode))
#define RA6M5_TIM_SETCOMPARE(d,ch,comp) ((d)->ops->setcompare(d,ch,comp))
#define RA6M5_TIM_GETCAPTURE(d,ch)      ((d)->ops->getcapture(d,ch))
#define RA6M5_TIM_SETISR(d,hnd,arg,s)   ((d)->ops->setisr(d,hnd,arg,s))
#define RA6M5_TIM_ENABLEINT(d,s)        ((d)->ops->enableint(d,s))
#define RA6M5_TIM_DISABLEINT(d,s)       ((d)->ops->disableint(d,s))
#define RA6M5_TIM_ACKINT(d,s)           ((d)->ops->ackint(d,s))
#define RA6M5_TIM_CHECKINT(d,s)         ((d)->ops->checkint(d,s))

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* TIM Device Structure */

struct ra6m5_tim_dev_s
{
  struct ra6m5_tim_ops_s *ops;
};

/* TIM Modes of Operation */

enum ra6m5_tim_mode_e
{
  RA6M5_TIM_MODE_UNUSED       = -1,

  /* One of the following */

  RA6M5_TIM_MODE_MASK         = 0x0310,
  RA6M5_TIM_MODE_DISABLED     = 0x0000,
  RA6M5_TIM_MODE_UP           = 0x0100,
  RA6M5_TIM_MODE_DOWN         = 0x0110,
  RA6M5_TIM_MODE_UPDOWN       = 0x0200,
  RA6M5_TIM_MODE_PULSE        = 0x0300,

  /* One of the following */

  RA6M5_TIM_MODE_CK_INT       = 0x0000,
#if 0
  RA6M5_TIM_MODE_CK_INT_TRIG  = 0x0400,
  RA6M5_TIM_MODE_CK_EXT       = 0x0800,
  RA6M5_TIM_MODE_CK_EXT_TRIG  = 0x0c00,
#endif

  /* Clock sources, OR'ed with CK_EXT */

#if 0
  RA6M5_TIM_MODE_CK_CHINVALID = 0x0000,
  RA6M5_TIM_MODE_CK_CH1       = 0x0001,
  RA6M5_TIM_MODE_CK_CH2       = 0x0002,
  RA6M5_TIM_MODE_CK_CH3       = 0x0003,
  RA6M5_TIM_MODE_CK_CH4       = 0x0004
#endif

  /* Todo: external trigger block */
};

/* TIM Channel Modes */

enum ra6m5_tim_channel_e
{
  RA6M5_TIM_CH_DISABLED       = 0x00,

  /* Common configuration */

  RA6M5_TIM_CH_POLARITY_POS   = 0x00,
  RA6M5_TIM_CH_POLARITY_NEG   = 0x01,

  /* MODES: */

  RA6M5_TIM_CH_MODE_MASK      = 0x06,

  /* Output Compare Modes */

  RA6M5_TIM_CH_OUTPWM         = 0x04,  /* Enable standard PWM mode, active high when counter < compare */
#if 0
  RA6M5_TIM_CH_OUTCOMPARE     = 0x06,
#endif

  /* TODO other modes ... as PWM capture, ENCODER and Hall Sensor */
};

/* TIM Operations */

struct ra6m5_tim_ops_s
{
  /* Basic Timers */

  int  (*setmode)(struct ra6m5_tim_dev_s *dev,
                  enum ra6m5_tim_mode_e mode);
  int  (*setclock)(struct ra6m5_tim_dev_s *dev, uint32_t freq);
  uint32_t (*getclock)(struct ra6m5_tim_dev_s *dev);
  void (*setperiod)(struct ra6m5_tim_dev_s *dev, uint32_t period);
  uint32_t (*getperiod)(struct ra6m5_tim_dev_s *dev);
  uint32_t (*getcounter)(struct ra6m5_tim_dev_s *dev);

  /* General and Advanced Timers Adds */

  int  (*setchannel)(struct ra6m5_tim_dev_s *dev, uint8_t channel,
                     enum ra6m5_tim_channel_e mode);
  int  (*setcompare)(struct ra6m5_tim_dev_s *dev, uint8_t channel,
                     uint32_t compare);
  int  (*getcapture)(struct ra6m5_tim_dev_s *dev, uint8_t channel);

  /* Timer interrupts */

  int  (*setisr)(struct ra6m5_tim_dev_s *dev,
                 xcpt_t handler, void *arg, int source);
  void (*enableint)(struct ra6m5_tim_dev_s *dev, int source);
  void (*disableint)(struct ra6m5_tim_dev_s *dev, int source);
  void (*ackint)(struct ra6m5_tim_dev_s *dev, int source);
  int  (*checkint)(struct ra6m5_tim_dev_s *dev, int source);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Power-up timer and get its structure */

struct ra6m5_tim_dev_s *ra6m5_tim_init(int timer);

/* Power-down timer, mark it as unused */

int ra6m5_tim_deinit(struct ra6m5_tim_dev_s *dev);

/****************************************************************************
 * Name: ra6m5_timer_initialize
 *
 * Description:
 *   Bind the configuration timer to a timer lower half instance and
 *   register the timer drivers at 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the timer device.  This should be of the form
 *             /dev/timer0
 *   timer - the timer number.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_TIMER
int ra6m5_timer_initialize(const char *devpath, int timer);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RA6M5_RA6M5_TIM_H */
