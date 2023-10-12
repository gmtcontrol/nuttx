/****************************************************************************
 * arch/risc-v/src/sty32c2/sty32c2_pwm.c
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
#include <nuttx/timers/pwm.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>

#include "riscv_internal.h"
#include "sty32c2.h"
#include "sty32c2_clockconfig.h"
#include "sty32c2_pwm.h"
#include "hardware/sty32c2_pwm.h"


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef MIN
#  define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* PWM channel configuration */

struct sty32c2_pwm_chan_s
{
  uint32_t period;                        /* Current duty */
  uint32_t duty;                          /* Current period */
};

/* This structure represents the state of one PWM device */

struct sty32c2_pwm_dev_s
{
  const struct pwm_ops_s    * ops;        /* PWM operations */
  const uint8_t               channels;   /* PWM Channels number */
  struct sty32c2_pwm_chan_s * chans;      /* PWM Channels pointer */
  uint32_t                    frequency;  /* PWM frequency */
  uint32_t                    period;     /* PWM frequency period */
  uint8_t                     pwmid;      /* PWM ID */
  uint8_t                     irq;        /* PWM update IRQ */
#ifdef CONFIG_PWM_PULSECOUNT
  uint32_t                    count;      /* Remaining pulse count */
  void                      * handle;     /* Handle used for upper-half callback */
#endif

};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/*  PWM management methods */

static int pwm_interrupt(int irq, void *context, void *arg);

/* PWM driver methods */

static int pwm_setup(struct pwm_lowerhalf_s *dev);
static int pwm_shutdown(struct pwm_lowerhalf_s *dev);
#ifdef CONFIG_PWM_PULSECOUNT
static int pwm_start(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info,
                     void *handle);
#endif
static int pwm_start(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info);
static int pwm_stop(struct pwm_lowerhalf_s *dev);
static int pwm_ioctl(struct pwm_lowerhalf_s *dev,
                     int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* PWM operations */

static const struct pwm_ops_s g_pwmops =
{
  .setup    = pwm_setup,
  .shutdown = pwm_shutdown,
  .start    = pwm_start,
  .stop     = pwm_stop,
  .ioctl    = pwm_ioctl,
};

/* PWM channels table */

static struct sty32c2_pwm_chan_s g_pwm0_chans[STY32C2_PWM_NCHANNELS];

/* PWM device #0 */

static struct sty32c2_pwm_dev_s g_sty32c2_pwm0 =
{
  .ops       = &g_pwmops,
  .channels  = STY32C2_PWM_NCHANNELS,
  .chans     = g_pwm0_chans,
  .frequency = 400,
  .pwmid     = 0,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq       = STY32C2_IRQ_PWM0,
#else
  .irq       = 0xff,
#endif  
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pwm_duty_update
 *
 * Description:
 *   Try to change only channel duty
 *
 * Input Parameters:
 *   dev     - A reference to the lower half PWM driver state structure
 *   channel - Channel to by updated
 *   duty    - New duty
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_duty_update(struct pwm_lowerhalf_s *dev, uint8_t channel,
                           ub16_t duty)
{
  struct sty32c2_pwm_dev_s *priv = (struct sty32c2_pwm_dev_s *)dev;

  DEBUGASSERT(priv != NULL);

  pwminfo("PWM%u channel: %u duty: %08" PRIx32 "\n",
          priv->pwmid, channel, duty);

#ifndef CONFIG_PWM_MULTICHAN
  DEBUGASSERT(channel == priv->channels[0].channel);
  DEBUGASSERT(duty >= 0 && duty < uitoub16(100));
#endif

  /* Duty cycle:
   *
   * duty cycle = (period / 65536) * duty
   */
  if (priv->chans[channel].duty != duty)
    {
      /* Update channel duty value */

      priv->chans[channel].duty = duty;

      /* Calculate "width" value according to duty  */

      uint32_t width = (uint32_t)((priv->chans[channel].period / 65536.0f) * duty);

      /* Write corresponding "width" register */

      putreg32(width, STY32C2_PWM_WIDTH_REG(channel));

      if (width > 0)
        {
          /* Enable the channel */

          if (getreg32(STY32C2_PWM_ENABLE_REG(channel)) == 0) 
            {
              putreg32(1, STY32C2_PWM_ENABLE_REG(channel));
            }
        } 
      else
        {
          /* Disable the channel */

          putreg32(0, STY32C2_PWM_ENABLE_REG(channel));
        }
    }

  return OK;
}

/****************************************************************************
 * Name: pwm_frequency_update
 *
 * Description:
 *   Update a PWM timer frequency
 *
 ****************************************************************************/

static int pwm_frequency_update(struct pwm_lowerhalf_s *dev,
                                uint32_t frequency)
{
  struct sty32c2_pwm_dev_s *priv = (struct sty32c2_pwm_dev_s *)dev;

  if (priv->frequency != frequency)
    {
      #ifdef CONFIG_PWM_NCHANNELS
      int channels = MIN(priv->channels, CONFIG_PWM_NCHANNELS);
      #else
      int channels = 1;
      #endif

      /* Update the PWM frequency */

      priv->frequency = frequency;

      /* Calculate the period value */

      priv->period = (frequency) ? (sty32c2_get_cpuclk() / frequency) : 0;

      /* Update all channels */

      for (int i = 0; i < channels; i++)
        {
          /* Update the channel configuration */

          priv->chans[i].period = priv->period;

          /* Update the channel value */

          putreg32(priv->period, STY32C2_PWM_PERIOD_REG(i));
        }
    }

  return OK;
}

/****************************************************************************
 * Name: pwm_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq'  It should call uart_transmitchars or
 *   uart_receivechar to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'irq' number into the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int pwm_interrupt(int irq, void *context, void *arg)
{
  struct sty32c2_pwm_dev_s *priv = (struct sty32c2_pwm_dev_s *)arg;
  uint32_t status;

  /* We don't want compilation warnings if no DEBUGASSERT */

  UNUSED(priv);

  DEBUGASSERT(priv != NULL);
 
  /* Retrieve interrupt pending status */

  status = getreg32(STY32C2_PWM_EV_PENDING_REG);

  if (status != 0)
    {
      /* Clear the pending bit */

      putreg32(status, STY32C2_PWM_EV_PENDING_REG);
    }

  return OK;
}

/****************************************************************************
 * Name: pwm_setup
 *
 * Description:
 *   This method is called when the driver is opened.  The lower half driver
 *   should configure and initialize the device so that it is ready for use.
 *   It should not, however, output pulses until the start method is called.
 *
 * Input parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ****************************************************************************/

static int pwm_setup(struct pwm_lowerhalf_s *dev)
{
  struct sty32c2_pwm_dev_s *priv = (struct sty32c2_pwm_dev_s *)dev;
#ifdef CONFIG_PWM_NCHANNELS
  int channels = MIN(priv->channels, CONFIG_PWM_NCHANNELS);
#else
  int channels = 1;
#endif
  uint32_t irq_mask = 0;

  /* Clear device configurations */

  pwm_frequency_update(dev, 0);

  /* Clear channel configurations */

  for (int i = 0; i < channels; i++)
    {

      /* Duty cycle configuration */

      pwm_duty_update(dev, i, 0);

      /* Prepare the PWM channels irq mask */

      irq_mask |=  (1 << i);
    }

    /* Initialize interrupt generation on the peripheral */

    if (priv->irq != 0xff) 
      {
        /* Enable PWM interrpts */

        putreg32(getreg32(STY32C2_PWM_EV_PENDING_REG), \
                          STY32C2_PWM_EV_PENDING_REG);
        putreg32(irq_mask, STY32C2_PWM_EV_ENABLE_REG);

        /* Attach the interrupt service */

        if (irq_attach(priv->irq, pwm_interrupt, priv) == OK)
          {
            /* Enable the interrupt */

            up_enable_irq(priv->irq);
          }
      }

  return OK;
}

/****************************************************************************
 * Name: pwm_shutdown
 *
 * Description:
 *   This method is called when the driver is closed.  The lower half driver
 *   stop pulsed output, free any resources, disable the timer hardware, and
 *   put the system into the lowest possible power usage state
 *
 * Input parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ****************************************************************************/

static int pwm_shutdown(struct pwm_lowerhalf_s *dev)
{
  struct sty32c2_pwm_dev_s *priv = (struct sty32c2_pwm_dev_s *)dev;
#ifdef CONFIG_PWM_NCHANNELS
  int channels = MIN(priv->channels, CONFIG_PWM_NCHANNELS);
#else
  int channels = 1;
#endif

  /* Stop PWM module */

  pwm_stop(dev);

  /* Clear channel configurations */

  for (int i = 0; i < channels; i++)
    {
      pwm_duty_update(dev, i, 0);
    }

    /* Disable interrupt generation on the peripheral */

    if (priv->irq != 0xff) 
      {
        putreg32(0, STY32C2_PWM_EV_ENABLE_REG);
        putreg32(getreg32(STY32C2_PWM_EV_PENDING_REG), \
                          STY32C2_PWM_EV_PENDING_REG);

        /* Disable interrupts */

        up_disable_irq(priv->irq);

        /* Detach from the interrupt */

        irq_detach(priv->irq);
      }

  return OK;
}

/****************************************************************************
 * Name: pwm_start
 *
 * Description:
 *   (Re-)initialize the timer resources and start the pulsed output
 *
 * Input parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *   info - A reference to the characteristics of the pulsed output
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ****************************************************************************/

static int pwm_start(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info)
{
  struct sty32c2_pwm_dev_s *priv = (struct sty32c2_pwm_dev_s *)dev;
#ifdef CONFIG_PWM_NCHANNELS
  int channels = MIN(priv->channels, CONFIG_PWM_NCHANNELS);
#else
  int channels = 1;
#endif
  uint32_t duty;

  /* Update timer with given PWM timer frequency */

  pwm_frequency_update(dev, info->frequency);

  /* Update timer with given PWM channel duty */

  for (int i = 0; i < channels; i++)
    {
#ifdef CONFIG_PWM_NCHANNELS
      duty = info->channels[i].duty;
#else
      duty = info[i].duty;
#endif
      pwm_duty_update(dev, i, duty);
    }

  return OK;
}

/****************************************************************************
 * Name: pwm_stop
 *
 * Description:
 *   Stop the pulsed output and reset the timer resources
 *
 * Input parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ****************************************************************************/

static int pwm_stop(struct pwm_lowerhalf_s *dev)
{
  struct sty32c2_pwm_dev_s *priv = (struct sty32c2_pwm_dev_s *)dev;
#ifdef CONFIG_PWM_NCHANNELS
  int channels = MIN(priv->channels, CONFIG_PWM_NCHANNELS);
#else
  int channels = 1;
#endif

  /* Disable interrupts momentary to stop any ongoing timer processing and
   * to prevent any concurrent access to the reset register.
   */

  irqstate_t flags = enter_critical_section();

  /* Stopped so frequency is zero */

  pwm_frequency_update(dev, 0);

  /* Clear pending event bits */

  putreg32(getreg32(STY32C2_PWM_EV_PENDING_REG), \
      STY32C2_PWM_EV_PENDING_REG);

  /* Disable the pwm and outputs */

  for (int i = 0; i < channels; i++)
    {
        /* Disable the pwm outputs */

        pwm_duty_update(dev, i, 0);
    }

  leave_critical_section(flags);


  return OK;
}

/****************************************************************************
 * Name: pwm_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 * Input parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *   cmd - The ioctl command
 *   arg - The argument accompanying the ioctl command
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ****************************************************************************/

static int pwm_ioctl(struct pwm_lowerhalf_s *dev, int cmd,
                     unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sty32c2_pwm_initialize
 *
 * Description:
 *   Initialize PWM channels for use with the upper_level PWM driver.
 *
 * Input Parameters:
 *   None. All channels initizlized by FPGA bit-stream
 *
 * Returned Value:
 *   On success, a pointer to the lower half PWM driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

struct pwm_lowerhalf_s *sty32c2_pwm_initialize(int pwm)
{
  pwminfo("PWM%u\n", pwm);

  if (pwm > 0)
    {
      pwmerr("ERROR: No such pwm configured %d\n", pwm);
      return NULL;
    }

  return (struct pwm_lowerhalf_s *)&g_sty32c2_pwm0;
}
