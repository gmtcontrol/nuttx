/****************************************************************************
 * arch/renesas/src/ra6m5/ra6m5_iic.c
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

#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/kthread.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include <arch/board/board.h>

#include "chip.h"
#include "ra6m5_rcc.h"
#include "ra6m5_gpio.h"
#include "ra6m5_iic.h"


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_RA6M5_IIC0) || defined(CONFIG_RA6M5_IIC1) || \
    defined(CONFIG_RA6M5_IIC2)

#define BUS_CHECK_COUNTER         (1000)
#define RIIC_REG_INIT             (0x00)
#define RIIC_BUS_BUSY             ((bool)(1))
#define RIIC_BUS_FREE             ((bool)(0))
#define RIIC_10BIT_SARU_MASK      (0x0300)
#define RIIC_10BIT_SARL_MASK      (0x00ff)
#define RIIC_I2C_SLV_ADDR         (0x80)
#define RIIC_READ_MASK            (0x01)

/****************************************************************************
 * Typedef definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum ra6m5_i2c_mode_e
{
  RIIC_NONE = 0,              /* Uninitialized state */
  RIIC_READY,                 /* Ready for operation */
  RIIC_M_TRANSMIT,            /* Master transmission mode */
  RIIC_M_RECEIVE,             /* Master reception mode */
  RIIC_FINISH,                /* Successful operation */
};

enum ra6m5_i2c_dev_sts_e
{
  RIIC_STS_NO_INIT = 0,        /* None initialization state */
  RIIC_STS_IDLE,               /* Idle state */
  RIIC_STS_ST_COND_WAIT,       /* Start condition generation completion wait state */
  RIIC_STS_SEND_SLVADR_W_WAIT, /* Slave address [Write] transmission completion wait state */
  RIIC_STS_SEND_SLVADR_R_WAIT, /* Slave address [Read] transmission completion wait state */
  RIIC_STS_SEND_DATA_WAIT,     /* Data transmission completion wait state */
  RIIC_STS_RECEIVE_DATA_WAIT,  /* Data reception completion wait state */
  RIIC_STS_SP_COND_WAIT,       /* Stop condition generation completion wait state */
  RIIC_STS_AL,                 /* Detect Arbitration Lost */
  RIIC_STS_TMO,                /* Detect Time out */
  RIIC_STS_MAX,                /* Prohibition of setup above here */
};

enum ra6m5_i2c_event_e
{
  RIIC_EV_NONE = 0,
  RIIC_EV_GEN_START_COND,     /* Called function of Start condition generation */
  RIIC_EV_INT_START,          /* Interrupted start codition generation */
  RIIC_EV_INT_ADD,            /* Interrupted address sending */
  RIIC_EV_INT_SEND,           /* Interrupted data sending */
  RIIC_EV_INT_RECEIVE,        /* Interrupted data receiving */
  RIIC_EV_INT_STOP,           /* Interrupted Stop condition generation */
  RIIC_EV_INT_AL,             /* Interrupted Arbitration-Lost */
  RIIC_EV_INT_NACK,           /* Interrupted No Acknowledge */
  RIIC_EV_INT_TXI,            /* Interrupted transmitted buffer empty */
  RIIC_EV_INT_TMO,            /* Interrupted Time out */
  RIIC_EV_MAX,                /* Prohibition of setup above here */
};

struct ra6m5_i2c_dev_s
{
  uint32_t  base;             /* Base address of registers */
  uint32_t  frequency;        /* Configured kbps */
  uint8_t   txi_irq;          /* TXI IRQ Number */
  uint8_t   rxi_irq;          /* RXI IRQ Number */
  uint8_t   tei_irq;          /* TEI IRQ Number */
  uint8_t   eri_irq;          /* ERI IRQ Number */
  uint32_t  scl_pin;          /* GPIO configuration for SCL as SCL */
  uint32_t  sda_pin;          /* GPIO configuration for SDA as SDA */
};

struct ra6m5_i2c_priv_s
{
  const struct      i2c_ops_s *ops;
  const struct      ra6m5_i2c_dev_s *dev;
  int               refs;            /* Referernce count */
  int               bus;             /* Bus number */
  volatile uint8_t  mode;            /* See enum ra6m5_i2c_mode_e */
  volatile uint8_t  dev_sts;         /* See enum ra6m5_i2c_dev_sts_e */
  volatile uint8_t  event;           /* See enum ra6m5_i2c_event_e */
  mutex_t           lock;            /* Mutual exclusion mutex */
  sem_t             sem_isr;         /* Interrupt wait semaphore */
  uint8_t           msgc;            /* Number of Messages */
  struct            i2c_msg_s *msgv; /* Message list */
  uint8_t           *ptr;            /* Current message buffer */
  int               dcnt;            /* Bytes remaining to transfer */
  uint16_t          flags;           /* Current message flags */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ra6m5_iic_iicrst(struct ra6m5_i2c_priv_s *priv);
static void ra6m5_iic_setclock(struct ra6m5_i2c_priv_s *priv,
                                uint32_t frequency);
static void ra6m5_iic_init(struct ra6m5_i2c_priv_s *priv);
static int ra6m5_iic_startcond(struct ra6m5_i2c_priv_s *priv);
static void ra6m5_iic_restartcond(struct ra6m5_i2c_priv_s *priv);
static void ra6m5_iic_send_slv_addr(struct ra6m5_i2c_priv_s *priv);
static void ra6m5_iic_after_send_slvadr(struct ra6m5_i2c_priv_s *priv);
static void ra6m5_iic_master_transmit(struct ra6m5_i2c_priv_s *priv);
static void ra6m5_iic_master_receive(struct ra6m5_i2c_priv_s *priv);
static void ra6m5_iic_stopcond(struct ra6m5_i2c_priv_s *priv);
static void ra6m5_iic_advance(struct ra6m5_i2c_priv_s *priv);
static uint8_t ra6m5_iic_read_data(struct ra6m5_i2c_priv_s *priv);
static void ra6m5_iic_wait_set(struct ra6m5_i2c_priv_s *priv);
static void ra6m5_iic_pre_end_set(struct ra6m5_i2c_priv_s *priv);
static void ra6m5_iic_end_set(struct ra6m5_i2c_priv_s *priv);
static void ra6m5_iic_nack(struct ra6m5_i2c_priv_s *priv);
static void ra6m5_iic_set_sending_data(struct ra6m5_i2c_priv_s *priv,
                                        uint8_t data);
static int ra6m5_iic_after_stop(struct ra6m5_i2c_priv_s *priv);
static bool ra6m5_iic_check_bus_busy(struct ra6m5_i2c_priv_s *priv);
static void ra6m5_iic_set_icier(struct ra6m5_i2c_priv_s *priv,
                                 uint8_t value);
static void ra6m5_iic_timeout(struct ra6m5_i2c_priv_s *priv);

/* RIIC Interrupt Handling */

static void ra6m5_iic_int_disable(struct ra6m5_i2c_priv_s *priv);
static void ra6m5_iic_int_enable(struct ra6m5_i2c_priv_s *priv);
static int ra6m5_iic_irq_init(struct ra6m5_i2c_priv_s *priv);

/*  Interrupt Handling */

static int ra6m5_iic_rxi_isr(int irq, void *context, void *arg);
static int ra6m5_iic_txi_isr(int irq, void *context, void *arg);
static int ra6m5_iic_eri_isr(int irq, void *context, void *arg);
static int ra6m5_iic_tei_isr(int irq, void *context, void *arg);

/* I2C operations */

static int ra6m5_i2c_transfer(struct i2c_master_s *dev,
                              struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int ra6m5_i2c_reset(struct i2c_master_s *dev);
#endif

/****************************************************************************
 * Private Macros
 ****************************************************************************/

#define RA6M5_IIC_STOP(n)       (1 << (7 + (2-(n))))

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void up_enable_irq(int irq);
void up_disable_irq(int irq);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Device Structures, Instantiation */

static const struct i2c_ops_s ra6m5_i2c_ops =
{
  .transfer = ra6m5_i2c_transfer
#ifdef CONFIG_I2C_RESET
  , .reset  = ra6m5_i2c_reset
#endif
};

#ifdef CONFIG_RA6M5_IIC0
static const struct ra6m5_i2c_dev_s ra6m5_riic0_dev =
{
  .base       = RA6M5_I2C0_BASE,
  .frequency  = CONFIG_RA6M5_IIC0_BITRATE,
#ifndef CONFIG_I2C_POLLED
  .txi_irq    = RA6M5_IRQ_IIC0_TXI,
  .rxi_irq    = RA6M5_IRQ_IIC0_RXI,
  .tei_irq    = RA6M5_IRQ_IIC0_TEI,
  .eri_irq    = RA6M5_IRQ_IIC0_ERI,
#endif
  .sda_pin    = GPIO_IIC0_SDA,
  .scl_pin    = GPIO_IIC0_SCL,
};

static struct ra6m5_i2c_priv_s ra6m5_riic0_priv =
{
  .ops       = &ra6m5_i2c_ops,
  .dev       = &ra6m5_riic0_dev,
  .refs      = 0,
  .bus       = 0,
  .lock      = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr   = SEM_INITIALIZER(0),
#endif
  .mode      = RIIC_NONE,
  .dev_sts   = RIIC_STS_NO_INIT,
  .event     = RIIC_EV_NONE,
  .msgc      = 0,
  .msgv      = NULL,
  .ptr       = NULL,
  .dcnt      = 0,
  .flags     = 0,
};
#endif

#ifdef CONFIG_RA6M5_IIC1
static const struct ra6m5_i2c_dev_s ra6m5_riic1_dev =
{
  .base      = RA6M5_I2C1_BASE,
  .frequency = CONFIG_RA6M5_IIC1_BITRATE,
#ifndef CONFIG_I2C_POLLED
  .txi_irq    = RA6M5_IRQ_IIC1_TXI,
  .rxi_irq    = RA6M5_IRQ_IIC1_RXI,
  .tei_irq    = RA6M5_IRQ_IIC1_TEI,
  .eri_irq    = RA6M5_IRQ_IIC1_ERI,
#endif
  .sda_pin    = GPIO_IIC1_SDA,
  .scl_pin    = GPIO_IIC1_SCL,
};

static struct ra6m5_i2c_priv_s ra6m5_riic1_priv =
{
  .ops       = &ra6m5_i2c_ops,
  .dev       = &ra6m5_riic1_dev,
  .refs      = 0,
  .bus       = 1,
  .lock      = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr   = SEM_INITIALIZER(0),
#endif
  .mode      = RIIC_NONE,
  .dev_sts   = RIIC_STS_NO_INIT,
  .event     = RIIC_EV_NONE,
  .msgc      = 0,
  .msgv      = NULL,
  .ptr       = NULL,
  .dcnt      = 0,
  .flags     = 0,
};
#endif

#ifdef CONFIG_RA6M5_IIC2
static const struct ra6m5_i2c_dev_s ra6m5_riic2_dev =
{
  .base      = RA6M5_I2C2_BASE,
  .frequency = CONFIG_RA6M5_IIC2_BITRATE,
#ifndef CONFIG_I2C_POLLED
  .txi_irq    = RA6M5_IRQ_IIC2_TXI,
  .rxi_irq    = RA6M5_IRQ_IIC2_RXI,
  .tei_irq    = RA6M5_IRQ_IIC2_TEI,
  .eri_irq    = RA6M5_IRQ_IIC2_ERI,
#endif
  .sda_pin    = GPIO_IIC2_SDA,
  .scl_pin    = GPIO_IIC2_SCL,
};

static struct ra6m5_i2c_priv_s ra6m5_riic2_priv =
{
  .ops       = &ra6m5_i2c_ops,
  .dev       = &ra6m5_riic2_dev,
  .refs      = 0,
  .bus       = 2,
  .lock      = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr   = SEM_INITIALIZER(0),
#endif
  .mode      = RIIC_NONE,
  .dev_sts   = RIIC_STS_NO_INIT,
  .event     = RIIC_EV_NONE,
  .msgc      = 0,
  .msgv      = NULL,
  .ptr       = NULL,
  .dcnt      = 0,
  .flags     = 0,
};
#endif


/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra6m5_riic_getreg
 *
 * Description:
 *   Get a 8-bit register value by offset
 *
 ****************************************************************************/

static inline uint8_t ra6m5_riic_getreg(struct ra6m5_i2c_priv_s *priv,
                                        uint32_t offset)
{
  return getreg8(priv->dev->base + offset);
}

/****************************************************************************
 * Name: ra6m5_riic_putreg
 *
 * Description:
 *  Put a 8-bit register value by offset
 *
 ****************************************************************************/

static inline void ra6m5_riic_putreg(struct ra6m5_i2c_priv_s *priv,
                                    uint32_t offset, uint8_t value)
{
  putreg8(value, priv->dev->base + offset);
}

/****************************************************************************
 * Name: ra6m5_riic_modifyreg
 *
 * Description:
 *   Modify a 8-bit register value by offset
 *
 ****************************************************************************/

static inline void ra6m5_riic_modifyreg(struct ra6m5_i2c_priv_s *priv,
                                       uint32_t offset, uint8_t clearbits,
                                       uint8_t setbits)
{
  modifyreg8(priv->dev->base + offset, clearbits, setbits);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: riic_mpc_enable
 *
 * Description:
 * Enable writing to registers
 ****************************************************************************/

static void riic_mpc_enable(void)
{
  /* Enable writing to registers related to operating modes,
   * LPC, CGC and software reset
   */


  /* Enable writing to MPC pin function control registers */
}

/****************************************************************************
 * Name: riic_mpc_disable
 *
 * Description:
 * Disable writing to registers
 ****************************************************************************/

static void riic_mpc_disable(void)
{
  /* Disable writing to MPC pin function control registers */

  /* Enable protection */

}

/****************************************************************************
 * Name: ra6m5_iic_iicrst
 *
 * Description:
 *   Disable writing to registers
 ****************************************************************************/

static int ra6m5_iic_iicrst(struct ra6m5_i2c_priv_s *priv)
{
  uint8_t regval;

  regval = ra6m5_riic_getreg(priv, RA6M5_IIC_ICCR1_OFFSET);
  regval |= IIC_ICCR1_ICE;
  ra6m5_riic_putreg(priv, RA6M5_IIC_ICCR1_OFFSET, regval);
  regval = ra6m5_riic_getreg(priv, RA6M5_IIC_ICCR1_OFFSET);

  regval = ra6m5_riic_getreg(priv, RA6M5_IIC_ICCR1_OFFSET);
  regval &= ~(IIC_ICCR1_ICE);
  ra6m5_riic_putreg(priv, RA6M5_IIC_ICCR1_OFFSET,regval );
  regval = ra6m5_riic_getreg(priv, RA6M5_IIC_ICCR1_OFFSET);

  return OK;
}

/****************************************************************************
 * Name: ra6m5_iic_set_icier
 *
 * Description:
 *   Sets interrupt enable register ICIER for RIIC communication
 *
 ****************************************************************************/

static void ra6m5_iic_set_icier(struct ra6m5_i2c_priv_s *priv,
                                 uint8_t value)
{
  uint8_t regval;
  regval = (value | IIC_ICIER_TMOIE);

  ra6m5_riic_putreg(priv, RA6M5_IIC_ICFER_OFFSET, regval);
  regval = ra6m5_riic_getreg(priv, RA6M5_IIC_ICFER_OFFSET);
}

/****************************************************************************
 * Name: ra6m5_iic_setclock
 *
 * Description:
 *   Sets the I2C bus clock frequency – frequency for the transfer
 *
 ****************************************************************************/

static void ra6m5_iic_setclock(struct ra6m5_i2c_priv_s *priv,
                                uint32_t frequency)
{
  /* Divider array of RIIC clock  */

  uint8_t regval;
  const uint8_t d_cks[RIIC_MAX_DIV] =
    { 1, 2, 4, 8, 16, 32, 64, 128
    };

  volatile double l_time;     /* L Width period */
  volatile double h_time;     /* H Width period */
  volatile double calc_val;   /* Using for Calculation */
  double calc_val_tmp;
  volatile uint8_t cnt;

  /* Calculation of L width time */

  l_time = (1.0 / (2.0 * frequency)); /* Half period of frequency */
  h_time = l_time;

  /* Check I2C mode of Speed */

  if (frequency > I2C_SPEED_FAST)
    {
      i2cinfo("Fast Plus Mode Selected - Transmission Rate: 1 Mbps\n");
      if (l_time < 0.5E-6)
        {
          /* Wnen L width less than 0.5us, subtract Rise up and down
           * time for SCL from H/L width
           */

          l_time = 0.5E-6;
          h_time = (((1.0 / frequency) - l_time) - SCL_RISE_TIME_FASTPLUS)
                 - SCL_FALL_TIME_FASTPLUS;
        }

      else
        {
          /* Subtract Rise up and down time for SCL from H/L width */

          l_time -= SCL_FALL_TIME_FASTPLUS;
          h_time -= SCL_RISE_TIME_FASTPLUS;
        }
    }

  else if (frequency > I2C_SPEED_STANDARD)
    {
      i2cinfo("Fast Mode Selected - Transmission Rate: 400 kbps\n");
      if (l_time < 1.3E-6)
        {
          /* Wnen L width less than 1.3us, subtract Rise up and down
           * time for SCL from H/L width
           */

          l_time = 1.3E-6;
          h_time = (((1.0 / frequency) - l_time) - SCL_RISE_TIME_FAST)
                    - SCL_FALL_TIME_FAST;
        }

      else
        {
          /* Subtract Rise up and down time for SCL from H/L width */

          l_time -= SCL_FALL_TIME_FAST;
          h_time -= SCL_RISE_TIME_FAST;
        }
    }

  else
    {
      i2cinfo("Standard Mode Selected - Transmission Rate: 100 kbps\n");

      /* Subtract Rise up and down time for SCL from H/L width */

      l_time -= SCL_FALL_TIME_STANDARD;
      h_time -= SCL_RISE_TIME_STANDARD;
    }

  /* Calculate ICBRL value */

  for (calc_val = RIIC_RATE_CALC, cnt = 0; RIIC_CALC_MAX < calc_val; cnt++)
    {
      calc_val = l_time; /* Set L width time */

      /* Check the range of divider of CKS */

      if (cnt >= RIIC_MAX_DIV)
        {
          /* Cannot set bps */
        }

      calc_val_tmp = calc_val;

      /* Calculation of ICBRL value */

      calc_val = (calc_val_tmp / (d_cks[cnt] / g_clock_freq[RA6M5_CLOCKS_SOURCE_PCLKB]));
      calc_val = calc_val + 0.5; /* round off */
    }

  /* Set ICMR1.CKS bits. */

  regval = ra6m5_riic_getreg(priv, RA6M5_IIC_ICMR1_OFFSET);
  regval &= IIC_ICMR1_CKS_MASK;
  regval |= IIC_ICMR1_CKS(cnt);
  ra6m5_riic_putreg(priv, RA6M5_IIC_ICMR1_OFFSET, regval);

  /* Set value to ICBRL register */

  regval = (uint8_t)((uint8_t)(calc_val - 1) | RIIC_ICBRL_MASK);
  ra6m5_riic_putreg(priv, RA6M5_IIC_ICBRL_OFFSET, regval);

  /* Calculate ICBRH value */

  calc_val = h_time;
  calc_val_tmp = calc_val;

  /* Calculation ICBRH value */

  calc_val = (calc_val_tmp / (d_cks[cnt - 1] / g_clock_freq[RA6M5_CLOCKS_SOURCE_PCLKB]));
  calc_val = (uint8_t)(calc_val + 0.5); /* Round off */

  /* If the calculated value is less than 1, it rounded up to 1. */

  if (1 > calc_val)
    {
      calc_val = 1;
    }

  /* Set value to ICBRH register */

  regval = (uint8_t)((uint8_t)(calc_val - 1) | RIIC_ICBRH_MASK);
  ra6m5_riic_putreg(priv, regval, RA6M5_IIC_ICBRH_OFFSET);
}

/****************************************************************************
 * Name: ra6m5_iic_int_enable
 *
 * Description:
 *   Enable Interrupts
 *
 ****************************************************************************/

static void ra6m5_iic_int_enable(struct ra6m5_i2c_priv_s *priv)
{
  up_enable_irq(priv->dev->txi_irq);
  up_enable_irq(priv->dev->rxi_irq);
  up_enable_irq(priv->dev->tei_irq);
  up_enable_irq(priv->dev->eri_irq);
}

/****************************************************************************
 * Name: ra6m5_iic_int_disable
 *
 * Description:
 *   Disable Interrupts
 *
 ****************************************************************************/

static void ra6m5_iic_int_disable(struct ra6m5_i2c_priv_s *priv)
{
  up_disable_irq(priv->dev->txi_irq);
  up_disable_irq(priv->dev->rxi_irq);
  up_disable_irq(priv->dev->tei_irq);
  up_disable_irq(priv->dev->eri_irq);
}

/****************************************************************************
 * Name: ra6m5_iic_irq_init
 *
 * Description:
 *   Setup the initial conditions of I2C interrupt handling
 *
 ****************************************************************************/

static int ra6m5_iic_irq_init(struct ra6m5_i2c_priv_s *priv)
{
  int ret;

  #if 0
  IEN(ICU, GROUPBL1) = 0;     /* Disable Group BL1 interrupts */
  IR(ICU, GROUPBL1)  = 0;     /* Clear interrupt flag */
  IPR(ICU, GROUPBL1) = RIIC_INTERRUPT_PRIO;
  IEN(ICU, GROUPBL1) = 1;     /* Enable Group BL1 interrupt */

  IR(RIIC0, RXI0)  = 0;
  IPR(RIIC0, RXI0) = RIIC_INTERRUPT_PRIO;

  IR(RIIC0, TXI0)  = 0;
  IPR(RIIC0, TXI0) = RIIC_INTERRUPT_PRIO;
  #endif

  ret = irq_attach(priv->dev->rxi_irq, ra6m5_iic_rxi_isr, priv);
  ret = irq_attach(priv->dev->txi_irq, ra6m5_iic_txi_isr, priv);
  ret = irq_attach(priv->dev->tei_irq, ra6m5_iic_tei_isr, priv);
  ret = irq_attach(priv->dev->eri_irq, ra6m5_iic_eri_isr, priv);

  ra6m5_iic_int_enable(priv);

  return ret;
}

/****************************************************************************
 * Name: ra6m5_iic_init
 *
 * Description:
 *   Setup the initial conditions of I2C hardware and be ready for operation
 *
 ****************************************************************************/

static void ra6m5_iic_init(struct ra6m5_i2c_priv_s *priv)
{
  uint8_t regval;

  /* Disable RXI, TXI, TEI and ERI interrupts */

  ra6m5_iic_int_disable(priv);

  /* Release from the module-stop state */

  modifyreg32(RA6M5_MSTP_REG(RA6M5_MSTP_MSTPCRB_OFFSET), RA6M5_IIC_STOP(priv->bus), 0);

  /* Configure pins */

  if (ra6m5_configgpio(priv->dev->sda_pin) < 0) {
      return;
  }

  if (ra6m5_configgpio(priv->dev->scl_pin) < 0) {
      return;
  }

  /* SCLn and SDAn pins not driven - RIIC disabled */

  regval = ra6m5_riic_getreg(priv, RA6M5_IIC_ICCR1_OFFSET);
  regval &= ~IIC_ICCR1_ICE;
  ra6m5_riic_putreg(priv, RA6M5_IIC_ICCR1_OFFSET, regval);

  /* RIIC Reset */

  regval = ra6m5_riic_getreg(priv, RA6M5_IIC_ICCR1_OFFSET);
  regval |= IIC_ICCR1_IICRST;
  ra6m5_riic_putreg(priv, RA6M5_IIC_ICCR1_OFFSET, regval);

  /* Internal reset, SCLn and SDAn pins in active state */

  regval = ra6m5_riic_getreg(priv, RA6M5_IIC_ICCR1_OFFSET);
  regval |= IIC_ICCR1_ICE;
  ra6m5_riic_putreg(priv, regval, RA6M5_IIC_ICCR1_OFFSET);

  /* Set SARLy and SARUy */

  ra6m5_riic_putreg(priv, RA6M5_IIC_SARL0_OFFSET, 0x00);
  ra6m5_riic_putreg(priv, RA6M5_IIC_SARU0_OFFSET, 0x00);
  ra6m5_riic_putreg(priv, RA6M5_IIC_SARL1_OFFSET, 0x00);
  ra6m5_riic_putreg(priv, RA6M5_IIC_SARU1_OFFSET, 0x00);
  ra6m5_riic_putreg(priv, RA6M5_IIC_SARL2_OFFSET, 0x00);
  ra6m5_riic_putreg(priv, RA6M5_IIC_SARU2_OFFSET, 0x00);

  /* Set I2C bus frequency */

  ra6m5_iic_setclock(priv, priv->dev->frequency);

  /* Clear status enable register */

  regval = ra6m5_riic_getreg(priv, RA6M5_IIC_ICSER_OFFSET);
  regval &= 0x00;
  ra6m5_riic_putreg(priv, RA6M5_IIC_ICSER_OFFSET, regval);

#ifndef CONFIG_RA6M5_IIC0_NF
  regval = ra6m5_riic_getreg(priv, RA6M5_IIC_ICFER_OFFSET);
  regval &= ~IIC_ICFER_NFE;
  ra6m5_riic_putreg(priv, RA6M5_IIC_ICSER_OFFSET, regval);
#else
  regval  = ra6m5_riic_getreg(priv, RA6M5_IIC_ICMR3_OFFSET);
  regval &= ~IIC_ICMR3_NF_MASK; 

  #if 0
  switch (CONFIG_RA6M5_IIC0_NF_STAGE)
    {
      case 1:
        regval |= RA6M5_RIIC_ICMR3_NF1;
        break;

      case 2:
        regval |= RA6M5_RIIC_ICMR3_NF2;
        break;

      case 3:
        regval |= RA6M5_RIIC_ICMR3_NF3;
        break;

      case 4:
        regval |= RA6M5_RIIC_ICMR3_NF4;
        break;
    }
  #else
  regval |= IIC_ICMR3_NF(1);
  #endif
  ra6m5_riic_putreg(priv, RA6M5_IIC_ICMR3_OFFSET, regval);
#endif

  /* Disable all interrupts */

  ra6m5_riic_putreg(priv, RA6M5_IIC_ICIER_OFFSET, 0x00);

  /* Enable timeout function */

  regval = ra6m5_riic_getreg(priv, RA6M5_IIC_ICFER_OFFSET);
  regval |= IIC_ICFER_TMOE;
  ra6m5_riic_putreg(priv, regval, RA6M5_IIC_ICFER_OFFSET);

#ifdef CONFIG_RA6M5_IIC0_SDA_DELAY
  regval  = ra6m5_riic_getreg(priv, RA6M5_IIC_ICMR2_OFFSET);
  regval |= IIC_ICMR2_DLCS;
  regval &= ~IIC_ICMR2_SDDL_MASK; 

  #if 0
  switch (CONFIG_RA6M5_IIC0_DELAY_CNT)
    {
      case 0:
        regval |= RA6M5_RIIC_ICMR2_SDDL0;
        break;

      case 1:
        regval |= RA6M5_RIIC_ICMR2_SDDL1;
        break;

      case 2:
        regval |= RA6M5_RIIC_ICMR2_SDDL2;
        break;

      case 3:
        regval |= RA6M5_RIIC_ICMR2_SDDL3;
        break;

      case 4:
        regval |= RA6M5_RIIC_ICMR2_SDDL4;
        break;

      case 5:
        regval |= RA6M5_RIIC_ICMR2_SDDL5;
        break;

      case 6:
        regval |= RA6M5_RIIC_ICMR2_SDDL6;
        break;

      case 7:
        regval |= RA6M5_RIIC_ICMR2_SDDL7;
        break;
    }
  #else
  regval |= IIC_ICMR2_SDDL(0);
  #endif

  ra6m5_riic_putreg(priv, RA6M5_IIC_ICMR2_OFFSET, regval);
#endif

  ra6m5_iic_irq_init(priv);

  /* Relese RIIC reset state */

  regval = ra6m5_riic_getreg(priv, RA6M5_IIC_ICCR1_OFFSET);
  regval &= ~(IIC_ICCR1_IICRST);
  ra6m5_riic_putreg(priv, regval, RA6M5_IIC_ICCR1_OFFSET);

  priv->mode = RIIC_READY;
  priv->dev_sts = RIIC_STS_IDLE;

  /* RIIC registers initialized */
}

/****************************************************************************
 * Function Name: ra6m5_iic_set_sending_data
 * Description  : Transmit Data Processing.
 *                Sets transmission data to ICDRT register.
 * Arguments    : ra6m5_i2c_priv_s *priv      ; IIC Information
 * Return Value : None
 ****************************************************************************/

static void ra6m5_iic_set_sending_data(struct ra6m5_i2c_priv_s *priv,
                                        uint8_t data)
{
  uint8_t regval;

  /* Clears TIE interrupt request register. */

  regval  = ra6m5_riic_getreg(priv, RA6M5_IIC_ICIER_OFFSET);
  regval &= ~IIC_ICIER_TIE;
  ra6m5_riic_putreg(priv, RA6M5_IIC_ICIER_OFFSET, regval);

  /* Sets the transmitting data. */

  ra6m5_riic_putreg(priv, RA6M5_IIC_ICDRT_OFFSET, data);
  data = ra6m5_riic_getreg(priv, RA6M5_IIC_ICDRT_OFFSET);
}

/****************************************************************************
 * Name: ra6m5_iic_advance
 *
 * Description:
 *  Check if RIIC Bus is busy
 *
 ****************************************************************************/

static void ra6m5_iic_advance(struct ra6m5_i2c_priv_s *priv)
{
  int ret;

  if ((RIIC_EV_INT_TMO == priv->event)  || (RIIC_STS_TMO == priv->dev_sts))
    {
      i2cerr("RIIC Transfer Timed Out\n");
      ra6m5_iic_timeout(priv);
    }

  if (RIIC_EV_INT_AL == priv->event)
    {
      ra6m5_iic_iicrst(priv);
    }

  if (RIIC_EV_INT_START == priv->event)
    {
      ra6m5_iic_send_slv_addr(priv);
    }

  if (RIIC_EV_INT_ADD == priv->event)
    {
      ra6m5_iic_after_send_slvadr(priv);
    }

  if (RIIC_EV_INT_SEND == priv->event)
    {
      ra6m5_iic_master_transmit(priv);
    }

  if (RIIC_EV_INT_RECEIVE == priv->event)
    {
      if (RIIC_STS_RECEIVE_DATA_WAIT == priv->dev_sts)
        {
          ra6m5_iic_master_receive(priv);
        }

      else
        {
          ra6m5_iic_after_send_slvadr(priv);
        }
    }

  if (RIIC_EV_INT_NACK == priv->event)
    {
      ra6m5_iic_nack(priv);
    }

  if (RIIC_EV_INT_STOP == priv->event)
    {
      ret = ra6m5_iic_after_stop(priv);
      if (RIIC_SUCCESS == ret)
        {
          priv->mode = RIIC_FINISH;
        }
    }
}

/****************************************************************************
 * Name: ra6m5_iic_nack
 *
 * Description:
 *  NACK reception handler
 *
 ****************************************************************************/

static void ra6m5_iic_nack(struct ra6m5_i2c_priv_s *priv)
{
  ra6m5_iic_set_icier(priv, IIC_ICIER_SPIE | IIC_ICIER_ALIE);

  priv->dev_sts = RIIC_STS_SP_COND_WAIT;
  ra6m5_iic_stopcond(priv);

  if (RIIC_M_RECEIVE == priv->mode)
    {
      ra6m5_iic_read_data(priv);
    }
}

/****************************************************************************
 * Name: ra6m5_iic_timeout
 *
 * Description:
 *  After Timeout condition processing
 *
 ****************************************************************************/

static void ra6m5_iic_timeout(struct ra6m5_i2c_priv_s *priv)
{
  priv->mode = RIIC_FINISH;
}

/****************************************************************************
 * Name: ra6m5_iic_after_stop
 *
 * Description:
 *  After stop condition processing
 *
 ****************************************************************************/

static int ra6m5_iic_after_stop(struct ra6m5_i2c_priv_s *priv)
{
  int ret;
  bool bus;

  bus = ra6m5_iic_check_bus_busy(priv);

  if (bus == RIIC_BUS_BUSY)
    {
      ret = RIIC_ERR_BUS_BUSY;
    }

  else
    {
      if (RIIC_EV_INT_TMO == priv->event)
        {
          ret = RIIC_ERR_TMO;
        }

      else
        {
          ret = RIIC_SUCCESS;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: ra6m5_iic_check_bus_busy
 *
 * Description:
 *  Check if RIIC Bus is busy
 *
 ****************************************************************************/

static bool ra6m5_iic_check_bus_busy(struct ra6m5_i2c_priv_s *priv)
{
  bool bus_state = RIIC_BUS_BUSY;
  uint8_t iccr1, iccr2;
  int i;

  for (i = BUS_CHECK_COUNTER; i >= 0; i--)
    {
      iccr1 = ra6m5_riic_getreg(priv, RA6M5_IIC_ICCR1_OFFSET);
      iccr2 = ra6m5_riic_getreg(priv, RA6M5_IIC_ICCR2_OFFSET);

      if ((iccr1 & IIC_ICCR1_SDAI) && (iccr1 & IIC_ICCR1_SCLI) && !(iccr2 & IIC_ICCR2_BBSY))
        {
          bus_state = RIIC_BUS_FREE;
          break;
        }
    }

  return bus_state;
}

/****************************************************************************
 * Name: ra6m5_iic_startcond
 *
 * Description:
 *  Issue the start condition for Master transmission/reception
 *
 ****************************************************************************/

static int ra6m5_iic_startcond(struct ra6m5_i2c_priv_s *priv)
{
  uint8_t regval;
  int ret = RIIC_SUCCESS;
  bool bus;

  bus = ra6m5_iic_check_bus_busy(priv);

  if (bus == RIIC_BUS_FREE)
    {
      /* Clear stop */

      regval = ra6m5_riic_getreg(priv, RA6M5_IIC_ICSR2_OFFSET);
      regval &= ~(IIC_ICSR2_STOP);
      ra6m5_riic_putreg(priv, RA6M5_IIC_ICSR2_OFFSET, regval);

      regval = ra6m5_riic_getreg(priv, RA6M5_IIC_ICSR2_OFFSET);
      while (0x00 != ((regval & IIC_ICSR2_START) || (regval & IIC_ICSR2_STOP)))
        {
          regval = ra6m5_riic_getreg(priv, RA6M5_IIC_ICSR2_OFFSET);
        }

      /* Set the internal status */

      priv->dev_sts = RIIC_STS_ST_COND_WAIT;

      /* Clear ALIE bit */

      regval = ra6m5_riic_getreg(priv, RA6M5_IIC_ICIER_OFFSET);
      regval &= ~(IIC_ICIER_ALIE);
      ra6m5_riic_putreg(priv, RA6M5_IIC_ICIER_OFFSET, regval);

      /* Interrupt setting */

      ra6m5_iic_set_icier(priv, IIC_ICIER_STIE | IIC_ICIER_NAKIE | IIC_ICIER_ALIE);

      /* Generate the start condition */

      /* Clear the start condition detection flag */

      regval = ra6m5_riic_getreg(priv, RA6M5_IIC_ICSR2_OFFSET);
      if (0x00 != ((regval & IIC_ICSR2_START) >> 2U))
        {
          /* Clears start condition detection flag. */

          regval &= ~(IIC_ICSR2_START);
          ra6m5_riic_putreg(priv, RA6M5_IIC_ICSR2_OFFSET, regval);
          regval = ra6m5_riic_getreg(priv, RA6M5_IIC_ICSR2_OFFSET);
        }

      /* Generate start condition */

      regval  = ra6m5_riic_getreg(priv, RA6M5_IIC_ICCR2_OFFSET);
      regval |= IIC_ICCR2_ST;
      ra6m5_riic_putreg(priv, RA6M5_IIC_ICCR2_OFFSET, regval);
      regval = ra6m5_riic_getreg(priv, RA6M5_IIC_ICCR2_OFFSET);
    }

  else
    {
      ret = RIIC_ERR_BUS_BUSY;
    }

  return ret;
}

/****************************************************************************
 * Name: ra6m5_iic_restartcond
 *
 * Description:
 *  Issue Re-start condition for Master Transmission/Reception
 *
 ****************************************************************************/

static void ra6m5_iic_restartcond(struct ra6m5_i2c_priv_s *priv)
{
  uint8_t regval;

  /* Clears start condition detection flag. */

  regval = getreg8(RA6M5_IIC_ICSR2_OFFSET);
  if (0x00 != ((regval & IIC_ICSR2_START) >> 2U))
    {
      regval &= ~(IIC_ICSR2_START);
      ra6m5_riic_putreg(priv, RA6M5_IIC_ICSR2_OFFSET, regval);
    }

  /* Generates a restart condition. */

  regval = getreg8(RA6M5_IIC_ICCR2_OFFSET);
  regval |= IIC_ICCR2_RS; /* Sets ICCR2.RS bit. */
  ra6m5_riic_putreg(priv, RA6M5_IIC_ICCR2_OFFSET, regval);
}

/****************************************************************************
 * Name: ra6m5_iic_send_slv_addr
 *
 * Description:
 *  Transmit slave address for communication
 *
 ****************************************************************************/

static void ra6m5_iic_send_slv_addr(struct ra6m5_i2c_priv_s *priv)
{
  uint16_t bit_addr;
  uint8_t regval;

  if (priv->msgv->flags == I2C_M_TEN)
    {
      /* 10 bit slave address handling */

      bit_addr = priv->msgv->addr;
      bit_addr &= RIIC_10BIT_SARU_MASK;
      bit_addr >>= 8;
      regval = (uint8_t)bit_addr;

      ra6m5_iic_set_sending_data(priv, regval);

      /* Check ACK/NACK reception */

      bit_addr = priv->msgv->addr;
      bit_addr &= RIIC_10BIT_SARL_MASK;
      regval = (uint8_t)bit_addr;
    }

  else
    {
      if (priv->flags == I2C_M_READ)
        {
          priv->mode = RIIC_M_RECEIVE;
          ra6m5_iic_set_icier(priv, IIC_ICIER_RIE | IIC_ICIER_NAKIE | IIC_ICIER_ALIE);

          /* 7-bit slave address with READ code */

          regval = priv->msgv->addr;
          regval <<= 1U;
          regval |= RIIC_READ_MASK;

          priv->dev_sts = RIIC_STS_SEND_SLVADR_R_WAIT;
        }

      else
        {
          priv->mode = RIIC_M_TRANSMIT;
          ra6m5_iic_set_icier(priv, IIC_ICIER_TEIE | IIC_ICIER_NAKIE | IIC_ICIER_ALIE);

          /* 7-bit slave address with WRITE code */

          regval = priv->msgv->addr;
          regval <<= 1U;
          regval &= ~(RIIC_READ_MASK);

          priv->dev_sts = RIIC_STS_SEND_SLVADR_W_WAIT;
        }
    }

  /* Send slave address */

  ra6m5_iic_set_sending_data(priv, regval);
}

/****************************************************************************
 * Name: ra6m5_iic_after_send_slvadr
 *
 * Description:
 *  Processing after sending slave address
 *
 ****************************************************************************/

static void ra6m5_iic_after_send_slvadr(struct ra6m5_i2c_priv_s *priv)
{
  uint8_t regval;
  uint8_t *data;

  if (RIIC_M_TRANSMIT == priv->mode)
    {
      /* Pattern write 1: msg[0] is memory addressand msg[1] is data byte */

      if (2U >= priv->msgc)
        {
          priv->dev_sts = RIIC_STS_SEND_DATA_WAIT;
          ra6m5_iic_set_icier(priv, IIC_ICIER_TEIE | IIC_ICIER_NAKIE | IIC_ICIER_ALIE);

          /* Transmit first byte (memory address) */

          if (0U != priv->dcnt)
            {
              data = priv->ptr;
              regval = *data;
              ra6m5_iic_set_sending_data(priv, regval);

              priv->dcnt--;
              priv->ptr++;
            }

          else
            {
              /* Do nothing */
            }
        }

      /* Pattern write 2: msg[0] is data byte */

      else if (1U >= priv->msgc)
        {
          priv->dev_sts = RIIC_STS_SEND_DATA_WAIT;
          ra6m5_iic_set_icier(priv, IIC_ICIER_TEIE | IIC_ICIER_NAKIE | IIC_ICIER_ALIE);

          /* Transmit data byte */

          if (0U != priv->dcnt)
            {
              data = priv->ptr;
              regval = *data;
              ra6m5_iic_set_sending_data(priv, regval);

              priv->dcnt--;
              priv->ptr++;
            }

          else
            {
              /* Do nothing */
            }
        }

      /* Pattern write 3: transmit only slave address */

      else if (0 == priv->msgc)
        {
          priv->dev_sts = RIIC_STS_SP_COND_WAIT;
          ra6m5_iic_set_icier(priv, IIC_ICIER_SPIE | IIC_ICIER_NAKIE | IIC_ICIER_ALIE);

          /* Generate stop condition */

          ra6m5_iic_stopcond(priv);
        }

      else
        {
          /* Do nothing */
        }
    }

  if (RIIC_M_RECEIVE == priv->mode)
    {
      priv->dev_sts = RIIC_STS_RECEIVE_DATA_WAIT;

      ra6m5_iic_set_icier(priv, IIC_ICIER_RIE | IIC_ICIER_NAKIE | IIC_ICIER_ALIE);

      if (2 >= priv->dcnt)
        {
          ra6m5_iic_wait_set(priv);
        }

      if (0x01 >= priv->dcnt)
        {
          ra6m5_iic_pre_end_set(priv);
        }

      /* Dummy read */

      regval = ra6m5_iic_read_data(priv);
    }
}

/****************************************************************************
 * Name: ra6m5_iic_stopcond
 *
 * Description:
 *  Issue stop condition for Master Transmission/Reception
 *
 ****************************************************************************/

static void ra6m5_iic_stopcond(struct ra6m5_i2c_priv_s *priv)
{
  uint8_t regval;

  regval = ra6m5_riic_getreg(priv, RA6M5_IIC_ICSR2_OFFSET);
  if (0x00 != ((regval & IIC_ICSR2_STOP) >> 3U))
    {
      regval &= ~(IIC_ICSR2_STOP);
      ra6m5_riic_putreg(priv, RA6M5_IIC_ICSR2_OFFSET, regval);
      regval = ra6m5_riic_getreg(priv, RA6M5_IIC_ICSR2_OFFSET);
    }

  /* Generates a stop condition. */

  regval = ra6m5_riic_getreg(priv, RA6M5_IIC_ICCR2_OFFSET);
  regval |= IIC_ICCR2_SP; /* Sets ICCR2.SP bit. */
  ra6m5_riic_putreg(priv, regval, RA6M5_IIC_ICCR2_OFFSET);

  regval = ra6m5_riic_getreg(priv, RA6M5_IIC_ICCR2_OFFSET);
}

/****************************************************************************
 * Name: ra6m5_iic_master_transmit
 *
 * Description:
 *  Transmission of data from Master to Slave
 *
 ****************************************************************************/

static void ra6m5_iic_master_transmit(struct ra6m5_i2c_priv_s *priv)
{
  uint8_t regval;
  uint8_t *data;

  priv->dev_sts = RIIC_STS_SEND_DATA_WAIT;

  if (0U == priv->dcnt)
    {
      /* Moving to the next message */

      priv->msgc--;

      if (0x00 == priv->msgc)
        {
          ra6m5_iic_set_icier(priv, IIC_ICIER_SPIE | IIC_ICIER_NAKIE | IIC_ICIER_ALIE);

          priv->dev_sts = RIIC_STS_SP_COND_WAIT;
          ra6m5_iic_stopcond(priv);
        }

      else
        {
          priv->msgv++;
          priv->ptr = priv->msgv->buffer;
          priv->flags = priv->msgv->flags;
          priv->dcnt = priv->msgv->length;

          if (priv->flags == I2C_M_READ)
            {
              priv->mode = RIIC_M_RECEIVE;
              priv->event = RIIC_EV_GEN_START_COND;

              ra6m5_iic_set_icier(priv, IIC_ICIER_STIE | IIC_ICIER_NAKIE | IIC_ICIER_ALIE);

              ra6m5_iic_restartcond(priv);
            }
        }
    }

  if (0U != priv->dcnt && priv->flags != I2C_M_READ)
    {
      ra6m5_iic_set_icier(priv, IIC_ICIER_TEIE | IIC_ICIER_NAKIE | IIC_ICIER_ALIE);

      data = priv->ptr;
      regval = *data;

      /* send the byte */

      ra6m5_iic_set_sending_data(priv, regval);

      /* decrement the count */

      priv->dcnt--;
      priv->ptr++;
    }
}

/****************************************************************************
 * Name: ra6m5_iic_read_data
 *
 * Description:
 *  Read received data from ICDRR
 *
 ****************************************************************************/

static uint8_t ra6m5_iic_read_data(struct ra6m5_i2c_priv_s *priv)
{
  return ra6m5_riic_getreg(priv, RA6M5_IIC_ICDRR_OFFSET);;
}

/****************************************************************************
 * Name: ra6m5_iic_wait_set
 *
 * Description:
 *  Receive "last byte - 2bytes" Setting Proccesing.
 *  Sets ICMR3.WAIT bit.
 *
 ****************************************************************************/

static void ra6m5_iic_wait_set(struct ra6m5_i2c_priv_s *priv)
{
  uint8_t enab = 0;

  if (0 == priv->bus)
    {
#ifdef CONFIG_RA6M5_IIC0_RCV_IN_BYTE_UNITS
      enab = 1;
#endif
    }

  else if (1 == priv->bus)
    {
#ifdef CONFIG_RA6M5_IIC1_RCV_IN_BYTE_UNITS
      enab = 1;
#endif
    }

  else
    {
#ifdef CONFIG_RA6M5_IIC2_RCV_IN_BYTE_UNITS
      enab = 1;
#endif
    }

  if (enab) 
  {
    uint8_t regval;
    regval = ra6m5_riic_getreg(priv, RA6M5_IIC_ICMR3_OFFSET);
    regval |= IIC_ICMR3_WAIT;
    ra6m5_riic_putreg(priv, RA6M5_IIC_ICMR3_OFFSET, regval);
    regval = ra6m5_riic_getreg(priv, RA6M5_IIC_ICMR3_OFFSET);
  }
}

/****************************************************************************
 * Name: ra6m5_iic_pre_end_set
 *
 * Description:
 *  Receive "last byte - 1byte" Setting Processing.
 *  Sets ICMR3.RDRFS bit and ACKBT bit.
 *
 ****************************************************************************/

static void ra6m5_iic_pre_end_set(struct ra6m5_i2c_priv_s *priv)
{
  uint8_t regval = ra6m5_riic_getreg(priv, RA6M5_IIC_ICMR3_OFFSET);

#ifdef CONFIG_RA6M5_IIC0_RCV_IN_BYTE_UNITS
  /* Set the RDRF flag on the rising edge of the 8th SCL clock cycle */
  regval |= IIC_ICMR3_RDRFS;
  ra6m5_riic_putreg(priv, RA6M5_IIC_ICMR3_OFFSET, regval);
#endif
  
  /* Write enable ACKBT bit */
  regval |= IIC_ICMR3_ACKWP;
  ra6m5_riic_putreg(priv, RA6M5_IIC_ICMR3_OFFSET, regval);

  /* Send 1 as the acknowledge bit  */
  regval |= IIC_ICMR3_ACKBT;
  ra6m5_riic_putreg(priv, RA6M5_IIC_ICMR3_OFFSET, regval);

  /* Write disable ACKBT bit */
  regval &= ~IIC_ICMR3_ACKWP;
  ra6m5_riic_putreg(priv, RA6M5_IIC_ICMR3_OFFSET, regval);

  /* Read back */
  ra6m5_riic_getreg(priv, RA6M5_IIC_ICMR3_OFFSET);
}

/****************************************************************************
 * Name: ra6m5_iic_end_set
 *
 * Description:
 *  Receive End Setting Processing.
 *  Sets ICMR3.ACKBT bit and clears ICMR3.WAIT bit.
 *
 ****************************************************************************/

static void ra6m5_iic_end_set(struct ra6m5_i2c_priv_s *priv)
{
  uint8_t regval = ra6m5_riic_getreg(priv, RA6M5_IIC_ICMR3_OFFSET);

  /* Write enable ACKBT bit */
  regval |= IIC_ICMR3_ACKWP;
  ra6m5_riic_putreg(priv, RA6M5_IIC_ICMR3_OFFSET, regval);

  /* Send 1 as the acknowledge bit  */
  regval |= IIC_ICMR3_ACKBT;
  ra6m5_riic_putreg(priv, RA6M5_IIC_ICMR3_OFFSET, regval);

  /* Write disable ACKBT bit */
  regval &= ~IIC_ICMR3_ACKWP;
  ra6m5_riic_putreg(priv, RA6M5_IIC_ICMR3_OFFSET, regval);

#ifdef CONFIG_RA6M5_IIC0_RCV_IN_BYTE_UNITS
  /* No wait ( */
  regval &= ~IIC_ICMR3_WAIT;
  ra6m5_riic_putreg(priv, RA6M5_IIC_ICMR3_OFFSET, regval);
#endif

  /* Read back */
  ra6m5_riic_getreg(priv, RA6M5_IIC_ICMR3_OFFSET);
}

/****************************************************************************
 * Name: ra6m5_iic_master_receive
 *
 * Description:
 *  Transmission of data from Master to Slave
 *
 ****************************************************************************/

static void ra6m5_iic_master_receive(struct ra6m5_i2c_priv_s *priv)
{
  uint8_t regval;
  priv->dev_sts = RIIC_STS_RECEIVE_DATA_WAIT;

  /* The period between the ninth clock cycle and the first clock cycle
   * is held low. Low-hold is released by reading the ICDRR register.
   */

  if (0x03 >= priv->dcnt)
    {
      ra6m5_iic_wait_set(priv);
    }

  if (0x02 >= priv->dcnt)
    {
      ra6m5_iic_pre_end_set(priv);
    }

  if (0x01 >= priv->dcnt)
    {
      ra6m5_iic_set_icier(priv, IIC_ICIER_SPIE | IIC_ICIER_NAKIE | IIC_ICIER_ALIE);

      priv->dev_sts = RIIC_STS_SP_COND_WAIT;
      ra6m5_iic_stopcond(priv);

      if (0x00 != priv->dcnt)
        {
          regval = ra6m5_iic_read_data(priv);
          *(priv->msgv->buffer) = regval;
          priv->msgv->buffer++;
          *(priv->ptr) = regval;
          priv->dcnt--;
        }

      else
        {
          regval = ra6m5_iic_read_data(priv);
        }

      ra6m5_iic_end_set(priv);
    }

  else
    {
      regval = ra6m5_iic_read_data(priv);
      *(priv->msgv->buffer) = regval;
      priv->msgv->buffer++;
      *(priv->ptr) = regval;
      priv->dcnt--;
    }
}

/****************************************************************************
 * Name: ra6m5_iic_rxi_isr
 *
 * Description:
 *  Interrupt RXI handler – Received Data Full Interrupt Handler
 *
 * Occurs under following conditions:
 *  - Address/data transmission completed in Master Receive Mode
 *  - Reception of (last data – 1) completed in Master Receive Mode
 *  - Reception of last data completed in Master Receive Mode
 *
 ****************************************************************************/

static int ra6m5_iic_rxi_isr(int irq, void *context, void *arg)
{
  struct ra6m5_i2c_priv_s *priv = (struct ra6m5_i2c_priv_s *)arg;

  priv->event = RIIC_EV_INT_RECEIVE;

  ra6m5_iic_advance(priv);
  return OK;
}

/****************************************************************************
 * Name: ra6m5_iic_txi_isr
 *
 * Description:
 *  Interrupt TEI handler – Transmit Data Empty Interrupt Handler
 *
 * Occurs under following conditions:
 *  - Transmit Buffer is empty
 *
 ****************************************************************************/

static int ra6m5_iic_txi_isr(int irq, void *context, void *arg)
{
  struct ra6m5_i2c_priv_s *priv = (struct ra6m5_i2c_priv_s *)arg;

  /* Ideally, should never get here as this interrupt only occurs during
   * Multi-master mode of operation and
   * Slave transmission/recption mode
   */

  priv->event = RIIC_EV_INT_TXI;

  ra6m5_iic_advance(priv);
  return OK;
}

/****************************************************************************
 * Name: ra6m5_iic_tei_isr
 *
 * Description:
 *  Interrupt TEI handler – Transmission End Interrupt Handler
 *
 * Occurs under following conditions:
 *  - Address/data transmission completed
 *
 ****************************************************************************/

static int ra6m5_iic_tei_isr(int irq, void *context, void *arg)
{
  struct ra6m5_i2c_priv_s *priv = (struct ra6m5_i2c_priv_s *)arg;
  uint8_t regval;

  /* Clear the TEND flag */
  regval  = ra6m5_riic_getreg(priv, RA6M5_IIC_ICSR2_OFFSET);
  regval &= ~IIC_ICSR2_TEND;
  ra6m5_riic_putreg(priv, RA6M5_IIC_ICSR2_OFFSET, regval);

  while (ra6m5_riic_getreg(priv, RA6M5_IIC_ICSR2_OFFSET) & IIC_ICSR2_TEND)
    {
      /* Do Nothing */
    }

  if ((RIIC_STS_SEND_SLVADR_W_WAIT == priv->dev_sts) ||
     (RIIC_STS_SEND_SLVADR_R_WAIT == priv->dev_sts))
    {
      /* Sets interrupted address sending - slave address transmitted */

      priv->event = RIIC_EV_INT_ADD;
    }

  else if (RIIC_STS_SEND_DATA_WAIT == priv->dev_sts)
    {
      /* Sets interrupted data sending */

      priv->event = RIIC_EV_INT_SEND;
    }

  ra6m5_iic_advance(priv);
  return OK;
}

/****************************************************************************
 * Name: ra6m5_iic_eri_isr
 *
 * Description:
 *  Interrupt ERI handler – Event/Error Generation Interrupt Handler
 *
 * Occurs under following conditions:
 *  - START condition detected
 *  - RESTART condition detected
 *  - STOP condition detected
 *  - NACK detected
 *  - AL (arbitration-lost) detected
 *  - TMO (timeout detection)
 *
 ****************************************************************************/

static int ra6m5_iic_eri_isr(int irq, void *context, void *arg)
{
  struct ra6m5_i2c_priv_s *priv = (struct ra6m5_i2c_priv_s *)arg;
 
  /* Get the status flag */
  uint8_t stat = ra6m5_riic_getreg(priv, RA6M5_IIC_ICSR2_OFFSET);

  /* Get the interrupt enable flag */
  uint8_t enab = ra6m5_riic_getreg(priv, RA6M5_IIC_ICIER_OFFSET);

  /* Check Timeout Condition */

  if ((stat & IIC_ICSR2_TMOF) && (enab & IIC_ICIER_TMOIE))
    {
      /* Disable the Timeout Interrupt */

      enab &= ~IIC_ICIER_TMOIE;
      ra6m5_riic_putreg(priv, RA6M5_IIC_ICIER_OFFSET, enab);

      while ((enab = ra6m5_riic_getreg(priv, RA6M5_IIC_ICIER_OFFSET)) & IIC_ICIER_TMOIE)
        {
          /* Wait for reset to complete */
        }

      priv->event = RIIC_EV_INT_TMO;
    }

  /* Check Arbitration-Lost Condition */

  if ((stat & IIC_ICSR2_AL) && (enab & IIC_ICIER_ALIE))
    {
      /* Disable the Arbitration-Lost Interrupt */

      enab &= ~IIC_ICIER_ALIE;
      ra6m5_riic_putreg(priv, RA6M5_IIC_ICIER_OFFSET, enab);

      while ((enab = ra6m5_riic_getreg(priv, RA6M5_IIC_ICIER_OFFSET)) & IIC_ICIER_ALIE)
        {
          /* Do Nothing */
        }

      priv->event = RIIC_EV_INT_AL;
    }

  /* Check Stop Condition */

  if ((stat & IIC_ICSR2_STOP) && (enab & IIC_ICIER_SPIE))
    {
      /* Disable Stop Condition Detection Interrupt */

      enab &= ~IIC_ICIER_SPIE;
      ra6m5_riic_putreg(priv, RA6M5_IIC_ICIER_OFFSET, enab);

      /* Clear RDRFS, ACKBT */

      uint8_t icmr3 = ra6m5_riic_getreg(priv, RA6M5_IIC_ICMR3_OFFSET);
      icmr3 &= IIC_ICMR3_RDRFS;
      ra6m5_riic_putreg(priv, RA6M5_IIC_ICMR3_OFFSET, icmr3);

      icmr3 |= IIC_ICMR3_ACKWP;
      ra6m5_riic_putreg(priv, RA6M5_IIC_ICMR3_OFFSET, icmr3);

      icmr3 &= ~IIC_ICMR3_ACKBT;
      ra6m5_riic_putreg(priv, RA6M5_IIC_ICMR3_OFFSET, icmr3);

      icmr3 &= ~IIC_ICMR3_ACKWP;
      ra6m5_riic_putreg(priv, RA6M5_IIC_ICMR3_OFFSET, icmr3);

      /* Wait for done */
      do
      {
        icmr3 = ra6m5_riic_getreg(priv, RA6M5_IIC_ICMR3_OFFSET);
      } while (icmr3 & (IIC_ICMR3_RDRFS | IIC_ICMR3_ACKBT));
      

      /* Clear Stop Condition Detection */

      stat &= ~IIC_ICSR2_STOP;
      ra6m5_riic_putreg(priv, RA6M5_IIC_ICSR2_OFFSET, stat);

      while ((stat = ra6m5_riic_getreg(priv, RA6M5_IIC_ICSR2_OFFSET)) & IIC_ICSR2_STOP)
        {
          /* Do Nothing */
        }

      priv->event = RIIC_EV_INT_STOP;
    }

  /* Check NACK reception. */

  if ((stat & IIC_ICSR2_NACKF) && (enab & IIC_ICIER_NAKIE))
    {
      /* Prohibits NACK interrupt to generate stop condition. */

      enab &= ~IIC_ICIER_NAKIE;
      ra6m5_riic_putreg(priv, RA6M5_IIC_ICIER_OFFSET, enab);

      /* Prohibits these interrupt.
       * After NACK interrupt, these interrupts will occur
       * when they do not stop the following interrupts.
       */

      enab &= ~ (IIC_ICIER_TEIE | IIC_ICIER_TIE | IIC_ICIER_RIE);
      ra6m5_riic_putreg(priv, RA6M5_IIC_ICIER_OFFSET, enab);

      /* Wait for done */
      do
      {
        enab = ra6m5_riic_getreg(priv, RA6M5_IIC_ICIER_OFFSET);
      } while (enab & (IIC_ICIER_TEIE | IIC_ICIER_TIE | IIC_ICIER_RIE));

      if (!(ra6m5_riic_getreg(priv, RA6M5_IIC_ICCR2_OFFSET) & IIC_ICCR2_TRS))
        {
          //IR(RIIC0, RXI0) = 0;
        }

      priv->event = RIIC_EV_INT_NACK;
    }

  /* Check Start condition detection. */

  if ((stat & IIC_ICSR2_START) && (enab & IIC_ICIER_STIE))
    {
      /* Disable Start Condition Detection Interrupt. */

      enab &= ~IIC_ICIER_STIE;
      ra6m5_riic_putreg(priv, RA6M5_IIC_ICIER_OFFSET, enab);

      /* Clear Start Condition Detection */

      stat &= ~IIC_ICSR2_START;
      ra6m5_riic_putreg(priv, RA6M5_IIC_ICSR2_OFFSET, stat);

      /* Wait till reset is completed */
      do 
      {
        stat = ra6m5_riic_getreg(priv, RA6M5_IIC_ICSR2_OFFSET);
        enab = ra6m5_riic_getreg(priv, RA6M5_IIC_ICIER_OFFSET);

      } while ((stat & IIC_ICSR2_START) || (enab & IIC_ICIER_STIE));

      /* Sets event flag. */

      priv->event = RIIC_EV_INT_START;
    }

  ra6m5_iic_advance(priv);
  return OK;
}

/****************************************************************************
 * Name: ra6m5_i2c_transfer
 *
 * Description:
 *   This function initializes the RIIC channel
 *
 ****************************************************************************/

static int ra6m5_i2c_transfer(struct i2c_master_s *dev,
                              struct i2c_msg_s *msgs, int count)
{
  struct ra6m5_i2c_priv_s *priv = (struct ra6m5_i2c_priv_s *)dev;
  int ret = 0;

  DEBUGASSERT(dev != NULL && msgs != NULL && count > 0);

  /* Get exclusive access to the I2C bus */

  nxmutex_lock(&priv->lock);

  priv->mode = RIIC_READY;
  priv->dev_sts = RIIC_STS_IDLE;
  priv->event = RIIC_EV_NONE;

  priv->dcnt = 0;
  priv->ptr = NULL;

  priv->msgv = msgs;
  priv->msgc = count;

  priv->ptr = priv->msgv->buffer;
  priv->dcnt = priv->msgv->length;
  priv->flags = priv->msgv->flags;

  /* Set reference clock for I2C bus */

  ra6m5_iic_setclock(priv, msgs->frequency);

  if ((priv->flags == 0) || (priv->flags == I2C_M_NOSTOP)
     || (priv->flags == I2C_M_TEN))
    {
      /* MASTER TRANSMISSION MODE
       * No communication is in progress
       * Initiate START condition and proceed with transfer
       */

      if (priv->mode == RIIC_READY)
        {
          /* Device is ready for communication */

          priv->event = RIIC_EV_GEN_START_COND;
          ret = ra6m5_iic_startcond(priv);
          if (ret == RIIC_ERR_BUS_BUSY)
            {
              priv->mode = RIIC_NONE;
              priv->dev_sts = RIIC_STS_NO_INIT;
              priv->event = RIIC_EV_NONE;

              /* Disable interrupts */

              ra6m5_iic_int_disable(priv);
            }
        }
    }

  else if (priv->flags == I2C_M_READ)
    {
      /* MASTER RECEPTION MODE
       * No communication is in progress
       * Initiate START condition and proceed with transfer
       */

      if (priv->mode == RIIC_READY)
        {
          /* Device is ready for communication */

          priv->event = RIIC_EV_GEN_START_COND;
          ret = ra6m5_iic_startcond(priv);
          if (ret == RIIC_ERR_BUS_BUSY)
            {
              priv->mode = RIIC_NONE;
              priv->dev_sts = RIIC_STS_NO_INIT;
              priv->event = RIIC_EV_NONE;

              /* Disable interrupts */

              ra6m5_iic_int_disable(priv);
            }
        }
    }

  else if (priv->flags == I2C_M_NOSTART)
    {
      /* COMMUNICATION IS IN PROGRESS */

      if (priv->mode == RIIC_M_TRANSMIT)
        {
          priv->ptr = priv->msgv->buffer;
          priv->dcnt = priv->msgv->length;
          ra6m5_iic_master_transmit(priv);
        }

      else if (priv->mode == RIIC_M_RECEIVE)
        {
          ra6m5_iic_master_receive(priv);
        }
    }

  nxmutex_unlock(&priv->lock);

  while (RIIC_FINISH != priv->mode && RIIC_NONE != priv->mode);

  if (ra6m5_riic_getreg(priv, RA6M5_IIC_ICSR2_OFFSET) & IIC_ICSR2_NACKF)
    {
      ret = -ENXIO;
    }

  else if ((ra6m5_riic_getreg(priv, RA6M5_IIC_ICCR2_OFFSET) & IIC_ICCR2_BBSY) || (RIIC_ERR_BUS_BUSY == ret))
    {
      ret = -EBUSY;
    }

  else
    {
      ret = RIIC_SUCCESS;
    }

  ra6m5_iic_iicrst(priv);
  return ret;
}

/****************************************************************************
 * Name: ra6m5_i2c_reset
 *
 * Description:
 *   Resets the RIIC channel
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_RESET
static int ra6m5_i2c_reset(struct i2c_master_s *dev)
{
  struct ra6m5_i2c_priv_s *priv = (struct ra6m5_i2c_priv_s *)dev;
  uint8_t regval;

  DEBUGASSERT(dev);

  DEBUGASSERT(priv->refs > 0);

  nxmutex_lock(&priv->lock);

  /* Assert the Internal Reset */
  regval = ra6m5_riic_getreg(priv, RA6M5_IIC_ICCR1_OFFSET);
  regval |= IIC_ICCR1_IICRST; 
  ra6m5_riic_putreg(priv, regval, RA6M5_IIC_ICCR1_OFFSET);
  regval = ra6m5_riic_getreg(priv, RA6M5_IIC_ICCR1_OFFSET);

  /* Release the Internal Reset */
  regval = ra6m5_riic_getreg(priv, RA6M5_IIC_ICCR1_OFFSET);
  regval &= ~(IIC_ICCR1_IICRST);
  ra6m5_riic_putreg(priv, regval, RA6M5_IIC_ICCR1_OFFSET);
  regval = ra6m5_riic_getreg(priv, RA6M5_IIC_ICCR1_OFFSET);

  nxmutex_unlock(&priv->lock);
  return OK;
}
#endif /* CONFIG_I2C_RESET */

/****************************************************************************
 * Name: ra6m5_i2cbus_initialize
 *
 * Description:
 *   This function initializes the RIIC channel
 *
 ****************************************************************************/

struct i2c_master_s *ra6m5_i2cbus_initialize(int channel)
{
  struct ra6m5_i2c_priv_s *priv = NULL;

  /* Get I2C private structure */

  i2cinfo("RA6M5 RIIC Bus Initialization:\n");
  riic_mpc_enable();
  switch (channel)
    {
#ifdef CONFIG_RA6M5_IIC0
      case 0:
        priv = (struct ra6m5_i2c_priv_s *)&ra6m5_riic0_priv;
        i2cinfo("RA6M5 RIIC0 Channel Initial Setup\n");
        break;
#endif
#ifdef CONFIG_RA6M5_IIC1
      case 1:
        priv = (struct ra6m5_i2c_priv_s *)&ra6m5_riic1_priv;
        i2cinfo("RA6M5 RIIC1 Channel Initial Setup\n");
        break;
#endif
#ifdef CONFIG_RA6M5_IIC2
      case 2:
        priv = (struct ra6m5_i2c_priv_s *)&ra6m5_riic2_priv;
        i2cinfo("RA6M5 RIIC2 Channel Initial Setup\n");
        break;
#endif
      default:
        i2cerr("Channel %d is not supported by RA6M5\n", channel);
        riic_mpc_disable();
        return NULL;
    }

  /* Initialize private data for the first time, increment reference count,
   * initialize RIIC registers and attach IRQs
   */

  nxmutex_lock(&priv->lock);

  if (priv->refs++ == 0)
    {
      /* Initialize the RIIC registers */

      ra6m5_iic_init(priv);
    }

  riic_mpc_disable();
  nxmutex_unlock(&priv->lock);

  return (struct i2c_master_s *)priv;
}

/****************************************************************************
 * Name: ra6m5_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialize an RIIC device
 *
 ****************************************************************************/

int ra6m5_i2cbus_uninitialize(struct i2c_master_s *dev)
{
  struct ra6m5_i2c_priv_s *priv = (struct ra6m5_i2c_priv_s *)dev;

  DEBUGASSERT(dev);

  /* Decrement reference count and check for underflow */

  if (priv->refs == 0)
    {
      return ERROR;
    }

  nxmutex_lock(&priv->lock);
  if (--priv->refs)
    {
      nxmutex_unlock(&priv->lock);
      return OK;
    }

  /* Unconfigure GPIO pins */

  ra6m5_unconfiggpio(priv->dev->scl_pin);
  ra6m5_unconfiggpio(priv->dev->sda_pin);

  /* Disable power and other HW resource (GPIO's) */

  ra6m5_iic_int_disable(priv);

  irq_detach(priv->dev->txi_irq);
  irq_detach(priv->dev->rxi_irq);
  irq_detach(priv->dev->tei_irq);
  irq_detach(priv->dev->eri_irq);

  nxmutex_unlock(&priv->lock);
  return OK;
}

#endif /* CONFIG_RA6M5_IIC0 || CONFIG_RA6M5_IIC1 || CONFIG_RA6M5_IIC2 */
