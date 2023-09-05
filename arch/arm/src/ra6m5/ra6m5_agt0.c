/****************************************************************************
 * arch/arm/src/ra6m5/ra6m5_agt0.c
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

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/irq.h>

#include "ra6m5_agt0.h"
#include "ra6m5_rcc.h"
#include "ra6m5_eth.h"
#include "hardware/ra6m5_tim.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra6m5_agt0_create
 *
 * Description:
 * AGT0 Timer Initialization
 ****************************************************************************/

void ra6m5_agt0_create(uint32_t txpoll_time, uint32_t txtimeout_time)
{
  uint8_t regval;

  /* Disable AGT0 interrupts */

  up_disable_irq(RA6M5_IRQ_AGT0_INT);
  up_disable_irq(RA6M5_IRQ_AGT0_CMPA);
  up_disable_irq(RA6M5_IRQ_AGT0_CMPB);

  /* Cancel stop state of timer */

  modifyreg32(RA6M5_MSTP_REG(RA6M5_MSTP_MSTPCRD_OFFSET), MSTP_MSTPCRD_AGT0, 0);

  /* Stop AGT0 count */

  putreg8(4, RA6M5_AGT0_AGTCR);
  up_udelay(100);

  /* Wait till stop */

  do {
    regval = getreg8(RA6M5_AGT0_AGTCR);
  } while (regval & AGT_AGTCR_TCSTF);

  /* Set timer I/O control register */

  putreg8(AGT_AGTCMSR_TCMEA | 
          AGT_AGTCMSR_TCMEB, RA6M5_AGT0_AGTCMSR);

  /* Set output compare register A */

  putreg16(txpoll_time, RA6M5_AGT0_AGTCMA);

  /* Set output compare register B */

  putreg16(txtimeout_time, RA6M5_AGT0_AGTCMB);

  /* Set control registers */

  /* AGTLCLK = PCLKB/128 */

  putreg8(0x40, RA6M5_AGT0_AGTMR1);
  putreg8(0x07, RA6M5_AGT0_AGTMR2);

  /* Enable Compare Match B interrupt */

  up_enable_irq(RA6M5_IRQ_AGT0_CMPB);

  /* Attach the IRQ for tx timeout */

  irq_attach(RA6M5_IRQ_AGT0_CMPB, (xcpt_t)ra6m5_txtimeout_expiry, NULL);
}

/****************************************************************************
 * Name: ra6m5_agt0_start
 *
 * Description:
 * AGT0 Timer Initialization
 ****************************************************************************/

void ra6m5_agt0_start(uint8_t type, uint32_t timeout)
{
  /* Update Output Compare A I/O for polling */

  if (type == ra6m5_agt0_txpoll)
    {
      putreg16(getreg16(RA6M5_AGT0_AGT) + timeout, RA6M5_AGT0_AGTCMA);

      /* Enabling Compare Match A */

      modifyreg8(RA6M5_AGT0_AGTCMSR, 0, AGT_AGTCMSR_TCMEA);
    }

  /* Update Output Compare B I/O for timeout */

  if (type == ra6m5_agt0_timeout)
    {
      putreg16(getreg16(RA6M5_AGT0_AGT) + timeout, RA6M5_AGT0_AGTCMB);

      /* Enabling Compare Match B */

      modifyreg8(RA6M5_AGT0_AGTCMSR, 0, AGT_AGTCMSR_TCMEB);
    }

  /* Start AGT0 count */

  modifyreg8(RA6M5_AGT0_AGTCR, 0, AGT_AGTCR_TSTART);
}

/****************************************************************************
 * Name: ra6m5_agt0_stop
 *
 * Description:
 * AGT0 Timer Initialization
 ****************************************************************************/

void ra6m5_agt0_stop(uint8_t type)
{
  /* Stop Output Compare A I/O for polling */

  if (type == ra6m5_agt0_txpoll)
    {
      /* Disabling Output Compare A */

      modifyreg8(RA6M5_AGT0_AGTCMSR, AGT_AGTCMSR_TCMEA, 0);
      modifyreg8(RA6M5_AGT0_AGTCR, AGT_AGTCR_TCMAF, 0);
    }

  /* Stop Output Compare B I/O for timeout */

  if (type == ra6m5_agt0_timeout)
    {
      /* Disabling Output Compare B */

      modifyreg8(RA6M5_AGT0_AGTCMSR, AGT_AGTCMSR_TCMEB, 0);
      modifyreg8(RA6M5_AGT0_AGTCR, AGT_AGTCR_TCMBF, 0);
    }
}
