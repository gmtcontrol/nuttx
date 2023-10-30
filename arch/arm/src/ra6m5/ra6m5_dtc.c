/****************************************************************************
 * arch/arm/src/ra6m5/ra6m5_dtc.c
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

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include <arch/board/board.h>

#include "ra6m5_rcc.h"
#include "ra6m5_dtc.h"
#include "hardware/ra6m5_dtc.h"
#include "hardware/ra6m5_icu.h"

#if defined(CONFIG_RA6M5_DTC)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DTC_MODULE_STOP             (1 << 22)
#define DTC_NCHANNELS               (1)
#define DTC_CHANNEL_ID              (0)
#define DTC_NUM_INTERRUPT_SRC       (RA6M5_IRQ_NEXTINTS)
#define DTC_VECTOR_ADDRESS_ALIGN    (0x400)     /* lower 10 bits of the base address */
#define DTC_VECTOR_ADDRESS_MASK     (0xfffffc00)
#define DTC_MIN_VECT_NUMBER         (0)
#define DTC_MAX_VECT_NUMBER         (95)
#define DTC_MIN_COUNT_VAL           (1)         /* The minimum of count value  and block size */
#define DTC_MAX_8BITS_COUNT_VAL     (256)       /* The maximum value of 8bit count value */
#define DTC_MAX_16BITS_COUNT_VAL    (65536)     /* The maximum value of 16bit count value */

#define DTC_VECTOR_NUMBER           (256)
#define DTC_EACH_VECTOR_SIZE        (4)
#define DTC_VECTOR_TABLE_SIZE       ((DTC_VECTOR_NUMBER) * (DTC_EACH_VECTOR_SIZE))

#define DTC_VECTOR_TABLE_SIZE_BYTES (DTC_VECTOR_ADDRESS_ALIGN + DTC_VECTOR_TABLE_SIZE)


/****************************************************************************
 * Typedef definitions
 ****************************************************************************/

/* The DTC Mode Internal Register A (MRA) structure */

struct st_dtc_mra_bit
{
  uint8_t rs:2;     /* reserved */
  uint8_t SM:2;     /* Transfer Source Address Addressing Mode */
  uint8_t SZ:2;     /* DTC Data Transfer Size */
  uint8_t MD:2;     /* DTC Transfer Mode Select */
};

typedef union dtc_mra
{
  uint8_t BYTE;
  struct st_dtc_mra_bit BIT;
} dtc_mra_t;

/* The DTC Mode Internal Register B (MRB) structure */

struct st_dtc_mrb_bit
{
  uint8_t rs   :2;  /* reserved */
  uint8_t DM   :2;  /* Transfer Destination Address Addressing Mode */
  uint8_t DTS  :1;  /* DTC Transfer Mode Select */
  uint8_t DISEL:1;  /* DTC Interrupt Select */
  uint8_t CHNS :1;  /* DTC Chain Transfer Select */
  uint8_t CHNE :1;  /* DTC Chain Transfer Enable */
};

typedef union dtc_mrb
{
  uint8_t BYTE;
  struct st_dtc_mrb_bit BIT;
} dtc_mrb_t;

/* The DTC Transfer Count Register A (CRA) structure */

struct st_dtc_cra_byte
{
  uint8_t CRA_L;
  uint8_t CRA_H;
};

typedef union dtc_cra
{
  uint16_t WORD;
  struct st_dtc_cra_byte BYTE;
} dtc_cra_t;

/* The DTC Transfer Count Register B (CRB) structure */

typedef union dtc_crb
{
  uint16_t WORD;
} dtc_crb_t;

struct st_first_lword
  {
  uint16_t  reserver;   /* reserve area */
  dtc_mrb_t MRB;
  dtc_mra_t MRA;
  };

struct st_fourth_lword
  {
    dtc_crb_t CRB;
    dtc_cra_t CRA;
  };

typedef union lword1
  {
    uint32_t LWORD;
    struct st_first_lword REG;
  } lword1_t;

typedef union lword2
  {
    uint32_t SAR;
  }lword2_t;

typedef union lword3
  {
    uint32_t DAR;
  } lword3_t;

typedef union lword4
  {
    uint32_t LWORD;
    struct st_fourth_lword REG;
  } lword4_t;

typedef struct st_dtc_full_transfer_data
  {
    lword1_t FIRST_LWORD;
    lword2_t SECOND_LWORD;
    lword3_t THIRD_LWORD;
    lword4_t FOURTH_LWORD;
  } st_dtc_internal_registers_t;

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes one dtc channel */

struct ra6m5_dtc_s
{
  uint8_t   chan;           /* DTC channel number */
  bool      initialized;    /* Initialization status */
  uint32_t  base;           /* DTC channel register base address */
  uint8_t * vectortable;    /* Vector table pointer */
  uint8_t   addmode;        /* Address mode */
#if defined(CONFIG_RA6M5_DTC_TRANSFER_DATA_READ_SKIP)
  uint8_t   readskip;       /* Read skip enable or disable */
#endif
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Statically allocated vector table */

uint8_t vectortable[DTC_VECTOR_TABLE_SIZE_BYTES];

/* This array describes the state of each dtc: Only one channel */

static struct ra6m5_dtc_s g_dtchandle[DTC_NCHANNELS] =
{
  {
    .chan  = DTC_CHANNEL_ID,
    .base  = RA6M5_DTC_BASE,
    .vectortable = vectortable,
  },
};

/* The array of all interrupt source */

const dtc_activation_source_t g_dtcsource[DTC_NUM_INTERRUPT_SRC] =
{
    0
};


/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void ra6m5_dtc_module_enable(void);
static dtc_err_t ra6m5_dtc_set_dynamic_transfer_data(
                        dtc_dynamic_transfer_data_cfg_t * p_transfer_cfg,
                        dtc_transfer_data_t * p_transfer_data);
static dtc_err_t ra6m5_dtc_validate_dynamic_params(
                        dtc_dynamic_transfer_data_cfg_t * p_transfer_cfg,
                        dtc_transfer_data_t * p_transfer_data);
static dtc_err_t ra6m5_dtc_set_static_transfer_data(
                        dtc_static_transfer_data_cfg_t * p_transfer_cfg,
                        dtc_transfer_data_t * p_transfer_data);
static void ra6m5_dtc_readskip_enable(DTC_HANDLE handle);
#ifndef CONFIG_RA6M5_DTC_TRANSFER_DATA_READ_SKIP
static void ra6m5_dtc_readskip_disable(DTC_HANDLE handle);
#endif


/****************************************************************************
 * DMA register access functions
 ****************************************************************************/

/* Read register from DTC */

static inline uint8_t dtcchan_getreg(struct ra6m5_dtc_s *dtc, uint32_t offset)
{
  return getreg8(dtc->base + offset);
}

static inline uint16_t dtcchan_getreg16(struct ra6m5_dtc_s *dtc, uint32_t offset)
{
  return getreg16(dtc->base + offset);
}

static inline uint32_t dtcchan_getreg32(struct ra6m5_dtc_s *dtc, uint32_t offset)
{
  return getreg32(dtc->base + offset);
}

/* Write to DTC register */

static inline void dtcchan_putreg(struct ra6m5_dtc_s *dtc, uint32_t offset, uint8_t value)
{
  putreg8(value, dtc->base + offset);
}

static inline void dtcchan_putreg16(struct ra6m5_dtc_s *dtc, uint32_t offset, uint16_t value)
{
  putreg16(value, dtc->base + offset);
}

static inline void dtcchan_putreg32(struct ra6m5_dtc_s *dtc, uint32_t offset, uint32_t value)
{
  putreg32(value, dtc->base + offset);
}


/****************************************************************************
 * Function Name: ra6m5_dtc_module_enable
 * Description  : Releases module stop state.
 * Arguments    : None
 * Return Value : None
 ****************************************************************************/

static void ra6m5_dtc_module_enable(void)
{
  /* Enable writing to MSTP registers. */

  /* Release from module stop state. */

  modifyreg32(RA6M5_MSTP_REG(RA6M5_MSTP_MSTPCRA_OFFSET), DTC_MODULE_STOP, 0);
}

/****************************************************************************
 * Function Name: ra6m5_dtc_set_dynamic_transfer_data
 * Description  :
 *       Set the dynamic dynamic transfer parameter in transfer data
 *
 * Input Parameters :
 *    p_transfer_cfg - Reguested configuration
 *    p_transfer_data  - Transfer information RAM memory
 *
 * Return :
 *     dtc_err_t -  error codes
 ****************************************************************************/

static dtc_err_t ra6m5_dtc_set_dynamic_transfer_data(
                    dtc_dynamic_transfer_data_cfg_t * p_transfer_cfg,
                    dtc_transfer_data_t * p_transfer_data)
{
  /* Map transfer data to st_dtc_internal_registers_t */

  volatile st_dtc_internal_registers_t *td_ptr
            = (volatile st_dtc_internal_registers_t *)p_transfer_data;

  /* Get  MRA, CRA and CRB internal register from td_ptr */

  volatile dtc_mra_t *p_mra = &(td_ptr->FIRST_LWORD.REG.MRA);
  volatile dtc_cra_t *p_cra = &(td_ptr->FOURTH_LWORD.REG.CRA);
  volatile dtc_crb_t *p_crb = &(td_ptr->FOURTH_LWORD.REG.CRB);

  /* DTC data transfer size */

  p_mra->BYTE &= ~DTC_MRA_SZ_MASK;
  p_mra->BYTE |= DTC_MRA_SZ(p_transfer_cfg->data_size);

  /* Set CRA and CRB internal register */

  switch (p_mra->BIT.MD) /* DTC transfer mode */
    {
      case 0x0: /* Normal mode */
        {
            if (DTC_MAX_16BITS_COUNT_VAL == p_transfer_cfg->transfer_count)/* Transfer count = 65536 */
            {
                p_cra->WORD = 0x0000;
            }
            else /* 1 - 65535 */
            {
                p_cra->WORD = (uint16_t)p_transfer_cfg->transfer_count;
            }
        break;
        }

        case 0x1: /* Repeat mode */
        {
            /* Set counter. */

            if (p_transfer_cfg->transfer_count < DTC_MAX_8BITS_COUNT_VAL) /* count 1-255 */
            {
                p_cra->BYTE.CRA_H = (uint8_t)p_transfer_cfg->transfer_count;
                p_cra->BYTE.CRA_L = (uint8_t)p_transfer_cfg->transfer_count;
            }
            else if (DTC_MAX_8BITS_COUNT_VAL
                     == p_transfer_cfg->transfer_count)
            {
                p_cra->BYTE.CRA_H = 0x00;
                p_cra->BYTE.CRA_L = 0x00;
            }
            else /* Transfer count > 256 */
            {
                return DTC_ERR_INVALID_ARG;
            }
        break;
        }

        case 0x2: /* DTC_TRANSFER_MODE_BLOCK - Block transfer mode */
        {
            /* Set counter. */

            if (DTC_MAX_16BITS_COUNT_VAL == p_transfer_cfg->transfer_count) /* Transfer count = 65536 */
            {
                p_crb->WORD = 0x0000;
            }
            else /* 1 - 65535 */
            {
                p_crb->WORD = (uint16_t)p_transfer_cfg->transfer_count;
            }

            if (p_transfer_cfg->block_size < DTC_MAX_8BITS_COUNT_VAL) /* Block size 1-255 */
            {
                p_cra->BYTE.CRA_H = (uint8_t)p_transfer_cfg->block_size;
                p_cra->BYTE.CRA_L = (uint8_t)p_transfer_cfg->block_size;
            }
            else if (DTC_MAX_8BITS_COUNT_VAL == p_transfer_cfg->block_size) /* Block size = 256 */
            {
                p_cra->BYTE.CRA_H = 0;
                p_cra->BYTE.CRA_L = 0;
            }
            else /* Invalid block size */
            {
                return DTC_ERR_INVALID_ARG;
            }
        break;
        }

        default:
        {
            return DTC_ERR_INVALID_ARG;
        break;
        }
    }

  /* Set Source and destination address in transfer memory area */

  /* settings for second long word: SAR */

  td_ptr->SECOND_LWORD.SAR = p_transfer_cfg->source_addr; /* 4 byte SAR */

  /* settings for third long word: DAR */

  td_ptr->THIRD_LWORD.DAR = p_transfer_cfg->dest_addr;    /* 4 byte DAR */

  return DTC_SUCCESS;
}

/****************************************************************************
 * Function Name: ra6m5_dtc_validate_dynamic_params
 * Description  :
 *       Validate the dynamic parameter
 *
 * Input Parameters :
 *    p_transfer_cfg - Reguested configuration
 *    p_transfer_data  - Transfer information RAM memory
 *
 * Return :
 *     dtc_err_t -  error codes
 *
 ****************************************************************************/

static dtc_err_t ra6m5_dtc_validate_dynamic_params(
                    dtc_dynamic_transfer_data_cfg_t * p_transfer_cfg,
                    dtc_transfer_data_t * p_transfer_data)
{
  if ((NULL == p_transfer_cfg) || (NULL == p_transfer_data))
    {
      return DTC_ERR_NULL_PTR;
    }

  /* Validate transfer count */

  if ((p_transfer_cfg->transfer_count < DTC_MIN_COUNT_VAL) ||
        (p_transfer_cfg->transfer_count > DTC_MAX_16BITS_COUNT_VAL))
    {
      return DTC_ERR_INVALID_ARG;
    }

  return DTC_SUCCESS;
}

/****************************************************************************
 * Function Name: ra6m5_dtc_set_transfer_data
 * Description  :
 *       Set transfer data
 *
 * Input Parameters :
 *    p_transfer_cfg - Reguested configuration
 *    p_transfer_data  - Transfer information RAM memory
 *
 * Return :
 *     dtc_err_t -  error codes
 *
 ****************************************************************************/

static dtc_err_t ra6m5_dtc_set_static_transfer_data(
                    dtc_static_transfer_data_cfg_t * p_transfer_cfg,
                    dtc_transfer_data_t * p_transfer_data)
{
  dtc_mra_t  t_mra;
  dtc_mrb_t  t_mrb;
  dtc_cra_t  t_cra;
  dtc_crb_t  t_crb;

  /* Initialize crb. When normal transfer mode or repeat transfer mode is
   * selected, this register is not used and the set value is ignored.
   */

  t_crb.WORD = 0x0000;

  /* Map transfer data to st_dtc_internal_registers_t */

  volatile st_dtc_internal_registers_t *td_ptr =
       (volatile st_dtc_internal_registers_t *)p_transfer_data;

  /* Prepare CRA and CRB internal register */

  t_mra.BYTE = (uint8_t)(p_transfer_cfg->src_addr_mode
                      | p_transfer_cfg->data_size
                      | p_transfer_cfg->transfer_mode);

  t_mrb.BYTE = (uint8_t)(p_transfer_cfg->dest_addr_mode
                      | p_transfer_cfg->repeat_block_side
                      | p_transfer_cfg->response_interrupt
                      | p_transfer_cfg->chain_transfer_enable
                      | p_transfer_cfg->chain_transfer_mode);

  switch (t_mra.BIT.MD) /* DTC transfer mode */
    {
      case 0x0: /* Normal mode */
        {
          if (DTC_MAX_16BITS_COUNT_VAL == p_transfer_cfg->transfer_count)/* Transfer count = 65536 */
            {
              t_cra.WORD = 0x0000;
            }
            else /* 1 - 65535 */
            {
              t_cra.WORD = (uint16_t)p_transfer_cfg->transfer_count;
            }
          break;
        }

      case 0x1: /* Repeat mode */
        {
          /* Set counter. */

          if (p_transfer_cfg->transfer_count < DTC_MAX_8BITS_COUNT_VAL) /* count 1-255 */
            {
              t_cra.BYTE.CRA_H = (uint8_t)p_transfer_cfg->transfer_count;
              t_cra.BYTE.CRA_L = (uint8_t)p_transfer_cfg->transfer_count;
            }
          else if (DTC_MAX_8BITS_COUNT_VAL == p_transfer_cfg->transfer_count)
            {
              t_cra.BYTE.CRA_H = 0x00;
              t_cra.BYTE.CRA_L = 0x00;
            }
          else /* Transfer count > 256 */
            {
              return DTC_ERR_INVALID_ARG;
            }
          break;
        }

      case 0x2: /* DTC_TRANSFER_MODE_BLOCK - Block transfer mode */
        {
          /* Set counter. */

          if (DTC_MAX_16BITS_COUNT_VAL == p_transfer_cfg->transfer_count)/* Transfer count = 65536 */
            {
              t_crb.WORD = 0x0000;
            }
          else /* 1 - 65535 */
            {
              t_crb.WORD = (uint16_t)p_transfer_cfg->transfer_count;
            }

          if (p_transfer_cfg->block_size < DTC_MAX_8BITS_COUNT_VAL) /* Block size 1-255 */
            {
              t_cra.BYTE.CRA_H = (uint8_t)p_transfer_cfg->block_size;
              t_cra.BYTE.CRA_L = (uint8_t)p_transfer_cfg->block_size;
            }
          else if (DTC_MAX_8BITS_COUNT_VAL == p_transfer_cfg->block_size) /* Block size = 256 */
            {
              t_cra.BYTE.CRA_H = 0;
              t_cra.BYTE.CRA_L = 0;
            }
          else /* Invalid block size */
            {
              return DTC_ERR_INVALID_ARG;
            }
          break;
        }

      default:
        {
          return DTC_ERR_INVALID_ARG;
          break;
        }
    }

  /* Set transfer information in RAM area */

  /* settings for fist long word: MRA & MRB */

  td_ptr->FIRST_LWORD.REG.MRA.BYTE = t_mra.BYTE; /* 1 byte MRA */
  td_ptr->FIRST_LWORD.REG.MRB.BYTE = t_mrb.BYTE; /* 1 byte MRB */

  /* settings for second long word: SAR */

  td_ptr->SECOND_LWORD.SAR = p_transfer_cfg->source_addr; /* 4 byte SAR */

  /* settings for third long word: DAR */

  td_ptr->THIRD_LWORD.DAR = p_transfer_cfg->dest_addr; /* 4 byte DAR */

  /* settings for fourth long word: CRA & CRB */

  td_ptr->FOURTH_LWORD.REG.CRA.WORD = t_cra.WORD;

  td_ptr->FOURTH_LWORD.REG.CRB.WORD = t_crb.WORD;

  return DTC_SUCCESS;
}

/****************************************************************************
 * Name: ra6m5_dtc_readskip_enable
 *
 * Description:
 *   DTC Transfer Information Read Skip enable
 *
 * Input Parameters:
 *    handle - DTC_HANDLE
 *
 * Assumptions:
 *   - DTC handle allocated by ra6m5_dtc_gethandle()
 *
 ****************************************************************************/

static void ra6m5_dtc_readskip_enable(DTC_HANDLE handle)
{
  struct ra6m5_dtc_s *dtchandle = (struct ra6m5_dtc_s *)handle;
  uint8_t regval;

  /* Configure Read skip bit in DTCCR register */

  regval = dtcchan_getreg(dtchandle, RA6M5_DTC_DTCCR_OFFSET);
  regval |= (DTC_DTCCR_RRS);
  dtcchan_putreg(dtchandle, RA6M5_DTC_DTCCR_OFFSET, regval);
}

/****************************************************************************
 * Name: ra6m5_dtc_readskip_disable
 *
 * Description:
 *   DTC Transfer Information Read Skip disable
 *
 * Input Parameters:
 *    handle - DTC_HANDLE
 *
 * Assumptions:
 *   - DTC handle allocated by ra6m5_dtc_gethandle()
 *
 ****************************************************************************/
#ifndef CONFIG_RX65N_DTC_TRANSFER_DATA_READ_SKIP
static void ra6m5_dtc_readskip_disable(DTC_HANDLE handle)
{
  struct ra6m5_dtc_s *dtchandle = (struct ra6m5_dtc_s *)handle;
  uint8_t regval;

  /* Configure Read skip bit in DTCCR register */

  regval = dtcchan_getreg(dtchandle, RA6M5_DTC_DTCCR_OFFSET);
  regval &= ~(DTC_DTCCR_RRS);
  dtcchan_putreg(dtchandle, RA6M5_DTC_DTCCR_OFFSET, regval);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra6m5_dtc_start
 *
 * Description:
 *   DTC Module Start
 *
 * Input Parameters:
 *    handle - DTC_HANDLE
 *
 * Assumptions:
 *   - DTC handle allocated by ra6m5_dtc_gethandle()
 *
 ****************************************************************************/

void ra6m5_dtc_start(DTC_HANDLE handle)
{
  struct ra6m5_dtc_s *dtchandle = (struct ra6m5_dtc_s *)handle;
  uint8_t regval;

  /* Configure Read skip bit in DTCST register */

  regval = dtcchan_getreg(dtchandle, RA6M5_DTC_DTCST_OFFSET);
  regval |= (DTC_DTCST_START);
  dtcchan_putreg(dtchandle, RA6M5_DTC_DTCST_OFFSET, regval);
}

/****************************************************************************
 * Name: ra6m5_dtc_stop
 *
 * Description:
 *   DTC Module Stop
 *
 * Input Parameters:
 *    handle - DTC_HANDLE
 *
 * Assumptions:
 *   - DTC handle allocated by ra6m5_dtc_gethandle()
 *
 ****************************************************************************/

void ra6m5_dtc_stop(DTC_HANDLE handle)
{
  struct ra6m5_dtc_s *dtchandle = (struct ra6m5_dtc_s *)handle;
  uint8_t regval;

  /* Configure Read skip bit in DTCST register */

  regval = dtcchan_getreg(dtchandle, RA6M5_DTC_DTCST_OFFSET);
  regval &= ~(DTC_DTCST_START);
  dtcchan_putreg(dtchandle, RA6M5_DTC_DTCST_OFFSET, regval);
}

/****************************************************************************
 * Name: ra6m5_dtc_status
 *
 * Description:
 *   Output the status of DTC
 *
 * Input Parameters:
 *    handle - DTC_HANDLE
 *
 * Output Parameters:
 *    p_stat - DTC status structure
 *
 * Assumptions:
 *   - DTC handle allocated by ra6m5_get_dtchandle()
 *
 ****************************************************************************/

void ra6m5_dtc_status(DTC_HANDLE handle, dtc_stat_t *p_stat)
{
  struct ra6m5_dtc_s *dtchandle = (struct ra6m5_dtc_s *)handle;
  uint16_t stat;

  DEBUGASSERT(dtchandle != NULL || p_stat != NULL);

  if (dtchandle->initialized)
    {
      /* Check DTC Status */

      stat = dtcchan_getreg16(dtchandle, RA6M5_DTC_DTCSTS_OFFSET);

      if (0 == (stat & DTC_DTCSTS_ACT)) /* DTC transfer operation is not in progress. */
        {
          p_stat->in_progress = false;

          /* DTC is not in progress. -> vector number is invalid. */
        }
      else /* DTC transfer operation is in progress. */
        {
          p_stat->in_progress = true;

      /* Get the current vector number. */

          p_stat->vect_nr = (uint8_t)(stat & DTC_DTCSTS_VECN_MASK);
        }
    }
}

/****************************************************************************
 * Name: ra6m5_dtc_srcactivation
 *
 * Description:
 *   Src activation enabling
 *
 * Input Parameters:
 *    handle - DTC_HANDLE
 *    src -Activation source
 *
 * Assumptions:
 *   -DTC vector table allocated by ra6m5_dtc_setup_static_transferdata
 *
 ****************************************************************************/

void ra6m5_dtc_srcactivation(DTC_HANDLE handle, uint8_t src)
{
  struct ra6m5_dtc_s *dtchandle = (struct ra6m5_dtc_s *)handle;

  DEBUGASSERT(handle != NULL);

  dtc_activation_source_t act_source = (dtc_activation_source_t)src;

  DEBUGASSERT(act_source >= DTC_MIN_VECT_NUMBER &&
              act_source <= DTC_MAX_VECT_NUMBER);

  if (dtchandle->initialized)
    {
      /* Enable the interrupt source */

      modifyreg32(RA6M5_ICU_IELSR_REG(act_source), 0, ICU_IELSR_DTCE);
    }
}

/****************************************************************************
 * Name: ra6m5_dtc_srcdeactivation
 *
 * Description:
 *   Src activation disabling
 *
 * Input Parameters:
 *    handle - DTC_HANDLE
 *    src -Activation source
 *
 * Assumptions:
 *   -DTC vector table allocated by ra6m5_dtc_setup_static_transferdata
 *
 ****************************************************************************/

void ra6m5_dtc_srcdeactivation(DTC_HANDLE handle, uint8_t src)
{
  struct ra6m5_dtc_s *dtchandle = (struct ra6m5_dtc_s *)handle;

  DEBUGASSERT(handle != NULL);

  dtc_activation_source_t act_source = (dtc_activation_source_t)src;

  DEBUGASSERT(act_source >= DTC_MIN_VECT_NUMBER &&
              act_source <= DTC_MAX_VECT_NUMBER);

  if (dtchandle->initialized)
    {
      /* Disable the interrupt source */

      modifyreg32(RA6M5_ICU_IELSR_REG(act_source), ICU_IELSR_DTCE, 0);
    }
}

/****************************************************************************
 * Name: ra6m5_dtc_setup_dynamic_transferdata
 *
 * Description:
 *   Setup dynamic transfer info like src and destination address and number
 *   of data to transfer
 *
 * Input Parameters:
 *    handle - DTC_HANDLE
 *    src -Activation source
 *    dcfg -Dynamic config parameter
 *    nchain - Number of chain transfer
 *
 * Returned Value:
 *  dtc_err_t -DTC API error codes
 ****************************************************************************/

dtc_err_t ra6m5_dtc_setup_dynamic_transferdata(DTC_HANDLE handle,
                                               uint8_t src,
                                               uint32_t dcfg, 
                                               uint32_t nchain)
{
  struct ra6m5_dtc_s *dtchandle = (struct ra6m5_dtc_s *)handle;
  dtc_err_t ret = DTC_SUCCESS;
  uint32_t *ptr         = NULL;
  dtc_transfer_data_t *p_transfer_data = NULL;
  uint8_t   rrs_backup  = 0;
  uint32_t  dtce_backup = 0;
  irqstate_t flags;
  int i;

  /* Chain transfer count */

  uint32_t count = nchain + 1;

  /* Dynamic user configuration */

  dtc_dynamic_transfer_data_cfg_t *p_dtransfer_cfg =
                           (dtc_dynamic_transfer_data_cfg_t *)dcfg;

  /* Activation source */

  dtc_activation_source_t act_source = (dtc_activation_source_t)src;

  flags = enter_critical_section();

  /* Store old value of DTCERn.DTCE bit. */

  dtce_backup = getreg32(RA6M5_ICU_IELSR_REG(act_source)) & ICU_IELSR_DTCE;

  /* Disable the interrupt source. Clear the DTCER */

  modifyreg32(RA6M5_ICU_IELSR_REG(act_source), ICU_IELSR_DTCE, 0);

  /* Store RRS and clear RRS */

  rrs_backup = dtcchan_getreg(dtchandle, RA6M5_DTC_DTCCR_OFFSET);
  dtcchan_putreg(dtchandle, RA6M5_DTC_DTCCR_OFFSET, 0x0);

  /* Get Transfer data pointer */

  ptr = (uint32_t *)(dtchandle->vectortable + (4 * act_source));
  p_transfer_data  = (dtc_transfer_data_t *) (*ptr);

  ret = ra6m5_dtc_validate_dynamic_params(p_dtransfer_cfg, p_transfer_data);
  if (DTC_SUCCESS != ret)
    {
      leave_critical_section(flags);
      return ret;
    }

  /* Update transfer data with address and counter */

  for (i = 0; i < count; i++)
    {
      if (ra6m5_dtc_set_dynamic_transfer_data(p_dtransfer_cfg,
                                  p_transfer_data) != DTC_SUCCESS)
        {
          leave_critical_section(flags);
          return DTC_ERR_INVALID_ARG;
        }

      p_dtransfer_cfg++;
      p_transfer_data++;
      count--;
    }

  /* Restore RRS bit */

  dtcchan_putreg(dtchandle, RA6M5_DTC_DTCCR_OFFSET, rrs_backup);

  /* Restore the DTCE bit. */

  modifyreg32(RA6M5_ICU_IELSR_REG(act_source), 0, dtce_backup);

  leave_critical_section(flags);

  return ret;
}

/****************************************************************************
 * Name: ra6m5_dtc_setup_static_transferdata
 *
 * Description:
 *   Configure and Creates the transfer data for a activation source.
 *
 * Input Parameters:
 *    handle - DTC_HANDLE
 *    src -Activation source
 *    scfg -Pointer to contains the settings for Transfer data
 *    pdt - pointer to data transfer
 *    nchain - Number of chain transfer
 *
 * Returned Value:
 *  dtc_err_t -DTC API error codes
 *
 ****************************************************************************/

dtc_err_t ra6m5_dtc_setup_static_transferdata(DTC_HANDLE handle, uint8_t src,
                               uint32_t scfg, uint32_t pdt, uint32_t nchain)
{
  struct ra6m5_dtc_s *dtchandle = (struct ra6m5_dtc_s *)handle;

  uint32_t *ptr = NULL;

  int i;

  /* Transfer Information count */

  uint32_t count = nchain + 1;

  /* Activation source */

  dtc_activation_source_t act_source = (dtc_activation_source_t)src;

  /* Static User Configuration */

  dtc_static_transfer_data_cfg_t *p_stransfer_cfg =
                          (dtc_static_transfer_data_cfg_t *)scfg;

  /* Allocate  memory in RAM for transfer information */

  dtc_transfer_data_t *p_transfer_data = (dtc_transfer_data_t *)pdt;

  if ((act_source < DTC_MIN_VECT_NUMBER) ||
      (act_source > DTC_MAX_VECT_NUMBER))
    {
      return DTC_ERR_INVALID_ARG;
    }

  if (p_stransfer_cfg == NULL)
    {
      return DTC_ERR_INVALID_ARG;
    }

  if (p_transfer_data == NULL)
    {
      return DTC_ERR_INVALID_ARG;
    }

  /* Set Transfer information pointer in vector table */

  ptr = (uint32_t *)(dtchandle->vectortable + (4 * act_source));
  *ptr = (uint32_t)p_transfer_data;

  /* Set transfer Information as per request */

  for (i = 0; i < count ; i++)
    {
      if (ra6m5_dtc_set_static_transfer_data(p_stransfer_cfg,
                      p_transfer_data) != DTC_SUCCESS)
        {
          return DTC_ERR_INVALID_ARG;
        }

      p_stransfer_cfg++;
      p_transfer_data++;
      count--;
    }

  return DTC_SUCCESS;
}

/****************************************************************************
 * Name: ra6m5_dtc_gethandle
 *
 * Description:
 *   Get DTC handle.
 *
 * Input Parameters:
 *   chan - Identifies the channel resource. For the RX65N, this
 *     is simply the channel number is 0
 *
 * Returned Value:
 *   dtchandle-Provided that 'chan' is valid, this function ALWAYS returns a
 *   non-NULL,void* DTC channel handle.  (If 'chndx' is invalid, the
 *   function will assert if debug is enabled or do something ignorant
 *    otherwise).
 *
 ****************************************************************************/

DTC_HANDLE ra6m5_dtc_gethandle(unsigned int chan)
{
  struct ra6m5_dtc_s *dtchandle = NULL;

  dtchandle = &g_dtchandle[chan];

  DEBUGASSERT(chan < DTC_NCHANNELS);

  return (DTC_HANDLE)dtchandle;
}


/****************************************************************************
 * Name: ra6m5_dtc_initialize
 *
 * Description:
 *   Initialize the DTC
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void ra6m5_dtc_initialize(void)
{
  struct ra6m5_dtc_s *dtchandle = NULL;

  int chndx;
  uint8_t *dtctable;
  volatile uint32_t dtce_cnt = 0;

  /* Initialize each DTC channel */

  for (chndx = 0; chndx < DTC_NCHANNELS; chndx++) /* RA6M5 support only one channel */
    {
      dtchandle = &g_dtchandle[chndx];

      /* Get DTC Vector table */

      dtctable = dtchandle->vectortable;

      /* Clear all DTCER registers of all activation sources */

      for (dtce_cnt = 0 ; dtce_cnt < DTC_NUM_INTERRUPT_SRC; dtce_cnt++)
        {
          modifyreg32(RA6M5_ICU_IELSR_REG(dtce_cnt), ICU_IELSR_DTCE, 0);
        }

      /* Cancel module stop for DMAC and DTC */

      ra6m5_dtc_module_enable();

      /* Configure DTC Vector Table Base Register */

      dtchandle->vectortable = (uint8_t *)((uint32_t)(dtctable + DTC_VECTOR_ADDRESS_ALIGN) & DTC_VECTOR_ADDRESS_MASK);
      dtcchan_putreg32(dtchandle, RA6M5_DTC_DTCVBR_OFFSET, (uint32_t)dtchandle->vectortable);

      /* Stop DTC */

      ra6m5_dtc_stop(dtchandle);

      /* Configure read skip enable/disbale */

#if defined(CONFIG_RA6M5_DTC_TRANSFER_DATA_READ_SKIP)
      ra6m5_dtc_readskip_enable(dtchandle);
#else
      ra6m5_dtc_readskip_disable(dtchandle);
#endif

      /* Update initializing status */

      dtchandle->initialized =  TRUE;

      /* Start DTC */

      ra6m5_dtc_start(dtchandle);
    }
}

#endif /* End of CONFIG_RA6M5_DTC*/
