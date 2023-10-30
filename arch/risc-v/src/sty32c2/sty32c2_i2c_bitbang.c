/****************************************************************************
 * arch/arm/src/sty32c2/sty32c2_i2c_bitbang.c
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

#include <assert.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/i2c/i2c_bitbang.h>
#include <nuttx/kmalloc.h>
#include <arch/board/board.h>

#include "riscv_internal.h"
#include "hardware/sty32c2_i2c.h"
#include "sty32c2.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sty32c2_i2c_bitbang_dev_s
{
  struct i2c_bitbang_lower_dev_s lower;
  uint8_t bus;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void i2c_bb_initialize(struct i2c_bitbang_lower_dev_s *lower);

static void i2c_bb_set_scl(struct i2c_bitbang_lower_dev_s *lower,
                           bool high);
static void i2c_bb_set_sda(struct i2c_bitbang_lower_dev_s *lower,
                           bool high);

static bool i2c_bb_get_scl(struct i2c_bitbang_lower_dev_s *lower);
static bool i2c_bb_get_sda(struct i2c_bitbang_lower_dev_s *lower);

/****************************************************************************
 * Private Data
 ****************************************************************************/

const static struct i2c_bitbang_lower_ops_s g_bb_ops =
{
  .initialize = i2c_bb_initialize,
  .set_scl    = i2c_bb_set_scl,
  .set_sda    = i2c_bb_set_sda,
  .get_scl    = i2c_bb_get_scl,
  .get_sda    = i2c_bb_get_sda
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void i2c_bb_initialize(struct i2c_bitbang_lower_dev_s *lower)
{
  struct sty32c2_i2c_bitbang_dev_s *dev = lower->priv;

  putreg32(2, STY32C2_I2C_BB_WR_REG(dev->bus));

}

static void i2c_bb_set_scl(struct i2c_bitbang_lower_dev_s *lower,
                           bool high)
{
  struct sty32c2_i2c_bitbang_dev_s *dev = lower->priv;

  /* If set high, pin is pulled up by output disable */

  if (high) {
    modifyreg32(STY32C2_I2C_BB_WR_REG(dev->bus), 0, 1);
  } else {
    modifyreg32(STY32C2_I2C_BB_WR_REG(dev->bus), 1, 0);
  }
}

static void i2c_bb_set_sda(struct i2c_bitbang_lower_dev_s *lower,
                           bool high)
{
  struct sty32c2_i2c_bitbang_dev_s *dev = lower->priv;

  /* If set high, pin is pulled up by output disable */

  if (high) {
    modifyreg32(STY32C2_I2C_BB_WR_REG(dev->bus), 0, 4);
  } else {
    modifyreg32(STY32C2_I2C_BB_WR_REG(dev->bus), 4, 0);
  }
}

static bool i2c_bb_get_scl(struct i2c_bitbang_lower_dev_s *lower)
{
  struct sty32c2_i2c_bitbang_dev_s *dev = lower->priv;

  return (getreg32(STY32C2_I2C_BB_WR_REG(dev->bus) & 1) != 0);
}

static bool i2c_bb_get_sda(struct i2c_bitbang_lower_dev_s *lower)
{
  struct sty32c2_i2c_bitbang_dev_s *dev = lower->priv;

  return (getreg32(STY32C2_I2C_BB_RD_REG(dev->bus) & 1) != 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct i2c_master_s *sty32c2_i2c_bitbang_initialize(int bus)
{
  struct sty32c2_i2c_bitbang_dev_s *dev =
      (struct sty32c2_i2c_bitbang_dev_s *)
          kmm_zalloc(sizeof(struct sty32c2_i2c_bitbang_dev_s));

  DEBUGASSERT(dev);

  dev->lower.ops  = &g_bb_ops;
  dev->lower.priv = dev;
  dev->bus        = bus;

  return i2c_bitbang_initialize(&dev->lower);
}
