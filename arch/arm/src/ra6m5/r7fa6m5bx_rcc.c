/****************************************************************************
 * arch/arm/src/ra6m5/r7fa6m5bx_rcc.c
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
#include <arch/ra6m5/chip.h>
#include <arch/board/board.h>

#include "ra6m5_rcc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Key code for writing PRCR register. */
#define BSP_PRV_PRCR_UNLOCK     ((BSP_PRV_PRCR_KEY) | 0x03U)
#define BSP_PRV_PRCR_LOCK       ((BSP_PRV_PRCR_KEY) | 0x00U)

/* Wait state definitions for MEMWAIT. */
#define BSP_PRV_MEMWAIT_0_WAIT_CYCLES           (0U)
#define BSP_PRV_MEMWAIT_2_WAIT_CYCLES           (1U)
#define BSP_PRV_MEMWAIT_MAX_0_WAIT_FREQ         (32000000U)

/* Wait state definitions for MCUS with SRAMWTSC and FLWT. */
#define BSP_PRV_SRAMWTSC_WAIT_CYCLES_DISABLE    (0U)
#define BSP_PRV_ROM_0_WAIT_CYCLES               (0U)
#define BSP_PRV_ROM_1_WAIT_CYCLES               (1U)
#define BSP_PRV_ROM_2_WAIT_CYCLES               (2U)
#define BSP_PRV_ROM_3_WAIT_CYCLES               (3U)

/* Calculate the value to write to SCKDIVCR. */
#define BSP_PRV_STARTUP_SCKDIVCR_FCLK_BITS      ((RA6M5_CFG_FCLK_DIV  & 7U) << 28U)
#define BSP_PRV_STARTUP_SCKDIVCR_ICLK_BITS      ((RA6M5_CFG_ICLK_DIV  & 7U) << 24U)
#define BSP_PRV_STARTUP_SCKDIVCR_BCLK_BITS      ((RA6M5_CFG_BCLK_DIV  & 7U) << 16U)
#define BSP_PRV_STARTUP_SCKDIVCR_PCLKA_BITS     ((RA6M5_CFG_PCLKA_DIV & 7U) << 12U)
#define BSP_PRV_STARTUP_SCKDIVCR_PCLKB_BITS     ((RA6M5_CFG_PCLKB_DIV & 7U) <<  8U)
#define BSP_PRV_STARTUP_SCKDIVCR_PCLKC_BITS     ((RA6M5_CFG_PCLKC_DIV & 7U) <<  4U)
#define BSP_PRV_STARTUP_SCKDIVCR_PCLKD_BITS     ((RA6M5_CFG_PCLKD_DIV & 7U)       )

#if (RA6M5_CFG_PLL_SOURCE == RA6M5_CLOCKS_SOURCE_MOSC)
#define BSP_PRV_PLLCCR                          (((RA6M5_CFG_PLL_MUL  & 0x3f) << 8) | (0 << 4) | RA6M5_CFG_PLL_DIV )
#else
#define BSP_PRV_PLLCCR                          (((RA6M5_CFG_PLL_MUL  & 0x3f) << 8) | (1 << 4) | RA6M5_CFG_PLL_DIV )
#endif

#if (RA6M5_CFG_PLL_SOURCE == RA6M5_CLOCKS_SOURCE_MOSC)
#define BSP_PRV_PLL2CCR                         (((RA6M5_CFG_PLL2_MUL & 0x3f) << 8) | (0 << 4) | RA6M5_CFG_PLL2_DIV)
#else
#define BSP_PRV_PLL2CCR                         (((RA6M5_CFG_PLL2_MUL & 0x3f) << 8) | (1 << 4) | RA6M5_CFG_PLL2_DIV)
#endif

#define BSP_PRV_SCKDIVCR                        (BSP_PRV_STARTUP_SCKDIVCR_ICLK_BITS  | \
                                                 BSP_PRV_STARTUP_SCKDIVCR_PCLKD_BITS | \
                                                 BSP_PRV_STARTUP_SCKDIVCR_PCLKC_BITS | \
                                                 BSP_PRV_STARTUP_SCKDIVCR_PCLKB_BITS | \
                                                 BSP_PRV_STARTUP_SCKDIVCR_PCLKA_BITS | \
                                                 BSP_PRV_STARTUP_SCKDIVCR_BCLK_BITS |  \
                                                 BSP_PRV_STARTUP_SCKDIVCR_FCLK_BITS)

#define BSP_PRV_PERIPHERAL_CLK_REQ_BIT_MASK     (1U << 6U)
#define BSP_PRV_PERIPHERAL_CLK_RDY_BIT_MASK     (1U << 7U)


/****************************************************************************
 * Public Data
 ****************************************************************************/
uint32_t g_clock_freq[RA6M5_CLOCKS_SOURCE_NUMBER] locate_data(".noinit");

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rcc_clock_freq_var_init
 *
 * Description:
 *   Initialize variable to store system clock frequencies..
 *
 ****************************************************************************/

static inline void rcc_clock_freq_var_init(void)
{
    g_clock_freq[RA6M5_CLOCKS_SOURCE_HOCO] = RA6M5_HOCO_FREQUENCY;
    g_clock_freq[RA6M5_CLOCKS_SOURCE_MOCO] = RA6M5_MOCO_FREQUENCY;
    g_clock_freq[RA6M5_CLOCKS_SOURCE_LOCO] = RA6M5_LOCO_FREQUENCY;
    g_clock_freq[RA6M5_CLOCKS_SOURCE_MOSC] = RA6M5_MOSC_FREQUENCY;
    g_clock_freq[RA6M5_CLOCKS_SOURCE_SOSC] = RA6M5_SOSC_FREQUENCY;

    /* The PLL Is the startup clock. */
    g_clock_freq[RA6M5_CLOCKS_SOURCE_PLL] = ((g_clock_freq[RA6M5_CFG_PLL_SOURCE] * (RA6M5_CFG_PLL_MUL + 1U)) >> 1U) / (RA6M5_CFG_PLL_DIV + 1U);;

    /* Update PLL2 Clock Frequency based on BSP Configuration. */
    g_clock_freq[RA6M5_CLOCKS_SOURCE_PLL2] = ((g_clock_freq[RA6M5_CFG_PLL2_SOURCE] * (RA6M5_CFG_PLL2_MUL + 1U)) >> 1U) / (RA6M5_CFG_PLL2_DIV + 1U);

    /* Update Peripheral clocks */
    g_clock_freq[RA6M5_CLOCKS_SOURCE_PCLKA] = g_clock_freq[RA6M5_CLOCKS_SOURCE_PLL] >> RA6M5_CFG_PCLKA_DIV;
    g_clock_freq[RA6M5_CLOCKS_SOURCE_PCLKB] = g_clock_freq[RA6M5_CLOCKS_SOURCE_PLL] >> RA6M5_CFG_PCLKB_DIV;
    g_clock_freq[RA6M5_CLOCKS_SOURCE_PCLKC] = g_clock_freq[RA6M5_CLOCKS_SOURCE_PLL] >> RA6M5_CFG_PCLKC_DIV;
    g_clock_freq[RA6M5_CLOCKS_SOURCE_PCLKD] = g_clock_freq[RA6M5_CLOCKS_SOURCE_PLL] >> RA6M5_CFG_PCLKD_DIV;
}


/****************************************************************************
 * Name: ra6m5_flash_cache_enable
 *
 * Description:
 *   Enable Flash cache.
 *
 ****************************************************************************/

static inline void ra6m5_flash_cache_enable(void)
{
    uint16_t regval;

    /* Invalidate the flash cache and wait until it is invalidated. (See section 55.3.2.2 "Operation" of the Flash Cache
     * in the RA6M3 manual R01UH0878EJ0100). */

    putreg16(1, RA6M5_FCACHE_REG(RA6M5_FCACHE_FCACHEIV_OFFSET));
    do {
        regval = getreg16(RA6M5_FCACHE_REG(RA6M5_FCACHE_FCACHEIV_OFFSET));
    } while (regval != 0);

    /* Enable flash cache. */
    putreg16(1, RA6M5_FCACHE_REG(RA6M5_FCACHE_FCACHEE_OFFSET));

    /* Configure the C-Cache line size. */
    putreg32(1, RA6M5_CACHE_REG(RA6M5_CACHE_CCALCF_OFFSET));

    /* Enable the C-Cache. */
    putreg32(1, RA6M5_CACHE_REG(RA6M5_CACHE_CCACTL_OFFSET));
}


/****************************************************************************
 * Name: ra6m5_flash_cache_disable
 *
 * Description:
 *   Disable Flash cache.
 *
 ****************************************************************************/

static inline void ra6m5_flash_cache_disable(void)
{
    /* Disable flash cache. */
    putreg16(0, RA6M5_FCACHE_REG(RA6M5_FCACHE_FCACHEE_OFFSET));

    /* Disable the C-Cache. */
    putreg32(0, RA6M5_CACHE_REG(RA6M5_CACHE_CCACTL_OFFSET));
}


/****************************************************************************
 * Name: ra6m5_set_peripheral_clock
 *
 * Description:
 *   Set the specified peripheral clocks.
 *
 ****************************************************************************/

static inline void ra6m5_set_peripheral_clock(uint32_t clk_ctrl_reg,
                                              uint32_t clk_div_reg,
                                              uint8_t  peripheral_clk_div,
                                              uint8_t  peripheral_clk_source)
{
    uint8_t regval;
    
    /* Request to stop the peripheral clock. */
    regval  = getreg8(clk_ctrl_reg);
    regval |= BSP_PRV_PERIPHERAL_CLK_REQ_BIT_MASK;
    putreg8(regval, clk_ctrl_reg);

    /* Wait for the peripheral clock to stop. */
    do {
        regval = getreg8(clk_ctrl_reg);
    } while (!(regval & BSP_PRV_PERIPHERAL_CLK_RDY_BIT_MASK));

    /* Select the peripheral clock divisor and source. */
    putreg8(peripheral_clk_div, clk_div_reg);
    regval = peripheral_clk_source | BSP_PRV_PERIPHERAL_CLK_REQ_BIT_MASK;
    putreg8(regval, clk_div_reg);

    /* Request to start the peripheral clock. */
    regval  = getreg8(clk_ctrl_reg);
    regval &= (uint8_t) ~BSP_PRV_PERIPHERAL_CLK_REQ_BIT_MASK;
    putreg8(regval, clk_ctrl_reg);

    /* Wait for the peripheral clock to start. */
    do {
        regval = getreg8(clk_ctrl_reg);
    } while (regval & BSP_PRV_PERIPHERAL_CLK_RDY_BIT_MASK);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra6m5_enable_peripherals_clock
 ****************************************************************************/

void ra6m5_enable_peripherals_clock(void)
{
    /* Unlock CGC and LPM protection registers. */
    putreg16(BSP_PRV_PRCR_UNLOCK, RA6M5_SYSTEM_REG(RA6M5_SYS_PRCR_OFFSET));

    /* USB clock configuration */
    ra6m5_set_peripheral_clock(RA6M5_SYSTEM_REG(RA6M5_SYS_USBCKCR_OFFSET),
                               RA6M5_SYSTEM_REG(RA6M5_SYS_USBCKDIVCR_OFFSET),
                               RA6M5_CLOCKS_SYS_CLOCK_DIV_4, 
                               RA6M5_CLOCKS_SOURCE_PLL2);

    /* SCI/SPI clock configuration */
    ra6m5_set_peripheral_clock(RA6M5_SYSTEM_REG(RA6M5_SYS_OCTACKCR_OFFSET),
                               RA6M5_SYSTEM_REG(RA6M5_SYS_OCTACKDIVCR_OFFSET),
                               RA6M5_CLOCKS_SYS_CLOCK_DIV_4, 
                               RA6M5_CLOCKS_SOURCE_PLL);

    /* CANFD clock configuration */
    ra6m5_set_peripheral_clock(RA6M5_SYSTEM_REG(RA6M5_SYS_CANFDCKCR_OFFSET),
                               RA6M5_SYSTEM_REG(RA6M5_SYS_CANFDCKDIVCR_OFFSET),
                               RA6M5_CLOCKS_SYS_CLOCK_DIV_6, 
                               RA6M5_CLOCKS_SOURCE_PLL2);

    /* Lock CGC and LPM protection registers. */
    putreg16(BSP_PRV_PRCR_LOCK, RA6M5_SYSTEM_REG(RA6M5_SYS_PRCR_OFFSET));
}

/****************************************************************************
 * Name: ra6m5_stdclockconfig
 *
 * Description:
 *   Called to change to new clock based on settings in board.h
 *
 *   NOTE:  This logic would need to be extended if you need to select low-
 *   power clocking modes!
 ****************************************************************************/

#ifndef CONFIG_ARCH_BOARD_STM32U5_CUSTOM_CLOCKCONFIG
void ra6m5_stdclockconfig(void)
{
    uint32_t regval;

    /* Unlock CGC and LPM protection registers. */
    putreg16(BSP_PRV_PRCR_UNLOCK, RA6M5_SYSTEM_REG(RA6M5_SYS_PRCR_OFFSET));

    /* Enable the flash cache and don't disable it while running from flash. On these MCUs, the flash cache does not
     * need to be disabled when adjusting the operating power mode. */
    ra6m5_flash_cache_enable();

    /* Enable the flash cache and don't disable it while running from flash. On these MCUs, the flash cache does not
     * need to be disabled when adjusting the operating power mode. */
    rcc_clock_freq_var_init();

    /* Update the main oscillator drive, source, and wait states if the main oscillator is stopped.  If the main
     * oscillator is running, the drive, source, and wait states are assumed to be already set appropriately. */

    if (getreg8(RA6M5_SYSTEM_REG(RA6M5_SYS_MOSCCR_OFFSET)))
    {
        /* Don't write to MOSCWTCR unless MOSTP is 1 and MOSCSF = 0. */
        do {
            regval = getreg8(RA6M5_SYSTEM_REG(RA6M5_SYS_OSCSF_OFFSET));
        } while (regval & SYS_OSCSF_MOSCSF);

        /* Configure main oscillator drive. */
        putreg8(0x20, RA6M5_SYSTEM_REG(RA6M5_SYS_MOMCR_OFFSET));

        /* Set the main oscillator wait time. */
        putreg8(0x09, RA6M5_SYSTEM_REG(RA6M5_SYS_MOSCWTCR_OFFSET));
    }

    /* If Sub-Clock Oscillator is started at reset, stop it before configuring the subclock drive. */
    if (getreg8(RA6M5_SYSTEM_REG(RA6M5_SYS_SOSCCR_OFFSET)) == 0)
    {
        /* Stop the Sub-Clock Oscillator to update the SOMCR register. */
        putreg8(1, RA6M5_SYSTEM_REG(RA6M5_SYS_SOSCCR_OFFSET));

        /* Allow a stop interval of at least 5 SOSC clock cycles before configuring the drive capacity
         * and restarting Sub-Clock Oscillator. */
        up_udelay(2000);
    }

    /* Configure the subclock drive as subclock is not running. */
    putreg8(0, RA6M5_SYSTEM_REG(RA6M5_SYS_SOMCR_OFFSET));

    /* Restart the Sub-Clock Oscillator. */
    putreg8(0, RA6M5_SYSTEM_REG(RA6M5_SYS_SOSCCR_OFFSET));


    /* Start HOCO */
    putreg8(0, RA6M5_SYSTEM_REG(RA6M5_SYS_HOCOCR_OFFSET));

    /* Wait for HOCO to stabilize. */
    do {
        regval = getreg8(RA6M5_SYSTEM_REG(RA6M5_SYS_OSCSF_OFFSET));
    } while (!(regval & SYS_OSCSF_HOCOSF));

    /* If the MOCO is not running, start it and wait for it to stabilize using a software delay. */
    if (getreg8(RA6M5_SYSTEM_REG(RA6M5_SYS_MOCOCR_OFFSET)) == 0)
    {
        /* Start MOCO */
        putreg8(0, RA6M5_SYSTEM_REG(RA6M5_SYS_MOCOCR_OFFSET));
        up_udelay(2000);
    }

    /* If the LOCO is not running, start it and wait for it to stabilize using a software delay. */
    if (getreg8(RA6M5_SYSTEM_REG(RA6M5_SYS_LOCOCR_OFFSET)) == 0)
    {
        /* Start LOCO */
        putreg8(0, RA6M5_SYSTEM_REG(RA6M5_SYS_LOCOCR_OFFSET));
        up_udelay(100);
    }

    /* Start MOSC */
    putreg8(0, RA6M5_SYSTEM_REG(RA6M5_SYS_MOSCCR_OFFSET));

    /* Wait for main oscillator to stabilize. */
    do {
        regval = getreg8(RA6M5_SYSTEM_REG(RA6M5_SYS_OSCSF_OFFSET));
    } while (!(regval & SYS_OSCSF_MOSCSF));

    /* Start clocks that require other clocks. At this point, all dependent clocks are running and stable if needed. */
    
    /*
        PLL2 SRC : MOSC
        PLL2 DIV : 2
        PLL2 MUL : 24
        PLL2 FRQ : 192MHZ
    */
    putreg16(BSP_PRV_PLL2CCR, RA6M5_SYSTEM_REG(RA6M5_SYS_PLL2CCR_OFFSET));

    /* Start PLL2. */
    putreg8(0, RA6M5_SYSTEM_REG(RA6M5_SYS_PLL2CR_OFFSET));

    /* Wait for PLL2 to stabilize. */
    do {
        regval = getreg8(RA6M5_SYSTEM_REG(RA6M5_SYS_OSCSF_OFFSET));
    } while (!(regval & SYS_OSCSF_PLL2SF));

    /*
        PLL SRC : MOSC
        PLL DIV : 2
        PLL MUL : 25
        PLL FRQ : 200MHZ
    */

     /* Configure the PLL registers. */
    putreg16(BSP_PRV_PLLCCR, RA6M5_SYSTEM_REG(RA6M5_SYS_PLLCCR_OFFSET));

    /* Start PLL */
    putreg8(0, RA6M5_SYSTEM_REG(RA6M5_SYS_PLLCR_OFFSET));

    /* Wait for PLL to stabilize. */
    do {
        regval = getreg8(RA6M5_SYSTEM_REG(RA6M5_SYS_OSCSF_OFFSET));
    } while (!(regval & SYS_OSCSF_PLLSF));


    /* Set the wait states for ROM */
    putreg8(BSP_PRV_ROM_3_WAIT_CYCLES, RA6M5_FCACHE_REG(RA6M5_FCACHE_FLWT_OFFSET));

    /* The MCU must be in high speed mode to set wait states to 2. High speed mode is the default out of reset. */
    putreg8(BSP_PRV_MEMWAIT_2_WAIT_CYCLES, RA6M5_SYSTEM_REG(RA6M5_SYS_MEMWAIT_OFFSET));

    /* Set the system dividers after setting the system clock source if ICLK divisor is smaller than reset value. */
    putreg32(BSP_PRV_SCKDIVCR, RA6M5_SYSTEM_REG(RA6M5_SYS_SCKDIVCR_OFFSET));

    /* Set the system source clock */
    putreg8(RA6M5_CLOCKS_SOURCE_PLL, RA6M5_SYSTEM_REG(RA6M5_SYS_SCKSCR_OFFSET));

    /* Lock CGC and LPM protection registers. */
    putreg16(BSP_PRV_PRCR_LOCK, RA6M5_SYSTEM_REG(RA6M5_SYS_PRCR_OFFSET));

    /* Calculate the system clock frequency */
    uint32_t idx = getreg8(RA6M5_SYSTEM_REG(RA6M5_SYS_SCKSCR_OFFSET));
    uint32_t ick = (getreg32(RA6M5_SYSTEM_REG(RA6M5_SYS_SCKDIVCR_OFFSET)) >> 24) & 7;
    g_clock_freq[RA6M5_CLOCKS_SOURCE_SYSYEM] = g_clock_freq[idx] >> ick;
}
#endif
