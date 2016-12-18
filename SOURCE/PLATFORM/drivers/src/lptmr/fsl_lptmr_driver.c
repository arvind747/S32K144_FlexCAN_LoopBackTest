/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "fsl_lptmr_driver.h"
#include "fsl_lptmr_hal.h"
#include "fsl_interrupt_manager.h"
#include "fsl_clock_manager.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @cond DRIVER_INTERNAL_USE_ONLY */

/* Table of base addresses for LPTMR instances. */
static LPTMR_Type* const g_lptmrBase[LPTMR_INSTANCE_COUNT] = LPTMR_BASE_PTRS;
static const IRQn_Type g_lptmrIrqId[LPTMR_INSTANCE_COUNT] = LPTMR_IRQS;
/* PCC clock sources, for getting the input clock frequency */
static const clock_names_t g_lptmrClock[LPTMR_INSTANCE_COUNT] = {PCC_LPTMR0_CLOCK};

/*******************************************************************************
 * Private Functions
 ******************************************************************************/
static inline uint8_t lptmr_cfg2p(
    const lptmr_prescaler_t prescval,
    const bool bypass
    );

static inline uint64_t lptmr_us2nn(
    const uint32_t clkfreq,
    const uint32_t us
    );

static inline uint64_t lptmr_compute_nticks(
    uint64_t nn,
    uint8_t p
    );

static inline bool nticks2compare_ticks(
    uint64_t nticks,
    uint16_t* ticks
    );

static uint32_t lptmr_GetClkFreq(
    const lptmr_clocksource_t clksrc,
    const uint32_t instance
    );

static bool lptmr_Ticks2Us(
    const uint32_t clkfreq,
    const lptmr_prescaler_t pval,
    const bool bypass,
    const uint16_t ticks,
    uint32_t* const us
    );

static bool lptmr_Us2Ticks(
    const uint32_t clkfreq,
    const lptmr_prescaler_t prescval,
    const bool bypass,
    const uint32_t us,
    uint16_t* const ticks
    );

static bool lptmr_ChooseClkConfig(
    const uint32_t clkfreq,
    const uint32_t us,
    lptmr_prescaler_t* const prescval,
    bool* const bypass,
    uint16_t* const ticks
    );

static lptmr_status_t lptmr_SetTimerConfig(
    LPTMR_Type* const base,
    const lptmr_timer_config_t* const config,
    const uint32_t clkfreq
    );

/*TIMER MODE CONFIGURATION******************************************************
 *
 * Timer Mode - Prescaler settings calculations
 * --------------------------------------------
 *
 * Timer Mode configuration takes a period (timeout) value expressed in
 * micro-seconds. To convert this to LPTMR prescaler (and compare value)
 * settings, the closest match must be found.
 * For best precision, the lowest prescaler that allows the corresponding
 * compare value to fit in the 16-bit register will be chosen.
 *
 * Algorithm for choosing prescaler and compare values:
 * =============================================================================
 * In: tper_us (period in microseconds), fclk (input clock frequency in Hertz)
 * Out: nticks (timer ticks), p (prescaler coefficient, 2^p = prescaler value)
 * ---
 * 1) Compute nn = tper_us * fclk / 1000000
 * 2) for p = 0..16
 *  2.1) nticks = nn / 2^p
 *  2.2) if nticks < 0x10000
 *      2.2.1) STOP, found nticks and p
 * 3) nticks = 0xFFFF, p = 16
 * =============================================================================
 *
 * A few names used throughout the static functions affecting Timer mode:
 *  nn - total number of timer ticks (undivided, unprescaled) that is necessary
 *      for a particular timeout.
 *      nn = (tper_us * fclk) / 1000000 = nticks * npresc
 *
 *  tper_us - a period (or timeout) expressed in microsecond units. In most
 *      functions will be denoted as 'us' for microseconds.
 *
 *  nticks - number of timer ticks that is necessary for a particular timeout,
 *      after prescaling
 *
 *  npresc - prescaler value (1, 2, 4 ... 65536)
 *
 *  p - prescaler coefficient, 2^p = npresc
 *
 *  fclk - input clock frequency, in Hertz. In most function will be denoted as
 *      'clkfreq'.
 *END**************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : lptmr_cfg2p
 * Description   : Transform prescaler settings (bypass on/off, prescaler value)
 *  to prescaler coefficient value (2's power), p.
 * Return: the value of p.
 *END**************************************************************************/
static inline uint8_t lptmr_cfg2p(
    const lptmr_prescaler_t prescval,
    const bool bypass
    )
{
    uint8_t p = 0u;

    if (!bypass)
    {
        p = ( (uint8_t)prescval ) + 1u;
    }

    return p;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lptmr_us2nn
 * Description   : Transform microseconds to undivided (unprescaled) timer units,
 *  nn.
 * Return: the value of nn.
 *END**************************************************************************/
static inline uint64_t lptmr_us2nn(
    const uint32_t clkfreq,
    const uint32_t us
    )
{
    /* Approximate the timeout in undivided (unprescaled) timer ticks.
        - us is the timeout in microseconds (1/10^6 seconds)
        - clkfreq is the frequency in Hertz
        Operation:
        nn = (us/1000000) * clkfreq
        In C:
        For better precision, first to the multiplication (us * clkfreq)
        To overcome the truncation of the div operator in C, add half of the
        denominator before the division. Hence:
        nn = (us * clkfreq + 500000) / 1000000
    */
    /* There is no risk of overflow since us is 32-bit wide and clkfreq can be
       a theoretical maximum of ~100 MHz (platform maximum), which is over the
       maximum input of the LPTMR anyway
     */
    uint64_t nn = (uint64_t)( (uint64_t)us * (uint64_t)clkfreq );
    nn = (nn + 500000u) / 1000000u;
    return nn;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lptmr_compute_nticks
 * Description   : Compute total number of divided (prescaled) timer ticks,
 * nticks.
 * Return: the value of nticks.
 *END**************************************************************************/
static inline uint64_t lptmr_compute_nticks(
    uint64_t nn,
    uint8_t p
    )
{
    uint32_t npresc = 1u << p;
    /* integer division */
    uint64_t nticks = (uint64_t)( ( nn + (uint64_t)(npresc >> 1u) ) / (uint64_t)npresc );
    return nticks;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : nticks2compare_ticks
 * Description   : Transform the value of divided (prescaled) timer ticks, nticks
 *  to a 16-bit value to be written to the hardware register. Cap or underflow
 *  cause an error.
 * Return: the success state.
 *  - true: no underflow or overflow detected
 *  - false: value written was capped, underflow or overflow detected
 *
 *END**************************************************************************/
static inline bool nticks2compare_ticks(
    uint64_t nticks,
    uint16_t* ticks
    )
{
    bool success = true;

    /* if nticks fits in 16 bits, write the value to ticks */
    if (nticks <= 0x10000u)
    {
        if (nticks == 0u)
        {
            /* timeout period (us) too low for prescaler settings */
            *ticks = 0u;
            success = false;
        }
        else{
            /* decrement nticks by one for the compare register */
            *ticks = (uint16_t)(nticks - 1u);
        }
    }
    else {
        /* timeout period (us) too high for prescaler settings */
        *ticks = 0xFFFFu;
        success = false;
    }

    return success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lptmr_GetClkFreq
 * Description   : Get the clock frequency for the selected clock source. If the
 * selected clock source is not enabled, a frequency of 0 is returned.
 * Return values:
 *  - the clock frequency or 0 if the clock is invalid.
 *
 *END**************************************************************************/
static uint32_t lptmr_GetClkFreq(
    const lptmr_clocksource_t clksrc,
    const uint32_t instance
    )
{
    uint32_t clkfreq = 0u;
    clock_names_t lptmr_input_clock_name = SIRC_CLOCK;
    bool getfreq = true;
    switch(clksrc)
    {
    case LPTMR_CLOCKSOURCE_SIRC:
        clkfreq = SCG_HAL_GetSircAsyncFreq(SCG, SCG_ASYNC_CLOCK_DIV2);
        getfreq = false;
        break;
    case LPTMR_CLOCKSOURCE_1KHZ_LPO:
        lptmr_input_clock_name = SIM_LPO_1K_CLOCK;
        break;
    case LPTMR_CLOCKSOURCE_32KHZ_LPO:
        lptmr_input_clock_name = SIM_CLK32K_CLOCK;
        break;
    case LPTMR_CLOCKSOURCE_PCC:
        lptmr_input_clock_name = g_lptmrClock[instance];
        break;
    default:
        /* Invalid clock source */
        getfreq = false;
        break;
    }

    if (getfreq)
    {
        /* If the GetFreq functions fails, clkfreq will be 0 */
        (void)CLOCK_SYS_GetFreq(lptmr_input_clock_name, &clkfreq);
    }

    return clkfreq;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lptmr_Ticks2Us
 * Description   : Transform timer ticks to microseconds using the given
 *  prescaler settings. Clock frequency must be valid ( different from 0).
 * Possible return values:
 * - true: conversion success
 * - false: conversion failed, result did not fit in 32-bit.
 *
 *END**************************************************************************/
static bool lptmr_Ticks2Us(
    const uint32_t clkfreq,
    const lptmr_prescaler_t pval,
    const bool bypass,
    const uint16_t ticks,
    uint32_t* const us
    )
{
    bool success = true;
    uint8_t p = lptmr_cfg2p(pval, bypass);
    uint64_t nn = ( (uint64_t)ticks + 1u ) << p;
    uint64_t us_real = (nn * 1000000u) / (clkfreq);
    uint32_t us_local;

    if ( us_real <= (0xFFFFFFFFu) )
    {
        us_local = (uint32_t)us_real;
    }
    else
    {
        us_local = 0xFFFFFFFFu;
        success = false;
    }

    *us = us_local;
    return success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lptmr_Us2Ticks
 * Description   : Transform microseconds to timer ticks using the given
 *  prescaler settings. Input clock frequency, clkfreq, must be greater than 0.
 * Possible return values:
 * - true: conversion completed successfully
 * - false: conversion failed, value did not fit in 16-bit.
 *
 *END**************************************************************************/
static bool lptmr_Us2Ticks(
    const uint32_t clkfreq,
    const lptmr_prescaler_t prescval,
    const bool bypass,
    const uint32_t us,
    uint16_t* const ticks
    )
{
    bool success = true;
    /* Transform prescaler configuration to prescaler coefficient p */
    uint8_t p = lptmr_cfg2p(prescval, bypass);
    /* Compute nn, the number of ticks necessary for the period in microseconds
        without any prescaler */
    uint64_t nn = lptmr_us2nn(clkfreq, us);
    /* Compute nticks, total number of ticks with prescaler */
    uint64_t nticks = lptmr_compute_nticks(nn, p);
    /* Transform nticks to value to be written to register */
    success = nticks2compare_ticks(nticks, ticks);
    return success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lptmr_ChooseClkConfig
 * Description   : Choose clocking configuration (prescaler value, timer ticks)
 *  for the desired timeout period, given in microseconds. Input clock frequency,
 *  clkfreq, must be greater than 0.
 * Possible return values:
 * - true: configuration found
 * - false: configuration mismatch, desired timeout period is too small or too
 * big for the clock settings.
 *
 *END**************************************************************************/
static bool lptmr_ChooseClkConfig(
    const uint32_t clkfreq,
    const uint32_t us,
    lptmr_prescaler_t* const prescval,
    bool* const bypass,
    uint16_t* const ticks
    )
{
    uint8_t p;
    uint64_t nticks;
    bool success;

    uint64_t nn = lptmr_us2nn(clkfreq, us);

    /* Find the lowest prescaler value that allows the compare value in 16-bits */
    for (p = 0u; p <= 16u; p++)
    {
        nticks = lptmr_compute_nticks(nn, p);

        if (nticks <= 0x10000u)
        {
            /* Search finished, value will fit in the 16-bit register */
            break;
        }
    }

    success = nticks2compare_ticks(nticks, ticks);

    /* Convert p to prescaler configuration */
    if (p == 0u)
    {
        /* Prescaler value of 1 */
        *bypass = true;
        *prescval = LPTMR_PRESCALE_2;
    }
    else{
        *bypass = false;
        *prescval = (lptmr_prescaler_t)(p - 1u);
    }

    return success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : lptmr_SetTimerConfig
 * Description   : This function configures the timer settings in Timer Mode,
 *  using microseconds for the period/timeout value.
 * Possible return values:
 * - LPTMR_DRV_SUCCESS: settings found and written to hardware.
 * - LPTMR_DRV_WARNING_INCORRECT_TIMING: settings might not be correct (period
 *  is too small/ too large for the selected clock source) but the closest
 *  possible settings were written.
 *
 *END**************************************************************************/
static lptmr_status_t lptmr_SetTimerConfig(
    LPTMR_Type* const base,
    const lptmr_timer_config_t* const config,
    const uint32_t clkfreq
    )
{
    lptmr_status_t return_code = LPTMR_DRV_SUCCESS;

    /* Choose what compare value to use for prescaler calculation */
    lptmr_prescaler_t prescval = LPTMR_PRESCALE_2;
    bool bypass = false;
    uint16_t compareval = 0u;
    bool success = true;

    if (!config->freeRun)
    {
        /* Find the best clock configuration */
        success = lptmr_ChooseClkConfig(clkfreq, config->compareValueUs, &prescval,
            &bypass, &compareval);
    }
    else
    {
        /* Find the smallest prescaler configuration that fits maxCompareValueUs*/
        uint16_t dummyval;
        bool success_config = lptmr_ChooseClkConfig(clkfreq, config->maxCompareValueUs, &prescval,
            &bypass, &dummyval);
        bool success_conv = lptmr_Us2Ticks(clkfreq, prescval, bypass, config->compareValueUs,
            &compareval);
        success = success_config && success_conv;
    }

    if (!success)
    {
        return_code = LPTMR_DRV_WARNING_INCORRECT_TIMING;
    }

    /* Write configuration */
    LPTMR_HAL_SetDmaRequest(base, config->dmaRequest);
    LPTMR_HAL_SetInterrupt(base, config->interruptEnable);
    LPTMR_HAL_SetFreeRunning(base, config->freeRun);
    LPTMR_HAL_SetWorkMode(base, LPTMR_WORKMODE_TIMER);
    LPTMR_HAL_SetClockSelect(base, config->clockSelect);
    LPTMR_HAL_SetPrescaler(base, prescval);
    LPTMR_HAL_SetBypass(base, bypass);
    LPTMR_HAL_SetCompareValue(base, compareval);

    return return_code;
}

/*! @endcond */

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : LPTMR_DRV_InitGeneralConfigStruct
 * Description   : Initialize the General use-case (Timer or Pulse-Counter)
 * configuration structure with default values.
 * Possible return values:
 * - LPTMR_DRV_SUCCESS.
 *
 *END**************************************************************************/
lptmr_status_t LPTMR_DRV_InitGeneralConfigStruct(
    lptmr_general_config_t* const config
    )
{
    DEV_ASSERT(config != NULL);

    /* Initialize struct with default values */
    config->dmaRequest = false;
    config->interruptEnable = false;
    config->pinSelect = LPTMR_PINSELECT_TRGMUX;
    config->pinPolarity = LPTMR_PINPOLARITY_RISING;
    config->freeRun = false;
    config->workMode = LPTMR_WORKMODE_TIMER;
    config->prescaler = LPTMR_PRESCALE_2;
    config->bypassPrescaler = false;
    config->clockSelect = LPTMR_CLOCKSOURCE_SIRC;
    config->compareValue = 0u;

    return LPTMR_DRV_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPTMR_DRV_InitGeneral
 * Description   : Initialize the hardware with given configuration in the
 *  General use-case (Timer or Pulse-Counter).
 * Possible return values:
 * - return value from LPTMR_DRV_SetGeneralConfig
 *
 *END**************************************************************************/
lptmr_status_t LPTMR_DRV_InitGeneral(
    const uint32_t instance,
    const lptmr_general_config_t* const config,
    const bool startcounter
    )
{
    DEV_ASSERT(instance < LPTMR_INSTANCE_COUNT);
    DEV_ASSERT(config != NULL);

    LPTMR_Type* const base = g_lptmrBase[instance];

    lptmr_status_t status = LPTMR_DRV_SetGeneralConfig(instance, config);

    /* Start the counter if requested */
    if ( (status == LPTMR_DRV_SUCCESS) && (startcounter) )
    {
        LPTMR_HAL_Enable(base);
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPTMR_DRV_SetGeneralConfig
 * Description   : Configure the hardware in the General use-case
 *  (Timer or Pulse-Counter).
 * Possible return values:
 * - LPTMR_DRV_SUCCESS:
 * - LPTMR_DRV_FAIL_CANNOT_CONFIGURE: counter already started
 * - LPTMR_DRV_FAIL_INVALID_CLOCK: invalid clock source
 *
 *END**************************************************************************/
lptmr_status_t LPTMR_DRV_SetGeneralConfig(
    const uint32_t instance,
    const lptmr_general_config_t* const config
    )
{
    DEV_ASSERT(instance < LPTMR_INSTANCE_COUNT);
    DEV_ASSERT(config != NULL);

    LPTMR_Type* const base = g_lptmrBase[instance];

    /* Check if the peripheral is enabled preventing a new configuration */
    bool enabled = LPTMR_HAL_GetEnable(base);

    if (enabled)
    {
        return LPTMR_DRV_FAIL_CANNOT_CONFIGURE;
    }

    /* Check if a valid clock is selected for the timer/glitch filter */
    if ( (!config->bypassPrescaler) || (config->workMode == LPTMR_WORKMODE_TIMER) )
    {
        uint32_t clkfreq = lptmr_GetClkFreq(config->clockSelect, instance);

        if (clkfreq == 0u)
        {
            return LPTMR_DRV_FAIL_INVALID_CLOCK;
        }
    }

    /* Initialize and write configuration */
    LPTMR_HAL_Init(base);
    LPTMR_HAL_SetDmaRequest(base, config->dmaRequest);
    LPTMR_HAL_SetInterrupt(base, config->interruptEnable);
    LPTMR_HAL_SetPinSelect(base, config->pinSelect);
    LPTMR_HAL_SetPinPolarity(base, config->pinPolarity);
    LPTMR_HAL_SetFreeRunning(base, config->freeRun);
    LPTMR_HAL_SetWorkMode(base, config->workMode);
    LPTMR_HAL_SetPrescaler(base, config->prescaler);
    LPTMR_HAL_SetBypass(base, config->bypassPrescaler);
    LPTMR_HAL_SetClockSelect(base, config->clockSelect);
    LPTMR_HAL_SetCompareValue(base, config->compareValue);

    return LPTMR_DRV_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPTMR_DRV_GetGeneralConfig
 * Description   : Return the current configuration of the LPTMR in the General
 * use-case layout.
 * Possible return values:
 * - LPTMR_DRV_SUCCESS:
 *
 *END**************************************************************************/
lptmr_status_t LPTMR_DRV_GetGeneralConfig(
    const uint32_t instance,
    lptmr_general_config_t* const config
    )
{
    DEV_ASSERT(instance < LPTMR_INSTANCE_COUNT);
    DEV_ASSERT(config != NULL);

    LPTMR_Type* const base = g_lptmrBase[instance];

    /* Read configuration */
    config->dmaRequest = LPTMR_HAL_GetDmaRequest(base);
    config->interruptEnable = LPTMR_HAL_GetInterruptEnable(base);
    config->pinSelect = LPTMR_HAL_GetPinSelect(base);
    config->pinPolarity = LPTMR_HAL_GetPinPolarity(base);
    config->freeRun = LPTMR_HAL_GetFreeRunning(base);
    config->workMode = LPTMR_HAL_GetWorkMode(base);
    config->prescaler = LPTMR_HAL_GetPrescaler(base);
    config->bypassPrescaler = LPTMR_HAL_GetBypass(base);
    config->clockSelect = LPTMR_HAL_GetClockSelect(base);
    config->compareValue = LPTMR_HAL_GetCompareValue(base);

    return LPTMR_DRV_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPTMR_DRV_InitTimerConfigStruct
 * Description   : Initialize the Timer use-case configuration structure with
 *  default values.
 * Possible return values:
 * - LPTMR_DRV_SUCCESS:
 *
 *END**************************************************************************/
lptmr_status_t LPTMR_DRV_InitTimerConfigStruct(
    lptmr_timer_config_t* const config
    )
{
    DEV_ASSERT(config != NULL);

    /* Initialize struct with default values */
    config->dmaRequest = false;
    config->interruptEnable = false;
    config->freeRun = false;
    config->clockSelect = LPTMR_CLOCKSOURCE_SIRC;
    config->compareValueUs = 0u;

    return LPTMR_DRV_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPTMR_DRV_InitTimer
 * Description   :  Initialize the hardware with given configuration in the
 *  Timer use-case.
 * Possible return values:
 * - LPTMR_DRV_SUCCESS:
 * - LPTMR_DRV_FAIL_CANNOT_CONFIGURE: timer already running
 * - LPTMR_DRV_FAIL_INVALID_CLOCK: clock source invalid. No settings written.
 * - return code from lptmr_SetTimerConfig
 *
 *END**************************************************************************/
lptmr_status_t LPTMR_DRV_InitTimer(
    const uint32_t instance,
    const lptmr_timer_config_t* const config,
    const bool startcounter
    )
{
    lptmr_status_t status = LPTMR_DRV_SUCCESS;

    DEV_ASSERT(instance < LPTMR_INSTANCE_COUNT);
    DEV_ASSERT(config != NULL);

    LPTMR_Type* const base = g_lptmrBase[instance];

    /* Check if the peripheral is enabled preventing a new configuration */
    bool enabled = LPTMR_HAL_GetEnable(base);

    if (enabled)
    {
        return LPTMR_DRV_FAIL_CANNOT_CONFIGURE;
    }

    uint32_t clkfreq = lptmr_GetClkFreq(config->clockSelect, instance);

    if (clkfreq == 0u)
    {
        return LPTMR_DRV_FAIL_INVALID_CLOCK;
    }

    LPTMR_HAL_Init(base);
    status = lptmr_SetTimerConfig(base, config, clkfreq);

    /* Start the counter if requested */
    if ( (status == LPTMR_DRV_SUCCESS) && (startcounter) )
    {
        LPTMR_HAL_Enable(base);
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPTMR_DRV_SetTimerConfig
 * Description   : Configure the hardware in the Timer use-case. Can be used
 *  only in Timer Mode.
 * Possible return values:
 * - LPTMR_DRV_SUCCESS:
 * - LPTMR_DRV_FAIL_CANNOT_CONFIGURE: timer started
 * - LPTMR_DRV_FAIL_MODE_MISMATCH: not in Timer Mode
 * - LPTMR_DRV_FAIL_INVALID_CLOCK: clock source invalid. No settings written.
 * - return code from lptmr_SetTimerConfig
 *
 *END**************************************************************************/
lptmr_status_t LPTMR_DRV_SetTimerConfig(
    const uint32_t instance,
    const lptmr_timer_config_t* const config
    )
{
    lptmr_status_t return_code = LPTMR_DRV_SUCCESS;

    DEV_ASSERT(instance < LPTMR_INSTANCE_COUNT);
    DEV_ASSERT(config != NULL);

    LPTMR_Type* const base = g_lptmrBase[instance];

    /* Check if the peripheral is enabled preventing a new configuration */
    bool enabled = LPTMR_HAL_GetEnable(base);

    if (enabled)
    {
        return LPTMR_DRV_FAIL_CANNOT_CONFIGURE;
    }

    /* Check if the peripheral is in Timer mode */
    lptmr_workmode_t workmode = LPTMR_HAL_GetWorkMode(base);

    if (workmode != LPTMR_WORKMODE_TIMER)
    {
        return LPTMR_DRV_FAIL_MODE_MISMATCH;
    }

    uint32_t clkfreq = lptmr_GetClkFreq(config->clockSelect, instance);

    if (clkfreq == 0u)
    {
        return LPTMR_DRV_FAIL_INVALID_CLOCK;
    }

    return_code = lptmr_SetTimerConfig(base, config, clkfreq);
    return return_code;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPTMR_DRV_GetTimerConfig
 * Description   : Return the current configuration of the LPTMR in the Timer
 * use-case layout. Can be used only in Timer Mode.
 * Possible return values:
 * - LPTMR_DRV_SUCCESS:
 * - LPTMR_DRV_FAIL_INVALID_CLOCK: no clock source
 * - LPTMR_DRV_FAIL_MODE_MISMATCH: not in Timer Mode
 * - LPTMR_DRV_WARNING_INCORRECT_TIMING: computed settings might not be correct
 *
 *END**************************************************************************/
lptmr_status_t LPTMR_DRV_GetTimerConfig(
    const uint32_t instance,
    lptmr_timer_config_t* const config
    )
{
    lptmr_status_t return_code = LPTMR_DRV_SUCCESS;

    DEV_ASSERT(instance < LPTMR_INSTANCE_COUNT);
    DEV_ASSERT(config != NULL);

    LPTMR_Type* const base = g_lptmrBase[instance];

    /* Check if the peripheral is in Timer mode */
    lptmr_workmode_t workmode = LPTMR_HAL_GetWorkMode(base);

    if (workmode != LPTMR_WORKMODE_TIMER)
    {
        return LPTMR_DRV_FAIL_MODE_MISMATCH;
    }

    uint32_t clkfreq = lptmr_GetClkFreq(config->clockSelect, instance);

    if (clkfreq == 0u)
    {
        return LPTMR_DRV_FAIL_INVALID_CLOCK;
    }

    /* Read configuration */
    config->dmaRequest = LPTMR_HAL_GetDmaRequest(base);
    config->interruptEnable = LPTMR_HAL_GetInterruptEnable(base);
    config->freeRun = LPTMR_HAL_GetFreeRunning(base);
    config->clockSelect = LPTMR_HAL_GetClockSelect(base);
    /* compute us from freq and prescaler */
    lptmr_prescaler_t prescval = LPTMR_HAL_GetPrescaler(base);
    bool bypass = LPTMR_HAL_GetBypass(base);
    uint16_t ticks = LPTMR_HAL_GetCompareValue(base);
    bool success = lptmr_Ticks2Us( clkfreq, prescval, bypass, ticks,
        &(config->compareValueUs) );

    if (!success)
    {
        return_code = LPTMR_DRV_WARNING_INCORRECT_TIMING;
    }

    return return_code;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPTMR_DRV_SetCompareValueUs
 * Description   : Set the Compare Value in Timer mode using microsecond units.
 *  Can be used only in Timer Mode.
 * Possible return values:
 * - LPTMR_DRV_SUCCESS:
 * - LPTMR_DRV_FAIL_CANNOT_CONFIGURE: timer started
 * - LPTMR_DRV_FAIL_MODE_MISMATCH: not in Timer Mode
 * - LPTMR_DRV_FAIL_INVALID_CLOCK: no clock source
 * - LPTMR_DRV_WARNING_EVENT_EXPIRED: compare value set too late
 * - LPTMR_DRV_WARNING_INCORRECT_TIMING: timing value might not be correct
 *
 *END**************************************************************************/
lptmr_status_t LPTMR_DRV_SetCompareValueUs(
    const uint32_t instance,
    uint32_t compareValueUs
    )
{
    lptmr_status_t return_code = LPTMR_DRV_SUCCESS;

    DEV_ASSERT(instance < LPTMR_INSTANCE_COUNT);

    LPTMR_Type* const base = g_lptmrBase[instance];

    /* Check if the compare value can be written:
        - counter must be disabled or the compare flag set
     */
    bool enabled = LPTMR_HAL_GetEnable(base);
    bool compareflag = LPTMR_HAL_GetCompareFlag(base);

    if (enabled && !compareflag)
    {
        return LPTMR_DRV_FAIL_CANNOT_CONFIGURE;
    }

    /* Check if the peripheral is in Timer mode */
    lptmr_workmode_t workmode = LPTMR_HAL_GetWorkMode(base);

    if (workmode != LPTMR_WORKMODE_TIMER)
    {
        return LPTMR_DRV_FAIL_MODE_MISMATCH;
    }

    /* Get configured clock value */
    lptmr_clocksource_t clksrc = LPTMR_HAL_GetClockSelect(base);
    uint32_t clkfreq = lptmr_GetClkFreq(clksrc, instance);

    if (clkfreq == 0u)
    {
        /* INVALID_CLOCKING */
        return LPTMR_DRV_FAIL_INVALID_CLOCK;
    }

    /* Get prescaler, bypass and compare value */
    lptmr_prescaler_t prescval = LPTMR_HAL_GetPrescaler(base);
    bool bypass = LPTMR_HAL_GetBypass(base);
    /* Compute ticks value */
    /* Prescaler *cannot* be changed */
    uint16_t val;
    bool success = lptmr_Us2Ticks(clkfreq, prescval, bypass, compareValueUs,
        &val);
    /* Write value and check if written successfully */
    LPTMR_HAL_SetCompareValue(base, val);
    uint16_t counterval = LPTMR_HAL_GetCounterValue(base);

    if (counterval >= val)
    {
        return_code = LPTMR_DRV_WARNING_EVENT_EXPIRED;
    }

    if (!success)
    {
        return_code = LPTMR_DRV_WARNING_INCORRECT_TIMING;
    }

    return return_code;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPTMR_DRV_GetCompareValueUs
 * Description   : Return the Compare Value in Timer mode using microsecond
 *  representation. Can be used only in Timer Mode.
 * Possible return values:
 * - LPTMR_DRV_SUCCESS:
 * - LPTMR_DRV_FAIL_INVALID_CLOCK: no clock source
 * - LPTMR_DRV_FAIL_MODE_MISMATCH: not in Timer Mode
 * - LPTMR_DRV_WARNING_INCORRECT_TIMING: timing value might not be correct
 *
 *END**************************************************************************/
lptmr_status_t LPTMR_DRV_GetCompareValueUs(
    const uint32_t instance,
    uint32_t* const compareValueUs
    )
{
    lptmr_status_t return_code = LPTMR_DRV_SUCCESS;

    DEV_ASSERT(instance < LPTMR_INSTANCE_COUNT);
    DEV_ASSERT(compareValueUs != NULL);

    LPTMR_Type* const base = g_lptmrBase[instance];

    /* Check if the peripheral is in Timer mode */
    lptmr_workmode_t workmode = ( LPTMR_HAL_GetWorkMode(base) );

    if (workmode != LPTMR_WORKMODE_TIMER)
    {
        return LPTMR_DRV_FAIL_MODE_MISMATCH;
    }

    /* Get configured clock value */
    lptmr_clocksource_t clksrc = LPTMR_HAL_GetClockSelect(base);
    uint32_t clkfreq = lptmr_GetClkFreq(clksrc, instance);

    if (clkfreq == 0u)
    {
        return LPTMR_DRV_FAIL_INVALID_CLOCK;
    }

    /* Get prescaler, bypass and compare value */
    lptmr_prescaler_t prescval = LPTMR_HAL_GetPrescaler(base);
    bool bypass = LPTMR_HAL_GetBypass(base);
    uint16_t ticks = LPTMR_HAL_GetCompareValue(base);
    /* Compute the compare value in microsec */
    bool success = lptmr_Ticks2Us(clkfreq, prescval, bypass, ticks,
        compareValueUs);

    if (!success)
    {
        return_code = LPTMR_DRV_WARNING_INCORRECT_TIMING;
    }

    return return_code;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPTMR_DRV_InitPulseCounterConfigStruct
 * Description   : Initialize the Pulse-Counter use-case configuration
 *  structure with default values.
 * Possible return values:
 * - LPTMR_DRV_SUCCESS:
 *
 *END**************************************************************************/
lptmr_status_t LPTMR_DRV_InitPulseCounterConfigStruct(
    lptmr_pulsecounter_config_t* const config
    )
{
    DEV_ASSERT(config != NULL);

    /* Initialize struct with default values */
    config->dmaRequest = false;
    config->interruptEnable = false;
    config->pinSelect = LPTMR_PINSELECT_TRGMUX;
    config->pinPolarity = LPTMR_PINPOLARITY_RISING;
    config->freeRun = false;
    config->prescaler = LPTMR_PRESCALE_4_GLITCHFILTER_2;
    config->bypassPrescaler = false;
    config->clockSelect = LPTMR_CLOCKSOURCE_SIRC;
    config->compareValue = 0u;

    return LPTMR_DRV_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPTMR_DRV_InitPulseCounter
 * Description   : Initialize the hardware with given configuration in the
 *  Pulse-Counter use-case.
 * Possible return values:
 * - LPTMR_DRV_SUCCESS:
 * - LPTMR_DRV_FAIL_CANNOT_CONFIGURE: counter already started
 * - LPTMR_DRV_FAIL_INVALID_CLOCK: invalid clock source
 *
 *END**************************************************************************/
lptmr_status_t LPTMR_DRV_InitPulseCounter(
    const uint32_t instance,
    const lptmr_pulsecounter_config_t* const config,
    const bool startcounter
    )
{
    DEV_ASSERT(instance < LPTMR_INSTANCE_COUNT);
    DEV_ASSERT(config != NULL);

    LPTMR_Type* const base = g_lptmrBase[instance];

    /* Check if the peripheral is enabled preventing a new configuration */
    bool enabled = LPTMR_HAL_GetEnable(base);

    if (enabled)
    {
        return LPTMR_DRV_FAIL_CANNOT_CONFIGURE;
    }

    /* Check if a valid clock is selected for the glitch filter */
    if (!config->bypassPrescaler)
    {
        uint32_t clkfreq = lptmr_GetClkFreq(config->clockSelect, instance);

        if (clkfreq == 0u)
        {
            return LPTMR_DRV_FAIL_INVALID_CLOCK;
        }
    }

    /* Initialize and write configuration */
    LPTMR_HAL_Init(base);
    LPTMR_HAL_SetDmaRequest(base, config->dmaRequest);
    LPTMR_HAL_SetInterrupt(base, config->interruptEnable);
    LPTMR_HAL_SetPinSelect(base, config->pinSelect);
    LPTMR_HAL_SetPinPolarity(base, config->pinPolarity);
    LPTMR_HAL_SetFreeRunning(base, config->freeRun);
    LPTMR_HAL_SetWorkMode(base, LPTMR_WORKMODE_PULSECOUNTER);
    LPTMR_HAL_SetPrescaler(base, config->prescaler);
    LPTMR_HAL_SetBypass(base, config->bypassPrescaler);
    LPTMR_HAL_SetClockSelect(base, config->clockSelect);
    LPTMR_HAL_SetCompareValue(base, config->compareValue);

    /* Start the counter if requested */
    if (startcounter)
    {
        LPTMR_HAL_Enable(base);
    }

    return LPTMR_DRV_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPTMR_DRV_SetPulseCounterConfig
 * Description   : Configure the hardware in the Pulse-Counter use-case. Can be
 *  used only in Pulse-Counter Mode.
 * Possible return values:
 * - LPTMR_DRV_SUCCESS:
 * - LPTMR_DRV_FAIL_MODE_MISMATCH: not in Pulse-Counter mode
 * - LPTMR_DRV_FAIL_CANNOT_CONFIGURE: counter already started
 * - LPTMR_DRV_FAIL_INVALID_CLOCK: invalid clock source for the glitch filter.
 *
 *END**************************************************************************/
lptmr_status_t LPTMR_DRV_SetPulseCounterConfig(
    const uint32_t instance,
    const lptmr_pulsecounter_config_t* const config
    )
{
    DEV_ASSERT(instance < LPTMR_INSTANCE_COUNT);
    DEV_ASSERT(config != NULL);

    LPTMR_Type* const base = g_lptmrBase[instance];

    /* Check if the peripheral is enabled preventing a new configuration */
    bool enabled = LPTMR_HAL_GetEnable(base);

    if (enabled)
    {
        return LPTMR_DRV_FAIL_CANNOT_CONFIGURE;
    }

    /* Check if the peripheral is in Pulse Counter mode */
    lptmr_workmode_t workmode = LPTMR_HAL_GetWorkMode(base);

    if (workmode != LPTMR_WORKMODE_PULSECOUNTER)
    {
        return LPTMR_DRV_FAIL_MODE_MISMATCH;
    }

    /* Check if a valid clock is selected for the glitch filter */
    if (!config->bypassPrescaler)
    {
        uint32_t clkfreq = lptmr_GetClkFreq(config->clockSelect, instance);

        if (clkfreq == 0u)
        {
            return LPTMR_DRV_FAIL_INVALID_CLOCK;
        }
    }

    /* Write configuration */
    LPTMR_HAL_SetDmaRequest(base, config->dmaRequest);
    LPTMR_HAL_SetInterrupt(base, config->interruptEnable);
    LPTMR_HAL_SetPinSelect(base, config->pinSelect);
    LPTMR_HAL_SetPinPolarity(base, config->pinPolarity);
    LPTMR_HAL_SetFreeRunning(base, config->freeRun);
    LPTMR_HAL_SetPrescaler(base, config->prescaler);
    LPTMR_HAL_SetBypass(base, config->bypassPrescaler);
    LPTMR_HAL_SetClockSelect(base, config->clockSelect);
    LPTMR_HAL_SetCompareValue(base, config->compareValue);

    return LPTMR_DRV_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPTMR_DRV_GetPulseCounterConfig
 * Description   : Return the current configuration of the LPTMR in the
 * Pulse-Counter use-case layout. Can be used only in Pulse-Counter Mode.
 * Possible return values:
 * - LPTMR_DRV_SUCCESS:
 * - LPTMR_DRV_FAIL_MODE_MISMATCH: not in Pulse-Counter mode.
 *
 *END**************************************************************************/
lptmr_status_t LPTMR_DRV_GetPulseCounterConfig(
    const uint32_t instance,
    lptmr_pulsecounter_config_t* const config
    )
{
    DEV_ASSERT(instance < LPTMR_INSTANCE_COUNT);
    DEV_ASSERT(config != NULL);

    LPTMR_Type* const base = g_lptmrBase[instance];

    /* Check if the peripheral is in Pulse Counter mode */
    lptmr_workmode_t workmode = LPTMR_HAL_GetWorkMode(base);

    if (workmode != LPTMR_WORKMODE_PULSECOUNTER)
    {
        return LPTMR_DRV_FAIL_MODE_MISMATCH;
    }

    /* Read configuration */
    config->dmaRequest = LPTMR_HAL_GetDmaRequest(base);
    config->interruptEnable = LPTMR_HAL_GetInterruptEnable(base);
    config->pinSelect = LPTMR_HAL_GetPinSelect(base);
    config->pinPolarity = LPTMR_HAL_GetPinPolarity(base);
    config->freeRun = LPTMR_HAL_GetFreeRunning(base);
    config->prescaler = LPTMR_HAL_GetPrescaler(base);
    config->bypassPrescaler = LPTMR_HAL_GetBypass(base);
    config->clockSelect = LPTMR_HAL_GetClockSelect(base);
    config->compareValue = LPTMR_HAL_GetCompareValue(base);

    return LPTMR_DRV_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPTMR_DRV_Deinit
 * Description   : De-initialize the LPTMR (stop the counter).
 * Possible return values:
 * - LPTMR_DRV_SUCCESS:
 *
 *END**************************************************************************/
lptmr_status_t LPTMR_DRV_Deinit(
    const uint32_t instance
    )
{
    DEV_ASSERT(instance < LPTMR_INSTANCE_COUNT);

    LPTMR_Type* const base = g_lptmrBase[instance];
    LPTMR_HAL_Disable(base);
    return LPTMR_DRV_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPTMR_DRV_SetCompareValueTicks
 * Description   : Set the Compare Value in Timer mode using timer units.
 * Possible return values:
 * - LPTMR_DRV_SUCCESS:
 * - LPTMR_DRV_FAIL_INVALID_CLOCK: no clock source
 * - LPTMR_DRV_FAIL_CANNOT_CONFIGURE: cannot reconfigure compare value
 * - LPTMR_DRV_WARNING_EVENT_EXPIRED: compare value set too late
 *
 *END**************************************************************************/
lptmr_status_t LPTMR_DRV_SetCompareValueTicks(
    const uint32_t instance,
    const uint16_t compareval
    )
{
    DEV_ASSERT(instance < LPTMR_INSTANCE_COUNT);

    LPTMR_Type* const base = g_lptmrBase[instance];

    /* Check if the compare value can be written:
        - counter must be disabled or the compare flag set
     */
    bool enabled = LPTMR_HAL_GetEnable(base);
    bool compareflag = LPTMR_HAL_GetCompareFlag(base);

    if (enabled && !compareflag)
    {
        return LPTMR_DRV_FAIL_CANNOT_CONFIGURE;
    }

    bool bypass = LPTMR_HAL_GetBypass(base);
    lptmr_workmode_t workmode = LPTMR_HAL_GetWorkMode(base);

    /* Check if a valid clock is selected for the timer/glitch filter */
    if ( (!bypass) || (workmode == LPTMR_WORKMODE_TIMER) )
    {
        lptmr_clocksource_t clocksrc = LPTMR_HAL_GetClockSelect(base);
        uint32_t clkfreq = lptmr_GetClkFreq(clocksrc, instance);

        if (clkfreq == 0u)
        {
            return LPTMR_DRV_FAIL_INVALID_CLOCK;
        }
    }

    /* Check if new value is below the current counter value */
    LPTMR_HAL_SetCompareValue(base, compareval);
    uint16_t counterval = LPTMR_HAL_GetCounterValue(base);

    if (counterval >= compareval)
    {
        return LPTMR_DRV_WARNING_EVENT_EXPIRED;
    }

    return LPTMR_DRV_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPTMR_DRV_GetCompareValueTicks
 * Description   : Return the Compare Value using timer ticks units
 *      representation.
 * Return value:
 *  - the compare value
 *
 *END**************************************************************************/
uint16_t LPTMR_DRV_GetCompareValueTicks(
    const uint32_t instance
    )
{
    DEV_ASSERT(instance < LPTMR_INSTANCE_COUNT);

    LPTMR_Type* const base = g_lptmrBase[instance];
    uint16_t comparevalue = LPTMR_HAL_GetCompareValue(base);
    return comparevalue;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPTMR_DRV_GetCompareFlag
 * Description   : Get the current state of the Compare Flag
 *
 *END**************************************************************************/
bool LPTMR_DRV_GetCompareFlag(
    const uint32_t instance
    )
{
    DEV_ASSERT(instance < LPTMR_INSTANCE_COUNT);

    LPTMR_Type* const base = g_lptmrBase[instance];
    bool compareflag = LPTMR_HAL_GetCompareFlag(base);
    return compareflag;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPTMR_DRV_ClearCompareFlag
 * Description   : Clear the Compare Flag.
 *
 *END**************************************************************************/
void LPTMR_DRV_ClearCompareFlag(
    const uint32_t instance
    )
{
    DEV_ASSERT(instance < LPTMR_INSTANCE_COUNT);

    LPTMR_Type* const base = g_lptmrBase[instance];
    LPTMR_HAL_ClearCompareFlag(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPTMR_DRV_SetInterrupt
 * Description   : Change the state of the Interrupt Enable setting.
 * Possible return values:
 * - LPTMR_DRV_SUCCESS:
 * - LPTMR_DRV_FAIL_CANNOT_CONFIGURE: cannot reconfigure setting
 *
 *END**************************************************************************/
lptmr_status_t LPTMR_DRV_SetInterrupt(
    const uint32_t instance,
    const bool enableinterrupt
    )
{
    DEV_ASSERT(instance < LPTMR_INSTANCE_COUNT);

    LPTMR_Type* const base = g_lptmrBase[instance];

    /* Check if the peripheral is enabled preventing a new configuration */
    bool enabled = LPTMR_HAL_GetEnable(base);

    if (enabled)
    {
        return LPTMR_DRV_FAIL_CANNOT_CONFIGURE;
    }

    LPTMR_HAL_SetInterrupt(base, enableinterrupt);
    return LPTMR_DRV_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPTMR_DRV_IsrInstallHandler
 * Description   : Install a new function to be executed as the LPTMR interrupt
 *  handler.
 * Possible return values:
 * - LPTMR_DRV_SUCCESS:
 *
 *END**************************************************************************/
lptmr_status_t LPTMR_DRV_IsrInstallHandler(
    const uint32_t instance,
    void (* isr)(
        void
        )
    )
{
    DEV_ASSERT(instance < LPTMR_INSTANCE_COUNT);
    DEV_ASSERT(isr != NULL);

    (void)INT_SYS_InstallHandler(g_lptmrIrqId[instance], isr, (isr_t*) 0);
    INT_SYS_EnableIRQ(g_lptmrIrqId[instance]);

    return LPTMR_DRV_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPTMR_DRV_GetCounterValueTicks
 * Description   : Get the current Counter Value using timer ticks
 * representation.
 * Return:
 *  - the counter value.
 *
 *END**************************************************************************/
uint16_t LPTMR_DRV_GetCounterValueTicks(
    const uint32_t instance
    )
{
    DEV_ASSERT(instance < LPTMR_INSTANCE_COUNT);

    LPTMR_Type* const base = g_lptmrBase[instance];
    uint16_t counterval = LPTMR_HAL_GetCounterValue(base);
    return counterval;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPTMR_DRV_StartCounter
 * Description   : Enable (start) the counter.
 * Possible return values:
 * - LPTMR_DRV_SUCCESS:
 *
 *END**************************************************************************/
lptmr_status_t LPTMR_DRV_StartCounter(
    const uint32_t instance
    )
{
    DEV_ASSERT(instance < LPTMR_INSTANCE_COUNT);

    LPTMR_Type* const base = g_lptmrBase[instance];
    LPTMR_HAL_Enable(base);
    return LPTMR_DRV_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPTMR_DRV_StopCounter
 * Description   : Disable (stop) the counter.
 * Possible return values:
 * - LPTMR_DRV_SUCCESS:
 *
 *END**************************************************************************/
lptmr_status_t LPTMR_DRV_StopCounter(
    const uint32_t instance
    )
{
    DEV_ASSERT(instance < LPTMR_INSTANCE_COUNT);

    LPTMR_Type* const base = g_lptmrBase[instance];
    LPTMR_HAL_Disable(base);
    return LPTMR_DRV_SUCCESS;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
