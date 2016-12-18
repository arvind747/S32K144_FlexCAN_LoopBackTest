/* * Copyright (c) 2014, Freescale Semiconductor, Inc.
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

#include "fsl_scg_hal.h"
#if FSL_FEATURE_SOC_SCG_COUNT

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_GetSystemClockFreq
 * Description   : This function gets the SCG system clock frequency, these
 * clocks are used for core, platform, external and bus clock domains.
 *
 *END*************************************************************************/
uint32_t SCG_HAL_GetSystemClockFreq(SCG_Type * base,
                                      scg_system_clock_type_t type)
{
    uint32_t freq;
    uint32_t regValue;
    scg_system_clock_src_t src = SCG_HAL_GetSystemClockSrc(base);

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(type < SCG_SYSTEM_CLOCK_MAX);
#endif

    switch(src)
    {
        case SCG_SYSTEM_CLOCK_SRC_SYS_OSC:
            freq = SCG_HAL_GetSysOscFreq(base);
            break;
        case SCG_SYSTEM_CLOCK_SRC_SIRC:
            freq = SCG_HAL_GetSircFreq(base);
            break;
        case SCG_SYSTEM_CLOCK_SRC_FIRC:
            freq = SCG_HAL_GetFircFreq(base);
            break;
        case SCG_SYSTEM_CLOCK_SRC_SYS_PLL:
            freq = SCG_HAL_GetSysPllFreq(base);
            break;
        default:
            freq = 0U;
            break;
    }

    regValue = base->CSR;
    regValue = (regValue & SCG_CSR_DIVCORE_MASK) >> SCG_CSR_DIVCORE_SHIFT;
    freq /= (regValue + 1U);

    switch(type)
    {
        case SCG_SYSTEM_CLOCK_CORE:
            /* Intentionally left blank */
            break;
        case SCG_SYSTEM_CLOCK_PLAT:
            regValue = base->CSR;
            regValue = (regValue & SCG_CSR_DIVPLAT_MASK) >> SCG_CSR_DIVPLAT_SHIFT;
            freq /= (regValue + 1U);
            break;
        case SCG_SYSTEM_CLOCK_BUS:
            regValue = base->CSR;
            regValue = (regValue & SCG_CSR_DIVBUS_MASK) >> SCG_CSR_DIVBUS_SHIFT;
            freq /= (regValue + 1U);
            break;
        case SCG_SYSTEM_CLOCK_SLOW:
            regValue = base->CSR;
            regValue = (regValue & SCG_CSR_DIVSLOW_MASK) >> SCG_CSR_DIVSLOW_SHIFT;
            freq /= (regValue + 1U);
            break;
        default:
            freq = 0U;
            break;
    }

    return freq;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_SetSystemClockConfig
 * Description   : This function sets the system configuration for the specified mode.
 *
 *END*************************************************************************/
void SCG_HAL_SetSystemClockConfig(SCG_Type * base,
                                  scg_system_clock_mode_t mode,
                                  scg_system_clock_config_t const *config)
{
    uint32_t value = (uint32_t)((((uint32_t)(config->src)     << SCG_CSR_SCS_SHIFT)     & SCG_CSR_SCS_MASK)     |
                                (((uint32_t)(config->divCore) << SCG_CSR_DIVCORE_SHIFT) & SCG_CSR_DIVCORE_MASK) |
                                (((uint32_t)(config->divPlat) << SCG_CSR_DIVPLAT_SHIFT) & SCG_CSR_DIVPLAT_MASK) |
                                (((uint32_t)(config->divBus)  << SCG_CSR_DIVBUS_SHIFT)  & SCG_CSR_DIVBUS_MASK)  |
                                (((uint32_t)(config->divSlow) << SCG_CSR_DIVSLOW_SHIFT) & SCG_CSR_DIVSLOW_MASK) );

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(mode != SCG_SYSTEM_CLOCK_MODE_CURRENT);
#endif

    switch (mode)
    {
        case SCG_SYSTEM_CLOCK_MODE_RUN:       /*!< Run mode.                */
            base->RCCR = value;
            break;
        case SCG_SYSTEM_CLOCK_MODE_VLPR:      /*!< Very Low Power Run mode. */
#ifdef DEV_ERROR_DETECT
            DEV_ASSERT((SCG_SYSTEM_CLOCK_SRC_SYS_OSC == config->src) ||
                       (SCG_SYSTEM_CLOCK_SRC_SIRC    == config->src));
#endif
            base->VCCR = value;
            break;
        case SCG_SYSTEM_CLOCK_MODE_HSRUN:     /*!< High Speed Run mode.     */
            base->HCCR = value;
            break;
        default:
            break;
    }
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_GetSystemClockConfig
 * Description   : This function gets the system configuration for the specified mode.
 *
 *END*************************************************************************/
void SCG_HAL_GetSystemClockConfig(SCG_Type * base,
                                  scg_system_clock_mode_t mode,
                                  scg_system_clock_config_t *config)
{
    uint32_t value;

    switch (mode)
    {
        case SCG_SYSTEM_CLOCK_MODE_CURRENT:   /*!< Current mode.            */
            value = base->CSR;
            break;
        case SCG_SYSTEM_CLOCK_MODE_RUN:       /*!< Run mode.                */
            value = base->RCCR;
            break;
        case SCG_SYSTEM_CLOCK_MODE_VLPR:      /*!< Very Low Power Run mode. */
            value = base->VCCR;
            break;
        case SCG_SYSTEM_CLOCK_MODE_HSRUN:     /*!< High Speed Run mode.     */
            value = base->HCCR;
            break;
        default:
            value = 0U;
            break;
    }

    config->src     = (scg_system_clock_src_t)
                      ((value & SCG_CSR_SCS_MASK)     >> SCG_CSR_SCS_SHIFT);
    config->divCore = (scg_system_clock_div_t)
                      ((value & SCG_CSR_DIVCORE_MASK) >> SCG_CSR_DIVCORE_SHIFT);
    config->divPlat = (scg_system_clock_div_t)
                      ((value & SCG_CSR_DIVPLAT_MASK) >> SCG_CSR_DIVPLAT_SHIFT);
    config->divBus  = (scg_system_clock_div_t)
                      ((value & SCG_CSR_DIVBUS_MASK)  >> SCG_CSR_DIVBUS_SHIFT);
    config->divSlow = (scg_system_clock_div_t)
                      ((value & SCG_CSR_DIVSLOW_MASK) >> SCG_CSR_DIVSLOW_SHIFT);
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_GetSysOscDefaultConfig
 * Description   : This function gets the default system OSC configuration.
 *
 *END*************************************************************************/
void SCG_HAL_GetSysOscDefaultConfig(scg_sosc_config_t *config)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(config);
#endif

    config->enableInStop      = false;
    config->enableInLowPower  = false;
    config->enableErClk       = false;

    config->monitorMode       = SCG_SOSC_MONITOR_DISABLE;
    config->locked            = false;

    config->div1              = SCG_ASYNC_CLOCK_DISABLE;
    config->div2              = SCG_ASYNC_CLOCK_DISABLE;

    config->extRef            = SCG_SOSC_REF_EXT;
    config->gain              = SCG_SOSC_GAIN_LOW;
    config->range             = SCG_SOSC_RANGE_LOW;
    config->initialize        = true;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_InitSysOsc
 * Description   : This function enables the SCG system OSC clock according
 * to the configuration.
 *
 *END*************************************************************************/
scg_status_t SCG_HAL_InitSysOsc(SCG_Type * base,
                                  scg_sosc_config_t const *config)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(config);
    /* Control register can be written. */
    DEV_ASSERT(!BITBAND_ACCESS32(&(base->SOSCCSR), SCG_SOSCCSR_LK_SHIFT));
#endif

    /* If clock is used by system, return error. */
    if (BITBAND_ACCESS32(&(base->SOSCCSR), SCG_SOSCCSR_SOSCSEL_SHIFT))
    {
        return STATUS_SCG_BUSY;
    }

    /* Disable monitor, disable clock and clear error. */
    base->SOSCCSR = SCG_SOSCCSR_SOSCERR_MASK;

    /* Now start to set up OSC clock. */
    /* Step 1. Setup dividers. */
    base->SOSCDIV = SCG_SOSCDIV_SOSCDIV1(config->div1) |
                    SCG_SOSCDIV_SOSCDIV2(config->div2);
    /* Step 2. Set OSC configuration. */
    base->SOSCCFG = SCG_SOSCCFG_RANGE(config->range)        |
                    SCG_SOSCCFG_HGO(config->gain)           |
                    SCG_SOSCCFG_EREFS(config->extRef);
    /* Step 3. Enable clock. */
    base->SOSCCSR = SCG_SOSCCSR_SOSCEN(1U)                         |
                    SCG_SOSCCSR_SOSCSTEN(config->enableInStop)     |
                    SCG_SOSCCSR_SOSCLPEN(config->enableInLowPower) |
                    SCG_SOSCCSR_SOSCERCLKEN(config->enableErClk);
    /* Step 4. Enable monitor. */
    base->SOSCCSR = (base->SOSCCSR
                      & ~(SCG_SOSCCSR_SOSCCM_MASK   |
                          SCG_SOSCCSR_SOSCCMRE_MASK |
                          SCG_SOSCCSR_SOSCERR_MASK))
                      |   (uint32_t)config->monitorMode;


    /* Step 5. Lock Control Status Register. */
    BITBAND_ACCESS32(&(base->SOSCCSR), SCG_SOSCCSR_LK_SHIFT) = config->locked;

    g_xtal0ClkFreq = config->freq;
    return STATUS_SCG_SUCCESS;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_DeinitSysOsc
 * Description   : Disables the SCG System OSC clock.
 *
 *END*************************************************************************/
scg_status_t SCG_HAL_DeinitSysOsc(SCG_Type * base)
{
    /* If clock is used by system, return error. */
    if (BITBAND_ACCESS32(&(base->SOSCCSR), SCG_SOSCCSR_SOSCSEL_SHIFT))
    {
        return STATUS_SCG_BUSY;
    }

    /* Clear LK bit field */
    BITBAND_ACCESS32(&(base->SOSCCSR), SCG_SOSCCSR_LK_SHIFT) = 0;

    /* Clear SOSCCSR */
    base->SOSCCSR = SCG_SOSCCSR_SOSCERR_MASK;

    g_xtal0ClkFreq = 0U;

    return STATUS_SCG_SUCCESS;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_GetSysOscFreq
 * Description   : Get SCG System OSC clock frequency (SYSOSC).
 *
 *END*************************************************************************/
uint32_t SCG_HAL_GetSysOscFreq(SCG_Type * base)
{
    if (BITBAND_ACCESS32(&(base->SOSCCSR), SCG_SOSCCSR_SOSCVLD_SHIFT)) /* System OSC clock is valid. */
    {
        return g_xtal0ClkFreq;
    }
    else
    {
        return 0U;
    }
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_GetSysOscAsyncFreq
 * Description   : Get SCG asynchronous clock frequency from system OSC.
 *
 *END*************************************************************************/
uint32_t SCG_HAL_GetSysOscAsyncFreq(SCG_Type * base,
                                      scg_async_clock_type_t type)
{
    uint32_t oscFreq = SCG_HAL_GetSysOscFreq(base);
    uint32_t divider = 0U;
    uint32_t regValue;

    /* Get divider. */
    if (oscFreq)
    {
        switch (type)
        {
            case SCG_ASYNC_CLOCK_DIV2:
                regValue = base->SOSCDIV;
                regValue = (regValue & SCG_SOSCDIV_SOSCDIV2_MASK) >> SCG_SOSCDIV_SOSCDIV2_SHIFT;
                divider = regValue;
                break;
            case SCG_ASYNC_CLOCK_DIV1:
                regValue = base->SOSCDIV;
                regValue = (regValue & SCG_SOSCDIV_SOSCDIV1_MASK) >> SCG_SOSCDIV_SOSCDIV1_SHIFT;
                divider = regValue;
                break;
            default:
                /* Invalid type */
                break;
        }
    }
    if (divider)
    {
        return (oscFreq >> (divider-1U));
    }
    else  /* Output disabled. */
    {
        return 0U;
    }
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_GetSircDefaultConfig
 * Description   : This function gets the default SIRC configuration.
 *
 *END*************************************************************************/
void SCG_HAL_GetSircDefaultConfig(scg_sirc_config_t *config)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(config);
#endif

    config->enableInStop      = false;
    config->enableInLowPower  = true;
    config->locked            = false;
    config->div1              = SCG_ASYNC_CLOCK_DISABLE;
    config->div2              = SCG_ASYNC_CLOCK_DISABLE;
    config->range             = SCG_SIRC_RANGE_HIGH;
    config->initialize        = true;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_InitSirc
 * Description   : This function enables the SCG SIRC clock according
 * to the configuration.
 *
 *END*************************************************************************/
scg_status_t SCG_HAL_InitSirc(SCG_Type * base,
                                const scg_sirc_config_t *config)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(config);
    /* Control register can be written. */
    DEV_ASSERT(!BITBAND_ACCESS32(&(base->SIRCCSR), SCG_SIRCCSR_LK_SHIFT));
#endif

    /* If clock is used by system, return error. */
    if (BITBAND_ACCESS32(&(base->SIRCCSR), SCG_SIRCCSR_SIRCSEL_SHIFT))
    {
        return STATUS_SCG_BUSY;
    }

    /* Disable clock. */
    BITBAND_ACCESS32(&(base->SIRCCSR), SCG_SIRCCSR_SIRCEN_SHIFT) = 0;

    /* Now start to set up SIRC clock. */
    /* Step 1. Setup dividers. */
    base->SIRCDIV = SCG_SIRCDIV_SIRCDIV1(config->div1) |
                    SCG_SIRCDIV_SIRCDIV2(config->div2);
    /* Step 2. Set SIRC configuration. */
    base->SIRCCFG = SCG_SIRCCFG_RANGE(config->range);
    /* Step 3. Enable clock. */
    base->SIRCCSR = SCG_SIRCCSR_SIRCEN(1U)                                 |
                    SCG_SIRCCSR_SIRCSTEN(config->enableInStop     ? 1: 0 ) |
                    SCG_SIRCCSR_SIRCLPEN(config->enableInLowPower ? 1: 0 );
    /* Step 4. Lock Control Status Register.  */
    BITBAND_ACCESS32(&(base->SIRCCSR), SCG_SIRCCSR_LK_SHIFT) = config->locked;

    return STATUS_SCG_SUCCESS;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_DeinitSirc
 * Description   : Disables the SCG slow IRC.
 *
 *END*************************************************************************/
scg_status_t SCG_HAL_DeinitSirc(SCG_Type * base)
{
    /* If clock is used by system, return error. */
    if (BITBAND_ACCESS32(&(base->SIRCCSR), SCG_SIRCCSR_SIRCSEL_SHIFT))
    {
        return STATUS_SCG_BUSY;
    }

    /* Clear LK bit field */
    BITBAND_ACCESS32(&(base->SIRCCSR), SCG_SIRCCSR_LK_SHIFT) = 0;

    /* Clear SIRCCSR */
    base->SIRCCSR = 0;

    return STATUS_SCG_SUCCESS;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_GetSircFreq
 * Description   : Get SCG SIRC clock frequency.
 *
 *END*************************************************************************/
uint32_t SCG_HAL_GetSircFreq(SCG_Type * base)
{
    if (BITBAND_ACCESS32(&(base->SIRCCSR), SCG_SIRCCSR_SIRCVLD_SHIFT)) /* SIRC is valid. */
    {
        return BITBAND_ACCESS32(&(base->SIRCCFG), SCG_SIRCCFG_RANGE_SHIFT) ?
               FSL_FEATURE_SCG_SIRC_HIGH_RANGE_FREQ : FSL_FEATURE_SCG_SIRC_LOW_RANGE_FREQ;
    }
    else
    {
        return 0U;
    }
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_GetSircAsyncFreq
 * Description   : Get SCG asynchronous clock frequency from SIRC.
 *
 *END*************************************************************************/
uint32_t SCG_HAL_GetSircAsyncFreq(SCG_Type * base,
                                      scg_async_clock_type_t type)
{
    uint32_t sircFreq = SCG_HAL_GetSircFreq(base);
    uint32_t divider = 0U;
    uint32_t regValue;

    /* Get divider. */
    if (sircFreq)
    {
        switch (type)
        {
            case SCG_ASYNC_CLOCK_DIV2:
                regValue = base->SIRCDIV;
                regValue = (regValue & SCG_SIRCDIV_SIRCDIV2_MASK) >> SCG_SIRCDIV_SIRCDIV2_SHIFT;
                divider = regValue;
                break;
            case SCG_ASYNC_CLOCK_DIV1:
                regValue = base->SIRCDIV;
                regValue = (regValue & SCG_SIRCDIV_SIRCDIV1_MASK) >> SCG_SIRCDIV_SIRCDIV1_SHIFT;
                divider = regValue;
                break;
            default:
                /* Invalid type */
                break;
        }
    }
    if (divider)
    {
        return (sircFreq >> (divider-1U));
    }
    else  /* Output disabled. */
    {
        return 0U;
    }
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_GetFircDefaultConfig
 * Description   : This function gets the default FIRC configuration.
 *
 *END*************************************************************************/
void SCG_HAL_GetFircDefaultConfig(scg_firc_config_t *config)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(config);
#endif

    config->enableInStop      = false;
    config->enableInLowPower  = false;
    config->regulator         = true;
    config->enableTrim        = false;
    config->updateTrim        = false;
    config->locked            = false;

    config->div1              = SCG_ASYNC_CLOCK_DISABLE;
    config->div2              = SCG_ASYNC_CLOCK_DISABLE;

    config->range             = SCG_FIRC_RANGE_48M;

    config->trimCoar          = 0U;
    config->trimFine          = 0U;
    config->initialize        = true;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_InitFirc
 * Description   : This function enables the SCG FIRC clock according
 * to the configuration.
 *
 *END*************************************************************************/
scg_status_t SCG_HAL_InitFirc(SCG_Type * base,
                                const scg_firc_config_t *config)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(config);
    /* Control register can be written. */
    DEV_ASSERT(!BITBAND_ACCESS32(&(base->FIRCCSR), SCG_FIRCCSR_LK_SHIFT));
#endif

    /* If clock is used by system, return error. */
    if (BITBAND_ACCESS32(&(base->FIRCCSR), SCG_FIRCCSR_FIRCSEL_SHIFT))
    {
        return STATUS_SCG_BUSY;
    }

    /* Disable clock. */
    BITBAND_ACCESS32(&(base->FIRCCSR), SCG_FIRCCSR_FIRCEN_SHIFT) = 0;
    /* Disable trimming. */
    BITBAND_ACCESS32(&(base->FIRCCSR), SCG_FIRCCSR_FIRCTREN_SHIFT) = 0;
    /* Clear error. */
    BITBAND_ACCESS32(&(base->FIRCCSR), SCG_FIRCCSR_FIRCERR_SHIFT) = 1;

    /* Now start to set up FIRC clock. */
    /* Step 1. Setup dividers. */
    base->FIRCDIV = SCG_FIRCDIV_FIRCDIV1(config->div1) |
                    SCG_FIRCDIV_FIRCDIV2(config->div2);

    /* Step 2. Set FIRC configuration. */
    base->FIRCCFG = SCG_FIRCCFG_RANGE(config->range);

    /* Step 3. Set trimming configuration. */
    if (config->enableTrim)
    {
        BITBAND_ACCESS32(&(base->FIRCCSR), SCG_FIRCCSR_FIRCTRUP_SHIFT) = config->updateTrim?1:0;
        BITBAND_ACCESS32(&(base->FIRCCSR), SCG_FIRCCSR_FIRCTREN_SHIFT) = config->enableTrim?1:0;

        if (!config->updateTrim)
        {
            (void) (config->trimCoar);
            (void) (config->trimFine);
            /* The folowing code generates a HardFault exception.
            base->FIRCSTAT = SCG_FIRCSTAT_TRIMCOAR(config->trimCoar) |
                             SCG_FIRCSTAT_TRIMFINE(config->trimFine);
            */
        }
    }
    /* Step 4. Enable clock. */
    base->FIRCCSR = (base->FIRCCSR
                   & ~((SCG_FIRCCSR_FIRCEN_MASK)   |
                       (SCG_FIRCCSR_FIRCSTEN_MASK) |
                       (SCG_FIRCCSR_FIRCLPEN_MASK) |
                       (SCG_FIRCCSR_FIRCREGOFF_MASK)))
                   | (SCG_FIRCCSR_FIRCEN(1U)                                 |
                      SCG_FIRCCSR_FIRCSTEN(config->enableInStop     ? 1: 0 ) |
                      SCG_FIRCCSR_FIRCLPEN(config->enableInLowPower ? 1: 0 ) |
                      SCG_FIRCCSR_FIRCREGOFF(config->regulator ? 0: 1 ));



    /* Step 5. Lock Control Status Register  */
    BITBAND_ACCESS32(&(base->FIRCCSR), SCG_FIRCCSR_LK_SHIFT) = config->locked;
    
    /* Check if an error was detected with the trimming */
    if ((config->enableTrim) && BITBAND_ACCESS32(&(base->FIRCCSR), SCG_FIRCCSR_FIRCERR_SHIFT))
    {
        return STATUS_SCG_FAILED;
    }

    return STATUS_SCG_SUCCESS;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_DeinitFirc
 * Description   : Disables the SCG fast IRC.
 *
 *END*************************************************************************/
scg_status_t SCG_HAL_DeinitFirc(SCG_Type * base)
{
    /* If clock is used by system, return error. */
    if (BITBAND_ACCESS32(&(base->FIRCCSR), SCG_FIRCCSR_FIRCSEL_SHIFT))
    {
        return STATUS_SCG_BUSY;
    }

    /* Clear LK bit field */
    BITBAND_ACCESS32(&(base->FIRCCSR), SCG_FIRCCSR_LK_SHIFT) = 0;

    /* Clear FIRCCSR */
    base->FIRCCSR = SCG_FIRCCSR_FIRCERR_MASK;

    return STATUS_SCG_SUCCESS;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_GetFircFreq
 * Description   : Get SCG FIRC clock frequency.
 *
 *END*************************************************************************/
uint32_t SCG_HAL_GetFircFreq(SCG_Type * base)
{
    static const uint32_t fircFreq[] = {
        FSL_FEATURE_SCG_FIRC_FREQ0,
        FSL_FEATURE_SCG_FIRC_FREQ1,
        FSL_FEATURE_SCG_FIRC_FREQ2,
        FSL_FEATURE_SCG_FIRC_FREQ3,
    };

    if (BITBAND_ACCESS32(&(base->FIRCCSR), SCG_FIRCCSR_FIRCVLD_SHIFT)) /* FIRC is valid. */
    {
        uint32_t regValue = base->FIRCCFG;
        regValue = (regValue & SCG_FIRCCFG_RANGE_MASK) >> SCG_FIRCCFG_RANGE_SHIFT;
        return fircFreq[regValue];
    }
    else
    {
        return 0U;
    }
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_GetFircAsyncFreq
 * Description   : Get SCG asynchronous clock frequency from FIRC.
 *
 *END*************************************************************************/
uint32_t SCG_HAL_GetFircAsyncFreq(SCG_Type * base,
                                      scg_async_clock_type_t type)
{
    uint32_t fircFreq = SCG_HAL_GetFircFreq(base);
    uint32_t divider = 0U;
    uint32_t regValue;

    /* Get divider. */
    if (fircFreq)
    {
        switch (type)
        {
            case SCG_ASYNC_CLOCK_DIV2:
                regValue = base->FIRCDIV;
                regValue = (regValue & SCG_FIRCDIV_FIRCDIV2_MASK) >> SCG_FIRCDIV_FIRCDIV2_SHIFT;
                divider = regValue;
                break;
            case SCG_ASYNC_CLOCK_DIV1:
                regValue = base->FIRCDIV;
                regValue = (regValue & SCG_FIRCDIV_FIRCDIV1_MASK) >> SCG_FIRCDIV_FIRCDIV1_SHIFT;
                divider = regValue;
                break;
            default:
                /* Invalid type */
                break;
        }
    }
    if (divider)
    {
        return (fircFreq >> (divider-1U));
    }
    else  /* Output disabled. */
    {
        return 0U;
    }
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_GetSysPllDefaultConfig
 * Description   : This function gets the default system PLL configuration.
 *
 *END*************************************************************************/
void SCG_HAL_GetSysPllDefaultConfig(scg_spll_config_t *config)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(config);
#endif

    config->enableInStop   = false;
    config->monitorMode    = SCG_SPLL_MONITOR_DISABLE;
    config->locked         = false;

    config->div1           = SCG_ASYNC_CLOCK_DISABLE;
    config->div2           = SCG_ASYNC_CLOCK_DISABLE;

    config->prediv         = 0;
    config->mult           = 0;
    config->initialize     = true;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_InitSysPll
 * Description   : This function enables the SCG system PLL clock according
 * to the configuration.
 *
 *END*************************************************************************/
scg_status_t SCG_HAL_InitSysPll(SCG_Type * base,
                                scg_spll_config_t const *config)
{
    uint32_t srcFreq;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(config);
    /* Control register can be written. */
    DEV_ASSERT(!BITBAND_ACCESS32(&(base->SPLLCSR), SCG_SPLLCSR_LK_SHIFT));
#endif

    /* If clock is used by system, return error. */
    if (BITBAND_ACCESS32(&(base->SPLLCSR), SCG_SPLLCSR_SPLLSEL_SHIFT))
    {
        return STATUS_SCG_BUSY;
    }

    /* Get clock source frequency. */
    srcFreq = SCG_HAL_GetSysOscFreq(base);

    if (!srcFreq)
    {
        return STATUS_SCG_INVALID_SRC;
    }

    /* Pre-divider checking. */
    srcFreq /= (config->prediv + SCG_SPLL_PREDIV_BASE);

    if ((srcFreq < SCG_SPLL_REF_MIN) || (srcFreq > SCG_SPLL_REF_MAX))
    {
        return STATUS_SCG_INVALID_PARAMETER;
    }

    /* Disable monitor, disable clock and clear error. */
    base->SPLLCSR = SCG_SPLLCSR_SPLLERR_MASK;

    /* Now start to set up PLL clock. */
    /* Step 1. Setup dividers. */
    base->SPLLDIV = SCG_SPLLDIV_SPLLDIV1(config->div1) |
                    SCG_SPLLDIV_SPLLDIV2(config->div2);
    /* Step 2. Set PLL configuration. */
    base->SPLLCFG = SCG_SPLLCFG_PREDIV(config->prediv)  |
                    SCG_SPLLCFG_MULT(config->mult);
    /* Step 3. Enable clock. */
    base->SPLLCSR = SCG_SPLLCSR_SPLLEN(1U)                     |
                    SCG_SPLLCSR_SPLLSTEN(config->enableInStop);
    /* Step 4. Enable monitor. */
    base->SPLLCSR = (base->SPLLCSR
                      & ~(SCG_SPLLCSR_SPLLCM_MASK   |
                          SCG_SPLLCSR_SPLLCMRE_MASK |
                          SCG_SPLLCSR_SPLLERR_MASK))
                      |   (uint32_t)config->monitorMode;

    /* Step 5. Lock Control Status Register */
    BITBAND_ACCESS32(&(base->SPLLCSR), SCG_SPLLCSR_LK_SHIFT) = config->locked;

    return STATUS_SCG_SUCCESS;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_DeinitSysPll
 * Description   : Disables the SCG system PLL.
 *
 *END*************************************************************************/
scg_status_t SCG_HAL_DeinitSysPll(SCG_Type * base)
{
    /* If clock is used by system, return error. */
    if (BITBAND_ACCESS32(&(base->SPLLCSR), SCG_SPLLCSR_SPLLSEL_SHIFT))
    {
        return STATUS_SCG_BUSY;
    }

    /* Clear LK bit field */
    BITBAND_ACCESS32(&(base->SPLLCSR), SCG_SPLLCSR_LK_SHIFT) = 0;

    /* Clear SPLLCSR */
    base->SPLLCSR = SCG_SPLLCSR_SPLLERR_MASK;

    return STATUS_SCG_SUCCESS;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_GetSysPllFreq
 * Description   : Get SCG system PLL clock frequency.
 *
 *END*************************************************************************/
uint32_t SCG_HAL_GetSysPllFreq(SCG_Type * base)
{
    uint32_t freq;
    uint32_t regValue;

    if (BITBAND_ACCESS32(&(base->SPLLCSR), SCG_SPLLCSR_SPLLVLD_SHIFT)) /* System PLL is valid. */
    {
        /* Get System OSC. frequency. */
        freq = SCG_HAL_GetSysOscFreq(base);

        if (freq) /* If source is valid. */
        {
            regValue = base->SPLLCFG;
            regValue = (regValue & SCG_SPLLCFG_PREDIV_MASK) >> SCG_SPLLCFG_PREDIV_SHIFT;
            freq /= (regValue + SCG_SPLL_PREDIV_BASE);  /* Pre-divider. */

            regValue = base->SPLLCFG;
            regValue = (regValue & SCG_SPLLCFG_MULT_MASK) >> SCG_SPLLCFG_MULT_SHIFT;
            freq *= (regValue + SCG_SPLL_MULT_BASE);      /* Multiplier. */
            
            freq = freq >> 1U;  /* Divide VCO by 2. */
        }

        return freq;
    }
    else
    {
        return 0U;
    }
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_GetSysPllAsyncFreq
 * Description   : Get SCG asynchronous clock frequency from system PLL.
 *
 *END*************************************************************************/
uint32_t SCG_HAL_GetSysPllAsyncFreq(SCG_Type * base,
                                  scg_async_clock_type_t type)
{
    uint32_t pllFreq = SCG_HAL_GetSysPllFreq(base);
    uint32_t divider = 0U;
    uint32_t regValue;

    /* Get divider. */
    if (pllFreq)
    {
        switch (type)
        {
            case SCG_ASYNC_CLOCK_DIV2:
                regValue = base->SPLLDIV;
                regValue = (regValue & SCG_SPLLDIV_SPLLDIV2_MASK) >> SCG_SPLLDIV_SPLLDIV2_SHIFT;
                divider = regValue;
                break;
            case SCG_ASYNC_CLOCK_DIV1:
                regValue = base->SPLLDIV;
                regValue = (regValue & SCG_SPLLDIV_SPLLDIV1_MASK) >> SCG_SPLLDIV_SPLLDIV1_SHIFT;
                divider = regValue;
                break;
            default:
                /* Invalid type */
                break;
        }
    }
    if (divider)
    {
        return (pllFreq >> (divider-1U));
    }
    else  /* Output disabled. */
    {
        return 0U;
    }
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_SetRtcClkInFreq
 * Description   : Configures SCG RTC CLKIN clock frequency.
 *
 *END*************************************************************************/
void SCG_HAL_SetRtcClkInFreq(SCG_Type * base, uint32_t frequency)
{
    (void) (base);
    g_RtcClkInFreq = frequency;;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SCG_HAL_GetRtcClkInFreq
 * Description   : Get SCG RTC CLKIN clock frequency.
 *
 *END*************************************************************************/
uint32_t SCG_HAL_GetRtcClkInFreq(SCG_Type * base)
{
    (void) (base);
    return g_RtcClkInFreq;
}

#endif

/******************************************************************************
 * EOF
 *****************************************************************************/

