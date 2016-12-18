/*
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
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

#include "fsl_device_registers.h"
#include "fsl_sim_hal.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * APIs
 ******************************************************************************/
 
/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_SetFtmExternalClkPinMode
 * Description   : Set FlexTimer x external clock pin select setting
 * This function will select the source of FTMx external clock pin select
 *
 *END**************************************************************************/
void SIM_HAL_SetFtmExternalClkPinMode(SIM_Type * base,
                                      uint32_t instance,
                                      sim_ftm_clk_sel_t select)
{
    uint32_t regValue;
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
#endif

    switch (instance)
    {
    case 0:
        regValue = base->FTMOPT0;
        regValue &= ~(SIM_FTMOPT0_FTM0CLKSEL_MASK);
        regValue |= SIM_FTMOPT0_FTM0CLKSEL(select);
        base->FTMOPT0 = regValue;
        break;
    case 1:
        regValue = base->FTMOPT0;
        regValue &= ~(SIM_FTMOPT0_FTM1CLKSEL_MASK);
        regValue |= SIM_FTMOPT0_FTM1CLKSEL(select);
        base->FTMOPT0 = regValue;
        break;
    case 2:
        regValue = base->FTMOPT0;
        regValue &= ~(SIM_FTMOPT0_FTM2CLKSEL_MASK);
        regValue |= SIM_FTMOPT0_FTM2CLKSEL(select);
        base->FTMOPT0 = regValue;
        break;
    case 3:
        regValue = base->FTMOPT0;
        regValue &= ~(SIM_FTMOPT0_FTM3CLKSEL_MASK);
        regValue |= SIM_FTMOPT0_FTM3CLKSEL(select);
        base->FTMOPT0 = regValue;
        break;
    default:
        break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_GetFtmExternalClkPinMode
 * Description   : Get FlexTimer x external clock pin select setting
 * This function will get FlexTimer x external clock pin select setting.
 *
 *END**************************************************************************/
sim_ftm_clk_sel_t SIM_HAL_GetFtmExternalClkPinMode(SIM_Type * base,
                                                   uint32_t instance)
{
    sim_ftm_clk_sel_t retValue = (sim_ftm_clk_sel_t)0;
    uint32_t regValue;
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
#endif

    switch (instance)
    {
    case 0:
        regValue = base->FTMOPT0;
        regValue = (regValue & SIM_FTMOPT0_FTM0CLKSEL_MASK) >> SIM_FTMOPT0_FTM0CLKSEL_SHIFT;
        retValue = (sim_ftm_clk_sel_t) regValue;
        break;
    case 1:
        regValue = base->FTMOPT0;
        regValue = (regValue & SIM_FTMOPT0_FTM1CLKSEL_MASK) >> SIM_FTMOPT0_FTM1CLKSEL_SHIFT;
        retValue = (sim_ftm_clk_sel_t) regValue;
        break;
    case 2:
        regValue = base->FTMOPT0;
        regValue = (regValue & SIM_FTMOPT0_FTM2CLKSEL_MASK) >> SIM_FTMOPT0_FTM2CLKSEL_SHIFT;
        retValue = (sim_ftm_clk_sel_t) regValue;
        break;
    case 3:
        regValue = base->FTMOPT0;
        regValue = (regValue & SIM_FTMOPT0_FTM3CLKSEL_MASK) >> SIM_FTMOPT0_FTM3CLKSEL_SHIFT;
        retValue = (sim_ftm_clk_sel_t) regValue;
        break;
    default:
        break;
    }

    return retValue;
}
 
/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_SetFtmFaultSelMode
 * Description   : Set FlexTimer x faults select settings
 * This function will set the FlexTimer x faults select settings.
 * See SIM_HAL_FTM_FLT0_MASK
 *
 *END**************************************************************************/
void SIM_HAL_SetFtmFaultSelMode(SIM_Type * base,
                                uint32_t instance,
                                uint8_t select)
{
    uint32_t regValue;
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    DEV_ASSERT(select < (1<<SIM_FTMOPT0_FTM0FLTxSEL_WIDTH));
#endif
    
    /* TODO: verify that all SIM_FTMOPT0_FTM1FLTxSEL_WIDTH are equal */

    switch (instance)
    {
    case 0:
        regValue = base->FTMOPT0;
        regValue &= ~(SIM_FTMOPT0_FTM0FLTxSEL_MASK);
        regValue |= SIM_FTMOPT0_FTM0FLTxSEL(select);
        base->FTMOPT0 = regValue;
        break;
    case 1:
        regValue = base->FTMOPT0;
        regValue &= ~(SIM_FTMOPT0_FTM1FLTxSEL_MASK);
        regValue |= SIM_FTMOPT0_FTM1FLTxSEL(select);
        base->FTMOPT0 = regValue;
        break;
    case 2:
        regValue = base->FTMOPT0;
        regValue &= ~(SIM_FTMOPT0_FTM2FLTxSEL_MASK);
        regValue |= SIM_FTMOPT0_FTM2FLTxSEL(select);
        base->FTMOPT0 = regValue;
        break;
    case 3:
        regValue = base->FTMOPT0;
        regValue &= ~(SIM_FTMOPT0_FTM3FLTxSEL_MASK);
        regValue |= SIM_FTMOPT0_FTM3FLTxSEL(select);
        base->FTMOPT0 = regValue;
        break;
    default:
        break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_GetFtmFaultSelMode
 * Description   : Get FlexTimer x faults select settings
 * This function will get FlexTimer x faults select settings.
 * See SIM_HAL_FTM_FLT0_MASK
 *
 *END**************************************************************************/
uint8_t SIM_HAL_GetFtmFaultSelMode(SIM_Type * base,
                                   uint32_t instance)
{
    uint8_t retValue = 0U;
    uint32_t regValue;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
#endif

    switch (instance)
    {
    case 0:
        regValue = base->FTMOPT0;
        regValue = (regValue & SIM_FTMOPT0_FTM0FLTxSEL_MASK) >> SIM_FTMOPT0_FTM0FLTxSEL_SHIFT;
        retValue = regValue;
        break;
    case 1:
        regValue = base->FTMOPT0;
        regValue = (regValue & SIM_FTMOPT0_FTM1FLTxSEL_MASK) >> SIM_FTMOPT0_FTM1FLTxSEL_SHIFT;
        retValue = regValue;
        break;
    case 2:
        regValue = base->FTMOPT0;
        regValue = (regValue & SIM_FTMOPT0_FTM2FLTxSEL_MASK) >> SIM_FTMOPT0_FTM2FLTxSEL_SHIFT;
        retValue = regValue;
        break;
    case 3:
        regValue = base->FTMOPT0;
        regValue = (regValue & SIM_FTMOPT0_FTM3FLTxSEL_MASK) >> SIM_FTMOPT0_FTM3FLTxSEL_SHIFT;
        retValue = regValue;
        break;
    default:
        break;
    }

    return retValue;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SIM_HAL_GetLpoFreq
 * Description   : Get SIM LPO 1KHz clock frequency (LPO_CLOCK).
 *
 *END*************************************************************************/
uint32_t SIM_HAL_GetLpoFreq(SIM_Type * base)
{
    uint32_t freq = 0U;
    uint32_t regValue = base->LPOCLKS;

    regValue = (regValue & SIM_LPOCLKS_LPOCLKSEL_MASK) >> SIM_LPOCLKS_LPOCLKSEL_SHIFT;
    sim_lpoclk_sel_src_t lpo_clk_sel = (sim_lpoclk_sel_src_t) regValue;

    switch (lpo_clk_sel)
    {
        case SIM_LPO_CLK_SEL_LPO_128K:
            freq = LPO_128K_FREQUENCY;
            break;
        case SIM_LPO_CLK_SEL_LPO_32K:
            freq = SIM_HAL_GetLpo32KFreq(SIM);
            break;
        case SIM_LPO_CLK_SEL_LPO_1K:
            freq = SIM_HAL_GetLpo1KFreq(SIM);
            break;
        default:
            /* Invalid LPOCLKSEL selection.*/
            break;
    }

    return freq;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SIM_HAL_GetLpo32KFreq
 * Description   : Get SIM LPO 1KHz clock frequency (LPO_32K_CLOCK).
 *
 *END*************************************************************************/
uint32_t SIM_HAL_GetLpo32KFreq(SIM_Type * base)
{
    if (BITBAND_ACCESS32(&(base->LPOCLKS), SIM_LPOCLKS_LPO32KCLKEN_SHIFT)) {
        return LPO_32K_FREQUENCY;
    } else {
        return 0U;
    }
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SIM_HAL_GetLpo1KFreq
 * Description   : Get SIM LPO 1KHz clock frequency (LPO_1K_CLOCK).
 *
 *END*************************************************************************/
uint32_t SIM_HAL_GetLpo1KFreq(SIM_Type * base)
{
    if (BITBAND_ACCESS32(&(base->LPOCLKS), SIM_LPOCLKS_LPO1KCLKEN_SHIFT)) {
        return LPO_1K_FREQUENCY;
    } else {
        return 0U;
    }
}

 /*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_SetAdcPreTriggerMode
 * Description   : Set ADCx pre-trigger select setting
 * This function will select the ADCx pre-trigger source 
 *
 *END**************************************************************************/
void SIM_HAL_SetAdcPreTriggerMode(SIM_Type * base,
                                  uint32_t instance,
                                  sim_adc_pretrg_sel_t select)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
#endif

    switch (instance)
    {
    case 0U:
        REG_RMW32(&(base->ADCOPT), SIM_ADCOPT_ADC0PRETRGSEL_MASK, SIM_ADCOPT_ADC0PRETRGSEL(select));
        break;
    case 1U:
        REG_RMW32(&(base->ADCOPT), SIM_ADCOPT_ADC1PRETRGSEL_MASK, SIM_ADCOPT_ADC1PRETRGSEL(select));
        break;
    default:
        break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_GetAdcPreTriggerMode
 * Description   : Get ADCx pre-trigger select setting
 * This function will get ADCx pre-trigger select setting.
 *
 *END**************************************************************************/
sim_adc_pretrg_sel_t SIM_HAL_GetAdcPreTriggerMode(SIM_Type * base,
                                                  uint32_t instance)
{
    sim_adc_pretrg_sel_t retValue = (sim_adc_pretrg_sel_t)0U;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
#endif

    switch (instance)
    {
    case 0U:
        retValue = (sim_adc_pretrg_sel_t)((REG_READ32(&(base->ADCOPT)) & SIM_ADCOPT_ADC0PRETRGSEL_MASK) >> SIM_ADCOPT_ADC0PRETRGSEL_SHIFT);
        break;
    case 1U:
        retValue = (sim_adc_pretrg_sel_t)((REG_READ32(&(base->ADCOPT)) & SIM_ADCOPT_ADC1PRETRGSEL_MASK) >> SIM_ADCOPT_ADC1PRETRGSEL_SHIFT);
        break;
    default:
        break;
    }

    return retValue;
}
 
 /*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_SetAdcSwPreTriggerMode
 * Description   : Set ADCx software pre-trigger select setting
 * This function will select the ADCx software pre-trigger source 
 *
 *END**************************************************************************/
void SIM_HAL_SetAdcSwPreTriggerMode(SIM_Type * base,
                                  uint32_t instance,
                                  sim_adc_sw_pretrg_sel_t select)
{
    uint32_t regValue;
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
#endif

    switch (instance)
    {
    case 0:
        regValue = base->ADCOPT;
        regValue &= ~(SIM_ADCOPT_ADC0SWPRETRG_MASK);
        regValue |= SIM_ADCOPT_ADC0SWPRETRG(select);
        base->ADCOPT = regValue;
        break;
    case 1:
        regValue = base->ADCOPT;
        regValue &= ~(SIM_ADCOPT_ADC1SWPRETRG_MASK);
        regValue |= SIM_ADCOPT_ADC1SWPRETRG(select);
        base->ADCOPT = regValue;
        break;
    default:
        break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_GetAdcSwPreTriggerMode
 * Description   : Get ADCx software pre-trigger select setting
 * This function will get ADCx software pre-trigger select setting.
 *
 *END**************************************************************************/
sim_adc_sw_pretrg_sel_t SIM_HAL_GetAdcSwPreTriggerMode(SIM_Type * base,
                                                  uint32_t instance)
{
    sim_adc_sw_pretrg_sel_t retValue = (sim_adc_sw_pretrg_sel_t)0;
    uint32_t regValue;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
#endif

    switch (instance)
    {
    case 0:
        regValue = base->ADCOPT;
        regValue = (regValue & SIM_ADCOPT_ADC0SWPRETRG_MASK) >> SIM_ADCOPT_ADC0SWPRETRG_SHIFT;
        retValue = (sim_adc_sw_pretrg_sel_t)regValue;
        break;
    case 1:
        regValue = base->ADCOPT;
        regValue = (regValue & SIM_ADCOPT_ADC1SWPRETRG_MASK) >> SIM_ADCOPT_ADC1SWPRETRG_SHIFT;
        retValue = (sim_adc_sw_pretrg_sel_t)regValue;
        break;
    default:
        break;
    }

    return retValue;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_SetAdcTriggerMode
 * Description   : Set ADCx trigger select setting
 * This function will select the ADCx trigger source
 *
 *END**************************************************************************/
void SIM_HAL_SetAdcTriggerMode(SIM_Type * base,
                               uint32_t instance,
                               sim_adc_trg_sel_t select)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
#endif

    switch (instance)
    {
    case 0:
        BITBAND_ACCESS32(&(base->ADCOPT), SIM_ADCOPT_ADC0TRGSEL_SHIFT) = select;
        break;
    case 1:
        BITBAND_ACCESS32(&(base->ADCOPT), SIM_ADCOPT_ADC1TRGSEL_SHIFT) = select;
        break;
    default:
        break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_GetAdcTriggerMode
 * Description   : Get ADCx trigger select setting
 * This function will get ADCx trigger select setting.
 *
 *END**************************************************************************/
sim_adc_trg_sel_t SIM_HAL_GetAdcTriggerMode(SIM_Type * base, uint32_t instance)
{
    sim_adc_trg_sel_t retValue = (sim_adc_trg_sel_t)0;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
#endif

    switch (instance)
    {
    case 0:
        retValue = (sim_adc_trg_sel_t)BITBAND_ACCESS32(&(base->ADCOPT), SIM_ADCOPT_ADC0TRGSEL_SHIFT);
        break;
    case 1:
        retValue = (sim_adc_trg_sel_t)BITBAND_ACCESS32(&(base->ADCOPT), SIM_ADCOPT_ADC1TRGSEL_SHIFT);
        break;
    default:
        break;
    }

    return retValue;
}

/* Macro for FTMxOCHySRC. */
#define FTM_CH_OUT_SRC_MASK(instance, channel) \
    (1U << ((((instance)>>1U)*8U) + channel + 16U))

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_SetFtmChOutSrcMode
 * Description   : FlexTimer x channel y output source select setting.
 * This function will select FlexTimer x channel y output source
 *
 *END**************************************************************************/
void SIM_HAL_SetFtmChOutSrcMode(SIM_Type * base,
                                uint32_t instance,
                                uint8_t channel,
                                sim_ftm_ch_out_src_t select)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT((0U==instance) || (3U==instance));
    DEV_ASSERT(SIM_FTMOPT1_FTM3_OUTSEL_WIDTH > channel);
#endif

    if (SIM_FTM_CH_OUT_SRC_0 == select)
    {
        base->FTMOPT1 = base->FTMOPT1 & ~(FTM_CH_OUT_SRC_MASK(instance, channel));
    }
    else
    {
        base->FTMOPT1 = base->FTMOPT1 | (FTM_CH_OUT_SRC_MASK(instance, channel));
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_GetFtmChOutSrcMode
 * Description   : Get FlexTimer x channel y output source select setting
 * This function will get FlexTimer x channel y output source select
 * setting.
 *
 *END**************************************************************************/
sim_ftm_ch_out_src_t SIM_HAL_GetFtmChOutSrcMode(SIM_Type * base,
                                                uint32_t instance,
                                                uint8_t channel)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT((0U==instance) || (3U==instance));
    DEV_ASSERT(SIM_FTMOPT1_FTM3_OUTSEL_WIDTH > channel);
#endif
    
    if (base->FTMOPT1 & FTM_CH_OUT_SRC_MASK(instance, channel))
    {
        return SIM_FTM_CH_OUT_SRC_1;
    }
    else
    {
        return SIM_FTM_CH_OUT_SRC_0;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_SetFtmChSrcMode
 * Description   : FlexTimer x channel y input source select setting
 * This function will select FlexTimer x channel y input source
 *
 *END**************************************************************************/
void SIM_HAL_SetFtmChSrcMode(SIM_Type * base,
                             uint32_t instance,
                             uint8_t  channel,
                             sim_ftm_ch_src_t select)
{
    uint32_t regValue;
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    DEV_ASSERT(instance == 2 && select > SIM_FTMOPT1_FTM2CH1SEL_WIDTH);
#endif

    switch (instance)
    {
    case 1:
        switch (channel)
        {
        case 0:
            regValue = base->FTMOPT1;
            regValue &= ~(SIM_FTMOPT1_FTM1CH0SEL_MASK);
            regValue |= SIM_FTMOPT1_FTM1CH0SEL(select);
            base->FTMOPT1 = regValue;
            break;
        default:
            break;
        }
        break;
    case 2:
        switch (channel)
        {
        case 0:
            regValue = base->FTMOPT1;
            regValue &= ~(SIM_FTMOPT1_FTM2CH0SEL_MASK);
            regValue |= SIM_FTMOPT1_FTM2CH0SEL(select);
            base->FTMOPT1 = regValue;
            break;
        case 1:
            BITBAND_ACCESS32(&(base->FTMOPT1), SIM_FTMOPT1_FTM2CH1SEL_SHIFT) = select;
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_GetFtmChSrcMode
 * Description   : Get FlexTimer x channel y input source select setting
 * This function will get FlexTimer x channel y input source select
 * setting.
 *
 *END**************************************************************************/
sim_ftm_ch_src_t SIM_HAL_GetFtmChSrcMode(SIM_Type * base,
                                         uint32_t instance,
                                         uint8_t channel)
{
    sim_ftm_ch_src_t retValue = (sim_ftm_ch_src_t)0;
    uint32_t regValue;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
#endif

    switch (instance)
    {
    case 1:
        switch (channel)
        {
        case 0:
            regValue = base->FTMOPT1;
            regValue = (regValue & SIM_FTMOPT1_FTM1CH0SEL_MASK) >> SIM_FTMOPT1_FTM1CH0SEL_SHIFT;
            retValue = (sim_ftm_ch_src_t)regValue;
            break;
        default:
            break;
        }
        break;
    case 2:
        switch (channel)
        {
        case 0:
            regValue = base->FTMOPT1;
            regValue = (regValue & SIM_FTMOPT1_FTM2CH0SEL_MASK) >> SIM_FTMOPT1_FTM2CH0SEL_SHIFT;
            retValue = (sim_ftm_ch_src_t)regValue;
            break;
        case 1:
            retValue = (sim_ftm_ch_src_t)BITBAND_ACCESS32(&(base->FTMOPT1), SIM_FTMOPT1_FTM2CH1SEL_SHIFT);
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }

    return retValue;
}

 /*FUNCTION**********************************************************************
 *
 * Function Name : SIM_HAL_SetFtmSyncCmd
 * Description   : Set FTMxSYNCBIT
 * This function sets FlexTimer x hardware trigger 0 software synchronization.
 *
 *END**************************************************************************/
void SIM_HAL_SetFtmSyncCmd(SIM_Type * base, uint32_t instance, bool sync)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
#endif
    if (sync)
    {
        base->FTMOPT1 = base->FTMOPT1 | (1U<<instance);
    }
    else
    {
        base->FTMOPT1 = base->FTMOPT1 & ~(1U<<instance);
    }
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SIM_HAL_GetTraceClockDefaultConfig
 * Description   : This function gets the default Debug Trace Clock configuration.
 *
 *END*************************************************************************/
void SIM_HAL_GetTraceClockDefaultConfig(sim_trace_clock_config_t *config)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(config);
#endif

    config->initialize        = true;
    config->divEnable         = true;
    config->source            = CLOCK_TRACE_SRC_CORE_CLK;
    config->divider           = 0U;
    config->divFraction       = false;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : SIM_HAL_InitTraceClock
 * Description   : This function enables the SIM Debug Trace clock according
 * to the configuration.
 * @note This function ignores initialize member
 *
 *END*************************************************************************/
void SIM_HAL_InitTraceClock(SIM_Type * base,
                            const sim_trace_clock_config_t *config)
{
    uint32_t regValue;
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(config);
#endif

    /* Disable divider. */
    BITBAND_ACCESS32(&(base->CLKDIV4), SIM_CLKDIV4_TRACEDIVEN_SHIFT) = false;
    
    /* Configure source. */
    BITBAND_ACCESS32(&(base->CHIPCTL), SIM_CHIPCTL_TRACECLK_SEL_SHIFT) = config->source;
    
    /* Configure divider. */
    regValue = base->CLKDIV4;
    regValue &= ~(SIM_CLKDIV4_TRACEDIV_MASK);
    regValue |= SIM_CLKDIV4_TRACEDIV(config->divider);
    base->CLKDIV4 = regValue;
    
    /* Configure fraction. */
    BITBAND_ACCESS32(&(base->CLKDIV4), SIM_CLKDIV4_TRACEFRAC_SHIFT) = config->divFraction;
    
    /* Configure divider enable. */
    BITBAND_ACCESS32(&(base->CLKDIV4), SIM_CLKDIV4_TRACEDIVEN_SHIFT) = config->divEnable;
}


/*******************************************************************************
 * EOF
 ******************************************************************************/

