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

#include "fsl_ftm_hal.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_HAL_Init
 * Description   : Initializes the FTM.
 *
 *END**************************************************************************/
void FTM_HAL_Init(FTM_Type* const ftmBase, ftm_clock_ps_t FtmClockPrescaler)
{
    /* Use FTM mode */
    FTM_HAL_Enable(ftmBase, true);
    FTM_HAL_SetClockPs(ftmBase, FtmClockPrescaler);
}


/*!
 * @brief Enables or disables the FTM peripheral timer channel pair output combine mode.
 *
 * @param ftmBase The FTM base address pointer
 * @param chnlPairNum The FTM peripheral channel pair number
 * @param enable  true to enable channel pair to combine, true to disable
 */
bool FTM_HAL_GetDualChnCombine(FTM_Type* const ftmBase, uint8_t chnlPairNum)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(chnlPairNum < (FSL_FEATURE_FTM_CHANNEL_COUNT >>1));
#endif
    return ((((ftmBase)->COMBINE) & FTM_COMBINE_COMBINE0_MASK << 
           (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH))) ? true : false;

}


/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_HAL_DisablePwmMode
 * Description   : Disables the PWM output mode.
 *
 *END**************************************************************************/
void FTM_HAL_DisablePwmMode(FTM_Type* const ftmBase, uint8_t channel)
{

    FTM_HAL_SetChnCountVal(ftmBase, channel, 0U);
    FTM_HAL_SetChnEdgeLevel(ftmBase, channel, 0U);
    FTM_HAL_SetChnMSnBAMode(ftmBase, channel, 0U);
    FTM_HAL_SetCpwms(ftmBase, 0U);

    /*configure polarity bit*/
    FTM_RMW_POL(ftmBase, (((uint32_t)1U) << channel), 0U);

    FTM_HAL_DisablePwmChannelOutputs(ftmBase, channel);

    ((ftmBase)->DEADTIME) = 0U;

    ((ftmBase)->SYNC) = 0U;

    /*clear combine register*/
    ((ftmBase)->COMBINE) = 0U;

    /*clear mode register*/
    ((ftmBase)->MODE) = 0U;

}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_HAL_Reset
 * Description   : Resets the FTM registers
 *
 *END**************************************************************************/
void FTM_HAL_Reset(FTM_Type* const ftmBase)
{
    uint8_t ChIndex;
    /*!<This is the reset value for MODE register. WPDIS bit is set to disble write protection*/
    ((ftmBase)->MODE) = 0x00000004U;
    ((ftmBase)->SC) = 0U;
    ((ftmBase)->CNT) = 0U;
    ((ftmBase)->MOD) = 0U;
    ((ftmBase)->CNTIN) = 0U;
    ((ftmBase)->STATUS) = 0U;
    ((ftmBase)->SYNC) = 0U;
    ((ftmBase)->OUTINIT) = 0U;
    ((ftmBase)->OUTMASK) = 0U;
    ((ftmBase)->COMBINE) = 0U;
    ((ftmBase)->DEADTIME) = 0U;
    ((ftmBase)->EXTTRIG) = 0U;
    ((ftmBase)->POL) = 0U;
    ((ftmBase)->FMS) = 0U;
    ((ftmBase)->FILTER) = 0U;
    ((ftmBase)->FLTCTRL) = 0U;
    ((ftmBase)->CONF) = 0U;
    ((ftmBase)->FLTPOL) = 0U;
    ((ftmBase)->SYNCONF) = 0U;
    ((ftmBase)->INVCTRL) = 0U;
    ((ftmBase)->SWOCTRL) = 0U;
    ((ftmBase)->PWMLOAD) = 0U;
    ((ftmBase)->HCR) = 0U;
    /* Set to reset value all CnV and CnSC registers */
    for(ChIndex = 0; ChIndex < FSL_FEATURE_FTM_CHANNEL_COUNT; ChIndex++)
    {
        ((ftmBase)->CONTROLS[ChIndex].CnSC) = 0U;
        ((ftmBase)->CONTROLS[ChIndex].CnV) = 0U;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_HAL_SetHardwareSyncTriggerSrc
 * Description   : Sets the FTM peripheral timer hardware trigger.
 *
 *END**************************************************************************/
void FTM_HAL_SetHardwareSyncTriggerSrc(FTM_Type* const ftmBase, uint32_t trigger_num, bool enable)
{
    switch(trigger_num)
    {
    case 0:
        BITBAND_ACCESS32(&((ftmBase)->SYNC), FTM_SYNC_TRIG0_SHIFT) = (enable ? 1U : 0U);
        break;
    case 1:
        BITBAND_ACCESS32(&((ftmBase)->SYNC), FTM_SYNC_TRIG1_SHIFT) = (enable ? 1U : 0U);
        break;
    case 2:
        BITBAND_ACCESS32(&((ftmBase)->SYNC), FTM_SYNC_TRIG2_SHIFT) = (enable ? 1U : 0U);
        break;
    default:
#ifdef DEV_ERROR_DETECT
DEV_ASSERT(0);
#endif
break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_HAL_SetChnTriggerCmd
 * Description   : Enables or disables the generation of the FTM peripheral timer channel trigger.
 * Enables or disables the generation of the FTM peripheral timer channel trigger when the
 * FTM counter is equal to its initial value. Channels 6 and 7 cannot be used as triggers.
 *
 *END**************************************************************************/
void FTM_HAL_SetChnTriggerCmd(FTM_Type* const ftmBase, uint8_t channel, bool enable)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < CHAN6_IDX);
#endif
    uint8_t bit = enable ? 1U : 0U;
    uint32_t value = (channel > 1U) ? 
                     (uint8_t)(bit << (channel - 2U)) : (uint8_t)(bit << (channel + 4U));

    if(true == enable)
    {
        ((ftmBase)->EXTTRIG) |=   value;
    }
    else
    {
        ((ftmBase)->EXTTRIG) &=   ~value;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_HAL_SetChnInputCaptureFilter
 * Description   : Sets the FTM peripheral timer channel input capture filter value.
 *
 *END**************************************************************************/
void FTM_HAL_SetChnInputCaptureFilter(FTM_Type* const ftmBase, uint8_t channel, uint8_t value)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < CHAN4_IDX);
#endif
    switch(channel)
    {
    case CHAN0_IDX:
        FTM_RMW_FILTER(ftmBase, FTM_FILTER_CH0FVAL_MASK, FTM_FILTER_CH0FVAL(value));
        break;
    case CHAN1_IDX:
        FTM_RMW_FILTER(ftmBase, FTM_FILTER_CH1FVAL_MASK, FTM_FILTER_CH1FVAL(value));
        break;
    case CHAN2_IDX:
        FTM_RMW_FILTER(ftmBase, FTM_FILTER_CH2FVAL_MASK, FTM_FILTER_CH2FVAL(value));
        break;
    case CHAN3_IDX:
        FTM_RMW_FILTER(ftmBase, FTM_FILTER_CH3FVAL_MASK, FTM_FILTER_CH3FVAL(value));
        break;
    default:
#ifdef DEV_ERROR_DETECT
DEV_ASSERT(0);
#endif
break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_HAL_GetChnPairIndex
 * Description   : Combines the channel control.
 * Returns an index for each channel pair.
 *
 *END**************************************************************************/
uint32_t FTM_HAL_GetChnPairIndex(uint8_t channel)
{
    uint32_t ReturnValue = 0U;

    if((channel == CHAN0_IDX) || (channel == CHAN1_IDX))
    {
        ReturnValue = 0U;
    }
    else if((channel == CHAN2_IDX) || (channel == CHAN3_IDX))
    {
        ReturnValue = 1U;
    }
    else if((channel == CHAN4_IDX) || (channel == CHAN5_IDX))
    {
        ReturnValue = 2U;
    }
    else
    {
        ReturnValue = 3U;
    }
    return  ReturnValue;
}


/*******************************************************************************
 * EOF
 ******************************************************************************/

