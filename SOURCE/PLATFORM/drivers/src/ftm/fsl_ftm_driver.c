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

#include "fsl_ftm_driver.h"
#include "fsl_clock_manager.h"
#include <stdint.h>
#include <stdbool.h>
#include "fsl_device_registers.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! variable holds the calculated period (in timer ticks) for each existing FTM module*/
uint16_t g_FtmModulePeriodTicks[FTM_INSTANCE_COUNT];

/*! variable holds the latest calculated signal measurement (in timer ticks) for each existing FTM module and channel*/
uint16_t g_FtmLatestSigMesurement[FTM_INSTANCE_COUNT][(FSL_FEATURE_FTM_CHANNEL_COUNT >> 1U)];

/*! variable holds the latest value read from the HW (in timer ticks) when a edge was detected*/
uint16_t g_FtmLatestTimeStamp[FTM_INSTANCE_COUNT][FSL_FEATURE_FTM_CHANNEL_COUNT];

/*! variable holds the current operation mode of the input channel*/
ftm_input_op_mode_t g_FtmInputCaptureOpMode[FTM_INSTANCE_COUNT][FSL_FEATURE_FTM_CHANNEL_COUNT];

/*! Stores FTM clock source setting */
static ftm_clock_source_t s_ftmClockSource[FTM_INSTANCE_COUNT];
static uint8_t s_ftmMaxFtmChannel[FTM_INSTANCE_COUNT];
/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_Init
 * Description   : Initializes the FTM driver.
 *
 *END**************************************************************************/
ftm_status_t FTM_DRV_Init(uint8_t instance, const ftm_user_config_t * info)
{

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
#endif
    FTM_Type *ftmBase = g_ftmBase[instance];
    uint8_t ChIndex;
    /* The reset operation doesn't care about write protection. FTM_HAL_Reset will
     * disable this protection.*/
    FTM_HAL_Reset(ftmBase);
    for(ChIndex = 0; ChIndex < FSL_FEATURE_FTM_CHANNEL_COUNT; ChIndex++)
    {
        g_FtmInputCaptureOpMode[instance][ChIndex] = FTM_NO_OPERATION;
        g_FtmLatestTimeStamp[instance][ChIndex] = 0U;
        if (ChIndex < (FSL_FEATURE_FTM_CHANNEL_COUNT >> 1))
        {
            g_FtmLatestSigMesurement[instance][ChIndex] = 0U;
        }
    }
    s_ftmMaxFtmChannel[instance] = 0U;
    g_FtmModulePeriodTicks[instance] = 0U;
    FTM_HAL_Init(ftmBase, info->ftmPrescaler);
    /* Configure sync for between registers and buffers */
    FTM_DRV_SetSync(instance, info->syncMethod);
    FTM_HAL_SetBdmMode(ftmBase, info->BDMMode);
    /* Timer overflow flag will be activated only when FTM clock
     * is different by "no clock source". This is a hardware behaviour.
     */
    /* Enable/disable timer overflow interrupt */
    FTM_HAL_SetTimerOverflowInt(ftmBase, info->isTofIsrEnabled);
    /* Configure clock source for FTM counter */
    FTM_HAL_SetClockPs(ftmBase, info->ftmPrescaler);
    s_ftmClockSource[instance] = info->ftmClockSource;
    return FTM_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_Deinit
 * Description   : Shuts down the FTM driver.
 *
 *END**************************************************************************/
ftm_status_t FTM_DRV_Deinit(uint8_t instance)
{

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
#endif
    FTM_Type *ftmBase = g_ftmBase[instance];
    FTM_HAL_Reset(ftmBase);
    return FTM_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_CounterStart
 * Description   : Starts the FTM counter. This function provides access to the
 * FTM counter settings. The counter can be run in Up counting or Up-down counting modes.
 * To run the counter in Free running mode, choose Up counting option and provide
 * 0x0 for the countStartVal and 0xFFFF for countFinalVal. Please call this
 * function only when FTM is used as timer/counter.
 *
 *END**************************************************************************/
ftm_status_t FTM_DRV_CounterStart(uint8_t instance, ftm_timer_param_t timer)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
#endif
    FTM_Type *ftmBase = g_ftmBase[instance];
    uint32_t channel = 0U;
    /* Disable counter clock */
    FTM_HAL_SetClockSource(ftmBase, FTM_CLOCK_SOURCE_NONE);
    /* Clear the overflow flag */
    FTM_HAL_ClearTimerOverflow(ftmBase);
    /* Set counter initial and max values */
    FTM_HAL_SetCounterInitVal(ftmBase, timer.initialValue);
    FTM_HAL_SetMod(ftmBase, timer.finalValue);
    /* Use FTM as counter, disable all the channels */
    for (channel = 0U; channel < FSL_FEATURE_FTM_CHANNEL_COUNT; channel++)
    {
        FTM_HAL_SetChnEdgeLevel(ftmBase, channel, 0U);
    }
    if ( timer.mode == FTM_MODE_UP_TIMER)
    {
        FTM_HAL_SetQuadDecoderCmd(ftmBase, 0U);
        FTM_HAL_SetCpwms(ftmBase, 0U);
    }
    else if (timer.mode == FTM_MODE_UP_DOWN_TIMER)
    {
        FTM_HAL_SetQuadDecoderCmd(ftmBase, 0U);
        FTM_HAL_SetCpwms(ftmBase, 1U);
    }
    else
    {
        /* Do nothing*/
    }
    /* Activate interrupts if required */
    FTM_HAL_SetClockSource(ftmBase, s_ftmClockSource[instance]);
    return FTM_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_CounterStop
 * Description   : Stops the FTM counter.
 *
 *END**************************************************************************/
ftm_status_t FTM_DRV_CounterStop(uint8_t instance)
{
    FTM_Type *ftmBase = g_ftmBase[instance];
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
#endif
    /* Stop the FTM counter */
    FTM_HAL_SetClockSource(ftmBase, FTM_CLOCK_SOURCE_NONE);
    return FTM_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_CounterRead
 * Description   : Reads back the current value of the FTM counter.
 *
 *END**************************************************************************/
uint32_t FTM_DRV_CounterRead(uint8_t instance)
{
    FTM_Type *ftmBase = g_ftmBase[instance];
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
#endif
    return FTM_HAL_GetCounter(ftmBase);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_DeinitPwm
 * Description   : Stops all PWM channels.
 *
 *END**************************************************************************/
ftm_status_t FTM_DRV_DeinitPwm(uint8_t instance)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
#endif
    FTM_Type *ftmBase = g_ftmBase[instance];
    uint8_t channel;
    /* Stop the FTM counter */
    FTM_HAL_SetClockSource(ftmBase, FTM_CLOCK_SOURCE_NONE);
    for (channel=0; channel<s_ftmMaxFtmChannel[instance]; channel++)
    {
        /* Disable PWM mode in hw*/
        FTM_HAL_DisablePwmMode(ftmBase, channel);
    }
    /* Clear fault control register */
    FTM_HAL_ClearFaultControl(ftmBase);
    /* Disable fault interrupt */
    FTM_HAL_SetFaultInt(ftmBase, false);
    /* Disable fault control */
    FTM_HAL_SetFaultControlMode(ftmBase, FTM_FAULT_CONTROL_DISABLED);
    /* Clear out the registers */
    FTM_HAL_SetMod(ftmBase, 0U);
    FTM_HAL_SetCounter(ftmBase, 0U);
    return FTM_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_InitPwm
 * Description   : Configures duty cycle and frequency and starts outputting
 * PWM on specified channels .
 *
 *END**************************************************************************/
ftm_status_t FTM_DRV_InitPwm(uint8_t instance, const ftm_pwm_param_t * param)
{
    ftm_status_t RetVal = FTM_STATUS_SUCCESS;
    uint8_t channel;
    uint8_t hwChannel;
    uint8_t fltchannel;
    uint8_t chnlPairNum = 0U;
    uint8_t ChannelId = 0U;
    ftm_independent_ch_param_t * independentChannelPtr = (void *)0;
    ftm_combined_ch_param_t * combinedChannelPtr = (void*)0;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    DEV_ASSERT(channel < FSL_FEATURE_FTM_CHANNEL_COUNT);
#endif
    FTM_Type *ftmBase = g_ftmBase[instance];
    /* Disable counter clock */
    FTM_HAL_SetClockSource(ftmBase, FTM_CLOCK_SOURCE_NONE);

    /* Clear the overflow flag */
    FTM_HAL_ClearTimerOverflow(ftmBase);
    /* Disable write protection */
    FTM_HAL_SetWriteProtectionCmd(ftmBase, false);

    /* Configure independent PWM channels */
    for (channel = 0U; channel < param->nNumIndependentPwmChannels ; channel++)
    {
        independentChannelPtr = &(*param->pwmIndependentChannelConfig)[channel];
        ChannelId = independentChannelPtr->hwChannelId;
        chnlPairNum =  FTM_HAL_GetChnPairIndex(ChannelId);
        FTM_HAL_SetDualEdgeCaptureCmd(ftmBase, chnlPairNum, false);
        /* Set ELSB bit and clear ELSA bit*/
        FTM_HAL_SetChnEdgeLevel(ftmBase, ChannelId, 2U);
        /* Set MSB and MSA bits*/
        FTM_HAL_SetChnMSnBAMode(ftmBase, ChannelId, 3U);
        /* Write FTMn_PWMLOAD register to enable synchronized loading points for the given channel */
        FTM_HAL_EnablePwmChannelOutputs(ftmBase, ChannelId);
        FTM_HAL_SetChnOutputPolarityCmd(ftmBase, ChannelId, independentChannelPtr->polarity);
        FTM_HAL_SetDualChnFaultCmd(ftmBase, chnlPairNum, ((param->faultConfig)->faultMode != FTM_FAULT_CONTROL_DISABLED)? true : false);
        /* Enable sync control for channels*/
        FTM_HAL_SetDualChnPwmSyncCmd(ftmBase, chnlPairNum, true);
        FTM_HAL_SetDualChnCombineCmd(ftmBase, chnlPairNum, false);
    }
    /* Configure combined PWM channels */
    for (channel = 0; channel < param->nNumCombinedPwmChannels ; channel++)
    {
        combinedChannelPtr = &(*param->pwmCombinedChannelConfig)[channel];
        ChannelId = combinedChannelPtr->hwChannelId;
        chnlPairNum =  FTM_HAL_GetChnPairIndex(ChannelId);
        FTM_HAL_SetDualEdgeCaptureCmd(ftmBase, chnlPairNum, false);
        /* Set ELSB bit and clear ELSA bit*/
        FTM_HAL_SetChnEdgeLevel(ftmBase, ChannelId, 2U);
        /* Set MSB and MSA bits*/
        FTM_HAL_SetChnMSnBAMode(ftmBase, ChannelId, 3U);
        /* Write FTMn_PWMLOAD register to enable synchronized loading points for the given channel */
        /* Enable channel (n) output */
        FTM_HAL_EnablePwmChannelOutputs(ftmBase, ChannelId);
        /* Enable channel (n+1) output */
        if (combinedChannelPtr->enableSecondChannelOutput)
        {
            FTM_HAL_EnablePwmChannelOutputs(ftmBase,ChannelId + 1U);
            /* When ELSB =0 and ELSA = o for channel (n+1) output is not available */
            FTM_HAL_SetChnEdgeLevel(ftmBase, ChannelId+1U, 2U);
            /* Configure complementary mode for channel (n+1) */
            FTM_HAL_SetDualChnCompCmd(ftmBase,chnlPairNum, combinedChannelPtr->secondChannelPolarity);
        }
        else
            FTM_HAL_DisablePwmChannelOutputs(ftmBase,ChannelId + 1U);
        FTM_HAL_SetChnOutputPolarityCmd(ftmBase, ChannelId, combinedChannelPtr->mainChannelPolarity);
        FTM_HAL_SetDualChnFaultCmd(ftmBase, chnlPairNum, ((param->faultConfig)->faultMode != FTM_FAULT_CONTROL_DISABLED)? true : false);
        /* Enable sync control for channels*/
        FTM_HAL_SetDualChnPwmSyncCmd(ftmBase, chnlPairNum, true);
        FTM_HAL_SetDualChnCombineCmd(ftmBase, chnlPairNum, true);
        /* Configure dead time */
        FTM_HAL_SetDualChnDeadtimeCmd(ftmBase, chnlPairNum, combinedChannelPtr->deadTime);
    }
    /* Set enable outputs to be set to Init/default value*/
    FTM_HAL_SetInitChnOutputCmd(ftmBase, true);
    /* Enable faults (if faults were configured)*/
    if((param->faultConfig)->faultMode != FTM_FAULT_CONTROL_DISABLED )
    {
        /* Configure PWM Output behavior*/
        FTM_HAL_SetPwmFaultBehavior(ftmBase, ((param->faultConfig)->pwmOutputStateOnFault==true)?1U:0U);
        /* Configure fault filter value*/
        FTM_HAL_SetFaultInputFilterVal(ftmBase, ((param->faultConfig)->faultFilterValue));
        for(fltchannel=0; fltchannel < FTM_FEATURE_FAULT_CHANNELS; fltchannel++)
        {
            if(true == (param->faultConfig)->ftmFaultChannelParam[fltchannel].faultChannelEnabled)
            {
                /* Enable fault channel*/
                FTM_HAL_SetFaultInputCmd(ftmBase,fltchannel, true);
                /* Configure fault filter*/
                FTM_HAL_SetFaultInputFilterCmd(ftmBase, fltchannel, ((param->faultConfig)->ftmFaultChannelParam[fltchannel].faultFilterEnabled==true? 1U : 0U));
                /* Configure fault outputs*/
                FTM_HAL_SetChnFaultInputPolarityCmd(ftmBase, fltchannel, (param->faultConfig)->ftmFaultChannelParam[fltchannel].ftmFaultPinPolarity);
            }
        }
        /* Set fault interrupt*/
        if (true == ((param->faultConfig)->pwmFaultInterrupt))
        {
            FTM_HAL_SetFaultInt(ftmBase, true);
        }
        /* Enable fault control*/
        FTM_HAL_SetFaultControlMode(ftmBase, (param->faultConfig)->faultMode);
    }

    /* Configure PWM mode: edge or center aligned */
    FTM_HAL_SetCpwms(ftmBase, (param->Mode == FTM_MODE_CEN_ALIGNED_PWM)? true : false);
    /* Calculate frequency of the give FTM hw module - all channels will run at the same
         frequency */
    g_FtmModulePeriodTicks[instance] = FTM_DRV_ConvertFreqToPeriodTicks(instance, param->uFrequencyHZ);
    /* Based on Ref manual, in PWM mode CNTIN is to be set 0*/
    FTM_HAL_SetCounterInitVal(ftmBase, 0U);
    /* Write Mod register with the value of the period */
    /* For center aligned mode MOD register should be divided by 2 */
    /* For edge aligned mode period is determined by: MOD-CNTIN+1 */
    if (param->Mode == FTM_MODE_CEN_ALIGNED_PWM)
        FTM_HAL_SetMod(ftmBase, g_FtmModulePeriodTicks[instance]>>1U);
    else
        FTM_HAL_SetMod(ftmBase, g_FtmModulePeriodTicks[instance]-1U);
    for (channel=0U; channel < param->nNumIndependentPwmChannels; channel++)
    {
        independentChannelPtr = &(*param->pwmIndependentChannelConfig)[channel];
        hwChannel =  independentChannelPtr->hwChannelId;
        /* Write CnV registers and setup duty cycle and phase values*/
        FTM_DRV_UpdatePwmChannel(instance, hwChannel , independentChannelPtr->uDutyCyclePercent,  0U, false);
    }

    for (channel=0U; channel < param->nNumCombinedPwmChannels; channel++)
    {
        combinedChannelPtr = &(*param->pwmCombinedChannelConfig)[channel];
        hwChannel =  combinedChannelPtr->hwChannelId;
        /* Write CnV registers and setup duty cycle and phase values*/
        FTM_DRV_UpdatePwmChannel(instance, hwChannel , combinedChannelPtr->firstEdge,  combinedChannelPtr->secondEdge, false);
    }
    /* Configure dead time for combine mode */
    FTM_HAL_SetDeadtimeCount(ftmBase, param->deadTimeValue);
    FTM_HAL_SetDeadtimePrescale(ftmBase, param->deadTimePrescaler);
    FTM_HAL_Enable(ftmBase,true);
    FTM_HAL_SetPwmSyncMode(ftmBase,true);
    /* Set clock source to start counter */
    FTM_HAL_SetClockSource(ftmBase, s_ftmClockSource[instance]);
    return RetVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_UpdatePwmChannel
 * Description   : This function converts the input parameters representing
 * frequency in Hz to a period value in ticks needed by the hw timer. Period is
 * calculated depending on the operating mode of the FMT module and is stored in
 * global variable: g_FtmModulePeriodTicks[]. firstEdge and secondEdge can have value between
 * 0 - FTM_MAX_DUTY_CYCLE(0 = 0% duty  and FTM_MAX_DUTY_CYCLE = 100% duty). secondEdge value is used only when
 * FTM module is used in PWM combine mode.
 *
 *END**************************************************************************/
ftm_status_t FTM_DRV_UpdatePwmChannel(uint8_t instance, uint8_t channel, uint16_t firstEdge, uint16_t secondEdge, bool softwareTrigger)
{

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    DEV_ASSERT(firstEdge < FTM_MAX_DUTY_CYCLE);
    DEV_ASSERT(channel<FSL_FEATURE_FTM_CHANNEL_COUNT)
    DEV_ASSERT(secondEdge < FTM_MAX_DUTY_CYCLE);
#endif
    uint16_t hwFirstEdge = 0U;
    uint16_t hwSecondEdge = 0U;
    FTM_Type *ftmBase = g_ftmBase[instance];
    uint8_t chnlPairNum = 0;
    /* Calculate DutyCycle based of the previously calculated frequency*/
    /*for greater resolution the DutyCycle values are in the range [0. FTM_MAX_DUTY_CYCLE]
    where 0 = 0% or PWM always at Low and FTM_MAX_DUTY_CYCLE = 100% or PWM always HIGH;
    a value of 0x4000 is equivalent of 50% DutyCycle.  */
    hwFirstEdge = (g_FtmModulePeriodTicks[instance] * firstEdge) >> FTM_DUTY_TO_TICKS_SHIFT;
    hwSecondEdge = (g_FtmModulePeriodTicks[instance] * secondEdge) >> FTM_DUTY_TO_TICKS_SHIFT;
    /*adjust DutyCycle if 100% value is to be achieved.*/
    if(FTM_MAX_DUTY_CYCLE == firstEdge)
    {
        /* If expected duty is 100% then increase by 1 the value that is to be written
       to Hw so it will exceed value of period*/
        hwFirstEdge = hwFirstEdge + 1U;
    }
    chnlPairNum =  FTM_HAL_GetChnPairIndex(channel);
    /* Write LDOK bit to enable transfer between registers and their buffers*/
    FTM_HAL_SetPwmLoadCmd(ftmBase, true);

    if (true == FTM_HAL_GetDualChnCombine(ftmBase, chnlPairNum))
    {
        FTM_HAL_SetChnCountVal(ftmBase, chnlPairNum * 2U,  hwFirstEdge);
        FTM_HAL_SetChnCountVal(ftmBase, chnlPairNum * 2U+1U,  hwSecondEdge);
        FTM_HAL_SetSoftwareTriggerCmd(ftmBase, true);
    }
    else
    {
        /* Channel value is divided by 2 for up down counter mode to keep same duty*/
        if (FTM_HAL_GetCpwms(ftmBase) ==  1U)
            FTM_HAL_SetChnCountVal(ftmBase, channel, hwFirstEdge>>1);
        else
            FTM_HAL_SetChnCountVal(ftmBase, channel, hwFirstEdge);
    }
    /* Software trigger is generated to change CnV regs*/
    /* Before this please configure sync mechanism to use software trigger*/
    if (softwareTrigger)
        FTM_HAL_SetSoftwareTriggerCmd(ftmBase, true);
    return FTM_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_SetSync
 * Description   : This function configure the synchronization for PWM register ( CnV, MOD, CINT, HCR, OUTMASK).
 * If this function is used whit wrong parameters it's possible to generate wrong waveform. Registers
 * synchronization need to be configured for PWM and output compare mode.
 *
 *END**************************************************************************/
ftm_status_t FTM_DRV_SetSync(uint8_t instance, ftm_pwm_sync_t param)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
#endif
    FTM_Type *ftmBase = g_ftmBase[instance];
    /* Software and hardware triggers are not allowed in the same time */
    if (param.softwareSync & (param.hardwareSync0 | param.hardwareSync1 | param.hardwareSync2 ))
        return FTM_STATUS_ERROR;
    /* Enhanced PWM sync is used */
    FTM_HAL_SetPwmSyncModeCmd(ftmBase, true);

    /* Configure trigger source for sync */
    FTM_HAL_SetHardwareSyncTriggerSrc(ftmBase, 0U, param.hardwareSync0);
    FTM_HAL_SetHardwareSyncTriggerSrc(ftmBase, 1U, param.hardwareSync1);
    FTM_HAL_SetHardwareSyncTriggerSrc(ftmBase, 2U, param.hardwareSync2);

    /* Configure loading points */
    FTM_HAL_SetMaxLoadingCmd(ftmBase, param.maxLoadingPoint);
    FTM_HAL_SetMinLoadingCmd(ftmBase, param.minLoadingPoint);

    /* Configure sync for OUTMASK register */
    FTM_HAL_SetOutmaskPwmSyncModeCmd(ftmBase, param.outRegSync);
    FTM_HAL_SetOutmaskHardwareSyncModeCmd(ftmBase, param.hardwareSync0 | param.hardwareSync1 | param.hardwareSync2);
    FTM_HAL_SetOutmaskSoftwareSyncModeCmd(ftmBase, param.softwareSync);

    /* Configure sync for INVCTRL register */
    FTM_HAL_SetInvctrlPwmSyncModeCmd(ftmBase, param.inverterSync);
    FTM_HAL_SetInvctrlHardwareSyncModeCmd(ftmBase, (param.hardwareSync0 | param.hardwareSync0 | param.hardwareSync2));
    FTM_HAL_SetInvctrlSoftwareSyncModeCmd(ftmBase, param.softwareSync);

    /* Configure sync for SWOCTRL register */
    FTM_HAL_SetSwoctrlPwmSyncModeCmd(ftmBase, param.inverterSync);
    FTM_HAL_SetSwoctrlHardwareSyncModeCmd(ftmBase, param.hardwareSync0 | param.hardwareSync1 | param.hardwareSync2);
    FTM_HAL_SetSwoctrlSoftwareSyncModeCmd(ftmBase, param.softwareSync);

    /* Configure sync for MOD, HCR, CNTIN, and CnV registers */
    FTM_HAL_SetModCntinCvHardwareSyncModeCmd(ftmBase, param.hardwareSync0 | param.hardwareSync1 | param.hardwareSync2);
    FTM_HAL_SetModCntinCvSoftwareSyncModeCmd(ftmBase, param.softwareSync);
    FTM_HAL_SetCntinPwmSyncModeCmd(ftmBase, param.initCounterSync);

    /* Configure synchronization method (waiting next loading point or now) */
    FTM_HAL_SetCounterSoftwareSyncModeCmd(ftmBase, param.softwareSync & param.syncPoint);
    FTM_HAL_SetCounterHardwareSyncModeCmd(ftmBase, param.syncPoint & (param.hardwareSync0 | param.hardwareSync1 | param.hardwareSync2));

    /* Configure if FTM clears TRIGj (j=0,1,2) when the hardware trigger j is detected. */
    FTM_HAL_SetHwTriggerSyncModeCmd(ftmBase, param.autoClearTrigger);
    return FTM_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_InitOutputCompare
 * Description   : Configures the FTM to generate timed pulses
 * When the FTM counter matches the value of compareVal argument (this is
 * written into CnV reg), the channel output is changed based on what is specified
 * in the compareMode argument.
 *
 *END**************************************************************************/
ftm_status_t FTM_DRV_InitOutputCompare(uint8_t instance, const ftm_output_cmp_param_t *param)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
#endif
    FTM_Type *ftmBase = g_ftmBase[instance];
    ftm_output_cmp_ch_param_t * channelPtr = (void *)0;
    uint32_t channel = 0U;
    uint32_t HwChannel = 0U;
    uint8_t chnlPairNum;
    chnlPairNum = FTM_HAL_GetChnPairIndex(HwChannel);
    channelPtr = &(*param->outputChannelConfig)[channel];
    FTM_HAL_SetClockSource(ftmBase, FTM_CLOCK_SOURCE_NONE);
    FTM_HAL_SetCpwms(ftmBase, 0U);
    FTM_HAL_SetDualChnCombineCmd(ftmBase, chnlPairNum, false);
    FTM_HAL_SetDualEdgeCaptureCmd(ftmBase, chnlPairNum, false);
    /* Clear the overflow flag */
    FTM_HAL_ClearTimerOverflow(ftmBase);
    FTM_HAL_SetCounterInitVal(ftmBase, 0U);
    FTM_HAL_SetMod(ftmBase, param->maxCountValue);
    FTM_HAL_SetCounter(ftmBase, 0U);
    FTM_HAL_SetQuadDecoderCmd(ftmBase, false);
    /* Use FTM as counter, disable all the channels */
    for (channel = 0U; channel < param->nNumOutputChannels ; channel++)
    {
        channelPtr = &(*param->outputChannelConfig)[channel];
        HwChannel = channelPtr->hwChannelId;
        /* Set Channel Output mode*/
        FTM_HAL_SetChnEdgeLevel(ftmBase, HwChannel, (uint32_t)(channelPtr->ChMode));
        /* Enter counter mode for all configured channels*/
        FTM_HAL_SetChnMSnBAMode(ftmBase, HwChannel, 1U);
        /* Write initial count value for all channels */
        FTM_HAL_SetChnCountVal(ftmBase, HwChannel, channelPtr->comparedValue);
        /* Enable channel output*/
        FTM_HAL_EnablePwmChannelOutputs(ftmBase, HwChannel);
    }
    /* Set software trigger */
    FTM_HAL_SetSoftwareTriggerCmd(ftmBase, true);
    /* Set clock source to start the counter */
    FTM_HAL_SetClockSource(ftmBase, s_ftmClockSource[instance]);
    return FTM_STATUS_SUCCESS;
}
/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_DeinitOutputCompare
 * Description   : Disables compare match output control and clears FTM timer configuration
 *
 *END**************************************************************************/
ftm_status_t FTM_DRV_DeinitOutputCompare(uint8_t instance, const ftm_output_cmp_param_t *param)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
#endif
    FTM_Type *ftmBase = g_ftmBase[instance];
    ftm_output_cmp_ch_param_t * channelPtr = (void *)0;
    uint32_t channel = 0U;
    uint32_t HwChannel = 0U;
    /* Stop the FTM counter */
    FTM_HAL_SetClockSource(ftmBase, FTM_CLOCK_SOURCE_NONE);
    /* Clear the overflow flag */
    FTM_HAL_ClearTimerOverflow(ftmBase);
    FTM_HAL_SetCounterInitVal(ftmBase, 0U);
    for (channel = 0U; channel < param->nNumOutputChannels ; channel++)
    {
        channelPtr = &(*param->outputChannelConfig)[channel];
        HwChannel = channelPtr->hwChannelId;
        /* Disable Channel Output mode*/
        FTM_HAL_SetChnEdgeLevel(ftmBase, HwChannel, (uint32_t)0U);
        /* Write initial count value for all channels to 0xFFFF */
        FTM_HAL_SetChnCountVal(ftmBase, HwChannel, 0U);
    }

    /* Disable channel output*/
    FTM_HAL_DisablePwmChannelOutputs(ftmBase, HwChannel);
    /* Clear out the registers */
    FTM_HAL_SetMod(ftmBase, 0U);
    FTM_HAL_SetCounter(ftmBase, 0U);
    return FTM_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_UpdateOutputCompareChannel
 * Description   : Sets the next compare match value on the given channel starting
 *                 from the current counter value.
 *
 *END**************************************************************************/
ftm_status_t FTM_DRV_UpdateOutputCompareChannel(uint8_t instance, uint8_t channel, uint16_t NextComparematchValue, bool softwareTrigger)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    DEV_ASSERT(channel < FSL_FEATURE_FTM_CHANNEL_COUNT);
#endif
    uint32_t u32CounterValue = FTM_HAL_GetCounter(g_ftmBase[instance]);
    uint32_t u32CompareValue = 0U;
    FTM_Type *ftmBase = g_ftmBase[instance];
    uint32_t maxCounterValue = FTM_HAL_GetMod(g_ftmBase[instance]);
        /* Configure channel compare register */
    if ((uint32_t)(u32CounterValue + NextComparematchValue) > maxCounterValue)
    {
        u32CompareValue = (uint32_t)(NextComparematchValue - (maxCounterValue - u32CounterValue));
    }
    else
    {
        u32CompareValue = (uint32_t)(u32CounterValue + NextComparematchValue);
    }
    /* Set CnV value and use software trigger for sync */
    FTM_HAL_SetChnCountVal(g_ftmBase[instance], channel, u32CompareValue);
    FTM_HAL_SetSoftwareTriggerCmd(g_ftmBase[instance], true);
    if (softwareTrigger)
        FTM_HAL_SetSoftwareTriggerCmd(ftmBase, true);
    return FTM_STATUS_SUCCESS;
}

/*FUNCTION*********************************************************************************
 *
 * Function Name : FTM_DRV_InitInputCapture
 * Description   : Configures Channel Input Capture for either getting time-stamps on edge detection
 * or on signal measurement . When the edge specified in the captureMode
 * argument occurs on the channel the FTM counter is captured into the CnV register.
 * The user will have to read the CnV register separately to get this value. The filter
 * function is disabled if the filterVal argument passed in is 0. The filter function
 * is available only on channels 0,1,2,3.
 *
 *END**************************************************************************************/
ftm_status_t FTM_DRV_InitInputCapture(uint8_t instance, const ftm_input_param_t * param)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
#endif

    FTM_Type *ftmBase = g_ftmBase[instance];
    const ftm_input_ch_param_t * channelPtr = (void *)0;
    uint8_t chnlPairNum;
    uint8_t channel = 0U;
    uint8_t HwChannel = 0U;
    FTM_HAL_SetClockSource(ftmBase, FTM_CLOCK_SOURCE_NONE);
    FTM_HAL_SetCounterInitVal(ftmBase, 0U);
    FTM_HAL_SetMod(ftmBase, param->nMaxCountValue);
    FTM_HAL_SetCpwms(ftmBase, 0U);
    s_ftmMaxFtmChannel[instance] = param->nNumChannels;
    for (channel=0U; channel<s_ftmMaxFtmChannel[instance]; channel++)
    {
        channelPtr = &(*param->inputChConfig)[channel];
        HwChannel = channelPtr->hwChannelId;
        chnlPairNum = FTM_HAL_GetChnPairIndex(HwChannel);
        /* Enable filtering for input channels*/
        if (HwChannel < CHAN4_IDX)
        {
            if (true == channelPtr->FilterEn)
            {
                FTM_HAL_SetChnInputCaptureFilter(ftmBase, HwChannel, channelPtr->FilterValue);
            }
            else
            {
                FTM_HAL_SetChnInputCaptureFilter(ftmBase, HwChannel, 0U);
            }
        }
        g_FtmInputCaptureOpMode[instance][channel] = channelPtr->InputMode;
        if (FTM_EDGE_DETECT == g_FtmInputCaptureOpMode[instance][channel])
        {
            FTM_HAL_SetDualChnCombineCmd(ftmBase, chnlPairNum, false);
            FTM_HAL_SetDualEdgeCaptureCmd(ftmBase, chnlPairNum, false);
            /* Set input capture mode*/
            FTM_HAL_SetChnMSnBAMode(ftmBase, HwChannel, 0U);
            if((channelPtr->EdgeAlignement == FTM_RISING_EDGE) || (channelPtr->EdgeAlignement == FTM_FALLING_EDGE) || (channelPtr->EdgeAlignement == FTM_BOTH_EDGES))
            {
                FTM_HAL_SetChnEdgeLevel(ftmBase, HwChannel, (uint8_t)channelPtr->EdgeAlignement);
                /*TODO Enable interrupt on channel (n)*/
            }
            else
            {
#ifdef DEV_ERROR_DETECT
                DEV_ASSERT((channelPtr->EdgeAlignement == FTM_RISING_EDGE) || (channelPtr->EdgeAlignement == FTM_FALLING_EDGE) || (channelPtr->EdgeAlignement == FTM_BOTH_EDGES));
#endif
            }
        }
        else if (FTM_SIGNAL_MEASUREMENT == g_FtmInputCaptureOpMode[instance][channel])
        {
            FTM_HAL_SetDualChnCombineCmd(ftmBase, chnlPairNum, false);
            FTM_HAL_SetDualEdgeCaptureCmd(ftmBase, chnlPairNum, true);
            /* Enable dual edge input capture */
            FTM_HAL_SetDualChnDecapCmd(ftmBase,  chnlPairNum, true);
            /* If continuous mode is set*/
            if(true == channelPtr->ContinuousModeEn)
            {
                /* Set MSnA and MSnB bit*/
                FTM_HAL_SetChnMSnBAMode(ftmBase, HwChannel, 3U);
            }
            else
            {
                /* Clear MSnA and Set MSnB bit*/
                FTM_HAL_SetChnMSnBAMode(ftmBase, HwChannel, 2U);
            }
            /*TODO  Enable interrupt on channel (n) and (n+1)*/
            if(FTM_DUTY_MEASUREMENT == channelPtr->MeasurementType)
            {
                /* Measure time between rising and falling edge - positive duty*/
                FTM_HAL_SetChnEdgeLevel(ftmBase, HwChannel, FTM_RISING_EDGE);
                FTM_HAL_SetChnEdgeLevel(ftmBase, HwChannel+1U, FTM_FALLING_EDGE);
            }
            else if(FTM_PERIOD_MEASUREMENT == channelPtr->MeasurementType)
            {
                /* If channel (n) is configured to capture rising edges (ELS(n)B:ELS(n)A = 0:1) than channel (n+1) is setup to capture also raising edges (ELS(n+1)B:ELS(n+1)A = 0:1),*/
                /* Alternatively if channel (n) is configured to capture falling edges (ELS(n)B:ELS(n)A = 1:0) then channel (n+1) also captures falling edges (ELS(n+1)B:ELS(n+1)A = 1:0),*/
                FTM_HAL_SetChnEdgeLevel(ftmBase, HwChannel, FTM_RISING_EDGE);
                FTM_HAL_SetChnEdgeLevel(ftmBase, HwChannel+1U, FTM_RISING_EDGE);
            }
            else
            {
                /* Do nothing */
            }
        }
    }
    /* Set clock source to start the counter */
    FTM_HAL_SetClockSource(ftmBase, s_ftmClockSource[instance]);
    return FTM_STATUS_SUCCESS;
}

/*FUNCTION*********************************************************************************
 *
 * Function Name : FTM_DRV_DeinitInputCapture
 * Description   : Disables Channel Input Capture
 *
 *END**************************************************************************************/
ftm_status_t FTM_DRV_DeinitInputCapture(uint8_t instance, const ftm_input_param_t * param)
{
#ifdef DEV_ERROR_DETECT
    DEV_DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
#endif
    FTM_Type *ftmBase = g_ftmBase[instance];
    const ftm_input_ch_param_t * channelPtr = (void *)0;
    uint8_t chnlPairNum;
    uint8_t channel = 0U;
    uint8_t HwChannel = 0U;
    /* FTM counter is disabled */
    FTM_HAL_SetClockSource(ftmBase, FTM_CLOCK_SOURCE_NONE);
    FTM_HAL_SetCounterInitVal(ftmBase, 0U);
    FTM_HAL_SetMod(ftmBase, 0xFFFFU);
    FTM_HAL_SetCpwms(ftmBase, 0U);
    s_ftmMaxFtmChannel[instance] = param->nNumChannels;
    for (channel=0U; channel<s_ftmMaxFtmChannel[instance]; channel++)
    {
        channelPtr = &(*param->inputChConfig)[channel];
        HwChannel = channelPtr->hwChannelId;
        chnlPairNum = FTM_HAL_GetChnPairIndex(HwChannel);
        /* Disable filtering for input channels*/
        if (HwChannel < CHAN4_IDX)
        {
            FTM_HAL_SetChnInputCaptureFilter(ftmBase, HwChannel, 0U);
        }
        FTM_HAL_SetDualChnCombineCmd(ftmBase, chnlPairNum, false);
        FTM_HAL_SetDualEdgeCaptureCmd(ftmBase, chnlPairNum, false);
        FTM_HAL_SetChnEdgeLevel(ftmBase, HwChannel, (uint8_t)0U);
        FTM_HAL_DisableChnInt(ftmBase, HwChannel);
    }
    return FTM_STATUS_SUCCESS;
}

/*FUNCTION*********************************************************************************
 *
 * Function Name : FTM_DRV_GetInputCaptureMeasurement
 * Description   : This function is used to calculate the measurement and/or time stamps values
 * which are read from the C(n, n+1)V registers and stored to the static buffers.
 *
 *END**************************************************************************************/
uint16_t FTM_DRV_GetInputCaptureMeasurement(uint8_t instance, uint8_t channel)
{
    FTM_Type *ftmBase = g_ftmBase[instance];
    uint8_t chnlPairNum;
    uint16_t first_event_time;
    uint16_t second_event_time;
    if(FTM_SIGNAL_MEASUREMENT == g_FtmInputCaptureOpMode[instance][channel])
    {
        /* Measurement for signal measurement (dual edge input capture) */
        chnlPairNum = FTM_HAL_GetChnPairIndex(channel);
        /* clear channels event flags */
        FTM_HAL_HasChnEventOccurred(ftmBase, channel+1U);
        FTM_HAL_HasChnEventOccurred(ftmBase, channel);
        FTM_HAL_ClearChnEventFlag(ftmBase, channel+1U);
        FTM_HAL_ClearChnEventFlag(ftmBase, channel);
        /* Wait while measurement is done */
        while (FTM_HAL_HasChnEventOccurred(ftmBase, channel + 1U) != true){}
        /* Get time stamp for each edge */
        first_event_time = FTM_HAL_GetChnCountVal(ftmBase, channel);
        second_event_time = FTM_HAL_GetChnCountVal(ftmBase, channel + 1U);
        if (second_event_time < first_event_time)
        {
            /* Measurement when overflow occurred */
            g_FtmLatestSigMesurement[instance][chnlPairNum] = second_event_time + (FTM_HAL_GetMod(ftmBase) - first_event_time);
            /* Clear flags for channels n and n+1 */
        }
        else
        {
            /* Measurement when overflow doesn't occurred */
            g_FtmLatestSigMesurement[instance][chnlPairNum] = second_event_time - first_event_time;
            /* Clear flags for channels n and n+1 */
        }
        return (g_FtmLatestSigMesurement[instance][chnlPairNum]);
    }
    else
    {
        FTM_HAL_HasChnEventOccurred(ftmBase, channel);
        FTM_HAL_ClearChnEventFlag(ftmBase, channel);
        /* Measurement for edge detect (single edge input capture) */
        /* Wait while measurement is done */
        while (FTM_HAL_HasChnEventOccurred(ftmBase, channel) != true){}
        /* Save time stamp */
        g_FtmLatestTimeStamp[instance][channel] = (uint16_t)(FTM_HAL_GetChnCountVal(ftmBase, channel));
        return (g_FtmLatestTimeStamp[instance][channel]);
    }
}
/*FUNCTION*********************************************************************************

 * Function Name : FTM_DRV_StartNewSignalMeasurement
 * Description   : This function starts new Signal Measurements on a dual input compare channel
 * that is configured as single-shot measurement.
 *
 *END**************************************************************************************/
ftm_status_t FTM_DRV_StartNewSignalMeasurement(uint8_t instance, uint8_t channel)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    DEV_ASSERT(channel < FSL_FEATURE_FTM_CHANNEL_COUNT);
#endif
    /* Clear CH(n)F and CH(n+1)F  flags and Set DECAP bit */
    FTM_Type *ftmBase = g_ftmBase[instance];
    uint8_t chnlPairNum;
    /* Get channel mode*/
    if(FTM_FEATURE_INPUT_CAPTURE_SINGLE_SHOT == FTM_HAL_GetChnMode(ftmBase, channel))
    {
        if(FTM_SIGNAL_MEASUREMENT == g_FtmInputCaptureOpMode[instance][channel])
        {
            chnlPairNum = FTM_HAL_GetChnPairIndex(channel);
            /* Clear event flags for channel n and n + 1*/
            FTM_HAL_ClearChnEventFlag(ftmBase, (channel + 1U));
            FTM_HAL_ClearChnEventFlag(ftmBase, channel);
            /* Set DECAP bit to start measurement*/
            FTM_HAL_SetDualChnDecapCmd(ftmBase,  chnlPairNum, true);
        }
        else
        {
#ifdef DEV_ERROR_DETECT
            DEV_ASSERT(FTM_SIGNAL_MEASUREMENT == g_FtmInputCaptureOpMode[instance][channel]);
#endif
        }
    }
    else
    {
#ifdef DEV_ERROR_DETECT
        DEV_ASSERT(FTM_FEATURE_INPUT_CAPTURE_SINGLE_SHOT == FTM_HAL_GetChnMode(ftmBase, channel));
#endif
    }
    return FTM_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_QuadDecodeStart
 * Description   : Configures the parameters needed and activates quadrature
 * decode mode.
 *
 *END**************************************************************************/
ftm_status_t FTM_DRV_QuadDecodeStart(uint8_t instance, ftm_quad_decode_config_t config)
{

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
#endif
    FTM_Type *ftmBase = g_ftmBase[instance];
    /* Disable Quadrature Decoder */
    FTM_HAL_SetQuadDecoderCmd(ftmBase, false);
    FTM_HAL_SetClockSource(ftmBase, FTM_CLOCK_SOURCE_NONE);
    /* Configure Quadrature Decoder */
    /* Set decoder mode Speed and direction or Phase A and Phase B encoding */
    FTM_HAL_SetQuadMode(ftmBase, config.mode);
    /* Set filter state for Phase A (enable/disable) */
    FTM_HAL_SetQuadPhaseAFilterCmd(ftmBase, config.phase_a_config.kFtmPhaseInputFilter);
    /* Set Phase A filter value if phase filter is enabled */
    if (config.phase_a_config.kFtmPhaseInputFilter)
    {
        FTM_HAL_SetChnInputCaptureFilter(ftmBase, CHAN0_IDX, config.phase_a_config.kFtmPhaseFilterVal);
    }
    /* Set filter state for Phase B (enable/disable) */
    FTM_HAL_SetQuadPhaseBFilterCmd(ftmBase, config.phase_b_config.kFtmPhaseInputFilter);
    /* Set Phase B filter value if phase filter is enabled */
    if (config.phase_b_config.kFtmPhaseInputFilter)
    {
        FTM_HAL_SetChnInputCaptureFilter(ftmBase,  CHAN1_IDX, config.phase_b_config.kFtmPhaseFilterVal);
    }
    /* Set polarity for Phase A and Phase B */
    FTM_HAL_SetQuadPhaseAPolarity(ftmBase, config.phase_a_config.kFtmPhasePolarity);
    FTM_HAL_SetQuadPhaseBPolarity(ftmBase, config.phase_b_config.kFtmPhasePolarity);
    /* Configure counter (initial value and max value) */
    FTM_HAL_SetCounterInitVal(ftmBase, config.initial_val);
    FTM_HAL_SetMod(ftmBase, config.max_val);
    FTM_HAL_SetCounter(ftmBase, config.initial_val);
    /* Enable Quadrature Decoder */
    FTM_HAL_SetQuadDecoderCmd(ftmBase, true);
    return FTM_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_QuadDecodeStop
 * Description   : De-activates quadrature decoder mode.
 *
 *END**************************************************************************/
ftm_status_t FTM_DRV_QuadDecodeStop(uint8_t instance)
{
    FTM_Type *ftmBase = g_ftmBase[instance];
    /* Disable Quadrature decoder */
    FTM_HAL_SetQuadDecoderCmd(ftmBase, false);
    return FTM_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_QuadGetState
 * Description   : Return the current quadrature decoder state
 * (counter value, overflow flag and overflow direction)
 *
 *END**************************************************************************/
ftm_quad_decoder_state_t FTM_DRV_QuadGetState(uint8_t instance)

{
    FTM_Type *ftmBase = g_ftmBase[instance];
    ftm_quad_decoder_state_t state;
    state.overflow_direction = FTM_HAL_GetQuadTimerOverflowDir(ftmBase);
    state.overflow_flag = FTM_HAL_HasTimerOverflowed(ftmBase);
    state.counter = FTM_HAL_GetCounter(ftmBase);
    return state;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_GetFrequency
 * Description   : Retrieves the frequency of the clock source feeding the FTM counter.
 * Function will return a 0 if no clock source is selected and the FTM counter is disabled.
 * The returned value is
 *END**************************************************************************/
uint32_t FTM_DRV_GetFrequency(uint8_t instance)
{

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
#endif
    FTM_Type *ftmBase = g_ftmBase[instance];
    const clock_names_t FixedClockName[FTM_INSTANCE_COUNT] = {SIM_FTM0_CLOCKSEL, SIM_FTM1_CLOCKSEL, SIM_FTM2_CLOCKSEL, SIM_FTM3_CLOCKSEL};
    uint8_t clkPs;
    uint32_t freq = 0U;
    clkPs = (1 << FTM_HAL_GetClockPs(ftmBase));
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
#endif
    switch(s_ftmClockSource[instance])
    {
    case FTM_CLOCK_SOURCE_EXTERNALCLK:
        (void)CLOCK_SYS_GetFreq(FixedClockName[instance],  &freq);
        break;
    case FTM_CLOCK_SOURCE_FIXEDCLK:
        (void) CLOCK_SYS_GetFreq(SIM_CLK32K_CLOCK,  &freq);
        break;
    case FTM_CLOCK_SOURCE_SYSTEMCLK:
        (void) CLOCK_SYS_GetFreq(PLATFORM_CLOCK, &freq);
        break;
    default:
        break;
    }
    return (uint32_t)(freq/clkPs);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_ConvertFreqToPeriodTicks
 * Description   : This function converts the input parameters representing
 * frequency in Hz to a period value in ticks needed by the hw timer.
 *
 *END**************************************************************************/
uint16_t FTM_DRV_ConvertFreqToPeriodTicks(uint8_t instance, uint32_t freqencyHz)
{
    uint32_t uFTMhz;
    uFTMhz = FTM_DRV_GetFrequency(instance);
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    DEV_ASSERT(freqencyHz = 0U);
#endif
    return (uint16_t)(uFTMhz / freqencyHz);
}
/*******************************************************************************
 * EOF
 ******************************************************************************/

