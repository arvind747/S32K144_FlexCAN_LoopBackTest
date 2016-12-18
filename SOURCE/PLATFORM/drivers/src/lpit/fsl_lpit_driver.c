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

#include "fsl_lpit_driver.h"

/*******************************************************************************
* Variables
******************************************************************************/

/* LPIT source clock variable which will be updated in LPIT_DRV_Init */
uint32_t g_lpitSourceClockFrequency[LPIT_INSTANCE_COUNT] = {0};

/*******************************************************************************
* Code
******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_Init
 * Description   : Initializes LPIT module.
 * This function must be called before calling all the other LPIT driver functions.
 * This function enables the LPIT module, resets LPIT module, configures LPIT module operation in
 * Debug and DOZE mode. The lpit configuration structure passed into function will
 * affect all timer channels.
 *
 *END**************************************************************************/
lpit_status_t LPIT_DRV_Init(uint32_t instance, const lpit_user_config_t *config)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);
    DEV_ASSERT(config);
#endif

    LPIT_Type * base = g_lpitBase[instance];
    clock_names_t instanceClkName = g_lpitClkNames[instance];
    lpit_status_t reVal = LPIT_STATUS_SUCCESS;
    /* Get current clock frequency of LPIT instance */
    CLOCK_SYS_GetFreq(instanceClkName, &g_lpitSourceClockFrequency[instance]);
    if(g_lpitSourceClockFrequency[instance] == 0U)
    {
        reVal = LPIT_STATUS_FAIL;
    }
    else
    {
        /* Reset LPIT module */
        LPIT_HAL_Reset(base);
        /* Enable LPIT module clock*/
        LPIT_HAL_Enable(base);
        /* Set timer run or stop in debug/DOZE mode*/
        LPIT_HAL_SetTimerRunInDebugCmd(base, config->enableRunInDebug);
        LPIT_HAL_SetTimerRunInDozeCmd(base, config->enableRunInDoze);
    }
    return reVal;

}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_Deinit
 * Description   : Disables LPIT module .
 * This function will disable functional clock for LPIT module.
 * LPIT_DRV_Init must be called in order to use LPIT again.
 *
 *END**************************************************************************/
lpit_status_t LPIT_DRV_Deinit(uint32_t instance)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);
#endif

    LPIT_Type * base = g_lpitBase[instance];
    lpit_status_t reVal = LPIT_STATUS_SUCCESS;
    clock_names_t instanceClkName = g_lpitClkNames[instance];
    /* Get current clock frequency of LPIT instance */
    CLOCK_SYS_GetFreq(instanceClkName, &g_lpitSourceClockFrequency[instance]);
    if(g_lpitSourceClockFrequency[instance] == 0U)
    {
        reVal = LPIT_STATUS_FAIL;
    }
    else
    {
        /* Disable LPIT module clock*/
        LPIT_HAL_Disable(base);
    }

    return reVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_InitChannel
 * Description   : Initializes LPIT channel.
 * This function initialize LPIT timers by channel. Pass in timer channel number and its
 * configuration structure. Timer channels do not start counting by default after calling this
 * function. Function LPIT_DRV_StartTimerChannels must be called to start timer channel counting.
 * Call LPIT_DRV_SetTimerPeriodByUs to re-set the period.
 *
 *END**************************************************************************/
lpit_status_t LPIT_DRV_InitChannel(uint32_t instance,
                          uint32_t channel,
                          const lpit_user_channel_config_t * userChannelConfig)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);
    DEV_ASSERT(userChannelConfig);
    DEV_ASSERT(channel < LPIT_TVAL_CVAL_TCTRL_COUNT);
#endif

    LPIT_Type * base = g_lpitBase[instance];
    lpit_status_t reVal = LPIT_STATUS_SUCCESS;
    if((channel == 0U) && (userChannelConfig->chainChannel == true))
    {
        reVal = LPIT_STATUS_FAIL;
    }
    else
    {
        /* Setup the timer channel operation mode, chain mode */
        LPIT_HAL_SetTimerChannelChainCmd(base, channel, userChannelConfig->chainChannel);
        LPIT_HAL_SetTimerChannelModeCmd(base, channel, userChannelConfig->timerMode);

        /* Setup timer period in microsecond unit */
        reVal = LPIT_DRV_SetTimerPeriodByUs(instance, channel, userChannelConfig->periodUs);

        if(reVal == LPIT_STATUS_VALID_PERIOD)
        {
            /* Setup the channel trigger source, trigger select, reload on trigger, stop on timeout,
               start on trigger */
            LPIT_HAL_SetTriggerSelectCmd(base, channel, userChannelConfig->triggerSelect);
            LPIT_HAL_SetTriggerSourceCmd(base, channel, userChannelConfig->triggerSource);
            LPIT_HAL_SetReloadOnTriggerCmd(base, channel, userChannelConfig->enableReloadOnTrigger);
            LPIT_HAL_SetStopOnInterruptCmd(base, channel, userChannelConfig->enableStopOnInterrupt);
            LPIT_HAL_SetStartOnTriggerCmd(base, channel, userChannelConfig->enableStartOnTrigger);
            /* Configure NVIC */
            if(userChannelConfig->isInterruptEnabled)
            {
                /* Setup interrupt enable for timer channel on timeout*/
                LPIT_HAL_EnableInterruptTimerChannels(base, 1 << channel);
                INT_SYS_EnableIRQ(g_lpitIrqId[instance]);
            }
            reVal = LPIT_STATUS_SUCCESS;
        }
        else
        {
            reVal = LPIT_STATUS_FAIL;
        }
    }

    return reVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_StartTimerChannels
 * Description   : Start timer channel counting.
 * This function allows starting timer channels simultaneously.
 * After calling this function, timer channels will operate according to the configured operation mode.
 * For example: In compare modes, the timer channels load period value and decrement.
 * Each time a timer channel reaches 0,it generates a trigger pulse , timeout pulse
 * and sets the timeout interrupt flag.
 *
 *END**************************************************************************/
void LPIT_DRV_StartTimerChannels(uint32_t instance, uint32_t mask)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);
    DEV_ASSERT(mask < (1U << LPIT_TVAL_CVAL_TCTRL_COUNT));
#endif

    LPIT_Type * base = g_lpitBase[instance];
    LPIT_HAL_StartTimerChannels(base, mask);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_StopTimerChannels
 * Description   : Stop timer channel from counting.
 * This function will stop timer channels from counting simultaneously. Timer channels will
 * reload their periods respectively after calling LPIT_DRV_StartTimerChannels next time.
 *
 *END**************************************************************************/
void LPIT_DRV_StopTimerChannels(uint32_t instance, uint32_t mask)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);
    DEV_ASSERT(mask < (1U << LPIT_TVAL_CVAL_TCTRL_COUNT));
#endif

    LPIT_Type * base = g_lpitBase[instance];
    LPIT_HAL_StopTimerChannels(base, mask);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_SetTimerPeriodByUs
 * Description   : Set timer channel period in microseconds unit.
 * The period range depends on the frequency of LPIT source clock. If required
 * period is out the range, try to use other mode timer if applicable.
 * This function is only valid for one single channel. If channels are chained together,
 * the period here will make no sense.
 *
 *END**************************************************************************/
lpit_status_t LPIT_DRV_SetTimerPeriodByUs(uint32_t instance, uint32_t channel, uint32_t us)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);
    DEV_ASSERT(channel < LPIT_TVAL_CVAL_TCTRL_COUNT);
#endif

    LPIT_Type * base = g_lpitBase[instance];
    lpit_timer_modes_t timerMode;
    lpit_status_t reVal = LPIT_STATUS_VALID_PERIOD;
    uint64_t count;
    clock_names_t instanceClkName = g_lpitClkNames[instance];
    /* Get current clock frequency of LPIT instance */
    CLOCK_SYS_GetFreq(instanceClkName, &g_lpitSourceClockFrequency[instance]);
    /* Calculate the count value, assign it to timer counter register.*/
    count = ((uint64_t)us) * g_lpitSourceClockFrequency[instance];
    count = count / 1000000U - 1U;
    /* Get current timer operation mode */
    timerMode = LPIT_HAL_GetTimerChannelModeCmd(base, channel);
    if (count <= MAX_PERIOD_COUNT)
    {
        if (timerMode == LPIT_DUAL_PERIODIC_COUNTER)
        {
            if(count > MAX_PERIOD_COUNT_IN_DUAL_16BIT_MODE)
            {
                reVal = LPIT_STATUS_INVALID_PERIOD;
            }
            else
            {
                if (count > (MAX_PERIOD_COUNT_IN_DUAL_16BIT_MODE / 2U))
                {
                    /* Calculate the count value, assign it to timer counter register.*/
                    count = (((count - (MAX_PERIOD_COUNT_IN_DUAL_16BIT_MODE / 2U)) << 16U) | (MAX_PERIOD_COUNT_IN_DUAL_16BIT_MODE / 2U));
                }
            }
        }
    }
    else
    {
        reVal = LPIT_STATUS_INVALID_PERIOD;
    }
    if (reVal == LPIT_STATUS_VALID_PERIOD)
    {
        LPIT_HAL_SetTimerPeriodByCount(base, channel, count);
    }

    return reVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_GetTimerPeriodByUs
 * Description   : Gets the timer channel period in microseconds.
 *
 *END**************************************************************************/
uint64_t LPIT_DRV_GetTimerPeriodByUs(uint32_t instance, uint32_t channel)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);
    DEV_ASSERT(channel < LPIT_TVAL_CVAL_TCTRL_COUNT);
#endif

    LPIT_Type * base = g_lpitBase[instance];
    lpit_timer_modes_t timerMode;
    uint64_t currentPeriod;
    clock_names_t instanceClkName = g_lpitClkNames[instance];
    /* Get current clock frequency of LPIT instance */
    CLOCK_SYS_GetFreq(instanceClkName, &g_lpitSourceClockFrequency[instance]);
    /* Get current timer period by count.*/
    currentPeriod = LPIT_HAL_GetTimerPeriodByCount(base, channel);
    /* Get current timer operation mode */
    timerMode = LPIT_HAL_GetTimerChannelModeCmd(base, channel);

    if ( timerMode == LPIT_DUAL_PERIODIC_COUNTER )
    {
        /* Convert count numbers to microseconds unit.*/
        currentPeriod = ((((currentPeriod & ((MAX_PERIOD_COUNT_IN_DUAL_16BIT_MODE / 2U) << 16U)) >> 16U) \
                        + (currentPeriod & (MAX_PERIOD_COUNT_IN_DUAL_16BIT_MODE / 2U)) + 1U) * 1000000U) / g_lpitSourceClockFrequency[instance];
    }
    else
    {
        /* Convert count numbers to microseconds unit.*/
        currentPeriod = ((currentPeriod + 1U) * 1000000U) / g_lpitSourceClockFrequency[instance];
    }

    return currentPeriod;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_GetCurrentTimerUs
 * Description   : Read current timer channel counting value in microseconds unit.
 * This function will return an absolute time stamp in the microseconds.
 * One common use of this function is to measure the running time of part of
 * code. Just call this function at both the beginning and end of code, the time
 * difference between these two time stamp will be the running time (Need to
 * make sure the running time will not exceed the timer period).
 *
 *END**************************************************************************/
uint64_t LPIT_DRV_GetCurrentTimerUs(uint32_t instance, uint32_t channel)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);
    DEV_ASSERT(channel < LPIT_TVAL_CVAL_TCTRL_COUNT);
#endif

    LPIT_Type * base = g_lpitBase[instance];
    lpit_timer_modes_t timerMode;
    uint64_t currentTime;
    clock_names_t instanceClkName = g_lpitClkNames[instance];
    /* Get current clock frequency of LPIT instance */
    CLOCK_SYS_GetFreq(instanceClkName, &g_lpitSourceClockFrequency[instance]);
    /* Get current timer count, and reverse it to up-counting.*/
    currentTime = LPIT_HAL_GetCurrentTimerCount(base, channel);
    /* Get current timer operation mode */
    timerMode = LPIT_HAL_GetTimerChannelModeCmd(base, channel);

    if ( timerMode == LPIT_DUAL_PERIODIC_COUNTER )
    {
        /* Convert count numbers to microseconds unit.*/
        currentTime = ((((currentTime & ((MAX_PERIOD_COUNT_IN_DUAL_16BIT_MODE / 2U) << 16U)) >> 16U) \
                      + (currentTime & (MAX_PERIOD_COUNT_IN_DUAL_16BIT_MODE / 2U)) + 1U) * 1000000U) / g_lpitSourceClockFrequency[instance];
    }
    else
    {
        /* Convert count numbers to microseconds unit.*/
        currentTime = ((currentTime + 1U) * 1000000U) / g_lpitSourceClockFrequency[instance];
    }

    return currentTime;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_SetTimerPeriodByCount
 * Description   : Sets the timer channel period in count unit.
 * Timer channels begin counting from the value set by this function.
 * The counter period of a running timer channel can be modified by first setting a new load value,
 * the value will be loaded after the timer channel expires, To abort the current cycle
 * and start a timer channel period with the new value, the timer channel must be disabled
 * and enabled again.
 *
 *END**************************************************************************/
lpit_status_t LPIT_DRV_SetTimerPeriodByCount(uint32_t instance, uint32_t channel, uint32_t count)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);
    DEV_ASSERT(channel < LPIT_TVAL_CVAL_TCTRL_COUNT);
#endif

    LPIT_Type * base = g_lpitBase[instance];
    lpit_timer_modes_t timerMode;
    lpit_status_t reVal = LPIT_STATUS_VALID_PERIOD;
    timerMode = LPIT_HAL_GetTimerChannelModeCmd(base, channel);
    if (timerMode == LPIT_DUAL_PERIODIC_COUNTER)
    {
        if(count > MAX_PERIOD_COUNT_IN_DUAL_16BIT_MODE)
        {
            reVal = LPIT_STATUS_INVALID_PERIOD;
        }
        else
        {
            if (count > (MAX_PERIOD_COUNT_IN_DUAL_16BIT_MODE / 2U))
            {
                /* Calculate the count value, assign it to timer channel counter register.*/
                count = (((count - (MAX_PERIOD_COUNT_IN_DUAL_16BIT_MODE / 2U)) << 16U) | (MAX_PERIOD_COUNT_IN_DUAL_16BIT_MODE / 2U));
            }
        }
    }
    if(reVal == LPIT_STATUS_VALID_PERIOD)
    {
        LPIT_HAL_SetTimerPeriodByCount(base, channel, count);
    }
    return reVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_GetTimerPeriodByCount
 * Description   : Returns the current timer channel period in count unit.
 *
 *END**************************************************************************/
uint32_t LPIT_DRV_GetTimerPeriodByCount(uint32_t instance, uint32_t channel)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);
    DEV_ASSERT(channel < LPIT_TVAL_CVAL_TCTRL_COUNT);
#endif

    LPIT_Type * base = g_lpitBase[instance];
    lpit_timer_modes_t timerMode;
    uint32_t currentPeriod;
    /* Get current timer channel period by count.*/
    currentPeriod = LPIT_HAL_GetTimerPeriodByCount(base, channel);
    /* Get current timer channel operation mode */
    timerMode = LPIT_HAL_GetTimerChannelModeCmd(base, channel);

    if ( timerMode == LPIT_DUAL_PERIODIC_COUNTER )
    {
        currentPeriod = ((currentPeriod & ((MAX_PERIOD_COUNT_IN_DUAL_16BIT_MODE / 2U) << 16U)) >> 16U) \
                        + (currentPeriod & (MAX_PERIOD_COUNT_IN_DUAL_16BIT_MODE / 2U));
    }

    return currentPeriod;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_GetCurrentTimerCount
 * Description   : Reads the current timer channel counting value in count.
 * This function returns the real-time timer channel counting value, in a range from 0
 * to a timer channel period.
 *
 *END**************************************************************************/
uint32_t LPIT_DRV_GetCurrentTimerCount(uint32_t instance, uint32_t channel)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);
    DEV_ASSERT(channel < LPIT_TVAL_CVAL_TCTRL_COUNT);
#endif

    LPIT_Type * base = g_lpitBase[instance];
    lpit_timer_modes_t timerMode;
    uint32_t currentTime;
    /* Get current timer period by count.*/
    currentTime = LPIT_HAL_GetCurrentTimerCount(base, channel);
    /* Get current timer operation mode */
    timerMode = LPIT_HAL_GetTimerChannelModeCmd(base, channel);

    if ( timerMode == LPIT_DUAL_PERIODIC_COUNTER )
    {
        currentTime = ((currentTime & ((MAX_PERIOD_COUNT_IN_DUAL_16BIT_MODE / 2U) << 16U)) >> 16U) \
                      + (currentTime & (MAX_PERIOD_COUNT_IN_DUAL_16BIT_MODE / 2U));
    }

    return currentTime;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_GetInterruptFlagTimerChannels
 * Description   : Reads the current interrupt flag of timer channels .
 *
 *END**************************************************************************/
uint32_t LPIT_DRV_GetInterruptFlagTimerChannels(uint32_t instance,  uint32_t mask)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);
    DEV_ASSERT(mask < (1U << LPIT_TVAL_CVAL_TCTRL_COUNT));
#endif

    LPIT_Type * base = g_lpitBase[instance];
    return LPIT_HAL_GetInterruptFlagTimerChannels(base, mask);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_DRV_ClearInterruptFlagTimerChannels
 * Description   : Clears the interrupt flag of timer channels.
 *
 *END**************************************************************************/
void LPIT_DRV_ClearInterruptFlagTimerChannels(uint32_t instance, uint32_t mask)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPIT_INSTANCE_COUNT);
    DEV_ASSERT(mask < (1U << LPIT_TVAL_CVAL_TCTRL_COUNT));
#endif

    LPIT_Type * base = g_lpitBase[instance];
    LPIT_HAL_ClearInterruptFlagTimerChannels(base, mask);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
