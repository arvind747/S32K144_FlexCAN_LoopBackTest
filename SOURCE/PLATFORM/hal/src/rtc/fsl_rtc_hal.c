/*
 * Copyright (c) 2014 - 2015, Freescale Semiconductor, Inc.
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

#include "fsl_rtc_hal.h"

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name RTC_HAL_Init
 * Description   This function initializes the RTC instance
 * Return        RTC_STAT_SUCCESS if the operation was successful, RTC_STAT_ERROR 
 *               if there was a problem or RTC_STATUS_LOCKED if at least one 
 *               register is locked
 *
 *END**************************************************************************/

rtc_status_t RTC_HAL_Init(RTC_Type * const base)
{
    /* Check if the registers are locked */
    if((base->LR & 0xFF) != 0xFF)
        return RTC_STAT_LOCKED;

    /* Set all registers to default values, except for RTC IER */
    /* Disable all interrupts */
    base -> IER = 0UL;
    /* Clear all flags and disable the counter */
    base -> SR  = 0UL;
    /* Set Time Seconds Registers to 1 to avoid triggering Time
     * Invalid Interrupt
     */
    base -> TSR = 1UL;
    /* Clear Time Prescaler Register */
    base -> TPR = 0UL;
    /* Clear Time Alarm Register */
    base -> TAR = 0UL;
    /* Set Configuration Register to reset value */
    base -> CR  = 0UL;
    /* Set Lock Register to default value */
    base -> LR  = 0xFFUL;

    /* Check if the configuration was successful */
    if(RTC_HAL_GetTimeInvalidFlag(base) == true)
        return RTC_STAT_ERROR;
    else
        return RTC_STAT_SUCCESS;

}


/*FUNCTION**********************************************************************
 *
 * Function Name RTC_HAL_Enable
 * Description   This function enables the RTC counter
 * Return        RTC_STAT_SUCCESS if the operation was successful, RTC_STAT_ERROR 
 *               if there was a problem
 *
 *END**************************************************************************/
rtc_status_t RTC_HAL_Enable(RTC_Type * const base)
{

    /* Check if the RTC counter is enabled or if the Time setup is invalid */
    if((RTC_HAL_GetTimeCounterEnable(base) == true) ||
            (RTC_HAL_GetTimeInvalidFlag(base) == true))
        return  RTC_STAT_ERROR;

    /* Enable oscillator and seconds counter */
    RTC_HAL_SetOscillatorEnable(base, true);
    RTC_HAL_SetTimeCounterEnable(base, true);

    return RTC_STAT_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name RTC_HAL_Disable
 * Description   This function disables the RTC counter
 * Return        RTC_STAT_SUCCESS if the operation was successful, RTC_STAT_ERROR 
 *               if there was a problem
 *
 *END**************************************************************************/
rtc_status_t RTC_HAL_Disable(RTC_Type * const base)
{

    if(RTC_HAL_GetTimeCounterEnable(base) == true)
    {
        RTC_HAL_SetTimeCounterEnable(base, false);
    }

    /* Read TCE bit to check if the counter is really disabled and return the
     * corresponding result.
     *  -   Error if the timer is still enabled (The register can be locked)
     *  -   Success if the timer is disabled
     */
    return RTC_HAL_GetTimeCounterEnable(base) ?
                                            RTC_STAT_ERROR : RTC_STAT_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name RTC_HAL_SetOscLoadConfig
 * Description   This function configures RTC Oscillator additional load.
 *               The load must be configured ONLY when the oscillator is
 *               disabled via the OSCE bit in the RTC Control register.
 * Return        RTC_STAT_SUCCESS if the operation was successful, RTC_STAT_ERROR 
 *               if there was a problem
 *
 *END**************************************************************************/
rtc_status_t RTC_HAL_SetOscLoadConfig(RTC_Type * const base,
                                              uint32_t mask)
{

    /* Check if the Oscillator is enabled or if the control register is
     * locked and return ERROR if true
     */
    if((RTC_HAL_GetOscillatorEnable(base) == true) ||
            (RTC_HAL_GetControlRegisterLock(base) == true))
        return RTC_STAT_ERROR;
    else
    {
        /* Write the load config to the RTC CR */
        uint32_t temp = base -> CR;
        temp &= ~(RTC_CR_SC2P_MASK | RTC_CR_SC4P_MASK |
                   RTC_CR_SC8P_MASK | RTC_CR_SC16P_MASK);
        base -> CR = (temp | mask);

        return RTC_STAT_SUCCESS;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name RTC_HAL_GetOscLoadConfig
 * Description   This function retrieves the Oscillator additional load
 *               configuration.
 * Return        Load config
 *
 *END**************************************************************************/
uint32_t RTC_HAL_GetOscLoadConfig(RTC_Type * const base)
{
    uint32_t mask = (RTC_CR_SC2P_MASK | RTC_CR_SC4P_MASK |
                        RTC_CR_SC8P_MASK | RTC_CR_SC16P_MASK);

    return (base->CR & mask);
}

/*FUNCTION**********************************************************************
 *
 * Function Name RTC_HAL_SetTimeSecondsRegister
 * Description
 *        This function along with SetTimePrescalerRegister will help you set
 *        the starting time at a specified value.
 *        The write will fail if the Time Counter is enabled and will return
 *        RTC_STAT_ERROR, otherwise the return will be RTC_STAT_SUCCESS
 *
 * Return RTC_STAT_SUCCESS if the write is succeeded or RTC_STAT_ERROR if
 *        the counter is enabled.
 *
 *END**************************************************************************/
rtc_status_t RTC_HAL_SetTimeSecondsRegister(RTC_Type * const base,
                                                     uint32_t seconds) {

    if(RTC_HAL_GetTimeCounterEnable(base) == true)
        return RTC_STAT_ERROR;
    else
    {
        uint32_t tmp = base->TSR;
        tmp &= ~(RTC_TSR_TSR_MASK);
        tmp |= RTC_TSR_TSR(seconds);
        base->TSR = tmp;
        return RTC_STAT_SUCCESS;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name RTC_HAL_SetTimePrescalerRegister
 * Description
 *        This function along with SetTimeSecondsRegister will help you set
 *        the starting time at a specified value.
 *        The write will fail if the Time Counter is enabled and will return
 *        RTC_STAT_ERROR, otherwise the return will be RTC_STAT_SUCCESS
 *
 * Return RTC_STAT_SUCCESS if the write is succeeded or RTC_STAT_ERROR if
 *        the counter is enabled.
 *
 *END**************************************************************************/
rtc_status_t RTC_HAL_SetTimePrescalerRegister(RTC_Type * const base,
                                                         uint16_t value) {

    if(RTC_HAL_GetTimeCounterEnable(RTC) == true)
        return RTC_STAT_ERROR;
    else
    {
        uint32_t tmp = base->TPR;
        tmp &= ~(RTC_TPR_TPR_MASK);
        tmp |= RTC_TPR_TPR(value);
        base->TPR = tmp;
        return RTC_STAT_SUCCESS;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name RTC_HAL_ConfigureRegisterLock
 * Description
 *          This method will allow you to lock the registers. It will return
 *          RTC_STAT_SUCCESS if the lock was successful or if the register
 *          was already locked, RTC_STATUS_LOCKED if the Lock Register is
 *          already locked and RTC_STAT_ERROR if the registerToConfig
 *          parameter is not a valid register.
 *
 *
 * Return Status of the operation
 *
 *END**************************************************************************/
rtc_status_t RTC_HAL_ConfigureRegisterLock(RTC_Type * const base,
                        rtc_lock_register_select_t registerToConfig)
{
    rtc_status_t statusCode = RTC_STAT_SUCCESS;

    /* Check if the Lock Register is already locked,
     * if true, any other register lock status cannot
     * be modified.
     */
    if(RTC_HAL_GetLockRegisterLock(base) == true)
    {
        statusCode = RTC_STAT_LOCKED;
    }
    else
    {
        /* If the Lock Register is not locked we can
         * configure the register lock.
         */
        switch(registerToConfig)
        {
        case RTC_LOCK_REG_LOCK:
            RTC_HAL_LockRegisterLock(base);
            break;
        case RTC_STAT_REG_LOCK:
            RTC_HAL_StatusRegisterLock(base);
            break;
        case RTC_CTRL_REG_LOCK:
            RTC_HAL_ControlRegisterLock(base);
            break;
        case RTC_TCE_REG_LOCK:
            RTC_HAL_TimeCompensationLock(base);
            break;
        default:
            /* If the register is not recognized, return error */
            statusCode = RTC_STAT_ERROR;
            break;
        }
    }

    return statusCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name RTC_HAL_isRegisterLocked
 * Description
 *              This method will get the register lock status
 *
 *
 * Return True if the register is locked, false if not
 *
 *END**************************************************************************/
bool RTC_HAL_isRegisterLocked(RTC_Type * const base,
                        rtc_lock_register_select_t reg)
{
    bool state = false;

    switch(reg)
    {
    case RTC_LOCK_REG_LOCK:
        state = RTC_HAL_GetLockRegisterLock(base);
        break;
    case RTC_CTRL_REG_LOCK:
        state = RTC_HAL_GetControlRegisterLock(base);
        break;
    case RTC_STAT_REG_LOCK:
        state = RTC_HAL_GetStatusRegisterLock(base);
        break;
    case RTC_TCE_REG_LOCK:
        state = RTC_HAL_GetTimeCompensationLock(base);
        break;
    }

    return state;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
