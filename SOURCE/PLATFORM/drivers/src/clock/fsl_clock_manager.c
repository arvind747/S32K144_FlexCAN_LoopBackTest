/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
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
#include "fsl_clock_manager.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Macro for clock manager critical section. */
#if (USE_RTOS)
    mutex_t g_clockLock;
    #define CLOCK_SYS_LOCK_INIT()    OSA_MutexCreate(&g_clockLock)
    #define CLOCK_SYS_LOCK()         OSA_MutexLock(&g_clockLock, OSA_WAIT_FOREVER)
    #define CLOCK_SYS_UNLOCK()       OSA_MutexUnlock(&g_clockLock)
    #define CLOCK_SYS_LOCK_DEINIT()  OSA_MutexDestroy(&g_clockLock)
#else
    #define CLOCK_SYS_LOCK_INIT()    do {}while(0)
    #define CLOCK_SYS_LOCK()         do {}while(0)
    #define CLOCK_SYS_UNLOCK()       do {}while(0)
    #define CLOCK_SYS_LOCK_DEINIT()  do {}while(0)
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/
static clock_manager_state_t g_clockState;


/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_Init
 * Description   : Install pre-defined clock configurations.
 * This function installs the pre-defined clock configuration table to the
 * clock manager.
 *
 *END**************************************************************************/
clock_manager_error_code_t CLOCK_SYS_Init(clock_manager_user_config_t const **clockConfigsPtr,
                              uint8_t configsNumber,
                              clock_manager_callback_user_config_t **callbacksPtr,
                              uint8_t callbacksNumber)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(clockConfigsPtr);
    DEV_ASSERT(callbacksPtr);
#endif

    CLOCK_SYS_LOCK_INIT();

    g_clockState.configTable     = clockConfigsPtr;
    g_clockState.clockConfigNum  = configsNumber;
    g_clockState.callbackConfig  = callbacksPtr;
    g_clockState.callbackNum     = callbacksNumber;

    /*
     * errorCallbackIndex is the index of the callback which returns error
     * during clock mode switch. If all callbacks return success, then the
     * errorCallbackIndex is callbacksNumber.
     */
    g_clockState.errorCallbackIndex = callbacksNumber;

    return CLOCK_MANAGER_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_UpdateConfiguration
 * Description   : Send notification and change system clock configuration.
 * This function sends the notification to all callback functions, if all
 * callbacks return OK or forceful policy is used, this function will change
 * system clock configuration.
 *
 *END**************************************************************************/
clock_manager_error_code_t CLOCK_SYS_UpdateConfiguration(uint8_t targetConfigIndex,
                                                   clock_manager_policy_t policy)
{
    uint8_t callbackIdx;
    clock_manager_error_code_t ret = CLOCK_MANAGER_SUCCESS;

    clock_manager_callback_user_config_t* callbackConfig;

    clock_notify_struct_t notifyStruct;
    notifyStruct.targetClockConfigIndex = targetConfigIndex;
    notifyStruct.policy                 = policy;

    /* Clock configuration index is out of range. */
    if (targetConfigIndex >= g_clockState.clockConfigNum)
    {
        return CLOCK_MANAGER_ERROR_OUT_OF_RANGE;
    }

    /* TODO:bareboard Implementation for EnterCritical */
    __asm volatile ("cpsid i" : : : "memory");
    /* Set errorcallbackindex as callbackNum, which means no callback error now.*/
    g_clockState.errorCallbackIndex = g_clockState.callbackNum;

    /* First step: Send "BEFORE" notification. */
    notifyStruct.notifyType = CLOCK_MANAGER_NOTIFY_BEFORE;

    /* Send notification to all callback. */
    for (callbackIdx=0; callbackIdx<g_clockState.callbackNum; callbackIdx++)
    {
        callbackConfig = g_clockState.callbackConfig[callbackIdx];
        if ((callbackConfig) &&
            ((uint8_t)callbackConfig->callbackType & (uint8_t)CLOCK_MANAGER_NOTIFY_BEFORE))
        {
            if (CLOCK_MANAGER_SUCCESS !=
                    (*callbackConfig->callback)(&notifyStruct,
                        callbackConfig->callbackData))
            {
                g_clockState.errorCallbackIndex = callbackIdx;
                /* Save the error callback index. */
                ret = CLOCK_MANAGER_ERROR_NOTIFICATION_BEFORE;

                if (CLOCK_MANAGER_POLICY_AGREEMENT == policy)
                {
                    break;
                }
            }
        }
    }

    /* If all callback success or forceful policy is used. */
    if ((CLOCK_MANAGER_SUCCESS == ret) ||
        (policy == CLOCK_MANAGER_POLICY_FORCIBLE))
    {
        /* clock mode switch. */
        /* TODO:bareboard Implementation for EnterCritical */
    	__asm volatile ("cpsid i" : : : "memory");
        CLOCK_SYS_SetConfiguration(g_clockState.configTable[targetConfigIndex]);

        g_clockState.curConfigIndex = targetConfigIndex;
        /* TODO:bareboard Implementation for ExitCritical */
        __asm volatile ("cpsie i" : : : "memory");

        notifyStruct.notifyType = CLOCK_MANAGER_NOTIFY_AFTER;

        for (callbackIdx=0; callbackIdx<g_clockState.callbackNum; callbackIdx++)
        {
            callbackConfig = g_clockState.callbackConfig[callbackIdx];
            if ((callbackConfig) &&
                ((uint8_t)callbackConfig->callbackType & (uint8_t)CLOCK_MANAGER_NOTIFY_AFTER))
            {
                if (CLOCK_MANAGER_SUCCESS !=
                        (*callbackConfig->callback)(&notifyStruct,
                            callbackConfig->callbackData))
                {
                    g_clockState.errorCallbackIndex = callbackIdx;
                    /* Save the error callback index. */
                    ret = CLOCK_MANAGER_ERROR_NOTIFICATION_AFTER;

                    if (CLOCK_MANAGER_POLICY_AGREEMENT == policy)
                    {
                        break;
                    }
                }
            }
        }
    }
    else /* Error occurs, need to send "RECOVER" notification. */
    {
        notifyStruct.notifyType = CLOCK_MANAGER_NOTIFY_RECOVER;
        while (callbackIdx--)
        {
            callbackConfig = g_clockState.callbackConfig[callbackIdx];
            if (callbackConfig)
            {
                (*callbackConfig->callback)(&notifyStruct,
                        callbackConfig->callbackData);
            }
        }
    }


    /* TODO:bareboard Implementation for ExitCritical */
    __asm volatile ("cpsie i" : : : "memory");

    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_GetCurrentConfiguration
 * Description   : Get current clock configuration index.
 *
 *END**************************************************************************/
uint8_t CLOCK_SYS_GetCurrentConfiguration(void)
{
    return g_clockState.curConfigIndex;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_GetErrorCallback
 * Description   : Get the callback which returns error in last clock switch.
 *
 *END**************************************************************************/
clock_manager_callback_user_config_t* CLOCK_SYS_GetErrorCallback(void)
{
    /* If all callbacks return success. */
    if (g_clockState.errorCallbackIndex >= g_clockState.clockConfigNum)
    {
        return (void*)0;
    }
    else
    {
        return g_clockState.callbackConfig[g_clockState.errorCallbackIndex];
    }
}


/*******************************************************************************
 * EOF
 ******************************************************************************/

