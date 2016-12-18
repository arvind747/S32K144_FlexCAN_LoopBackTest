/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
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

 /*!
 * @file fsl_wdog_driver.c
 *
 * @note Violates MISRA 2012 Rule 1.1, required, The program shall contain no
 * violation of the standard C syntax and constraints, and shall not exceed the
 * implementation's translation limits.
 * The standard permits usage of language extensions, and __asm is a keyword
 * supported by the used compilers as a language extension.
 *
 * @note Violates MISRA 2012 Rule 1.3, required, There shall be no occurrence of
 * undefined or critical unspecified behaviour.
 * The addresses of the stack variables are only used at local scope.
 *
 * @note Violates MISRA 2012 Rule 2.2, required, There shall be no dead code.
 * This is caused by the usage of a NOP sequence, but the standard gives 
 * __asm("NOP") as an example of code compliant with this rule.
 *
 * @note Violates MISRA 2012 Rule 11.6, required, A cast shall not be performed
 * between pointer to void and an arithmetic type.
 * The base addresses are provided as integers so they need to be cast to
 * pointers.
 */
 
#include "fsl_wdog_driver.h"


/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief Table of base addresses for WDOG instances. */
WDOG_Type * const g_wdogBase[] = WDOG_BASE_PTRS;

/*! @brief Table to save WDOG IRQ enum numbers defined in CMSIS header file. */
const IRQn_Type g_wdogIrqId[] = WDOG_IRQS;

static uint32_t wdogUnlockWindow;

/*******************************************************************************
 * Private Functions
 ******************************************************************************/
 wdog_status_t WDOG_DRV_Config(uint32_t instance, wdog_user_config_t wdogUserConfig);
 
 /*FUNCTION****************************************************************
 *
 * Function Name : WDOG_DRV_WaitUnlockWindowClose
 * Description   : Wait until the unlock window is closed
 * This function is used wait until the unlock window is closed (128 bus cycles).
 * This ensures that the watchdog's new configuration takes effect before the MCU enters low power mode,
 * if STOP or WAIT instructions are executed after reconfiguring the watchdog.
 *
 *END*********************************************************************/
 static void WDOG_DRV_WaitUnlockWindowClose(void)
{
    uint32_t count;

    for ( count = 0 ; count < wdogUnlockWindow; count++ )
    {
        __asm("nop");
    }
}


/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : WDOG_DRV_Init
 * Description   : initialize the WDOG driver
 *
 * Implements    : WDOG_DRV_Init_Activity
 *END**************************************************************************/
wdog_status_t WDOG_DRV_Init(uint32_t instance, 
                            const wdog_user_config_t *userConfigPtr)
{
    WDOG_Type *baseAddr;
    uint32_t coreClockHz, busClockHz;
    clock_manager_error_code_t errCode;
    wdog_status_t status;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(wdog_state_t != NULL);
    DEV_ASSERT(instance < WDOG_INSTANCE_COUNT);
#endif


    if (userConfigPtr == NULL)
    {
        return WDOG_STATUS_NULL_PARAM;
    }
    
    baseAddr = g_wdogBase[instance];
    
    /* enable WDOG timeout interrupt */
    INT_SYS_EnableIRQ(g_wdogIrqId[instance]);
    
    /* Compute instructions count of unlock window */
    errCode = CLOCK_SYS_GetFreq(CORE_CLOCK, &coreClockHz);
    if (errCode != CLOCK_MANAGER_SUCCESS)
    {
        return WDOG_STATUS_FAIL;
    }

    errCode = CLOCK_SYS_GetFreq(BUS_CLOCK, &busClockHz);
    if (errCode != CLOCK_MANAGER_SUCCESS)
    {
        return WDOG_STATUS_FAIL;
    }

    wdogUnlockWindow = ((coreClockHz/busClockHz) << 7); /* 128 bus clock */

    INT_SYS_DisableIRQGlobal();

    /* Initialize module */
    WDOG_HAL_Init(baseAddr);

    /* Configure module */
    status = WDOG_DRV_Config(instance, *userConfigPtr);
    if (status != WDOG_STATUS_SUCCESS)
    {
        return WDOG_STATUS_FAIL;
    }
    
    /* Enable WDOG */
    WDOG_HAL_Enable(baseAddr);
    WDOG_DRV_WaitUnlockWindowClose();

    INT_SYS_EnableIRQGlobal();

    return WDOG_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : WDOG_DRV_Deinit
 * Description   : deinitialize the WDOG driver
 *
 * Implements    : WDOG_DRV_Deinit_Activity
 *END**************************************************************************/
wdog_status_t WDOG_DRV_Deinit(uint32_t instance)
{
    WDOG_Type *baseAddr;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < WDOG_INSTANCE_COUNT);
#endif

    baseAddr = g_wdogBase[instance];

    INT_SYS_DisableIRQGlobal();
    
    /* Disable WDOG */
    WDOG_HAL_Disable(baseAddr);

    INT_SYS_EnableIRQGlobal();

    /* disable WDOG timeout interrupt */
    INT_SYS_DisableIRQ(g_wdogIrqId[instance]);

    return WDOG_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : WDOG_DRV_Config
 * Description   : config the WDOG driver
 *
 *END**************************************************************************/
 wdog_status_t WDOG_DRV_Config(uint32_t instance, wdog_user_config_t wdogUserConfig)
{
    WDOG_Type *baseAddr;
    
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < WDOG_INSTANCE_COUNT);
#endif

    baseAddr = g_wdogBase[instance];

    INT_SYS_DisableIRQGlobal();
    
    if (!WDOG_HAL_IsUpdateEnabled(baseAddr))
    {
        return WDOG_STATUS_FAIL;
    }
    
    WDOG_HAL_Config(baseAddr, &wdogUserConfig);

    WDOG_DRV_WaitUnlockWindowClose();
    
    INT_SYS_EnableIRQGlobal();

    return WDOG_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : WDOG_DRV_GetConfig
 * Description   : get the current configuration of the WDOG driver
 *
 * Implements    : WDOG_DRV_GetConfig_Activity
 *END**************************************************************************/
 wdog_status_t WDOG_DRV_GetConfig(uint32_t instance, wdog_user_config_t *config)
{
    WDOG_Type *baseAddr;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < WDOG_INSTANCE_COUNT);
#endif

    baseAddr = g_wdogBase[instance];

    *config = WDOG_HAL_GetConfig(baseAddr);

    return WDOG_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : WDOG_DRV_SetInt
 * Description   : enable/disable the WDOG timeout interrupt and  set handler
 *
 * Implements    : WDOG_DRV_SetInt_Activity
 *END**************************************************************************/
 wdog_status_t WDOG_DRV_SetInt(uint32_t instance, bool enable, void (*handler)(void))
{
    WDOG_Type *baseAddr;
    
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < WDOG_INSTANCE_COUNT);
#endif

    baseAddr = g_wdogBase[instance];

    if (!WDOG_HAL_IsUpdateEnabled(baseAddr))
    {
        return WDOG_STATUS_FAIL;
    }
    
    INT_SYS_InstallHandler(g_wdogIrqId[instance], handler, (isr_t*) 0);
    
    INT_SYS_DisableIRQGlobal();
    WDOG_HAL_SetInt(baseAddr, enable);
    WDOG_DRV_WaitUnlockWindowClose();
    INT_SYS_EnableIRQGlobal();
    
    return WDOG_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : WDOG_DRV_Trigger
 * Description   : reset the WDOG counter
 *
 * Implements    : WDOG_DRV_Trigger_Activity
 *END**************************************************************************/
 wdog_status_t WDOG_DRV_Trigger(uint32_t instance)
{
    WDOG_Type *baseAddr;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < WDOG_INSTANCE_COUNT);
#endif

    baseAddr = g_wdogBase[instance];

    WDOG_HAL_Trigger(baseAddr);

    return WDOG_STATUS_SUCCESS;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
