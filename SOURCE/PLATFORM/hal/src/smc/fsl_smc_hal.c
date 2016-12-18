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

#include "fsl_smc_hal.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* WFI is targeted at entering either standby, dormant or shutdown mode, where an
 * interrupt is required to wake-up the processor */
#define STANDBY() __asm volatile("wfi")

/*******************************************************************************
 * PROTOTYPES
 ******************************************************************************/

static bool SMC_HAL_WaitForStatChange(const SMC_Type* const baseAddr, const power_mode_stat_t mode, const uint32_t timeout);

/*! Timeout used for waiting to set new mode */
#define SMC_TIMEOUT 1000U

/*! @brief Power mode transition table
 *         Specifies valid power modes for transitioning to the next mode (in comment)
 */ 
static const uint16_t transitionTable[] =
{
    STAT_VLPR | STAT_HSRUN,         /* Next mode POWER_MODE_RUN        */
    STAT_RUN,                       /* Next mode POWER_MODE_WAIT       */
    STAT_RUN,                       /* Next mode POWER_MODE_STOP       */
    STAT_RUN,                       /* Next mode POWER_MODE_VLPR       */
    STAT_VLPR,                      /* Next mode POWER_MODE_VLPW       */
    STAT_RUN | STAT_VLPR,           /* Next mode POWER_MODE_VLPS       */
    STAT_RUN,                       /* Next mode POWER_MODE_HSRUN      */
};


/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : SMC_HAL_GetVersion
 * Description   : Get the version of the SMC module
 * This function will get the version of the SMC module.
 * function for more details.
 *
 *END**************************************************************************/
void SMC_HAL_GetVersion(const SMC_Type* const baseAddr, smc_version_info_t* const versionInfo)
{
    uint32_t regValue;

    regValue = baseAddr->VERID;
    regValue = (regValue & SMC_VERID_MAJOR_MASK) >> SMC_VERID_MAJOR_SHIFT;
    versionInfo->majorNumber = regValue;
    
    regValue = baseAddr->VERID;
    regValue = (regValue & SMC_VERID_MINOR_MASK) >> SMC_VERID_MINOR_SHIFT;
    versionInfo->minorNumber = regValue;

    regValue = baseAddr->VERID;
    regValue = (regValue & SMC_VERID_FEATURE_MASK) >> SMC_VERID_FEATURE_SHIFT;
    versionInfo->featureNumber = regValue;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SMC_HAL_SetPowerMode
 * Description   : Config the power mode
 * This function will configure the power mode control for any run, stop and
 * stop submode if needed. It will also configure the power options for specific
 * power mode. Application should follow the proper procedure to configure and 
 * switch power mode between the different run and stop mode. Refer to reference
 * manual for the proper procedure and supported power mode that can be configured
 * and switch between each other. Refert to smc_power_mode_config_t for required
 * parameters to configure the power mode and the supported options. Other options
 * may need to configure through the hal driver individaully. Refer to hal driver
 * header for details. 
 * 
 *END**************************************************************************/
smc_hal_error_code_t SMC_HAL_SetPowerMode(SMC_Type* const baseAddr, 
                                          const smc_power_mode_config_t* const powerModeConfig)
{
    smc_hal_error_code_t retCode;
    smc_stop_mode_t stopMode;
    power_modes_t powerModeName = powerModeConfig->powerModeName;

    /* Verify the power mode name*/
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(powerModeName < POWER_MODE_MAX);
#endif

    /* Check the current and the next power mode in the transitioning table */
    if (!(SMC_HAL_GetPowerModeStatus(baseAddr) & transitionTable[powerModeName]))
    {
        /* System mode controller (SMC) cannnot transition to the next mode */
        retCode = SMC_HAL_FAILED;
    }
    else
    {
        /* Branch based on power mode name*/
        switch (powerModeName)
        {
        case POWER_MODE_RUN:
            /* Set to RUN mode. */
            SMC_HAL_SetRunModeControl(baseAddr, SMC_RUN);
            /* Wait for stat change */
            if (!SMC_HAL_WaitForStatChange(baseAddr,STAT_RUN,SMC_TIMEOUT))
            {
                /* Timeout for power mode change expired. */
                retCode = SMC_HAL_TIMEOUT_MODE_CHANGE;
                break;
            }
            retCode = SMC_HAL_SUCCESS;
            break;

        case POWER_MODE_VLPR:
            /* "Very-Low-Power Modes" is allowed? */
            if (!SMC_HAL_GetProtectionMode(baseAddr, ALLOW_VLP))
            {
                retCode = SMC_HAL_NOT_ALLOWED_MODE;
                break;
            }

            /* Set power mode to VLPR*/
            SMC_HAL_SetRunModeControl(baseAddr, SMC_VLPR);
            /* Wait for stat change */
            if (!SMC_HAL_WaitForStatChange(baseAddr,STAT_VLPR,SMC_TIMEOUT))
            {
                /* Timeout for power mode change expired. */
                retCode = SMC_HAL_TIMEOUT_MODE_CHANGE;
                break;
            }
            retCode = SMC_HAL_SUCCESS;
            break;

#if FSL_FEATURE_SMC_HAS_HIGH_SPEED_RUN_MODE
        case POWER_MODE_HSRUN:
            /* "High Speed Run Mode" is allowed? */
            if (!SMC_HAL_GetProtectionMode(baseAddr, ALLOW_HSRUN))
            {
                retCode = SMC_HAL_NOT_ALLOWED_MODE;
                break;
            }

            /* Set power mode to HSRUN */
            SMC_HAL_SetRunModeControl(baseAddr, SMC_HSRUN);
            /* Wait for stat change */
            if (!SMC_HAL_WaitForStatChange(baseAddr,STAT_HSRUN,SMC_TIMEOUT))
            {
                /* Timeout for power mode change expired. */
                retCode = SMC_HAL_TIMEOUT_MODE_CHANGE;
                break;
            }
            retCode = SMC_HAL_SUCCESS;
            break;
#endif

        case POWER_MODE_WAIT:
            /* Fall-through */
        case POWER_MODE_VLPW:
            if (powerModeName == POWER_MODE_VLPW && !SMC_HAL_GetProtectionMode(baseAddr, ALLOW_VLP))
            {
                retCode = SMC_HAL_NOT_ALLOWED_MODE;
                break;
            }

            /* Clear the SLEEPDEEP bit to disable deep sleep mode - WAIT */
            FSL_SCB->SCR &= ~FSL_SCB_SCR_SLEEPDEEP_MASK;

            /* Cpu is going into sleep state */
            STANDBY();

            retCode = SMC_HAL_SUCCESS;
            break;
        case POWER_MODE_STOP:
            /* Fall-through */
        case POWER_MODE_VLPS:
            if (powerModeName == POWER_MODE_STOP)
            {
                stopMode = SMC_STOP;
                
#if FSL_FEATURE_SMC_HAS_PSTOPO
                if (powerModeConfig->pstopOption)
                {
                    SMC_HAL_SetPartialStopOption(baseAddr, powerModeConfig->pstopOptionValue);
                }
#endif
            }
            else
            {
                stopMode = SMC_VLPS;
                
                /* "Very-Low-Power Modes" is allowed? */
                if (!SMC_HAL_GetProtectionMode(baseAddr, ALLOW_VLP))
                {
                    retCode = SMC_HAL_NOT_ALLOWED_MODE;
                    break;
                }
            }

            /* Set power mode to specified STOP mode*/
            SMC_HAL_SetStopModeControl(baseAddr, stopMode);

            /* Set the SLEEPDEEP bit to enable deep sleep mode (STOP)*/
            FSL_SCB->SCR |= FSL_SCB_SCR_SLEEPDEEP_MASK;

            /* Cpu is going into deep sleep state */
            STANDBY();

            retCode = SMC_HAL_SUCCESS;
            break;
        default:
            retCode = SMC_HAL_NO_SUCH_MODE_NAME;
            break;
        }
    }
    return retCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SMC_HAL_SetProtectionMode
 * Description   : Config all power mode protection settings
 * This function will configure the power mode protection settings for
 * supported power mode on the specified chip family. The availabe power modes
 * are defined in smc_power_mode_protection_config_t. Application should provide
 * the protect settings for all supported power mode on the chip and aslo this
 * should be done at early system level init stage. Refer to reference manual
 * for details. This register can only write once after power reset. So either
 * use this function or use the individual set function if you only have single
 * option to set.
 * 
 *END**************************************************************************/
void SMC_HAL_SetProtectionMode(SMC_Type* const baseAddr, 
                               const smc_power_mode_protection_config_t* const protectConfig)
{
    /* Initialize the setting */
    uint8_t regValue = 0U;

    /* Check configurations for each mode and combine the setting together */
    if (protectConfig->vlpProt)
    {
        regValue |= SMC_PMPROT_AVLP(1);
    }

#if FSL_FEATURE_SMC_HAS_HIGH_SPEED_RUN_MODE 
    if (protectConfig->hsrunProt)
    {
        regValue |= SMC_PMPROT_AHSRUN(1);
    }
#endif

    /* Write once into PMPROT register*/
    baseAddr->PMPROT = regValue;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SMC_HAL_GetProtectionMode
 * Description   : Get the current power mode protection setting
 * This function will get the current power mode protection settings for
 * a specified power mode.
 * 
 *
 *END**************************************************************************/
bool SMC_HAL_GetProtectionMode(const SMC_Type* const baseAddr, const power_modes_protect_t protect)
{
    bool retValue;

    /* Check the mode range */
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(protect < ALLOW_MAX);
#endif

    /* Branch according to the mode and read the setting */
    switch (protect)
    {
        case ALLOW_VLP:
            retValue = BITBAND_ACCESS32(&(baseAddr->PMPROT), SMC_PMPROT_AVLP_SHIFT);
            break;
#if FSL_FEATURE_SMC_HAS_HIGH_SPEED_RUN_MODE
        case ALLOW_HSRUN:
            retValue = BITBAND_ACCESS32(&(baseAddr->PMPROT), SMC_PMPROT_AHSRUN_SHIFT);
            break;
#endif
        default:
            /* Invalid command */
            retValue = false;
            break;
    }
    return retValue;
}


/*FUNCTION**********************************************************************
 * Function Name : SMC_HAL_WaitForStatChange
 * Description   : Internal function used by SMC_HAL_SetPowerMode function 
 * to wait until the state is changed or timeout expires
 *
 * return power mode status change
 *                - true: power mode has been changed successfully
 *                - false: timeout expired, power mode has not been changed
 *END**************************************************************************/
static bool SMC_HAL_WaitForStatChange(const SMC_Type* const baseAddr, const power_mode_stat_t mode, const uint32_t timeout)
{
    uint32_t i;
    bool retValue;
        
    for (i = 0U; i < timeout; i++)
    {
        if (mode == SMC_HAL_GetPowerModeStatus(baseAddr))
        {
            /* Power mode has been changed successfully */
            break;
        }
    }

    /* If i greater or equal to timeout, then timeout expired (the power mode has not been changed)*/
    retValue = (i < timeout);

    return retValue;
}



/*******************************************************************************
 * EOF
 ******************************************************************************/

