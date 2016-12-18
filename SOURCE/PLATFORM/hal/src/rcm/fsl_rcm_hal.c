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

#include "fsl_rcm_hal.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : RCM_HAL_GetVersion
 * Description   : Get the version of the RCM module
 * This function will get the version of the RCM module.
 * function for more details.
 *
 *END**************************************************************************/
void  RCM_HAL_GetVersion(const RCM_Type* const baseAddr, rcm_version_info_t* const versionInfo)
{ 
  uint32_t regValue;

  regValue = baseAddr->VERID;
  regValue = (regValue & RCM_VERID_MAJOR_MASK) >> RCM_VERID_MAJOR_SHIFT;
  versionInfo->majorNumber = regValue;

  regValue = baseAddr->VERID;
  regValue = (regValue & RCM_VERID_MINOR_MASK) >> RCM_VERID_MINOR_SHIFT;  
  versionInfo->minorNumber = regValue;

  regValue = baseAddr->VERID;
  regValue = (regValue & RCM_VERID_FEATURE_MASK) >> RCM_VERID_FEATURE_SHIFT;
  versionInfo->featureNumber = regValue;
}
 
/*FUNCTION**********************************************************************
 *
 * Function Name : RCM_HAL_GetSrcStatusCmd
 * Description   : Get the reset source status
 * 
 * This function will get the current reset source status for specified source
 *
 *END**************************************************************************/
bool RCM_HAL_GetSrcStatusCmd(const RCM_Type* const baseAddr, const rcm_source_names_t srcName)
{
    bool retValue;

    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(srcName < RCM_SRC_NAME_MAX);
    #endif

    switch (srcName)
    {
    case RCM_LOW_VOLT_DETECT:              /* low voltage detect reset */
        retValue = (bool)BITBAND_ACCESS32(&(baseAddr->SRS), RCM_SRS_LVD_SHIFT);
        break;
    case RCM_LOSS_OF_CLK:                  /* loss of clock reset */
        retValue = (bool)BITBAND_ACCESS32(&(baseAddr->SRS), RCM_SRS_LOC_SHIFT);
        break;
    case RCM_LOSS_OF_LOCK:                 /* loss of lock reset */
        retValue = (bool)BITBAND_ACCESS32(&(baseAddr->SRS), RCM_SRS_LOL_SHIFT);
        break;
    case RCM_WATCH_DOG:                    /* watch dog reset */
        retValue = (bool)BITBAND_ACCESS32(&(baseAddr->SRS), RCM_SRS_WDOG_SHIFT);
        break;
    case RCM_EXTERNAL_PIN:                 /* external pin reset */
        retValue = (bool)BITBAND_ACCESS32(&(baseAddr->SRS), RCM_SRS_PIN_SHIFT);
        break;
    case RCM_POWER_ON:                     /* power on reset */
        retValue = (bool)BITBAND_ACCESS32(&(baseAddr->SRS), RCM_SRS_POR_SHIFT);
        break;
    case RCM_SJTAG:                        /* JTAG generated reset */
        retValue = (bool)BITBAND_ACCESS32(&(baseAddr->SRS), RCM_SSRS_SJTAG_SHIFT);
        break;
    case RCM_CORE_LOCKUP:                  /* core lockup reset */
        retValue = (bool)BITBAND_ACCESS32(&(baseAddr->SRS), RCM_SRS_LOCKUP_SHIFT);
        break;
    case RCM_SOFTWARE:                     /* software reset */
        retValue = (bool)BITBAND_ACCESS32(&(baseAddr->SRS), RCM_SRS_SW_SHIFT);
        break;
    case RCM_SMDM_AP:                      /* MDM-AP system reset */
        retValue = (bool)BITBAND_ACCESS32(&(baseAddr->SRS), RCM_SSRS_SMDM_AP_SHIFT);
        break;
    case RCM_STOP_MODE_ACK_ERR:            /* stop mode ack error reset */
        retValue = (bool)BITBAND_ACCESS32(&(baseAddr->SRS), RCM_SRS_SACKERR_SHIFT);
        break;
    default:
        retValue = false;
        break;
    }

    return retValue;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : RCM_HAL_SetResetIntCmd
 * Description   : Enables/disables a specified system reset interrupt.
 * 
 * This function will enable/disable the specified system reset interrupt
 *
 *END**************************************************************************/
void RCM_HAL_SetResetIntCmd(RCM_Type* const baseAddr, const rcm_source_names_t resetInterrupt, const bool enable)
{
    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(interrupt < RCM_SRC_NAME_MAX);
    #endif

    switch (resetInterrupt)
    {
    case RCM_LOW_VOLT_DETECT:            /* low-voltage detect reset */
        BITBAND_ACCESS32(&(baseAddr->SRIE), RCM_SSRS_SLVD_SHIFT) = enable;
        break;
    case RCM_LOSS_OF_CLK:                  /* loss of clock reset */
        BITBAND_ACCESS32(&(baseAddr->SRIE), RCM_SRIE_LOC_SHIFT) = enable;
        break;
    case RCM_LOSS_OF_LOCK:                 /* loss of lock reset */
        BITBAND_ACCESS32(&(baseAddr->SRIE), RCM_SRIE_LOL_SHIFT) = enable;
        break;
    case RCM_WATCH_DOG:                   /* watch dog reset */
        BITBAND_ACCESS32(&(baseAddr->SRIE), RCM_SRIE_WDOG_SHIFT) = enable;
        break;
    case RCM_EXTERNAL_PIN:                /* external pin reset */
        BITBAND_ACCESS32(&(baseAddr->SRIE), RCM_SRIE_PIN_SHIFT) = enable;
        break;
    case RCM_SJTAG:                /* JTAG generated reset */
        BITBAND_ACCESS32(&(baseAddr->SRIE), RCM_SRIE_JTAG_SHIFT) = enable;
        break;
    case RCM_CORE_LOCKUP:                 /* core lockup reset */
        BITBAND_ACCESS32(&(baseAddr->SRIE), RCM_SRIE_LOCKUP_SHIFT) = enable;
        break;
    case RCM_SOFTWARE:                   /* software reset */
        BITBAND_ACCESS32(&(baseAddr->SRIE), RCM_SRIE_SW_SHIFT) = enable;
        break;
    case RCM_SMDM_AP:                /* MDM-AP system reset */
        BITBAND_ACCESS32(&(baseAddr->SRIE), RCM_SRIE_MDM_AP_SHIFT) = enable;
        break;
    case RCM_STOP_MODE_ACK_ERR:          /* stop mode ack error reset */
        BITBAND_ACCESS32(&(baseAddr->SRIE), RCM_SRIE_SACKERR_SHIFT) = enable;
        break;
    default:
        /* invalid command */
        break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : RCM_HAL_GetStickySrcStatusCmd
 * Description   : Get the sticky reset source status
 *
 * This function gets the current reset source status that have not been cleared
 * by software for a specified source.
 *
 *END**************************************************************************/
bool RCM_HAL_GetStickySrcStatusCmd(const RCM_Type* const baseAddr, const rcm_source_names_t srcName)
{
    bool retValue;

    #ifdef DEV_ERROR_DETECT
    DEV_ASSERT(srcName < RCM_SRC_NAME_MAX);
    #endif


    switch (srcName)
    {
    case RCM_LOW_VOLT_DETECT:             /* low voltage detect reset */
        retValue = (bool)BITBAND_ACCESS32(&(baseAddr->SSRS), RCM_SSRS_SLVD_SHIFT);
        break;
    case RCM_LOSS_OF_CLK:                 /* loss of clock reset */
        retValue = (bool)BITBAND_ACCESS32(&(baseAddr->SSRS), RCM_SSRS_SLOC_SHIFT);
        break;
    case RCM_LOSS_OF_LOCK:                /* loss of lock reset */
        retValue = (bool)BITBAND_ACCESS32(&(baseAddr->SSRS), RCM_SSRS_SLOL_SHIFT);
        break;
    case RCM_WATCH_DOG:                   /* watch dog reset */
        retValue = (bool)BITBAND_ACCESS32(&(baseAddr->SSRS), RCM_SSRS_SWDOG_SHIFT);
        break;
    case RCM_EXTERNAL_PIN:                /* external pin reset */
        retValue = (bool)BITBAND_ACCESS32(&(baseAddr->SSRS), RCM_SSRS_SPIN_SHIFT);
        break;
    case RCM_POWER_ON:                    /* power on reset */
        retValue = (bool)BITBAND_ACCESS32(&(baseAddr->SSRS), RCM_SSRS_SPOR_SHIFT);
        break;
    case RCM_SJTAG:                       /* JTAG generated reset */
        retValue = (bool)BITBAND_ACCESS32(&(baseAddr->SSRS), RCM_SSRS_SJTAG_SHIFT);
        break;
    case RCM_CORE_LOCKUP:                 /* core lockup reset */
        retValue = (bool)BITBAND_ACCESS32(&(baseAddr->SSRS), RCM_SSRS_SLOCKUP_SHIFT);
        break;
    case RCM_SOFTWARE:                    /* software reset */
        retValue = (bool)BITBAND_ACCESS32(&(baseAddr->SSRS), RCM_SSRS_SSW_SHIFT);
        break;
    case RCM_SMDM_AP:                     /* MDM-AP system reset */
        retValue = (bool)BITBAND_ACCESS32(&(baseAddr->SSRS), RCM_SSRS_SMDM_AP_SHIFT);
        break;
    case RCM_STOP_MODE_ACK_ERR:           /* stop mode ack error reset */
        retValue = (bool)BITBAND_ACCESS32(&(baseAddr->SSRS), RCM_SSRS_SSACKERR_SHIFT);
        break;
    default:
        retValue = false;
        break;
    }

    return retValue;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : RCM_HAL_ClearStickySrcStatus
 * Description   : Clear the sticky reset source status
 *
 * This function clears all the sticky system reset flags.
 *
 *END**************************************************************************/
void RCM_HAL_ClearStickySrcStatus(RCM_Type* const baseAddr)
{
    uint8_t status;

    status = baseAddr->SSRS;
    baseAddr->SSRS = status;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
