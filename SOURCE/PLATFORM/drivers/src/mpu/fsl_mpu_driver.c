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

#include "fsl_mpu_driver.h"
#include "fsl_clock_manager.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of base addresses for MPU instances. */
MPU_Type * const g_mpuBase[] = MPU_BASE_PTRS;

/*******************************************************************************
 * Code
 *******************************************************************************/
/*FUNCTION**********************************************************************
 *
 * Function Name : MPU_DRV_Init
 * Description   : Checks clock gate status of the module, sets the MPU regions according to user input, and then enables the MPU.
 * Please note that access rights for region 0 will always be configured and
 * regionCnt takes values between 1 and the maximum region count supported by the hardware.
 *  e.g. In S32K144 the number of supported regions is 8.
 *
 *END**************************************************************************/
mpu_status_t MPU_DRV_Init(uint32_t instance,
                          uint8_t regionCnt,
                          const mpu_user_config_t *userConfigArr)
{
    MPU_Type * base = g_mpuBase[instance];
    mpu_status_t retStatus = MPU_STATUS_SUCCESS;
    uint32_t     frequence;
    uint8_t i;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < MPU_INSTANCE_COUNT);
    DEV_ASSERT(userConfigArr);
    DEV_ASSERT((regionCnt > 0U) && (regionCnt <= MPU_RGD_COUNT));
#endif

    /* Get the MPU clock as configured in the clock manager */
    CLOCK_SYS_GetFreq(PCC_MPU0_CLOCK, &frequence);

    if(!frequence)
    {
        /* The MPU clock gate was not initialization */
        retStatus = MPU_STATUS_NOT_INITLIALIZED;
    }
    else
    {
        /* Initialize the MPU module */
        MPU_HAL_Init(base);

        /*
         * In region 0:
         *  Only access right for CORE, DMA can be changed
         *  access right for DEBUG, start address and end address ignored.
         */
        /* Sets Master Access Right for region 0 (CORE, DMA) and DEBUG ignore */
        for (i = 0U; i < FSL_FEATURE_MPU_MASTER_COUNT; i++)
        {
            MPU_DRV_SetMasterAccessRights(instance, 0U, userConfigArr[0U].masterAccRight + i);
        }

        /*
         * From region 1 to the next region
         */
        /* Initializes the regions, array index 1 is for region 1 and so on */
        for (i = 1U; i < regionCnt; i++)
        {
            retStatus = MPU_DRV_SetRegionConfig(instance, i, &userConfigArr[i]);
            if (retStatus != MPU_STATUS_SUCCESS)
            {
                break;
            }
        }

        if (MPU_STATUS_SUCCESS == retStatus)
        {
            /* Enables the MPU module operation */
            MPU_HAL_Enable(base);
        }
    }

    return retStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MPU_DRV_Deinit
 * Description   : De-initializes the MPU region by disabling MPU module.
 *
 *END**************************************************************************/
void MPU_DRV_Deinit(uint32_t instance)
{
    MPU_Type * base = g_mpuBase[instance];

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < MPU_INSTANCE_COUNT);
#endif

    /* Disable MPU module operation */
    MPU_HAL_Disable(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MPU_DRV_SetRegionAddr
 * Description   : Sets the region start and end address.
 *
 *END**************************************************************************/
void MPU_DRV_SetRegionAddr(uint32_t instance,
                           uint8_t regionNum,
                           uint32_t startAddr,
                           uint32_t endAddr)
{
    MPU_Type * base = g_mpuBase[instance];

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < MPU_INSTANCE_COUNT);
    DEV_ASSERT(regionNum < MPU_RGD_COUNT);
    DEV_ASSERT(startAddr <= endAddr);
#endif

    /* Changing a region's start and end addresses */
    MPU_HAL_SetRegionAddr(base, regionNum, startAddr, endAddr);

    /*  Re-enables the region descriptor valid bit */
    MPU_HAL_SetRegionValidCmd(base, regionNum, true);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MPU_DRV_SetRegionConfig
 * Description   : Sets the region configuration.
 *
 *END**************************************************************************/
mpu_status_t MPU_DRV_SetRegionConfig(uint32_t instance,
                                     uint8_t regionNum,
                                     const mpu_user_config_t *userConfigPtr)
{
    MPU_Type * base = g_mpuBase[instance];
    mpu_access_rights_t accRight;
    mpu_high_masters_access_rights_t highAccRight;
    mpu_low_masters_access_rights_t lowAccRight;
    mpu_status_t returnCode = MPU_STATUS_SUCCESS;
    uint8_t masterNum;
    uint8_t i;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < MPU_INSTANCE_COUNT);
    DEV_ASSERT(userConfigPtr);
    DEV_ASSERT((regionNum > 0U) && (regionNum < MPU_RGD_COUNT));
#endif

    /* Configures region start and end address */
    MPU_HAL_SetRegionAddr(base, regionNum, userConfigPtr->startAddr, userConfigPtr->endAddr);

    for (i = 0U; i < FSL_FEATURE_MPU_MASTER_COUNT; i++)
    {
        accRight = userConfigPtr->masterAccRight[i].accessRight;
        masterNum = userConfigPtr->masterAccRight[i].masterNum;
        /* Configure access rights for bus masters */
        if (masterNum > FSL_FEATURE_MPU_MAX_LOW_MASTER_NUMBER)
        {
            if (masterNum > FSL_FEATURE_MPU_MAX_HIGH_MASTER_NUMBER)
            {
                returnCode = MPU_STATUS_FAIL;
                break;
            }
            else
            {
                /* For masters which have only read and write permissions(e.g. master4~7 in S32K144). */
                highAccRight.writeEnable = (accRight & MPU_W_MASK) >> MPU_W_SHIFT;
                highAccRight.readEnable = (accRight & MPU_R_MASK) >> MPU_R_SHIFT;
                MPU_HAL_SetHighMasterAccessRights(base, regionNum, masterNum, &highAccRight);
            }
        }
        else
        {
            /* For masters which have separated privilege rights for user and supervisor mode accesses (e.g. master0~3 in S32K144) */
            lowAccRight.userAccessRights = (mpu_user_access_rights_t)((accRight & MPU_USER_MASK) >> MPU_USER_SHIFT);
            lowAccRight.superAccessRights = (mpu_supervisor_access_rights_t)((accRight & MPU_SUPERVISOR_MASK) >> MPU_SUPERVISOR_SHIFT);
        #if FSL_FEATURE_MPU_HAS_PROCESS_IDENTIFIER
            lowAccRight.processIdentifierEnable = userConfigPtr->masterAccRight[i].processIdentifierEnable;
        #endif
            MPU_HAL_SetLowMasterAccessRights(base, regionNum, masterNum, &lowAccRight);
        }
    }

#if FSL_FEATURE_MPU_HAS_PROCESS_IDENTIFIER
    /* Set Process Identifier */
    MPU_HAL_SetProcessIdentifier(base, regionNum, userConfigPtr->processIdentifier);

    /* Set Process Identifier mask */
    MPU_HAL_SetProcessIdentifierMask(base, regionNum, userConfigPtr->processIdMask);
#endif

    /* Validate this region */
    if (MPU_STATUS_SUCCESS == returnCode)
    {
        MPU_HAL_SetRegionValidCmd(base, regionNum, true);
    }

    return returnCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MPU_DRV_SetMasterAccessRights
 * Description   : Configures access permission.
 *
 *END**************************************************************************/
mpu_status_t MPU_DRV_SetMasterAccessRights(uint32_t instance,
                                           uint8_t regionNum,
                                           const mpu_master_access_right_t *accessRightsPtr)
{
    MPU_Type * base = g_mpuBase[instance];
    mpu_high_masters_access_rights_t highAccRight;
    mpu_low_masters_access_rights_t lowAccRight;
    mpu_access_rights_t  accRight;
    mpu_status_t returnCode = MPU_STATUS_SUCCESS;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < MPU_INSTANCE_COUNT);
    DEV_ASSERT(regionNum < MPU_RGD_COUNT);
    DEV_ASSERT(accessRightsPtr);
#endif

    accRight = accessRightsPtr->accessRight;
    if (accessRightsPtr->masterNum > FSL_FEATURE_MPU_MAX_LOW_MASTER_NUMBER)
    {
        if (accessRightsPtr->masterNum > FSL_FEATURE_MPU_MAX_HIGH_MASTER_NUMBER)
        {
            returnCode = MPU_STATUS_FAIL;
        }
        else
        {
            /* For masters which have only read and write permissions(e.g. master4~7 in S32K144). */
            highAccRight.writeEnable = (accRight & MPU_W_MASK) >> MPU_W_SHIFT;
            highAccRight.readEnable = (accRight & MPU_R_MASK) >> MPU_R_SHIFT;
            MPU_HAL_SetHighMasterAccessRightsByAlternateReg(base, regionNum, accessRightsPtr->masterNum, &highAccRight);
        }
    }
    else
    {
        /* For masters which have separated privilege rights for user and supervisor mode accesses (e.g. master0~3 in S32K144) */
        lowAccRight.userAccessRights = (mpu_user_access_rights_t)((accRight & MPU_USER_MASK) >> MPU_USER_SHIFT);
        lowAccRight.superAccessRights = (mpu_supervisor_access_rights_t)((accRight & MPU_SUPERVISOR_MASK) >> MPU_SUPERVISOR_SHIFT);
    #if FSL_FEATURE_MPU_HAS_PROCESS_IDENTIFIER
        lowAccRight.processIdentifierEnable = accessRightsPtr->processIdentifierEnable;
    #endif
        MPU_HAL_SetLowMasterAccessRightsByAlternateReg(base, regionNum, accessRightsPtr->masterNum, &lowAccRight);
    }

    return returnCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MPU_DRV_GetDetailErrorAccessInfo
 * Description   : Gets the MPU access error detail information for a slave port.
 *
 *END**************************************************************************/
void  MPU_DRV_GetDetailErrorAccessInfo(uint32_t instance,
                                       uint8_t slavePortNum,
                                       mpu_access_err_info_t *errInfoArrayPtr)
{
    MPU_Type * base = g_mpuBase[instance];

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < MPU_INSTANCE_COUNT);
    DEV_ASSERT(slavePortNum < FSL_FEATURE_MPU_SLAVE_COUNT);
    DEV_ASSERT(errInfoArrayPtr);
#endif
    /* Check if there is access violation in the slave port */
    if (MPU_HAL_GetSlavePortErrorStatus(base, slavePortNum))
    {
        /* Get the slave port detail error */
        MPU_HAL_GetDetailErrorAccessInfo(base, slavePortNum, errInfoArrayPtr);
    }
}

/*******************************************************************************
 * EOF
 *******************************************************************************/
