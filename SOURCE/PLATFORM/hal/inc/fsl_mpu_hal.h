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
#ifndef FSL_MPU_HAL_H_
#define FSL_MPU_HAL_H_

#include <stdint.h>
#include <stdbool.h>
#include "fsl_device_registers.h"

/*!
 * @addtogroup mpu_hal
 * @{
 */

/*******************************************************************************
 * Definitions
 *******************************************************************************/
/*! @brief MPU access error. */
typedef enum _mpu_err_access_type {
    MPU_ERR_TYPE_READ  = 0U,     /*!< MPU error type---read  */
    MPU_ERR_TYPE_WRITE = 1U      /*!< MPU error type---write */
} mpu_err_access_type_t;

/*! @brief MPU access error attributes. */
typedef enum _mpu_err_attributes {
    MPU_INSTRUCTION_ACCESS_IN_USER_MODE       = 0U,  /*!< Access instruction error in user mode       */
    MPU_DATA_ACCESS_IN_USER_MODE              = 1U,  /*!< Access data error in user mode              */
    MPU_INSTRUCTION_ACCESS_IN_SUPERVISOR_MODE = 2U,  /*!< Access instruction error in supervisor mode */
    MPU_DATA_ACCESS_IN_SUPERVISOR_MODE        = 3U   /*!< Access data error in supervisor mode        */
} mpu_err_attributes_t;

/*! @brief MPU access modes. */
typedef enum _mpu_access_mode {
    MPU_ACCESS_IN_USER_MODE       = 0U, /*!< Access data or instruction in user mode*/
    MPU_ACCESS_IN_SUPERVISOR_MODE = 1U  /*!< Access data or instruction in supervisor mode*/
} mpu_access_mode_t;

/*! @brief MPU access rights in supervisor mode. */
typedef enum _mpu_supervisor_access_rights {
    MPU_SUPERVISOR_READ_WRITE_EXECUTE = 0U,   /*!< Read write and execute operations are allowed in supervisor mode */
    MPU_SUPERVISOR_READ_EXECUTE       = 1U,   /*!< Read and execute operations are allowed in supervisor mode       */
    MPU_SUPERVISOR_READ_WRITE         = 2U,   /*!< Read write operations are allowed in supervisor mode             */
    MPU_SUPERVISOR_EQUAL_TO_USERMODE  = 3U    /*!< Access permission equal to user mode                             */
} mpu_supervisor_access_rights_t;

/*! @brief MPU access rights in user mode. */
typedef enum _mpu_user_access_rights {
    MPU_USER_NO_ACCESS_RIGHTS   = 0U,   /*!< No access allowed in user mode                             */
    MPU_USER_EXECUTE            = 1U,   /*!< Execute operation is allowed in user mode                  */
    MPU_USER_WRITE              = 2U,   /*!< Write operation is allowed in user mode                    */
    MPU_USER_WRITE_EXECUTE      = 3U,   /*!< Write and execute operations are allowed in user mode      */
    MPU_USER_READ               = 4U,   /*!< Read is allowed in user mode                               */
    MPU_USER_READ_EXECUTE       = 5U,   /*!< Read and execute operations are allowed in user mode       */
    MPU_USER_READ_WRITE         = 6U,   /*!< Read and write operations are allowed in user mode         */
    MPU_USER_READ_WRITE_EXECUTE = 7U    /*!< Read write and execute operations are allowed in user mode */
} mpu_user_access_rights_t;

/*! @brief MPU detail error access info. */
typedef struct _mpu_access_err_info {
    uint8_t                master;                    /*!< Access error master                   */
    mpu_err_attributes_t   attributes;                /*!< Access error attributes               */
    mpu_err_access_type_t  accessType;                /*!< Access error type                     */
    uint16_t               accessCtr;                 /*!< Access error control                  */
    uint32_t               addr;                      /*!< Access error address                  */
#if FSL_FEATURE_MPU_HAS_PROCESS_IDENTIFIER
    uint8_t                processorIdentification;   /*!< Access error processor identification */
#endif
} mpu_access_err_info_t;

/*! @brief MPU access rights for masters which have separated privilege rights for user and
   supervisor mode accesses (e.g. master0~3 in S32K144). */
typedef struct _mpu_low_masters_access_rights {
    mpu_user_access_rights_t       userAccessRights;          /*!< Master access rights in user mode       */
    mpu_supervisor_access_rights_t superAccessRights;         /*!< Master access rights in supervisor mode */
#if FSL_FEATURE_MPU_HAS_PROCESS_IDENTIFIER
    bool                           processIdentifierEnable;   /*!< Enables or disables process identifier  */
#endif
} mpu_low_masters_access_rights_t;

/*! @brief MPU access rights for master which have only read and write permissions(e.g. master4~7 in S32K144). */
typedef struct _mpu_high_masters_access_rights {
    bool writeEnable;    /*!< Enables or disables write permission */
    bool readEnable;     /*!< Enables or disables read permission  */
} mpu_high_masters_access_rights_t;

/*******************************************************************************
 * API
 *******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Enables the MPU module.
 *
 * @param base The MPU peripheral base address.
 */
static inline void MPU_HAL_Enable(MPU_Type * base)
{
    BITBAND_ACCESS32(&base->CESR, MPU_CESR_VLD_SHIFT) = (uint32_t)1U;
}

/*!
 * @brief Disables the MPU module.
 *
 * @param base The MPU peripheral base address.
 */
static inline void MPU_HAL_Disable(MPU_Type * base)
{
    BITBAND_ACCESS32(&base->CESR, MPU_CESR_VLD_SHIFT) = (uint32_t)0U;
}

/*!
 * @brief Checks whether the MPU module is enabled
 *
 * @param base The MPU peripheral base address.
 * @return State of the module
 *         true  -  MPU module is enabled.
 *         false -  MPU module is disabled.
 */
static inline bool MPU_HAL_IsEnable(MPU_Type * base)
{
    return (bool)BITBAND_ACCESS32(&base->CESR, MPU_CESR_VLD_SHIFT);
}

/*!
 * @brief Gets MPU hardware revision level.
 *
 * @param base The MPU peripheral base address.
 * @return Hardware revision level
 */
static inline uint8_t MPU_HAL_GetHardwareRevision(MPU_Type *base)
{
    return (uint8_t)((base->CESR & MPU_CESR_HRL_MASK) >> MPU_CESR_HRL_SHIFT);
}

/*!
 * @brief Gets the error status of a specified slave port.
 *
 * @param base       The MPU peripheral base address.
 * @param slaveNum   MPU slave port number.
 * @return The slave ports error status.
 *         true  - error happens in this slave port.
 *         false - error didn't happen in this slave port.
 */
bool MPU_HAL_GetSlavePortErrorStatus(MPU_Type *base,
                                     uint8_t slaveNum);

/*!
 * @brief Gets MPU detail error access info.
 *
 * @param base The MPU peripheral base address.
 * @param slaveNum MPU slave port number.
 * @param errInfoPtr The pointer to the MPU access error information.
 */
void MPU_HAL_GetDetailErrorAccessInfo(MPU_Type *base,
                                      uint8_t slaveNum,
                                      mpu_access_err_info_t *errInfoPtr);

/*!
 * @brief Sets region start and end address.
 *   Please note that using this function will clear the valid bit of the region,
 *   and a further validation might be needed.
 *
 * @param base The MPU peripheral base address.
 * @param regionNum MPU region number.
 * @param startAddr Region start address.
 * @param endAddr Region end address.
 */
void MPU_HAL_SetRegionAddr(MPU_Type * base,
                           uint8_t regionNum,
                           uint32_t startAddr,
                           uint32_t endAddr);

/*!
 * @brief Sets access permission for master which has separated privilege rights for user and
 *  supervisor mode accesses in a specific region.
 *  Please note that using this function will clear the valid bit of the region.
 *  In order to keep the region valid,
 *  the MPU_HAL_SetLowMasterAccessRightsByAlternateReg function can be used.
 *
 * @param base The MPU peripheral base address.
 * @param regionNum MPU region number.
 * @param masterNum MPU master number.
 * @param accessRightsPtr The pointer of master access rights see #mpu_low_masters_access_rights_t.
 */
void MPU_HAL_SetLowMasterAccessRights(MPU_Type * base,
                                      uint8_t regionNum,
                                      uint8_t masterNum,
                                      const mpu_low_masters_access_rights_t *accessRightsPtr);

/*!
 * @brief Sets access permission for master which has only read and write permissions in a specific region.
 *  Please note that using this function will clear the valid bit of the region.
 *  In order to keep the region valid,
 *  the MPU_HAL_SetHighMasterAccessRightsByAlternateReg function can be used.
 *
 * @param base The MPU peripheral base address.
 * @param regionNum MPU region number.
 * @param masterNum MPU master number.
 * @param accessRightsPtr The pointer of master access rights see #mpu_high_masters_access_rights_t.
 */
void MPU_HAL_SetHighMasterAccessRights(MPU_Type * base,
                                       uint8_t regionNum,
                                       uint8_t masterNum,
                                       const mpu_high_masters_access_rights_t *accessRightsPtr);

/*!
 * @brief Sets the region valid value.
 * When a region changed not by alternating registers should set the valid again.
 *
 * @param base The MPU peripheral base address.
 * @param regionNum MPU region number.
 * @param enable Enables or disables region.
 */
static inline void MPU_HAL_SetRegionValidCmd(MPU_Type * base,
                                             uint8_t regionNum,
                                             bool enable)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(regionNum < MPU_RGD_COUNT);
#endif
    BITBAND_ACCESS32(&base->RGD[regionNum].WORD3, MPU_WORD3_VLD_SHIFT) = (uint32_t)(enable);
}

#if FSL_FEATURE_MPU_HAS_PROCESS_IDENTIFIER
/*!
 * @brief Sets the process identifier mask.
 *
 * @param base The MPU peripheral base address.
 * @param regionNum MPU region number.
 * @param processIdentifierMask Process identifier mask value.
 */
static inline void MPU_HAL_SetProcessIdentifierMask(MPU_Type * base,
                                                    uint8_t regionNum,
                                                    uint8_t processIdentifierMask)
{
    uint32_t temp;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(regionNum < MPU_RGD_COUNT);
#endif

    temp = base->RGD[regionNum].WORD3;
    temp &= ~(MPU_WORD3_PIDMASK_MASK);
    temp |= MPU_WORD3_PIDMASK(processIdentifierMask);
    base->RGD[regionNum].WORD3 = temp;
}

/*!
 * @brief Sets the process identifier.
 *
 * @param base The MPU peripheral base address.
 * @param regionNum MPU region number.
 * @param processIdentifier Process identifier.
 */
static inline void MPU_HAL_SetProcessIdentifier(MPU_Type * base,
                                                uint8_t regionNum,
                                                uint8_t processIdentifier)
{
    uint32_t temp;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(regionNum < MPU_RGD_COUNT);
#endif

    temp = base->RGD[regionNum].WORD3;
    temp &= ~(MPU_WORD3_PID_MASK);
    temp |= MPU_WORD3_PID(processIdentifier);
    base->RGD[regionNum].WORD3 = temp;
}
#endif

/*!
 * @brief Sets access permission for master which has separated privilege rights for user and
 *  supervisor mode accesses in a specific region  by alternate register.
 *
 * @param base The MPU peripheral base address.
 * @param regionNum MPU region number.
 * @param masterNum MPU master number.
 * @param accessRightsPtr The pointer of master access rights see #mpu_low_masters_access_rights_t.
 */
void MPU_HAL_SetLowMasterAccessRightsByAlternateReg(MPU_Type * base,
                                                    uint8_t regionNum,
                                                    uint8_t masterNum,
                                                    const mpu_low_masters_access_rights_t *accessRightsPtr);

/*!
 * @brief Sets access permission for master which has only read and write permissions in a specific region by alternate register.
 *
 * @param base The MPU peripheral base address.
 * @param regionNum MPU region number.
 * @param masterNum MPU master number.
 * @param accessRightsPtr The pointer of master access rights see #mpu_high_masters_access_rights_t.
 */
void MPU_HAL_SetHighMasterAccessRightsByAlternateReg(MPU_Type * base,
                                                     uint8_t regionNum,
                                                     uint8_t masterNum,
                                                     const mpu_high_masters_access_rights_t *accessRightsPtr);

/*!
 * @brief Initializes the MPU module and all regions will be invalid after cleared access permission.
 *
 * @param base The MPU peripheral base address.
 */
void MPU_HAL_Init(MPU_Type * base);

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* FSL_MPU_HAL_H_*/
/*******************************************************************************
 * EOF
 *******************************************************************************/
