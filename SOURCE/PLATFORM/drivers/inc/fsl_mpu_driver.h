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
#ifndef FSL_MPU_DRIVER_H_
#define FSL_MPU_DRIVER_H_

#include "fsl_mpu_hal.h"

/*******************************************************************************
 * Definitions
 *******************************************************************************/
#define MPU_USER_MASK               (0x07U)
#define MPU_USER_SHIFT              (0U)
#define MPU_SUPERVISOR_MASK         (0x18U)
#define MPU_SUPERVISOR_SHIFT        (3U)
#define MPU_W_MASK                  (0x20U)
#define MPU_W_SHIFT                 (5U)
#define MPU_R_MASK                  (0x40U)
#define MPU_R_SHIFT                 (6U)

/*!
 * @addtogroup mpu_driver
 * @{
 */

/*! @brief MPU access rights:
 * |      Code                     |  Supervisor  |     User    |  Description                                                                       |
 * |-------------------------------|--------------|-------------|------------------------------------------------------------------------------------|
 * |  MPU_SUPERVISOR_RWX_USER_NONE |    r w x     |   - - -     | Allow Read, write, execute in supervisor mode; no access in user mode              |
 * |  MPU_SUPERVISOR_RWX_USER_X    |    r w x     |   - - x     | Allow Read, write, execute in supervisor mode; execute in user mode                |
 * |  MPU_SUPERVISOR_RWX_USER_W    |    r w x     |   - w -     | Allow Read, write, execute in supervisor mode; write in user mode                  |
 * |  MPU_SUPERVISOR_RWX_USER_WX   |    r w x     |   - w x     | Allow Read, write, execute in supervisor mode; write and execute in user mode      |
 * |  MPU_SUPERVISOR_RWX_USER_R    |    r w x     |   r - -     | Allow Read, write, execute in supervisor mode; read in user mode                   |
 * |  MPU_SUPERVISOR_RWX_USER_RX   |    r w x     |   r - x     | Allow Read, write, execute in supervisor mode; read and execute in user mode       |
 * |  MPU_SUPERVISOR_RWX_USER_RW   |    r w x     |   r w -     | Allow Read, write, execute in supervisor mode; read and write in user mode         |
 * |  MPU_SUPERVISOR_RWX_USER_RWX  |    r w x     |   r w x     | Allow Read, write, execute in supervisor mode; read, write and execute in user mode|
 * |  MPU_SUPERVISOR_RX_USER_NONE  |    r - x     |   - - -     | Allow Read, execute in supervisor mode; no access in user mode                     |
 * |  MPU_SUPERVISOR_RX_USER_X     |    r - x     |   - - x     | Allow Read, execute in supervisor mode; execute in user mode                       |
 * |  MPU_SUPERVISOR_RX_USER_W     |    r - x     |   - w -     | Allow Read, execute in supervisor mode; write in user mode                         |
 * |  MPU_SUPERVISOR_RX_USER_WX    |    r - x     |   - w x     | Allow Read, execute in supervisor mode; write and execute in user mode             |
 * |  MPU_SUPERVISOR_RX_USER_R     |    r - x     |   r - -     | Allow Read, execute in supervisor mode; read in user mode                          |
 * |  MPU_SUPERVISOR_RX_USER_RX    |    r - x     |   r - x     | Allow Read, execute in supervisor mode; read and execute in user mode              |
 * |  MPU_SUPERVISOR_RX_USER_RW    |    r - x     |   r w -     | Allow Read, execute in supervisor mode; read and write in user mode                |
 * |  MPU_SUPERVISOR_RX_USER_RWX   |    r - x     |   r w x     | Allow Read, execute in supervisor mode; read, write and execute in user mode       |
 * |  MPU_SUPERVISOR_RW_USER_NONE  |    r w -     |   - - -     | Allow Read, write in supervisor mode; no access in user mode                       |
 * |  MPU_SUPERVISOR_RW_USER_X     |    r w -     |   - - x     | Allow Read, write in supervisor mode; execute in user mode                         |
 * |  MPU_SUPERVISOR_RW_USER_W     |    r w -     |   - w -     | Allow Read, write in supervisor mode; write in user mode                           |
 * |  MPU_SUPERVISOR_RW_USER_WX    |    r w -     |   - w x     | Allow Read, write in supervisor mode; write and execute in user mode               |
 * |  MPU_SUPERVISOR_RW_USER_R     |    r w -     |   r - -     | Allow Read, write in supervisor mode; read in user mode                            |
 * |  MPU_SUPERVISOR_RW_USER_RX    |    r w -     |   r - x     | Allow Read, write in supervisor mode; read and execute in user mode                |
 * |  MPU_SUPERVISOR_RW_USER_RW    |    r w -     |   r w -     | Allow Read, write in supervisor mode; read and write in user mode                  |
 * |  MPU_SUPERVISOR_RW_USER_RWX   |    r w -     |   r w x     | Allow Read, write in supervisor mode; read, write and execute in user mode         |
 * |  MPU_SUPERVISOR_USER_NONE     |    - - -     |   - - -     | No access allowed in user and supervisor modes                                     |
 * |  MPU_SUPERVISOR_USER_X        |    - - x     |   - - x     | Execute operation is allowed in user and supervisor modes                          |
 * |  MPU_SUPERVISOR_USER_W        |    - w -     |   - w -     | Write operation is allowed in user and supervisor modes                            |
 * |  MPU_SUPERVISOR_USER_WX       |    - w x     |   - w x     | Write and execute operations are allowed in user and supervisor modes              |
 * |  MPU_SUPERVISOR_USER_R        |    r - -     |   r - -     | Read is allowed in user and supervisor modes                                       |
 * |  MPU_SUPERVISOR_USER_RX       |    r - x     |   r - x     | Read and execute operations are allowed in user and supervisor modes               |
 * |  MPU_SUPERVISOR_USER_RW       |    r w -     |   r w -     | Read and write operations are allowed in user and supervisor modes                 |
 * |  MPU_SUPERVISOR_USER_RWX      |    r w x     |   r w x     | Read write and execute operations are allowed in user and supervisor modes         |
 *
 *
 * |      Code                     | Read/Write permission   | Description                       |
 * |-------------------------------|-------------------------|-----------------------------------|
 * |  MPU_NONE                     |          - -            | No Read/Write access permission   |
 * |  MPU_W                        |          - w            | Write access permission           |
 * |  MPU_R                        |          r -            | Read access permission            |
 * |  MPU_RW                       |          r w            | Read/Write access permission      |
 */
typedef enum _mpu_access_rights {
    /* Format: M_R_W_SS_UUU
     * M  : 1 bit  - Specify that access right is for masters which have separated
     * privilege rights for user and supervisor mode accesses (e.g. master0~3)
     * R  : 1 bit  - Read access permission
     * W  : 1 bit  - Write access permission
     * SS : 2 bits - Supervisor Mode Access Control
     * UUU: 3 bits - User Mode Access Control
     */
    MPU_SUPERVISOR_RWX_USER_NONE    = 0x00U,  /*!< 0b00000000U : rwx|--- */
    MPU_SUPERVISOR_RWX_USER_X       = 0x01U,  /*!< 0b00000001U : rwx|--x */
    MPU_SUPERVISOR_RWX_USER_W       = 0x02U,  /*!< 0b00000010U : rwx|-w- */
    MPU_SUPERVISOR_RWX_USER_WX      = 0x03U,  /*!< 0b00000011U : rwx|-wx */
    MPU_SUPERVISOR_RWX_USER_R       = 0x04U,  /*!< 0b00000100U : rwx|r-- */
    MPU_SUPERVISOR_RWX_USER_RX      = 0x05U,  /*!< 0b00000101U : rwx|r-x */
    MPU_SUPERVISOR_RWX_USER_RW      = 0x06U,  /*!< 0b00000110U : rwx|rw- */
    MPU_SUPERVISOR_RWX_USER_RWX     = 0x07U,  /*!< 0b00000111U : rwx|rwx */
    MPU_SUPERVISOR_RX_USER_NONE     = 0x08U,  /*!< 0b00001000U : r-x|--- */
    MPU_SUPERVISOR_RX_USER_X        = 0x09U,  /*!< 0b00001001U : r-x|--x */
    MPU_SUPERVISOR_RX_USER_W        = 0x0AU,  /*!< 0b00001010U : r-x|-w- */
    MPU_SUPERVISOR_RX_USER_WX       = 0x0BU,  /*!< 0b00001011U : r-x|-wx */
    MPU_SUPERVISOR_RX_USER_R        = 0x0CU,  /*!< 0b00001100U : r-x|r-- */
    MPU_SUPERVISOR_RX_USER_RX       = 0x0DU,  /*!< 0b00001101U : r-x|r-x */
    MPU_SUPERVISOR_RX_USER_RW       = 0x0EU,  /*!< 0b00001110U : r-x|rw- */
    MPU_SUPERVISOR_RX_USER_RWX      = 0x0FU,  /*!< 0b00001111U : r-x|rwx */
    MPU_SUPERVISOR_RW_USER_NONE     = 0x10U,  /*!< 0b00010000U : rw-|--- */
    MPU_SUPERVISOR_RW_USER_X        = 0x11U,  /*!< 0b00010001U : rw-|--x */
    MPU_SUPERVISOR_RW_USER_W        = 0x12U,  /*!< 0b00010010U : rw-|-w- */
    MPU_SUPERVISOR_RW_USER_WX       = 0x13U,  /*!< 0b00010011U : rw-|-wx */
    MPU_SUPERVISOR_RW_USER_R        = 0x14U,  /*!< 0b00010100U : rw-|r-- */
    MPU_SUPERVISOR_RW_USER_RX       = 0x15U,  /*!< 0b00010101U : rw-|r-x */
    MPU_SUPERVISOR_RW_USER_RW       = 0x16U,  /*!< 0b00010110U : rw-|rw- */
    MPU_SUPERVISOR_RW_USER_RWX      = 0x17U,  /*!< 0b00010111U : rw-|rwx */
    MPU_SUPERVISOR_USER_NONE        = 0x18U,  /*!< 0b00011000U : ---|--- */
    MPU_SUPERVISOR_USER_X           = 0x19U,  /*!< 0b00011001U : --x|--x */
    MPU_SUPERVISOR_USER_W           = 0x1AU,  /*!< 0b00011010U : -w-|-w- */
    MPU_SUPERVISOR_USER_WX          = 0x1BU,  /*!< 0b00011011U : -wx|-wx */
    MPU_SUPERVISOR_USER_R           = 0x1CU,  /*!< 0b00011100U : r--|r-- */
    MPU_SUPERVISOR_USER_RX          = 0x1DU,  /*!< 0b00011101U : r-x|r-x */
    MPU_SUPERVISOR_USER_RW          = 0x1EU,  /*!< 0b00011110U : rw-|rw- */
    MPU_SUPERVISOR_USER_RWX         = 0x1FU,  /*!< 0b00011111U : rwx|rwx */
    MPU_NONE                        = 0x80U,  /*!< 0b10000000U : --      */
    MPU_W                           = 0xA0U,  /*!< 0b10100000U : w-      */
    MPU_R                           = 0xC0U,  /*!< 0b11000000U : -r      */
    MPU_RW                          = 0xE0U   /*!< 0b11100000U : wr      */
} mpu_access_rights_t;

/*! @brief MPU status return codes. */
typedef enum _mpu_status {
    MPU_STATUS_SUCCESS           = 0x0U,  /*!< MPU Succeed.                */
    MPU_STATUS_FAIL              = 0x1U,  /*!< MPU failed.                 */
    MPU_STATUS_NOT_INITLIALIZED  = 0x2U   /*!< MPU is not initialized yet. */
} mpu_status_t;

/*! @brief MPU master access rights. */
typedef struct _mpu_master_access_right {
    uint8_t                     masterNum;                  /*!< Master number */
    mpu_access_rights_t         accessRight;                /*!< Access right  */
#if FSL_FEATURE_MPU_HAS_PROCESS_IDENTIFIER
    bool                        processIdentifierEnable;    /*!< Enables or disables process identifier  */
#endif
} mpu_master_access_right_t;

/*!
 * @brief MPU user region configuration structure
 *   This structure is used when calling the MPU_DRV_Init function.
 */
typedef struct MpuUserConfig {
    uint32_t                            startAddr;         /*!< Memory region start address   */
    uint32_t                            endAddr;           /*!< Memory region end address     */
    const mpu_master_access_right_t     *masterAccRight;   /*!< Access permission for masters */
#if FSL_FEATURE_MPU_HAS_PROCESS_IDENTIFIER
    uint8_t                             processIdentifier; /*!< Process identifier  */
    uint8_t                             processIdMask;     /*!< Process identifier mask. The setting bit will ignore the same bit in process identifier */
#endif /* FSL_FEATURE_MPU_HAS_PROCESS_IDENTIFIER */
} mpu_user_config_t;

/*******************************************************************************
 * API
 *******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief The function checks clock gate status of the module, sets the MPU regions according to user input, and then enables the MPU.
 *  Please note that access rights for region 0 will always be configured and
 *  regionCnt takes values between 1 and the maximum region count supported by the hardware.
 *   e.g. In S32K144 the number of supported regions is 8.
 *
 * @param instance The MPU peripheral instance number.
 * @param regionCnt Number of configuration regions.
 * @param userConfigArr The pointer to the array of MPU user configure structure, see #mpu_user_config_t.
 * @return STATUS_MPU_SUCCESS means success. Otherwise, means failure.
 */
mpu_status_t MPU_DRV_Init(uint32_t instance,
                          uint8_t regionCnt,
                          const mpu_user_config_t *userConfigArr);

/*!
 * @brief De-initializes the MPU region by disabling MPU module.
 *
 * @param instance The MPU peripheral instance number.
 */
void MPU_DRV_Deinit(uint32_t instance);

/*!
 * @brief Sets the region start and end address.
 *
 * @param instance The MPU peripheral instance number.
 * @param regionNum The region number.
 * @param startAddr Region start address.
 * @param endAddr Region end address.
 */
void MPU_DRV_SetRegionAddr(uint32_t instance,
                           uint8_t regionNum,
                           uint32_t startAddr,
                           uint32_t endAddr);

/*!
 * @brief Sets the region configuration.
 *
 * @param instance The MPU peripheral instance number.
 * @param regionNum The region number.
 * @param userConfigPtr Region configuration structure pointer.
 * @return STATUS_MPU_SUCCESS means success. Otherwise, means failure.
 */
mpu_status_t MPU_DRV_SetRegionConfig(uint32_t instance,
                                     uint8_t regionNum,
                                     const mpu_user_config_t *userConfigPtr);

/*!
 * @brief Configures access permission.
 *
 * @param instance The MPU peripheral instance number.
 * @param regionNum The MPU region number.
 * @param accessRightsPtr A pointer to access permission structure.
 * @return STATUS_MPU_SUCCESS means success. Otherwise, means failure.
 */
mpu_status_t MPU_DRV_SetMasterAccessRights(uint32_t instance,
                                           uint8_t regionNum,
                                           const mpu_master_access_right_t *accessRightsPtr);

/*!
 * @brief Gets the MPU access error detail information for a slave port.
 *
 * @param instance The MPU peripheral instance number.
 * @param slavePortNum The slave port number to get Error Detail.
 * @param errInfoPtr A pointer to access error info structure.
 */
void MPU_DRV_GetDetailErrorAccessInfo(uint32_t instance,
                                      uint8_t slavePortNum,
                                      mpu_access_err_info_t *errInfoPtr);

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* FSL_MPU_DRIVER_H_ */
/*******************************************************************************
 * EOF
 *******************************************************************************/
