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
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES,
 * LOSS OF USE, DATA, OR PROFITS, OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "fsl_crc_driver.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Table of base addresses for CRC instances. */
CRC_Type * const g_crcBase[] = CRC_BASE_PTRS;

/*******************************************************************************
 * Code
 ******************************************************************************/
/*FUNCTION*********************************************************************
 *
 * Function Name : CRC_DRV_Init
 * Description   : This function initializes CRC driver based on user configuration input.
 *
 *END*************************************************************************/
crc_status_t CRC_DRV_Init(uint32_t instance,
                          const crc_user_config_t *userConfigPtr)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < CRC_INSTANCE_COUNT);
#endif
    CRC_Type * base = g_crcBase[instance];

    /* Set the default configuration */
    CRC_HAL_Init(base);

    /* Set the CRC configuration */
    return CRC_DRV_Configure(instance, userConfigPtr);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CRC_DRV_Deinit
 * Description   : This function sets the default configuration.
 *
 *END**************************************************************************/
crc_status_t CRC_DRV_Deinit(uint32_t instance)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < CRC_INSTANCE_COUNT);
#endif
    CRC_Type * base = g_crcBase[instance];

    /* Set the default configuration */
    CRC_HAL_Init(base);

    return CRC_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CRC_DRV_GetCrcBlock
 * Description   : This method appends block of bytes to current CRC calculation
 *                 and returns new result
 *
 *END**************************************************************************/
uint32_t CRC_DRV_GetCrcBlock(uint32_t instance,
                             const uint8_t *data,
                             uint32_t dataSize)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < CRC_INSTANCE_COUNT);
    DEV_ASSERT(data);
    DEV_ASSERT(dataSize);
#endif
    const uint32_t *data32;
    const uint8_t *data8;
    const uint32_t sizeof32 = sizeof(uint32_t);
    CRC_Type * base = g_crcBase[instance];

    /* Start the checksum calculation */
    /* 8-bit reads and writes till source address is aligned 4 bytes */
    while (((uint32_t)data & 3U) && (dataSize > 0U))
    {
        CRC_HAL_SetDataLLReg(base, *(data++));
        dataSize--;
    }

    /* Use 32-bit reads and writes as long as possible */
    data32 = (const uint32_t *)data;
    while (dataSize >= sizeof32)
    {
        CRC_HAL_SetDataReg(base, *(data32++));  /* 32-bit data writing */
        dataSize -= sizeof32;
    }

    /* 8-bit reads and writes till end of data buffer */
    data8 = (const uint8_t *)data32;
    while (dataSize--)
    {
        CRC_HAL_SetDataLLReg(base, *(data8++));
    }

    /* Gets CRC result */
    return CRC_HAL_GetCrcResult(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CRC_DRV_Configure
 * Description   : Configures the CRC module from a user configuration structure.
 *
 *END**************************************************************************/
crc_status_t CRC_DRV_Configure(uint32_t instance,
                               const crc_user_config_t *userConfigPtr)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < CRC_INSTANCE_COUNT);
#endif
    crc_status_t ret = CRC_STATUS_SUCCESS;
    CRC_Type * base = g_crcBase[instance];

    if (!userConfigPtr)
    {
        ret = CRC_STATUS_INVALID_ARGUMENT;
    }
    else
    {
        /* 1. Set CRC mode */
        CRC_HAL_SetProtocolWidth(base, userConfigPtr->crcWidth);

        /* 2. Set transpose and complement options */
        CRC_HAL_SetWriteTranspose(base, userConfigPtr->writeTranspose);
        CRC_HAL_SetReadTranspose(base, userConfigPtr->readTranspose);
        CRC_HAL_SetFXorMode(base, userConfigPtr->complementChecksum);

        /* 3. Write polynomial */
        CRC_HAL_SetPolyReg(base, userConfigPtr->polynomial);

        /* 4. Write seed (initial checksum) */
        CRC_HAL_SetSeedOrDataMode(base, true);
        CRC_HAL_SetDataReg(base, userConfigPtr->seed);
        CRC_HAL_SetSeedOrDataMode(base, false);
    }

    return ret;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
