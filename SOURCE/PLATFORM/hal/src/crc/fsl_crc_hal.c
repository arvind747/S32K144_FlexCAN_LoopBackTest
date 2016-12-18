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

#include "fsl_crc_hal.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Default polynomial 0x1021U */
#define CRC_DEFAULT_POLYNOMIAL 0x1021U
/* Initial checksum */
#define CRC_INITIAL_SEED 0U

/*******************************************************************************
 * Code
 ******************************************************************************/
/*FUNCTION**********************************************************************
 *
 * Function Name : CRC_HAL_Init
 * Description   : This function initializes the module to default configuration
 *                  Initial checksum: 0U
 *                  Default polynomial: 0x1021U
 *                  Type of read transpose: CRC_TRANSPOSE_NONE
 *                  Type of write transpose: CRC_TRANSPOSE_NONE
 *                  No complement of checksum read
 *                  32-bit CRC
 *
 *END**************************************************************************/
void CRC_HAL_Init(CRC_Type * base)
{
    /* Set CRC mode */
    CRC_HAL_SetProtocolWidth(base, CRC_BITS_32);

    /* Set transpose and complement options */
    CRC_HAL_SetWriteTranspose(base, CRC_TRANSPOSE_NONE);
    CRC_HAL_SetReadTranspose(base, CRC_TRANSPOSE_NONE);
    CRC_HAL_SetFXorMode(base, false);

    /* Write polynomial */
    CRC_HAL_SetPolyReg(base, CRC_DEFAULT_POLYNOMIAL);

    /* Write seed (initial checksum) */
    CRC_HAL_SetSeedOrDataMode(base, true);
    CRC_HAL_SetDataReg(base, CRC_INITIAL_SEED);
    CRC_HAL_SetSeedOrDataMode(base, false);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CRC_HAL_GetCrc32
 * Description   : This method appends 32-bit data to current CRC calculation
 *                 and returns new result
 *
 *END**************************************************************************/
uint32_t CRC_HAL_GetCrc32(CRC_Type * base,
                          uint32_t data,
                          bool newSeed,
                          uint32_t seed)
{
    if (newSeed)
    {
        CRC_HAL_SetSeedOrDataMode(base, true);
        CRC_HAL_SetDataReg(base, seed);
        CRC_HAL_SetSeedOrDataMode(base, false);
    }

    CRC_HAL_SetDataReg(base, data);
    return CRC_HAL_GetCrcResult(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CRC_HAL_GetCrc16
 * Description   : This method appends 16-bit data to current CRC calculation
 *                 and returns new result
 *
 *END**************************************************************************/
uint32_t CRC_HAL_GetCrc16(CRC_Type * base,
                          uint16_t data,
                          bool newSeed,
                          uint32_t seed)
{
    if (newSeed)
    {
        CRC_HAL_SetSeedOrDataMode(base, true);
        CRC_HAL_SetDataReg(base, seed);
        CRC_HAL_SetSeedOrDataMode(base, false);
    }

    CRC_HAL_SetDataLReg(base, data);
    return CRC_HAL_GetCrcResult(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CRC_HAL_GetCrc8
 * Description   : This method appends 8-bit data to current CRC calculation
 *                 and returns new result
 *
 *END**************************************************************************/
uint32_t CRC_HAL_GetCrc8(CRC_Type * base,
                         uint8_t data,
                         bool newSeed,
                         uint32_t seed)
{
    if (newSeed)
    {
        CRC_HAL_SetSeedOrDataMode(base, true);
        CRC_HAL_SetDataReg(base, seed);
        CRC_HAL_SetSeedOrDataMode(base, false);
    }

    CRC_HAL_SetDataLLReg(base, data);
    return CRC_HAL_GetCrcResult(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CRC_HAL_GetCrcResult
 * Description   : This method returns current result of CRC calculation
 *
 *END**************************************************************************/
uint32_t CRC_HAL_GetCrcResult(CRC_Type * base)
{
    uint32_t result = 0U;
    crc_bit_width_t width = CRC_HAL_GetProtocolWidth(base);
    crc_transpose_t transpose;

    if (width == CRC_BITS_16)
    {
        transpose = CRC_HAL_GetReadTranspose(base);
        if((transpose == CRC_TRANSPOSE_BITS_AND_BYTES) || (transpose == CRC_TRANSPOSE_BYTES))
        {
            /* Return upper 16 bits of CRC because of transposition in 16-bit mode */
            result = (uint32_t)CRC_HAL_GetDataHReg(base);
        }
        else
        {
            result = (uint32_t)CRC_HAL_GetDataLReg(base);
        }
    }
    else
    {
        result = CRC_HAL_GetDataReg(base);
    }

    return result;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
