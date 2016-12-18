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

#ifndef FSL_CRC_HAL_H_
#define FSL_CRC_HAL_H_

#include <stdint.h>
#include <stdbool.h>
#include "fsl_device_registers.h"

/*!
 * @addtogroup crc_hal
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief CRC type of transpose of read write data. */
typedef enum _crc_transpose {
    CRC_TRANSPOSE_NONE              = 0x00U,    /*!< No transpose */
    CRC_TRANSPOSE_BITS              = 0x01U,    /*!< Transpose bits in bytes */
    CRC_TRANSPOSE_BITS_AND_BYTES    = 0x02U,    /*!< Transpose bytes and bits in bytes */
    CRC_TRANSPOSE_BYTES             = 0x03U     /*!< Transpose bytes */
} crc_transpose_t;

/*! @brief CRC bit width. */
typedef enum _crc_bit_width {
    CRC_BITS_16 = 0U,   /*!< Generate 16-bit CRC code */
    CRC_BITS_32 = 1U    /*!< Generate 32-bit CRC code */
} crc_bit_width_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief This function initializes the module to default configuration @n
 *  Initial checksum: 0U @n
 *  Default polynomial: 0x1021U @n
 *  Type of read transpose: CRC_TRANSPOSE_NONE @n
 *  Type of write transpose: CRC_TRANSPOSE_NONE @n
 *  No complement of checksum read. @n
 *  32-bit CRC. @n
 *
 * @param base The CRC peripheral base address.
 */
void CRC_HAL_Init(CRC_Type * base);

/*!
 * @brief Gets the current CRC result from the data register.
 *
 * @param base The CRC peripheral base address.
 * @return Returns the current CRC result.
 */
static inline uint32_t CRC_HAL_GetDataReg(CRC_Type * base)
{
    return base->DATAu.DATA;
}

/*!
 * @brief Sets the 32 bits of CRC data register.
 *
 * @param base The CRC peripheral base address.
 * @param value New data for CRC computation.
 */
static inline void CRC_HAL_SetDataReg(CRC_Type * base,
                                      uint32_t value)
{
    base->DATAu.DATA = value;
}

/*!
 * @brief Gets the upper 16 bits of the current CRC result from the data register.
 *
 * @param base The CRC peripheral base address.
 * @return Returns the upper 16 bits of the current CRC result.
 */
static inline uint16_t CRC_HAL_GetDataHReg(CRC_Type * base)
{
    return base->DATAu.DATA_16.H;
}

/*!
 * @brief Sets the upper 16 bits of CRC data register.
 *
 * @param base The CRC peripheral base address.
 * @param value New data for CRC computation.
 */
static inline void CRC_HAL_SetDataHReg(CRC_Type * base,
                                       uint16_t value)
{
    base->DATAu.DATA_16.H = value;
}

/*!
 * @brief Gets the lower 16 bits of the current CRC result from the data register.
 *
 * @param base The CRC peripheral base address.
 * @return Returns the lower 16 bits of the current CRC result.
 */
static inline uint16_t CRC_HAL_GetDataLReg(CRC_Type * base)
{
    return base->DATAu.DATA_16.L;
}

/*!
 * @brief Sets the lower 16 bits of CRC data register.
 *
 * @param base The CRC peripheral base address.
 * @param value New data for CRC computation.
 */
static inline void CRC_HAL_SetDataLReg(CRC_Type * base,
                                       uint16_t value)
{
    base->DATAu.DATA_16.L = value;
}

/*!
 * @brief Sets the High Upper Byte - HU of CRC data register.
 *
 * @param base The CRC peripheral base address.
 * @param value New data for CRC computation.
 */
static inline void CRC_HAL_SetDataHUReg(CRC_Type * base,
                                        uint8_t value)
{
    base->DATAu.DATA_8.HU = value;
}

/*!
 * @brief Sets the High Lower Byte - HL of CRC data register.
 *
 * @param base The CRC peripheral base address.
 * @param value New data for CRC computation.
 */
static inline void CRC_HAL_SetDataHLReg(CRC_Type * base,
                                        uint8_t value)
{
    base->DATAu.DATA_8.HL = value;
}

/*!
 * @brief Sets the Low Upper Byte - LU of CRC data register.
 *
 * @param base The CRC peripheral base address.
 * @param value New data for CRC computation.
 */
static inline void CRC_HAL_SetDataLUReg(CRC_Type * base,
                                        uint8_t value)
{
    base->DATAu.DATA_8.LU = value;
}

/*!
 * @brief Sets the Low Lower Byte - LL of CRC data register.
 *
 * @param base The CRC peripheral base address.
 * @param value New data for CRC computation.
 */
static inline void CRC_HAL_SetDataLLReg(CRC_Type * base,
                                        uint8_t value)
{
    base->DATAu.DATA_8.LL = value;
}

/*!
 * @brief Gets the polynomial register value.
 *
 * @param base The CRC peripheral base address.
 * @return Returns the polynomial register value.
 */
static inline uint32_t CRC_HAL_GetPolyReg(CRC_Type * base)
{
    return base->GPOLY;
}

/*!
 * @brief Sets the polynomial register.
 *
 * @param base The CRC peripheral base address.
 * @param value Polynomial value.
 */
static inline void CRC_HAL_SetPolyReg(CRC_Type * base,
                                      uint32_t value)
{
    base->GPOLY = value;
}

/*!
 * @brief Gets the upper 16 bits of polynomial register.
 *  Note that this upper part of the register is not used in 16-bit CRC mode.
 *
 * @param base The CRC peripheral base address.
 * @return Returns the upper 16 bits of polynomial register.
 */
static inline uint16_t CRC_HAL_GetPolyHReg(CRC_Type * base)
{
    return (uint16_t)((base->GPOLY & CRC_GPOLY_HIGH_MASK) >> CRC_GPOLY_HIGH_SHIFT);
}

/*!
 * @brief Sets the upper 16 bits of polynomial register.
 *  Note that this upper part of the register is ignored in 16-bit CRC mode.
 *
 * @param base The CRC peripheral base address.
 * @param value Polynomial value.
 */
static inline void CRC_HAL_SetPolyHReg(CRC_Type * base,
                                       uint16_t value)
{
    uint32_t gpolyTemp = base->GPOLY;

    gpolyTemp &= ~(CRC_GPOLY_HIGH_MASK);
    gpolyTemp |= CRC_GPOLY_HIGH(value);
    base->GPOLY = gpolyTemp;
}

/*!
 * @brief Gets the lower 16 bits of polynomial register.
 *
 * @param base The CRC peripheral base address.
 * @return Returns the lower 16 bits of polynomial register.
 */
static inline uint16_t CRC_HAL_GetPolyLReg(CRC_Type * base)
{
    return (uint16_t)((base->GPOLY & CRC_GPOLY_LOW_MASK) >> CRC_GPOLY_LOW_SHIFT);
}

/*!
 * @brief Sets the lower 16 bits of polynomial register.
 *
 * @param base The CRC peripheral base address.
 * @param value Polynomial value.
 */
static inline void CRC_HAL_SetPolyLReg(CRC_Type * base,
                                       uint16_t value)
{
    uint32_t gpolyTemp = base->GPOLY;

    gpolyTemp &= ~(CRC_GPOLY_LOW_MASK);
    gpolyTemp |= CRC_GPOLY_LOW(value);
    base->GPOLY = gpolyTemp;
}

/*!
 * @brief Gets the CRC_DATA register mode.
 *
 * @param base The CRC peripheral base address.
 * @return CRC_DATA register mode
 *         -true: CRC_DATA register is used for seed values
 *         -false: CRC_DATA register is used for data values
 */
static inline bool CRC_HAL_GetSeedOrDataMode(CRC_Type * base)
{
    return (bool)BITBAND_ACCESS32(&(base->CTRL), CRC_CTRL_WAS_SHIFT);
}

/*!
 * @brief Sets the CRC_DATA register mode.
 *
 * @param base The CRC peripheral base address.
 * @param enable Enable CRC data register to use for seed value.
          -true: use CRC data register for seed values
          -false: use CRC data register for data values
 */
static inline void CRC_HAL_SetSeedOrDataMode(CRC_Type * base,
                                             bool enable)
{
    BITBAND_ACCESS32(&(base->CTRL), CRC_CTRL_WAS_SHIFT) = (uint32_t)enable;
}

/*!
 * @brief Gets the CRC transpose type for writes.
 *
 * @param base The CRC peripheral base address.
 * @return CRC input transpose type for writes.
 */
static inline crc_transpose_t CRC_HAL_GetWriteTranspose(CRC_Type * base)
{
    return (crc_transpose_t)((base->CTRL & CRC_CTRL_TOT_MASK) >> CRC_CTRL_TOT_SHIFT);
}

/*!
 * @brief Sets the CRC transpose type for writes.
 *
 * @param base The CRC peripheral base address.
 * @param transp The CRC input transpose type.
 */
static inline void CRC_HAL_SetWriteTranspose(CRC_Type * base,
                                             crc_transpose_t transp)
{
    uint32_t ctrlTemp = base->CTRL;

    ctrlTemp &= ~(CRC_CTRL_TOT_MASK);
    ctrlTemp |= CRC_CTRL_TOT(transp);
    base->CTRL = ctrlTemp;
}

/*!
 * @brief Gets the CRC transpose type for reads.
 *
 * @param base The CRC peripheral base address.
 * @return CRC output transpose type.
 */
static inline crc_transpose_t CRC_HAL_GetReadTranspose(CRC_Type * base)
{
    return (crc_transpose_t)((base->CTRL & CRC_CTRL_TOTR_MASK) >> CRC_CTRL_TOTR_SHIFT);
}

/*!
 * @brief Sets the CRC transpose type for reads.
 *
 * @param base The CRC peripheral base address.
 * @param transp The CRC output transpose type.
 */
static inline void CRC_HAL_SetReadTranspose(CRC_Type * base,
                                            crc_transpose_t transp)
{
    uint32_t ctrlTemp = base->CTRL;

    ctrlTemp &= ~(CRC_CTRL_TOTR_MASK);
    ctrlTemp |= CRC_CTRL_TOTR(transp);
    base->CTRL = ctrlTemp;
}

/*!
 * @brief Gets complement read of CRC data register.
 *
 * Some CRC protocols require the final checksum to be XORed with 0xFFFFFFFF
 * or 0xFFFF. Complement mode enables "on the fly" complementing of read data.
 *
 * @param base The CRC peripheral base address.
 * @return Complement read
 *         -true: Invert or complement the read value of the CRC Data register.
 *         -false: No XOR on reading.
 */
static inline bool CRC_HAL_GetFXorMode(CRC_Type * base)
{
    return (bool)BITBAND_ACCESS32(&(base->CTRL), CRC_CTRL_FXOR_SHIFT);
}

/*!
 * @brief Sets complement read of CRC data register.
 *
 * Some CRC protocols require the final checksum to be XORed with 0xFFFFFFFF
 * or 0xFFFF. Complement mode enables "on the fly" complementing of read data.
 *
 * @param base The CRC peripheral base address.
 * @param enable Enable or disable complementing of read data.
 */
static inline void CRC_HAL_SetFXorMode(CRC_Type * base,
                                       bool enable)
{
    BITBAND_ACCESS32(&(base->CTRL), CRC_CTRL_FXOR_SHIFT) = (uint32_t)enable;
}

/*!
 * @brief Gets the CRC protocol width.
 *
 * @param base The CRC peripheral base address.
 * @return CRC protocol width
 *         - CRC_BITS_16: 16-bit CRC protocol
 *         - CRC_BITS_32: 32-bit CRC protocol
 */
static inline crc_bit_width_t CRC_HAL_GetProtocolWidth(CRC_Type * base)
{
    return (crc_bit_width_t)BITBAND_ACCESS32(&(base->CTRL), CRC_CTRL_TCRC_SHIFT);
}

/*!
 * @brief Sets the CRC protocol width.
 *
 * @param base The CRC peripheral base address.
 * @param width The CRC protocol width
 *         - CRC_BITS_16: 16-bit CRC protocol
 *         - CRC_BITS_32: 32-bit CRC protocol
 */
static inline void CRC_HAL_SetProtocolWidth(CRC_Type * base,
                                            crc_bit_width_t width)
{
    BITBAND_ACCESS32(&(base->CTRL), CRC_CTRL_TCRC_SHIFT) = (uint32_t)width;
}

/*!
 * @brief CRC_HAL_GetCrc32
 *
 * This method appends 32-bit data to the current CRC calculation
 * and returns new result. If the newSeed is true, seed set and
 * result are calculated from the seed new value (new CRC calculation).
 *
 * @param base The CRC peripheral base address.
 * @param data Input data for CRC calculation
 * @param newSeed Sets new CRC calculation.
 *        - true: New seed set and used for new calculation.
 *        - false: Seed argument ignored, continues old calculation.
 * @param seed New seed if newSeed is true, else ignored
 * @return new CRC result.
 */
uint32_t CRC_HAL_GetCrc32(CRC_Type * base,
                          uint32_t data,
                          bool newSeed,
                          uint32_t seed);

/*!
 * @brief CRC_HAL_GetCrc16
 *
 * This method appends the 16-bit data to the current CRC calculation
 * and returns a new result. If the newSeed is true, seed set and
 * result are calculated from the seed new value (new CRC calculation).
 *
 * @param base The CRC peripheral base address.
 * @param data Input data for CRC calculation
 * @param newSeed Sets new CRC calculation.
 *        - true: New seed set and used for new calculation.
 *        - false: Seed argument ignored, continues old calculation.
 * @param seed New seed if newSeed is true, else ignored
 * @return new CRC result.
 */
uint32_t CRC_HAL_GetCrc16(CRC_Type * base,
                          uint16_t data,
                          bool newSeed,
                          uint32_t seed);

/*!
 * @brief CRC_HAL_GetCrc8
 *
 * This method appends the 8-bit data to the current CRC calculation
 * and returns a new result. If the newSeed is true, seed set and
 * result are calculated from the seed new value (new CRC calculation).
 *
 * @param base The CRC peripheral base address.
 * @param data Input data for CRC calculation
 * @param newSeed Sets new CRC calculation.
 *        - true: New seed set and used for new calculation.
 *        - false: Seed argument ignored, continues old calculation.
 * @param seed New seed if newSeed is true, else ignored
 * @return new CRC result.
 */
uint32_t CRC_HAL_GetCrc8(CRC_Type * base,
                         uint8_t data,
                         bool newSeed,
                         uint32_t seed);

/*!
 * @brief CRC_HAL_GetCrcResult
 *
 * This method returns the current result of the CRC calculation.
 * The result is the ReadTranspose dependent.
 *
 * @param base The CRC peripheral base address.
 * @return result of CRC calculation.
 */
uint32_t CRC_HAL_GetCrcResult(CRC_Type * base);

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* FSL_CRC_HAL_H__ */
/*******************************************************************************
 * EOF
 ******************************************************************************/
