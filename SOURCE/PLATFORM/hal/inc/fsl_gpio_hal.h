/*
 * Copyright (c) 2014 - 2015, Freescale Semiconductor, Inc.
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

#ifndef __FSL_GPIO_HAL_H__
#define __FSL_GPIO_HAL_H__

#include <stdint.h>
#include "fsl_device_registers.h"

/*! @file */

/*!
 * @addtogroup gpio_hal
 * @{
 */

/*******************************************************************************
 * API
 ******************************************************************************/

 #if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name GPIO
 * General GPIO functions.
 */
/*! @{*/

/*!
 * @brief Write all pins of a port
 *
 * This function writes all pins configured as output with the values given in
 * the parameter pins. '0' represents LOW, '1' represents HIGH.
 *
 * @param baseAddr  GPIO base pointer (PTA, PTB, PTC, etc.)
 * @param pins pin mask to be written
 *        - 0: corresponding pin is set to LOW
 *        - 1: corresponding pin is set to HIGH
 */
static inline void GPIO_HAL_WritePins(GPIO_Type* const baseAddr, const uint32_t pins)
{
    baseAddr->PDOR = GPIO_PDOR_PDO(pins);
}

/*!
 * @brief Get the current output from a port
 *
 * This function returns the current output that is written to a port. Only pins
 * that are configured as output will have meaningful values.
 *
 * @param baseAddr  GPIO base pointer (PTA, PTB, PTC, etc.)
 * @return GPIO outputs. Each bit represents one pin (LSB is pin 0, MSB is pin
 * 31). For each bit:
 *        - 0: corresponding pin is set to LOW
 *        - 1: corresponding pin is set to HIGH
 */
static inline uint32_t GPIO_HAL_GetPinsOutput(const GPIO_Type* const baseAddr)
{
    return (uint32_t)(baseAddr->PDOR);
}

/*!
 * @brief Write pins with 'Set' value
 *
 * This function configures output pins listed in parameter pins (bits that are
 * '1') to have a value of 'set' (HIGH). Pins corresponding to '0' will be
 * unaffected.
 *
 * @param baseAddr  GPIO base pointer (PTA, PTB, PTC, etc.)
 * @param pins pin mask of bits to be set.  Each bit represents one pin (LSB is
 * pin 0, MSB is pin 31). For each bit:
 *        - 0: corresponding pin is unaffected
 *        - 1: corresponding pin is set to HIGH
 */
static inline void GPIO_HAL_SetPins(GPIO_Type* const baseAddr, const uint32_t pins)
{
	baseAddr->PSOR = GPIO_PSOR_PTSO(pins);
}

/*!
 * @brief Write pins to 'Clear' value
 *
 * This function configures output pins listed in parameter pins (bits that are
 * '1') to have a 'cleared' value (LOW). Pins corresponding to '0' will be
 * unaffected.
 *
 * @param baseAddr  GPIO base pointer (PTA, PTB, PTC, etc.)
 * @param pins pin mask of bits to be cleared.  Each bit represents one pin (LSB
 * is pin 0, MSB is pin 31). For each bit:
 *        - 0: corresponding pin is unaffected
 *        - 1: corresponding pin is cleared (set to LOW)
 */
static inline void GPIO_HAL_ClearPins(GPIO_Type* const baseAddr, const uint32_t pins)
{
	baseAddr->PCOR = GPIO_PCOR_PTCO(pins);
}

/*!
 * @brief Toggle pins value
 *
 * This function toggles output pins listed in parameter pins (bits that are
 * '1'). Pins corresponding to '0' will be unaffected.
 *
 * @param baseAddr  GPIO base pointer (PTA, PTB, PTC, etc.)
 * @param pins pin mask of bits to be toggled.  Each bit represents one pin (LSB
 * is pin 0, MSB is pin 31). For each bit:
 *        - 0: corresponding pin is unaffected
 *        - 1: corresponding pin is toggled
 */
static inline void GPIO_HAL_TogglePins(GPIO_Type* const baseAddr, const uint32_t pins)
{
	baseAddr->PTOR = GPIO_PTOR_PTTO(pins);
}

/*!
 * @brief Read input pins
 *
 * This function returns the current input values from a port. Only pins
 * configured as input will have meaningful values.
 *
 * @param baseAddr  GPIO base pointer (PTA, PTB, PTC, etc.)
 * @return GPIO inputs. Each bit represents one pin (LSB is pin 0, MSB is pin
 * 31). For each bit:
 *        - 0: corresponding pin is read as LOW
 *        - 1: corresponding pin is read as HIGH
 */
static inline uint32_t GPIO_HAL_ReadPins(const GPIO_Type* const baseAddr)
{
    return (uint32_t)(baseAddr->PDIR);
}

/*!
 * @brief Get the pins directions configuration for a port
 *
 * This function returns the current pins directions for a port. Pins
 * corresponding to bits with value of '1' are configured as output and
 * pins corresponding to bits with value of '0' are configured as input.
 *
 * @param baseAddr  GPIO base pointer (PTA, PTB, PTC, etc.)
 * @return GPIO directions. Each bit represents one pin (LSB is pin 0, MSB is
 * pin 31). For each bit:
 *        - 0: corresponding pin is set to input
 *        - 1: corresponding pin is set to output
 */
static inline uint32_t GPIO_HAL_GetPinsDirection(const GPIO_Type* const baseAddr)
{
    return (uint32_t)(baseAddr->PDDR);
}

/*!
 * @brief Set the pins directions configuration for a port
 *
 * This function set the a direction configuration for all pins
 * in a port. Pins corresponding to bits with value of '1' will be configured as
 * output and pins corresponding to bits with value of '0' will be configured as
 * input.
 *
 * @param baseAddr  GPIO base pointer (PTA, PTB, PTC, etc.)
 * @param pins pin mask
 * @return GPIO directions. Each bit represents one pin (LSB is pin 0, MSB is
 * pin 31). For each bit:
 *        - 0: corresponding pin is set to input
 *        - 1: corresponding pin is set to output
 */
static inline void GPIO_HAL_SetPinsDirection(GPIO_Type* const baseAddr, const uint32_t pins)
{
	baseAddr->PDDR = GPIO_PDDR_PDD(pins);
}

/*! @}*/

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __FSL_GPIO_HAL_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/
