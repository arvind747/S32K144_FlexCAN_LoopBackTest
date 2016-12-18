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
#if !defined(__FSL_PMC_HAL_H__)
#define __FSL_PMC_HAL_H__

#include "fsl_device_registers.h"
#include <stdbool.h>

/*! @file */

/*!
 * @defgroup fsl_pmc_hal Power Management Controller (PMC)
 * @ingroup power_manager
 * @brief This module covers the functionality of the Power Management Controller (PMC) peripheral.
 * <p>
 *  PMC HAL provides the API for reading and writing register bit-fields belonging to the PMC module.
 * </p>
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief power management control interrupts */
typedef enum _pmc_int_select {
  PMC_INT_LOW_VOLT_DETECT,                   /*!< Low Voltage Detect Interrupt  */
  PMC_INT_LOW_VOLT_WARN                      /*!< Low Voltage Warning Interrupt */
} pmc_int_select_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

/*! @name Power Management Controller Control APIs*/
/*@{*/


/*!
 * @brief Enables/Disables the low voltage-related interrupts.
 *
 * This function  enables  the low voltage detection, warning, 
 * etc. interrupts. 
 *
 * @param[in] baseAddr  Base address for current PMC instance.
 * @param[in] intSelect interrupt select
 * @param[in] enable    enable/disable the interrupt
 */
void PMC_HAL_SetLowVoltIntCmd(PMC_Type* const baseAddr, const pmc_int_select_t intSelect, const bool enable);


/*!
 * @brief Acknowledges the low voltage-related interrupts.
 *
 * This function  acknowledges  the low voltage detection, warning, 
 * etc. interrupts
 *
 * @param[in] baseAddr  Base address for current PMC instance.
 * @param[in] intSelect interrupt select
 */
void PMC_HAL_SetLowVoltIntAckCmd(PMC_Type* const baseAddr, const pmc_int_select_t intSelect);


/*!
 * @brief Gets the flag for the low voltage-related interrupts.
 *
 * This function gets the flag for the low voltage detection, warning, 
 * etc. interrupts
 *
 * @param[in] baseAddr  Base address for current PMC instance.
 * @param[in] intSelect interrupt select
 * @return status Current low voltage interrupt flag
 *                - true: Low-Voltage Interrupt flag is set
 *                - false: Low-Voltage Interrupt flag is not set
 */
bool PMC_HAL_GetLowVoltIntFlag(const PMC_Type* const baseAddr, const pmc_int_select_t intSelect);


/*!
 * @brief Low-Voltage Detect Hardware Reset Enable/Disable (write once)
 *
 * This function enables/disables the  hardware reset for the low voltage 
 * detection. When enabled, if the LVDF (Low Voltage Detect Flag) is set, a 
 * hardware reset occurs. This setting is a write-once-only. Any additional writes 
 * are ignored.
 *
 * @param[in] baseAddr  Base address for current PMC instance.
 * @param[in] enable    enable/disable the LVD hardware reset
 */
static inline void PMC_HAL_SetLowVoltDetectResetCmd(PMC_Type* const baseAddr, const bool enable)
{
  BITBAND_ACCESS32(&(baseAddr->LVDSC1), PMC_LVDSC1_LVDRE_SHIFT) = enable;
}


/*!
 * @brief Enables/Disables the Bias.
 *
 * This function  enables/disables the Bias.
 *
 * @param[in] baseAddr  Base address for current PMC instance.
 * @param[in] enable    enable/disable the Bias.
 */
static inline void PMC_HAL_SetBiasMode(PMC_Type* const baseAddr, const bool enable)
{
  BITBAND_ACCESS32(&(baseAddr->REGSC), PMC_REGSC_BIASEN_SHIFT) = enable;
}

/*!
 * @brief Enables/Disables the Low Power Oscillator.
 *
 * This function  enables/disables the Low Power Oscillator.
 *
 * @param[in] baseAddr  Base address for current PMC instance.
 * @param[in] enable    enable/disable the Low Power Oscillator.
 */
static inline void PMC_HAL_SetLpoMode(PMC_Type* const baseAddr, const bool enable)
{
  BITBAND_ACCESS32(&(baseAddr->REGSC), PMC_REGSC_LPODIS_SHIFT) = enable;
}


/*!
 * @brief Gets the Regulator regulation status.
 *
 * This function provides the current status of 
 * the internal voltage regulator. 
 *
 * @param[in] baseAddr  Base address for current PMC instance.
 * @return value Regulation status
 *               0 - Regulator is in low power mode or transition to/from.
 *               1 - Regulator is in full performance mode.
 *
 */
static inline uint8_t PMC_HAL_GetRegulatorStatus(const PMC_Type* const baseAddr)
{
  return (uint8_t)BITBAND_ACCESS32(&(baseAddr->REGSC), PMC_REGSC_REGFPM_SHIFT);
}

/*!
 * @brief Gets the Low Power Oscillator status.
 *
 * This function provides the current status of 
 * the Low Power Oscillator.
 *
 * @param[in] baseAddr  Base address for current PMC instance.
 * @return value LPO status
 *               0 - Low power oscillator in low phase.
 *               1 - Low power oscillator in high phase.
 *
 */
static inline uint8_t PMC_HAL_GetLpoStatus(const PMC_Type* const baseAddr)
{
  return (uint8_t)BITBAND_ACCESS32(&(baseAddr->REGSC), PMC_REGSC_LPOSTAT_SHIFT);
}

/*!
 * @brief Low Power Oscillator Trimming Value
 * 
 * This function sets the trimming value for the low power oscillator
 *
 * @param[in] baseAddr  Base address for current PMC instance.
 * @param[in] value     Trimming value
 */
static inline void PMC_HAL_SetLpoTrimValue(PMC_Type* const baseAddr, const uint8_t value)
{
  uint8_t regValue = (uint8_t)baseAddr->LPOTRIM;
  regValue &= (uint8_t)(~(PMC_LPOTRIM_LPOTRIM_MASK));
  regValue |= PMC_LPOTRIM_LPOTRIM(value);
  baseAddr->LPOTRIM = (uint32_t)regValue;
}

/*@}*/

#if defined(__cplusplus)
}
#endif /* __cplusplus*/

/*! @}*/

#endif /* __FSL_PMC_HAL_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/

