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


#ifndef __FSL_CMP_HAL_H__
#define __FSL_CMP_HAL_H__

#include <stdint.h>
#include <stdbool.h>
#include "fsl_device_registers.h"

/*!
 * @addtogroup cmp_hal Comparator HAL
 * @ingroup cmp 
 * @brief Comparator Hardware Abstraction Layer 
 * @{
 */

#define CMP_INPUT_FLAGS_MASK 0xFF0000
#define CMP_INPUT_FLAGS_SHIFT 16U
#define CMP_ROUND_ROBIN_CHANNELS_MASK 0xFF0000
#define CMP_ROUND_ROBIN_CHANNELS_SHIFT 16U

/*******************************************************************************
 * Enumerations.
 ******************************************************************************/
 /*! @brief Power Modes selection*/
typedef enum
{
    CMP_LOW_SPEED   = 0U,   /*!< Module in low speed mode. */
    CMP_HIGH_SPEED  = 1U    /*!< Module in high speed mode. */
} cmp_power_mode_t;

 /*! @brief Voltage Reference selection*/
typedef enum
{
    CMP_VIN1 = 0U,  /*!< Use Vin1 as supply reference source for DAC. */
    CMP_VIN2 = 1U   /*!< Use Vin2 as supply reference source for DAC. */
} cmp_voltage_reference_t;

 /*! @brief Port Mux Source selection*/
typedef enum
{
    CMP_DAC = 0U,   /*!< Select DAC as source for the comparator port. */
    CMP_MUX = 1U    /*!< Select MUX8 as source for the comparator port. */
} cmp_port_mux_t;

 /*! @brief Comparator output invert selection*/
typedef enum
{
    CMP_NORMAL = 0U,    /*!< Output signal isn't inverted. */
    CMP_INVERT = 1U     /*!< Output signal is inverted. */
} cmp_inverter_t;

 /*! @brief Comparator output select selection*/
typedef enum
{
    CMP_COUT  = 0U,     /*!< Select COUT as comparator output signal. */
    CMP_COUTA = 1U      /*!< Select COUTA as comparator output signal. */
} cmp_output_select_t;

 /*! @brief Comparator output pin enable selection*/
typedef enum
{
    CMP_UNAVAILABLE  = 0U,  /*!< Comparator output isn't available to a specific pin*/
    CMP_AVAILABLE    = 1U   /*!< Comparator output is available to a specific pin*/
} cmp_output_enable_t;

 /*! @brief Comparator hard block offset control*/
typedef enum
{
    CMP_LEVEL_OFFSET_0 = 0U,
    CMP_LEVEL_OFFSET_1 = 1U
} cmp_offset_t;

 /*! @brief Comparator hysteresis control*/
typedef enum
{
    CMP_LEVEL_HYS_0 = 0U,
    CMP_LEVEL_HYS_1 = 1U,
    CMP_LEVEL_HYS_2 = 2U,
    CMP_LEVEL_HYS_3 = 3U
} cmp_hysteresis_t;

 /*! @brief Comparator Round-Robin fixed port*/
typedef enum
{
    CMP_PLUS_FIXED      = 0U,   /*!< The Plus port is fixed. Only the inputs to the Minus port are swept in each round. */
    CMP_MINUS_FIXED     = 1U    /*!< The Minus port is fixed. Only the inputs to the Plus port are swept in each round. */
} cmp_fixed_port_t;

/*! @brief Comparator output interrupt configuration*/
typedef enum
{
    CMP_NO_EVENT            = 0U,   /*!< Comparator output interrupts are disabled OR no event occurred. */
    CMP_FALLING_EDGE        = 1U,   /*!< Comparator output interrupts will be generated only on falling edge OR only falling edge event occurred. */
    CMP_RISING_EDGE         = 2U,   /*!< Comparator output interrupts  will be generated only on rising edge OR only rising edge event occurred. */
    CMP_BOTH_EDGES          = 3U    /*!< Comparator output interrupts  will be generated on both edges OR both edges event occurred. */
} cmp_output_trigger_t;

/*! @brief Comparator functional modes*/
typedef enum
{
    CMP_DISABLED                        = 0U,
    CMP_CONTINUOUS                      = 1U,
    CMP_SAMPLED_NONFILTRED_INT_CLK      = 2U,
    CMP_SAMPLED_NONFILTRED_EXT_CLK      = 3U,
    CMP_SAMPLED_FILTRED_INT_CLK         = 4U,
    CMP_SAMPLED_FILTRED_EXT_CLK         = 5U,
    CMP_WINDOWED                        = 6U,
    CMP_WINDOWED_RESAMPLED              = 7U,
    CMP_WINDOWED_FILTRED                = 8U
} cmp_mode_t;

/*! @brief Comparator channels list (1bit/channel)*/
/*! |---------|---------|-----|---------|---------|*/
/*! |CH7_state|CH6_state|.....|CH1_state|CH0_state|*/
/*! |---------|---------|-----|---------|---------|*/
typedef  uint8_t cmp_ch_list_t;

/*! @brief Number of channel*/
typedef  uint8_t cmp_ch_number_t;

 

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @name CMP_HAL.
 * @{
 */

 #if defined(__cplusplus)
extern C {
#endif
/*!
 * @brief Initializes the comparator registers with reset values
 * @param baseAddr - cmp base pointer
 * @return - void
 */
void CMP_HAL_Init(CMP_Type* const baseAddr);

/*!
 * @brief Gets the comparator functional mode. If you want to get filter count and filter period please use CMP_HAL_GetFilterSamplePeriod
 * and CMP_HAL_GetSamplingState.
 * @param baseAddr - cmp base pointer
 * @return - functional mode
 *   CMP_DISABLED
 *   CMP_CONTINUOUS
 *   CMP_SAMPLED_NONFILTRED_INT_CLK 
 *   CMP_SAMPLED_NONFILTRED_EXT_CLK
 *   CMP_SAMPLED_FILTRED_INT_CLK
 *   CMP_SAMPLED_FILTRED_EXT_CLK
 *   CMP_WINDOWED
 *   CMP_WINDOWED_RESAMPLED
 *   CMP_WINDOWED_FILTRED
 */
cmp_mode_t CMP_HAL_GetFunctionalMode(CMP_Type* const baseAddr);

/*!
 * @brief Sets the comparator functional mode (mode, filter count, filter period)
 * @param baseAddr - cmp base pointer
 * @param mode - functional mode
 *  CMP_DISABLED
 *  CMP_CONTINUOUS
 *  CMP_SAMPLED_NONFILTRED_INT_CLK 
 *  CMP_SAMPLED_NONFILTRED_EXT_CLK
 *  CMP_SAMPLED_FILTRED_INT_CLK
 *  CMP_SAMPLED_FILTRED_EXT_CLK
 *  CMP_WINDOWED
 *  CMP_WINDOWED_RESAMPLED
 *  CMP_WINDOWED_FILTRED
 * @param filter_sample_count - number of consecutive samples that must agree prior to the comparator ouput filter
 * accepting a new output state
 * @param filter_sample_period - sampling period
 * @return -void
 */
void CMP_HAL_SetFunctionalMode(CMP_Type* const baseAddr, cmp_mode_t mode, uint8_t filter_sample_count, uint8_t filter_sample_period);

/*!
 * @brief Verify if the DMA transfer trigger is enabled
 * @param baseAddr - cmp base pointer
 * @return - DMA transfer trigger state
 *  true - DMA trigger is enabled
 *  false - DAM trigger is disabled
 */
static inline bool CMP_HAL_GetDMATriggerState(CMP_Type* const baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->C0), CMP_C0_DMAEN_SHIFT);
}

/*!
 * @brief Configure the DMA transfer trigger
 * @param baseAddr - cmp base pointer
 * @param to_set - DMA transfer trigger state
 *  true - DMA trigger is enabled
 *  false - DAM trigger is disabled
 * @return - void
 */
static inline void CMP_HAL_SetDMATriggerState(CMP_Type* const baseAddr, bool to_set)
{
    BITBAND_ACCESS32(&(baseAddr->C0), CMP_C0_DMAEN_SHIFT) = to_set;
}

/*!
 * @brief Return the comparator output interrupts source configuration(none, rising edge, falling edge or both edges) 
 * @param baseAddr - cmp base pointer
 * @return - comparator output interrupts configuration
 *  CMP_NO_EVENT
 *  CMP_FALLING_EDGE
 *  CMP_RISING_EDGE
 *  CMP_BOTH_EDGES
 */
static inline cmp_output_trigger_t CMP_HAL_GetOutputInterruptTrigger(CMP_Type* const baseAddr)
{
    bool rising_enabled = BITBAND_ACCESS32(&(baseAddr->C0), CMP_C0_IER_SHIFT);
    bool falling_enabled = BITBAND_ACCESS32(&(baseAddr->C0), CMP_C0_IEF_SHIFT);
    return (cmp_output_trigger_t) ((rising_enabled<<1) | (falling_enabled));
}

/*!
 * @brief Set the comparator output interrupts source configuration(none, rising edge, falling edge or both edges) 
 * @param baseAddr - cmp base pointer
 * @param to_set - comparator output interrupts configuration
 *  CMP_NO_EVENT
 *  CMP_FALLING_EDGE
 *  CMP_RISING_EDGE
 *  CMP_BOTH_EDGES
 * @return - void
 */
static inline void CMP_HAL_SetOutputInterruptTrigger(CMP_Type* const baseAddr, cmp_output_trigger_t to_set)
{
    uint32_t tmp = baseAddr->C0;
    tmp &= ~(CMP_C0_IER_MASK) & ~(CMP_C0_IEF_MASK);
    tmp |= CMP_C0_IER(to_set>>1) | CMP_C0_IEF(to_set & 0x01);
    baseAddr->C0 = tmp;
}

/*!
 * @brief Return type of event occurred at the comparator output
 * @param baseAddr - cmp base pointer
 * @return - comparator output flags
 *  CMP_NO_EVENT
 *  CMP_FALLING_EDGE
 *  CMP_RISING_EDGE
 *  CMP_BOTH_EDGES 
 */
static inline cmp_output_trigger_t CMP_HAL_GetOutputEvent(CMP_Type* const baseAddr)
{
    bool rising_enabled = BITBAND_ACCESS32(&(baseAddr->C0), CMP_C0_CFR_SHIFT);
    bool falling_enabled = BITBAND_ACCESS32(&(baseAddr->C0), CMP_C0_CFF_SHIFT);
    return (cmp_output_trigger_t) ((rising_enabled<<1) | (falling_enabled));
}

/*!
 * @brief Clear all output flags
 * @param baseAddr - cmp base pointer
 * @return - void
 */
static inline void CMP_HAL_ClearOutputEvent(CMP_Type* const baseAddr)
{
    uint32_t tmp = baseAddr->C0;
    tmp &= ~(CMP_C0_CFR_MASK);
    tmp |= CMP_C0_CFR(1);
    tmp &= ~(CMP_C0_CFF_MASK);
    tmp |= CMP_C0_CFF(1);
    baseAddr->C0 = tmp;
}

/*!
 * @brief Verify if a rising edge occurred on COUT
 * @param baseAddr - cmp base pointer
 * @return - rising-edge flag state
 *  true - rising-edge event occurred on COUT
 *  false - rising-edge event doesn't occurred on COUT
 */
static inline bool CMP_HAL_GetOutputRisingFlag(CMP_Type* const baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->C0), CMP_C0_CFR_SHIFT);
}

/*!
 * @brief Clear rising edge flag
 * @param baseAddr - cmp base pointer
 * @return - void
 */
static inline void CMP_HAL_ClearOutputRisingFlag(CMP_Type* const baseAddr)
{
    BITBAND_ACCESS32(&(baseAddr->C0), CMP_C0_CFR_SHIFT) = 1U;
}

/*!
 * @brief Verify if a falling-edge occurred on COUT
 * @param baseAddr cmp base pointer
 * @return -  Falling edge flag state
 *  true - falling-edge event occurred on COUT
 *  false - falling-edge event doesn't occurred on COUT
 */
static inline bool CMP_HAL_GetOutputFallingFlag(CMP_Type* const baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->C0), CMP_C0_CFF_SHIFT);
}

/*!
 * @brief Clear falling edge flag
 * @param baseAddr - cmp base pointer
 * @terund - void
 */
static inline void CMP_HAL_ClearOutputFallingFlag(CMP_Type* const baseAddr)
{
    BITBAND_ACCESS32(&(baseAddr->C0), CMP_C0_CFF_SHIFT) = 1U;
}

/*!
 * @brief Return the analog comparator output value
 * @param baseAddr - cmp base pointer
 * @return - analog comparator output value
 */
static inline bool CMP_HAL_GetComparatorOutput(CMP_Type* const baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->C0), CMP_C0_COUT_SHIFT);
}

/*!
 * @brief Return the sample period for filter(clock cycles)
 * @param baseAddr - cmp base pointer
 * @return - sampling period(in bus cycles)
 */
static inline uint8_t CMP_HAL_GetFilterSamplePeriod(CMP_Type* const baseAddr)
{
    uint32_t tmp = baseAddr->C0;
    tmp = (tmp & CMP_C0_FPR_MASK) >> CMP_C0_FPR_SHIFT;
    return ( uint8_t ) (tmp);
}

/*!
 * @brief Set the filter sample period(clock cycles)
 * @param baseAddr -cmp base pointer
 * @param to_set - number of bus cycles 
 * @return - void
 */
static inline void CMP_HAL_SetFilterSamplePeriod(CMP_Type* const baseAddr, uint8_t to_set)
{
    uint32_t tmp = baseAddr->C0;
    baseAddr->C0 &= ~(CMP_C0_SE_MASK & CMP_C0_FPR_MASK & CMP_C0_FILTER_CNT_MASK);
    tmp &= ~(CMP_C0_FPR_MASK);
    tmp |= CMP_C0_FPR(to_set);
    baseAddr->C0 = tmp;
}

/*!
 * @brief Verify if the sampling mode is selected
 * @param baseAddr - cmp base pointer
 * @return - sampling mode state
 *  true - sampling mode is used
 *  false - sampling mode isn't used
 */
static inline bool CMP_HAL_GetSamplingState(CMP_Type* const baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->C0), CMP_C0_SE_SHIFT);
}

/*!
 * @brief Set the sampling mode state
 * @param baseAddr - cmp the base pointer
 * @param to_set - sampling mode state
 *  true - sampling mode is used
 *  false - sampling mode isn't used
 * @return - void
 */
static inline void CMP_HAL_SetSamplingState(CMP_Type* const baseAddr, bool to_set)
{
    BITBAND_ACCESS32(&(baseAddr->C0), CMP_C0_SE_SHIFT) = to_set;
}

/*!
 * @brif Verify if the windowing mode is selected
 * @param baseAddr - cmp base pointer
 * @return - windowing mode state
 *  true - windowing mode is used
 *  false - windowing mode isn't used
 */
static inline bool CMP_HAL_GetWindowingModeState(CMP_Type* const baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->C0), CMP_C0_WE_SHIFT);
}

/*!
 * @brief Set the windowing mode state
 * @param baseAddr - cmp base pointer
 * @param to_set - windowing mode state;
 *  true - windowing mode is used
 *  false - windowing mode isn't used
 * @return void
 */
static inline void CMP_HAL_SetWindowingModeState(CMP_Type* const baseAddr, bool to_set)
{
    BITBAND_ACCESS32(&(baseAddr->C0), CMP_C0_WE_SHIFT) = to_set;
}

/*!
 * @brief Return the current power mode
 * @param baseAddr - cmp base pointer
 * @return - current power mode 
 *  CMP_LOW_SPEED
 *  CMP_HIGH_SPEED
 */
static inline cmp_power_mode_t CMP_HAL_GetPowerMode(CMP_Type* const baseAddr)
{
    return (cmp_power_mode_t)BITBAND_ACCESS32(&(baseAddr->C0), CMP_C0_PMODE_SHIFT);
}

/*!
 * @brief Set the power mode
 * @param baseAddr - cmp base pointer
 * @param to_set - power mode
 *  CMP_LOW_SPEED
 *  CMP_HIGH_SPEED
 */
static inline void CMP_HAL_SetPowerMode(CMP_Type* const baseAddr, cmp_power_mode_t to_set)
{
    BITBAND_ACCESS32(&(baseAddr->C0), CMP_C0_PMODE_SHIFT) = to_set;
}

/*!
 * @brief Return the current comparator output inverter
 * @param baseAddr cmp base pointer
 * @return - inverter state
 *  CMP_NORMAL
 *  CMP_INVERT
 */
static inline cmp_inverter_t CMP_HAL_GetInverterState(CMP_Type* const baseAddr)
{
    return (cmp_inverter_t)BITBAND_ACCESS32(&(baseAddr->C0), CMP_C0_INVT_SHIFT);
}

/*!
 * @brief Configure the comparator output inverter mode
 * @param baseAddr - cmp base pointer
 * @param to_set - comparator output inverter mode
 *  CMP_NORMAL
 *  CMP_INVERT
 * @return - void
 */
static inline void CMP_HAL_SetInverterState(CMP_Type* const baseAddr, cmp_inverter_t to_set)
{
    BITBAND_ACCESS32(&(baseAddr->C0), CMP_C0_INVT_SHIFT) = to_set;
}

/*!
 * @brief Return the current comparator output selected
 * @param baseAddr - cmp base pointer
 * @return - comparator output signal source
 *  CMP_COUT
 *  CMP_COUTA
 */
static inline cmp_output_select_t CMP_HAL_GetComparatorOutputSource(CMP_Type* const baseAddr)
{
    return (cmp_output_select_t)BITBAND_ACCESS32(&(baseAddr->C0), CMP_C0_COS_SHIFT);
}

/*!
 * @brief Select the comparator output signal source
 * @param baseAddr - cmp base pointer
 * @param to_set - comparator output signal source
 *  CMP_COUT
 *  CMP_COUTA
 * @return void
 */
static inline void CMP_HAL_SetComparatorOutputSource(CMP_Type* const baseAddr, cmp_output_select_t to_set)
{
    BITBAND_ACCESS32(&(baseAddr->C0), CMP_C0_COS_SHIFT) = to_set;
}

/*!
 * @brief Verify if the comparator output state(available/not available in a packaged pin)
 * @param baseAddr - cmp base pointer
 * @return - comparator output state
 *  CMP_UNAVAILABLE
 *  CMP_AVAILABLE
 */
static inline cmp_output_enable_t CMP_HAL_GetOutputPinState(CMP_Type* const baseAddr)
{
    return (cmp_output_enable_t)BITBAND_ACCESS32(&(baseAddr->C0), CMP_C0_OPE_SHIFT);
}

/*!
 * @brief Set the comparator output pin state(available/not available in a packaged pin)
 * @param baseAddr - cmp base pointer
 * @param to_set - comparator output state
 *  CMP_UNAVAILABLE
 *  CMP_AVAILABLE
 * @return - void
 */
static inline void CMP_HAL_SetOutputPinState(CMP_Type* const baseAddr, cmp_output_enable_t to_set)
{

    BITBAND_ACCESS32(&(baseAddr->C0), CMP_C0_OPE_SHIFT) = to_set;
}

/*!
 * @brief Verify if the analog comparator module is enabled
 * @param baseAddr - cmp base pointer
 * @return - module state
 *  true - module is enabled
 *  false - module is disabled
 */
static inline bool CMP_HAL_GetAnalogComparatorState(CMP_Type* const baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->C0), CMP_C0_EN_SHIFT);
}

/*!
 * @brief Set the analog comparator module state
 * @param baseAddr - cmp base pointer
 * @param to_set - analog comparator module state
 *  true - module is enabled
 *  false - module is disabled
 */
static inline void CMP_HAL_SetAnalogComparatorState(CMP_Type* const baseAddr, bool to_set)
{
    BITBAND_ACCESS32(&(baseAddr->C0), CMP_C0_EN_SHIFT) = to_set;
}

/*!
 * @brief Return the number of consecutive samples that must agree prior to the comparator output filter
accepting a new output state
 * @param baseAddr - cmp base pointer
 * @return - filter sample count
 */
static inline uint8_t CMP_HAL_GetFilterSampleCount(CMP_Type* const baseAddr)
{
    uint32_t tmp = baseAddr->C0;
    tmp = (tmp & CMP_C0_FILTER_CNT_MASK) >> CMP_C0_FILTER_CNT_SHIFT;
    return ( uint8_t ) (tmp);
}

/*!
 * @brief Set the number of consecutive samples that must agree prior to the comparator output filter
accepting a new output state
 * @param baseAddr - cmp base pointer
 * @param to_set - filter sample count(min value 0, max value 7)
 * @return - void
 */
static inline void CMP_HAL_SetFilterSampleCount(CMP_Type* const baseAddr, uint8_t to_set)
{
    uint32_t tmp = baseAddr->C0;
    baseAddr->C0 &= ~(CMP_C0_SE_MASK & CMP_C0_FPR_MASK & CMP_C0_FILTER_CNT_MASK);
    tmp &= ~(CMP_C0_FILTER_CNT_MASK);
    tmp |= CMP_C0_FILTER_CNT(to_set);
    baseAddr->C0 = tmp;
}

/*!
 * @brief Return the current offset level
 * @param baseAddr - cmp base pointer
 * @return - offset level
 *  CMP_LEVEL_OFFSET_0
 *  CMP_LEVEL_OFFSET_1
 */
static inline cmp_offset_t CMP_HAL_GetOffset(CMP_Type* const baseAddr)
{
    return (cmp_offset_t)BITBAND_ACCESS32(&(baseAddr->C0), CMP_C0_OFFSET_SHIFT);
}

/*!
 * @brief Set the offset level
 * @param baseAddr - cmp base pointer
 * @param to_set - offset level
 *  CMP_LEVEL_OFFSET_0
 *  CMP_LEVEL_OFFSET_1
 * @return - void
 */
static inline void CMP_HAL_SetOffset(CMP_Type* const baseAddr, cmp_offset_t to_set)
{
    uint32_t tmp = baseAddr->C0;
    tmp &= ~(CMP_C0_OFFSET_MASK);
    tmp |= CMP_C0_OFFSET(to_set);
    baseAddr->C0 = tmp;
}

/*!
 * @brief Return the current hysteresis level
 * @param baseAddr - cmp base pointer
 * @return - current hysteresis level
 *  CMP_LEVEL_HYS_0
 *  CMP_LEVEL_HYS_1
 *  CMP_LEVEL_HYS_2
 *  CMP_LEVEL_HYS_3
 */
static inline cmp_hysteresis_t CMP_HAL_GetHysteresis(CMP_Type* const baseAddr)
{
    uint32_t tmp = baseAddr->C0;
    tmp = (tmp & CMP_C0_HYSTCTR_MASK) >> CMP_C0_HYSTCTR_SHIFT;
    return ( cmp_hysteresis_t ) (tmp);
}

/*!
 * @brief Set the hysteresis level
 * @param baseAddr - cmp base pointer
 * @param to_set - hysteresis level
 *  CMP_LEVEL_HYS_0
 *  CMP_LEVEL_HYS_1
 *  CMP_LEVEL_HYS_2
 *  CMP_LEVEL_HYS_3
 * @return - void
 */
static inline void CMP_HAL_SetHysteresis(CMP_Type* const baseAddr, cmp_hysteresis_t to_set)
{
    uint32_t tmp = baseAddr->C0;
    tmp &= ~(CMP_C0_HYSTCTR_MASK);
    tmp |= CMP_C0_HYSTCTR(to_set);
    baseAddr->C0 = tmp;
}


/*!
 * @brief Set if the DAC output is enabled to go outside of this block
 * @param baseAddr - cmp base pointer
 * @param to_set - DAC output state
 *  true - DAC output go outside of DAC block(to packaged pin)
 *  false - DAC output doesn't go outside of DAC block(to packaged pin)
 * @return - void
 */
static inline void CMP_HAL_SetDACOutputState(CMP_Type* const baseAddr, bool to_set)
{
    BITBAND_ACCESS32(&(baseAddr->C0), CMP_C0_EN_SHIFT) = to_set;
}

/*!
 * @brief Return the current source for positive port of the comparator
 * @param baseAddr - cmp base pointer
 * @return - signal source
 *  CMP_DAC
 *  CMP_MUX
 */
static inline cmp_port_mux_t CMP_HAL_GetPositivePortInput(CMP_Type* const baseAddr)
{
    uint32_t tmp = baseAddr->C1;
    tmp = (tmp & CMP_C1_INPSEL_MASK) >> CMP_C1_INPSEL_SHIFT;
    return ( cmp_port_mux_t ) (tmp);
}

/*!
 * @brief Set the source for positive port of the comparator
 * @param baseAddr cmp base pointer
 * @param to_set - signal source
 *  CMP_DAC
 *  CMP_MUX
 * @return - void
 */
static inline void CMP_HAL_SetPositivePortInput(CMP_Type* const baseAddr, cmp_port_mux_t to_set)
{
    uint32_t tmp = baseAddr->C1;
    tmp &= ~(CMP_C1_INPSEL_MASK);
    tmp |= CMP_C1_INPSEL(to_set);
    baseAddr->C1 = tmp;
}

/*!
 * @brief Return the current source for negative port of the comparator
 * @param baseAddr - cmp base pointer
 * @return - signal source
 *  CMP_PLUS_FIXED
 *  CMP_MINUS_FIXED
 */
static inline cmp_port_mux_t CMP_HAL_GetNegativePortInput(CMP_Type* const baseAddr)
{
    uint32_t tmp = baseAddr->C1;
    tmp = (tmp & CMP_C1_INNSEL_MASK) >> CMP_C1_INNSEL_SHIFT;
    return ( cmp_port_mux_t ) (tmp);
}

/*!
 * @brief Set the source for negative port of the comparator
 * @param baseAddr - cmp base pointer
 * @param to_set - signal source
 *  CMP_PLUS_FIXED
 *  CMP_MINUS_FIXED
 * @return - void
 */
static inline void CMP_HAL_SetNegativePortInput(CMP_Type* const baseAddr, cmp_port_mux_t to_set)
{
    uint32_t tmp = baseAddr->C1;
    tmp &= ~(CMP_C1_INNSEL_MASK);
    tmp |= CMP_C1_INNSEL(to_set);
    baseAddr->C1 = tmp;
}

/*!
 * @brief Return which channels are used for round-robin checker
 * @param baseAddr - cmp base pointer
 * @return - channels states, one bite for each channel state 
 * |---------|---------|-----|---------|---------|
 * |CH7_state|CH6_state|.....|CH1_state|CH0_state|
 * |---------|---------|-----|---------|---------|
 */
static inline cmp_ch_list_t CMP_HAL_GetRoundRobinChannels(CMP_Type* const baseAddr)
{
    uint32_t tmp = baseAddr->C2;
    tmp &= CMP_ROUND_ROBIN_CHANNELS_MASK;
    return(cmp_ch_list_t) tmp >> CMP_ROUND_ROBIN_CHANNELS_SHIFT;
}

/*!
 * @brief Set which channels are use for round-robin checker
 * @param baseAddr - cmp base pointer
 * @param to_set - channels states, one bite for each channel state
 * |---------|---------|-----|---------|---------|
 * |CH7_state|CH6_state|.....|CH1_state|CH0_state|
 * |---------|---------|-----|---------|---------|
 * @return - void
 */
static inline void CMP_HAL_SetRoundRobinChannels(CMP_Type* const baseAddr, cmp_ch_list_t to_set)
{
    uint32_t tmp = baseAddr->C1;
    tmp &= ~(CMP_ROUND_ROBIN_CHANNELS_MASK);
    tmp |= to_set << CMP_ROUND_ROBIN_CHANNELS_SHIFT;
    baseAddr->C1 = tmp;
}

/*!
 * @brief Verify if the DAC is enabled
 * @param baseAddr - cmp base pointer
 * @return - dac state
 *  true - DAC is enabled
 *  false - DAC is disabled
 */
static inline bool CMP_HAL_GetDACState(CMP_Type* const baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->C1), CMP_C1_DACEN_SHIFT);
}

/*!
 * @brief Set the DAC state (enabled/disabled)
 * @param baseAddr - cmp base pointer
 * @param to_set - DAC state
 *  true - DAC is enabled
 *  false - DAC is disabled
 * @return - void
 */
static inline void CMP_HAL_SetDACState(CMP_Type* const baseAddr, bool to_set)
{
    BITBAND_ACCESS32(&(baseAddr->C1), CMP_C1_DACEN_SHIFT) = to_set;
}

/*!
 * @brief Return the current voltage reference
 * @param baseAddr - cmp base pointer
 * @return - voltage referece
 *  CMP_VIN1
 *  CMP_VIN2
 */
static inline cmp_voltage_reference_t CMP_HAL_GetVoltageReference(CMP_Type* const baseAddr)
{
    return (cmp_voltage_reference_t)BITBAND_ACCESS32(&(baseAddr->C1), CMP_C1_VRSEL_SHIFT);
}

/*!
 * @brief Set the voltage reference
 * @param baseAddr - cmp base pointer
 * @param to_set - voltage reference
 *  CMP_VIN1
 *  CMP_VIN2
 * @return - void
 */
static inline void CMP_HAL_SetVoltageReference(CMP_Type* const baseAddr, cmp_voltage_reference_t to_set)
{
    BITBAND_ACCESS32(&(baseAddr->C1), CMP_C1_VRSEL_SHIFT) = to_set;
}

/*!
 * @brief Determine which input is selected for the plus mux
 * @param baseAddr - cmp base pointer
 * @return - channel for the plus mux
 */
static inline cmp_ch_number_t CMP_HAL_GetPlusMUXControl(CMP_Type* const baseAddr)
{
    uint32_t tmp = baseAddr->C1;
    tmp = (tmp & CMP_C1_PSEL_MASK) >> CMP_C1_PSEL_SHIFT;
    return ( cmp_ch_number_t ) (tmp);
}

/*!
 * @brief Select input for the plus mux
 * @param baseAddr cmp base pointer
 * @param to_set - channel for the plus mux
 * @return - void
 */
static inline void CMP_HAL_SetPlusMuxControl(CMP_Type* const baseAddr, cmp_ch_number_t to_set)
{
    uint32_t tmp = baseAddr->C1;
    tmp &= ~(CMP_C1_PSEL_MASK);
    tmp |= CMP_C1_PSEL(to_set);
    baseAddr->C1 = tmp;
}

/*!
 * @brief Determine which input is selected for the minus mux
 * @param baseAddr - cmp base pointer
 * @return - channel for the minus mux
 */
static inline cmp_ch_number_t CMP_HAL_GetMinusMUXControl(CMP_Type* const baseAddr)
{
    uint32_t tmp = baseAddr->C1;
    tmp = (tmp & CMP_C1_MSEL_MASK) >> CMP_C1_MSEL_SHIFT;
    return ( cmp_ch_number_t ) (tmp);
}

/*!
 * @brief Select input for the minus mux
 * @param baseAddr - cmp base pointer
 * @param to_set - channel for the minus mux
 * @return - void
 */
static inline void CMP_HAL_SetMinusMUXControl(CMP_Type* const baseAddr, cmp_ch_number_t to_set)
{
    uint32_t tmp = baseAddr->C1;
    tmp &= ~(CMP_C1_MSEL_MASK);
    tmp |= CMP_C1_MSEL(to_set);
    baseAddr->C1 = tmp;
}

/*!
 * @brief Return the current output voltage level(0-255)
 * @param baseAddr - cmp base pointer
 * @return - voltage level
 */
static inline uint8_t CMP_HAL_GetVoltage(CMP_Type* const baseAddr)
{
    uint32_t tmp = baseAddr->C1;
    tmp = (tmp & CMP_C1_VOSEL_MASK) >> CMP_C1_VOSEL_SHIFT;
    return ( uint8_t ) (tmp);
}

/*!
 * @brief Set the output voltage level
 * @param baseAddr - cmp base pointer
 * @param to_set - voltage level
 * @return - void
 */
static inline void CMP_HAL_SetVoltage(CMP_Type* const baseAddr, uint8_t to_set)
{
    uint32_t tmp = baseAddr->C1;
    tmp &= ~(CMP_C1_VOSEL_MASK);
    tmp |= CMP_C1_VOSEL(to_set);
    baseAddr->C1 = tmp;
}

/*!
 * @brief Verify if the round robin operation is enabled
 * @param baseAddr -cmp base pointer
 * @return - round-robin operation state
 *  true - round robin operation is enabled
 *  false - round robin operation is disabled
 */
static inline bool CMP_HAL_GetRoundRobinState(CMP_Type* const baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->C2), CMP_C2_RRE_SHIFT);
}

/*!
 * @brief Set the round robin operation state
 * @param baseAddr cmp base pointer
 * @param to_set - round robin operation state
 *  true - round robin operation is enabled
 *  false - round robin operation is disabled
 * @return - void
 */
static inline void CMP_HAL_SetRoundRobinState(CMP_Type* const baseAddr, bool to_set)
{
    BITBAND_ACCESS32(&(baseAddr->C2), CMP_C2_RRE_SHIFT) = to_set;
}

/*!
 * @brief Verify if the round robin interrupt is enabled
 * @param baseAddr - cmp base pointer
 * @return - round-robin interrupt state
 *  true - round robin interrupt is enabled
 *  false - round robin interrupt is disabled
 */
static inline bool CMP_HAL_GetRoundRobinInterruptState(CMP_Type* const baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->C2), CMP_C2_RRIE_SHIFT);
}

/*!
 * @brief Set the round robin interrupt state
 * @param baseAddr - cmp base pointer
 * @param to_set - round robin interrupt state
 *  true - round robin interrupt is enabled
 *  false - round robin interrupt is disabled
 * @return - void
 */
static inline void CMP_HAL_SetRoundRobinInterruptState(CMP_Type* const baseAddr, bool to_set)
{
    BITBAND_ACCESS32(&(baseAddr->C2), CMP_C2_RRIE_SHIFT) = to_set;
}

/*!
 * @brief Return the port fixed for round-robin operation
 * @param baseAddr - cmp base pointer
 * @return - fixed port
 */
static inline cmp_fixed_port_t CMP_HAL_GetFixedPort(CMP_Type* const baseAddr)
{
    return (cmp_fixed_port_t)BITBAND_ACCESS32(&(baseAddr->C2), CMP_C2_FXMP_SHIFT);
}

/*!
 * @brief Set the fixed port for round-robin operation
 * @param baseAddr - cmp base pointer
 * @param to_set - fixed port
 *  CMP_PLUS_FIXED
 *  CMP_MINUS_FIXED
 * @return - void
 */
static inline void CMP_HAL_SetFixedPort(CMP_Type* const baseAddr, cmp_fixed_port_t to_set)
{
    BITBAND_ACCESS32(&(baseAddr->C2), CMP_C2_FXMP_SHIFT) = to_set;
}

/*!
 * @brief Return which channel is selected for fixed mux port(as fixed reference)
 * @param baseAddr - cmp base pointer
 * @return - fixed channel 
 *  CMP_PLUS_FIXED
 *  CMP_MINUS_FIXED
 */
static inline cmp_ch_number_t CMP_HAL_GetFixedChannel(CMP_Type* const baseAddr)
{
    uint32_t tmp = baseAddr->C2;
    tmp = (tmp & CMP_C2_FXMXCH_MASK) >> CMP_C2_FXMXCH_SHIFT;
    return ( cmp_ch_number_t ) (tmp);
}

/*!
 * @brief Set which channel is used as the fixed reference input for the fixed mux port
 * @param baseAddr - cmp base pointer
 * @param to_set - fixed channel
 * @return - void
 */
static inline void CMP_HAL_SetFixedChannel(CMP_Type* const baseAddr, cmp_ch_number_t to_set)
{
    uint32_t tmp = baseAddr->C2;
    tmp &= ~(CMP_C2_FXMXCH_MASK);
    tmp |= CMP_C2_FXMXCH(to_set);
    baseAddr->C2 = tmp;
}

/*!
 * @brief Return all input changed flags
 * @param baseAddr - cmp base pointer
 * @return - flags status
 * |--------|--------|-----|--------|--------|
 * |CH7_flag|CH6_flag|.....|CH1_flag|CH0_flag|
 * |--------|--------|-----|--------|--------|
 */
static inline cmp_ch_list_t CMP_HAL_GetInputChangedFlags(CMP_Type* const baseAddr)
{   
    uint32_t tmp = baseAddr->C2;
    tmp = (tmp & CMP_INPUT_FLAGS_MASK) >> CMP_INPUT_FLAGS_SHIFT;
    return ( cmp_ch_list_t ) (tmp);
}

/*!
 * @brief Clear all input changed flags
 * @param baseAddr - cmp base pointer
 * @return - void
 */
static inline void CMP_HAL_ClearInputChangedFlags(CMP_Type* const baseAddr)
{
    uint32_t tmp = baseAddr->C2;
    tmp |= CMP_INPUT_FLAGS_MASK;
    baseAddr->C2 = tmp;
}

/*!
 * @brief Return how many round-robin clock cycles takes sampling
 * @param baseAddr - cmp base pointer
 * @return - number of sample clocks
 */
static inline uint8_t CMP_HAL_GetRoundRobinSamplesNumber(CMP_Type* const baseAddr)
{
    uint32_t tmp = baseAddr->C2;
    tmp = (tmp & CMP_C2_NSAM_MASK) >> CMP_C2_NSAM_SHIFT;
    return ( uint8_t ) (tmp);
}

/*!
 * @brief Set how many round-robin clock cycles takes sampling
 * @param baseAddr - cmp base pointer
 * @param to_set - number of sample clocks(min value 0, max value 3)
 * @return - void
 */
static inline void CMP_HAL_SetRoundRobinSamplesNumber(CMP_Type* const baseAddr, uint8_t to_set)
{
    uint32_t tmp = baseAddr->C2;
    tmp &= ~(CMP_C2_NSAM_MASK);
    tmp |= CMP_C2_NSAM(to_set);
    baseAddr->C2 = tmp;
}

/*!
 * @brief Return the comparator and DAC initialization delay
 * @param baseAddr - cmp base pointer
 * @return - delay(round-robin clock period)
 */
static inline uint8_t CMP_HAL_GetInitDelay(CMP_Type* const baseAddr)
{
    uint32_t tmp = baseAddr->C2;
    tmp = (tmp & CMP_C2_INITMOD_MASK) >> CMP_C2_INITMOD_SHIFT;
    return ( uint8_t ) (tmp);
}

/*!
 * @brief Set the comparator and  DAC initialization delay
 * @param baseAddr - cmp base pointer
 * @param to_set - delay (min value 0, max value 63)
 * @return - void
 */
static inline void CMP_HAL_SetInitDelay(CMP_Type* const baseAddr, uint8_t to_set)
{
    uint32_t tmp = baseAddr->C2;
    tmp &= ~(CMP_C2_INITMOD_MASK);
    tmp |= CMP_C2_INITMOD(to_set);
    baseAddr->C2 = tmp;
}

/*!
 * @brief Return last input comparison results for all channels
 * @param baseAddr - cmp base pointer
 * @return - comparison results
 */
static inline cmp_ch_list_t CMP_HAL_GetLastComparisonResult(CMP_Type* const baseAddr)
{
    uint32_t tmp = baseAddr->C2;
    tmp = (tmp & CMP_C2_ACOn_MASK) >> CMP_C2_ACOn_SHIFT;
    return ( cmp_ch_list_t ) (tmp);
}

/*!
 * @brief Defines the pre-set state of input channels.
 * @param baseAddr cmp base pointer
 * @param to_set - state
 * @return void
 */
static inline void CMP_HAL_SetPresetState(CMP_Type* const baseAddr, cmp_ch_list_t to_set)
{
    uint32_t tmp = baseAddr->C2;
    tmp &= ~(CMP_C2_ACOn_MASK);
    tmp |= CMP_C2_ACOn(to_set);
    baseAddr->C2 = tmp;
}

/*! @}*/
#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __FSL_CMP_HAL_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/

