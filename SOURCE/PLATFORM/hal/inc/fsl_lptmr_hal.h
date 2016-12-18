/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
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

#ifndef __FSL_LPTMR_HAL_H__
#define __FSL_LPTMR_HAL_H__

#include <stdint.h>
#include <stdbool.h>
#include "fsl_device_registers.h"

/*! @file */

/*!
 * @addtogroup lptmr_hal
 * @{
 */

 /*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief Pulse Counter Input selection */
typedef enum _lptmr_pinselect_t {
    LPTMR_PINSELECT_TRGMUX  = 0x00u, /*!< Count pulses from TRGMUX trigger */
    LPTMR_PINSELECT_ALT1    = 0x01u, /*!< Count pulses from pin alternative 1 */
    LPTMR_PINSELECT_ALT2    = 0x02u, /*!< Count pulses from pin alternative 2 */
    LPTMR_PINSELECT_ALT3    = 0x03u  /*!< Count pulses from pin alternative 3 */
} lptmr_pinselect_t;

/*! @brief Pulse Counter input polarity */
typedef enum _lptmr_pinpolarity_t {
    LPTMR_PINPOLARITY_RISING    = 0u, /*!< Count pulse on rising edge */
    LPTMR_PINPOLARITY_FALLING   = 1u  /*!< Count pulse on falling edge */
} lptmr_pinpolarity_t;

/*! @brief Work Mode */
typedef enum _lptmr_workmode_t {
    LPTMR_WORKMODE_TIMER        = 0u, /*!< Timer */
    LPTMR_WORKMODE_PULSECOUNTER = 1u  /*!< Pulse counter */
} lptmr_workmode_t;

/*! @brief Prescaler Selection */
typedef enum _lptmr_prescaler_t {
    LPTMR_PRESCALE_2                        = 0x00u, /*!< Timer mode: prescaler 2, Glitch filter mode: invalid */
    LPTMR_PRESCALE_4_GLITCHFILTER_2         = 0x01u, /*!< Timer mode: prescaler 4, Glitch filter mode: 2 clocks */
    LPTMR_PRESCALE_8_GLITCHFILTER_4         = 0x02u, /*!< Timer mode: prescaler 8, Glitch filter mode: 4 clocks */
    LPTMR_PRESCALE_16_GLITCHFILTER_8        = 0x03u, /*!< Timer mode: prescaler 16, Glitch filter mode: 8 clocks */
    LPTMR_PRESCALE_32_GLITCHFILTER_16       = 0x04u, /*!< Timer mode: prescaler 32, Glitch filter mode: 16 clocks */
    LPTMR_PRESCALE_64_GLITCHFILTER_32       = 0x05u, /*!< Timer mode: prescaler 64, Glitch filter mode: 32 clocks */
    LPTMR_PRESCALE_128_GLITCHFILTER_64      = 0x06u, /*!< Timer mode: prescaler 128, Glitch filter mode: 64 clocks */
    LPTMR_PRESCALE_256_GLITCHFILTER_128     = 0x07u, /*!< Timer mode: prescaler 256, Glitch filter mode: 128 clocks */
    LPTMR_PRESCALE_512_GLITCHFILTER_256     = 0x08u, /*!< Timer mode: prescaler 512, Glitch filter mode: 256 clocks */
    LPTMR_PRESCALE_1024_GLITCHFILTER_512    = 0x09u, /*!< Timer mode: prescaler 1024, Glitch filter mode: 512 clocks */
    LPTMR_PRESCALE_2048_GLITCHFILTER_1024   = 0x0Au, /*!< Timer mode: prescaler 2048, Glitch filter mode: 1024 clocks */
    LPTMR_PRESCALE_4096_GLITCHFILTER_2048   = 0x0Bu, /*!< Timer mode: prescaler 4096, Glitch filter mode: 2048 clocks */
    LPTMR_PRESCALE_8192_GLITCHFILTER_4096   = 0x0Cu, /*!< Timer mode: prescaler 8192, Glitch filter mode: 4096 clocks */
    LPTMR_PRESCALE_16384_GLITCHFILTER_8192  = 0x0Du, /*!< Timer mode: prescaler 16384, Glitch filter mode: 8192 clocks */
    LPTMR_PRESCALE_32768_GLITCHFILTER_16384 = 0x0Eu, /*!< Timer mode: prescaler 32768, Glitch filter mode: 16384 clocks */
    LPTMR_PRESCALE_65536_GLITCHFILTER_32768 = 0x0Fu  /*!< Timer mode: prescaler 65536, Glitch filter mode: 32768 clocks */
} lptmr_prescaler_t;

/*! @brief Clock Source selection */
typedef enum _lptmr_clocksource_t {
    LPTMR_CLOCKSOURCE_SIRC      = 0x00u, /*!< SIRC clock */
    LPTMR_CLOCKSOURCE_1KHZ_LPO  = 0x01u, /*!< 1kHz LPO clock */
    LPTMR_CLOCKSOURCE_32KHZ_LPO = 0x02u, /*!< 32kHz LPO clock */
    LPTMR_CLOCKSOURCE_PCC       = 0x03u  /*!< PCC configured clock */
} lptmr_clocksource_t;
/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @name LPTMR
 * @{
 */

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initialize the LPTMR instance to reset values.
 *
 * This function initializes the LPTMR instance to a known state (the register
 * are written with their reset values from the Reference Manual).
 *
 * @param[in] base lptmr base pointer
 */
void LPTMR_HAL_Init(LPTMR_Type* const base);


/*!
 * @brief Get the DMA Request Enable Flag
 *
 * This function checks whether a DMA Request feature of the LPTMR is enabled.
 * The DMA Request is issued when a Compare Match is asserted. If enabled, the
 * Compare Match/Interrupt Pending flag is cleared when the DMA controller is
 * done.
 *
 * @param[in] base lptmr base pointer
 * @return DMA Request enable
 *      - true: enable DMA Request
 *      - false: disable DMA Request
 */
static inline bool LPTMR_HAL_GetDmaRequest(const LPTMR_Type* const base)
{
    return (bool)BITBAND_ACCESS8(&(base->CSR), LPTMR_CSR_TDRE_SHIFT);
}


/*!
 * @brief Configure the DMA Request Enable Flag state
 *
 * This function configures the DMA Request feature of the LPTMR. If enabled,
 * a DMA Request is issued when the Compare Match event occurs. If enabled, the
 * Compare Match/Interrupt Pending flag is cleared when the DMA controller is
 * done.
 *
 * @param[in] base lptmr base pointer
 * @param[in] enable the new state of the DMA Request Enable Flag
 *      - true: enable DMA Request
 *      - false: disable DMA Request
 */
static inline void LPTMR_HAL_SetDmaRequest(LPTMR_Type* const base, bool enable)
{
    BITBAND_ACCESS8(&(base->CSR), LPTMR_CSR_TDRE_SHIFT) = (uint32_t)enable;
}


/*!
 * @brief Get the Compare Flag state
 *
 * This function checks whether a Compare Match event has occurred or if there is
 * an Interrupt Pending.
 *
 * @param[in] base lptmr base pointer
 * @return the Compare Flag state
 *      - true: Compare Match/Interrupt Pending asserted
 *      - false: Compare Match/Interrupt Pending not asserted
 */
static inline bool LPTMR_HAL_GetCompareFlag(const LPTMR_Type* const base)
{
    return (bool)BITBAND_ACCESS8(&(base->CSR), LPTMR_CSR_TCF_SHIFT);
}


/*!
 * @brief Clear the Compare Flag
 *
 * This function clears the Compare Flag/Interrupt Pending state.
 *
 * @param[in] base lptmr base pointer
 */
static inline void LPTMR_HAL_ClearCompareFlag(LPTMR_Type* const base)
{
    BITBAND_ACCESS8(&(base->CSR), LPTMR_CSR_TCF_SHIFT) = 1u;
    /* If the flag is cleared in the last instruction in the ISR, the interrupt
    might be triggered a second time. This ensures that the flag clears before
    continuing.*/
    __asm("dsb");
}


/*!
 * @brief Get the Interrupt Enable state
 *
 * This function returns the Interrupt Enable state for the LPTMR. If enabled,
 * an interrupt is generated when a Compare Match event occurs.
 *
 * @param[in] base lptmr base pointer
 * @return Interrupt Enable state
 *      - true: Interrupt enabled
 *      - false: Interrupt disabled
 */
static inline bool LPTMR_HAL_GetInterruptEnable(const LPTMR_Type* const base)
{
    return (bool)BITBAND_ACCESS8(&(base->CSR), LPTMR_CSR_TIE_SHIFT);
}


/*!
 * @brief Configure the Interrupt Enable state
 *
 * This function configures the Interrupt Enable state for the LPTMR. If enabled,
 * an interrupt is generated when a Compare Match event occurs.
 *
 * @param[in] base lptmr base pointer
 * @param[in] enable the new state for the interrupt
 *          - true: enable Interrupt
 *          - false: disable Interrupt
 */
static inline void LPTMR_HAL_SetInterrupt(LPTMR_Type* const base, bool enable)
{
    uint32_t tmp = base->CSR;
    /* Clear the affected bitfield and write '0' to the w1c bits to avoid side-effects */
    tmp &= ~(LPTMR_CSR_TIE_MASK | LPTMR_CSR_TCF_MASK);
    tmp |= LPTMR_CSR_TIE(((uint32_t)enable));
    base->CSR = tmp;
}


/*!
 * @brief Get the Pin select for Counter Mode
 *
 * This function returns the configured Input Pin for Pulse Counter Mode.
 *
 * @param[in] base lptmr base pointer
 * @return Input pin selection
 *          - LPTMR_PINSELECT_TRGMUX: count pulses from TRGMUX output
 *          - LPTMR_PINSELECT_ALT1: count pulses from pin alt 1
 *          - LPTMR_PINSELECT_ALT2: count pulses from pin alt 2
 *          - LPTMR_PINSELECT_ALT3: count pulses from pin alt 3
 */
static inline lptmr_pinselect_t LPTMR_HAL_GetPinSelect(const LPTMR_Type* const base)
{
    uint32_t tmp = base->CSR;
    tmp = (tmp & LPTMR_CSR_TPS_MASK) >> LPTMR_CSR_TPS_SHIFT;
    return (lptmr_pinselect_t)(tmp);
}


/*!
 * @brief Configure the Pin selection for Pulse Counter Mode
 *
 * This function configures the input Pin selection for Pulse Counter Mode.
 * This feature can be configured only when the LPTMR is disabled.
 *
 * @param[in] base lptmr base pointer
 * @param[in] pinsel pin selection
 *          - LPTMR_PINSELECT_TRGMUX: count pulses from TRGMUX output
 *          - LPTMR_PINSELECT_ALT1: count pulses from pin alt 1
 *          - LPTMR_PINSELECT_ALT2: count pulses from pin alt 2
 *          - LPTMR_PINSELECT_ALT3: count pulses from pin alt 3
 */
static inline void LPTMR_HAL_SetPinSelect(LPTMR_Type* const base, const lptmr_pinselect_t pinsel)
{
    uint32_t tmp = base->CSR;
    /* Clear the affected bitfield and write '0' to the w1c bits to avoid side-effects */
    tmp &= ~(LPTMR_CSR_TPS_MASK | LPTMR_CSR_TCF_MASK);
    tmp |= LPTMR_CSR_TPS(pinsel);
    base->CSR = tmp;
}


/*!
 * @brief Get Pin Polarity for Pulse Counter Mode
 *
 * This function returns the configured pin polarity that triggers an increment
 * in Pulse Counter Mode.
 *
 * @param[in] base lptmr base pointer
 * @return the pin polarity for Pulse Counter Mode
 *          - LPTMR_PINPOLARITY_RISING: count pulse on Rising Edge
 *          - LPTMR_PINPOLARITY_FALLING: count pulse on Falling Edge
 */
static inline lptmr_pinpolarity_t LPTMR_HAL_GetPinPolarity(const LPTMR_Type* const base)
{
    return (lptmr_pinpolarity_t)BITBAND_ACCESS8(&(base->CSR), LPTMR_CSR_TPP_SHIFT);
}


/*!
 * @brief Configure Pin Polarity for Pulse Counter Mode
 *
 * This function configures the pin polarity that triggers an increment in Pulse
 * Counter Mode. This feature can be configured only when the LPTMR is disabled.
 *
 * @param[in] base lptmr base pointer
 * @param[in] pol the pin polarity to count in Pulse Counter Mode
 *          - LPTMR_PINPOLARITY_RISING: count pulse on Rising Edge
 *          - LPTMR_PINPOLARITY_FALLING: count pulse on Falling Edge
 */
static inline void LPTMR_HAL_SetPinPolarity(LPTMR_Type* const base, const lptmr_pinpolarity_t pol)
{
    uint32_t tmp = base->CSR;
    /* Clear the affected bitfield and write '0' to the w1c bits to avoid side-effects */
    tmp &= ~(LPTMR_CSR_TPP_MASK | LPTMR_CSR_TCF_MASK);
    tmp |= LPTMR_CSR_TPP(pol);
    base->CSR = tmp;
}


/*!
 * @brief Get the Free Running state
 *
 * This function checks whether the Free Running feature of the LPTMR is enabled
 * or disabled.
 *
 * @param[in] base lptmr base pointer
 * @return Free running mode state
 *          - true: Free Running Mode enabled. Reset counter on 16-bit overflow
 *          - false: Free Running Mode disabled. Reset counter on Compare Match.
 */
static inline bool LPTMR_HAL_GetFreeRunning(const LPTMR_Type* const base)
{
    return (bool)BITBAND_ACCESS8(&(base->CSR), LPTMR_CSR_TFC_SHIFT);
}


/*!
 * @brief Configure the Free Running state
 *
 * This function configures the Free Running feature of the LPTMR. This feature
 * can be configured only when the LPTMR is disabled.
 *
 * @param[in] base lptmr base pointer
 * @param[in] enable the new Free Running state
 *          - true: Free Running Mode enabled. Reset counter on 16-bit overflow
 *          - false: Free Running Mode disabled. Reset counter on Compare Match.
 */
static inline void LPTMR_HAL_SetFreeRunning(LPTMR_Type* const base, const bool enable)
{
    uint32_t tmp = base->CSR;
    /* Clear the affected bitfield and write '0' to the w1c bits to avoid side-effects */
    tmp &= ~(LPTMR_CSR_TFC_MASK | LPTMR_CSR_TCF_MASK);
    tmp |= LPTMR_CSR_TFC(((uint32_t)enable));
    base->CSR = tmp;
}


/*!
 * @brief Get current Work Mode
 *
 * This function returns the currently configured Work Mode for the LPTMR.
 *
 *
 * @param[in] base lptmr base pointer
 * @return Work Mode
 *          - LPTMR_WORKMODE_TIMER: LPTMR is in Timer Mode
 *          - LPTMR_WORKMODE_PULSECOUNTER: LPTMR is in Pulse Counter Mode
 */
static inline lptmr_workmode_t LPTMR_HAL_GetWorkMode(const LPTMR_Type* const base)
{
    return (lptmr_workmode_t)BITBAND_ACCESS8(&(base->CSR), LPTMR_CSR_TMS_SHIFT);
}


/*!
 * @brief Configure the Work Mode
 *
 * This function configures the LPTMR to either Timer Mode or Pulse Counter
 * Mode. This feature can be configured only when the LPTMR is disabled.
 *
 * @param[in] base lptmr base pointer
 * @param[in] mode new Work Mode
 *          - LPTMR_WORKMODE_TIMER: LPTMR set to Timer Mode
 *          - LPTMR_WORKMODE_PULSECOUNTER: LPTMR set to Pulse Counter Mode
 */
static inline void LPTMR_HAL_SetWorkMode(LPTMR_Type* const base, const lptmr_workmode_t mode)
{
    uint32_t tmp = base->CSR;
    /* Clear the affected bitfield and write '0' to the w1c bits to avoid side-effects */
    tmp &= ~(LPTMR_CSR_TMS_MASK | LPTMR_CSR_TCF_MASK);
    tmp |= LPTMR_CSR_TMS(mode);
    base->CSR = tmp;
}


/*!
 * @brief Get the Enable state.
 *
 * Prior to reconfiguring the LPTMR, it is necessary to disable it.
 *
 * @param[in] base lptmr base pointer
 * @return The state of the LPTMR
 *          - true: LPTMR enabled
 *          - false: LPTMR disabled
 */
static inline bool LPTMR_HAL_GetEnable(const LPTMR_Type* const base)
{
    return (bool)BITBAND_ACCESS8(&(base->CSR), LPTMR_CSR_TEN_SHIFT);
}


/*!
 * @brief Enable the LPTMR
 *
 * Enable the LPTMR. Starts the timer/counter.
 *
 *
 * @param[in] base lptmr base pointer
 */
static inline void LPTMR_HAL_Enable(LPTMR_Type* const base)
{
    uint32_t tmp = base->CSR;
    /* Clear the affected bitfield and write '0' to the w1c bits to avoid side-effects */
    tmp &= ~(LPTMR_CSR_TEN_MASK | LPTMR_CSR_TCF_MASK);
    tmp |= LPTMR_CSR_TEN(1u);
    base->CSR = tmp;
}


/*!
 * @brief Disable the LPTMR
 *
 * Disable the LPTMR. Stop the Counter/Timer and allow reconfiguration.
 *
 * @param[in] base lptmr base pointer
 */
static inline void LPTMR_HAL_Disable(LPTMR_Type* const base)
{
    uint32_t tmp = base->CSR;
    /* Clear the affected bitfield and write '0' to the w1c bits to avoid side-effects */
    tmp &= ~(LPTMR_CSR_TEN_MASK | LPTMR_CSR_TCF_MASK);
    tmp |= LPTMR_CSR_TEN(0u);
    base->CSR = tmp;
}


/*!
 * @brief Get Prescaler/Glitch Filter divider value
 *
 * This function returns the currently configured Prescaler/Glitch Filter divider
 * value.
 *
 * @param[in] base lptmr base pointer
 * @return The Prescaler/Glitch filter value
 *          - LPTMR_PRESCALE_2: Timer mode: prescaler 2, Glitch filter mode: invalid
 *          - LPTMR_PRESCALE_4_GLITCHFILTER_2: Timer mode: prescaler 4, Glitch filter mode: 2 clocks
 *          - LPTMR_PRESCALE_8_GLITCHFILTER_4: Timer mode: prescaler 8, Glitch filter mode: 4 clocks
 *          - LPTMR_PRESCALE_16_GLITCHFILTER_8: Timer mode: prescaler 16, Glitch filter mode: 8 clocks
 *          - LPTMR_PRESCALE_32_GLITCHFILTER_16: Timer mode: prescaler 32, Glitch filter mode: 16 clocks
 *          - LPTMR_PRESCALE_64_GLITCHFILTER_32: Timer mode: prescaler 64, Glitch filter mode: 32 clocks
 *          - LPTMR_PRESCALE_128_GLITCHFILTER_64: Timer mode: prescaler 128, Glitch filter mode: 64 clocks
 *          - LPTMR_PRESCALE_256_GLITCHFILTER_128: Timer mode: prescaler 256, Glitch filter mode: 128 clocks
 *          - LPTMR_PRESCALE_512_GLITCHFILTER_256: Timer mode: prescaler 512, Glitch filter mode: 256 clocks
 *          - LPTMR_PRESCALE_1024_GLITCHFILTER_512: Timer mode: prescaler 1024, Glitch filter mode: 512 clocks
 *          - LPTMR_PRESCALE_2048_GLITCHFILTER_1024: Timer mode: prescaler 2048, Glitch filter mode: 1024 clocks
 *          - LPTMR_PRESCALE_4096_GLITCHFILTER_2048: Timer mode: prescaler 4096, Glitch filter mode: 2048 clocks
 *          - LPTMR_PRESCALE_8192_GLITCHFILTER_4096: Timer mode: prescaler 8192, Glitch filter mode: 4096 clocks
 *          - LPTMR_PRESCALE_16384_GLITCHFILTER_8192: Timer mode: prescaler 16384, Glitch filter mode: 8192 clocks
 *          - LPTMR_PRESCALE_32768_GLITCHFILTER_16384: Timer mode: prescaler 32768, Glitch filter mode: 16384 clocks
 *          - LPTMR_PRESCALE_65536_GLITCHFILTER_32768: Timer mode: prescaler 65536, Glitch filter mode: 32768 clocks
 */
static inline lptmr_prescaler_t LPTMR_HAL_GetPrescaler(const LPTMR_Type* const base)
{
    uint32_t tmp = base->PSR;
    tmp = (tmp & LPTMR_PSR_PRESCALE_MASK) >> LPTMR_PSR_PRESCALE_SHIFT;
    return (lptmr_prescaler_t)(tmp);
}


/*!
 * @brief Configure the Prescaler/Glitch Filter divider value
 *
 * This function configures the value for the Prescaler/Glitch Filter. This
 * feature can be configured only when the LPTMR is disabled.
 *
 * @param[in] base lptmr base pointer
 * @param[in] presc the new Prescaler value
 *          - LPTMR_PRESCALE_2: Timer mode: prescaler 2, Glitch filter mode: invalid
 *          - LPTMR_PRESCALE_4_GLITCHFILTER_2: Timer mode: prescaler 4, Glitch filter mode: 2 clocks
 *          - LPTMR_PRESCALE_8_GLITCHFILTER_4: Timer mode: prescaler 8, Glitch filter mode: 4 clocks
 *          - LPTMR_PRESCALE_16_GLITCHFILTER_8: Timer mode: prescaler 16, Glitch filter mode: 8 clocks
 *          - LPTMR_PRESCALE_32_GLITCHFILTER_16: Timer mode: prescaler 32, Glitch filter mode: 16 clocks
 *          - LPTMR_PRESCALE_64_GLITCHFILTER_32: Timer mode: prescaler 64, Glitch filter mode: 32 clocks
 *          - LPTMR_PRESCALE_128_GLITCHFILTER_64: Timer mode: prescaler 128, Glitch filter mode: 64 clocks
 *          - LPTMR_PRESCALE_256_GLITCHFILTER_128: Timer mode: prescaler 256, Glitch filter mode: 128 clocks
 *          - LPTMR_PRESCALE_512_GLITCHFILTER_256: Timer mode: prescaler 512, Glitch filter mode: 256 clocks
 *          - LPTMR_PRESCALE_1024_GLITCHFILTER_512: Timer mode: prescaler 1024, Glitch filter mode: 512 clocks
 *          - LPTMR_PRESCALE_2048_GLITCHFILTER_1024: Timer mode: prescaler 2048, Glitch filter mode: 1024 clocks
 *          - LPTMR_PRESCALE_4096_GLITCHFILTER_2048: Timer mode: prescaler 4096, Glitch filter mode: 2048 clocks
 *          - LPTMR_PRESCALE_8192_GLITCHFILTER_4096: Timer mode: prescaler 8192, Glitch filter mode: 4096 clocks
 *          - LPTMR_PRESCALE_16384_GLITCHFILTER_8192: Timer mode: prescaler 16384, Glitch filter mode: 8192 clocks
 *          - LPTMR_PRESCALE_32768_GLITCHFILTER_16384: Timer mode: prescaler 32768, Glitch filter mode: 16384 clocks
 *          - LPTMR_PRESCALE_65536_GLITCHFILTER_32768: Timer mode: prescaler 65536, Glitch filter mode: 32768 clocks
 */
static inline void LPTMR_HAL_SetPrescaler(LPTMR_Type* const base, const lptmr_prescaler_t presc)
{
    uint32_t tmp = base->PSR;
    tmp &= ~(LPTMR_PSR_PRESCALE_MASK);
    tmp |= LPTMR_PSR_PRESCALE(presc);
    base->PSR = tmp;
}


/*!
 * @brief Get the Prescaler/Glitch Filter Bypass enable state
 *
 * This function checks whether the Prescaler/Glitch Filter Bypass is enabled.
 *
 * @param[in] base lptmr base pointer
 * @return the Prescaler Bypass state
 *          - true: Prescaler/Glitch Filter Bypass enabled
 *          - false: Prescaler/Glitch Filter Bypass disabled
 */
static inline bool LPTMR_HAL_GetBypass(const LPTMR_Type* const base)
{
    return (bool)BITBAND_ACCESS8(&(base->PSR), LPTMR_PSR_PBYP_SHIFT);
}


/*!
 * @brief Configure the Prescaler/Glitch Filter Bypass enable state
 *
 * This function configures the Prescaler/Glitch filter Bypass. This feature
 * can be configured only when the LPTMR is disabled.
 *
 * @param[in] base lptmr base pointer
 * @param[in] enable the new Prescaler/Glitch Filter Bypass state
 *          - true: Prescaler/Glitch Filter Bypass enabled
 *          - false: Prescaler/Glitch Filter Bypass disabled
 */
static inline void LPTMR_HAL_SetBypass(LPTMR_Type* const base, const bool enable)
{
    BITBAND_ACCESS8(&(base->PSR), LPTMR_PSR_PBYP_SHIFT) = (uint32_t)enable;
}


/*!
 * @brief Get the LPTMR input Clock selection
 *
 * This function returns the current configured input Clock for the LPTMR.
 *
 * @param[in] base lptmr base pointer
 * @return the Clock source
 *          - LPTMR_CLOCKSOURCE_SIRC: clock for SIRC
 *          - LPTMR_CLOCKSOURCE_1KHZ_LPO: clock from 1kHz LPO
 *          - LPTMR_CLOCKSOURCE_32KHZ_LPO: clock from 32kHz LPO
 *          - LPTMR_CLOCKSOURCE_PCC: clock from PCC
 */
static inline lptmr_clocksource_t LPTMR_HAL_GetClockSelect(const LPTMR_Type* const base)
{
    uint32_t tmp = base->PSR;
    tmp = (tmp & LPTMR_PSR_PCS_MASK) >> LPTMR_PSR_PCS_SHIFT;
    return (lptmr_clocksource_t)(tmp);
}


/*!
 * @brief Configure the LPTMR input Clock selection
 *
 * This function configures a clock source for the LPTMR.  This feature  can be
 * configured only when the LPTMR is disabled.
 *
 * @param[in] base lptmr base pointer
 * @param[in] clocksel new Clock Source
 *          - LPTMR_CLOCKSOURCE_SIRC: clock for SIRC
 *          - LPTMR_CLOCKSOURCE_1KHZ_LPO: clock from 1kHz LPO
 *          - LPTMR_CLOCKSOURCE_32KHZ_LPO: clock from 32kHz LPO
 *          - LPTMR_CLOCKSOURCE_PCC: clock from PCC
 */
static inline void LPTMR_HAL_SetClockSelect(LPTMR_Type* const base, const lptmr_clocksource_t clocksel)
{
    uint32_t tmp = base->PSR;
    tmp &= ~(LPTMR_PSR_PCS_MASK);
    tmp |= LPTMR_PSR_PCS(clocksel);
    base->PSR = tmp;
}


/*!
 * @brief Get the Compare Value
 *
 * This function returns the current Compare Value.
 *
 * @param[in] base lptmr base pointer
 * @return the Compare Value
 */
static inline uint16_t LPTMR_HAL_GetCompareValue(const LPTMR_Type* const base)
{
    uint32_t tmp = base->CMR;
    tmp = (tmp & LPTMR_CMR_COMPARE_MASK) >> LPTMR_CMR_COMPARE_SHIFT;
    return (uint16_t)(tmp);
}


/*!
 * @brief Configure the Compare Value
 *
 * This function configures the Compare Value. If set to 0, the Compare Match
 * event and the hardware trigger assert and remain asserted until the timer is
 * disabled.
 *
 * @param[in] base lptmr base pointer
 * @param[in] compval the new Compare Value
 */
static inline void LPTMR_HAL_SetCompareValue(LPTMR_Type* const base, const uint16_t compval)
{
    uint32_t tmp = base->CMR;
    tmp &= ~(LPTMR_CMR_COMPARE_MASK);
    tmp |= LPTMR_CMR_COMPARE(compval);
    base->CMR = tmp;
}


/*!
 * @brief Get the current Counter Value
 *
 * This function returns the Counter Value.
 *
 * @param[in] base lptmr base pointer
 * @return The Counter Value
 */
static inline uint16_t LPTMR_HAL_GetCounterValue(LPTMR_Type* const base)
{
    /* Write dummy value before reading register */
    base->CNR = LPTMR_CNR_COUNTER(0u);
    uint16_t cnr = (uint16_t)base->CNR;
    return cnr;
}


/*! @}*/
#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __FSL_LPTMR_HAL_H__ */
/*******************************************************************************
 * EOF
 ******************************************************************************/
