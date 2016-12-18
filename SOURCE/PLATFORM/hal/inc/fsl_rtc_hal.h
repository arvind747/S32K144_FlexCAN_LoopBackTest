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

#ifndef __FSL_RTC_HAL_H__
#define __FSL_RTC_HAL_H__

#include "fsl_device_registers.h"
#include <stdbool.h>

/*! @file */

/*!
 * @addtogroup rtc_hal Real Time Clock HAL
 * @ingroup rtc
 * @brief Real Time Clock Hardware Abstraction Layer
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*!
 * @brief RTC Seconds interrupt configuration
 */
typedef enum _rtc_second_int_cfg_t {
    RTC_INT_1HZ = 0x00,     /*!< RTC seconds interrupt occures at 1 Hz      */
    RTC_INT_2HZ,            /*!< RTC seconds interrupt occures at 2 Hz      */
    RTC_INT_4HZ,            /*!< RTC seconds interrupt occures at 4 Hz      */
    RTC_INT_8HZ,            /*!< RTC seconds interrupt occures at 8 Hz      */
    RTC_INT_16HZ,           /*!< RTC seconds interrupt occures at 16 Hz     */
    RTC_INT_32HZ,           /*!< RTC seconds interrupt occures at 32 Hz     */
    RTC_INT_64HZ,           /*!< RTC seconds interrupt occures at 64 Hz     */
    RTC_INT_128HZ           /*!< RTC seconds interrupt occures at 128 Hz    */
} rtc_second_int_cfg_t;

/*!
 * @brief RTC Status codes
 */
typedef enum _rtc_status_t {
    RTC_STAT_SUCCESS = 0x00, /*!< RTC SUCCESS                */
    RTC_STAT_ERROR,          /*!< RTC ERROR                  */
    RTC_STAT_LOCKED          /*!< RTC REGISTER LOCKED        */
} rtc_status_t;

/*!
 * @brief RTC CLKOUT pin config
 */
typedef enum _rtc_clk_pin_cfg_t {
    /* TODO: Find which pins are truly for RTC Output */
    RTC_CLK_PIN_DISABLED = 0x00, /*!< Disable RTC_CLKOUT         */
    RTC_CLK_PIN_PTE0,            /*!< Enable RTC_CLKOUT on PTE0  */
    RTC_CLK_PIN_PTE26            /*!< Enable RTC_CLKOUT on PTE26 */
} rtc_clk_pin_cfg_t;

/*!
 * @brief RTC CLKOUT Source select
 */
typedef enum _rtc_clk_pin_select_t {
    RTC_CLKOUT_SELECT_TSIC = 0x0, /*!< Output on RTC_CLKOUT as configured on 
                                   *   Time seconds interrupt                  */
    RTC_CLKOUT_SELECT_32KHZ       /*!< Output on RTC_CLKOUT of the 32KHz clock */
} rtc_clk_pin_select_t;

/*!
 * @brief RTC clock select
 */
typedef enum _rtc_clk_select_t {
    RTC_CLK_SRC_32KHZ = 0x00, /*!< RTC Prescaler increments using 32 KHz crystal  */
    RTC_CLK_SRC_LPO_1KHZ      /*!< RTC Prescaler increments using 1KHz LPO        */
} rtc_clk_select_t;

/*!
 * @brief RTC register lock
 */
typedef enum _rtc_lock_register_select_t {
    RTC_LOCK_REG_LOCK = 0x00,   /*!< RTC Lock Register lock         */
    RTC_STAT_REG_LOCK,          /*!< RTC Status Register lock       */
    RTC_CTRL_REG_LOCK,          /*!< RTC Control Register lock      */
    RTC_TCE_REG_LOCK            /*!< RTC Time Compensation Reg lock */
} rtc_lock_register_select_t;


/*!
 * @brief RTC wakeup pin output select
 */
typedef enum _rtc_wakeup_pin_select_t {
    RTC_WAKEUP_ASSERT_ON_INT = 0x00, /*!< Wakeup pin asserts on RTC interrupt 
                                      *   if the pin is enabled               */
    RTC_WAKEUP_OUTPUT_32KHz          /*!< Wakeup pin outputs 32KHz clock to other
                                      *   peripherals if the pin is enabled   */
} rtc_wakeup_pin_select_t;


/*******************************************************************************
 * Code
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Configuration
 * @{
 */

/*****************************************************************************
 * Methods for RTC Control
 ****************************************************************************/

/*!
 * @brief Initialize RTC instance
 * @param base RTC base pointer
 * @return  RTC_STAT_SUCCESS if the operation was successful, RTC_STAT_ERROR 
 *          if there was a problem
 * @return type rtc_status_t
 */
rtc_status_t RTC_HAL_Init(RTC_Type * const base);

/*!
 * @brief Enable RTC instance counter
 * @param base RTC base pointer
 * @return RTC_STAT_SUCCESS if the operation was successful, RTC_STAT_ERROR 
 *         if there was a problem
 * @return type rtc_status_t
 */
rtc_status_t RTC_HAL_Enable(RTC_Type * const base);

/*!
 * @brief Disable RTC instance counter
 * @param base RTC base pointer
 * @return   RTC_STAT_SUCCESS if the operation was successful, RTC_STAT_ERROR 
 *           if there was a problem
 */
rtc_status_t RTC_HAL_Disable(RTC_Type * const base);

/*!
 * @brief This function configures RTC Oscillator additional load. The load
 *        must be configured ONLY when the oscillator is disabled via the
 *        OSCE bit in the RTC Control register.
 * @param base RTC base pointer
 * @param mask Configuration mask for the osc load. For example:
 *             mask = (RTC_CR_SC2P_MASK | RTC_CR_SC8P_MASK) to configure 2pF
 *             and 8pF additional load.
 * @return RTC_STAT_SUCCESS if the operation was successful, RTC_STAT_ERROR if 
 *         the oscillator was not disabled or if the write failed.
 * @return type rtc_status_t
 */
rtc_status_t RTC_HAL_SetOscLoadConfig(RTC_Type * const base,
                                              uint32_t mask);


/*!
 * @brief This function retrieves the Oscillator additional load configuration.
 * @param base RTC base pointer
 * @return the current load configuration
 * @return type uint32_t
 */
uint32_t RTC_HAL_GetOscLoadConfig(RTC_Type * const base);

/*!
 * @brief This function configures register lock status
 * @param base RTC base pointer
 * @param registerToConfig Register to configure
 * @return type rtc_status_t
 * @return Returns the status of the operation, RTC_STAT_SUCCESS
 *         if the lock was successful ,RTC_STAT_LOCKED if the Lock
 *         register was already locked and RTC_STAT_ERROR if the
 *         registerToConfig is not a valid register
 */

rtc_status_t RTC_HAL_ConfigureRegisterLock(RTC_Type * const base,
                        rtc_lock_register_select_t registerToConfig);

/*!
 * @brief This function gets register lock status
 * @param base RTC base pointer
 * @param reg The register for which to check lock status
 * @return type bool
 * @return Return true if the register is locked, false if
 *          it is not locked
 */
bool RTC_HAL_isRegisterLocked(RTC_Type * const base,
                        rtc_lock_register_select_t reg);

/*****************************************************************************
 * Methods for RTC Time Seconds register
 ****************************************************************************/

/*!
 * @brief Get Time Seconds Register Value
 * @param base RTC base pointer
 * @return Number of seconds passed
 * @return type uint32
 */
static inline uint32_t RTC_HAL_GetTimeSecondsRegister(RTC_Type * const base) {
    uint32_t tmp = base->TSR;
    tmp = (tmp & RTC_TSR_TSR_MASK) >> RTC_TSR_TSR_SHIFT;
    return (uint32_t) (tmp);
}

/*!
 * @brief Set Time Seconds Register
 *        This function along with SetTimePrescalerRegister will help you set
 *        the starting time at a specified value.
 *        The write will fail if the Time Counter is enabled and will return
 *        RTC_STAT_ERROR, otherwise the return will be RTC_STAT_SUCCESS
 *
 * @param base  : RTC base pointer
 * @param seconds   : number of seconds passed
 * @return RTC_STAT_SUCCESS if the write is succeeded or RTC_STAT_ERROR if
 *         the counter is enabled.
 * @return type rtc_status_t
 */

rtc_status_t RTC_HAL_SetTimeSecondsRegister(RTC_Type * const base,
                                                     uint32_t seconds);

/*****************************************************************************
 * Methods for RTC Time Prescaler register
 ****************************************************************************/

/*!
 * @brief Get Time Prescaler Register
 * @param base RTC base pointer
 * @return Value stored in the Time Prescaler Register
 * @return type uint16
 */
static inline uint16_t RTC_HAL_GetTimePrescalerRegister(RTC_Type * const base) {
    uint32_t tmp = base->TPR;
    tmp = (tmp & RTC_TPR_TPR_MASK) >> RTC_TPR_TPR_SHIFT;
    return (uint16_t) (tmp);
}

/*!
 * @brief Set Time Prescaler Register.
 *        This function along with SetTimeSecondsRegister will help you set
 *        the starting time at a specified value.
 *        The write will fail if the Time Counter is enabled and will return
 *        RTC_STAT_ERROR, otherwise the return will be RTC_STAT_SUCCESS
 *
 * @param base  : RTC base pointer
 * @param value     : Number of RTC CLK IN periods
 * @return RTC_STAT_SUCCESS if the write is succeeded or RTC_STAT_ERROR if
 *         the counter is enabled.
 * @return type rtc_status_t
 */
rtc_status_t RTC_HAL_SetTimePrescalerRegister(RTC_Type * const base,
                                                         uint16_t value);


/*****************************************************************************
 * Methods for RTC Time Alarm register
 ****************************************************************************/

/*!
 * @brief Get Time Alarm Register
 * @param base RTC base pointer
 * @return Value in seconds of the Time Alarm Register
 * @return type uint32
 */

static inline uint32_t RTC_HAL_GetTimeAlarmRegister(RTC_Type * const base) {
    uint32_t tmp = base->TAR;
    tmp = (tmp & RTC_TAR_TAR_MASK) >> RTC_TAR_TAR_SHIFT;
    return (uint32_t) (tmp);
}

/*!
 * @brief Set Time Alarm Register
 * @param base RTC base pointer
 * @param seconds Number of seconds at which the alarm is triggered. The TAR
 *                value is correct only if the value is greater than current
 *                time (Time seconds register)
 *
 */

static inline void RTC_HAL_SetTimeAlarmRegister(RTC_Type * const base,
                                                    uint32_t seconds) {
    uint32_t tmp = base->TAR;
    tmp &= ~(RTC_TAR_TAR_MASK);
    tmp |= RTC_TAR_TAR(seconds);
    base->TAR = tmp;
}

/*****************************************************************************
 * Methods for RTC Time Compensation register
 ****************************************************************************/

/*!
 * @brief Get Time Compensation Register Value
 *
 *        The Time Prescaler register overflows at every 32768 - (compRegister)
 *        cycles.
 *        For example if the compRegister is -128 TPR overflows at
 *        32768 - (-128) = 32896 cycles.
 *        Else if compRegister is 127 TPR overflows at 32641 cycles.
 *
 * @param base RTC base pointer
 * @return Compensation register value
 * @return type int8_t
 */
static inline int8_t RTC_HAL_GetTimeCompensationRegister(RTC_Type * const base) {
    uint32_t tmp = base->TCR;
    tmp = (tmp & RTC_TCR_TCR_MASK) >> RTC_TCR_TCR_SHIFT;
    return (int8_t) (tmp);
}
/*!
 * @brief Set Time Compensation Register
 *        Configure the frequency of the Time Seconds counter together with
 *        Compensation Interval register.
 *
 *        The Time Prescaler register overflows at every 32768 - (compValue)
 *        cycles. For example if the compValue is -128 TPR overflows at
 *        32768 - (-128) = 32896 cycles
 *
 *        Else if compValue is 127 TPR overflows at 32641 cycles
 *
 * @param base RTC base pointer
 * @param compValue - the value which is subtracted from the counter
 *                      valid range -128 127
 * @param type int8
 */
static inline void RTC_HAL_SetTimeCompensationRegister(RTC_Type * const base,
                                                            int8_t compValue) {
    uint32_t tmp = base->TCR;
    tmp &= ~(RTC_TCR_TCR_MASK);
    tmp |= RTC_TCR_TCR(compValue);
    base->TCR = tmp;
}
/*!
 * @brief Get Time Compensation Interval
 * @param base RTC base pointer
 * @return The value stored in the Time Compensation Interval Register
 */
static inline uint8_t RTC_HAL_GetCompensationIntervalRegister(
                                                    RTC_Type * const base) {
    uint32_t tmp = base->TCR;
    tmp = (tmp & RTC_TCR_CIR_MASK) >> RTC_TCR_CIR_SHIFT;
    return (uint8_t) (tmp);
}
/*!
 * @brief Set Time Compensation Interval
 *      Configures the compensation interval in seconds from 1 to 256 to
 *      control how frequently the TCR should adjust the number of 32.768 kHz
 *      cycles in each second. The value written should be one less than the
 *      number of seconds. For example, write zero to configure for a
 *      compensation interval of one second. This register is double buffered
 *      and writes do not tke affect until the end of the current compensation
 *      interval.
 * @param base RTC base pointer
 * @param compValue
 * @param type uint8
 */
static inline void RTC_HAL_SetCompensationIntervalRegister(
                                    RTC_Type * const base, uint8_t compValue) {

    uint32_t tmp = base->TCR;
    tmp &= ~(RTC_TCR_CIR_MASK);
    tmp |= RTC_TCR_CIR(compValue);
    base->TCR = tmp;
}
/*!
 * @brief Get TimeCompensation Value
 *      Returns current value used by the compensation logic for the present
 *      second interval. Updated once a second if the CIC equals 0 with the
 *      contents of the TCR field. If the CIC does not equal zero then it
 *      is loaded with zero.
 * @param base RTC base pointer
 * @return Current value used by the compensation logic for the present second interval
 */
static inline int8_t RTC_HAL_GetTimeCompensationValue(RTC_Type * const base) {
    uint32_t tmp = base->TCR;
    tmp = (tmp & RTC_TCR_TCV_MASK) >> RTC_TCR_TCV_SHIFT;
    return (int8_t) (tmp);
}
/*!
 * @brief Get Compensation Interval Counter
 *      Current value of the compensation interval counter. If the compensation
 *      interval counter equals zero then it is loaded with the contents of the CIR.
 *      If the CIC does not equal zero then it is decremented once a second.
 * @param base RTC base pointer
 * @return Current value of the compensation interval counter.
 */
static inline uint8_t RTC_HAL_GetCompensationIntervalCounter(
                                                    RTC_Type * const base) {
    uint32_t tmp = base->TCR;
    tmp = (tmp & RTC_TCR_CIC_MASK) >> RTC_TCR_CIC_SHIFT;
    return (uint8_t) (tmp);
}

/*****************************************************************************
 * Methods for RTC Control register
 ****************************************************************************/

/*!
 * @brief Enable or disable the RTC_CLKOUT pin
 * @param base RTC base pointer
 * @param pinCfg select which pin you want to use as
 *                  output
 * @param type rtc_clk_pin_cfg_t
 */
static inline void RTC_HAL_SetClockPinEnable(RTC_Type * const base,
                                            rtc_clk_pin_cfg_t pinCfg) {
    uint32_t tmp = base->CR;
    tmp &= ~(RTC_CR_CPE_MASK);
    tmp |= RTC_CR_CPE(pinCfg);
    base->CR = tmp;
}
/*!
 * @brief Get the Clock pin enable register value
 * @param base RTC base pointer
 * @return pin which is selected as output
 */
static inline rtc_clk_pin_cfg_t RTC_HAL_GetClockPinEnable(
                                                        RTC_Type * const base) {

    uint32_t tmp = base->CR;
    tmp = (tmp & RTC_CR_CPE_MASK) >> RTC_CR_CPE_SHIFT;
    return (rtc_clk_pin_cfg_t) (tmp);
}

/*!
 * @brief Enable or disable RTC_CLKOUT
 * @param base RTC base pointer
 * @param enable Enable or disable the clock output
 * @param type bool
 */
static inline void RTC_HAL_SetClockOutput(RTC_Type * const base, bool enable) {
    uint32_t tmp = base->CR;
    tmp &= ~(RTC_CR_CLKO_MASK);
    tmp |= RTC_CR_CLKO(~enable);
    base->CR = tmp;
}
/*!
 * @brief Get Clock Output enablement status
 * @param base RTC base pointer
 * @return true if the clock output is enabled, false if not
 */
static inline bool RTC_HAL_GetClockOutput(RTC_Type * const base) {
    uint32_t tmp = base->CR;
    tmp = (tmp & RTC_CR_CLKO_MASK) >> RTC_CR_CLKO_SHIFT;
    return !((bool) (tmp));
}
/*!
 * @brief Enable or disable the RTC oscillator
 * @param base RTC base pointer
 * @param enable Enable or disable crystal oscillator
 */
static inline void RTC_HAL_SetOscillatorEnable(RTC_Type * const base,
        bool enable) {
    uint32_t tmp = base->CR;
    tmp &= ~(RTC_CR_OSCE_MASK);
    tmp |= RTC_CR_OSCE(enable);
    base->CR = tmp;
}
/*!
 * @brief Get Oscillator enablement status
 * @param base RTC base pointer
 * @return Oscillator enablement
 */
static inline bool RTC_HAL_GetOscillatorEnable(RTC_Type * const base) {
    uint32_t tmp = base->CR;
    tmp = (tmp & RTC_CR_OSCE_MASK) >> RTC_CR_OSCE_SHIFT;
    return (bool) (tmp);
}
/*!
 * @brief Select clock source for RTC prescaler
 *      When set, the RTC prescaler increments using the LPO 1kHz
 *      clock and not the RTC 32kHz crystal clock. The LPO increments
 *      the prescaler from bit TPR[5] (TPR[4:0] are ignored),
 *      supporting close to 1 second increment of the seconds register.
 * @param base RTC base pointer
 * @param clk_select clock source
 * @param type rtc_lpo_select_t
 */
static inline void RTC_HAL_SetLPOSelect(RTC_Type * const base,
        rtc_clk_select_t clk_select) {
    uint32_t tmp = base->CR;
    tmp &= ~(RTC_CR_LPOS_MASK);
    tmp |= RTC_CR_LPOS(clk_select);
    base->CR = tmp;
}
/*!
 * @brief Get the selected clock source for RTC prescaler
 *      When set, the RTC prescaler increments using the LPO 1kHz
 *      clock and not the RTC 32kHz crystal clock. The LPO increments
 *      the prescaler from bit TPR[5] (TPR[4:0] are ignored),
 *      supporting close to 1 second increment of the seconds register.
 * @param base RTC base pointer
 * @return Selected clock source for RTC
 */
static inline rtc_clk_select_t RTC_HAL_GetLPOSelect(RTC_Type * const base) {
    uint32_t tmp = base->CR;
    tmp = (tmp & RTC_CR_LPOS_MASK) >> RTC_CR_LPOS_SHIFT;
    return (rtc_clk_select_t) (tmp);
}
/*!
 * @brief Select which signal is output on the RTC_CLKOUT
 * @param base RTC base pointer
 * @param selectedPin choose which signal to use as output
 * @param type rtc_clk_pin_select_t
 */
static inline void RTC_HAL_SetClockPinSelect(RTC_Type * const base,
                                        rtc_clk_pin_select_t selectedPin) {
    uint32_t tmp = base->CR;
    tmp &= ~(RTC_CR_CPS_MASK);
    tmp |= RTC_CR_CPS(selectedPin);
    base->CR = tmp;
}
/*!
 * @brief Get which signal is output on the RTC_CLKOUT
 * @param base RTC base pointer
 * @return signal output on the RTC_CLKOUT
 */
static inline rtc_clk_pin_select_t RTC_HAL_GetClockPinSelect( 
                                                        RTC_Type * const base) {

    uint32_t tmp = base->CR;
    tmp = (tmp & RTC_CR_CPS_MASK) >> RTC_CR_CPS_SHIFT;
    return (rtc_clk_pin_select_t) (tmp);
}

#if FSL_DEVICE_HAS_WAKEUP_PIN
/*!
 * @brief Select Wakeup pin operation
 * @param base RTC base pointer
 * @param value Wakeup pin operation
 * @param type rtc_wakeup_pin_select_t
 */
static inline void RTC_HAL_SetWakeupPinSelect(RTC_Type * const base,
                                                rtc_wakeup_pin_select_t value)
{
    uint32_t tmp = base->CR;
    tmp &= ~(RTC_CR_WPS_MASK);
    tmp |= RTC_CR_WPS(value);
    base->CR = tmp;
}
/*!
 * @brief Get wakeup pin selected operation
 * @param base RTC base pointer
 * @return wakeup pin selected operation
 * @return type rtc_wakeup_pin_select_t
 */
static inline rtc_wakeup_pin_select_t RTC_HAL_GetWakeupPinSelect
                                                (RTC_Type * const base) {
    uint32_t tmp = base->CR;
    tmp = (tmp & RTC_CR_WPS_MASK) >> RTC_CR_WPS_SHIFT;
    return (rtc_wakeup_pin_select_t) (tmp);
}
#endif

/*!
 * @brief Set Update Mode of the registers when locked
 *      - true to enable writing of the registers when locked
 *      - false to disable writing of the registers when locked
 * @param base RTC base pointer
 * @param updateEnable value to be written in the register field
 */
static inline void RTC_HAL_SetUpdateMode(RTC_Type * const base,
                                            bool updateEnable) {
    uint32_t tmp = base->CR;
    tmp &= ~(RTC_CR_UM_MASK);
    tmp |= RTC_CR_UM(updateEnable);
    base->CR = tmp;
}
/*!
 * @brief Get the Update Mode of the registers when locked
 * @param base RTC base pointer
 * @return Update mode value
 *       - true if writing of the registers when locked is enabled
 *       - false if writing of the registers when locked is disabled
 */
static inline bool RTC_HAL_GetUpdateMode(RTC_Type * const base) {
    uint32_t tmp = base->CR;
    tmp = (tmp & RTC_CR_UM_MASK) >> RTC_CR_UM_SHIFT;
    return (bool) (tmp);
}
/*!
 * @brief Set Supervisor access mode
 *      - if true Non-supervisor mode write accesses are supported.
 *      - if false Non-supervisor mode write accesses are not supported and
 *          generate a bus error.
 * @param base RTC base pointer
 * @param enable supervisor acces
 * @param type bool
 */
static inline void RTC_HAL_SetSupervisorAccess(RTC_Type * const base,
                                                bool enable) {
    uint32_t tmp = base->CR;
    tmp &= ~(RTC_CR_SUP_MASK);
    tmp |= RTC_CR_SUP(enable);
    base->CR = tmp;
}
/*!
 * @brief Get Supervisor access mode
 * @param base RTC base pointer
 * @return supervisor access mode
 *      - true if Non-supervisor mode write accesses are supported.
 *      - false if Non-supervisor mode write accesses are not supported.
 */
static inline bool RTC_HAL_GetSupervisorAccess(RTC_Type * const base) {
    uint32_t tmp = base->CR;
    tmp = (tmp & RTC_CR_SUP_MASK) >> RTC_CR_SUP_SHIFT;
    return (bool) (tmp);
}

#if FSL_DEVICE_HAS_WAKEUP_PIN
/*!
 * @brief Enable or disable the wakeup pin
 * @param base RTC base pointer
 * @param enable :
 *      - true to enable wakeup pin
 *      - false to disable it
 */
static inline void RTC_HAL_SetWakeupPinEnable(RTC_Type * const base,
                                                bool enable)
{
    uint32_t tmp = base->CR;
    tmp &= ~(RTC_CR_WPE_MASK);
    tmp |= RTC_CR_WPE(enable);
    base->CR = tmp;
}
/*!
 * @brief Get Wakeup pin enablement
 * @param base RTC base pointer
 * @return
 *      - false if Wakeup pin is disabled.
 *      - true if Wakeup pin is enabled (wakeup pin asserts if the RTC
 *        interrupt asserts or the wakeup pin is turned on).
 */
static inline bool RTC_HAL_GetWakeupPinEnable(RTC_Type * const base)
{
    uint32_t tmp = base->CR;
    tmp = (tmp & RTC_CR_WPE_MASK) >> RTC_CR_WPE_SHIFT;
    return (bool) (tmp);
}
#endif

/*!
 * @brief Trigger a software reset
 * @param base RTC base pointer
 */
static inline void RTC_HAL_SetSoftwareReset(RTC_Type * const base) {
    uint32_t tmp = base->CR;
    tmp &= ~(RTC_CR_SWR_MASK);
    tmp |= RTC_CR_SWR(1);
    base->CR = tmp;
}
/*!
 * @brief Clear Software reset flag
 * @param base RTC base pointer
 */
static inline void RTC_HAL_ClearSoftwareReset(RTC_Type * const base) {
    uint32_t tmp = base->CR;
    tmp &= ~(RTC_CR_SWR_MASK);
    base->CR = tmp;
}

/*****************************************************************************
 * Methods for RTC Status register
 ****************************************************************************/

/*!
 * @brief Enable or disable the Time counter
 *      When time counter is disabled the TSR register and TPR register are
 *      writable, but do not increment.
 *      When time counter is enabled the TSR register and TPR register are
 *      not writable, but increment.
 * @param base RTC base pointer
 * @param enable :
 *      - true to enable the counter
 *      - false to disable the counter
 */
static inline void RTC_HAL_SetTimeCounterEnable(RTC_Type * const base,
                                                          bool enable) {
    uint32_t tmp = base->SR;
    tmp &= ~(RTC_SR_TCE_MASK);
    tmp |= RTC_SR_TCE(enable);
    base->SR = tmp;
}
/*!
 * @brief Get the Time Counter Enable value
 * @param base RTC base pointer
 * @return :
 *      - true if the counter is enabled
 *      - false if the counter is disabled
 */
static inline bool RTC_HAL_GetTimeCounterEnable(RTC_Type * const base) {
    uint32_t tmp = base->SR;
    tmp = (tmp & RTC_SR_TCE_MASK) >> RTC_SR_TCE_SHIFT;
    return (bool) (tmp);
}

/*!
 * @brief Get the Time alarm flag
 *      - The alarm flag is cleared after a write in Time Alarm Register
 * @param base RTC base pointer
 * @return :
 *      - true if an alarm occurred
 *      - false if an alarm was not occurred
 */
static inline bool RTC_HAL_GetTimeAlarmFlag(RTC_Type * const base) {
    uint32_t tmp = base->SR;
    tmp = (tmp & RTC_SR_TAF_MASK) >> RTC_SR_TAF_SHIFT;
    return (bool) (tmp);
}

/*!
 * @brief Get Time Overflow Flag
 *      The TOF is set when Time Seconds Register overflows. Disable the
 *      counter and write TSR to clear this bit
 * @param base RTC base pointer
 * @return :
 *      - true if an overflow has occurred
 *      - false if an overflow has not occurred
 */
static inline bool RTC_HAL_GetTimeOverflowFlag(RTC_Type * const base) {
    uint32_t tmp = base->SR;
    tmp = (tmp & RTC_SR_TOF_MASK) >> RTC_SR_TOF_SHIFT;
    return (bool) (tmp);
}

/*!
 * @brief Get Time Invalid flag
 *      - The time invalid flag is set on POR or software reset. The TSR and
 *      TPR do not increment and read as zero when this bit is set. This bit
 *      is cleared by writing the TSR register when the time counter is
 *      disabled.
 * @param base RTC base pointer
 * @return :
 *      - true if TIF is set
 *      - false if TIF is clear
 */
static inline bool RTC_HAL_GetTimeInvalidFlag(RTC_Type * const base) {
    uint32_t tmp = base->SR;
    tmp = (tmp & RTC_SR_TIF_MASK) >> RTC_SR_TIF_SHIFT;
    return (bool) (tmp);
}

/*****************************************************************************
 * Methods for RTC Lock register
 ****************************************************************************/

/*!
 * @brief Lock the Lock Register
 *      This method locks the Lock Register. If the register is locked, it can
 *      be unlocked only with power-on reset(POR) or a software reset.
 * @param base RTC base pointer
 */
static inline void RTC_HAL_LockRegisterLock(RTC_Type * const base) {
    uint32_t tmp = base->LR;
    tmp &= ~(RTC_LR_LRL_MASK);
    base->LR = tmp;
}

/*!
 * @brief Get the Lock Register Lock state
 * @param base RTC base pointer
 * @return :
 *      -   true if register is locked
 *      -   false if the register is not locked
 */
static inline bool RTC_HAL_GetLockRegisterLock(RTC_Type * const base) {
    uint32_t tmp = base->LR;
    tmp = (tmp & RTC_LR_LRL_MASK) >> RTC_LR_LRL_SHIFT;
    return (bool) (!tmp);
}
/*!
 * @brief Lock the Status Register
 *      This method locks the Status Register. If the register is locked, it can
 *      be unlocked only with power-on reset(POR) or a software reset.
 * @param base RTC base pointer
 */
static inline void RTC_HAL_StatusRegisterLock(RTC_Type * const base) {
    uint32_t tmp = base->LR;
    tmp &= ~(RTC_LR_SRL_MASK);
    base->LR = tmp;
}
/*!
 * @brief Get the Status Register Lock state
 * @param base RTC base pointer
 * @return :
 *      -   true if register is locked
 *      -   false if the register is not locked
 */
static inline bool RTC_HAL_GetStatusRegisterLock(RTC_Type * const base) {
    uint32_t tmp = base->LR;
    tmp = (tmp & RTC_LR_SRL_MASK) >> RTC_LR_SRL_SHIFT;
    return (bool) (!tmp);
}
/*!
 * @brief Get the Control Register Lock state
 * @param base RTC base pointer
 * @return :
 *      -   true if register is locked
 *      -   false if the register is not locked
 */
static inline bool RTC_HAL_GetControlRegisterLock(RTC_Type * const base) {
    uint32_t tmp = base->LR;
    tmp = (tmp & RTC_LR_CRL_MASK) >> RTC_LR_CRL_SHIFT;
    return (bool) (!tmp);
}
/*!
 * @brief Lock the Control Register
 *      This method locks the Control Register. If the register is locked, 
 *      it can be unlocked only with power-on reset(POR).
 * @param base RTC base pointer
 */
static inline void RTC_HAL_ControlRegisterLock(RTC_Type * const base) {
    uint32_t tmp = base->LR;
    tmp &= ~(RTC_LR_CRL_MASK);
    base->LR = tmp;
}
/*!
 * @brief Get the TimeCompensation Register Lock state
 * @param base RTC base pointer
 * @return :
 *      -   true if register is locked
 *      -   false if the register is not locked
 */
static inline bool RTC_HAL_GetTimeCompensationLock(RTC_Type * const base) {
    uint32_t tmp = base->LR;
    tmp = (tmp & RTC_LR_TCL_MASK) >> RTC_LR_TCL_SHIFT;
    return (bool) (!tmp);
}
/*!
 * @brief Lock the TimeCompensation Register
 *      This method locks the TimeCompensation Register. If the register 
 *      is locked, it can be unlocked only with power-on reset(POR) or a 
 *      software reset.
 * @param base RTC base pointer
 */
static inline void RTC_HAL_TimeCompensationLock(RTC_Type * const base) {
    uint32_t tmp = base->LR;
    tmp &= ~(RTC_LR_TCL_MASK);
    base->LR = tmp;
}

/*****************************************************************************
 * Methods for RTC Interrupt Enable register
 ****************************************************************************/

/*!
 * @brief Configure Time Seconds interrupt
 * @param base RTC base pointer
 * @param intCfg Select at which frequency the interrupt
 *                 will occur.
 * @param type rtc_second_int_cfg_t
 */
static inline void RTC_HAL_SetTimeSecondsIntConf(RTC_Type * const base,
                                            rtc_second_int_cfg_t intCfg) {
    uint32_t tmp = base->IER;
    tmp &= ~(RTC_IER_TSIC_MASK);
    tmp |= RTC_IER_TSIC((uint8_t )intCfg);
    base->IER = tmp;
}
/*!
 * @brief Get Time Seconds interrupt config
 * @param base RTC base pointer
 * @return the Time Seconds interrupt configuration
 */
static inline rtc_second_int_cfg_t RTC_HAL_GetTimeSecondsIntConf(
                                                        RTC_Type * const base) {

    uint32_t tmp = base->IER;
    tmp = (tmp & RTC_IER_TSIC_MASK) >> RTC_IER_TSIC_SHIFT;
    return (rtc_second_int_cfg_t) (tmp);
}

#if FSL_DEVICE_HAS_WAKEUP_PIN
/*!
 * @brief Get Wakeup pin assertion
 * @param base RTC base pointer
 * @return :
 *      - true if the Wakeup pin is asserted
 */
static inline bool RTC_HAL_GetWakeupPinOn(RTC_Type * const base)
{
    uint32_t tmp = base->IER;
    tmp = (tmp & RTC_IER_WPON_MASK) >> RTC_IER_WPON_SHIFT;
    return (bool) (tmp);
}

/*!
 * @brief Software assert Wakeup pin, if the wakeup pin is enabled
 * @param base RTC base pointer
 * @param enable:
 *      - write true to assert the Wakeup pin
 * @param type bool
 */
static inline void RTC_HAL_SetWakeupPinOn(RTC_Type * const base, bool enable)
{
    uint32_t tmp = base->IER;
    tmp &= ~(RTC_IER_WPON_MASK);
    tmp |= RTC_IER_WPON((uint8_t)enable);
    base->IER = tmp;
}
#endif

/*!
 * @brief Enable TimeSeconds interrupt
 * @param base RTC base pointer
 * @param enable :
 *      - true to enable the interrupt
 *      - false to disable it
 * @param type bool
 */
static inline void RTC_HAL_SetTimeSecondsIntEnable(RTC_Type * const base,
                                                             bool enable) {
    uint32_t tmp = base->IER;
    tmp &= ~(RTC_IER_TSIE_MASK);
    tmp |= RTC_IER_TSIE(enable);
    base->IER = tmp;
}
/*!
 * @brief Get the TimeSeconds interrupt enable status
 * @param base RTC base pointer
 * @return The TimeSeconds interrupt enablement
 *      - true if interrupt is enabled
 *      - false if interrupt is disabled
 */
static inline bool RTC_HAL_GetTimeSecondsIntEnable(RTC_Type * const base) {
    uint32_t tmp = base->IER;
    tmp = (tmp & RTC_IER_TSIE_MASK) >> RTC_IER_TSIE_SHIFT;
    return (bool) (tmp);
}
/*!
 * @brief Enable TimeAlarm interrupt
 * @param base RTC base pointer
 * @param enable :
 *      - true to enable the interrupt
 *      - false to disable it
 * @param type bool
 */
static inline void RTC_HAL_SetTimeAlarmIntEnable(RTC_Type * const base,
                                                           bool enable) {
    uint32_t tmp = base->IER;
    tmp &= ~(RTC_IER_TAIE_MASK);
    tmp |= RTC_IER_TAIE(enable);
    base->IER = tmp;
}
/*!
 * @brief Get the TimeAlarm interrupt enable status
 * @param base RTC base pointer
 * @return The TimeAlarm interrupt enablement
 *      - true if interrupt is enabled
 *      - false if interrupt is disabled
 */
static inline bool RTC_HAL_GetTimeAlarmIntEnable(RTC_Type * const base) {
    uint32_t tmp = base->IER;
    tmp = (tmp & RTC_IER_TAIE_MASK) >> RTC_IER_TAIE_SHIFT;
    return (bool) (tmp);
}
/*!
 * @brief Enable TimeOverflow interrupt
 * @param base RTC base pointer
 * @param enable :
 *      - true to enable the interrupt
 *      - false to disable it
 * @param type bool
 */
static inline void RTC_HAL_SetTimeOverflowIntEnable(RTC_Type * const base,
                                                              bool enable) {
    uint32_t tmp = base->IER;
    tmp &= ~(RTC_IER_TOIE_MASK);
    tmp |= RTC_IER_TOIE(enable);
    base->IER = tmp;
}
/*!
 * @brief Get the TimeAlarm interrupt enable status
 * @param base RTC base pointer
 * @return The TimeOverflow interrupt enablement
 *      - true if interrupt is enabled
 *      - false if interrupt is disabled
 */
static inline bool RTC_HAL_GetTimeOverflowIntEnable(RTC_Type * const base) {
    uint32_t tmp = base->IER;
    tmp = (tmp & RTC_IER_TOIE_MASK) >> RTC_IER_TOIE_SHIFT;
    return (bool) (tmp);
}
/*!
 * @brief Enable TimeInvalid interrupt
 * @param base RTC base pointer
 * @param enable :
 *      - true to enable the interrupt
 *      - false to disable it
 * @param type bool
 */
static inline void RTC_HAL_SetTimeInvalidIntEnable(RTC_Type * const base,
                                                            bool enable) {
    uint32_t tmp = base->IER;
    tmp &= ~(RTC_IER_TIIE_MASK);
    tmp |= RTC_IER_TIIE(enable);
    base->IER = tmp;
}
/*!
 * @brief Get the TimeInvalid interrupt enable status
 * @param base RTC base pointer
 * @return The TimeInvalid interrupt enablement
 *      - true if interrupt is enabled
 *      - false if interrupt is disabled
 */
static inline bool RTC_HAL_GetTimeInvalidIntEnable(RTC_Type * const base) {
    uint32_t tmp = base->IER;
    tmp = (tmp & RTC_IER_TIIE_MASK) >> RTC_IER_TIIE_SHIFT;
    return (bool) (tmp);
}
/*! @}*/

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __FSL_RTC_HAL_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/
