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
#if !defined(__FSL_FTM_HAL_H__)
#define __FSL_FTM_HAL_H__

#include "fsl_device_registers.h"
#include "stdbool.h"

/*
 * S32K144 FTM
 *
 * FlexTimer Module
 *
 * Registers defined in this header file:
 * - FTM_SC - Status And Control
 * - FTM_CNT - Counter
 * - FTM_MOD - Modulo
 * - FTM_C0SC - Channel (n) Status And Control
 * - FTM_C0V - Channel (n) Value
 * - FTM_C1SC - Channel (n) Status And Control
 * - FTM_C1V - Channel (n) Value
 * - FTM_C2SC - Channel (n) Status And Control
 * - FTM_C2V - Channel (n) Value
 * - FTM_C3SC - Channel (n) Status And Control
 * - FTM_C3V - Channel (n) Value
 * - FTM_C4SC - Channel (n) Status And Control
 * - FTM_C4V - Channel (n) Value
 * - FTM_C5SC - Channel (n) Status And Control
 * - FTM_C5V - Channel (n) Value
 * - FTM_C6SC - Channel (n) Status And Control
 * - FTM_C6V - Channel (n) Value
 * - FTM_C7SC - Channel (n) Status And Control
 * - FTM_C7V - Channel (n) Value
 * - FTM_CNTIN - Counter Initial Value
 * - FTM_STATUS - Capture And Compare Status
 * - FTM_MODE - Features Mode Selection
 * - FTM_SYNC - Synchronization
 * - FTM_OUTINIT - Initial State For Channels Output
 * - FTM_OUTMASK - Output Mask
 * - FTM_COMBINE - Function For Linked Channels
 * - FTM_DEADTIME - Deadtime Insertion Control
 * - FTM_EXTTRIG - FTM External Trigger
 * - FTM_POL - Channels Polarity
 * - FTM_FMS - Fault Mode Status
 * - FTM_FILTER - Input Capture Filter Control
 * - FTM_FLTCTRL - Fault Control
 * - FTM_QDCTRL - Quadrature Decoder Control And Status
 * - FTM_CONF - Configuration
 * - FTM_FLTPOL - FTM Fault Input Polarity
 * - FTM_SYNCONF - Synchronization Configuration
 * - FTM_INVCTRL - FTM Inverting Control
 * - FTM_SWOCTRL - FTM Software Output Control
 * - FTM_PWMLOAD - FTM PWM Load
 * - FTM_HCR - Half Cycle Register
 */

#define FTM0_IDX (0U) /*!< Instance number for FTM0. */
#define FTM1_IDX (1U) /*!< Instance number for FTM1. */
#define FTM2_IDX (2U) /*!< Instance number for FTM2. */
#define FTM3_IDX (3U) /*!< Instance number for FTM3. */

/*!
 * @brief FTM_SC - Read modify write to Status And Control (RW)
 */
#define FTM_RMW_SC(base, mask, value) (((base)->SC) = (((base)->SC) & ~(mask)) | (value))


/*!
 * @brief FTM_CNT - Read modify write to Counter (RW)
 */
#define FTM_RMW_CNT(base, mask, value) (((base)->CNT) = ((((base)->CNT) & ~(mask)) | (value)))

/*!
 * @brief FTM_MOD - Read modify write Modulo (RW)
 */
#define FTM_RMW_MOD(base, mask, value) (((base)->MOD) = ((((base)->MOD) & ~(mask)) | (value)))

/*!
 * @brief FTM_CNTIN - Read modify write Counter Initial Value (RW)
 */
#define FTM_RMW_CNTIN(base, mask, value) (((base)->CNTIN) = ((((base)->CNTIN) & ~(mask)) | (value)))

/*!
 * @brief FTM_STATUS - Read modify write Capture And Compare Status (RW)
 */
#define FTM_RMW_STATUS(base, mask, value) (((base)->STATUS) \
        = ((((base)->STATUS) & ~(mask)) | (value)))

/*!
 * @brief FTM_MODE -  Read modify write Counter Features Mode Selection (RW)
 */
#define FTM_RMW_MODE(base, mask, value) (((base)->MODE) = ((((base)->MODE) & ~(mask)) | (value)))

/*!
 * @brief FTM_CnSCV -  Read modify write Channel (n) Status And Control (RW)
 */
#define FTM_RMW_CnSCV_REG(base, channel, mask, value) (((base)->CONTROLS[channel].CnSC) \
        = ((((base)->CONTROLS[channel].CnSC) & ~(mask)) | (value)))

/*!
 * @brief FTM_DEADTIME - Read Modify Write Deadtime Insertion Control (RW)
 */
#define FTM_RMW_DEADTIME(base, mask, value) (((base)->DEADTIME) \
        = ((((base)->DEADTIME) & ~(mask)) | (value)))

/*!
 * @brief FTM_FLTCTRL -  Read Modify Write Fault Control (RW)
 */
#define FTM_RMW_FLTCTRL(base, mask, value) (((base)->FLTCTRL) \
        = ((((base)->FLTCTRL) & ~(mask)) | (value)))

/*!
 * @brief FTM_FMS -  Read modify write Fault Mode Status (RW)
 */
#define FTM_RMW_FMS(base, mask, value) (((base)->FMS) = ((((base)->FMS) & ~(mask)) | (value)))

/*!
 * @brief FTM_CONF -  Read Modify Write Configuration (RW)
 */
#define FTM_RMW_CONF(base, mask, value) (((base)->CONF) = ((((base)->CONF) & ~(mask)) | (value)))

/*!
 * @brief POL -  Read Modify Write Polarity (RW)
 */
#define FTM_RMW_POL(base, mask, value) (((base)->POL) =  ((((base)->POL) & ~(mask)) | (value)))

/*!
 * @brief FILTER -  Read Modify Write Filter (RW)
 */
#define FTM_RMW_FILTER(base, mask, value) (((base)->FILTER) \
        =  ((((base)->FILTER) & ~(mask)) | (value)))

/*!
 * @brief SYNC -  Read Modify Write Synchronization (RW)
 */
#define FTM_RMW_SYNC(base, mask, value) (((base)->SYNC) =  ((((base)->SYNC) & ~(mask)) | (value)))

/*!
 * @brief QDCTRL -  Read Modify Write Quadrature Decoder Control And Status (RW)
 */
#define FTM_RMW_QDCTRL(base, mask, value) (((base)->QDCTRL) \
        =  ((((base)->QDCTRL) & ~(mask)) | (value)))


/*!
 * @addtogroup ftm_hal FTM HAL
 * @ingroup ftm
 * @brief FlexTimer Hardware Abstration Layer
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define CHAN0_IDX (0U) /*!< Channel number for CHAN0.*/
#define CHAN1_IDX (1U) /*!< Channel number for CHAN1.*/
#define CHAN2_IDX (2U) /*!< Channel number for CHAN2.*/
#define CHAN3_IDX (3U) /*!< Channel number for CHAN3.*/
#define CHAN4_IDX (4U) /*!< Channel number for CHAN4.*/
#define CHAN5_IDX (5U) /*!< Channel number for CHAN5.*/
#define CHAN6_IDX (6U) /*!< Channel number for CHAN6.*/
#define CHAN7_IDX (7U) /*!< Channel number for CHAN7.*/



/*! @brief FlexTimer clock source selection*/
typedef enum
{
    FTM_CLOCK_SOURCE_NONE           = 0x00U,    /*!< None */
    FTM_CLOCK_SOURCE_SYSTEMCLK      = 0x01U,    /*!< System clock */
    FTM_CLOCK_SOURCE_FIXEDCLK       = 0x02U,    /*!< Fixed clock */
    FTM_CLOCK_SOURCE_EXTERNALCLK    = 0x03U     /*!< External clock */
}ftm_clock_source_t;

/*! @brief FlexTimer pre-scaler factor selection for the clock source. In quadrature
    decoder mode set FTM_CLOCK_DIVID_BY_1.*/
typedef enum
{
    FTM_CLOCK_DIVID_BY_1    = 0x00U,    /*!< Divide by 1 */
    FTM_CLOCK_DIVID_BY_2    = 0x01U,    /*!< Divide by 2 */
    FTM_CLOCK_DIVID_BY_4    = 0x02U,    /*!< Divide by 4 */
    FTM_CLOCK_DIVID_BY_8    = 0x03U,    /*!< Divide by 8 */
    FTM_CLOCK_DIVID_BY_16   = 0x04U,    /*!< Divide by 16 */
    FTM_CLOCK_DIVID_BY_32   = 0x05U,    /*!< Divide by 32 */
    FTM_CLOCK_DIVID_BY_64   = 0x06U,    /*!< Divide by 64 */
    FTM_CLOCK_DIVID_BY_128  = 0x07U     /*!< Divide by 128 */
}ftm_clock_ps_t;

/*! @brief FlexTimer pre-scaler factor for the deadtime insertion*/
typedef enum
{
    FTM_DEADTIME_DIVID_BY_1 = 0x01U, /*!< Divide by 1 */
    FTM_DEADTIME_DIVID_BY_4 = 0x02U, /*!< Divide by 4 */
    FTM_DEADTIME_DIVID_BY_16= 0x03U, /*!< Divide by 16 */
}ftm_deadtime_ps_t;

/*! @brief FlexTimer output compare edge mode. Toggle, clear or set.*/
typedef enum
{
    FTM_DISABLE_OUTPUT  = 0x00U,    /*! No action on output pin */
    FTM_TOGGLE_ON_MATCH = 0x01U,    /*! Toggle on match */
    FTM_CLEAR_ON_MATCH  = 0x02U,    /*! Clear on match */
    FTM_SET_ON_MATCH    = 0x03U     /*! Set on match */
}ftm_output_compare_mode_t;

/*! @brief FlexTimer PWM output pulse mode, high-true or low-true on match up */
typedef enum
{
    FTM_POLARITY_LOW  = 0x00U,  /*!< When counter > CnV output signal is LOW */
    FTM_POLARITY_HIGH = 0x01U   /*!< When counter > CnV output signal is HIGH */
}ftm_polarity_t;

/*! @brief FlexTimer PWM channel (n+1) polarity for combine mode */
typedef enum
{
    FTM_MAIN_INVERTED   = 0x01U,  /*!< The channel (n+1) output is the inverse of the
                                    channel (n) output  */
    FTM_MAIN_DUPLICATED = 0x00U   /*!< The channel (n+1) output is the same as the
                                    channel (n) output */
}ftm_second_channel_polarity_t;

/*! @brief FlexTimer quadrature decode modes, phase encode or count and direction mode */
typedef enum
{
    FTM_QUAD_PHASE_ENCODE   = 0x00U,    /*!< Phase encode mode */
    FTM_QUAD_COUNT_AND_DIR  = 0x01U     /*!< Counter and direction moe */
}ftm_quad_decode_mode_t;

/*! @brief FlexTimer quadrature phase polarities, normal or inverted polarity */
typedef enum
{
    FTM_QUAD_PHASE_NORMAL = 0x00U,  /*!< Phase input signal is not inverted before identifying
                                    the rising and falling edges of this signal. */
    FTM_QUAD_PHASE_INVERT = 0x01U   /*!< Phase input signal is inverted before identifying
                                    the rising and falling edges of this signal.*/
}ftm_quad_phase_polarity_t;


/*! @brief Options for the FlexTimer behaviour in BDM Mode */
typedef enum
{
    FTM_BDM_MODE_00 = 0x00U,    /*!< FTM counter stopped, CH(n)F bit can be set, FTM channels
                                in functional mode, writes to MOD,CNTIN and C(n)V registers bypass
                                the register buffers.*/
    FTM_BDM_MODE_01 = 0x01U,    /*!< FTM counter stopped, CH(n)F bit is not set, FTM channels
                                outputs are forced to their safe value , writes to MOD,CNTIN and
                                C(n)V registers bypass the register buffers. */
    FTM_BDM_MODE_10 = 0x02U,    /*!< FTM counter stopped, CH(n)F bit is not set, FTM channels
                                outputs are frozen when chip enters in BDM mode, writes to MOD,
                                CNTIN and C(n)V registers bypass the register buffers.*/
    FTM_BDM_MODE_11 = 0x03U     /*!< FTM counter in functional mode, CH(n)F bit can be set,
                                FTM channels in functional mode, writes to MOD,CNTIN and C(n)V
                                registers is in fully functional mode. */
}ftm_bdm_mode_t;

/*! @brief FlexTimer fault control  */
typedef enum
{
    FTM_FAULT_CONTROL_DISABLED  = 0x00U,    /*!< Fault control is disabled for all channels. */
    FTM_FAULT_CONTROL_MAN_EVEN  = 0x01U,    /*!< Fault control is enabled for even channels
                                            only (channels 0, 2, 4, and 6), and the selected
                                            mode is the manual fault clearing. */
    FTM_FAULT_CONTROL_MAN_ALL   = 0x02U,    /*!< Fault control is enabled for all channels,
                                            and the selected mode is the manual fault clearing. */
    FTM_FAULT_CONTROL_AUTO_ALL  = 0x03U     /*!< Fault control is enabled for all channels, and
                                            the selected mode is the automatic fault clearing. */
}ftm_fault_mode_t;

/*! @brief FTM sync source */
typedef enum
{
    FTM_SYSTEM_CLOCK    = 0U,       /*!< Register is updated with its buffer value at all rising
                                    edges of system clock.*/
    FTM_PWM_SYNC        = 1U,       /*!< Register is updated with its buffer value at the
                                    FTM synchronization.*/
} ftm_reg_update_t;

/*! @brief FTM sync mode  */
typedef enum
{
    FTM_WAIT_LOADING_POINTS = 0U,   /*!< FTM register is updated at first loading point.*/
    FTM_UPDATE_NOW          = 1U,   /*!< FTM register is updated immediately.*/
} ftm_pwm_sync_mode_t;



/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*FTM Half Cycle Register*/
/*!
 * @brief Sets the value for the half cycle reload register
 *
 * @param ftmBase The FTM base address pointer
 * @param value  16 bit counter value
 */
static inline void FTM_HAL_SetHalfCycleValue(FTM_Type* const ftmBase, uint16_t value)
{
    ((ftmBase)->HCR) = value;
}


/*FTM timer control*/
/*!
 * @brief Sets the FTM clock source.
 *
 * @param ftmBase The FTM base address pointer
 * @param clock  The FTM peripheral clock selection\n
 *        bits - 00: No clock  01: system clock  10: fixed clock   11: External clock
 */
static inline void FTM_HAL_SetClockSource(FTM_Type* const ftmBase, ftm_clock_source_t clock)
{
    FTM_RMW_SC(ftmBase, FTM_SC_CLKS_MASK, FTM_SC_CLKS(clock));
}

/*!
 * @brief Reads the FTM clock source.
 *
 * @param ftmBase The FTM base address pointer
 *
 * @return  The FTM clock source selection\n
 *          bits - 00: No clock  01: system clock  10: fixed clock   11:External clock
 */
static inline uint8_t FTM_HAL_GetClockSource(FTM_Type* const ftmBase)
{
    return ((((ftmBase)->SC) & FTM_SC_CLKS_MASK) >> FTM_SC_CLKS_SHIFT);
}

/*!
 * @brief Sets the FTM clock divider.
 *
 * @param ftmBase The FTM base address pointer
 * @param ps  The FTM peripheral clock pre-scale divider
 */
static inline void FTM_HAL_SetClockPs(FTM_Type* const ftmBase, ftm_clock_ps_t ps)
{
    FTM_RMW_SC(ftmBase, FTM_SC_PS_MASK, FTM_SC_PS(ps));
}

/*!
 * @brief Reads the FTM clock divider.
 *
 * @param ftmBase The FTM base address pointer
 *
 * @return The FTM clock pre-scale divider
 */
static inline uint8_t FTM_HAL_GetClockPs(FTM_Type* const ftmBase)
{
    return  ((((ftmBase)->SC) & FTM_SC_PS_MASK) >> FTM_SC_PS_SHIFT);
}

/*!
 * @brief Enables the FTM peripheral timer overflow interrupt.
 *
 * @param ftmBase The FTM base address pointer
 * @param state true - overflow interrupt enabled false - overflow interrupt disabled
 */
static inline void FTM_HAL_SetTimerOverflowInt(FTM_Type* const ftmBase, bool state)
{
    FTM_RMW_SC(ftmBase, FTM_SC_TOIE_MASK, FTM_SC_TOIE(state));
}


/*!
 * @brief Reads the bit that controls enabling the FTM timer overflow interrupt.
 *
 * @param ftmBase The FTM base address pointer
 * @return true if overflow interrupt is enabled, false if not
 */
static inline bool FTM_HAL_IsOverflowIntEnabled(FTM_Type* const ftmBase)
{
    return (BITBAND_ACCESS32(&((ftmBase)->SC), FTM_SC_TOIE_SHIFT)) ? true : false;
}


/*!
 * @brief  Enable Pwm channel Outputs
 *
 * @param ftmBase The FTM base address pointer
 * @param channel The FTM channel
 */
static inline void FTM_HAL_EnablePwmChannelOutputs(FTM_Type* const ftmBase, uint8_t channel)
{
    FTM_RMW_SC(ftmBase, (1U << (channel + FTM_FEATURE_OUTPUT_CHANNEL_OFFSET)), (1U << (channel + FTM_FEATURE_OUTPUT_CHANNEL_OFFSET)));
}

/*!
 * @brief Disable Pwm channel Outputs
 *
 * @param ftmBase The FTM base address pointer
 * @param channel The FTM channel
 */
static inline void FTM_HAL_DisablePwmChannelOutputs(FTM_Type* const ftmBase, uint8_t channel)
{
    uint32_t regValue = ((ftmBase)->SC);
    regValue = regValue & (~(1U << (channel + FTM_FEATURE_OUTPUT_CHANNEL_OFFSET)));
    ((ftmBase)->SC) = (regValue);
}


/*!
 * @brief Clears the timer overflow interrupt flag.
 *
 * @param ftmBase The FTM base address pointer
 */
static inline void FTM_HAL_ClearTimerOverflow(FTM_Type* const ftmBase)
{
    /*BITBAND_ACCESS32(&((ftmBase)->SC), FTM_SC_TOF_SHIFT) = (0U);*/
    FTM_RMW_SC(ftmBase, FTM_SC_TOF_MASK, FTM_SC_TOF(0U));
}


/*!
 * @brief Returns the FTM peripheral timer overflow interrupt flag.
 *
 * @param ftmBase The FTM base address pointer
 * @return true if overflow, false if not
 */
static inline bool FTM_HAL_HasTimerOverflowed(FTM_Type* const ftmBase)
{
    return ((BITBAND_ACCESS32(&((ftmBase)->SC), FTM_SC_TOF_SHIFT)) ? true: false);
}

/*!
 * @brief Sets the FTM count direction bit.
 *
 * @param ftmBase The FTM base address pointer
 * @param mode 1:up counting mode 0:up down counting mode
 */
static inline void FTM_HAL_SetCpwms(FTM_Type* const ftmBase, uint8_t mode)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(mode < 2U);
#endif

    /*BITBAND_ACCESS32(&((ftmBase)->SC), FTM_SC_CPWMS_SHIFT) = (mode); */
    FTM_RMW_SC(ftmBase, FTM_SC_CPWMS_MASK, FTM_SC_CPWMS(mode));
}

/*!
 * @brief Gets the FTM count direction bit.
 *
 * @param ftmBase The FTM base address pointer
 * @return mode 1:up counting mode 0:up down counting mode
 */
static inline uint8_t FTM_HAL_GetCpwms(FTM_Type* const ftmBase)
{
    return ((((ftmBase)->SC) & FTM_SC_CPWMS_MASK) >> FTM_SC_CPWMS_SHIFT);
}

/*!
 * @brief Sets the FTM peripheral current counter value.
 *
 * @param ftmBase The FTM base address pointer
 * @param val  FTM timer counter value to be set
 */
static inline void  FTM_HAL_SetCounter(FTM_Type* const ftmBase, uint16_t value)
{
    FTM_RMW_CNT(ftmBase, FTM_CNT_COUNT_MASK, FTM_CNT_COUNT(value));
}

/*!
 * @brief Returns the FTM peripheral current counter value.
 *
 * @param ftmBase The FTM base address pointer
 * @return current FTM timer counter value
 */
static inline uint16_t  FTM_HAL_GetCounter(FTM_Type* const ftmBase)
{
    return ((((ftmBase)->CNT) & FTM_CNT_COUNT_MASK) >> FTM_CNT_COUNT_SHIFT);
}

/*!
 * @brief Sets the FTM peripheral timer modulo value.
 *
 * @param ftmBase The FTM base address pointer
 * @param val The value to be set to the timer modulo
 */
static inline void FTM_HAL_SetMod(FTM_Type* const ftmBase, uint16_t value)
{
    FTM_RMW_MOD(ftmBase, FTM_MOD_MOD_MASK, FTM_MOD_MOD(value));
}

/*!
 * @brief Returns the FTM peripheral counter modulo value.
 *
 * @param ftmBase The FTM base address pointer
 * @return FTM timer modulo value
 */
static inline uint16_t  FTM_HAL_GetMod(FTM_Type* const ftmBase)
{
    return ((((ftmBase)->MOD) & FTM_MOD_MOD_MASK) >> FTM_MOD_MOD_SHIFT);
}

/*!
 * @brief Sets the FTM peripheral timer counter initial value.
 *
 * @param ftmBase The FTM base address pointer
 * @param val initial value to be set
 */
static inline void FTM_HAL_SetCounterInitVal(FTM_Type* const ftmBase, uint16_t value)
{
    FTM_RMW_CNTIN(ftmBase, FTM_CNTIN_INIT_MASK, FTM_CNTIN_INIT(value));
}

/*!
 * @brief Returns the FTM peripheral counter initial value.
 *
 * @param ftmBase The FTM base address pointer
 * @return FTM timer counter initial value
 */
static inline uint16_t  FTM_HAL_GetCounterInitVal(FTM_Type* const ftmBase)
{
    return ((((ftmBase)->CNTIN) & FTM_CNTIN_INIT_MASK) >> FTM_CNTIN_INIT_SHIFT);
}

/*!
 * @brief Sets the FTM peripheral timer channel mode.
 *
 * @param ftmBase The FTM base address pointer
 * @param channel  The FTM peripheral channel number
 * @param selection The mode to be set valid value MSnB:MSnA :00,01, 10, 11
 */
static inline void FTM_HAL_SetChnMSnBAMode(FTM_Type* const ftmBase, uint8_t channel,
                                            uint8_t selection)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_FTM_CHANNEL_COUNT);
#endif

    //write MSA bit*/
    FTM_RMW_CnSCV_REG(ftmBase,channel, FTM_CnSC_MSA_MASK, FTM_CnSC_MSA(selection & 1U) );

    //write MSB bit*/
    FTM_RMW_CnSCV_REG(ftmBase,channel, FTM_CnSC_MSB_MASK, FTM_CnSC_MSB(selection & 2U ? 1U : 0U) );
}

/*!
 * @brief Sets the FTM peripheral timer channel edge level.
 *
 * @param ftmBase   The FTM base address pointer
 * @param channel   The FTM peripheral channel number
 * @param config    ELSnB:ELSnA :00,01, 10, 11
 */
static inline void FTM_HAL_SetChnEdgeLevel(FTM_Type* const ftmBase, uint8_t channel, uint8_t level)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_FTM_CHANNEL_COUNT);
#endif

    //write MSA bit*/
    FTM_RMW_CnSCV_REG(ftmBase,channel, FTM_CnSC_ELSA_MASK, FTM_CnSC_ELSA(level & 1U ? 1U : 0U) );

    //write MSB bit*/
    FTM_RMW_CnSCV_REG(ftmBase,channel, FTM_CnSC_ELSB_MASK, FTM_CnSC_ELSB(level & 2U ? 1U : 0U) );
}


/*!
 * @brief Clears the content of Channel (n) Status And Control.
 *
 * @param ftmBase The FTM base address pointer
 * @param channel  The FTM peripheral channel number
 */
static inline void FTM_HAL_ClearChSC(FTM_Type* const ftmBase, uint8_t channel)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_FTM_CHANNEL_COUNT);
#endif

    ((ftmBase)->CONTROLS[channel].CnSC) = 0U;
}


/*!
 * @brief Gets the FTM peripheral timer channel mode.
 *
 * @param ftmBase The FTM base address pointer
 * @param channel  The FTM peripheral channel number
 * @return The MSnB:MSnA mode value, will be 00,01, 10, 11
 */
static inline uint8_t FTM_HAL_GetChnMode(FTM_Type* const ftmBase, uint8_t channel)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_FTM_CHANNEL_COUNT);
#endif
    uint8_t retValue;

    retValue = ((((ftmBase)->CONTROLS[channel].CnSC) & FTM_CnSC_MSA_MASK) >> FTM_CnSC_MSA_SHIFT);

    retValue |= (((((ftmBase)->CONTROLS[channel].CnSC) & FTM_CnSC_MSB_MASK) >>
                FTM_CnSC_MSB_SHIFT) << 1U);

    return  retValue;
}

/*!
 * @brief Gets the FTM peripheral timer channel edge level.
 *
 * @param ftmBase The FTM base address pointer
 * @param channel  The FTM peripheral channel number
 * @return The ELSnB:ELSnA mode value, will be 00,01, 10, 11
 */
static inline uint8_t FTM_HAL_GetChnEdgeLevel(FTM_Type* const ftmBase, uint8_t channel)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_FTM_CHANNEL_COUNT);
#endif
    uint8_t retValue;

    retValue = ((((ftmBase)->CONTROLS[channel].CnSC) & FTM_CnSC_ELSA_MASK) >> FTM_CnSC_ELSA_SHIFT);

    retValue |= (((((ftmBase)->CONTROLS[channel].CnSC) & FTM_CnSC_ELSB_MASK) >>
                FTM_CnSC_ELSB_SHIFT) << 1U);

    return  retValue;
}

/*!
 * @brief Enables or disables the FTM peripheral timer channel DMA.
 *
 * @param ftmBase The FTM base address pointer
 * @param channel  The FTM peripheral channel number
 * @param val enable or disable
 */
static inline void FTM_HAL_SetChnDmaCmd(FTM_Type* const ftmBase, uint8_t channel, bool val)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_FTM_CHANNEL_COUNT);
#endif

    //write DMA bit*/
    FTM_RMW_CnSCV_REG(ftmBase,channel, FTM_CnSC_DMA_MASK, FTM_CnSC_DMA(val? 1U : 0U));
}

/*!
 * @brief Returns whether the FTM peripheral timer channel DMA is enabled.
 *
 * @param ftmBase The FTM base address pointer
 * @param channel  The FTM peripheral channel number
 * @return true if enabled, false if disabled
 */
static inline bool FTM_HAL_IsChnDma(FTM_Type* const ftmBase, uint8_t channel)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_FTM_CHANNEL_COUNT);
#endif
    return ((((ftmBase)->CONTROLS[channel].CnSC) & FTM_CnSC_DMA_MASK) ? true : false);
}

/*!
 * @brief Get FTM channel(n) interrupt enabled or not.
 * @param ftmBase FTM module base address.
 * @param channel  The FTM peripheral channel number
 */
static inline bool FTM_HAL_IsChnIntEnabled(FTM_Type* const ftmBase, uint8_t channel)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_FTM_CHANNEL_COUNT);
#endif
    return ((((ftmBase)->CONTROLS[channel].CnSC) & FTM_CnSC_CHIE_MASK) ? true : false);
}

/*!
 * @brief Enables the FTM peripheral timer channel(n) interrupt.
 *
 * @param ftmBase The FTM base address pointer
 * @param channel  The FTM peripheral channel number
 */
static inline void FTM_HAL_EnableChnInt(FTM_Type* const ftmBase, uint8_t channel)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_FTM_CHANNEL_COUNT);
#endif
    FTM_RMW_CnSCV_REG(ftmBase, channel, FTM_CnSC_CHIE_MASK, FTM_CnSC_CHIE(1U));
}
/*!
 * @brief Disables the FTM peripheral timer channel(n) interrupt.
 *
 * @param ftmBase The FTM base address pointer
 * @param channel  The FTM peripheral channel number
 */
static inline void FTM_HAL_DisableChnInt(FTM_Type* const ftmBase, uint8_t channel)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_FTM_CHANNEL_COUNT);
#endif
    FTM_RMW_CnSCV_REG(ftmBase, channel, FTM_CnSC_CHIE_MASK, FTM_CnSC_CHIE(0U));
}

/*!
 * @brief Returns whether any event for the FTM peripheral timer channel has occurred.
 *
 * @param ftmBase The FTM base address pointer
 * @param channel  The FTM peripheral channel number
 * @return true if event occurred, false otherwise.
 */
static inline bool FTM_HAL_HasChnEventOccurred(FTM_Type* const ftmBase, uint8_t channel)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_FTM_CHANNEL_COUNT);
#endif
    return ((((ftmBase)->CONTROLS[channel].CnSC) & FTM_CnSC_CHF_MASK) ? true : false);
}

/*!
 * @brief Clear the channel flag by writing a 0 to the CHF bit.
 *
 * @param ftmBase The FTM base address pointer
 * @param channel  The FTM peripheral channel number
 */
static inline void FTM_HAL_ClearChnEventFlag(FTM_Type* const ftmBase, uint8_t channel)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_FTM_CHANNEL_COUNT);
#endif
    FTM_RMW_CnSCV_REG(ftmBase, channel, FTM_CnSC_CHF_MASK, FTM_CnSC_CHF(0U));
}

/*FTM channel control*/
/*!
 * @brief Sets the FTM peripheral timer channel counter value.
 *
 * @param ftmBase The FTM base address pointer
 * @param channel  The FTM peripheral channel number
 * @param val counter value to be set
 */
static inline void FTM_HAL_SetChnCountVal(FTM_Type* const ftmBase, uint8_t channel, uint16_t val)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_FTM_CHANNEL_COUNT);
#endif
    ((ftmBase)->CONTROLS[channel].CnV) = val;
}

/*!
 * @brief Gets the FTM peripheral timer channel counter value.
 *
 * @param ftmBase The FTM base address pointer
 * @param channel  The FTM peripheral channel number
 * @return return channel counter value
 */
static inline uint16_t FTM_HAL_GetChnCountVal(FTM_Type* const ftmBase, uint8_t channel)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_FTM_CHANNEL_COUNT);
#endif
    return (uint16_t)((ftmBase)->CONTROLS[channel].CnV);
}

/*!
 * @brief Gets the FTM peripheral timer  channel event status.
 *
 * @param ftmBase The FTM base address pointer
 * @param channel  The FTM peripheral channel number
 * @return return channel event status value
 */
static inline uint32_t FTM_HAL_GetChnEventStatus(FTM_Type* const ftmBase, uint8_t channel)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_FTM_CHANNEL_COUNT);
#endif
    return (((ftmBase)->STATUS) & (1U << channel));
    /*return BR_FTM_STATUS(ftmBase, channel);*/
}

/*!
 * @brief Gets the FTM peripheral timer status info for all channels.
 *
 * @param ftmBase The FTM base address pointer
 * @return return channel event status value
 */
static inline uint32_t FTM_HAL_GetEventStatus(FTM_Type* const ftmBase)
{
    return (((ftmBase)->STATUS) & (0xFFU));
}

/*!
 * @brief Clears the FTM peripheral timer all channel event status.
 *
 * @param ftmBase The FTM base address pointer
 * @param channel  The FTM peripheral channel number
 */
static inline void FTM_HAL_ClearChnEventStatus(FTM_Type* const ftmBase, uint8_t channel)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_FTM_CHANNEL_COUNT);
#endif
    ((ftmBase)->STATUS) |= (1U << channel);
}

/*!
 * @brief Writes the provided value to the OUTMASK register.
 *
 * This function will mask/unmask multiple channels.
 *
 * @param ftmBase The FTM base address pointer
 * @param regVal  value to be written to the register
 */
static inline void FTM_HAL_SetOutmaskReg(FTM_Type* const ftmBase, uint32_t regVal)
{
    ((ftmBase)->OUTMASK) = regVal;
}

/*!
 * @brief Sets the FTM peripheral timer channel output mask.
 *
 * @param ftmBase The FTM base address pointer
 * @param channel  The FTM peripheral channel number
 * @param mask mask to be set 0 or 1, unmasked or masked
 */
static inline void FTM_HAL_SetChnOutputMask(FTM_Type* const ftmBase, uint8_t channel, bool  mask)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_FTM_CHANNEL_COUNT);
#endif
    if(true == mask)
    {
        ((ftmBase)->OUTMASK)  |=    (1U << channel);
    }
    else
    {
        ((ftmBase)->OUTMASK)  &=    ~(1U << channel);
    }
}

/*!
 * @brief Sets the FTM peripheral timer channel output initial state 0 or 1.
 *
 * @param ftmBase The FTM base address pointer
 * @param channel  The FTM peripheral channel number
 * @param state counter value to be set 0 or 1
 */
static inline void FTM_HAL_SetChnOutputInitStateCmd(FTM_Type* const ftmBase, uint8_t channel,
                                                    bool state)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_FTM_CHANNEL_COUNT);
#endif
    if(true == state)
    {
        ((ftmBase)->OUTINIT) |= (1U << channel);
    }
    else
    {
        ((ftmBase)->OUTINIT) &= ~(1U << channel);
    }
}

/*!
 * @brief Sets the FTM peripheral timer channel output polarity.
 *
 * @param ftmBase The FTM base address pointer
 * @param channel  The FTM peripheral channel number
 * @param pol polarity to be set 0 or 1
 */
static inline void FTM_HAL_SetChnOutputPolarityCmd(FTM_Type* const ftmBase, uint8_t channel,
                                                    ftm_polarity_t pol)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_FTM_CHANNEL_COUNT);
#endif
    if (FTM_POLARITY_HIGH == pol)
    {
        ((ftmBase)->POL) &= ~(1U << channel);
    }
    else
    {
        ((ftmBase)->POL) |= (1U << channel);
    }
}

/*!
 * @brief Sets the FTM peripheral timer channel fault input polarity.
 *
 * @param ftmBase The FTM base address pointer
 * @param channel  The FTM peripheral channel number
 * @param pol polarity  to be set, 0: active high, 1:active low
 */
static inline void FTM_HAL_SetChnFaultInputPolarityCmd(FTM_Type* const ftmBase, uint8_t fltChannel,
                                                        bool pol)
{
    if (FTM_POLARITY_HIGH == pol)
    {
        ((ftmBase)->FLTPOL) &= ~(1U << fltChannel);
    }
    else
    {
        ((ftmBase)->FLTPOL) |= (1U << fltChannel);
    }
}

/*!
 * @brief Enables/disables the FTM peripheral timer fault interrupt.
 *
 * @param ftmBase The FTM base address pointer
 * @param state Timer fault interrupt state (true - enabled, false - disabled)
 */

static inline void FTM_HAL_SetFaultInt(FTM_Type* const ftmBase, bool state)
{
    FTM_RMW_MODE(ftmBase, FTM_MODE_FAULTIE_MASK, FTM_MODE_FAULTIE(state));
}

/*!
 * @brief Disables the FTM peripheral timer fault interrupt.
 *
 * @param ftmBase The FTM base address pointer
 */
static inline void FTM_HAL_DisableFaultInt(FTM_Type* const ftmBase)
{
    FTM_RMW_MODE(ftmBase, FTM_MODE_FAULTIE_MASK, FTM_MODE_FAULTIE(0U));
}

/*!
 * @brief Return true/false whether the Fault interrupt was enabled or not
 *
 * @param ftmBase The FTM base address pointer
 */
static inline bool FTM_HAL_IsFaultIntEnabled(FTM_Type* const ftmBase)
{
    return (BITBAND_ACCESS32(&((ftmBase)->MODE), FTM_MODE_FAULTIE_SHIFT)) ? true : false;
}

/*!
 * @brief Clears all fault interrupt flags that are active.
 *
 * @param ftmBase The FTM base address pointer
 */
static inline void FTM_HAL_ClearFaultsIsr(FTM_Type* const ftmBase)
{
    FTM_RMW_FMS(ftmBase, FTM_FMS_FAULTF0_MASK | FTM_FMS_FAULTF_MASK, FTM_FMS_FAULTF0(0U)
                | FTM_FMS_FAULTF(0U));
    FTM_RMW_FMS(ftmBase, FTM_FMS_FAULTF1_MASK | FTM_FMS_FAULTF_MASK, FTM_FMS_FAULTF1(0U)
                | FTM_FMS_FAULTF(0U));
    FTM_RMW_FMS(ftmBase, FTM_FMS_FAULTF2_MASK | FTM_FMS_FAULTF_MASK, FTM_FMS_FAULTF2(0U)
                | FTM_FMS_FAULTF(0U));
    FTM_RMW_FMS(ftmBase, FTM_FMS_FAULTF3_MASK | FTM_FMS_FAULTF_MASK, FTM_FMS_FAULTF3(0U)
                | FTM_FMS_FAULTF(0U));
}

/*!
 * @brief Defines the FTM fault control mode.
 *
 * @param ftmBase The FTM base address pointer
 * @param mode, 0: Fault control disabled
                1: Fault control enabled for even channel (0, 2, 4, 6) and manual fault clearing.
                2: Fault control enabled for all channels and manual fault clearing is enabled.
                3: Fault control enabled for all channels and automatic fault clearing is enabled.
 */
static inline void FTM_HAL_SetFaultControlMode(FTM_Type* const ftmBase, ftm_fault_mode_t mode)
{
    FTM_RMW_MODE(ftmBase, FTM_MODE_FAULTM_MASK, FTM_MODE_FAULTM((uint8_t)mode));
}

/*!
 * @brief Enables or disables the FTM peripheral timer capture test mode.
 *
 * @param ftmBase The FTM base address pointer
 * @param enable  true to enable capture test mode, false to disable
 */
static inline void FTM_HAL_SetCaptureTestCmd(FTM_Type* const ftmBase, bool enable)
{
    FTM_RMW_MODE(ftmBase, FTM_MODE_CAPTEST_MASK, FTM_MODE_CAPTEST(enable ? 1U : 0U));
}

/*!
 * @brief Enables or disables the FTM write protection.
 *
 * @param ftmBase The FTM base address pointer
 * @param enable  true: Write-protection is enabled, false: Write-protection is disabled
 */
static inline void FTM_HAL_SetWriteProtectionCmd(FTM_Type* const ftmBase, bool enable)
{
    if (true == enable)
    {
        BITBAND_ACCESS32(&((ftmBase)->FMS), FTM_FMS_WPEN_SHIFT) = (1U);
    }
    else
    {
        BITBAND_ACCESS32(&((ftmBase)->MODE), FTM_MODE_WPDIS_SHIFT) = (1U);
    }
}

/*!
 * @brief Enables the FTM peripheral timer group.
 *
 * @param ftmBase The FTM base address pointer
 * @param enable  true: all registers including FTM-specific registers are available
 *                false: only the TPM-compatible registers are available
 */
static inline void FTM_HAL_Enable(FTM_Type* const ftmBase, bool enable)
{
    FTM_RMW_MODE(ftmBase, FTM_MODE_FTMEN_MASK, FTM_MODE_FTMEN(enable ? 1U : 0U));
}

/*!
 * @brief Initializes the channels output.
 *
 * @param ftmBase The FTM base address pointer
 * @param enable  true: the channels output is initialized according to the state of OUTINIT reg
 *                false: has no effect
 */
static inline void FTM_HAL_SetInitChnOutputCmd(FTM_Type* const ftmBase, bool enable)
{
    FTM_RMW_MODE(ftmBase, FTM_MODE_INIT_MASK, FTM_MODE_INIT(enable ? 1U : 0U));
}

/*!
 * @brief Sets the FTM peripheral timer sync mode.
 *
 * @param ftmBase The FTM base address pointer
 * @param enable  true: no restriction both software and hardware triggers can be used\n
 *                false: software trigger can only be used for MOD and CnV synch, hardware trigger
 *                       only for OUTMASK and FTM counter synch.
 */
static inline void FTM_HAL_SetPwmSyncMode(FTM_Type* const ftmBase, bool enable)
{
    FTM_RMW_MODE(ftmBase, FTM_MODE_PWMSYNC_MASK, FTM_MODE_PWMSYNC(enable ? 1U : 0U));
}


/*FTM synchronization control*/
/*!
 * @brief Enables or disables the FTM peripheral timer software trigger.
 *
 * @param ftmBase The FTM base address pointer.
 * @param enable  true: software trigger is selected, false: software trigger is not selected
 */
static inline void FTM_HAL_SetSoftwareTriggerCmd(FTM_Type* const ftmBase, bool enable)
{
    FTM_RMW_SYNC(ftmBase, FTM_SYNC_SWSYNC_MASK, FTM_SYNC_SWSYNC(enable ? 1U : 0U));
}

/*!
 * @brief Sets the FTM peripheral timer hardware trigger.
 *
 * @param ftmBase The FTM base address pointer
 * @param trigger_num  0, 1, 2 for trigger0, trigger1 and trigger3
 * @param enable true: enable hardware trigger from field trigger_num for PWM synch
 *               false: disable hardware trigger from field trigger_num for PWM synch
 */
void FTM_HAL_SetHardwareSyncTriggerSrc(FTM_Type* const ftmBase, uint32_t trigger_num, bool enable);

/*!
 * @brief Determines when the OUTMASK register is updated with the value of its buffer.
 *
 * @param ftmBase The FTM base address pointer
 * @param enable  true if OUTMASK register is updated only by PWM sync\n
 *                false if OUTMASK register is updated in all rising edges of the system clock
 */
static inline void FTM_HAL_SetOutmaskPwmSyncModeCmd(FTM_Type* const ftmBase, bool enable)
{
    FTM_RMW_SYNC(ftmBase, FTM_SYNC_SYNCHOM_MASK, FTM_SYNC_SYNCHOM(enable ? 1U : 0U));
}

/*!
 * @brief Determines if the FTM counter is re-initialized when the selected trigger for
 * synchronization is detected.
 *
 * @param ftmBase The FTM base address pointer
 * @param enable  true to update FTM counter when triggered , false to count normally
 */
static inline void FTM_HAL_SetCountReinitSyncCmd(FTM_Type* const ftmBase, bool enable)
{
    FTM_RMW_SYNC(ftmBase, FTM_SYNC_REINIT_MASK, FTM_SYNC_REINIT(enable ? 1U : 0U));
}

/*!
 * @brief Enables or disables the FTM peripheral timer maximum loading points.
 *
 * @param ftmBase The FTM base address pointer
 * @param enable  true to enable maximum loading point, false to disable
 */
static inline void FTM_HAL_SetMaxLoadingCmd(FTM_Type* const ftmBase, bool enable)
{
    FTM_RMW_SYNC(ftmBase, FTM_SYNC_CNTMAX_MASK, FTM_SYNC_CNTMAX(enable ? 1U : 0U));
}
/*!
 * @brief Enables or disables the FTM peripheral timer minimum loading points.
 *
 * @param ftmBase The FTM base address pointer
 * @param enable  true to enable minimum loading point, false to disable
 */
static inline void FTM_HAL_SetMinLoadingCmd(FTM_Type* const ftmBase, bool enable)
{
    FTM_RMW_SYNC(ftmBase, FTM_SYNC_CNTMIN_MASK, FTM_SYNC_CNTMIN(enable ? 1U : 0U));
}



/*!
 * @brief Enables the FTM peripheral timer channel pair fault control.
 *
 * @param ftmBase The FTM base address pointer
 * @param chnlPairNum The FTM peripheral channel pair number
 * @param enable  true to enable fault control, false to disable
 */
static inline  void FTM_HAL_SetDualChnFaultCmd(FTM_Type* const ftmBase, uint8_t chnlPairNum,
                                               bool enable)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(chnlPairNum < (FSL_FEATURE_FTM_CHANNEL_COUNT / 2));
#endif
    if (true == enable)
    {
        ((ftmBase)->COMBINE) |= (FTM_COMBINE_FAULTEN0_MASK <<
                                (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
    else
    {
        ((ftmBase)->COMBINE) &= ~(FTM_COMBINE_FAULTEN0_MASK <<
                                 (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
}

/*!
 * @brief Enables or disables the FTM peripheral timer channel pair counter PWM sync.
 *
 * @param ftmBase The FTM base address pointer
 * @param chnlPairNum The FTM peripheral channel pair number
 * @param enable  true to enable PWM synchronization, false to disable
 */
static inline void FTM_HAL_SetDualChnPwmSyncCmd(FTM_Type* const ftmBase, uint8_t chnlPairNum,
                                                bool enable)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(chnlPairNum < (FSL_FEATURE_FTM_CHANNEL_COUNT / 2));
#endif
    if (true == enable)
    {
        ((ftmBase)->COMBINE) |= (FTM_COMBINE_SYNCEN0_MASK <<
                                (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
    else
    {
        ((ftmBase)->COMBINE) &= ~(FTM_COMBINE_SYNCEN0_MASK <<
                                 (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
}

/*!
 * @brief Enables or disabled the FTM peripheral timer channel pair deadtime insertion.
 *
 * @param ftmBase The FTM base address pointer
 * @param chnlPairNum The FTM peripheral channel pair number
 * @param enable  true to enable deadtime insertion, false to disable
 */
static inline void FTM_HAL_SetDualChnDeadtimeCmd(FTM_Type* const ftmBase, uint8_t chnlPairNum, bool enable)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(chnlPairNum < FSL_FEATURE_FTM_CHANNEL_COUNT);
#endif
    if (true == enable)
    {
        ((ftmBase)->COMBINE) |= (FTM_COMBINE_DTEN0_MASK <<
                                (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
    else
    {
        ((ftmBase)->COMBINE) &= ~(FTM_COMBINE_DTEN0_MASK <<
                                 (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
}

/*!
 * @brief Enables or disables the FTM peripheral timer channel dual edge capture.
 *
 * @param ftmBase The FTM base address pointer
 * @param chnlPairNum The FTM peripheral channel pair number
 * @param enable  true to enable dual edge capture mode, false to disable
 */
static inline void FTM_HAL_SetDualChnDecapCmd(FTM_Type* const ftmBase, uint8_t chnlPairNum, bool enable)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(chnlPairNum < (FSL_FEATURE_FTM_CHANNEL_COUNT / 2));
#endif
    if (true == enable)
    {
        ((ftmBase)->COMBINE) |= (FTM_COMBINE_DECAP0_MASK <<
                                (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
    else
    {
        ((ftmBase)->COMBINE) &= ~(FTM_COMBINE_DECAP0_MASK <<
                                 (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
}

/*!
 * @brief Enables the FTM peripheral timer dual edge capture mode.
 *
 * @param ftmBase The FTM base address pointer
 * @param chnlPairNum The FTM peripheral channel pair number
 * @param enable  true to enable dual edge capture, false to disable
 */
static inline void FTM_HAL_SetDualEdgeCaptureCmd(FTM_Type* const ftmBase, uint8_t chnlPairNum,
                                                 bool enable)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(chnlPairNum < (FSL_FEATURE_FTM_CHANNEL_COUNT / 2));
#endif
    if (true == enable)
    {
        ((ftmBase)->COMBINE) |= (FTM_COMBINE_DECAPEN0_MASK <<
                                (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
    else
    {
        ((ftmBase)->COMBINE) &= ~(FTM_COMBINE_DECAPEN0_MASK <<
                                 (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
}

/*!
 * @brief Enables the FTM peripheral timer dual edge capture mode.
 *
 * @param ftmBase The FTM base address pointer
 * @param chnlPairNum The FTM peripheral channel pair number
 * @param enable  true to enable dual edge capture, false to disable
 */
static inline bool FTM_HAL_GetDualEdgeCaptureBit(FTM_Type* const ftmBase, uint8_t chnlPairNum)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(chnlPairNum < (FSL_FEATURE_FTM_CHANNEL_COUNT / 2));
#endif
    return (((ftmBase)->COMBINE) & (FTM_COMBINE_DECAPEN0_MASK <<
           (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH))  ? true : false);
}


/*!
 * @brief Enables or disables the FTM peripheral timer channel pair output complement mode.
 *
 * @param ftmBase The FTM base address pointer
 * @param chnlPairNum The FTM peripheral channel pair number
 * @param enable  true to enable complementary mode, false to disable
 */
static inline void FTM_HAL_SetDualChnCompCmd(FTM_Type* const ftmBase, uint8_t chnlPairNum,
                                            ftm_second_channel_polarity_t polarity)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(chnlPairNum < (FSL_FEATURE_FTM_CHANNEL_COUNT / 2));
#endif
    if (polarity == FTM_MAIN_INVERTED)
    {
        ((ftmBase)->COMBINE) |= (FTM_COMBINE_COMP0_MASK <<
                                (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
    else
    {
        ((ftmBase)->COMBINE) &= ~(FTM_COMBINE_COMP0_MASK <<
                                 (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
}

/*!
 * @brief Enables or disables the FTM peripheral timer channel pair output combine mode.
 *
 * @param ftmBase The FTM base address pointer
 * @param chnlPairNum The FTM peripheral channel pair number
 * @param enable  true to enable channel pair to combine, false to disable
 */
static inline void FTM_HAL_SetDualChnCombineCmd(FTM_Type* const ftmBase, uint8_t chnlPairNum,
                                                bool enable)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(chnlPairNum < (FSL_FEATURE_FTM_CHANNEL_COUNT / 2));
#endif
    if (true == enable)
    {
        ((ftmBase)->COMBINE) |= (FTM_COMBINE_COMBINE0_MASK <<
                                (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
    else
    {
        ((ftmBase)->COMBINE) &= ~(FTM_COMBINE_COMBINE0_MASK <<
                                 (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
}



/*FTM dead time insertion control*/
/*!
 * @brief Sets the FTM dead time divider.
 *
 * @param ftmBase The FTM base address pointer
 * @param divider  The FTM peripheral prescaler divider\n
 *                 0x :divided by 1, 10: divided by 4, 11:divided by 16
 */
static inline void FTM_HAL_SetDeadtimePrescale(FTM_Type* const ftmBase, ftm_deadtime_ps_t divider)
{
    FTM_RMW_DEADTIME(ftmBase, FTM_DEADTIME_DTPS_MASK, FTM_DEADTIME_DTPS((uint8_t)divider));
}

/*!
 * @brief Sets the FTM deadtime value.
 *
 * @param ftmBase The FTM base address pointer
 * @param count  The FTM peripheral  prescale divider\n
 *               0: no counts inserted, 1: 1 count is inserted, 2: 2 count is inserted....
 */
static inline void FTM_HAL_SetDeadtimeCount(FTM_Type* const ftmBase, uint8_t count)
{
    FTM_RMW_DEADTIME(ftmBase, FTM_DEADTIME_DTVAL_MASK, FTM_DEADTIME_DTVAL(count));
}

/*!
 * @brief Enables or disables the generation of the trigger when the FTM counter is equal
 *to the CNTIN register.
 *
 * @param ftmBase The FTM base address pointer
 * @param enable  true to enable, false to disable
 */
static inline void FTM_HAL_SetInitTriggerCmd(FTM_Type* const ftmBase, bool enable)
{
    BITBAND_ACCESS32(&((ftmBase)->EXTTRIG), FTM_EXTTRIG_INITTRIGEN_SHIFT) = (enable ? 1U : 0U);
}

/*FTM external trigger */
/*!
 * @brief Enables or disables the generation of the FTM peripheral timer channel trigger.
 *
 * Enables or disables the generation of the FTM peripheral timer channel trigger when the
 * FTM counter is equal to its initial value. Channels 6 and 7 cannot be used as triggers.
 *
 * @param ftmBase The FTM base address pointer
 * @param channel Channel to be enabled,  valid value 0, 1, 2, 3, 4, 5
 * @param val  true to enable, false to disable
 */
void FTM_HAL_SetChnTriggerCmd(FTM_Type* const ftmBase, uint8_t channel, bool val);

/*!
 * @brief Checks whether any channel trigger event has occurred.
 *
 * @param ftmBase The FTM base address pointer
 * @return true if there is a channel trigger event, false if not.
 */
static inline bool FTM_HAL_IsChnTriggerGenerated(FTM_Type* const ftmBase)
{
    return ((BITBAND_ACCESS32(&((ftmBase)->EXTTRIG), FTM_EXTTRIG_TRIGF_SHIFT)) ? true : false);
}

/*Fault mode status*/
/*!
 * @brief Gets the FTM detected fault input.
 *
 * This function reads the status for all fault inputs
 *
 * @param ftmBase The FTM base address pointer
 * @return Return fault byte
 */
static inline uint8_t FTM_HAL_GetDetectedFaultInput(FTM_Type* const ftmBase)
{
    return (((ftmBase)->FMS) & 0x0f);
}

/*!
 * @brief Checks whether the write protection is enabled.
 *
 * @param ftmBase The FTM base address pointer
 * @return true if enabled, false if not
 */
static inline bool FTM_HAL_IsWriteProtectionEnabled(FTM_Type* const ftmBase)
{
    return BITBAND_ACCESS32(&((ftmBase)->FMS), FTM_FMS_WPEN_SHIFT) ? true : false;
}

/*Quadrature decoder control*/

/*!
 * @brief Enables the channel quadrature decoder.
 *
 * @param ftmBase The FTM base address pointer
 * @param enable  true to enable, false to disable
 */
static inline void FTM_HAL_SetQuadDecoderCmd(FTM_Type* const ftmBase, bool enable)
{
    /* BITBAND_ACCESS32(&((ftmBase)->QDCTRL), FTM_QDCTRL_QUADEN_SHIFT) = (enable ? 1U : 0U);*/
    if (true == enable)
    {
        ((ftmBase)->QDCTRL) |= (1U << FTM_QDCTRL_QUADEN_SHIFT);
    }
    else
    {
        ((ftmBase)->QDCTRL) &= ~(1U << FTM_QDCTRL_QUADEN_SHIFT);
    }
}

/*!
 * @brief Enables or disables the phase A input filter.
 *
 * @param ftmBase The FTM base address pointer
 * @param enable true enables the phase input filter, false disables the filter
 */
static inline void FTM_HAL_SetQuadPhaseAFilterCmd(FTM_Type* const ftmBase, bool enable)
{
    /* BITBAND_ACCESS32(&((ftmBase)->QDCTRL), FTM_QDCTRL_PHAFLTREN_SHIFT) = (enable ? 1U : 0U);*/
    if (true == enable)
    {
        ((ftmBase)->QDCTRL) |= (1U << FTM_QDCTRL_PHAFLTREN_SHIFT);
    }
    else
    {
        ((ftmBase)->QDCTRL) &= ~(1U << FTM_QDCTRL_PHAFLTREN_SHIFT);
    }
}

/*!
 * @brief Enables or disables the phase B input filter.
 *
 * @param ftmBase The FTM base address pointer
 * @param enable true enables the phase input filter, false disables the filter
 */
static inline void FTM_HAL_SetQuadPhaseBFilterCmd(FTM_Type* const ftmBase, bool enable)
{
    /* BITBAND_ACCESS32(&((ftmBase)->QDCTRL), FTM_QDCTRL_PHBFLTREN_SHIFT) = (enable ? 1U : 0U);*/
    if (true == enable)
    {
        ((ftmBase)->QDCTRL) |= (1U << FTM_QDCTRL_PHBFLTREN_SHIFT);
    }
    else
    {
        ((ftmBase)->QDCTRL) &= ~(1U << FTM_QDCTRL_PHBFLTREN_SHIFT);
    }
}

/*!
 * @brief Selects polarity for the quadrature decode phase A input.
 *
 * @param ftmBase The FTM base address pointer
 * @param mode 0: Normal polarity, 1: Inverted polarity
 */
static inline void FTM_HAL_SetQuadPhaseAPolarity(FTM_Type* const ftmBase,
        ftm_quad_phase_polarity_t mode)
{
    /*BITBAND_ACCESS32(&((ftmBase)->QDCTRL), FTM_QDCTRL_PHAPOL_SHIFT) = (uint8_t)(mode);*/
    FTM_RMW_QDCTRL(ftmBase, FTM_QDCTRL_PHAPOL_MASK, FTM_QDCTRL_PHAPOL((uint8_t)mode));

}

/*!
 * @brief Selects polarity for the quadrature decode phase B input.
 *
 * @param ftmBase The FTM base address pointer
 * @param mode 0: Normal polarity, 1: Inverted polarity
 */
static inline void FTM_HAL_SetQuadPhaseBPolarity(FTM_Type* const ftmBase,
        ftm_quad_phase_polarity_t mode)
{
    /*BITBAND_ACCESS32(&((ftmBase)->QDCTRL), FTM_QDCTRL_PHBPOL_SHIFT) =(uint8_t)(mode);*/
    FTM_RMW_QDCTRL(ftmBase, FTM_QDCTRL_PHBPOL_MASK, FTM_QDCTRL_PHBPOL((uint8_t)mode));
}

/*!
 * @brief Sets the encoding mode used in quadrature decoding mode.
 *
 * @param ftmBase The FTM base address pointer
 * @param quadMode 0: Phase A and Phase B encoding mode\n
 *                 1: Count and direction encoding mode
 */
static inline void FTM_HAL_SetQuadMode(FTM_Type* const ftmBase, ftm_quad_decode_mode_t quadMode)
{
    /*BITBAND_ACCESS32(&((ftmBase)->QDCTRL), FTM_QDCTRL_QUADMODE_SHIFT) = (uint8_t)(quadMode);*/
    FTM_RMW_QDCTRL(ftmBase, FTM_QDCTRL_QUADMODE_MASK, FTM_QDCTRL_QUADMODE((uint8_t)quadMode));
}

/*!
 * @brief Gets the FTM counter direction in quadrature mode.
 *
 * @param ftmBase The FTM base address pointer
 *
 * @return 1 if counting direction is increasing, 0 if counting direction is decreasing
 */
static inline uint8_t FTM_HAL_GetQuadDir(FTM_Type* const ftmBase)
{
    return BITBAND_ACCESS32(&((ftmBase)->QDCTRL), FTM_QDCTRL_QUADMODE_SHIFT);
}

/*!
 * @brief Gets the Timer overflow direction in quadrature mode.
 *
 * @param ftmBase The FTM base address pointer
 *
 * @return 1 if TOF bit was set on the top of counting,
 *         0 if TOF bit was set on the bottom of counting
 */
static inline uint8_t FTM_HAL_GetQuadTimerOverflowDir(FTM_Type* const ftmBase)
{
    return BITBAND_ACCESS32(&((ftmBase)->QDCTRL), FTM_QDCTRL_TOFDIR_SHIFT);
}


/*!
 * @brief Sets the fault input filter value.
 *
 * @param ftmBase The FTM base address pointer
 * @param value fault input filter value
 */
static inline void FTM_HAL_SetFaultInputFilterVal(FTM_Type* const ftmBase, uint32_t value)
{
    FTM_RMW_FLTCTRL(ftmBase, FTM_FLTCTRL_FFVAL_MASK, FTM_FLTCTRL_FFVAL(value));
}

/*!
 * @brief Enables or disables the fault input filter.
 *
 * @param ftmBase The FTM base address pointer
 * @param inputNum fault input to be configured, valid value 0, 1, 2, 3
 * @param enable  true to enable fault input filter, false to disable fault input filter
 */
static inline void FTM_HAL_SetFaultInputFilterCmd(FTM_Type* const ftmBase, uint8_t inputNum,
                                                 bool enable)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(inputNum < CHAN4_IDX);
#endif
    if(true == enable)
    {
        ((ftmBase)->FLTCTRL) |=  (1U << (inputNum + FTM_FLTCTRL_FFLTR0EN_SHIFT));
    }
    else
    {
        ((ftmBase)->FLTCTRL) &=  ~(1U << (inputNum + FTM_FLTCTRL_FFLTR0EN_SHIFT));
    }
}


/*!
 * @brief Clears the entire content value of the Fault control register.
 *
 * @param ftmBase The FTM base address pointer
 */
static inline void FTM_HAL_ClearFaultControl(FTM_Type* const ftmBase)
{
    ((ftmBase)->FLTCTRL) =  0U;
}

/*!
 * @brief Enables or disables the fault input.
 *
 * @param ftmBase The FTM base address pointer
 * @param inputNum fault input to be configured, valid value 0, 1, 2, 3
 * @param enable  true to enable fault input, false to disable fault input
 */
static inline void FTM_HAL_SetFaultInputCmd(FTM_Type* const ftmBase, uint8_t inputNum, bool enable)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(inputNum < CHAN4_IDX);
#endif
    if(true == enable)
    {
        ((ftmBase)->FLTCTRL) |=  (1U << inputNum);
    }
    else
    {
        ((ftmBase)->FLTCTRL) &=  ~(1U << inputNum);
    }
}


/*!
 * @brief Configures the behaviour of the PWM outputs when a fault is detected
 *
 * @param ftmBase The FTM base address pointer
 * @param enable  true output pins are set tri-state,
                  false pins are set to a safe state determined by POL bits
 */
static inline void FTM_HAL_SetPwmFaultBehavior(FTM_Type* const ftmBase,bool enable)
{
    if(true == enable)
    {
        ((ftmBase)->FLTCTRL) |=  (1U << FTM_FLTCTRL_FSTATE_SHIFT);
    }
    else
    {
        ((ftmBase)->FLTCTRL) &=  ~(1U << FTM_FLTCTRL_FSTATE_SHIFT);
    }
}



/*!
 * @brief Enables or disables the channel invert for a channel pair.
 *
 * @param ftmBase The FTM base address pointer
 * @param chnlPairNum The FTM peripheral channel pair number
 * @param enable  true to enable channel inverting, false to disable channel inversion
 */
static inline void FTM_HAL_SetDualChnInvertCmd(FTM_Type* const ftmBase, uint8_t chnlPairNum,
                                                bool enable)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(chnlPairNum < (FSL_FEATURE_FTM_CHANNEL_COUNT / 2));
#endif
    if(true == enable)
    {
        ((ftmBase)->INVCTRL) |=  (1U << chnlPairNum);
    }
    else
    {
        ((ftmBase)->INVCTRL) &=  ~(1U << chnlPairNum);
    }
}

/*!
 * @brief Writes the provided value to the Inverting control register.
 *
 * This function is enable/disable inverting control on multiple channel pairs.
 *
 * @param ftmBase The FTM base address pointer
 * @param regVal  value to be written to the register
 */
static inline void FTM_HAL_SetInvctrlReg(FTM_Type* const ftmBase, uint32_t regVal)
{
    ((ftmBase)->INVCTRL) = regVal;
}

/*FTM software output control*/
/*!
 * @brief Enables or disables the channel software output control.
 * @param ftmBase The FTM base address pointer
 * @param channel Channel to be enabled or disabled
 * @param enable  true to enable, channel output will be affected by software output control\n
                false to disable, channel output is unaffected
 */
static inline void FTM_HAL_SetChnSoftwareCtrlCmd(FTM_Type* const ftmBase, uint8_t channel,
                                                bool enable)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_FTM_CHANNEL_COUNT);
#endif
    if(true == enable)
    {
        ((ftmBase)->SWOCTRL) |=  (1U << channel);
    }
    else
    {
        ((ftmBase)->SWOCTRL) &=  ~(1U << channel);
    }
}
/*!
 * @brief Sets the channel software output control value.
 *
 * @param ftmBase The FTM base address pointer.
 * @param channel Channel to be configured
 * @param enable  true to set 1, false to set 0
 */
static inline void FTM_HAL_SetChnSoftwareCtrlVal(FTM_Type* const ftmBase, uint8_t channel,
                                                bool enable)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_FTM_CHANNEL_COUNT);
#endif
    if(true == enable)
    {
        ((ftmBase)->SWOCTRL) |=  (1U << (channel + FTM_SWOCTRL_CH0OCV_SHIFT));
    }
    else
    {
        ((ftmBase)->SWOCTRL) &=  ~(1U << (channel + FTM_SWOCTRL_CH0OCV_SHIFT));
    }
}

/*FTM PWM load control*/
/*!
 * @brief Enables or disables the loading of MOD, CNTIN and CV with values of their write buffer.
 *
 * @param ftmBase The FTM base address pointer
 * @param enable  true to enable, false to disable
 */
static inline void FTM_HAL_SetPwmLoadCmd(FTM_Type* const ftmBase, bool enable)
{
    /*BITBAND_ACCESS32(&((ftmBase)->PWMLOAD), FTM_PWMLOAD_LDOK_SHIFT) = ( enable ? 1U : 0U);*/
    if(true == enable)
    {
        ((ftmBase)->PWMLOAD) |=  (1U << FTM_PWMLOAD_LDOK_SHIFT );
    }
    else
    {
        ((ftmBase)->PWMLOAD) &=  ~(1U << FTM_PWMLOAD_LDOK_SHIFT);
    }
}

/*!
 * @brief Includes or excludes the channel in the matching process.
 *
 * @param ftmBase The FTM base address pointer
 * @param channel Channel to be configured
 * @param val   true means include the channel in the matching process\n
 *              false means do not include channel in the matching process
 */
static inline void FTM_HAL_SetPwmLoadChnSelCmd(FTM_Type* const ftmBase, uint8_t channel, bool enable)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_FTM_CHANNEL_COUNT);
#endif
    if(true == enable)
    {
        ((ftmBase)->PWMLOAD) |=  (1U << channel );
    }
    else
    {
        ((ftmBase)->PWMLOAD) &=  ~(1U << channel);
    }
}

/*FTM configuration*/
/*!
 * @brief Enables or disables the FTM global time base signal generation to other FTM's.
 *
 * @param ftmBase The FTM base address pointer
 * @param enable  true to enable, false to disable
 */
static inline void FTM_HAL_SetGlobalTimeBaseOutputCmd(FTM_Type* const ftmBase, bool enable)
{
    BITBAND_ACCESS32(&((ftmBase)->CONF), FTM_CONF_GTBEOUT_SHIFT) = ( enable ? 1 : 0);
}

/*!
 * @brief Enables or disables the FTM timer global time base.
 *
 * @param ftmBase The FTM base address pointer
 * @param enable  true to enable, false to disable
 */
static inline void FTM_HAL_SetGlobalTimeBaseCmd(FTM_Type* const ftmBase, bool enable)
{
    BITBAND_ACCESS32(&((ftmBase)->CONF), FTM_CONF_GTBEEN_SHIFT) = (enable ? 1U : 0U);
}

/*!
 * @brief Sets the BDM mode.
 *
 * @param ftmBase The FTM base address pointer
 * @param val FTM behaviour in BDM mode, options are defined in the enum ftm_bdm_mode_t
 */
static inline void FTM_HAL_SetBdmMode(FTM_Type* const ftmBase, ftm_bdm_mode_t val)
{
    FTM_RMW_CONF(ftmBase, FTM_CONF_BDMMODE_MASK, FTM_CONF_BDMMODE((uint8_t)val));
}

/*!
 * @brief Sets the FTM timer TOF Frequency
 *
 * @param ftmBase The FTM base address pointer
 * @param val  Value of the TOF bit set frequency
 */
static inline void FTM_HAL_SetLoadFreq(FTM_Type* const ftmBase, uint8_t val)
{
    FTM_RMW_CONF(ftmBase, FTM_CONF_LDFQ_MASK, FTM_CONF_LDFQ((uint8_t)val));
}



/*!
 * @brief Sets the sync mode for the FTM SWOCTRL register when using a hardware trigger.
 *
 * @param ftmBase The FTM base address pointer
 * @param enable  true means the hardware trigger activates register sync\n
 *                false means the hardware trigger does not activate register sync.
 */
static inline void FTM_HAL_SetSwoctrlHardwareSyncModeCmd(FTM_Type* const ftmBase, bool enable)
{
    BITBAND_ACCESS32(&((ftmBase)->SYNCONF), FTM_SYNCONF_HWSOC_SHIFT) = (enable ? 1U : 0U);
}

/*!
 * @brief Sets sync mode for FTM INVCTRL register when using a hardware trigger.
 *
 * @param ftmBase The FTM base address pointer
 * @param enable  true means the hardware trigger activates register sync\n
 *                false means the hardware trigger does not activate register sync.
 */
static inline void FTM_HAL_SetInvctrlHardwareSyncModeCmd(FTM_Type* const ftmBase, bool enable)
{
    BITBAND_ACCESS32(&((ftmBase)->SYNCONF), FTM_SYNCONF_HWINVC_SHIFT) = (enable ? 1U : 0U);
}

/*!
 * @brief Sets sync mode for FTM OUTMASK register when using a hardware trigger.
 *
 * @param ftmBase The FTM base address pointer
 * @param enable  true means hardware trigger activates register sync\n
 *                false means hardware trigger does not activate register sync.
 */
static inline void FTM_HAL_SetOutmaskHardwareSyncModeCmd(FTM_Type* const ftmBase, bool enable)
{
    BITBAND_ACCESS32(&((ftmBase)->SYNCONF), FTM_SYNCONF_HWOM_SHIFT) = (enable ? 1U : 0U);
}

/*!
 * @brief Sets sync mode for FTM MOD, CNTIN and CV registers when using a hardware trigger.
 *
 * @param ftmBase The FTM base address pointer
 * @param enable  true means hardware trigger activates register sync\n
 *                false means hardware trigger does not activate register sync.
 */
static inline void FTM_HAL_SetModCntinCvHardwareSyncModeCmd(FTM_Type* const ftmBase, bool enable)
{
    BITBAND_ACCESS32(&((ftmBase)->SYNCONF), FTM_SYNCONF_HWWRBUF_SHIFT) = (enable ? 1U : 0U);
}

/*!
 * @brief Sets sync mode for FTM counter register when using a hardware trigger.
 *
 * @param ftmBase The FTM base address pointer
 * @param enable  true means hardware trigger activates register sync\n
 *                false means hardware trigger does not activate register sync.
 */
static inline void FTM_HAL_SetCounterHardwareSyncModeCmd(FTM_Type* const ftmBase, bool enable)
{
    BITBAND_ACCESS32(&((ftmBase)->SYNCONF), FTM_SYNCONF_HWRSTCNT_SHIFT) = (enable ? 1U : 0U);
}

/*!
 * @brief Sets sync mode for FTM SWOCTRL register when using a software trigger.
 *
 * @param ftmBase The FTM base address pointer
 * @param enable  true means software trigger activates register sync\n
 *                false means software trigger does not activate register sync.
 */
static inline void FTM_HAL_SetSwoctrlSoftwareSyncModeCmd(FTM_Type* const ftmBase, bool enable)
{
    BITBAND_ACCESS32(&((ftmBase)->SYNCONF), FTM_SYNCONF_SWSOC_SHIFT) = (enable ? 1U : 0U);
}

/*!
 * @brief Sets sync mode for FTM INVCTRL register when using a software trigger.
 *
 * @param ftmBase The FTM base address pointer
 * @param enable  true means software trigger activates register sync\n
 *                false means software trigger does not activate register sync.
 */
static inline void FTM_HAL_SetInvctrlSoftwareSyncModeCmd(FTM_Type* const ftmBase, bool enable)
{
    BITBAND_ACCESS32(&((ftmBase)->SYNCONF), FTM_SYNCONF_SWINVC_SHIFT) = (enable ? 1U : 0U);
}

/*!
 * @brief Sets sync mode for FTM OUTMASK register when using a software trigger.
 *
 * @param ftmBase The FTM base address pointer
 * @param enable  true means software trigger activates register sync\n
 *                false means software trigger does not activate register sync.
 */
static inline void FTM_HAL_SetOutmaskSoftwareSyncModeCmd(FTM_Type* const ftmBase, bool enable)
{
    BITBAND_ACCESS32(&((ftmBase)->SYNCONF), FTM_SYNCONF_SWOM_SHIFT) = (enable ? 1U : 0U);
}

/*!
 * @brief Sets synch mode for FTM MOD, CNTIN and CV registers when using a software trigger.
 *
 * @param ftmBase The FTM base address pointer
 * @param enable  true means software trigger activates register sync\n
 *                false means software trigger does not activate register sync.
 */
static inline void FTM_HAL_SetModCntinCvSoftwareSyncModeCmd(FTM_Type* const ftmBase, bool enable)
{
    BITBAND_ACCESS32(&((ftmBase)->SYNCONF), FTM_SYNCONF_SWWRBUF_SHIFT) = (enable ? 1U : 0U);
}

/*!
 * @brief Sets hardware trigger mode.
 *
 * @param ftmBase The FTM base address pointer
 * @param enable  true FTM does not clear the TRIGx bit when the hardware trigger j is detected
 *                false FTM clears the TRIGx bit when the hardware trigger j is detected
 */
static inline void FTM_HAL_SetHwTriggerSyncModeCmd(FTM_Type* const ftmBase, bool enable)
{
    BITBAND_ACCESS32(&((ftmBase)->SYNCONF), FTM_SYNCONF_HWTRIGMODE_SHIFT) = (enable ? 1U : 0U);
}

/*!
 * @brief Sets sync mode for FTM counter register when using a software trigger.
 *
 * @param ftmBase The FTM base address pointer
 * @param enable  true means software trigger activates register sync
 *                false means software trigger does not activate register sync.
 */
static inline void FTM_HAL_SetCounterSoftwareSyncModeCmd(FTM_Type* const ftmBase, bool update_mode)
{
    BITBAND_ACCESS32(&((ftmBase)->SYNCONF), FTM_SYNCONF_SWRSTCNT_SHIFT) = (update_mode ? 1U : 0U);
}

/*!
 * @brief Sets the PWM synchronization mode to enhanced or legacy.
 *
 * @param ftmBase The FTM base address pointer
 * @param enable  true means use Enhanced PWM synchronization\n
 *                false means to use Legacy mode
 */
static inline void FTM_HAL_SetPwmSyncModeCmd(FTM_Type* const ftmBase, bool mode)
{
    BITBAND_ACCESS32(&((ftmBase)->SYNCONF), FTM_SYNCONF_SYNCMODE_SHIFT) = (mode ? 1U : 0U);
}

/*!
 * @brief Sets the SWOCTRL register PWM synchronization mode.
 *
 * @param ftmBase The FTM base address pointer
 * @param enable  true means SWOCTRL register is updated by PWM synch\n
 *                false means SWOCTRL register is updated at all rising edges of system clock
 */
static inline void FTM_HAL_SetSwoctrlPwmSyncModeCmd(FTM_Type* const ftmBase, ftm_reg_update_t mode)
{
    BITBAND_ACCESS32(&((ftmBase)->SYNCONF), FTM_SYNCONF_SWOC_SHIFT) = (mode ? 1U : 0U);
}

/*!
 * @brief Sets the INVCTRL register PWM synchronization mode.
 *
 * @param ftmBase The FTM base address pointer
 * @param enable  true means INVCTRL register is updated by PWM synch\n
 *                false means INVCTRL register is updated at all rising edges of system clock
 */
static inline void FTM_HAL_SetInvctrlPwmSyncModeCmd(FTM_Type* const ftmBase, ftm_reg_update_t mode)
{
    BITBAND_ACCESS32(&((ftmBase)->SYNCONF), FTM_SYNCONF_INVC_SHIFT) = (mode ? 1U : 0U);
}

/*!
 * @brief Sets the CNTIN register PWM synchronization mode.
 *
 * @param ftmBase The FTM base address pointer
 * @param enable  true means CNTIN register is updated by PWM synch\n
 *                false means CNTIN register is updated at all rising edges of system clock
 */
static inline void FTM_HAL_SetCntinPwmSyncModeCmd(FTM_Type* const ftmBase, ftm_reg_update_t mode)
{
    BITBAND_ACCESS32(&((ftmBase)->SYNCONF), FTM_SYNCONF_CNTINC_SHIFT) = (mode ? 1U : 0U);
}


/*!
 * @brief Combines the channel control.
 *
 * Returns an index for each channel pair.
 *
 * @param channel  The FTM peripheral channel number.
 * @return 0 for channel pair 0 & 1
 *         1 for channel pair 2 & 3
 *         2 for channel pair 4 & 5
 *         3 for channel pair 6 & 7
 */
uint32_t FTM_HAL_GetChnPairIndex(uint8_t channel);


/*HAL functionality*/
/*!
 * @brief Resets the FTM registers
 *
 * @param ftmBase The FTM base address pointer
 */
void FTM_HAL_Reset(FTM_Type* const ftmBase);

/*!
 * @brief Initializes the FTM.
 *
 * @param ftmBase The FTM base address pointer.
 */
void FTM_HAL_Init(FTM_Type* const ftmBase, ftm_clock_ps_t FtmClockPrescaler);

/*!
 * @brief Disables the PWM output mode.
 *
 * @param ftmBase The FTM base address pointer
 * @param config PWM configuration parameter
 * @param channel The channel or channel pair number(combined mode).
 */
void FTM_HAL_DisablePwmMode(FTM_Type* const ftmBase, uint8_t channel);

/*FTM sync configuration*/

/*!
 * @brief Sets the FTM register synchronization method.
 *
 * This function will set the necessary bits for the synchronization mode that user wishes to use.
 *
 * @param ftmBase The FTM base address pointer
 * @param syncMethod  Synchronization method defined by ftm_sync_method_t enum. User can choose
 *                    multiple synch methods by OR'ing options
 */
void FTM_HAL_SetSyncMode(FTM_Type* const ftmBase, uint32_t syncMethod);

/*!
 * @brief Sets the FTM peripheral timer channel input capture filter value.
 * @param ftmBase The FTM base address pointer
 * @param channel  The FTM peripheral channel number, only 0,1,2,3, channel 4, 5,6, 7 don't have.
 * @param val  Filter value to be set
 */
void FTM_HAL_SetChnInputCaptureFilter(FTM_Type* const ftmBase, uint8_t channel, uint8_t value);

bool FTM_HAL_GetDualChnCombine(FTM_Type* const ftmBase, uint8_t chnlPairNum);

#if defined(__cplusplus)
}
#endif

/*! @}*/


#endif /* __FSL_FTM_HAL_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/
