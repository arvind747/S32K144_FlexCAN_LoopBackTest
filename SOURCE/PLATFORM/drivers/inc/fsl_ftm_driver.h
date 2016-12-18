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
#ifndef __FSL_FTM_DRIVER_H__
#define __FSL_FTM_DRIVER_H__


#include <stdint.h>
#include <stdbool.h>
#include "fsl_ftm_hal.h"
/*!
 * @addtogroup ftm_driver FTM Driver
 * @ingroup ftm
 * @brief FlexTimer Peripheral Driver.
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief Max value for PWM duty cycle */
#define FTM_MAX_DUTY_CYCLE      0x8000U
/*! @brief Shift value which converts duty to ticks */
#define FTM_DUTY_TO_TICKS_SHIFT 15U
/*! @brief Table of base addresses for FTM instances. */
extern FTM_Type * const g_ftmBase[FTM_INSTANCE_COUNT];

/*! @brief Table to save FTM IRQ enumeration numbers defined in the CMSIS header file. */
extern const IRQn_Type g_ftmIrqId[FTM_INSTANCE_COUNT];

/*! @brief FTM status */
typedef enum
{
    FTM_STATUS_SUCCESS  = 0U,   /*!< FTM success status.*/
    FTM_STATUS_ERROR    = 1U,   /*!< FTM error status.*/
} ftm_status_t;

/*! @brief FTM status */
typedef enum
{
    FTM_EDGE_DETECT         = 0U,    /*!< FTM edge detect.*/
    FTM_SIGNAL_MEASUREMENT  = 1U,    /*!< FTM signal measurement.*/
    FTM_NO_OPERATION        = 2U,    /*!< FTM no operation.*/
} ftm_input_op_mode_t;

/*! @brief FlexTimer input capture measurement type for dual edge input capture*/
typedef enum
{
    FTM_NO_MEASUREMENT      = 0x00U,    /*!< No measurement */
    FTM_PERIOD_MEASUREMENT  = 0x01U,    /*!< Period measurement */
    FTM_DUTY_MEASUREMENT    = 0x02U     /*!< Duty measurement */
}ftm_signal_measurement_mode_t;

/*! @brief FlexTimer operation mode */
typedef enum
{
    FTM_MODE_INPUT_CAPTURE      = 0x00U,    /*!< Input capture */
    FTM_MODE_OUTPUT_COMPARE     = 0x01U,    /*!< Output compare */
    FTM_MODE_EDGE_ALIGNED_PWM   = 0x02U,    /*!< Edge aligned PWM */
    FTM_MODE_CEN_ALIGNED_PWM    = 0x03U,    /*!< Centre aligned PWM */
    FTM_DUAL_EDGE_CAPTURE       = 0x04U,    /*!< Dual edge capture */
    FTM_MODE_QUADRATURE_DECODER = 0x05U,    /*!< Quadrature decoder */
    FTM_MODE_UP_TIMER           = 0x06U,    /*!< Timer with up counter */
    FTM_MODE_UP_DOWN_TIMER      = 0x07U     /*!< timer with up-down counter */
}ftm_config_mode_t;

/*!
 * @brief FlexTimer Registers sync parameters
 */
typedef struct _ftm_sync_t
{
    /*!< Please don't use software and hardware trigger simultaneously */
    bool softwareSync;                      /*!< True - enable software sync,
                                                 False - disable software sync */
    bool hardwareSync0;                     /*!< True - enable hardware0 sync,
                                                 False - disable hardware0 sync */
    bool hardwareSync1;                     /*!< True - enable hardware1 sync,
                                                 False - disable hardware1 sync */
    bool hardwareSync2;                     /*!< True - enable hardware2 sync,
                                                 False - disable hardware2 sync */
    bool maxLoadingPoint;                   /*!< True - enable max loading point,
                                                 False - disable max loading point */
    bool minLoadingPoint;                   /*!< True - enable min loading point,
                                                 False - disable min loading point */
    ftm_reg_update_t inverterSync;          /*!< Configures INVCTRL sync */
    ftm_reg_update_t outRegSync;            /*!< Configures SWOCTRL sync */
    ftm_reg_update_t maskRegSync;           /*!< Configures OUTMASK sync */
    ftm_reg_update_t initCounterSync;       /*!< Configures CNTIN sync */
    bool autoClearTrigger;                  /*!< Available only for hardware trigger */
    ftm_pwm_sync_mode_t syncPoint;          /*!< Configure synchronization method
                                                (waiting next loading point or immediate) */
} ftm_pwm_sync_t;

/*! @brief Configuration structure that the user needs to set */

typedef struct FtmUserConfig {
    ftm_pwm_sync_t syncMethod;          /*!< Register synch options available in the
                                            ftm_sync_method_t enumeration  */
    ftm_clock_ps_t ftmPrescaler;        /*!< Register prescaler options available in the
                                            ftm_clock_ps_t enumeration  */
    ftm_clock_source_t ftmClockSource;  /*!< Select clock source for FTM */
    ftm_bdm_mode_t BDMMode;             /*!< Select FTM behaviour in BDM mode */
    uint8_t tofFrequency;               /*!< Select ratio between number of overflows to
                                            times TOF is set */
    bool isWriteProtection;             /*!< true: enable write protection,
                                             false: write protection is disabled  */
    bool isTofIsrEnabled;               /*!< true: enable interrupt,
                                             false: write interrupt is disabled */
} ftm_user_config_t;

/*!
 * @brief FlexTimer driver timer mode config structure
 */
typedef struct _ftm_timer_param_t
{
    ftm_config_mode_t mode;         /*!< FTM mode */
    uint16_t initialValue;          /*!< Initial counter value */
    uint16_t finalValue;            /*!< Final counter value */
}ftm_timer_param_t;


/*!
 * @brief FlexTimer driver PWM Fault channel parameters
 */
typedef struct _ftm_pwm_ch_fault_param_t
{
    bool faultChannelEnabled;       /*!< Fault channel state */
    bool faultFilterEnabled;        /*!< Fault channel filter state */
    bool ftmFaultPinPolarity;       /*!< Channel output state on fault */
}ftm_pwm_ch_fault_param_t;

/*!
 * @brief FlexTimer driver PWM Fault parameter
 */
typedef struct _ftm_pwm_fault_param_t
{
    /*!< Output pin state on fault */
    bool pwmOutputStateOnFault;
    /*!< PWM fault interrupt state */
    bool pwmFaultInterrupt;
    /*!< Fault filter value */
    uint8_t faultFilterValue;
    /*!< Fault mode */
    ftm_fault_mode_t faultMode;
     /*!< Fault channels configuration */
    ftm_pwm_ch_fault_param_t ftmFaultChannelParam[FTM_FEATURE_FAULT_CHANNELS];
}ftm_pwm_fault_param_t;



/*!
 * @brief FlexTimer driver independent PWM parameter
 */
typedef struct _ftm_independent_ch_param_t
{
    uint8_t                 hwChannelId;            /*!< Physical hw channel ID*/
    ftm_polarity_t          polarity;               /*!< PWM output polarity */
    uint32_t                uDutyCyclePercent;      /*!< PWM pulse width, value should be between
                                                         0 (0%) to FTM_MAX_DUTY_CYCLE (100%). */
}ftm_independent_ch_param_t;


/*!
 * @brief FlexTimer driver combined PWM parameter
 */

typedef struct _ftm_pwm_combined_ch_param_t
{   /*!< Physical hw channel ID for channel (n) */
    uint8_t                         hwChannelId;
    /*!< First edge time. This time is relative to signal period. The value for this parameter is
    between 0 and FTM_MAX_DUTY_CYCLE( 0 = 0% from period and FTM_MAX_DUTY_CYCLE = 100% from period) */
    uint32_t                        firstEdge;
    /*!< Second edge time. This time is relative to signal period. The value for this parameter is
    between 0 and FTM_MAX_DUTY_CYCLE( 0 = 0% from period and FTM_MAX_DUTY_CYCLE = 100% from period) */
    uint32_t                        secondEdge;
    /*!< Enable/disable dead time for channel */
    bool                            deadTime;
    /*!< Main channel polarity. For FTM_POLARITY_HIGH first output value is 0 and for
    FTM_POLAIRTY first output value is 1. */
    ftm_polarity_t                  mainChannelPolarity;
    /*!< Select if channel (n+1)  output is enabled/disabled */
    bool                            enableSecondChannelOutput;
    /*!< Select channel (n+1) polarity relative to channel (n) */
    ftm_second_channel_polarity_t   secondChannelPolarity;
}ftm_combined_ch_param_t;
/*!
 * @brief FlexTimer driver PWM parameters
 */
typedef struct _ftm_pwm_param_t
{
    /*!< Number of independent PWM channels */
    uint8_t                      nNumIndependentPwmChannels;
    /*!< Number of combined PWM channels */
    uint8_t                      nNumCombinedPwmChannels;
    /*!< FTM mode */
    ftm_config_mode_t            Mode;
    /*!< Dead time value in [ticks] */
    uint8_t                      deadTimeValue;
    /*!< Dead time prescaler value[ticks] */
    ftm_deadtime_ps_t            deadTimePrescaler;
    /*!< PWM period in Hz */
    uint32_t                     uFrequencyHZ;
    /*!< Configuration for independent PWM channels */
    ftm_independent_ch_param_t   (*pwmIndependentChannelConfig)[];
    /*!< Configuration for combined PWM channels */
    ftm_combined_ch_param_t      (*pwmCombinedChannelConfig)[];
    /*!< Configuration for PWM fault */
    ftm_pwm_fault_param_t        * faultConfig;
}ftm_pwm_param_t;

/*! @brief FlexTimer input capture edge mode, rising edge, or falling edge */
typedef enum
{
    FTM_NO_PIN_CONTROL = 0x00U,     /*!< No trigger */
    FTM_RISING_EDGE = 0x01U,        /*!< Rising edge trigger */
    FTM_FALLING_EDGE = 0x02U,       /*!< Falling edge trigger */
    FTM_BOTH_EDGES = 0x03U,         /*!< Rising and falling edge trigger */
}ftm_edge_alignment_mode_t;

/*!
 * @brief FlexTimer driver Input capture parameters
 */
typedef struct _ftm_input_ch_param_t
{
    uint8_t                         hwChannelId;      /*!< Physical hw channel ID*/
    ftm_input_op_mode_t             InputMode;        /*!< FlexTimer module mode of operation  */
    ftm_edge_alignment_mode_t       EdgeAlignement;   /*!< Edge alignment Mode for signal measurement*/
    ftm_signal_measurement_mode_t   MeasurementType;  /*!< Measurement Mode for signal measurement*/
    uint16_t                        FilterValue;      /*!< Filter Value */
    bool                            FilterEn;         /*!< Input capture filter state */
    bool                            ContinuousModeEn; /*!< Continuous measurement state */
}ftm_input_ch_param_t;

/*!
 * @brief FlexTimer driver input capture parameters
 */
typedef struct _ftm_input_param_t
{
    uint8_t                 nNumChannels;       /*!< Number of input capture channel used */
    uint16_t                nMaxCountValue;     /*!< Max counter value. Min value is 0 for this mode */
    ftm_input_ch_param_t    (*inputChConfig)[]; /*!< Input capture channels configuration */
}ftm_input_param_t;


/*!
 * @brief FlexTimer driver PWM parameters
 */
typedef struct _ftm_output_cmp_ch_param_t
{
    uint8_t                   hwChannelId;      /*!< Physical hw channel ID*/
    ftm_output_compare_mode_t ChMode;           /*!< Channel output mode*/
    uint32_t                  comparedValue;    /*!< The compared value */
}ftm_output_cmp_ch_param_t;


/*!
 * @brief FlexTimer driver PWM parameters
 */
typedef struct _ftm_output_cmp_param_t
{
    uint8_t              nNumOutputChannels;            /*!< Number of output compare channels */
    ftm_config_mode_t    mode;                          /*!< FlexTimer PWM operation mode */
    uint32_t             maxCountValue;                 /*!< Maximum count value in ticks */
    ftm_output_cmp_ch_param_t (*outputChannelConfig)[]; /*!< Output compare channels config */
}ftm_output_cmp_param_t;

/*!
 * @brief FlexTimer qudrature decoder channel parameters
 */
typedef struct FtmPhaseParam
{
    bool kFtmPhaseInputFilter;                      /*!< True: disable phase filter,
                                                         False: enable phase filter */
    uint32_t kFtmPhaseFilterVal;                    /*!< Filter value (if input filter is enabled )*/
    ftm_quad_phase_polarity_t kFtmPhasePolarity;    /*!< Phase polarity */
}ftm_phase_params_t;


/*! @brief FTM quadrature config structure */
typedef struct ftm_quad_decode_config_t
{
    ftm_quad_decode_mode_t mode;        /*!< FTM_QUAD_PHASE_ENCODE or FTM_QUAD_COUNT_AND_DIR */
    uint16_t initial_val;               /*!< Initial counter value*/
    uint16_t max_val;                   /*!< Max counter value*/
    ftm_phase_params_t phase_a_config;  /*!< Configuration for the input phase a*/
    ftm_phase_params_t phase_b_config;  /*!< Configuration for the input phase b*/
} ftm_quad_decode_config_t;

/*! @brief FTM quadrature state(counter value and flags) */
typedef struct ftm_quad_decoder_state_t
{
    uint16_t counter;           /*!< Counter value */
    bool overflow_flag;         /*!< True if overflow occurred,
                                     False if overflow doesn't occurred*/
    bool overflow_direction;    /*!< False if overflow occurred at minimum value,
                                     True if overflow occurred at maximum value*/
} ftm_quad_decoder_state_t;


/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initializes the FTM driver.
 *
 * @param instance The FTM peripheral instance number.
 * @param info The FTM user configuration structure, see #ftm_user_config_t.
 * @return success
 *        - FTM_STATUS_SUCCESS : Completed successfully.
 *        - FTM_STATUS_ERROR : Error occurred.
 */
ftm_status_t FTM_DRV_Init(uint8_t instance, const ftm_user_config_t *info);

/*!
 * @brief Shuts down the FTM driver.
 *
 * @param instance The FTM peripheral instance number.
 * @return success
 *        - FTM_STATUS_SUCCESS : Completed successfully.
 *        - FTM_STATUS_ERROR : Error occurred.
 */
ftm_status_t FTM_DRV_Deinit(uint8_t instance);

/*!
 * @brief Starts the FTM counter.
 *
 * Starts the FTM counter. This function provides access to the
 * FTM counter settings. The counter can be run in Up counting and Up-down counting modes.
 * To run the counter in Free running mode, choose Up counting option and provide
 * 0x0 for the countStartVal and 0xFFFF for countFinalVal. Please call this
 * function only when FTM is used as timer/counter.
 *
 * @param instance The FTM peripheral instance number.
 * @param timer Timer configuration structure.
 * @return success
 *        - FTM_STATUS_SUCCESS : Completed successfully.
 *        - FTM_STATUS_ERROR : Error occurred.
 */
ftm_status_t FTM_DRV_CounterStart(uint8_t instance, ftm_timer_param_t timer);
/*!
 * @brief Stops the FTM counter.
 *
 * @param instance The FTM peripheral instance number.
 * @return success
 *        - FTM_STATUS_SUCCESS : Completed successfully.
 *        - FTM_STATUS_ERROR : Error occurred.
 */
ftm_status_t FTM_DRV_CounterStop(uint8_t instance);

/*!
 * @brief Reads back the current value of the FTM counter.
 *
 * @param instance The FTM peripheral instance number.
 * @return success
 *        - FTM_STATUS_SUCCESS : Completed successfully.
 *        - FTM_STATUS_ERROR : Error occurred.
 */
uint32_t FTM_DRV_CounterRead(uint8_t instance);

/*!
 * @brief Stops all PWM channels .
 *
 * @param instance The FTM peripheral instance number.
 * @return counter the current counter value
 */
ftm_status_t FTM_DRV_DeinitPwm(uint8_t instance);

/*!
 * @brief Configures the duty cycle and frequency and starts outputting the PWM on
 * all channels configured in param.
 *
 * @param instance The FTM peripheral instance number.
 * @param param FTM driver PWM parameter to configure PWM options
 * @return success
 *        - FTM_STATUS_SUCCESS : Completed successfully.
 *        - FTM_STATUS_ERROR : Error occurred.
 */
ftm_status_t FTM_DRV_InitPwm(uint8_t instance, const ftm_pwm_param_t *  param);

/*!
 * @brief This function updates the waveform output in PWM mode (duty cycle and phase).
 *
 * @param instance The FTM peripheral instance number.
 * @param channel The channel number. In combined mode, the code  finds the channel
 * @param firstEdge  Duty cycle or first edge time for combined mode. Can take value between
 *                   0 - FTM_MAX_DUTY_CYCLE(0 = 0% from period  and FTM_MAX_DUTY_CYCLE = 100% from period)
 * @param secondEdge Second edge time - only for combined mode. Can take value
 *                   between 0 - FTM_MAX_DUTY_CYCLE(0 = 0% from period  and FTM_MAX_DUTY_CYCLE = 100% from period).
 * @param softwareTrigger - if true a software trigger is generate to update PWM parameters
 * @return success
 *        - FTM_STATUS_SUCCESS : Completed successfully.
 *        - FTM_STATUS_ERROR : Error occurred.
 */
ftm_status_t FTM_DRV_UpdatePwmChannel(uint8_t instance, uint8_t channel, uint16_t firstEdge,
                                        uint16_t secondEdge, bool softwareTrigger);
/*!
 * @brief This function configures sync mechanism for some FTM registers ( MOD, CNINT, HCR,
 *          CnV, OUTMASK, INVCTRL, SWOCTRL).
 *
 * @param instance The FTM peripheral instance number.
 * @param param The sync configuration structure.
 * @return success
 *        - FTM_STATUS_SUCCESS : Completed successfully.
 *        - FTM_STATUS_ERROR : Error occurred.
 */
ftm_status_t FTM_DRV_SetSync(uint8_t instance, ftm_pwm_sync_t param);

/*!
 * @brief Configures the FTM to generate timed pulses(Output compare mode).
 *
 * When the FTM counter matches the value of CnV, the channel output is changed based on what is
 * specified in the compareMode argument. The signal period can be modified using
 * param->MaxCountValue. After this function max counter value and CnV are equal.
 * FTM_DRV_SetNextComparematchValue function ca be used to change CnV value.
 *
 * @param instance    The FTM peripheral instance number.
 * @param param configuration of the output compare channels
 * @return success
 *        - FTM_STATUS_SUCCESS : Completed successfully.
 *        - FTM_STATUS_ERROR : Error occurred.
 */
ftm_status_t FTM_DRV_InitOutputCompare(uint8_t instance, const ftm_output_cmp_param_t *param);

/*!
 * @brief  Disables compare match output control and clears FTM timer configuration
 *
 * @param instance    The FTM peripheral instance number.
 * @param param configuration of the output compare channel
 * @return success
 *        - FTM_STATUS_SUCCESS : Completed successfully.
 *        - FTM_STATUS_ERROR : Error occurred.
 */
ftm_status_t FTM_DRV_DeinitOutputCompare(uint8_t instance, const ftm_output_cmp_param_t *param);

/*!
 * @brief Sets the next compare match value based on the current counter value
 *
 * @param instance    The FTM peripheral instance number.
 * @param channel configuration of the output compare channel
 * @param NextComparematchValue timer value in ticks until the next compare match event should appear
 * @return success
 *        - FTM_STATUS_SUCCESS : Completed successfully.
 *        - FTM_STATUS_ERROR : Error occurred.
 */
ftm_status_t FTM_DRV_UpdateOutputCompareChannel(uint8_t instance, uint8_t channel,
                                                uint16_t NextComparematchValue, bool softwareTrigger);

/*!
 * @brief   Configures Channel Input Capture for either getting time-stamps on edge detection
 * or on signal measurement . When the edge specified in the captureMode
 * argument occurs on the channel the FTM counter is captured into the CnV register.
 * The user will have to read the CnV register separately to get this value. The filter
 * function is disabled if the filterVal argument passed in is 0. The filter function
 * is available only on channels 0,1,2,3.
 *
 * @param instance    The FTM peripheral instance number
 * @param param configuration of the input capture channel
 * @return success
 *        - FTM_STATUS_SUCCESS : Completed successfully.
 *        - FTM_STATUS_ERROR : Error occurred.
 */
ftm_status_t FTM_DRV_InitInputCapture(uint8_t instance, const ftm_input_param_t * param);
/*!
 * @brief  Disables input capture mode and clears FTM timer configuration
 *
 * @param instance    The FTM peripheral instance number.
 * @param param configuration of the output compare channel
 * @return success
 *        - FTM_STATUS_SUCCESS : Completed successfully.
 *        - FTM_STATUS_ERROR : Error occurred.
 */
ftm_status_t FTM_DRV_DeinitInputCapture(uint8_t instance, const ftm_input_param_t * param);
/*!
 * @brief  This function is used to calculate the measurement and/or time stamps values
 * which are read from the C(n, n+1)V registers and stored to the static buffers.
 *
 * @param instance    The FTM peripheral instance number
 * @param channel     FTM Hw channel or pair Id
 * @return value   the value measured
 */
uint16_t FTM_DRV_GetInputCaptureMeasurement(uint8_t instance, uint8_t channel);

/*!
 * @brief  Starts new single-shot signal measurement of the given channel.
 *
 * @param instance    The FTM peripheral instance number.
 * @param channel configuration of the output compare channel
 * @return success
 *        - FTM_STATUS_SUCCESS : Completed successfully.
 *        - FTM_STATUS_ERROR : Error occurred.
 */
ftm_status_t FTM_DRV_StartNewSignalMeasurement(uint8_t instance, uint8_t channel);
/*!
 * @brief Configures the quadrature mode and starts measurement
 *
 * @param instance     Instance number of the FTM module.
 * @param config       Config structure( quadrature decode mode, polarity for both phases,
 *                      initial and max value for the counter, filter config)
 * @return success
 *        - FTM_STATUS_SUCCESS : Completed successfully.
 *        - FTM_STATUS_ERROR : Error occurred.
 */
ftm_status_t FTM_DRV_QuadDecodeStart(uint8_t instance, ftm_quad_decode_config_t config);

/*!
 * @brief De-activates the quadrature decode mode.
 *
 * @param instance  Instance number of the FTM module.
 * @return success
 *        - FTM_STATUS_SUCCESS : Completed successfully.
 *        - FTM_STATUS_ERROR : Error occurred.
 */
ftm_status_t FTM_DRV_QuadDecodeStop(uint8_t instance);

/*!
 * @brief Return the current quadrature decoder state (counter value, overflow flag and
 * overflow direction)
 *
 * @param instance  Instance number of the FTM module.
 * @return The current state of quadrature decoder
 */
ftm_quad_decoder_state_t FTM_DRV_QuadGetState(uint8_t instance);

/*!
 * @brief Retrieves the frequency of the clock source feeding the FTM counter.
 *
 * Function will return a 0 if no clock source is selected and the FTM counter is disabled
 *
 * @param instance The FTM peripheral instance number.
 * @return The frequency of the clock source running the FTM counter (0 if counter is disabled)
 */
uint32_t FTM_DRV_GetFrequency(uint8_t instance);


/*!
 * @brief This function is used to covert the given frequency to period in ticks
 *
 * @param instance The FTM peripheral instance number
 * @param freqencyHz  frequency value in Hz
 *
 * @return uint16_t
 */
uint16_t FTM_DRV_ConvertFreqToPeriodTicks(uint8_t instance,  uint32_t freqencyHz);

#if defined(__cplusplus)
}
#endif

/*! @}*/


#endif /* __FSL_FTM_DRIVER_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/

/*! @}*/ /* End of addtogroup ftm_driver */
