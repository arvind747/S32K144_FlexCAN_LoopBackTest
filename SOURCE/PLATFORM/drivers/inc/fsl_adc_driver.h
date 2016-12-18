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

#ifndef __FSL_ADC_DRIVER_H__
#define __FSL_ADC_DRIVER_H__

#include <stdint.h>
#include <stdbool.h>
#include "fsl_device_registers.h"
#include "fsl_adc_hal.h"

/*! @file */

/*!
 * @addtogroup adc_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*!
 * @brief Defines the converter configuration
 *
 * This structure is used to configure the ADC converter
 */
typedef struct
{
    adc_clk_divide_t clockDivide;       /*!< Divider of the input clock for the ADC */
    uint8_t sampleTime;                 /*!< Sample time in AD Clocks */
    adc_resolution_t resolution;        /*!< ADC resolution (8,10,12 bit) */
    adc_input_clock_t inputClock;       /*!< Input clock source */
    adc_trigger_t trigger;              /*!< ADC trigger type (software, hardware) */
    bool dmaEnable;                     /*!< Enable DMA for the ADC */
    adc_voltage_reference_t voltageRef; /*!< Voltage reference used */
    bool continuousConvEnable;          /*!< Enable Continuous conversions */
} adc_converter_config_t;

/*!
 * @brief Defines the hardware compare configuration
 *
 * This structure is used to configure the hardware compare
 * feature for the ADC
 */
typedef struct
{
    bool compareEnable;              /*!< Enable the compare feature */
    bool compareGreaterThanEnable;   /*!< Enable Greater-Than functionality */
    bool compareRangeFuncEnable;     /*!< Enable Range functionality */
    uint16_t compVal1;               /*!< First Compare Value */
    uint16_t compVal2;               /*!< Second Compare Value */
} adc_compare_config_t;

/*!
 * @brief Defines the hardware average configuration
 *
 * This structure is used to configure the hardware average
 * feature for the ADC
 */
typedef struct
{
    bool hwAvgEnable;        /*!< Enable averaging functionality */
    adc_average_t hwAverage; /*!< Selection for number of samples used for averaging */
} adc_average_config_t;

/*!
 * @brief Defines the measurement channel configuration
 *
 * This structure is used to configure a measurement channel
 * of the ADC
 */
typedef struct
{
    bool interruptEnable;       /*!< Enable interrupts for this channel */
    adc_inputchannel_t channel; /*!< Selection of input channel for measurement */
} adc_chan_config_t;

/*!
 * @brief Defines the user calibration configuration
 *
 * This structure is used to configure the user calibration
 * parameters of the ADC.
 */
typedef struct
{
    uint16_t userGain;    /*!< User-configurable gain */
    uint16_t userOffset;  /*!< User-configurable Offset (2's complement, subtracted from result) */
} adc_calibration_t;

 /*! @brief ADC DRV API return status*/
typedef enum
{
    ADC_DRV_SUCCESS,  /*!< Success.      */
    ADC_DRV_FAIL,     /*!< Error occurs. */
} adc_drv_status_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Converter
 */
/*! @{*/

/*!
 * @brief Initializes the converter configuration structure
 *
 * This function initializes the members of the adc_converter_config_t
 * structure to default values (Reference Manual resets). This function should
 * be called on a structure before using it to configure the converter with
 * ADC_DRV_ConfigConverter(), otherwise all members must be written
 * (initialized) by the user. This function insures that all members are written
 * with safe values, so the user can modify only the desired members.
 *
 * @param[out] config the configuration structure
 */
void ADC_DRV_InitConverterStruct(adc_converter_config_t* const config);

/*!
 * @brief Configures the converter with the given configuration structure
 *
 * This function configures the ADC converter with the options
 * provided in the provided structure.
 *
 * @param[in] instance instance number
 * @param[in] config the configuration structure
 * @return success
 *        - ADC_DRV_SUCCESS : Completed successfully.
 *        - ADC_DRV_FAIL : Error occurred.
 */
adc_drv_status_t ADC_DRV_ConfigConverter(const uint32_t instance, const adc_converter_config_t* const config);

/*!
 * @brief Gets the current converter configuration
 *
 * This functions returns the configuration for converter in
 * the form of a configuration structure.
 *
 * @param[in] instance instance number
 * @param[out] config the configuration structure
 * @return success
 *        - ADC_DRV_SUCCESS : Completed successfully.
 *        - ADC_DRV_FAIL : Error occurred.
 */
adc_drv_status_t ADC_DRV_GetConverterConfig(const uint32_t instance, adc_converter_config_t* const config);

/*!
 * @brief Resets the converter (sets all configurations to reset values)
 *
 * This function resets all the internal ADC registers to
 * reset values.
 *
 * @param[in] instance instance number
 * @return success
 *        - ADC_DRV_SUCCESS : Completed successfully.
 *        - ADC_DRV_FAIL : Error occurred.
 */
adc_drv_status_t ADC_DRV_Reset(const uint32_t instance);

/*! @}*/

/*!
 * @name Hardware Compare
 * The Hardware Compare feature of the S32K144 ADC is a versatile mechanism that
 * can be used to monitor that a value is within certain values. Measurements can
 * be monitored to be within certain ranges:
 *  - less than/ greater than a fixed value
 *  - inside or outside of a certain range
 *
 * Two compare values can be configured (the second value is used only for range
 * function mode). The compare values must be written in 12-bit resolution mode
 * regardless of the actual used resolution mode.
 *
 * Once the hardware compare feature is enabled, a conversion is considered
 * complete only when the measured value is within the allowable range set by
 * the configuration.
 */
/*! @{*/

/*!
 * @brief Initializes the Hardware Compare configuration structure
 *
 * This function initializes the Hardware Compare configuration
 * structure to default values (Reference Manual resets). This function should be
 * called before configuring the Hardware Compare feature (ADC_DRV_ConfigHwCompare),
 * otherwise all members must be written by the caller. This function insures
 * that all members are written with safe values, so the user can modify the
 * desired members.
 *
 * @param[out] config the configuration structure
 */
void ADC_DRV_InitHwCompareStruct(adc_compare_config_t* const config);

/*!
 * @brief Configures the Hardware Compare feature with the given configuration
 * structure
 *
 * This functions sets the configuration for the Hardware
 * Compare feature using the configuration structure.
 *
 * @param[in] instance instance number
 * @param[in] config the configuration structure
 * @return success
 *        - ADC_DRV_SUCCESS : Completed successfully.
 *        - ADC_DRV_FAIL : Error occurred.
 */
adc_drv_status_t ADC_DRV_ConfigHwCompare(const uint32_t instance, const adc_compare_config_t* const config);

/*!
 * @brief Gets the current Hardware Compare configuration
 *
 * This function returns the configuration for the Hardware
 * Compare feature.
 *
 * @param[in] instance instance number
 * @param[out] config the configuration structure
 * @return success
 *        - ADC_DRV_SUCCESS : Completed successfully.
 *        - ADC_DRV_FAIL : Error occurred.
 */
adc_drv_status_t ADC_DRV_GetHwCompareConfig(const uint32_t instance, adc_compare_config_t* const config);

/*! @}*/
/*!
 * @name Hardware Average
 * The Hardware Average feature of the S32K144 allows for a set of measurements
 * to be averaged together as a single conversion. The number of samples to be
 * averaged is selectable (4, 8, 16 or 32 samples).
 */
/*! @{*/

/*!
 * @brief Initializes the Hardware Average configuration structure
 *
 * This function initializes the Hardware Average configuration
 * structure to default values (Reference Manual resets). This function should be
 * called before configuring the Hardware Average feature (ADC_DRV_ConfigHwAverage),
 * otherwise all members must be written by the caller. This function insures
 * that all members are written with safe values, so the user can modify the
 * desired members.
 *
 * @param[out] config the configuration structure
 */
void ADC_DRV_InitHwAverageStruct(adc_average_config_t* const config);

/*!
 * @brief Configures the Hardware Average feature with the given configuration
 * structure
 *
 * This function sets the configuration for the Hardware
 * Average feature.
 *
 * @param[in] instance instance number
 * @param[in] config the configuration structure
 * @return success
 *        - ADC_DRV_SUCCESS : Completed successfully.
 *        - ADC_DRV_FAIL : Error occurred.
 */
adc_drv_status_t ADC_DRV_ConfigHwAverage(const uint32_t instance, const adc_average_config_t* const config);

/*!
 * @brief Gets the current Hardware Average configuration
 *
 * This function returns the configuration for the Hardware
 * Average feature.
 *
 * @param[in] instance instance number
 * @param[out] config the configuration structure
 * @return success
 *        - ADC_DRV_SUCCESS : Completed successfully.
 *        - ADC_DRV_FAIL : Error occurred.
 */
adc_drv_status_t ADC_DRV_GetHwAverageConfig(const uint32_t instance, adc_average_config_t* const config);

/*! @}*/

/*!
 * @name Channel configuration
 */
/*! @{*/

/*!
 * @brief Initializes the measurement channel configuration structure
 *
 * This function initializes the measurement channel
 * configuration structure to default values (Reference Manual resets). This
 * function should be called on a structure before using it to configure a
 * channel (ADC_DRV_ConfigChan), otherwise all members must be written by the
 * caller. This function insures that all members are written with safe values,
 * so the user can modify only the desired members.
 *
 * @param[out] config the configuration structure
 */
void ADC_DRV_InitChanStruct(adc_chan_config_t* const config);

/*!
 * @brief Configures the selected measurement channel with the given
 * configuration structure
 *
 * This function sets a measurement channel configuration.
 *
 * @param[in] instance instance number
 * @param[in] chanIndex the converter channel index
 * @param[in] config the configuration structure
 * @return success
 *        - ADC_DRV_SUCCESS : Completed successfully.
 *        - ADC_DRV_FAIL : Error occurred.
 */
adc_drv_status_t ADC_DRV_ConfigChan(const uint32_t instance, const uint8_t chanIndex, const adc_chan_config_t* const config);

/*!
 * @brief Gets the current measurement channel configuration for the selected
 * channel index
 *
 * This function returns the configuration for a measurement
 * channel.
 *
 * @param[in] instance instance number
 * @param[in] chanIndex the channel index
 * @param[out] config the configuration structure
 * @return success
 *        - ADC_DRV_SUCCESS : Completed successfully.
 *        - ADC_DRV_FAIL : Error occurred.
 */
adc_drv_status_t ADC_DRV_GetChanConfig(const uint32_t instance, const uint8_t chanIndex, adc_chan_config_t* const config);

/*! @}*/
/*!
 * @name Converter
 * Converter specific methods. These are used to configure and use the A/D
 * Converter specific functionality, including:
 *  - clock input and divider
 *  - sample time in A/D clocks
 *  - resolution
 *  - trigger source
 *  - voltage reference
 *  - enable DMA
 *  - enable continuous conversion on one channel
 *
 * To start a conversion, a result channel (see \ref chan_config "Channel
 * Configuration") and a
 * trigger source must be configured. Once a conversion is started, the user
 * application can wait for it to be finished by calling the
 * ADC_DRV_WaitConvDone() function.
 *
 * Only the first result channel can be triggered by software. To start a
 * conversion in this case, an input channel must be written in the channel
 * selection register using the ADC_DRV_ConfigChan() method. Writing a value to
 * the result register while a conversion is being performed on that channel
 * will start a new conversion.
 *
 */
/*! @{*/

/*!
 * @brief Waits for a conversion/calibration to finish
 *
 * This functions waits for a conversion to complete by
 * continuously polling the Conversion Active Flag.
 *
 * @param[in] instance instance number
 * @return success
 *        - ADC_DRV_SUCCESS : Completed successfully.
 *        - ADC_DRV_FAIL : Error occurred.
 */
adc_drv_status_t ADC_DRV_WaitConvDone(const uint32_t instance);


/*! @}*/

/*!
 * @anchor chan_config
 * @name Channel configuration
 * Result channel specific functions. These functions control configurations
 * for each result channel (input channel selection and interrupt enable).
 *
 * When software triggering is enabled, calling the ADC_DRV_ConfigChan() method
 * for result channel 0 starts a new conversion.
 *
 * After a conversion is finished, the result can be retrieved using the
 * ADC_DRV_GetChanResult() method.
 */
/*! @{*/

/*!
 * @brief Gets the last result for the selected measurement channel
 *
 * This function returns the conversion result from a
 * measurement channel.
 *
 * @param[in] instance instance number
 * @param[in] chanIndex the converter channel index
 * @param[out] result the result in raw value
 * @return success
 *        - ADC_DRV_SUCCESS : Completed successfully.
 *        - ADC_DRV_FAIL : Error occurred.
 */
adc_drv_status_t ADC_DRV_GetChanResult(const uint32_t instance, const uint8_t chanIndex, uint16_t* const result);

/*! @}*/

/*!
 * @name Automatic Calibration
 * These methods control the Calibration feature of the ADC.
 *
 * The ADC_DRV_AutoCalibration() method can be called to execute a calibration
 * sequence, or a calibration can be retrieved with the ADC_DRV_GetUserCalibration()
 * and saved to non-volatile storage, to avoid calibration on every power-on.
 * The calibration structure can be written with the ADC_DRV_ConfigUserCalibration()
 * method.
 */
/*! @{*/

/*!
 * @brief Executes an Auto-Calibration
 *
 * This functions executes an Auto-Calibration sequence. It
 * is recommended to run this sequence before using the ADC
 * converter.
 *
 * @param[in] instance instance number
 * @return success
 *        - ADC_DRV_SUCCESS : Completed successfully.
 *        - ADC_DRV_FAIL : Error occurred.
 */
adc_drv_status_t  ADC_DRV_AutoCalibration(const uint32_t instance);

/*!
 * @brief Initializes the User Calibration configuration structure
 *
 * This function initializes the User Calibration configuration
 * structure to default values (Reference Manual resets). This function should
 * be called on a structure before using it to configure the User Calibration
 * feature (ADC_DRV_ConfigUserCalibration), otherwise all members must be
 * written by the caller. This function insures that all members are written
 * with safe values, so the user can modify only the desired members.
 *
 * @param[out] config the configuration structure
 */
void ADC_DRV_InitUserCalibrationStruct(adc_calibration_t* const config);

/*!
 * @brief Configures the User Calibration feature with the given configuration
 * structure
 *
 * This function sets the configuration for the user calibration
 * registers.
 *
 * @param[in] instance instance number
 * @param[in] config the configuration structure
 * @return success
 *        - ADC_DRV_SUCCESS : Completed successfully.
 *        - ADC_DRV_FAIL : Error occurred.
 */
adc_drv_status_t ADC_DRV_ConfigUserCalibration(const uint32_t instance, const adc_calibration_t* const config);

/*!
 * @brief Gets the current User Calibration configuration
 *
 * This function returns the current user calibration
 * register values.
 *
 * @param[in] instance instance number
 * @param[out] config the configuration structure
 * @return success
 *        - ADC_DRV_SUCCESS : Completed successfully.
 *        - ADC_DRV_FAIL : Error occurred.
 */
adc_drv_status_t ADC_DRV_GetUserCalibration(const uint32_t instance, adc_calibration_t* const config);

/*! @}*/

/*!
 * @name Interrupts
 * This method installs a user-defined ISR handler for the ADC instance (if the
 * interrupt vector table is located in RAM).
 * It is an alternative to defining the ADCx_Handler(void) functions.
 */
/*! @{*/

/*!
 * @brief Installs a function to be executed from the ADC interrupt service
 * routine
 *
 * This function sets an user-defined function to be called from
 * the ADC Interrupt Service Routine.
 *
 * @param[in] instance instance number
 * @param[in] isr the ISR handler
 * @return success
 *        - ADC_DRV_SUCCESS : Completed successfully.
 *        - ADC_DRV_FAIL : Error occurred.
 */
adc_drv_status_t ADC_DRV_InstallHandlerIsr(const uint32_t instance, void (*isr)(void));

/*! @}*/
#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __FSL_ADC_DRIVER_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/
