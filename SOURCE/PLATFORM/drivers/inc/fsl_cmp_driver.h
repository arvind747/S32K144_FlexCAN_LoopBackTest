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

#ifndef __FSL_CMP_DRIVER_H__
#define __FSL_CMP_DRIVER_H__


#include <stdint.h>
#include <stdbool.h>
#include "fsl_device_registers.h"
#include "fsl_cmp_hal.h"

/*!
 * @addtogroup cmp_driver Comparator Driver
 * @ingroup cmp
 * @brief Comparator Peripheral Driver
 * @{
 *******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief Defines the block configuration
 *
 * This structure is used to configure only comparator block module(filtering, sampling, power_mode etc.)
 */
typedef struct cmp_comparator_t
{
    bool                    dmaTriggerState;        /*!< True if DMA transfer trigger from comparator is enable.*/
    cmp_output_trigger_t    outputInterruptTrigger; /*!< CMP_NO_INTERRUPT comparator output would not trigger any interrupt.*/
                                                    /*!< CMP_FALLING_EDGE comparator output would trigger an interrupt on falling edge.*/
                                                    /*!< CMP_RISING_EDGE comparator output would trigger an interrupt on rising edge.*/
                                                    /*!< CMP_BOTH_EDGES comparator output would trigger an interrupt on rising and falling edges.*/
    cmp_mode_t              mode;                   /*!< Configuration structure which define: the comparator functional mode, sample period and sample count.*/ 
    uint8_t                 filterSamplePeriod;     /*!< Filter sample period. */
    uint8_t                 filterSampleCount;      /*!< Number of sample count for filtering. */
    cmp_power_mode_t        powerMode;              /*!< CMP_LOW_SPEED if low speed mode is selected. */
                                                    /*!< CMP_HIGH_SPEED if high speed mode is selected*/
    cmp_inverter_t          inverterState;          /*!< CMP_NORMAL if does not invert the comparator output.*/
                                                    /*!< CMP_INVERT if inverts the comparator output.*/
    cmp_output_enable_t     pinState;               /*!< CMP_UNAVAILABLE if comparator output is not available to package pin.*/
                                                    /*!< CMP_AVAILABLE if comparator output is available to package pin.*/
    cmp_output_select_t     outputSelect;           /*!< CMP_COUT if output signal is equal to COUT(filtered).*/     
                                                    /*!< CMP_COUTA if output signal is equal to COUTA(unfiltered).*/ 
    cmp_offset_t            offsetLevel;            /*!< CMP_LEVEL_OFFSET_0 if hard block output has level 0 offset.*/
                                                    /*!< CMP_LEVEL_OFFSET_1 if hard block output has level 1 offset.*/
    cmp_hysteresis_t        hysteresisLevel;        /*!< CMP_LEVEL_HYS_0 if hard block output has level 0 hysteresis.*/
                                                    /*!< CMP_LEVEL_HYS_1 if hard block output has level 1 hysteresis.*/
                                                    /*!< CMP_LEVEL_HYS_2 if hard block output has level 2 hysteresis.*/     
                                                    /*!< CMP_LEVEL_HYS_3 if hard block output has level 3 hysteresis.*/
}cmp_comparator_t;

/*!
 * @brief Defines the analog mux
 *
 * This structure is used to configure the analog multiplexor to select compared signals 
 */
typedef struct cmp_anmux_t
{
    cmp_port_mux_t      positivePortMux;    /*!< Select positive port signal.*/
                                            /*!< CMP_DAC if source is digital to analog converter.*/
                                            /*!< CMP_MUX if source is 8 ch MUX/*/
    cmp_port_mux_t      negativePortMux;    /*!< Select negative port signal.*/
                                            /*!< CMP_DAC if source is digital to analog converter.*/
                                            /*!< CMP_MUX if source is 8 ch MUX/*/
    cmp_ch_number_t     positiveInputMux;   /*!< Select which channel is selected for the plus mux.*/
    cmp_ch_number_t     negativeInputMux;   /*!< Select which channel is selected for the minus mux.*/
}cmp_anmux_t;

/*!
 * @brief Defines the DAC block
 *
 * This structure is used to configure the DAC block integrated in comparator module
 */
typedef struct cmp_dac_t
{
    cmp_voltage_reference_t     voltageReferenceSource;     /*!< CMP_VIN1 if selected voltage reference is VIN1.*/  
                                                            /*!< CMP_VIN2 if selected voltage reference is VIN2.*/
    uint8_t                     voltage;                    /*!< The digital value which is converted to analog signal.*/
    bool                        state;                      /*!< True if DAC is enabled.*/
}cmp_dac_t;

/*!
 * @brief Defines the trigger mode
 *
 * This structure is used to configure the trigger mode operation when MCU enters STOP modes
 */
typedef struct cmp_trigger_mode_t
{
    bool                roundRobinState;            /*!< True if Round-Robin is enabled.*/
    bool                roundRobinInterruptState;   /*!< True if Round-Robin interrupt is enabled.*/
    cmp_fixed_port_t    fixedPort;                  /*!< CMP_PLUS_FIXED if plus port is fixed.*/
                                                    /*!< CMP_MINUS_FIXED if minus port is fixed.*/
    cmp_ch_number_t     fixedChannel;               /*!< Select which channel would be assigned to the fixed port.*/
    uint8_t             samples;                    /*!< Select number of round-robin clock cycles for a given channel.*/
    uint8_t             initializationDelay;        /*!< Select dac and comparator initialization delay(clock cycles).*/
    cmp_ch_list_t       roundRobinChannelsState;    /*!< One bite for each channel state.*/
                                                    /*!< |---------|---------|-----|---------|---------|*/
                                                    /*!< |CH7_state|CH6_state|.....|CH1_state|CH0_state|*/
                                                    /*!< |---------|---------|-----|---------|---------|*/
    cmp_ch_list_t       programedState;             /*!< Pre-programmed state for comparison result.*/
}cmp_trigger_mode_t;

/*!
 * @brief Defines the comparator module configuration
 *
 * This structure is used to configure all components of comparator module
 */
typedef struct cmp_module_t
{
    cmp_comparator_t        comparator;
    cmp_anmux_t             mux;
    cmp_dac_t               dac;
    cmp_trigger_mode_t      triggerMode;
}cmp_module_t;


/*******************************************************************************
 * Enumerations.
 ******************************************************************************/
 /*! @brief Function return status */
typedef enum
{
    CMP_DRV_FAIL        = 0,
    CMP_DRV_SUCCESS     = 1
} cmp_drv_status_t;


/*******************************************************************************
 * API
 ******************************************************************************/
 /*!
 * @name cMP DRV.
 * @{
 */

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Reset all registers
 *
 * This function set all CMP registers to reset values.
 *
 * @param instance - instance number
 * @return 
 *        - CMP_DRV_SUCCESS : Completed successfully.
 *        - CMP_DRV_FAIL : Error occurred.
 */
cmp_drv_status_t CMP_DRV_Reset(const uint32_t instance);

/*!
 * @brief Get reset configuration for all registers
 *
 * This function returns a configuration structure with reset values for all registers
 * from comparator module.
 *
 * @param config - the configuration structure
 * @return void
 */
void CMP_DRV_GetInitConfigAll(cmp_module_t* config);

/*!
 * @brief Configure all comparator features with the given configuration structure
 *
 * This function configures the comparator module with the options
 * provided in the config structure.
 *
 * @param instance - instance number
 * @param config - the configuration structure
 * @return 
 *        - CMP_DRV_SUCCESS : Completed successfully.
 *        - CMP_DRV_FAIL : Error occurred.
 */
cmp_drv_status_t CMP_DRV_Init(const uint32_t instance, const cmp_module_t* const config);

/*!
 * @brief Gets the current comparator configuration
 *
 * This function returns the current configuration for comparator as a configuration structure.
 *
 * @param instance - instance number
 * @param config - the configuration structure
 * @return 
 *        - CMP_DRV_SUCCESS : Completed successfully.
 *        - CMP_DRV_FAIL : Error occurred.
 */
cmp_drv_status_t CMP_DRV_GetConfigAll(const uint32_t instance, cmp_module_t* const config);

/*!
 * @brief Get reset configuration for registers related with DAC
 *
 * This function returns a configuration structure with reset values for features associated with DAC.
 *
 * @param config - the configuration structure
 * @return void
 */
void CMP_DRV_GetInitConfigDAC(cmp_dac_t* config);

/*!
 * @brief Configure only the DAC component
 *
 * This function configures the DAC with the options provided in the config structure.
 *
 * @param instance - instance number
 * @param config - the configuration structure
 * @return 
 *        - CMP_DRV_SUCCESS : Completed successfully.
 *        - CMP_DRV_FAIL : Error occurred.
 */
cmp_drv_status_t CMP_DRV_ConfigDAC(const uint32_t instance, cmp_dac_t* const config);

/*!
 * @brief Return current configuration for DAC
 *
 * This function returns current configuration only for DAC.
 *
 * @param instance - instance number
 * @param config - the configuration structure
 * @return 
 *        - CMP_DRV_SUCCESS : Completed successfully.
 *        - CMP_DRV_FAIL : Error occurred.
 */
cmp_drv_status_t CMP_DRV_GetDACConfig(const uint32_t instance, cmp_dac_t* const config);

/*!
 * @brief Get reset configuration for registers related with MUX
 *
 * This function returns a configuration structure with reset values for features associated with MUX.
 *
 * @param config - the configuration structure
 * @return void
 */
void CMP_DRV_GetInitConfigMUX(cmp_anmux_t* config);

/*!
 * @brief Configure only the MUX component
 *
 * This function configures the MUX with the options provided in the config structure.
 *
 * @param instance - instance number
 * @param config - the configuration structure
 * @return 
 *        - CMP_DRV_SUCCESS : Completed successfully.
 *        - CMP_DRV_FAIL : Error occurred.
 */
cmp_drv_status_t CMP_DRV_ConfigMUX(const uint32_t instance, cmp_anmux_t* const config);

/*!
 * @brief Return configuration only for the MUX component
 *
 * This function returns current configuration to determine which signals go to comparator ports.
 *
 * @param instance - instance number
 * @param config - the configuration structure
 * @return 
 *        - CMP_DRV_SUCCESS : Completed successfully.
 *        - CMP_DRV_FAIL : Error occurred.
 */
cmp_drv_status_t CMP_DRV_GetMUXConfig(const uint32_t instance, cmp_anmux_t* const config);

/*!
 * @brief Get reset configuration for registers related with Trigger Mode
 *
 * This function returns a configuration structure with reset values for features associated with Trigger Mode.
 *
 * @param config - the configuration structure
 * @return void
 */
void CMP_DRV_GetInitTriggerMode(cmp_trigger_mode_t* config);

/*!
 * @brief Configure trigger mode
 *
 * This function configures the trigger mode with the options provided in the config structure.
 *
 * @param instance - instance number
 * @param config - the configuration structure
 * @return 
 *        - CMP_DRV_SUCCESS : Completed successfully.
 *        - CMP_DRV_FAIL : Error occurred.
 */
cmp_drv_status_t CMP_DRV_ConfigTriggerMode(const uint32_t instance, cmp_trigger_mode_t* const config);

/*!
 * @brief Get current trigger mode configuration
 *
 * This function returns the current trigger mode configuration for trigger mode.
 *
 * @param instance - instance number
 * @param config - the configuration structure
 * @return 
 *        - CMP_DRV_SUCCESS : Completed successfully.
 *        - CMP_DRV_FAIL : Error occurred.
 */
cmp_drv_status_t CMP_DRV_GetTriggerModeConfig(const uint32_t instance, cmp_trigger_mode_t* const config);

/*!
 * @brief Get comparator output flags
 *
 * This function returns in <flags> comparator output flags(rising and falling edge).
 *
 * @param instance - instance number
 * @param - flags
 *      NO_EVENT    
 *      RISING_EDGE     
 *      FALLING_EDGE    
 *      BOTH_EDGE       
 * @return 
 *        - CMP_DRV_SUCCESS : Completed successfully.
 *        - CMP_DRV_FAIL : Error occurred.
 */
cmp_drv_status_t CMP_DRV_GetOutputFlags(const uint32_t instance, cmp_output_trigger_t *flags);

/*!
 * @brief Clear comparator output flags
 *
 * This function clear comparator output flags(rising and falling edge).
 *
 * @param instance - instance number
 * @param config - the configuration structure 
 * @return 
 *        - CMP_DRV_SUCCESS : Completed successfully.
 *        - CMP_DRV_FAIL : Error occurred.
 */
cmp_drv_status_t CMP_DRV_ClearOutputFlags(const uint32_t instance);

/*!
 * @brief Gets input channels change flags
 *
 * This function return in <flags> all input channels flags as uint8_t(1 bite
 *for each channel flag).
 *
 * @param instance - instance number
 * @param flags 
 * @return 
 *        - CMP_DRV_SUCCESS : Completed successfully.
 *        - CMP_DRV_FAIL : Error occurred.
 */
cmp_drv_status_t CMP_DRV_GetInputFlags(const uint32_t instance, cmp_ch_list_t *flags);

/*!
 * @brief Clear comparator input channels flags
 *
 * This function clear comparator input channels flags.
 *
 * @param config - the configuration structure 
 * @return void
 */
cmp_drv_status_t CMP_DRV_ClearInputFlags(const uint32_t instance);

/*!
 * @brief Get reset configuration for registers related with comparator features
 *
 * This function return a configuration structure with reset values for features associated with
 * comparator (DMA request, power mode, output select, interrupts enable, invert,
 * offset, hysteresis).
 *
 * @param config - the configuration structure
 * @return void
 */
void CMP_DRV_GetInitConfigComparator(cmp_comparator_t* config);

/*!
 * @brief Configure only comparator features
 *
 * This function configure only features related with comparator:
 * DMA request, power mode, output select, interrupts enable, invert,
 * offset, hysteresis.
 *
 * @param instance - instance number
 * @param config - the configuration structure 
 * @return 
 *        - CMP_DRV_SUCCESS : Completed successfully.
 *        - CMP_DRV_FAIL : Error occurred.
 */
cmp_drv_status_t CMP_DRV_ConfigComparator(const uint32_t instance, cmp_comparator_t *config);

/*!
 * @brief Return configuration for comparator from CMP module
 *
 * This function return configuration for features related with comparator:
 * DMA request, power mode, output select, interrupts enable, invert,
 * offset, hysteresis.
 *
 * @param instance - instance number
 * @param config - the configuration structure returned
 * @return 
 *        - CMP_DRV_SUCCESS : Completed successfully.
 *        - CMP_DRV_FAIL : Error occurred.
 */
cmp_drv_status_t CMP_DRV_GetComparatorConfig(const uint32_t instance, cmp_comparator_t *config);


/*! @}*/
#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __FSL_CMP_DRIVER_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/

