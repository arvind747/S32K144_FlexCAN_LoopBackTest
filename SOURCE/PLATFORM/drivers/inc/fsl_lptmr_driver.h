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
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __FSL_LPTMR_DRIVER_H__
#define __FSL_LPTMR_DRIVER_H__

#include <stdint.h>
#include <stdbool.h>
#include "fsl_device_registers.h"
#include "fsl_lptmr_hal.h"

/*! @file */

/*!
 * @addtogroup lptmr_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief Defines the configuration structure for general (generic) use-case
 *
 * This structure is used to configure the LPTMR in any mode (Timer or Pulse Counter)
 */
typedef struct _lptmr_general_config_t
{
    bool dmaRequest;                    /*!< Enable/Disable DMA requests */
    bool interruptEnable;               /*!< Enable/Disable Interrupt */
    lptmr_pinselect_t pinSelect;        /*!< Pin selection for Pulse-Counter */
    lptmr_pinpolarity_t pinPolarity;    /*!< Pin Polarity for Pulse-Counter */
    bool freeRun;                       /*!< Enable/Disable Free Running Mode */
    lptmr_workmode_t workMode;          /*!< (Timer/Pulse) Counter Mode */
    lptmr_prescaler_t prescaler;        /*!< Prescaler Selection */
    bool bypassPrescaler;               /*!< Enable/Disable prescaler bypass */
    lptmr_clocksource_t clockSelect;    /*!< Clock selection for Timer/Glitch filter*/
    uint16_t compareValue;              /*!< Compare value (ticks) */
} lptmr_general_config_t;

/*!
 * @brief Defines the configuration structure for Pulse Counter use-case
 *
 * This structure is used to configure the LPTMR in Pulse Counter Mode
 */
typedef struct _lptmr_pulsecounter_config_t
{
    bool dmaRequest;                    /*!< Enable/Disable DMA requests */
    bool interruptEnable;               /*!< Enable/Disable Interrupt */
    lptmr_pinselect_t pinSelect;        /*!< Pin selection for Pulse-Counter */
    lptmr_pinpolarity_t pinPolarity;    /*!< Pin Polarity for Pulse-Counter */
    bool freeRun;                       /*!< Enable/Disable Free Running Mode */
    lptmr_prescaler_t prescaler;        /*!< Prescaler Selection */
    bool bypassPrescaler;               /*!< Enable/Disable prescaler bypass */
    lptmr_clocksource_t clockSelect;    /*!< Clock selection for Glitch filter*/
    uint16_t compareValue;              /*!< Compare value (ticks) */
} lptmr_pulsecounter_config_t;

/*!
 * @brief Defines the configuration structure for Timer use-case
 *
 * This structure is used to configure the LPTMR in Timer Mode
 */
typedef struct _lptmr_timer_config_t
{
    bool dmaRequest;                    /*!< Enable/Disable DMA requests */
    bool interruptEnable;               /*!< Enable/Disable Interrupt */
    bool freeRun;                       /*!< Enable/Disable Free Running Mode */
    lptmr_clocksource_t clockSelect;    /*!< Clock selection for Timer*/
    uint32_t compareValueUs;            /*!< Compare value (microseconds) */
    uint32_t maxCompareValueUs;         /*!< Max Compare value (microseconds) for Free-Running mode */
} lptmr_timer_config_t;

/*! @brief LPTMR DRV API return status*/
typedef enum _lptmr_drv_status_t
{
    LPTMR_DRV_SUCCESS,                  /*!< Success */
    LPTMR_DRV_WARNING_INCORRECT_TIMING, /*!< Timing might not be correct*/
    LPTMR_DRV_WARNING_EVENT_EXPIRED,    /*!< Event might be lost */
    LPTMR_DRV_FAIL_CANNOT_CONFIGURE,    /*!< Error, the timer already started */
    LPTMR_DRV_FAIL_MODE_MISMATCH,       /*!< Incorrect mode already configured */
    LPTMR_DRV_FAIL_INVALID_CLOCK,       /*!< No clock source */
} lptmr_status_t;

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @name LPTMR Driver Functions
 * @{
 */

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name General Mode
 * These functions can be called either in Timer or Pulse-Counter operational
 * modes and can be used to configure any hardware feature. It is possible to
 * configure the driver any of the other modes and retrieve the configuration
 * is this mode.
 */

/*! @{*/

/*!
 * @brief Initialize configuration structure, General mode.
 *
 * Initialize the General use-case configuration structure with default values.
 *
 *
 *
 *
 * @param[out] config pointer to the the configuration structure
 * @return  One of the possible status codes:
 * - LPTMR_DRV_SUCCESS
 */
lptmr_status_t LPTMR_DRV_InitGeneralConfigStruct(
    lptmr_general_config_t* const config
    );


/*!
 * @brief Initializes the LPTMR in General mode (Timer or Pulse Counter)
 *
 * Initialize the hardware with given configuration in the General use-case
 * (Timer or Pulse-Counter).
 *
 *
 *
 *
 * @param[in] instance LPTMR peripheral instance number
 * @param[in] config pointer to the the configuration structure
 * @param[in] startcounter start the counter immediately after configuration
 * @return  One of the possible status codes:
 * - LPTMR_DRV_SUCCESS:
 * - LPTMR_DRV_FAIL_CANNOT_CONFIGURE: counter already started
 * - LPTMR_DRV_FAIL_INVALID_CLOCK: invalid clock source
 */
lptmr_status_t LPTMR_DRV_InitGeneral(
    const uint32_t instance,
    const lptmr_general_config_t* const config,
    const bool startcounter
    );


/*!
 * @brief Configure LPTMR In General mode.
 *
 * Configure the hardware in the General use-case (Timer or Pulse-Counter).
 *
 *
 *
 *
 * @param[in] instance LPTMR peripheral instance number
 * @param[in] config pointer to the the configuration structure
 * @return  One of the possible status codes:
 * - LPTMR_DRV_SUCCESS: completed successfully
 * - LPTMR_DRV_FAIL_CANNOT_CONFIGURE: counter already started
 * - LPTMR_DRV_FAIL_INVALID_CLOCK: invalid clock source
 */
lptmr_status_t LPTMR_DRV_SetGeneralConfig(
    const uint32_t instance,
    const lptmr_general_config_t* const config
    );


/*!
 * @brief Get the current configuration of the LPTMR as a General mode
 * configuration.
 *
 * This function returns the current config for the LPTMR.
 *
 *
 *
 *
 * @param[in] instance LPTMR peripheral instance number
 * @param[out] config pointer to the the configuration structure
 * @return  One of the possible status codes:
 * - LPTMR_DRV_SUCCESS: completed successfully
 */
lptmr_status_t LPTMR_DRV_GetGeneralConfig(
    const uint32_t instance,
    lptmr_general_config_t* const config
    );

/*! @}*/


/*!
 * @name Timer Mode
 * These functions can be called only in Timer operational mode, otherwise
 * will return an error.
 */
/*! @{*/

/*!
 * @brief Initialize configuration structure, Timer mode.
 *
 * Initialize the Timer use-case configuration structure with default values.
 *
 *
 *
 *
 * @param[out] config pointer to the the configuration structure
 * @return  One of the possible status codes:
 * - LPTMR_DRV_SUCCESS: completed successfully
 */
lptmr_status_t LPTMR_DRV_InitTimerConfigStruct(
    lptmr_timer_config_t* const config
    );


/*!
 * @brief Initializes the LPTMR in Timer mode
 *
 * This function initializes LPTMR peripheral for the timer use case.
 *
 *
 *
 *
 * @param[in] instance LPTMR peripheral instance number
 * @param[in] config pointer to the the configuration structure
 * @param[in] startcounter start the counter immediately after configuration
 * @return  One of the possible status codes:
 * - LPTMR_DRV_SUCCESS: completed successfully
 * - LPTMR_DRV_FAIL_CANNOT_CONFIGURE: timer already running
 * - LPTMR_DRV_FAIL_INVALID_CLOCK: clock source invalid. No settings written.
 * - LPTMR_DRV_WARNING_INCORRECT_TIMING: settings might not be correct (period
 *  is too small/ too large for the selected clock source) but the closest
 *  possible settings were written.
 */
lptmr_status_t LPTMR_DRV_InitTimer(
    const uint32_t instance,
    const lptmr_timer_config_t* const config,
    const bool startcounter
    );


/*!
 * @brief Change the Timer mode configuration of the LPTMR
 *
 * This function reconfigures LPTMR, so the peripheral it must be stopped before
 * changes can be made. This function works only in Timer mode.
 *
 *
 *
 *
 * @param[in] instance LPTMR peripheral instance number
 * @param[in] config pointer to the the configuration structure
 * @return  One of the possible status codes:
 * - LPTMR_DRV_SUCCESS: completed successfully
 * - LPTMR_DRV_FAIL_CANNOT_CONFIGURE: timer started
 * - LPTMR_DRV_FAIL_MODE_MISMATCH: not in Timer Mode
 * - LPTMR_DRV_FAIL_INVALID_CLOCK: clock source invalid. No settings written.
 * - LPTMR_DRV_WARNING_INCORRECT_TIMING: settings might not be correct (period
 *  is too small/ too large for the selected clock source) but the closest
 *  possible settings were written.
 */
lptmr_status_t LPTMR_DRV_SetTimerConfig(
    const uint32_t instance,
    const lptmr_timer_config_t* const config
    );


/*!
 * @brief Get the current configuration of the LPTMR as a Timer config
 *
 * This function returns the current config for the LPTMR. This function works
 * only in Timer mode.
 *
 *
 *
 *
 * @param[in] instance LPTMR peripheral instance number
 * @param[in] config pointer to the the configuration structure
 * @return  One of the possible status codes:
 * - LPTMR_DRV_SUCCESS: completed successfully
 * - LPTMR_DRV_FAIL_INVALID_CLOCK: no clock source
 * - LPTMR_DRV_FAIL_MODE_MISMATCH: not in Timer Mode
 * - LPTMR_DRV_WARNING_INCORRECT_TIMING: computed settings might not be correct
 */
lptmr_status_t LPTMR_DRV_GetTimerConfig(
    const uint32_t instance,
    lptmr_timer_config_t* const config
    );


/*!
 * @brief Set the compare value in Timer Mode in microseconds
 *
 * This function works only in Timer Mode.
 *
 *
 *
 *
 * @param[in] instance LPTMR peripheral instance number
 * @param[in] compareValueUs compare value in microseconds
 * @return  One of the possible status codes:
 * - LPTMR_DRV_SUCCESS: completed successfully
 * - LPTMR_DRV_FAIL_CANNOT_CONFIGURE: timer started
 * - LPTMR_DRV_FAIL_MODE_MISMATCH: not in Timer Mode
 * - LPTMR_DRV_FAIL_INVALID_CLOCK: no clock source
 * - LPTMR_DRV_WARNING_EVENT_EXPIRED: compare value set too late
 * - LPTMR_DRV_WARNING_INCORRECT_TIMING: timing value might not be correct
 */
lptmr_status_t LPTMR_DRV_SetCompareValueUs(
    const uint32_t instance,
    uint32_t compareValueUs
    );


/*!
 * @brief Get the compare value in Timer mode expressed in microseconds
 *
 * Compare value is returned in microsecond units. This function works only in 
 * Timer Mode.
 *
 *
 *
 *
 * @param[in] instance LPTMR peripheral instance number
 * @param[out] compareValueUs compare value in microseconds
 * @return  One of the possible status codes:
 * - LPTMR_DRV_SUCCESS: completed successfully
 * - LPTMR_DRV_FAIL_INVALID_CLOCK: no clock source
 * - LPTMR_DRV_FAIL_MODE_MISMATCH: not in Timer Mode
 * - LPTMR_DRV_WARNING_INCORRECT_TIMING: timing value might not be correct
 */
lptmr_status_t LPTMR_DRV_GetCompareValueUs(
    const uint32_t instance,
    uint32_t* const compareValueUs
    );

/*! @}*/

/*!
 * @name Pulse-Counter Mode
 * These functions can be called only in Pulse-Counter operational mode, otherwise
 * will return an error.
 */
/*! @{*/

/*!
 * @brief Initialize configuration structure, Pulse-Counter mode.
 *
 * Initialize the Pulse-Counter use-case configuration structure with default
 * values.
 *
 *
 *
 *
 * @param[out] config pointer to the the configuration structure
 * @return  One of the possible status codes:
 * - LPTMR_DRV_SUCCESS: completed successfully
 */
lptmr_status_t LPTMR_DRV_InitPulseCounterConfigStruct(
    lptmr_pulsecounter_config_t* const config
    );


/*!
 * @brief Initializes the LPTMR in Pulse Counter mode
 *
 * This function initializes LPTMR peripheral in Pulse-Counter Mode.
 *
 *
 *
 *
 * @param[in] instance LPTMR peripheral instance number
 * @param[in] config pointer to the the configuration structure
 * @param[in] startcounter start the counter immediately after configuration
 * @return  One of the possible status codes:
 * - LPTMR_DRV_SUCCESS: completed successfully
 * - LPTMR_DRV_FAIL_CANNOT_CONFIGURE: counter already started
 * - LPTMR_DRV_FAIL_INVALID_CLOCK: invalid clock source
 */
lptmr_status_t LPTMR_DRV_InitPulseCounter(
    const uint32_t instance,
    const lptmr_pulsecounter_config_t* const config,
    const bool startcounter
    );

/*!
 * @brief Change the Pulse-Counter configuration of the LPTMR
 *
 * This function reconfigures LPTMR, so the peripheral it must be stopped before
 * changes can be made. This function works only in Pulse-Counter mode.
 *
 *
 *
 *
 * @param[in] instance LPTMR peripheral instance number
 * @param[in] config pointer to the the configuration structure
 * @return  One of the possible status codes:
 * - LPTMR_DRV_SUCCESS: completed successfully
 * - LPTMR_DRV_FAIL_MODE_MISMATCH: not in Pulse-Counter mode
 * - LPTMR_DRV_FAIL_CANNOT_CONFIGURE: counter already started
 * - LPTMR_DRV_FAIL_INVALID_CLOCK: invalid clock source for the glitch filter.
 */
lptmr_status_t LPTMR_DRV_SetPulseCounterConfig(
    const uint32_t instance,
    const lptmr_pulsecounter_config_t* const config
    );


/*!
 * @brief Get the current configuration of the LPTMR as a pulse counter config
 *
 * This function returns the current config for the LPTMR. This function works
 * only in Pulse-Counter Mode.
 *
 *
 *
 *
 * @param[in] instance LPTMR peripheral instance number
 * @param[in] config pointer to the the configuration structure
 * @return  One of the possible status codes:
 * - LPTMR_DRV_SUCCESS: completed successfully
 * - LPTMR_DRV_FAIL_MODE_MISMATCH: not in Pulse-Counter mode.
 */
lptmr_status_t LPTMR_DRV_GetPulseCounterConfig(
    const uint32_t instance,
    lptmr_pulsecounter_config_t* const config
    );

/*! @}*/

/*!
 * @name Operational mode independent functions
 * These functions can be called in any mode
 */
/*! @{*/

/*!
 * @brief De-initialize the LPTMR peripheral
 *
 * This function de-initializes the LPTMR peripheral.
 *
 *
 *
 *
 * @param[in] instance LPTMR peripheral instance number
 * @return  One of the possible status codes:
 * - LPTMR_DRV_SUCCESS: completed successfully
 */
lptmr_status_t LPTMR_DRV_Deinit(
    const uint32_t instance
    );



/*!
 * @brief Change the Compare Value in counter tick units.
 *
 * This function changes the Compare Value using clock ticks units.
 *
 *
 *
 * @param[in] instance LPTMR peripheral instance number
 * @param[in] compareval the new Compare Value
 * @return  One of the possible status codes:
 * - LPTMR_DRV_SUCCESS: completed successfully
 * - LPTMR_DRV_FAIL_INVALID_CLOCK: no clock source
 * - LPTMR_DRV_FAIL_CANNOT_CONFIGURE: cannot reconfigure compare value
 * - LPTMR_DRV_WARNING_EVENT_EXPIRED: compare value set too late
 */
lptmr_status_t LPTMR_DRV_SetCompareValueTicks(
    const uint32_t instance,
    const uint16_t compareval
    );


/*!
 * @brief Get the Compare Value in counter tick units
 *
 * This function returns the Compare Value using clock ticks units.
 *
 *
 *
 * @param[in] instance LPTMR peripheral instance number
 * @return    the current Compare Value
 */
uint16_t LPTMR_DRV_GetCompareValueTicks(
    const uint32_t instance
    );


/*!
 * @brief Get the current state of the Compare Flag (Interrupt Pending)
 *
 * This function returns the current state of the Compare Flag (Interrupt Pending)
 *
 *
 *
 * @param[in] instance LPTMR peripheral instance number
 * @return    the state of the Compare Flag
 */
bool LPTMR_DRV_GetCompareFlag(
    const uint32_t instance
    );


/*!
 * @brief Clear the Compare/Interrupt Pending Flag
 *
 * This function clears the Compare/Interrupt Pending Flag
 *
 *
 *
 * @param[in] instance LPTMR peripheral instance number
 */
void LPTMR_DRV_ClearCompareFlag(
    const uint32_t instance
    );


/*!
 * @brief Configure the Interrupt of the LPTMR (enable or disable)
 *
 * This function sets a new state for the Interrupt of LPTMR.
 *
 *
 *
 * @param[in] instance LPTMR peripheral instance number
 * @param[in] enableinterrupt the new state of the LPTMR interrupt enable.
 * @return  One of the possible status codes:
 * - LPTMR_DRV_SUCCESS: completed successfully
 * - LPTMR_DRV_FAIL_CANNOT_CONFIGURE: cannot reconfigure setting
 */
lptmr_status_t LPTMR_DRV_SetInterrupt(
    const uint32_t instance,
    const bool enableinterrupt
    );


/*!
 * @brief Set a new interrupt handler for the LPTMR
 *
 * This function configures a new Interrupt Handler Routine for the LPTMR.
 *
 *
 *
 * @param[in] instance LPTMR peripheral instance number
 * @param[in] isr the new LPTMR Interrupt Handler.
 * @return  One of the possible status codes:
 * - LPTMR_DRV_SUCCESS: completed successfully
 */
lptmr_status_t LPTMR_DRV_IsrInstallHandler(
    const uint32_t instance,
    void (* isr)(
        void
        )
    );


/*!
 * @brief Get the counter value in counter tick units.
 *
 * This function returns the current counter value in ticks units.
 *
 *
 *
 * @param[in] instance LPTMR peripheral instance number
 * @return The Counter Value
 */
uint16_t LPTMR_DRV_GetCounterValueTicks(
    const uint32_t instance
    );


/*!
 * @brief Enable the LPTMR / Start the counter
 *
 * This function starts the LPTMR counter.
 *
 *
 *
 * @param[in] instance LPTMR peripheral instance number
 * @return  One of the possible status codes:
 * - LPTMR_DRV_SUCCESS: completed successfully
 */
lptmr_status_t LPTMR_DRV_StartCounter(
    const uint32_t instance
    );


/*!
 * @brief Disable the LPTMR / Stop the counter
 *
 * This function stops the counter.
 *
 *
 *
 * @param[in] instance LPTMR peripheral instance number
 * @return  One of the possible status codes:
 * - LPTMR_DRV_SUCCESS: completed successfully
 */
lptmr_status_t LPTMR_DRV_StopCounter(
    const uint32_t instance
    );

/*! @}*/

/*! @}*/

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __FSL_LPTMR_DRIVER_H__ */
/*******************************************************************************
 * EOF
 ******************************************************************************/
