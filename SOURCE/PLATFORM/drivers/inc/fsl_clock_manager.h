/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
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

#if !defined(__FSL_CLOCK_MANAGER_H__)
#define __FSL_CLOCK_MANAGER_H__

#include "fsl_device_registers.h"
#include "fsl_sim_hal.h"
#include "fsl_scg_hal.h"
#include "fsl_pcc_hal.h"

/*
 * Include the cpu specific clock API header files.
 */
#if (defined(S32K144_SERIES))
    /* Clock System Level API header file */
    #include "../src/clock/S32K144/fsl_clock_S32K144.h"
#else
    #error "No valid CPU defined!"
#endif

/*! @addtogroup clock_manager*/
/*! @{*/

/*! @file*/

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief Error code definition for the clock manager APIs
 */
typedef enum _clock_manager_error_code {
    CLOCK_MANAGER_SUCCESS,                    /*!< Success */
    CLOCK_MANAGER_ERROR,                      /*!< Some error occurs. */
    CLOCK_MANAGER_NO_SUCH_CLOCK_NAME,         /*!< Invalid name */
    CLOCK_MANAGER_INVALID_PARAM,              /*!< Invalid parameter */
    CLOCK_MANAGER_ERROR_OUT_OF_RANGE,         /*!< Configuration index out of range.               */
    CLOCK_MANAGER_ERROR_NOTIFICATION_BEFORE,  /*!< Error occurs during send "BEFORE" notification. */
    CLOCK_MANAGER_ERROR_NOTIFICATION_AFTER,   /*!< Error occurs during send "AFTER" notification.  */
    CLOCK_MANAGER_ERROR_UNKNOWN,              /*!< Unknown error.                                  */
} clock_manager_error_code_t;

/*******************************************************************************
 * API
 ******************************************************************************/

/*! @brief Clock configuration structure. */
typedef struct ClockUserConfig
{
    scg_config_t                scgConfig;      /*!< SCG Clock configuration.      */
    sim_clock_config_t          simConfig;      /*!< SIM Clock configuration.      */
    pcc_config_t                pccConfig;      /*!< PCC Clock configuration.      */
} clock_manager_user_config_t;

/*! @brief The clock notification type. */
typedef enum _clock_manager_notify
{
    CLOCK_MANAGER_NOTIFY_RECOVER = 0x00U,  /*!< Notify IP to recover to previous work state.      */
    CLOCK_MANAGER_NOTIFY_BEFORE  = 0x01U,  /*!< Notify IP that system will change clock setting.  */
    CLOCK_MANAGER_NOTIFY_AFTER   = 0x02U,  /*!< Notify IP that have changed to new clock setting. */
} clock_manager_notify_t;

/*! @brief The callback type, indicates what kinds of notification this callback handles. */
typedef enum _clock_manager_callback_type
{
    CLOCK_MANAGER_CALLBACK_BEFORE       = 0x01U, /*!< Callback handles BEFORE notification.          */
    CLOCK_MANAGER_CALLBACK_AFTER        = 0x02U, /*!< Callback handles AFTER notification.           */
    CLOCK_MANAGER_CALLBACK_BEFORE_AFTER = 0x03U  /*!< Callback handles BEFORE and AFTER notification */
} clock_manager_callback_type_t;

/*! @brief Clock transition policy. */
typedef enum ClockManagerPolicy
{
    CLOCK_MANAGER_POLICY_AGREEMENT,  /*!< Clock transfers gracefully. */
    CLOCK_MANAGER_POLICY_FORCIBLE    /*!< Clock transfers forcefully. */
} clock_manager_policy_t;

/*! @brief Clock notification structure passed to clock callback function. */
typedef struct ClockNotifyStruct
{
    uint8_t targetClockConfigIndex;    /*!< Target clock configuration index. */
    clock_manager_policy_t policy;     /*!< Clock transition policy.          */
    clock_manager_notify_t notifyType; /*!< Clock notification type.          */
} clock_notify_struct_t;

/*! @brief Type of clock callback functions. */
typedef clock_manager_error_code_t (*clock_manager_callback_t)(clock_notify_struct_t *notify,
                                                               void* callbackData);

/*! @brief Structure for callback function and its parameter. */
typedef struct ClockManagerCallbackUserConfig
{
    clock_manager_callback_t      callback;      /*!< Entry of callback function.     */
    clock_manager_callback_type_t callbackType;  /*!< Callback type.                  */
    void* callbackData;                          /*!< Parameter of callback function. */
} clock_manager_callback_user_config_t;

/*! @brief Clock manager state structure. */
typedef struct ClockManagerState
{
    clock_manager_user_config_t const **configTable;/*!< Pointer to clock configure table.*/
    uint8_t clockConfigNum;                         /*!< Number of clock configurations.  */
    uint8_t curConfigIndex;                         /*!< Index of current configuration.  */
    clock_manager_callback_user_config_t **callbackConfig; /*!< Pointer to callback table. */
    uint8_t callbackNum;                            /*!< Number of clock callbacks.       */
    uint8_t errorCallbackIndex;                     /*!< Index of callback returns error. */
} clock_manager_state_t;

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

/*!
 * @name Dynamic clock setting
 * @{
 */

/*!
 * @brief Install pre-defined clock configurations.
 *
 * This function installs the pre-defined clock configuration table to
 * clock manager.
 *
 * @param clockConfigsPtr    Pointer to the clock configuration table.
 * @param configsNumber      Number of clock configurations in table.
 * @param callbacksPtr       Pointer to the callback configuration table.
 * @param callbacksNumber    Number of callback configurations in table.
 *
 * @return Error code.
 */
clock_manager_error_code_t CLOCK_SYS_Init(clock_manager_user_config_t const **clockConfigsPtr,
                                          uint8_t configsNumber,
                                          clock_manager_callback_user_config_t **callbacksPtr,
                                          uint8_t callbacksNumber);

/*!
 * @brief Set system clock configuration according to pre-defined structure.
 *
 * This function sets system to target clock configuration, before transition,
 * clock manager will send notifications to all drivers registered to the
 * callback table.  When graceful policy is used, if some drivers are not ready
 * to change, clock transition will not occur, all drivers still work in
 * previous configuration and error is returned. When forceful policy is used,
 * all drivers should stop work and system changes to new clock configuration.
 *
 * @param targetConfigIndex Index of the clock configuration.
 * @param policy            Transaction policy, graceful or forceful.
 *
 * @return Error code.
 *
 * @note If external clock is used in the target mode, please make sure it is
 * enabled, for example, if the external oscillator is used, please setup
 * EREFS/HGO correctly and make sure OSCINIT is set.
 */
clock_manager_error_code_t CLOCK_SYS_UpdateConfiguration(uint8_t targetConfigIndex,
                                                         clock_manager_policy_t policy);

/*!
 * @brief Set system clock configuration.
 *
 * This function sets the system to target configuration, it only sets the
 * clock modules registers for clock mode change, but not send notifications
 * to drivers. This function is different by different SoCs.
 *
 * @param config Target configuration.
 *
 * @return Error code.
 *
 * @note If external clock is used in the target mode, please make sure it is
 * enabled, for example, if the external oscillator is used, please setup
 * EREFS/HGO correctly and make sure OSCINIT is set.
 */
clock_manager_error_code_t CLOCK_SYS_SetConfiguration(clock_manager_user_config_t const * config);

/*!
 * @brief Get current system clock configuration.
 *
 * @return Current clock configuration index.
 */
uint8_t CLOCK_SYS_GetCurrentConfiguration(void);

/*!
 * @brief Get the callback which returns error in last clock switch.
 *
 * When graceful policy is used, if some IP is not ready to change clock
 * setting, the callback will return error and system stay in current
 * configuration. Applications can use this function to check which
 * IP callback returns error.
 *
 * @return Pointer to the callback which returns error.
 */
clock_manager_callback_user_config_t* CLOCK_SYS_GetErrorCallback(void);


/*!
 * @brief Gets the clock frequency for a specific clock name.
 *
 * This function checks the current clock configurations and then calculates
 * the clock frequency for a specific clock name defined in clock_names_t.
 * The SCG must be properly configured before using this function. See
 * the reference manual for supported clock names for different chip families.
 * The returned value is in Hertz. If it cannot find the clock name
 * or the name is not supported for a specific chip family, it returns an
 * error.
 *
 * @param clockName Clock names defined in clock_names_t
 * @param frequency Returned clock frequency value in Hertz
 * @return status   Error code defined in clock_manager_error_code_t
 */
clock_manager_error_code_t CLOCK_SYS_GetFreq(clock_names_t clockName,
                                             uint32_t *frequency);

/* @} */


#if defined(__cplusplus)
}
#endif /* __cplusplus*/

/*! @}*/

#endif /* __FSL_CLOCK_MANAGER_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/

