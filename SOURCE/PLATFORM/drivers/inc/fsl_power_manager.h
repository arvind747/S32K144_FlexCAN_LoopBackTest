/*
 * Copyright (c) 2014, Freescale Semiconductor, Inc.
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

#ifndef __FSL_POWER_MANAGER_H__
#define __FSL_POWER_MANAGER_H__

#include "fsl_device_registers.h"
#include "fsl_smc_hal.h"



 /*! @file */

/*!
 * @defgroup power_manager Power Manager
 * @brief The S32 SDK Power Manager provides a set of API/services to configure the power-related IPs, such as SMC, PMC, RCM, and so on.
 * @{
 *
 * ## Hardware background ## 
 *
 * Reset Control Module (RCM) implements many of the reset functions for the chip. 
 * It indicates the source of the most recent reset. It also implements features like 
 * power on reset filter or delay interrupt ( it delays the assertion of a system reset 
 * for a period of time while an interrupt is generated, this allows software to perform 
 * a graceful shutdown).
 *
 * Power management control (PMC) implements features like internal voltage regulator, 
 * power on reset, low voltage detect system (LVD),low power oscillator (LPO) and low 
 * voltage reset.
 * 
 * System mode controller (SMC) is passing the system into and out of all low-power 
 * Stop and Run modes. Controls the power, clocks and memories of the system to achieve 
 * the power consumption and functionality of that mode.
 *
 *
 * ## Driver consideration ##
 *
 * The Power Manager driver is developed on top of the SMC HAL, PMC HAL, and RCM HAL.
 * Low Power Manager provides API to handle the device power modes. It also supports 
 * run-time switching between multiple power modes. Each power mode is described by 
 * configuration structures with multiple power-related options. Low Power Manager 
 * provides a notification mechanism for registered callbacks and API for static and 
 * dynamic callback registration.
 *
 * The Driver uses structures for configuration. The user application can use the 
 * default for most settings, changing only what is necessary. There is a power mode
 * and a callback configuration structure. These structures may be generated using
 * Processor Expert.
 *
 * This driver provides functions for initializing power manager and changing the
 * power mode entry. Power mode entry and sleep-on-exit option are provided at 
 * initialization time through the power manager user configuration structure.
 * The available power mode entries are the following ones: HSRUN, RUN, VLPR, WAIT, 
 * VLPW, VLPS, PSTOP1 and PSTOP2
 *
 * All methods that access the hardware layer will return an error code to signal
 * if the operation succeeded or failed. These values are defined by the
 * power_manager_error_code_t enumeration, and the possible values are: success,
 * index out of range, switch error, wrong protection level, callback notification
 * errors, wrong clock setup error.
 *
 */


/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief Power modes enumeration.
 *
 * Defines power mode. Used in the power mode configuration structure
 * (power_manager_user_config_t). From ARM core perspective, Power modes
 * can be generally divided to run modes (High speed run, Run and
 * Very low power run), sleep (Wait and Very low power wait) and deep sleep modes
 * (all Stop modes).
 * List of power modes supported by specific chip along with requirements for entering
 * and exiting of these modes can be found in chip documentation.
 * List of all supported power modes:\n
 *  \li POWER_MANAGER_HSRUN - High speed run mode.
 *  \li POWER_MANAGER_RUN - Run mode.
 *  \li POWER_MANAGER_VLPR - Very low power run mode.
 *  \li POWER_MANAGER_WAIT - Wait mode.
 *  \li POWER_MANAGER_VLPW - Very low power wait mode.
 *  \li POWER_MANAGER_STOP - Stop mode.
 *  \li POWER_MANAGER_VLPS - Very low power stop mode.
 *  \li POWER_MANAGER_PSTOP1 - Partial stop 1 mode.
 *  \li POWER_MANAGER_PSTOP2 - Partial stop 2 mode.
 */
typedef enum _power_manager_modes {
#if FSL_FEATURE_SMC_HAS_HIGH_SPEED_RUN_MODE
  POWER_MANAGER_HSRUN,            /*!< High speed run mode.  */
#endif
  POWER_MANAGER_RUN,              /*!< Run mode. */
  POWER_MANAGER_VLPR,             /*!< Very low power run mode.  */
  POWER_MANAGER_WAIT,             /*!< Wait mode.  */
  POWER_MANAGER_VLPW,             /*!< Very low power wait mode.  */
  POWER_MANAGER_STOP,             /*!< Stop mode.  */
  POWER_MANAGER_VLPS,             /*!< Very low power stop mode.  */
#if FSL_FEATURE_SMC_HAS_PSTOPO
  POWER_MANAGER_PSTOP1,           /*!< Partial stop 1 mode. */
  POWER_MANAGER_PSTOP2,           /*!< Partial stop 2 mode. */
#endif
  POWER_MANAGER_MAX
} power_manager_modes_t;

/*!
 * @brief Power manager success code and error codes.
 *
 * Used as return value of Power manager functions.
 */
typedef enum _power_manager_error_code {
  POWER_MAN_SUCCESS,                   /*!< Success */
  POWER_MAN_ERR,                       /*!< Some error occurs. */
  POWER_MAN_ERR_OUT_OF_RANGE,          /*!< Configuration index out of range. */
  POWER_MAN_ERR_SWITCH,                /*!< Error occurs during mode switch. */
  POWER_MAN_ERR_PROTECTION,            /*!< Error occurs due to wrong protection level for power modes */
  POWER_MAN_ERR_NOTIFY_BEFORE,         /*!< Error occurs during send "BEFORE" notification. */
  POWER_MAN_ERR_NOTIFY_AFTER,          /*!< Error occurs during send "AFTER" notification.  */
  POWER_MAN_ERR_CLOCK                  /*!< Error occurs due to wrong clock setup for power modes */
} power_manager_error_code_t;

/*!
 * @brief Power manager policies.
 *
 * Define whether the power mode change is forced or not. Used to specify whether
 * the mode switch initiated by the POWER_SYS_SetMode() depends on the callback
 * notification results. For POWER_MANAGER_POLICY_FORCIBLE the power mode is changed
 * regardless of the results, while POWER_MANAGER_POLICY_AGREEMENT policy is used
 * the POWER_SYS_SetMode() is exited when any of the callbacks returns error code.
 * See also POWER_SYS_SetMode() description.
 */
typedef enum _power_manager_policy {
  POWER_MANAGER_POLICY_AGREEMENT,      /*!< POWER_SYS_SetMode() method is exited when any of the callbacks returns error code. */ 
  POWER_MANAGER_POLICY_FORCIBLE        /*!< Power mode is changed regardless of the results. */
} power_manager_policy_t;

/*! @brief The PM notification type. Used to notify registered callbacks */
typedef enum _power_manager_notify
{
  POWER_MANAGER_NOTIFY_RECOVER = 0x00U,  /*!< Notify IP to recover to previous work state.      */
  POWER_MANAGER_NOTIFY_BEFORE  = 0x01U,  /*!< Notify IP that system will change power setting.  */
  POWER_MANAGER_NOTIFY_AFTER   = 0x02U   /*!< Notify IP that have changed to new power setting. */
} power_manager_notify_t;

/*!
 * @brief The callback type, indicates what kinds of notification this callback handles.
 *
 * Used in the callback configuration structures (power_manager_callback_user_config_t) 
 * to specify when the registered callback will be called during power mode change initiated by 
 * POWER_SYS_SetMode().
 * Callback can be invoked in following situations:
 *  - before the power mode change (Callback return value can affect POWER_SYS_SetMode()
 *    execution. Refer to the  POWER_SYS_SetMode() and power_manager_policy_t documentation).
 *  - after entering one of the run modes or after exiting from one of the (deep) sleep power
 *    modes back to the run mode.
 *  - after unsuccessful attempt to switch power mode
 */
typedef enum _power_manager_callback_type {
  POWER_MANAGER_CALLBACK_BEFORE       = 0x01U, /*!< Before callback. */
  POWER_MANAGER_CALLBACK_AFTER        = 0x02U, /*!< After callback. */
  POWER_MANAGER_CALLBACK_BEFORE_AFTER = 0x03U  /*!< Before-After callback. */
} power_manager_callback_type_t;

/*!
 * @brief Callback-specific data.
 *
 * Reference to data of this type is passed during callback registration. The reference is
 * part of the power_manager_callback_user_config_t structure and is passed to the callback during 
 * power mode change notifications.
 */
typedef void power_manager_callback_data_t;

/*!
 * @brief Power mode user configuration structure.
 *
 * This structure defines S32K power mode with additional power options and specifies
 * transition to and out of this mode. Application may define multiple power modes and
 * switch between them. List of defined power modes is passed to the Power manager during
 * initialization as an array of references to structures of this type (see POWER_SYS_Init()).
 * Power modes can be switched by calling POWER_SYS_SetMode() which accepts index to the list
 * of power modes passed during manager initialization. Currently used power mode can be
 * retrieved by calling POWER_SYS_GetLastMode(), which returns index of the current power mode, or
 * by POWER_SYS_GetLastModeConfig(), which returns reference to the structure of current mode.
 * List of power mode configuration structure members depends on power options available
 * for specific chip. Complete list contains:
 *  mode - S32K power mode. List of available modes is chip-specific. See power_manager_modes_t
 *   list of modes.
 *  sleepOnExitOption - Controls whether the sleep-on-exit option value is used (when set to true)
 *   or ignored (when set to false). See sleepOnExitValue.
 *  sleepOnExitValue - When set to true, ARM core returns to sleep (S32K wait modes) or deep sleep
 *   state (S32K stop modes) after interrupt service finishes. When set to false, core stays
 *   woken-up.
 */
typedef struct _power_manager_mode_user_config {
  power_manager_modes_t powerMode;
  bool sleepOnExitOption;
  bool sleepOnExitValue;

} power_manager_user_config_t;

/*! @brief Power notification structure passed to registered callback function. */
typedef struct _power_notify_struct
{
  power_manager_user_config_t *targetPowerConfigPtr; /*!< Pointer to target power configuration */
  uint8_t targetPowerConfigIndex;    /*!< Target power configuration index. */
  power_manager_policy_t policy;     /*!< Clock transition policy.          */
  power_manager_notify_t notifyType; /*!< Clock notification type.          */
} power_manager_notify_struct_t;

/*!
 * @brief Callback prototype.
 *
 * Declaration of callback. It is common for registered callbacks.
 * Reference to function of this type is part of power_manager_callback_user_config_t callback 
 * configuration structure.
 * Depending on callback type, function of this prototype is called during power mode change
 * (see POWER_SYS_SetMode()) before the mode change, after it or in both cases to notify about
 * the change progress (see power_manager_callback_type_t). When called, type of the notification
 * is passed as parameter along with reference to entered power mode configuration structure
 * (see power_manager_notify_struct_t) and any data passed during the callback registration (see
 * power_manager_callback_data_t).
 * When notified before the mode change, depending on the power mode change policy (see
 * power_manager_policy_t) the callback may deny the mode change by returning any error code different
 * from POWER_MANAGER_SUCCESS (see POWER_SYS_SetMode()).
 * @param notify Notification structure. 
 * @param dataPtr Callback data. Refers to the data passed during callback registration. Intended to
 *  pass any driver or application data such as internal state information.
 * @return An error code or POWER_MANAGER_SUCCESS.
 */
typedef power_manager_error_code_t (* power_manager_callback_t)(
  power_manager_notify_struct_t * notify,
  power_manager_callback_data_t * dataPtr
);

/*!
 * @brief callback configuration structure
 *
 * This structure holds configuration of callbacks passed
 * to the Power manager during its initialization.
 * Callbacks of this type are expected to be statically
 * allocated.
 * This structure contains following application-defined data:
 *  callback - pointer to the callback function
 *  callbackType - specifies when the callback is called
 *  callbackData - pointer to the data passed to the callback
 */
typedef struct _power_manager_callback_user_config {
    power_manager_callback_t callbackFunction;
    power_manager_callback_type_t callbackType;
    power_manager_callback_data_t * callbackData;
} power_manager_callback_user_config_t;

/*!
 * @brief Power manager internal state structure.
 *
 * Power manager internal structure. Contains data necessary for Power manager proper
 * function. Stores references to registered power mode configurations,
 * callbacks, information about their numbers and other internal data.
 * This structure is statically allocated and initialized after POWER_SYS_Init() call.
 */
typedef struct _power_manager_state {
    power_manager_user_config_t * (* configs)[];   /*!< Pointer to power configure table.*/
    uint8_t configsNumber;                         /*!< Number of power configurations */
    power_manager_callback_user_config_t * (* staticCallbacks)[]; /*!< Pointer to callback table. */
    uint8_t staticCallbacksNumber;                 /*!< Max. number of callback configurations */
    uint8_t errorCallbackIndex;                    /*!< Index of callback returns error. */
    uint8_t currentConfig;                         /*!< Index of current configuration.  */
} power_manager_state_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Power manager initialization for operation.
 *
 * This function initializes the Power manager and its run-time state structure.
 * Reference to an array of Power mode configuration structures has to be passed
 * as a parameter along with a parameter specifying its size. At least one power mode
 * configuration is required. Optionally, reference to the array of predefined
 * callbacks can be passed with its size parameter.
 * For details about callbacks, refer to the power_manager_callback_user_config_t.
 * As Power manager stores only references to array of these structures, they have
 * to exist while Power manager is used.
 * It is expected that prior to the POWER_SYS_Init() call the write-once protection
 * register was configured appropriately allowing for entry to all required low power
 * modes.
 * The following is an example of how to set up two power modes and one
 * callback, and initialize the Power manager with structures containing their settings.
 * The example shows two possible ways the configuration structures can be stored
 * (ROM or RAM), although it is expected that they will be placed in the read-only
 * memory to save the RAM space. (Note: In the example it is assumed that the programmed chip
 * doesn't support any optional power options described in the power_manager_user_config_t)
 * :
 * @code
 
  power_manager_user_config_t vlprConfig = {   vlprConfig power mode configuration 
      .powerMode = POWER_MANAGER_VLPR,
      .sleepOnExitOption = false,
      .sleepOnExitValue = false,
  };       

  power_manager_user_config_t stopConfig = {   stopConfig power mode configuration 
      .powerMode = POWER_MANAGER_STOP,
      .sleepOnExitOption = false,
      .sleepOnExitValue = false,
  };        

  power_manager_user_config_t const * powerConfigsArr[] = {    Power mode configurations array
      &vlprConfig,
      &stopConfig
  };

  power_manager_callback_user_config_t callbackCfg0 = {  Callback configuration structure callbackCfg0
      .callbackFunction                     = &callback0,
      .callbackType                         = POWER_MANAGER_CALLBACK_BEFORE_AFTER,
      .callbackData                         = (void *)0,
  };     

  power_manager_callback_user_config_t const * callbacksConfigsArr[] = {  Callback configuration structures array
      &callbackCfg0
  };
 
  power_manager_error_code_t callback0(power_manager_notify_struct_t * notify,   Definition of power manager callback
                                       power_manager_callback_data_t * dataPtr)
  {
    power_manager_error_code_t ret = POWER_MAN_SUCCESS;
    ...    
    return ret;
  }

  int main(void) Main function 
  {    
    power_manager_error_code_t ret = POWER_MAN_SUCCESS;

    Calling of init method
    POWER_SYS_Init(&powerConfigsArr, 2U, &powerStaticCallbacksConfigsArr, 1U);  
    
    Switch to VLPR mode
    ret = POWER_SYS_SetMode(MODE_VLPR,POWER_MANAGER_POLICY_AGREEMENT);
    
    if (ret != POWER_MAN_SUCCESS) 
    {
      return -1;
    }
    return 0;
  }
  
 * @endcode
 *
 * @param[in] powerConfigsPtr A pointer to an array with references to all power
 *  configurations which will be handled by Power manager.
 * @param[in] configsNumber Number of power configurations. Size of powerConfigsPtr
 *  array.
 * @param[in] callbacksPtr A pointer to an array with references to callback configurations.
 *  If there are no callbacks to register during Power manager initialization, use NULL value.
 * @param[in] callbacksNumber Number of registered callbacks. Size of callbacksPtr
 *  array.
 * @return An error code or POWER_MANAGER_SUCCESS.
 */
power_manager_error_code_t POWER_SYS_Init(power_manager_user_config_t const * (* powerConfigsPtr)[],
                                          uint8_t configsNumber,
                                          power_manager_callback_user_config_t const * (* callbacksPtr)[],
                                          uint8_t callbacksNumber);

/*!
 * @brief This function deinitializes the Power manager.
 *
 * @return An error code or POWER_MANAGER_SUCCESS.
 */
power_manager_error_code_t POWER_SYS_Deinit(void);

/*!
 * @brief This function configures the power mode.
 *
 * This function switches to one of the defined power modes. Requested mode number is passed
 * as an input parameter. This function notifies all registered callback functions before
 * the mode change (using  POWER_MANAGER_CALLBACK_BEFORE set as callback type parameter),
 * sets specific power options defined in the power mode configuration and enters the specified
 * mode. In case of run modes (for example, Run, Very low power run, or High speed run), this function
 * also invokes all registered callbacks after the mode change (using POWER_MANAGER_CALLBACK_AFTER).
 * In case of sleep or deep sleep modes, if the requested mode is not exited through
 * a reset, these notifications are sent after the core wakes up.
 * Callbacks are invoked in the following order: All registered callbacks are notified
 * ordered by index in the callbacks array (see callbacksPtr parameter of POWER_SYS_Init()).
 * The same order is used for before and after switch notifications.
 * The notifications before the power mode switch can be used to obtain confirmation about
 * the change from registered callbacks. If any registered callback denies the power
 * mode change, further execution of this function depends on mode change policy: the mode
 * change is either forced (POWER_MANAGER_POLICY_FORCIBLE) or exited (POWER_MANAGER_POLICY_AGREEMENT).
 * When mode change is forced, the result of the before switch notifications are ignored. If
 * agreement is required, if any callback returns an error code then further notifications
 * before switch notifications are cancelled and all already notified callbacks are re-invoked
 * with POWER_MANAGER_CALLBACK_AFTER set as callback type parameter. The index of the callback
 * which returned error code during pre-switch notifications is stored (any error codes during
 * callbacks re-invocation are ignored) and POWER_SYS_GetErrorCallback() can be used to get it.
 * Regardless of the policies, if any callback returned an error code, an error code denoting in which phase
 * the error occurred is returned when POWER_SYS_SetMode() exits.
 * It is possible to enter any mode supported by the processor. Refer to the chip reference manual
 * for list of available power modes. If it is necessary to switch into intermediate power mode prior to
 * entering requested mode (for example, when switching from Run into Very low power wait through Very low
 * power run mode), then the intermediate mode is entered without invoking the callback mechanism.
 *
 * @param[in] powerModeIndex Requested power mode represented as an index into
 *  array of user-defined power mode configurations passed to the POWER_SYS_Init().
 * @param[in] policy Transaction policy
 * @return An error code or POWER_MANAGER_SUCCESS.
 */
power_manager_error_code_t POWER_SYS_SetMode(uint8_t powerModeIndex, power_manager_policy_t policy);

/*!
 * @brief This function returns power mode set as the last one.
 *
 * This function returns index of power mode which was set using POWER_SYS_SetMode() as the last one.
 * If the power mode was entered even though some of the registered callback denied the mode change,
 * or if any of the callbacks invoked after the entering/restoring run mode failed, then the return
 * code of this function has POWER_MANAGER_ERROR value.
 *
 * @param[out] powerModeIndexPtr Power mode which has been set represented as an index into array of power mode
 * configurations passed to the POWER_SYS_Init().
 * @return An error code or POWER_MANAGER_SUCCESS.
 */
power_manager_error_code_t POWER_SYS_GetLastMode(uint8_t* const powerModeIndexPtr);

/*!
 * @brief This function returns user configuration structure of power mode set as the last one.
 *
 * This function returns reference to configuration structure which was set using POWER_SYS_SetMode()
 * as the last one. If the current power mode was entered even though some of the registered callback denied
 * the mode change, or if any of the callbacks invoked after the entering/restoring run mode failed, then
 * the return code of this function has POWER_MANAGER_ERROR value.
 *
 * @param[out] powerModePtr Pointer to power mode configuration structure of power mode set as last one.
 * @return An error code or POWER_MANAGER_SUCCESS.
 */
power_manager_error_code_t POWER_SYS_GetLastModeConfig(power_manager_user_config_t** const powerModePtr);

/*!
 * @brief This function returns currently running power mode.
 *
 * This function reads hardware settings and returns currently running power mode. Generally,
 * this function can return only POWER_MANAGER_RUN, POWER_MANAGER_VLPR or POWER_MANAGER_HSRUN value.
 *
  * @return Currently used run power mode.
 */
power_manager_modes_t POWER_SYS_GetCurrentMode(void);

/*!
 * @brief This function returns the last failed notification callback.
 *
 * This function returns index of the last callback that failed during the power mode switch while
 * the last POWER_SYS_SetMode() was called. If the last POWER_SYS_SetMode() call ended successfully 
 * value equal to callbacks number is returned. Returned value represents index in the array of 
 * static call-backs.
 *
 * @return Callback index of last failed callback or value equal to callbacks count.
 */
uint8_t POWER_SYS_GetErrorCallbackIndex(void);

/*!
 * @brief This function returns the last failed notification callback configuration structure.
 *
 * This function returns pointer to configuration structure of the last callback that failed during 
 * the power mode switch while the last POWER_SYS_SetMode() was called. 
 * If the last POWER_SYS_SetMode() call ended successfully value NULL is returned. 
 *
 * @return Pointer to the callback configuration which returns error.
 */
power_manager_callback_user_config_t* POWER_SYS_GetErrorCallback(void);

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __FSL_POWER_MANAGER_H__ */
/*******************************************************************************
 * EOF
 ******************************************************************************/

