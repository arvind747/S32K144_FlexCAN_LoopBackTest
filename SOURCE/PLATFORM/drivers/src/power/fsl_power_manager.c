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

#include "fsl_power_manager.h"
#include "fsl_smc_hal.h"
#include "fsl_pmc_hal.h"
#include "fsl_rcm_hal.h"
#include "fsl_sim_hal.h"
#include "fsl_clock_manager.h"
#include <string.h>



/*******************************************************************************
 * Internal Variables
 ******************************************************************************/

/*! @brief Power manager internal structure. */
static power_manager_state_t gPowerManagerState;

#ifdef USE_RTOS
#if (USE_RTOS)
/*! @brief Power manager internal structure lock. */
mutex_t gPowerManagerStateSync;
#endif
#endif

/*******************************************************************************
 * PROTOTYPES
 ******************************************************************************/

static power_manager_error_code_t POWER_SYS_WaitForModeStatus(smc_run_mode_t mode);
static power_manager_error_code_t POWER_SYS_CallbacksManagement(power_manager_notify_struct_t *notifyStruct, uint8_t *currentStaticCallback, power_manager_policy_t policy);
static power_manager_error_code_t POWER_SYS_SwitchToSleepingPowerMode(const power_manager_user_config_t* const configPtr);
static power_manager_error_code_t POWER_SYS_SwitchToRunningPowerMode(const power_manager_user_config_t* const configPtr);
static power_manager_error_code_t POWER_SYS_CheckPowerModeIndex(uint8_t powerModeIndex);

/*!
 * @brief Macros for power manager lock mechanism.
 *
 * Mutex is used when operating system is present otherwise critical section
 * (global interrupt disable).
 *
 */
#ifdef USE_RTOS
#if (USE_RTOS)
  #define POWER_SYS_LOCK_INIT()    OSA_MutexCreate(&gPowerManagerStateSync)
  #define POWER_SYS_LOCK()         OSA_MutexLock(&gPowerManagerStateSync, OSA_WAIT_FOREVER)
  #define POWER_SYS_UNLOCK()       OSA_MutexUnlock(&gPowerManagerStateSync)
  #define POWER_SYS_LOCK_DEINIT()  OSA_MutexDestroy(&gPowerManagerStateSync)
#else
  #define POWER_SYS_LOCK_INIT()    do {}while(0)
  #define POWER_SYS_LOCK()         do {}while(0)
  #define POWER_SYS_UNLOCK()       do {}while(0)
  #define POWER_SYS_LOCK_DEINIT()  do {}while(0)
#endif
#else
  #define POWER_SYS_LOCK_INIT()    do {}while(0)
  #define POWER_SYS_LOCK()         do {}while(0)
  #define POWER_SYS_UNLOCK()       do {}while(0)
  #define POWER_SYS_LOCK_DEINIT()  do {}while(0)
#endif

/*! Timeout used for waiting to set new mode */
#define POWER_SET_MODE_TIMEOUT 1000U
/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : POWER_SYS_Init
 * Description   : Initializes the Power manager for operation.
 * This function initializes the Power manager and its run-time state structure.
 * Reference to an array of Power mode configuration structures has to be passed
 * as parameter along with parameter specifying its size. At least one power mode
 * configuration is required. Optionally, reference to array of predefined
 * call-backs can be passed with its size parameter.
 * For details about call-backs refer to the power_manager_callback_user_config_t.
 * As Power manager stores only references to array of these structures they have
 * to exist while Power manager is used.
 * It is expected that prior POWER_SYS_Init() call the write-once protection
 * register was configured appropriately allowing to enter all required low power
 * modes.
 * The following is an example of how to set up two power modes and three
 * call-backs and initialize the Power manager with structures containing their settings.
 * The example shows two possible ways how where the configuration structures can be stored
 * (ROM or RAM) although it is expected that they will be placed rather in the read-only
 * memory to save the RAM space. (Note: In the example it is assumed that the programmed chip
 * doesn't support any optional power options described in the power_manager_user_config_t)
 *
 *END**************************************************************************/
power_manager_error_code_t POWER_SYS_Init(power_manager_user_config_t const * (* powerConfigsPtr)[],
                                          uint8_t configsNumber,
                                          power_manager_callback_user_config_t const * (* callbacksPtr)[],
                                          uint8_t callbacksNumber)
{
  power_manager_error_code_t returnCode; /* Function return */

  /* Check input parameter - at least one power mode configuration is required */
  if ((powerConfigsPtr == NULL) || (configsNumber == 0U))
  {
    returnCode = POWER_MAN_ERR;
  }
  else
  {
    returnCode = POWER_MAN_SUCCESS;
  }
  /* Initialize internal state structure lock */
  POWER_SYS_LOCK_INIT();
  POWER_SYS_LOCK();

  /* Store references to user-defined power mode configurations */
  gPowerManagerState.configs = (power_manager_user_config_t *(*)[])powerConfigsPtr;
  gPowerManagerState.configsNumber = configsNumber;
  gPowerManagerState.currentConfig = 0U;
  /* Store references to user-defined callback configurations and increment call-back handle counter */
  if (callbacksPtr != NULL)
  {
    gPowerManagerState.staticCallbacks = (power_manager_callback_user_config_t *(*)[])callbacksPtr;
    gPowerManagerState.staticCallbacksNumber = callbacksNumber;
    /* Default value of handle of last call-back that returned error */
    gPowerManagerState.errorCallbackIndex = callbacksNumber;
  }
  else
  {
    gPowerManagerState.staticCallbacks = NULL;
    gPowerManagerState.staticCallbacksNumber = 0U;
    gPowerManagerState.errorCallbackIndex = 0U;
  }

  POWER_SYS_UNLOCK();

  return returnCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : POWER_SYS_Deinit
 * Description   : Deinitializes the Power manager.
 *
 *END**************************************************************************/
power_manager_error_code_t POWER_SYS_Deinit(void)
{
  /* Deinitialize internal state structure lock */
  POWER_SYS_LOCK_DEINIT();
  return POWER_MAN_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : POWER_SYS_SetMode
 * Description   : Configures the power mode.
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
 *END**************************************************************************/
power_manager_error_code_t POWER_SYS_SetMode(uint8_t powerModeIndex, power_manager_policy_t policy)
{
  power_manager_user_config_t * configPtr; /* Local pointer to the requested user-defined power mode configuration */
  power_manager_error_code_t returnCode; /* Function return */

  POWER_SYS_LOCK();

  /* Default value of handle of last call-back that returned error */
  gPowerManagerState.errorCallbackIndex = gPowerManagerState.staticCallbacksNumber;

  /* Initialization of local pointer to the requested user-defined power mode configuration */
  configPtr = (*gPowerManagerState.configs)[powerModeIndex];

  POWER_SYS_UNLOCK();

  /* Check the power mode index parameter */
  returnCode = POWER_SYS_CheckPowerModeIndex(powerModeIndex);
  if (POWER_MAN_SUCCESS != returnCode)
  {
    return returnCode;
  }

  bool successfulSwitch;                                 /* Power mode switch is successful or not */
  uint8_t currentStaticCallback = 0U;                    /* Index to array of statically registered call-backs */
  power_manager_notify_struct_t notifyStruct;            /* Callback notification structure */
  
  /* Set the transaction policy in the notification structure */
  notifyStruct.policy = policy;
  
  /* Set the target power mode configuration in the notification structure */
  notifyStruct.targetPowerConfigIndex = powerModeIndex;
  notifyStruct.targetPowerConfigPtr = configPtr;
  
  /* Notify those which asked to be called before the power mode change */
  notifyStruct.notifyType = POWER_MANAGER_NOTIFY_BEFORE;
  returnCode = POWER_SYS_CallbacksManagement(&notifyStruct,&currentStaticCallback,policy);
  
  /* Power mode switch */
  
  /* In case that any call-back returned error code and  policy doesn't force the mode switch go to after switch call-backs */
  if ((policy == POWER_MANAGER_POLICY_FORCIBLE) || (returnCode == POWER_MAN_SUCCESS))
  {
    /* Check whether the power mode is a sleeping or a running power mode */
    if (configPtr->powerMode >= POWER_MANAGER_WAIT)
    {
      /* Switch to a sleeping power mode */
      returnCode = POWER_SYS_SwitchToSleepingPowerMode(configPtr);
    }
    else
    {
      /* Switch to a running power mode */
      returnCode = POWER_SYS_SwitchToRunningPowerMode(configPtr);
    }
    successfulSwitch = (POWER_MAN_SUCCESS == returnCode);
  }
  else
  {
    /* Unsuccessful switch */
    successfulSwitch = false;
  }
  
  if (successfulSwitch)
  {
    /* End of successful switch */
  
    POWER_SYS_LOCK();
  
    /* Update current configuration index */
    gPowerManagerState.currentConfig = powerModeIndex;
  
    POWER_SYS_UNLOCK();
  
    /* Notify those which asked to be called after the power mode change */
    notifyStruct.notifyType = POWER_MANAGER_NOTIFY_AFTER;
    returnCode = POWER_SYS_CallbacksManagement(&notifyStruct,&currentStaticCallback,POWER_MANAGER_POLICY_FORCIBLE);
  }
  else
  {
    /* End of unsuccessful switch */
  
    /* Notify those which have been called before the power mode change */
    notifyStruct.notifyType = POWER_MANAGER_NOTIFY_RECOVER;
    POWER_SYS_CallbacksManagement(&notifyStruct,&currentStaticCallback,POWER_MANAGER_POLICY_FORCIBLE);
  }
  return returnCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : POWER_SYS_GetLastModeIndex
 * Description   : This function returns power mode set as the last one.
 *
 * This function returns index of power mode which was set using POWER_SYS_SetMode() as the last one.
 * If the power mode was entered although some of the registered call-back denied the mode change
 * or if any of the call-backs invoked after the entering/restoring run mode failed then the return
 * code of this function has POWER_MAN_ERR value.
 * value.
 *
 *END**************************************************************************/
power_manager_error_code_t POWER_SYS_GetLastMode(uint8_t* const powerModeIndexPtr)
{
  power_manager_error_code_t returnCode; /* Function return */
  POWER_SYS_LOCK();
  /* Pass index of user-defined configuration structure of currently running power mode */
  *powerModeIndexPtr = gPowerManagerState.currentConfig;
  /* Return whether all call-backs executed without error */
  if (gPowerManagerState.errorCallbackIndex == gPowerManagerState.staticCallbacksNumber)
  {
    returnCode = POWER_MAN_SUCCESS;
  }
  else
  {
    returnCode = POWER_MAN_ERR;
  }
  POWER_SYS_UNLOCK();
  return returnCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : POWER_SYS_GetLastModeConfig
 * Description   : This function returns user configuration structure of power mode set as the last one.
 *
 * This function returns reference to configuration structure which was set using POWER_SYS_SetMode()
 * as the last one. If the current power mode was entered although some of the registered call-back denied
 * the mode change or if any of the call-backs invoked after the entering/restoring run mode failed then
 * the return code of this function has POWER_MAN_ERR value.
 *
 *END**************************************************************************/
power_manager_error_code_t POWER_SYS_GetLastModeConfig(power_manager_user_config_t** const powerModePtr)
{
  power_manager_error_code_t returnCode; /* Function return */
  POWER_SYS_LOCK();
  /* Pass reference to user-defined configuration structure of currently running power mode */
  *powerModePtr = (*gPowerManagerState.configs)[gPowerManagerState.currentConfig];
  /* Return whether all call-backs executed without error */
  if (gPowerManagerState.errorCallbackIndex == gPowerManagerState.staticCallbacksNumber)
  {
    returnCode = POWER_MAN_SUCCESS;
  }
  else
  {
    returnCode = POWER_MAN_ERR;
  }
  POWER_SYS_UNLOCK();
  return returnCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : POWER_SYS_GetCurrentMode
 * Description   : Returns currently running power mode.
 *
 *END**************************************************************************/
power_manager_modes_t POWER_SYS_GetCurrentMode(void)
{
  power_manager_modes_t retVal;
  switch (SMC_HAL_GetPowerModeStatus(SMC))
  {
#if FSL_FEATURE_SMC_HAS_HIGH_SPEED_RUN_MODE
    /* High speed run mode */
    case STAT_HSRUN:
      retVal = POWER_MANAGER_HSRUN;
      break;
#endif
    /* Run mode */
    case STAT_RUN:
      retVal = POWER_MANAGER_RUN;
      break;
    /* Very low power run mode */
    case STAT_VLPR:
      retVal = POWER_MANAGER_VLPR;
      break;
    /* This should never happen - core has to be in some run mode to execute code */
    default:
      retVal = POWER_MANAGER_MAX;
      break;
  }
  return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : POWER_SYS_GetErrorCallbackIndex
 * Description   : Returns the last failed notification callback.
 *
 * This function returns index of the last call-back that failed during the power mode switch while
 * the last POWER_SYS_SetMode() was called. If the last POWER_SYS_SetMode() call ended successfully
 * value equal to callbacks number is returned. Returned value represents index in the array of
 * static call-backs.
 *
 *END**************************************************************************/
uint8_t POWER_SYS_GetErrorCallbackIndex (void)
{
  return gPowerManagerState.errorCallbackIndex;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : POWER_SYS_GetErrorCallback
 * Description   : Get the callback which returns error in last mode switch.
 *
 *END**************************************************************************/
power_manager_callback_user_config_t* POWER_SYS_GetErrorCallback(void) {
  /* If all callbacks return success. */
  return (gPowerManagerState.errorCallbackIndex
      >= gPowerManagerState.staticCallbacksNumber) ?
      NULL :
      (*gPowerManagerState.staticCallbacks)[gPowerManagerState.errorCallbackIndex];
}

/*FUNCTION**************************************************************************
 * Function Name : POWER_SYS_WaitForRunStatus
 * Description   :Internal function used by POWER_SYS_SwitchToSleepingPowerMode and
 *                POWER_SYS_SwitchToRunningPowerMode functions
 * mode           The expected running mode
 *
 *END*******************************************************************************/
static power_manager_error_code_t POWER_SYS_WaitForModeStatus(smc_run_mode_t mode)
{
  power_manager_error_code_t retCode;
  power_mode_stat_t modeStat;
  uint32_t i = 0U;

  switch(mode)
  {
    case SMC_RUN:
      modeStat = STAT_RUN;
      retCode = POWER_MAN_SUCCESS;
      break;
    case SMC_VLPR:
      modeStat = STAT_VLPR;
      retCode = POWER_MAN_SUCCESS;
      break;
#if FSL_FEATURE_SMC_HAS_HIGH_SPEED_RUN_MODE
    case SMC_HSRUN:
      modeStat = STAT_HSRUN;
      retCode = POWER_MAN_SUCCESS;
      break;
#endif
    default:
      /* invalid parameter */
      modeStat = STAT_INVALID;
      retCode = POWER_MAN_ERR;
      break;
  }
  if (POWER_MAN_SUCCESS == retCode)
  {
    for (; i < POWER_SET_MODE_TIMEOUT ; i++)
    {
      if(SMC_HAL_GetPowerModeStatus(SMC) == modeStat)
      {
        break;
      }
    }
  }
  if (i >= POWER_SET_MODE_TIMEOUT)
  {
    retCode = POWER_MAN_ERR_SWITCH;
  }
  return retCode;
}


/*FUNCTION**********************************************************************************************
 * Function Name : POWER_SYS_SwitchToRunningPowerMode
 * Description   :Internal function used by POWER_SYS_SetMode function to switch to a running power mode
 * configPtr   pointer to the requested user-defined power mode configuration
 *
 *END***************************************************************************************************/
static power_manager_error_code_t POWER_SYS_SwitchToRunningPowerMode(const power_manager_user_config_t* const configPtr)
{
  smc_power_mode_config_t halModeConfig; /* SMC HAL layer configuration structure */
  power_manager_error_code_t returnCode;

  /* Configure the HAL layer */
  switch (configPtr->powerMode) {
#if FSL_FEATURE_SMC_HAS_HIGH_SPEED_RUN_MODE
  /* High speed run mode */
  case POWER_MANAGER_HSRUN:
    /* High speed run mode can be entered only from Run mode */
    if (SMC_HAL_GetPowerModeStatus(SMC) != STAT_RUN)
    {
      SMC_HAL_SetRunModeControl(SMC, SMC_RUN);
      returnCode = POWER_SYS_WaitForModeStatus(SMC_RUN);
    }
    else
    {
      returnCode = POWER_MAN_SUCCESS;
    }
    if (POWER_MAN_SUCCESS == returnCode)
    {
      halModeConfig.powerModeName = POWER_MODE_HSRUN;

      /* Switch the mode */
      if (SMC_HAL_SetPowerMode(SMC, &halModeConfig) == SMC_HAL_SUCCESS)
      {
        returnCode = POWER_SYS_WaitForModeStatus(SMC_HSRUN);
      }
      else
      {
        returnCode = POWER_MAN_ERR_SWITCH;
      }
    }
    break;
#endif
  /* Run mode */
  case POWER_MANAGER_RUN:
    halModeConfig.powerModeName = POWER_MODE_RUN;
    returnCode = POWER_MAN_SUCCESS;
    /* Switch the mode */
    if (SMC_HAL_SetPowerMode(SMC, &halModeConfig) == SMC_HAL_SUCCESS)
    {
      returnCode = POWER_SYS_WaitForModeStatus(SMC_RUN);
    }
    else
    {
      returnCode = POWER_MAN_ERR_SWITCH;
    }
    break;
  /* Very low power run mode */
  case POWER_MANAGER_VLPR:
    /* Very low power run mode can be entered only from Run mode */
    if (SMC_HAL_GetPowerModeStatus(SMC) != STAT_RUN)
    {
      SMC_HAL_SetRunModeControl(SMC, SMC_RUN);
      returnCode = POWER_SYS_WaitForModeStatus(SMC_RUN);
    }
    else
    {
      returnCode = POWER_MAN_SUCCESS;
    }

    if (POWER_MAN_SUCCESS == returnCode)
    {
      halModeConfig.powerModeName = POWER_MODE_VLPR;

      /* Switch the mode */
      if (SMC_HAL_SetPowerMode(SMC, &halModeConfig) == SMC_HAL_SUCCESS)
      {
        returnCode = POWER_SYS_WaitForModeStatus(SMC_VLPR);
      }
      else
      {
        returnCode = POWER_MAN_ERR_SWITCH;
      }
    }
    break;
  /* Wait mode */
  default:
    /* invalid power mode */
    returnCode = POWER_MAN_ERR_SWITCH;
    halModeConfig.powerModeName = POWER_MODE_MAX;
    break;
  }

  return returnCode;
}


/*FUNCTION**********************************************************************************************
 * Function Name : POWER_SYS_SwitchToSleepingPowerMode
 * Description   :Internal function used by POWER_SYS_SetMode function to switch to a sleeping power mode
 * configPtr   pointer to the requested user-defined power mode configuration
 *
 *END***************************************************************************************************/
static power_manager_error_code_t POWER_SYS_SwitchToSleepingPowerMode(const power_manager_user_config_t* const configPtr)
{
  smc_power_mode_config_t halModeConfig; /* SMC HAL layer configuration structure */
  power_manager_error_code_t returnCode;

  /* Configure the HAL layer */
  switch (configPtr->powerMode) {
  /* Wait mode */
  case POWER_MANAGER_WAIT:
    /* Wait mode can be entered only from Run mode */
    if (SMC_HAL_GetPowerModeStatus(SMC) != STAT_RUN)
    {
      SMC_HAL_SetRunModeControl(SMC, SMC_RUN);
      returnCode = POWER_SYS_WaitForModeStatus(SMC_RUN);
    }
    else
    {
      returnCode = POWER_MAN_SUCCESS;
    }
    halModeConfig.powerModeName = POWER_MODE_WAIT;
    break;
  /* Very low power wait mode */
  case POWER_MANAGER_VLPW:
    /* Very low power wait mode can be netered only from Very low power run mode */
    if (SMC_HAL_GetPowerModeStatus(SMC) != STAT_VLPR)
    {
      SMC_HAL_SetRunModeControl(SMC, SMC_VLPR);
      returnCode = POWER_SYS_WaitForModeStatus(SMC_VLPR);
    }
    else
    {
      returnCode = POWER_MAN_SUCCESS;
    }
    halModeConfig.powerModeName = POWER_MODE_VLPW;
    break;
#if FSL_FEATURE_SMC_HAS_PSTOPO
    /* Partial stop modes */
  case POWER_MANAGER_PSTOP1:
    /* fall-through */
  case POWER_MANAGER_PSTOP2:
    /* fall-through */
#endif
  /* Stop mode */
  case POWER_MANAGER_STOP:
    /* Stop mode can be entered only from Run mode */
    if (SMC_HAL_GetPowerModeStatus(SMC) != STAT_RUN)
    {
      SMC_HAL_SetRunModeControl(SMC, SMC_RUN);
      returnCode = POWER_SYS_WaitForModeStatus(SMC_RUN);
    }
    else
    {
      returnCode = POWER_MAN_SUCCESS;
    }
    halModeConfig.powerModeName = POWER_MODE_STOP;
#if FSL_FEATURE_SMC_HAS_PSTOPO
    halModeConfig.pstopOption = true;
    /* Set the partial stop option value */
    if (POWER_MANAGER_PSTOP1 == configPtr->powerMode)
    {
      halModeConfig.pstopOptionValue = SMC_PSTOP_STOP1;
    }
    else if(POWER_MANAGER_PSTOP2 == configPtr->powerMode)
    {
      halModeConfig.pstopOptionValue = SMC_PSTOP_STOP2;
    }
    else
    {
      halModeConfig.pstopOptionValue = SMC_PSTOP_STOP;
    }
#endif
      break;
  /* Very low power stop mode */
  case POWER_MANAGER_VLPS:
    /* Very low power stop mode can be entered only from Run mode or Very low power run mode*/
    if ((SMC_HAL_GetPowerModeStatus(SMC) != STAT_RUN) && (SMC_HAL_GetPowerModeStatus(SMC) != STAT_VLPR))
    {
      SMC_HAL_SetRunModeControl(SMC, SMC_RUN);
      returnCode = POWER_SYS_WaitForModeStatus(SMC_RUN);
    }
    else
    {
      returnCode = POWER_MAN_SUCCESS;
    }
    halModeConfig.powerModeName = POWER_MODE_VLPS;
    break;
  default:
    /* invalid power mode */
    returnCode = POWER_MAN_ERR_SWITCH;
    halModeConfig.powerModeName = POWER_MODE_MAX;
    break;
  }

  if (POWER_MAN_SUCCESS == returnCode)
  {
    /* Configure ARM core what to do after interrupt invoked in (deep) sleep state */
    if ((configPtr->sleepOnExitOption) && (configPtr->powerMode >= POWER_MANAGER_WAIT))
    {
      if (configPtr->sleepOnExitValue)
      {
        /* Go back to (deep) sleep state on ISR exit */
        FSL_SCB->SCR |= FSL_SCB_SCR_SLEEPONEXIT_MASK;
      }
      else
      {
        /* Do not re-enter (deep) sleep state on ISR exit */
        FSL_SCB->SCR &= ~(FSL_SCB_SCR_SLEEPONEXIT_MASK);
      }
    }

    /* Switch the mode */
    if (SMC_HAL_SetPowerMode(SMC, &halModeConfig) != SMC_HAL_SUCCESS)
    {
      returnCode = POWER_MAN_ERR_SWITCH;
    }
  }
  return returnCode;
}

/*FUNCTION**********************************************************************************************
 * Function Name : POWER_SYS_CallbacksManagement
 * Description   : Internal function used by POWER_SYS_SetMode function for callback management
 * notifyStruct            callback notification structure
 * currentStaticCallback   index to array of statically registered call-backs
 * policy                  transaction policy
 *
 *END***************************************************************************************************/
static power_manager_error_code_t POWER_SYS_CallbacksManagement(power_manager_notify_struct_t *notifyStruct, uint8_t *currentStaticCallback, power_manager_policy_t policy)
{
  uint8_t callbacksNumber = 0;                               /* The total number of callbacks */
  power_manager_callback_user_config_t * callbackConfig;     /* Pointer to callback configuration */
  power_manager_error_code_t returnCode = POWER_MAN_SUCCESS; /* Function return */
  power_manager_error_code_t errorCode;                      /* Error code to be returned (error case) */
  power_manager_callback_type_t callbackTypeFilter;          /* Callback types to be excluded */

  POWER_SYS_LOCK();

  switch(notifyStruct->notifyType)
  {
    /* notify before */
    case POWER_MANAGER_NOTIFY_BEFORE:
      callbacksNumber = gPowerManagerState.staticCallbacksNumber;
      callbackTypeFilter = POWER_MANAGER_CALLBACK_AFTER;
      errorCode = POWER_MAN_ERR_NOTIFY_BEFORE;
      break;
    /* notify after */
    case POWER_MANAGER_NOTIFY_AFTER:
      callbacksNumber = gPowerManagerState.staticCallbacksNumber;
      callbackTypeFilter = POWER_MANAGER_CALLBACK_BEFORE;
      errorCode = POWER_MAN_ERR_NOTIFY_AFTER;
      break;
     /* notify recover */
    case POWER_MANAGER_NOTIFY_RECOVER:
      callbacksNumber = *currentStaticCallback;
      callbackTypeFilter = POWER_MANAGER_CALLBACK_AFTER;
      errorCode = POWER_MAN_ERR_NOTIFY_BEFORE;
      break;
    default:
    /* nothing to execute */
    break;
  }

  /* From all statically registered call-backs... */
  for ((*currentStaticCallback) = 0U; (*currentStaticCallback) < callbacksNumber; (*currentStaticCallback)++)
  {
    callbackConfig = ((*gPowerManagerState.staticCallbacks)[*currentStaticCallback]);
    /* Check pointer to static callback configuration */
    if ( callbackConfig != NULL ){
      /* ...notify only those which asked to be called */
      if ( callbackTypeFilter != callbackConfig->callbackType)
      {
        /* In case that call-back returned error code mark it, store the call-back handle and eventually cancel the mode switch */
        if (POWER_MAN_SUCCESS != ((power_manager_error_code_t)(callbackConfig->callbackFunction(notifyStruct, callbackConfig->callbackData))))
        {
          returnCode = errorCode;
          gPowerManagerState.errorCallbackIndex = *currentStaticCallback;
          /* If not forcing power mode switch, call all already notified call-backs to revert their state as the mode change is canceled */
          if (policy != POWER_MANAGER_POLICY_FORCIBLE)
          {
            break;
          }
        }
      }
    }
  }

  POWER_SYS_UNLOCK();

  return returnCode;
}

/*FUNCTION**********************************************************************************************
 * Function Name : POWER_SYS_CallbacksManagement
 * Description   : Internal function used by POWER_SYS_SetMode function for power mode index parameter checking
 * powerModeIndex          Requested power mode represented as an index into
 *
 *END***************************************************************************************************/
static power_manager_error_code_t POWER_SYS_CheckPowerModeIndex(uint8_t powerModeIndex)
{
  power_manager_user_config_t* configPtr;  /* Local pointer to the requested user-defined power mode configuration */
  power_manager_error_code_t returnCode;   /* Function return */

  POWER_SYS_LOCK();

  /* Initialization of local pointer to the requested user-defined power mode configuration */
  configPtr = (*gPowerManagerState.configs)[powerModeIndex];

  /* Requested power mode configuration availability check */
  if (powerModeIndex >= gPowerManagerState.configsNumber)
  {
    returnCode = POWER_MAN_ERR_OUT_OF_RANGE;
  }
  /* Check that pointer to the requested user-defined power mode configuration is valid */
  else if (configPtr == NULL)
  {
    returnCode = POWER_MAN_ERR;
  }
  /* Check that the requested power mode is not protected */
  else if( ((configPtr->powerMode == POWER_MANAGER_VLPR) ||
      (configPtr->powerMode == POWER_MANAGER_VLPW) ||
      (configPtr->powerMode == POWER_MANAGER_VLPS)) &&
    (SMC_HAL_GetProtectionMode(SMC, ALLOW_VLP) == false) )
  {
    returnCode = POWER_MAN_ERR_PROTECTION;
  }
#if FSL_FEATURE_SMC_HAS_HIGH_SPEED_RUN_MODE
  else if( (configPtr->powerMode == POWER_MANAGER_HSRUN) &&
    (SMC_HAL_GetProtectionMode(SMC, ALLOW_HSRUN) == false) )
  {
    returnCode = POWER_MAN_ERR_PROTECTION;
  }
#endif
  else
  {
    returnCode = POWER_MAN_SUCCESS;
  }
  POWER_SYS_UNLOCK();
  return returnCode;
}


/*******************************************************************************
 * EOF
 ******************************************************************************/

