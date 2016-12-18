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

#if !defined(__FSL_SMC_HAL_H__)
#define __FSL_SMC_HAL_H__

#include "fsl_device_registers.h"
#include <stdbool.h>

/*! @file */

/*!
 * @defgroup fsl_smc_hal System Mode Controller (SMC)
 * @ingroup power_manager
 * @brief This module covers the functionality of the System Mode Controller (SMC) peripheral.
 * <p>
 *  SMC HAL provides the API for reading and writing register bit-fields belonging to the SMC module.
 * </p>
 * <p>
 *  For higher-level functionality, use the Power Manager driver.
 * </p>
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief Power Modes */
typedef enum _power_modes {
  POWER_MODE_RUN,                            /*!< RUN power mode */
  POWER_MODE_WAIT,                           /*!< WAIT power mode */
  POWER_MODE_STOP,                           /*!< STOP power mode */
  POWER_MODE_VLPR,                           /*!< VLPR power mode */
  POWER_MODE_VLPW,                           /*!< VLPW power mode */
  POWER_MODE_VLPS,                           /*!< VLPS power mode */
  POWER_MODE_HSRUN,                          /*!< HSRUN power mode */
  POWER_MODE_MAX                             /*!< The total number of power modes */
} power_modes_t;

/*!
 * @brief Error code definition for the system mode controller manager APIs.
 */
typedef enum _smc_hal_error_code {
  SMC_HAL_SUCCESS,                           /*!< Success */
  SMC_HAL_NO_SUCH_MODE_NAME,                 /*!< Cannot find the mode name specified*/
  SMC_HAL_ALREADY_IN_THE_STATE,              /*!< Already in the required state*/
  SMC_HAL_NOT_ALLOWED_MODE,                  /*!< The specified mode is not allowed */
  SMC_HAL_TIMEOUT_MODE_CHANGE,               /*!< Power mode change operation failed */
  SMC_HAL_FAILED                             /*!< Unknown error, operation failed*/
} smc_hal_error_code_t;

/*! @brief Power Modes in PMSTAT*/
typedef enum _power_mode_stat {
  STAT_RUN     = 0x01,             /*!< 0000_0001 - Current power mode is RUN*/
  STAT_STOP    = 0x02,             /*!< 0000_0010 - Current power mode is STOP*/
  STAT_VLPR    = 0x04,             /*!< 0000_0100 - Current power mode is VLPR*/
  STAT_VLPW    = 0x08,             /*!< 0000_1000 - Current power mode is VLPW*/
  STAT_VLPS    = 0x10,             /*!< 0001_0000 - Current power mode is VLPS*/
  STAT_HSRUN   = 0x80,              /*!< 1000_0000 - Current power mode is HSRUN*/
  STAT_INVALID = 0xFF               /*!< 1111_1111 - Non-existing power mode*/
} power_mode_stat_t;

/*! @brief Power Modes Protection*/
typedef enum _power_modes_protect {
  ALLOW_HSRUN,                    /*!< Allow High Speed Run mode*/
  ALLOW_VLP,                      /*!< Allow Very-Low-Power Modes*/
  ALLOW_MAX
} power_modes_protect_t;

/*!
 * @brief Run mode definition
 */
typedef enum _smc_run_mode {
  SMC_RUN,                                /*!< normal RUN mode*/
  SMC_RESERVED_RUN,
  SMC_VLPR,                               /*!< Very-Low-Power RUN mode*/
  SMC_HSRUN                               /*!< High Speed Run mode (HSRUN)*/
} smc_run_mode_t;

/*!
 * @brief Stop mode definition
 */
typedef enum _smc_stop_mode {
  SMC_STOP            = 0U,    /*!< Normal STOP mode*/
  SMC_RESERVED_STOP1  = 1U,    /*!< Reserved*/
  SMC_VLPS            = 2U     /*!< Very-Low-Power STOP mode*/
} smc_stop_mode_t;



/*! @brief Partial STOP option*/
typedef enum _smc_pstop_option {
  SMC_PSTOP_STOP,                          /*!< STOP - Normal Stop mode*/
  SMC_PSTOP_STOP1,                         /*!< Partial Stop with both system and bus clocks disabled*/
  SMC_PSTOP_STOP2                          /*!< Partial Stop with system clock disabled and bus clock enabled*/
} smc_pstop_option_t;


/*! @brief Power mode protection configuration*/
typedef struct _smc_power_mode_protection_config {
  bool                vlpProt;            /*!< VLP protect*/
#if FSL_FEATURE_SMC_HAS_HIGH_SPEED_RUN_MODE 
  bool                hsrunProt;          /*!< HSRUN protect */
#endif
} smc_power_mode_protection_config_t;

/*! @brief Power mode control configuration used for calling the SMC_SYS_SetPowerMode API. */
typedef struct _smc_power_mode_config {
  power_modes_t       powerModeName;      /*!< Power mode(enum), see power_modes_t */
#if FSL_FEATURE_SMC_HAS_PSTOPO
  bool                pstopOption;        /*!< If PSTOPO option is needed */
  smc_pstop_option_t  pstopOptionValue;   /*!< PSTOPO option(enum), see smc_pstop_option_t */
#endif
} smc_power_mode_config_t;

/*! @brief SMC module version number */
typedef struct _smc_version_info_t {
  uint8_t  majorNumber;       /**< Major Version Number */
  uint8_t  minorNumber;       /**< Minor Version Number */
  uint16_t featureNumber;     /**< Feature Specification Number */
} smc_version_info_t;



/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

/*! @name System mode controller APIs*/
/*@{*/

/*!
 * @brief Get the version of the SMC module
 * 
 * @param[in]  baseAddr  base address of the SMC module
 * @param[out] versionInfo  Device Version Number
 */
void SMC_HAL_GetVersion(const SMC_Type* const baseAddr, smc_version_info_t* const versionInfo);

/*!
 * @brief Configures the power mode.
 *
 * This function configures the power mode control for both run, stop, and
 * stop sub mode if needed. Also it configures the power options for a specific
 * power mode. An application should follow the proper procedure to configure and 
 * switch power modes between  different run and stop modes. For proper procedures 
 * and supported power modes, see an appropriate chip reference
 * manual. See the smc_power_mode_config_t for required
 * parameters to configure the power mode and the supported options. Other options
 * may need to be individually configured through the HAL driver. See the HAL driver
 * header file for details.
 *
 * @param baseAddr  Base address for current SMC instance.
 * @param powerModeConfig Power mode configuration structure smc_power_mode_config_t 
 * @return errorCode SMC error code
 */
smc_hal_error_code_t SMC_HAL_SetPowerMode(SMC_Type* const baseAddr, 
                                          const smc_power_mode_config_t* const powerModeConfig);

/*!
 * @brief Configures all power mode protection settings.
 *
 * This function  configures the power mode protection settings for
 * supported power modes in the specified chip family. The available power modes
 * are defined in the smc_power_mode_protection_config_t. An application should provide
 * the protect settings for all supported power modes on the chip. This
 * should be done at an early system level initialization stage. See the reference manual
 * for details. This register can only write once after the power reset. If the user has 
 * only a single option to set,
 * either use this function or use the individual set function.
 * 
 * 
 * @param[in] baseAddr  Base address for current SMC instance.
 * @param[in] protectConfig Configurations for the supported power mode protect settings
 *                      - See smc_power_mode_protection_config_t for details.
 */
void SMC_HAL_SetProtectionMode(SMC_Type* const baseAddr, 
                               const smc_power_mode_protection_config_t* const protectConfig);
/*!
 * @brief Gets the the current power mode protection setting.
 *
 * This function  gets the current power mode protection settings for
 * a specified power mode.
 *
 * @param baseAddr[in]  Base address for current SMC instance.
 * @param protect[in]   Power mode to set for protection
 * @return state  Status of the protection setting
 *                - true: Allowed
 *                - false: Not allowed
*/
bool SMC_HAL_GetProtectionMode(const SMC_Type* const baseAddr, const power_modes_protect_t protect);

/*!
 * @brief Configures the the RUN mode control setting.
 *
 * This function  sets the run mode settings, for example, normal run mode,
 * very lower power run mode, etc. See the smc_run_mode_t for supported run
 * mode on the chip family and the reference manual for details about the 
 * run mode.
 *
 * @param[in] baseAddr  Base address for current SMC instance.
 * @param[in] runMode   Run mode setting defined in smc_run_mode_t
 */
static inline void SMC_HAL_SetRunModeControl(SMC_Type* const baseAddr, const smc_run_mode_t runMode)
{
  uint32_t regValue = baseAddr->PMCTRL;
  regValue &= ~(SMC_PMCTRL_RUNM_MASK);
  regValue |= SMC_PMCTRL_RUNM(runMode);
  baseAddr->PMCTRL = regValue;
}

/*!
 * @brief Gets  the current RUN mode configuration setting.
 *
 * This function  gets the run mode settings. See the smc_run_mode_t 
 * for a supported run mode on the chip family and the reference manual for 
 * details about the run mode.
 *
 * @param[in] baseAddr  Base address for current SMC instance.
 * @return setting Run mode configuration setting
 */
static inline smc_run_mode_t SMC_HAL_GetRunModeControl(const SMC_Type* const baseAddr)
{
  uint32_t regValue = baseAddr->PMCTRL;
  regValue = (regValue & SMC_PMCTRL_RUNM_MASK) >> SMC_PMCTRL_RUNM_SHIFT;
  return (smc_run_mode_t)regValue;
}

/*!
 * @brief Configures  the STOP mode control setting.
 *
 * This function  sets the stop mode settings, for example, normal stop mode,
 * very lower power stop mode, etc. See the  smc_stop_mode_t for supported stop
 * mode on the chip family and the reference manual for details about the 
 * stop mode.
 *
 * @param[in] baseAddr  Base address for current SMC instance.
 * @param[in] stopMode  Stop mode defined in smc_stop_mode_t
 */
static inline void SMC_HAL_SetStopModeControl(SMC_Type* const baseAddr, const smc_stop_mode_t stopMode)
{
  uint32_t regValue = baseAddr->PMCTRL;
  regValue &= ~(SMC_PMCTRL_STOPM_MASK);
  regValue |= SMC_PMCTRL_STOPM(stopMode);
  baseAddr->PMCTRL = regValue;
}


/*!
 * @brief Gets the current STOP mode control settings.
 *
 * This function  gets the stop mode settings, for example, normal stop mode,
 * very lower power stop mode, etc. See the  smc_stop_mode_t for supported stop
 * mode on the chip family and the reference manual for details about the 
 * stop mode.
 *
 * @param[in] baseAddr  Base address for current SMC instance.
 * @return setting Current stop mode configuration setting
 */
static inline smc_stop_mode_t SMC_HAL_GetStopModeControl(const SMC_Type* const baseAddr)
{
  uint32_t regValue = baseAddr->PMCTRL;
  regValue = (regValue & SMC_PMCTRL_STOPM_MASK) >> SMC_PMCTRL_STOPM_SHIFT;
  return (smc_stop_mode_t)regValue;
}


#if FSL_FEATURE_SMC_HAS_PSTOPO
/*!
 * @brief Configures the PSTOPO (Partial Stop Option).
 *
 * This function  sets the PSTOPO option. It controls whether a Partial 
 * Stop mode is entered when the STOPM=STOP. When entering a Partial Stop mode from the
 * RUN mode, the PMC, SCG and Flash remain fully powered allowing the device
 * to wakeup almost instantaneously at the expense of a higher power consumption.
 * In PSTOP2, only the system clocks are gated, which allows the peripherals running on bus
 * clock to remain fully functional. In PSTOP1, both system and bus clocks are
 * gated. Refer to the smc_pstop_option_t for supported options. See the reference
 * manual for details.
 *
 * @param[in] baseAddr  Base address for current SMC instance.
 * @param[in] option PSTOPO option setting defined in smc_pstop_option_t
 */
static inline void SMC_HAL_SetPartialStopOption(SMC_Type* const baseAddr, const smc_pstop_option_t option)
{
  uint32_t regValue = baseAddr->STOPCTRL;
  regValue &= ~(SMC_STOPCTRL_PSTOPO_MASK);
  regValue |= SMC_STOPCTRL_PSTOPO(option);
  baseAddr->STOPCTRL = regValue;
}




/*!
 * @brief Gets the configuration of the PSTOPO option.
 *
 * This function  gets the current PSTOPO option setting. See the  configuration
 * function for more details.
 *
 * @param[in] baseAddr  Base address for current SMC instance.
 * @return option Current PSTOPO option setting
 */
static inline smc_pstop_option_t SMC_HAL_GetPartialStopOption(const SMC_Type* const baseAddr)
{
  uint32_t regValue = baseAddr->STOPCTRL;
  regValue = (regValue & SMC_STOPCTRL_PSTOPO_MASK) >> SMC_STOPCTRL_PSTOPO_SHIFT;
  return (smc_pstop_option_t)regValue;
}

#endif

/*!
 * @brief Gets the current power mode stat.
 *
 * This function  returns the current power mode stat. Once application
 * switches the power mode, it should always check the stat to check whether it 
 * runs into the specified mode or not. An application  should  check 
 * this mode before switching to a different mode. The system  requires that
 * only certain modes can switch to other specific modes. See the 
 * reference manual for details and the _power_mode_stat for information about
 * the power stat.
 *
 * @param[in] baseAddr  Base address for current SMC instance.
 * @return stat  Current power mode stat
 */
static inline power_mode_stat_t SMC_HAL_GetPowerModeStatus(const SMC_Type* const baseAddr)
{
  uint32_t regValue = baseAddr->PMSTAT;
  regValue = (regValue & SMC_PMSTAT_PMSTAT_MASK) >> SMC_PMSTAT_PMSTAT_SHIFT;
  return (power_mode_stat_t)regValue;
}



/*@}*/

#if defined(__cplusplus)
}
#endif /* __cplusplus*/

/*! @}*/

#endif /* __FSL_SMC_HAL_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/

