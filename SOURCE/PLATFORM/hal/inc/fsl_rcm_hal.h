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
#if !defined(__FSL_RCM_HAL_H__)
#define __FSL_RCM_HAL_H__

#include "fsl_device_registers.h"
#include <stdbool.h>


/*! @file */

/*!
 * @defgroup fsl_rcm_hal Reset Control Module (RCM)
 * @ingroup power_manager
 * @brief This module covers the functionality of the Reset Control Module (RCM) peripheral.
 * <p>
 *  RCM HAL provides the API for reading and writing register bit-fields belonging to the RCM module.
 * </p>
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
 
/*! @brief System Reset Source Name definitions */
typedef enum _rcm_source_names {
    RCM_LOW_VOLT_DETECT,             /* low voltage detect reset */
    RCM_LOSS_OF_CLK,                 /* loss of clock reset */
    RCM_LOSS_OF_LOCK,                /* loss of lock reset */
    RCM_WATCH_DOG,                   /* watch dog reset */
    RCM_EXTERNAL_PIN,                /* external pin reset */
    RCM_POWER_ON,                    /* power on reset */
    RCM_SJTAG,                       /* JTAG generated reset */
    RCM_CORE_LOCKUP,                 /* core lockup reset */
    RCM_SOFTWARE,                    /* software reset */
    RCM_SMDM_AP,                     /* MDM-AP system reset */
    RCM_STOP_MODE_ACK_ERR,           /* stop mode ack error reset */
    RCM_SRC_NAME_MAX
} rcm_source_names_t;

/*! @brief Reset pin filter select in Run and Wait modes */
typedef enum _rcm_filter_run_wait_modes {
    RCM_FILTER_DISABLED,          /* All filtering disabled */
    RCM_FILTER_BUS_CLK,           /* Bus clock filter enabled */
    RCM_FILTER_LPO_CLK,           /* LPO clock filter enabled */
    RCM_FILTER_RESERVED           /* Reserved setting */
} rcm_filter_run_wait_modes_t;

/*! @brief Boot from ROM configuration. */
typedef enum _rcm_boot_rom_config {
    RCM_BOOT_FLASH,        /* boot from flash */
    RCM_BOOT_ROM_CFG0,     /* boot from boot rom due to BOOTCFG0 */
    RCM_BOOT_ROM_FOPT,     /* boot from boot rom due to FOPT[7] */
    RCM_BOOT_ROM_BOTH      /* boot from boot rom due to both BOOTCFG0 and FOPT[7] */
} rcm_boot_rom_config_t;


/*! @brief Reset delay time. */
typedef enum _rcm_reset_delay_time_t {
    RCM_8LPO_CYCLES_DELAY,       /* reset delay time 8 LPO cycles */
    RCM_32LPO_CYCLES_DELAY,      /* reset delay time 32 LPO cycles */
    RCM_128LPO_CYCLES_DELAY,     /* reset delay time 128 LPO cycles */
    RCM_512LPO_CYCLES_DELAY      /* reset delay time 512 LPO cycles */
} rcm_reset_delay_time_t;

/*! @brief RCM module version number */
typedef struct _rcm_version_info_t {
    uint8_t  majorNumber;       /**< Major Version Number */
    uint8_t  minorNumber;       /**< Minor Version Number */
    uint16_t featureNumber;     /**< Feature Specification Number */
} rcm_version_info_t;


/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

/*! @name Reset Control Module APIs*/
/*@{*/

/*!
 * @brief Get the version of the RCM module
 * 
 * @param[in]  baseAddr     base address of the RCM module
 * @param[out] versionInfo  Device Version Number
 */
void  RCM_HAL_GetVersion(const RCM_Type* const baseAddr, rcm_version_info_t* const versionInfo);

/*!
 * @brief Gets the reset source status
 *
 * This function gets the current reset source status for a specified source.
 *
 * @param[in] baseAddr     Register base address of RCM
 * @param[in] srcName      reset source name
 * @return status          true or false for specified reset source
 */
bool RCM_HAL_GetSrcStatusCmd(const RCM_Type* const baseAddr, const rcm_source_names_t srcName);

/*!
 * @brief Enables/disables a specified system reset interrupt.
 *
 * This function will enable/disable the specified system reset interrupt.
 *
 * @param[in] baseAddr        Register base address of RCM
 * @param[in] resetInterrupt  Reset source name
 * @param[in] enable          true or false for the specified reset interrupt
 */
void RCM_HAL_SetResetIntCmd(RCM_Type* const baseAddr, const rcm_source_names_t resetInterrupt, const bool enable);


/*!
 * @brief Enables/disables all system reset interrupts.
 *
 * This function  enables/disables all system reset interrupts.
 *
 * @param[in] baseAddr     Register base address of RCM
 * @param[in] enable       enable or disable the filter in stop mode
 */
static inline void RCM_HAL_SetAllResetIntCmd(RCM_Type* const baseAddr, const bool enable)
{
    uint32_t globalIntEnableVal = (enable == true) ? 1U:0U;
    BITBAND_ACCESS32(&(baseAddr->SRIE), RCM_SRIE_GIE_SHIFT) = globalIntEnableVal;
}

/*!
 * @brief Gets the sticky reset source status.
 *
 * This function gets the current reset source status that have not been cleared
 * by software for a specified source.
 *
 * @param[in] baseAddr     Register base address of RCM
 * @param[in] srcName      reset source name
 * @return status          true or false for specified reset source
 */
bool RCM_HAL_GetStickySrcStatusCmd(const RCM_Type* const baseAddr, const rcm_source_names_t srcName);

/*!
 * @brief Clear the sticky reset source status.
 *
 * This function clears all the sticky system reset flags.
 *
 * @param[in] baseAddr     Register base address of RCM
 */
void RCM_HAL_ClearStickySrcStatus(RCM_Type* const baseAddr);

/*!
 * @brief Sets the reset pin filter in stop mode.
 *
 * This function  sets the reset pin filter enable setting in stop mode.
 *
 * @param[in] baseAddr     Register base address of RCM
 * @param[in] enable       enable or disable the filter in stop mode
 */
static inline void RCM_HAL_SetFilterStopModeCmd(RCM_Type* const baseAddr, const bool enable)
{
    BITBAND_ACCESS32(&(baseAddr->RPC), RCM_RPC_RSTFLTSS_SHIFT) = enable;
}

/*!
 * @brief Gets the reset pin filter in stop mode.
 *
 * This function gets the reset pin filter enable setting in stop mode.
 *
 * @param[in] baseAddr  Register base address of RCM
 * @return enable       true/false to enable or disable the filter in stop mode
 */
static inline bool RCM_HAL_GetFilterStopModeCmd(const RCM_Type* const baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->RPC), RCM_RPC_RSTFLTSS_SHIFT);
}

/*!
 * @brief Sets the reset pin filter in run and wait mode.
 *
 * This function sets the reset pin filter enable setting in run/wait mode.
 *
 * @param[in] baseAddr    Register base address of RCM
 * @param[in] mode        to be set for reset filter in run/wait mode
 */
static inline void RCM_HAL_SetFilterRunWaitMode(RCM_Type* const baseAddr, const rcm_filter_run_wait_modes_t mode)
{
    uint32_t regValue = baseAddr->RPC;
    regValue &= ~(RCM_RPC_RSTFLTSRW_MASK);
    regValue |= RCM_RPC_RSTFLTSRW(mode);
    baseAddr->RPC = regValue;
}

/*!
 * @brief Gets the reset pin filter for stop mode.
 *
 * This function gets the reset pin filter enable setting for stop mode.
 *
 * @param[in] baseAddr  Register base address of RCM
 * @return mode  for reset filter in run/wait mode
 */
static inline rcm_filter_run_wait_modes_t RCM_HAL_GetFilterRunWaitMode(const RCM_Type* const baseAddr)
{
    uint32_t regValue = baseAddr->RPC;
    regValue = (regValue & RCM_RPC_RSTFLTSRW_MASK) >> RCM_RPC_RSTFLTSRW_SHIFT;
    return (rcm_filter_run_wait_modes_t)regValue;
}

/*!
 * @brief Sets the reset pin filter width.
 *
 * This function sets the reset pin filter width.
 *
 * @param[in] baseAddr     Register base address of RCM
 * @param[in] width        to be set for reset filter width
 */
static inline void RCM_HAL_SetFilterWidth(RCM_Type* const baseAddr, const uint32_t width)
{
    uint32_t regValue = baseAddr->RPC;
    regValue &= ~(RCM_RPC_RSTFLTSEL_MASK);
    regValue |= RCM_RPC_RSTFLTSEL(width);
    baseAddr->RPC = regValue;
}

/*!
 * @brief Gets the reset pin filter for stop mode.
 *
 * This function gets the reset pin filter width.
 *
 * @param[in] baseAddr     Register base address of RCM
 * @return width reset filter width
 */
static inline uint32_t RCM_HAL_GetFilterWidth(const RCM_Type* const baseAddr)
{
    uint32_t regValue = baseAddr->RPC;
    regValue = (regValue & RCM_RPC_RSTFLTSEL_MASK) >> RCM_RPC_RSTFLTSEL_SHIFT;
    return (uint32_t)regValue;
}

/*!
 * @brief Sets reset delay time.
 *
 * This function configures the maximum reset delay time from when the interrupt is asserted.
 *
 * @param[in] baseAddr    Register base address of RCM
 * @param[in] value       Reset delay time
 */
static inline void RCM_HAL_SetResetDelayTimeValue(RCM_Type* const baseAddr,
                                                  const rcm_reset_delay_time_t value)
{
    uint32_t regValue = baseAddr->SRIE;
    regValue &= ~(RCM_SRIE_DELAY_MASK);
    regValue |= RCM_SRIE_DELAY(value);
    baseAddr->SRIE = regValue;
}

/*@}*/

#if defined(__cplusplus)
}
#endif /* __cplusplus*/

/*! @}*/

#endif /* __FSL_RCM_HAL_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/

