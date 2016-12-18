/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
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

/*!
 * @file fsl_wdog_driver.h
 */
 
#ifndef FSL_WDOG_DRIVER_H
#define FSL_WDOG_DRIVER_H

#include <stddef.h>
#include <stdbool.h>
#include "fsl_device_registers.h"
#include "fsl_wdog_hal.h"
#include "fsl_interrupt_manager.h"
#include "fsl_clock_manager.h"


/*!
 * @defgroup wdog_drv WDOG Driver
 * @ingroup wdog
 * @brief Watchdog Timer Peripheral Driver.
 * @addtogroup wdog_drv
 * @{
 */

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* Table of base addresses for WDOG instances. */
extern WDOG_Type * const g_wdogBase[];

/* Table to save WDOG IRQ enumeration numbers defined in the CMSIS header file. */
extern const IRQn_Type g_wdogIrqId[WDOG_INSTANCE_COUNT];

/*******************************************************************************
 * Enumerations.
 ******************************************************************************/
 
/*!
 * @brief WDOG driver status codes 
 * Implements : wdog_status_t_Class
 */
typedef enum _wdog_status {
    WDOG_STATUS_SUCCESS             = 0U,  /*!< Operation was successful */
    WDOG_STATUS_FAIL                = 1U,  /*!< Operation failed */
    WDOG_STATUS_NULL_PARAM          = 2U,  /*!< Operation failed due to a null
                                           parameter */
} wdog_status_t;
 
 /*******************************************************************************
 * Definitions
 ******************************************************************************/
 

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @name WDOG Driver API
 * @{
 */

#if defined(__cplusplus)
extern "C" {
#endif


/*!
 * @brief Initializes the WDOG driver.
 *
 * @param[in] instance  WDOG peripheral instance number
 * @param[in] userConfigPtr    pointer to the WDOG user configuration structure
 * @return operation status
 *        - WDOG_STATUS_SUCCESS:    Operation was successful.
 *        - WDOG_STATUS_FAIL:       Operation failed.
 *        - WDOG_STATUS_NULL_PARAM: Operation failed due to a null parameter.
 */
wdog_status_t WDOG_DRV_Init(uint32_t instance, 
                            const wdog_user_config_t *userConfigPtr);
/*!
 * @brief De-initializes the WDOG driver
 *
 * @param[in] instance  WDOG peripheral instance number
 * @return operation status
 *        - WDOG_STATUS_SUCCESS:    Operation was successful.
 *        - WDOG_STATUS_FAIL:       Operation failed.
 */
wdog_status_t WDOG_DRV_Deinit(uint32_t instance);

/*!
 * @brief Gets the current configuration of the WDOG.
 *
 * @param[in] instance  WDOG peripheral instance number
 * @param[out] config  the current configuration
 * @return operation status
 *        - WDOG_STATUS_SUCCESS:    Operation was successful.
 */
wdog_status_t WDOG_DRV_GetConfig(uint32_t instance, wdog_user_config_t *config);

/*!
 * @brief  Enables/Disables the WDOG timeout interrupt and sets a function to be
 * called when a timeout interrupt is received, before reset.
 *
 * @param[in] instance  WDOG peripheral instance number
 * @param[in] enable  enable/disable interrupt
 * @param[in] handler   timeout interrupt handler
 * @return operation status
 *        - WDOG_STATUS_SUCCESS:    Operation was successful.
 *        - WDOG_STATUS_FAIL:       Operation failed.
 */
wdog_status_t WDOG_DRV_SetInt(uint32_t instance, bool enable, void (*handler)(void));

/*!
 * @brief Refreshes the WDOG counter.
 *
 * @param[in] instance  WDOG peripheral instance number
 * @return operation status
 *        - WDOG_STATUS_SUCCESS:    Operation was successful.
 */
wdog_status_t WDOG_DRV_Trigger(uint32_t instance);



/*! @}*/
#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* FSL_WDOG_DRIVER_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
