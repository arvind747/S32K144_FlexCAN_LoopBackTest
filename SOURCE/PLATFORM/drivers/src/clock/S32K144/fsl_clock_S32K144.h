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

#if !defined(__FSL_CLOCK_S32K144_H__)
#define __FSL_CLOCK_S32K144_H__

#include "fsl_scg_hal.h"
#include "fsl_pcc_hal.h"
#include "fsl_sim_hal.h"
#include "fsl_smc_hal.h"  /* Required for SCG alternative clock usage */

/*! @file clock_manager_S32K144.h */

/*!
 * @ingroup clock_manager
 * @defgroup clock_manager_s32k144
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

/*!
 * @brief Configures SCG module.
 *
 * This function configures the SCG module according to the
 * configuration.
 *
 * @param scgConfig   Pointer to the configuration structure.
 * @return Status of module initialization
 */
scg_status_t CLOCK_SYS_SetScgConfiguration(const scg_config_t *scgConfig);

/*!
 * @brief Configures PCC module.
 *
 * This function configures the PCC module according to the
 * configuration.
 *
 * @param peripheralClockConfig   Pointer to the configuration structure.
 */
void CLOCK_SYS_SetPccConfiguration(const pcc_config_t *peripheralClockConfig);

/*!
 * @brief Configures SIM module.
 *
 * This function configures the SIM module according to the
 * configuration.
 *
 * @param simClockConfig   Pointer to the configuration structure.
 */
void CLOCK_SYS_SetSimConfiguration(const sim_clock_config_t *simClockConfig);

#if defined(__cplusplus)
}
#endif /* __cplusplus*/

/*! @}*/

#endif /* __FSL_CLOCK_S32K144_H__ */
/*******************************************************************************
 * EOF
 ******************************************************************************/

