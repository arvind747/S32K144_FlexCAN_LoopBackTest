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

#ifndef __FSL_TRGMUX_DRIVER_H__
#define __FSL_TRGMUX_DRIVER_H__

#include "fsl_trgmux_hal.h"

/*! @file */

/*!
 * @addtogroup trgmux_driver
 * @{
 */
 
/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief Table of base addresses for TRGMUX instances. */
extern TRGMUX_Type * const g_trgmuxBase[TRGMUX_INSTANCE_COUNT];

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief Source trigger to target module configuration structure for TRGMUX driver.
 *
 * Use an instance of this structure to define in trgmux IP a link between a in (trigger source) to an out (target module).
 * This structure is used by the user configuration structure
 */
typedef struct
{
    trgmux_trigger_source_t     triggerSource;          /*!< selects one of the trgmux 64 available trigger sources */
    trgmux_target_module_t      targetModule;           /*!< selects one of the trgmux 63 available target modules  */
    bool                        lockTargetModuleReg;    /*!< if true, the LOCK bit of the target module register will
                                                             be set by TRGMUX_DRV_INIT() function after configuring the
                                                             mapping between in (source trigger) and out (target module)
                                                             defined in the current entry */
} trgmux_inout_mapping_config_t;


/*!
 * @brief User configuration structure for the TRGMUX driver.
 *
 * Use an instance of this structure with the TRGMUX_DRV_Init()function. This enables configuration of the user
 * defined mappings between inputs (source triggers) and outputs (target modules) in the TRGMUX peripheral with
 * a single function call.
 * @internal gui name="TRGMUX configuration" id="Configuration"
 */
typedef struct
{
    uint8_t numInOutMappingConfigs;                            /*!< number of in to out mappings defined in trgmux configuration */
    const trgmux_inout_mapping_config_t* inOutMappingConfig;   /*!< pointer to array of in-out mappings structures */
} trgmux_user_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif


/*!
 * @brief Initializes an TRGMUX instance for operation.
 *
 * This function sets first the source trigger of all TRGMUX target modules to their default values,
 * then loops through all in-out mappings defined in the user configuration structure and configures
 * the user defined target modules with the corresponding source triggers.
 * This example shows how to set up the trgmux_user_config_t parameters and how to call the
 * TRGMUX_DRV_Init function by passing in the needed parameters:
   @code
    trgmux_user_config_t             trgmuxConfig;
    trgmux_inout_mapping_config_t    trgmuxInoutMappingConfig[] =
    {
        {TRGMUX_TRIG_SOURCE_TRGMUX_IN9,     TRGMUX_TARGET_MODULE_DMA_CH0,     false},
        {TRGMUX_TRIG_SOURCE_FTM1_EXT_TRIG,  TRGMUX_TARGET_MODULE_TRGMUX_OUT4, true}
    };

    trgmuxConfig.numInOutMappingConfigs = 2;
    trgmuxConfig.inOutMappingConfig     = trgmuxInoutMappingConfig;

    TRGMUX_DRV_Init(instance, &trgmuxConfig);
    @endcode
 *
 * @param instance              The TRGMUX instance number.
 * @param trgmuxUserConfig      The user configuration structure of type trgmux_user_config_t. The user
 *                              populates the members of this structure and passes the pointer of this
 *                              structure to this function.
 * @return                      An error code or TRGMUX_STATUS_SUCCESS.
 */
trgmux_status_t TRGMUX_DRV_Init(
                                uint32_t                        instance,
                                const trgmux_user_config_t *    trgmuxUserConfig
                               );

/*!
 * @brief Reset the values of source triggers for all target modules to the default ones.
 *
 *
 * @param instance              The TRGMUX instance number.
 * @return                      TRGMUX_STATUS_SUCCESS.
 */
trgmux_status_t TRGMUX_DRV_Deinit(
                                   uint32_t  instance
                                 );

/*!
 * @brief Configures a source trigger for a target module.
 *
 * This function configures in TRGMUX IP the link between an input (source trigger) and
 * and output (target module).
 *
 * @param instance          The TRGMUX instance number.
 * @param triggerSource     One of the values in the trgmux_trigger_source_t enumeration
 * @param targetModule      One of the values in the trgmux_target_module_t enumeration
 * @return                  TRGMUX_STATUS_SUCCESS. */
trgmux_status_t TRGMUX_DRV_SetTrigSourceForTargetModule(
                                                        uint32_t                instance,
                                                        trgmux_trigger_source_t triggerSource,
                                                        trgmux_target_module_t  targetModule
                                                       );

/*!
 * @brief Gets the source trigger configured for a target module.
 *
 * This function reads from the TRGMUX IP the input (source trigger) that is linked
 * to a given output (target module).
 *
 * @param instance          The TRGMUX instance number.
 * @param targetModule      One of the values in the trgmux_target_module_t enumeration
 * @return                  Enum value corresponding to the trigger source configured
 *                          for the given target module
 */
trgmux_trigger_source_t TRGMUX_DRV_GetTrigSourceForTargetModule(
                                                        uint32_t                instance,
                                                        trgmux_target_module_t  targetModule
                                                               );

/*!
 * @brief Locks the TRGMUX register of a target module.
 *
 * This function sets to 0x1 the LK bit of the TRGMUX register containing SEL bitfield for
 * a given target module. Please note that some TRGMUX registers can contain up to 4
 * SEL bitfields, meaning that these registers can be used to configure up to 4 target
 * modules independently. Because of the fact that the LK bit is only one per register,
 * the first target module that locks the register will lock it for the other 3 target
 * modules too.
 *
 * @param instance          The TRGMUX instance number.
 * @param targetModule      One of the values in the trgmux_target_module_t enumeration\
 * @return                  TRGMUX_STATUS_SUCCESS.
 */
trgmux_status_t TRGMUX_DRV_SetLockForTargetModule(
                                                    uint32_t                instance,
                                                    trgmux_target_module_t  targetModule
                                                 );

/*!
 * @brief Get the Lock bit status of the TRGMUX register of a target module.
 *
 * This function gets the value of the LK bit of the TRGMUX register containing SEL bitfield for
 * a given target module.
 *
 * @param instance          The TRGMUX instance number.
 * @param targetModule      One of the values in the trgmux_target_module_t enumeration
 * @return                  true or false, depending if the targetModule register is locked
 */
bool  TRGMUX_DRV_GetLockForTargetModule(
                                        uint32_t                instance,
                                        trgmux_target_module_t  targetModule
                                       );

/*@}*/

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __FSL_TRGMUX_DRIVER_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/
