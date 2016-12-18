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

#include "fsl_pins_driver.h"
#include <stdint.h>

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : Pins_DRV_Init
 * Description   : This function configures the pins with the options provided
 * in the provided structure.
 *
 *END**************************************************************************/
void Pins_DRV_Init(const uint32_t pin_count, const pin_settings_config_t config[])
{
    uint32_t i;
    for (i = 0U; i < pin_count; i++) {
#if FSL_FEATURE_PORT_HAS_PULL_SELECTION
        PORT_HAL_SetPullSel(          config[i].base, config[i].pinPortIdx, config[i].pullConfig);
#endif
#if FSL_FEATURE_PORT_HAS_SLEW_RATE
        PORT_HAL_SetSlewRateMode(     config[i].base, config[i].pinPortIdx, config[i].rateSelect);
#endif
#if FSL_FEATURE_PORT_HAS_PASSIVE_FILTER
        PORT_HAL_SetPassiveFilterMode(config[i].base, config[i].pinPortIdx, config[i].passiveFilter);
#endif
#if FSL_FEATURE_PORT_HAS_OPEN_DRAIN
        PORT_HAL_SetOpenDrainMode(    config[i].base, config[i].pinPortIdx, config[i].openDrain);
#endif
#if FSL_FEATURE_PORT_HAS_DRIVE_STRENGTH
        PORT_HAL_SetDriveStrengthMode(config[i].base, config[i].pinPortIdx, config[i].driveSelect);
#endif
        PORT_HAL_SetMuxModeSel(       config[i].base, config[i].pinPortIdx, config[i].mux);
#if FSL_FEATURE_PORT_HAS_PIN_CONTROL_LOCK
        PORT_HAL_SetPinCtrlLockMode(  config[i].base, config[i].pinPortIdx, config[i].pinLock);
#endif
#if FSL_FEATURE_PORT_HAS_DIGITAL_FILTER
        PORT_HAL_SetPinIntSel(        config[i].base, config[i].pinPortIdx, config[i].intConfig);
        if(config[i].clearIntFlag){
            PORT_HAL_ClearPinIntFlagCmd(  config[i].base, config[i].pinPortIdx);
        }
#endif
        if (PORT_MUX_AS_GPIO == config[i].mux)
        {
            switch(config[i].direction)
            {
                case GPIO_INPUT_DIRECTION:
                    BITBAND_ACCESS32(&(config[i].gpioBase->PDDR), config[i].pinPortIdx) = 0U;
                    break;
                case GPIO_OUTPUT_DIRECTION:
                    BITBAND_ACCESS32(&(config[i].gpioBase->PDDR), config[i].pinPortIdx) = 1U;
                    break;
                case GPIO_UNSPECIFIED_DIRECTION:
                    /* pass-through */
                default:
                    /* nothing to configure */
                    break;
            }
        }
    }
}


/******************************************************************************
 * EOF
 *****************************************************************************/
