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

#include "fsl_pcc_hal.h"
#include <stddef.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*! @brief Clock name mappings
 *         Constant array storing the mappings between clock names and peripheral clock control indexes.
 *         If there is no peripheral clock control index for a clock name, then the corresponding value is 
 *         PCC_INVALID_INDEX.
 */
const uint16_t clockNameMappings[] = PCC_CLOCK_NAME_MAPPINGS;

/*******************************************************************************
 * Code
 ******************************************************************************/
 
/*FUNCTION*********************************************************************
 *
 * Function Name : PCC_HAL_SetPeripheralClockConfig
 * Description   : This function sets the peripheral clock configuration
 *
 *END*************************************************************************/
void PCC_HAL_SetPeripheralClockConfig(PCC_Type* const base,
                                          const pcc_config_t* const config)
{
  uint32_t i;
  uint32_t clkGate;
  peripheral_clock_config_t *peripheral_clock_config;

  if ((config != NULL) && (config->peripheralClocks != NULL))
  {
    for (i = 0U; i < config->count; i++) {

      peripheral_clock_config = &config->peripheralClocks[i];

      /* Disable the peripheral clock */
      BITBAND_ACCESS32(&(base->PCCn[clockNameMappings[peripheral_clock_config->clockName]]), PCC_PCCn_CGC_SHIFT) = false;

      /* Clock gate value */
      clkGate = (peripheral_clock_config->clkGate == true) ? 1U : 0U;

      /* Configure the peripheral clock source, the fractional clock divider and the clock gate */
      base->PCCn[clockNameMappings[peripheral_clock_config->clockName]] = PCC_PCCn_PCS(peripheral_clock_config->clkSrc)   |
                     PCC_PCCn_FRAC(peripheral_clock_config->frac)    |
                     PCC_PCCn_PCD(peripheral_clock_config->divider)  |
                     PCC_PCCn_CGC(clkGate);

    }
  }
}

 

/*******************************************************************************
 * EOF
 ******************************************************************************/
