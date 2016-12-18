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

#include "fsl_adc_hal.h"

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_HAL_Init
 * Description   : This function initializes the ADC instance to a known
 * state (the register are written with their reset values from the Reference 
 * Manual).
 *
 *END**************************************************************************/
void ADC_HAL_Init(ADC_Type* const baseAddr)
{
    baseAddr->SC1[ 0U] = ADC_SC1_ADCH(ADC_INPUTCHAN_DISABLED) | ADC_SC1_AIEN(0x00U);
    baseAddr->SC1[ 1U] = ADC_SC1_ADCH(ADC_INPUTCHAN_DISABLED) | ADC_SC1_AIEN(0x00U);
    baseAddr->SC1[ 2U] = ADC_SC1_ADCH(ADC_INPUTCHAN_DISABLED) | ADC_SC1_AIEN(0x00U);
    baseAddr->SC1[ 3U] = ADC_SC1_ADCH(ADC_INPUTCHAN_DISABLED) | ADC_SC1_AIEN(0x00U);
    baseAddr->SC1[ 4U] = ADC_SC1_ADCH(ADC_INPUTCHAN_DISABLED) | ADC_SC1_AIEN(0x00U);
    baseAddr->SC1[ 5U] = ADC_SC1_ADCH(ADC_INPUTCHAN_DISABLED) | ADC_SC1_AIEN(0x00U);
    baseAddr->SC1[ 6U] = ADC_SC1_ADCH(ADC_INPUTCHAN_DISABLED) | ADC_SC1_AIEN(0x00U);
    baseAddr->SC1[ 7U] = ADC_SC1_ADCH(ADC_INPUTCHAN_DISABLED) | ADC_SC1_AIEN(0x00U);
    baseAddr->SC1[ 8U] = ADC_SC1_ADCH(ADC_INPUTCHAN_DISABLED) | ADC_SC1_AIEN(0x00U);
    baseAddr->SC1[ 9U] = ADC_SC1_ADCH(ADC_INPUTCHAN_DISABLED) | ADC_SC1_AIEN(0x00U);
    baseAddr->SC1[10U] = ADC_SC1_ADCH(ADC_INPUTCHAN_DISABLED) | ADC_SC1_AIEN(0x00U);
    baseAddr->SC1[11U] = ADC_SC1_ADCH(ADC_INPUTCHAN_DISABLED) | ADC_SC1_AIEN(0x00U);
    baseAddr->SC1[12U] = ADC_SC1_ADCH(ADC_INPUTCHAN_DISABLED) | ADC_SC1_AIEN(0x00U);
    baseAddr->SC1[13U] = ADC_SC1_ADCH(ADC_INPUTCHAN_DISABLED) | ADC_SC1_AIEN(0x00U);
    baseAddr->SC1[14U] = ADC_SC1_ADCH(ADC_INPUTCHAN_DISABLED) | ADC_SC1_AIEN(0x00U);
    baseAddr->SC1[15U] = ADC_SC1_ADCH(ADC_INPUTCHAN_DISABLED) | ADC_SC1_AIEN(0x00U);
    baseAddr->CFG1 = ADC_CFG1_ADICLK(ADC_CLK_ALT_1) | ADC_CFG1_MODE(ADC_RESOLUTION_8BIT) | ADC_CFG1_ADIV(ADC_CLK_DIVIDE_1);
    baseAddr->CFG2 = ADC_CFG2_SMPLTS(0x0CU);
    baseAddr->CV[0U] = ADC_CV_CV(0U);
    baseAddr->CV[1U] = ADC_CV_CV(0U);
    baseAddr->SC2 = ADC_SC2_REFSEL(ADC_VOLTAGEREF_VREF) | ADC_SC2_DMAEN(0x00U) | ADC_SC2_ACREN(0x00U) | ADC_SC2_ACFGT(0x00U) | ADC_SC2_ACFE(0x00U) | ADC_SC2_ADTRG(0x00U);
    baseAddr->SC3 = ADC_SC3_AVGS(ADC_AVERAGE_4) | ADC_SC3_AVGE(0x00U) | ADC_SC3_ADCO(0x00U) | ADC_SC3_CAL(0x00U);
    baseAddr->USR_OFS = ADC_USR_OFS_USR_OFS(0U);
    baseAddr->UG = ADC_UG_UG(4U);
    
}
/*******************************************************************************
 * EOF
 ******************************************************************************/
