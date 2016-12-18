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

#include <stdint.h>
#include <stdbool.h>
#include "fsl_device_registers.h"
#include "fsl_cmp_hal.h"

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : CMP_HAL_Init
 * Description   : This function initializes the CMP instance to a known
 * state (the register are written with their reset values from the Reference 
 * Manual).
 *
 *END**************************************************************************/
void CMP_HAL_Init(CMP_Type* const baseAddr)
{
	baseAddr->C0 = 	CMP_C0_DMAEN(0U) | CMP_C0_IER(0U) | CMP_C0_IEF(0U) | CMP_C0_CFR(1U) | 
			CMP_C0_CFF(1U) | CMP_C0_FPR(0U) | CMP_C0_SE(0U) | CMP_C0_WE(0U) | 
			CMP_C0_PMODE(0U) | CMP_C0_INVT(0U) | CMP_C0_COS(0U) | CMP_C0_OPE(0U) | 
			CMP_C0_EN(0U) |CMP_C0_FILTER_CNT(0U) | CMP_C0_OFFSET(0U) | CMP_C0_HYSTCTR(0U);
        
	baseAddr->C1 = 	 CMP_C1_INPSEL(0U) | CMP_C1_INNSEL(0U) | CMP_C1_CHN7(0U) |
			CMP_C1_CHN6(0U) | CMP_C1_CHN5(0U) | CMP_C1_CHN4(0U) | CMP_C1_CHN3(0U) | CMP_C1_CHN2(0U) |
			CMP_C1_CHN1(0U) | CMP_C1_CHN0(0U) | CMP_C1_DACEN(0U) | CMP_C1_VRSEL(0U) |
			CMP_C1_PSEL(0U) | CMP_C1_MSEL(0U) |  CMP_C1_VOSEL(0U);
        
        baseAddr->C2 =	CMP_C2_RRE(0U) | CMP_C2_RRIE(0U) | CMP_C2_FXMP(0U) | CMP_C2_FXMXCH(0U) | CMP_C2_CH7F(1U) |
			CMP_C2_CH6F(1U) | CMP_C2_CH5F(1U) | CMP_C2_CH4F(1U) | CMP_C2_CH3F(1U) | CMP_C2_CH2F(1U) | 
			CMP_C2_CH1F(1U) | CMP_C2_CH0F(1U) | CMP_C2_NSAM(0U) | CMP_C2_NSAM(0U) | CMP_C2_INITMOD(0U) |
			CMP_C2_ACOn(0U);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CMP_HAL_SetFunctionalMode
 * Description   : This function set CMP functional mode based on configuration
 * structure. If values for filter count or filter period are specific for selected mode
 * the value from input structure will be ignored.
 *   
 *   FUNCTIONAL MODE                 | FILTER COUNT  | FILTER PERIOD
 *   ----------------------------------------------------------------
 *   CMP_DISABLED                    |    IGNORED    |    IGNORED           
 *   CMP_CONTINUOUS                  |    IGNORED    |    IGNORED
 *   CMP_SAMPLED_NONFILTRED_INT_CLK  |    IGNORED    |    IGNORED
 *   CMP_SAMPLED_NONFILTRED_EXT_CLK  |    IGNORED    |     USED
 *   CMP_SAMPLED_FILTRED_INT_CLK     |     USED      |    IGNORED
 *   CMP_SAMPLED_FILTRED_EXT_CLK     |     USED      |     USED 
 *   CMP_WINDOWED                    |    IGNORED    |    IGNORED
 *   CMP_WINDOWED_RESAMPLED          |    IGNORED    |     USED
 *   CMP_WINDOWED_FILTRED            |     USED      |     USED 
 *
 *END**************************************************************************/
 void CMP_HAL_SetFunctionalMode(CMP_Type* const baseAddr, cmp_mode_t mode, uint8_t filter_sample_count, uint8_t filter_sample_period)
 {
	uint32_t tmp = baseAddr->C0;
	tmp = tmp & ( ~(CMP_C0_SE_MASK & CMP_C0_FPR_MASK & CMP_C0_FILTER_CNT_MASK & CMP_C0_EN_MASK));
	baseAddr->C0 = tmp;
        switch(mode)
        {
        case CMP_DISABLED:
            break;
        case CMP_CONTINUOUS:
            tmp |= CMP_C0_EN(1U);
            tmp |= CMP_C0_WE(0U);
            tmp |= CMP_C0_FPR(0U);
            break;
        case CMP_SAMPLED_NONFILTRED_INT_CLK:
            tmp |= CMP_C0_EN(1U);
            tmp |= CMP_C0_WE(0U);
            tmp |= CMP_C0_SE(1U);
            tmp |= CMP_C0_FILTER_CNT(1U);
            break;
        case CMP_SAMPLED_NONFILTRED_EXT_CLK:
            tmp |= CMP_C0_EN(1U);
            tmp |= CMP_C0_WE(0U);
            tmp |= CMP_C0_FILTER_CNT(filter_sample_period);
            break;
        case CMP_SAMPLED_FILTRED_INT_CLK:
            tmp |= CMP_C0_EN(1U);
            tmp |= CMP_C0_WE(0U);
            tmp |= CMP_C0_SE(1U);
            tmp |= CMP_C0_FILTER_CNT(filter_sample_count);
            break;
        case CMP_SAMPLED_FILTRED_EXT_CLK:
            tmp |= CMP_C0_EN(1U);
            tmp |= CMP_C0_WE(0U);
            tmp |= CMP_C0_FILTER_CNT(filter_sample_count);
            tmp |= CMP_C0_FPR(filter_sample_period);
            break;
        case CMP_WINDOWED:
            tmp |= CMP_C0_EN(1U);
            tmp |= CMP_C0_WE(1U);
            tmp |= CMP_C0_FILTER_CNT(0U);
            break;
        case CMP_WINDOWED_RESAMPLED:
            tmp |= CMP_C0_EN(1U);
            tmp |= CMP_C0_WE(1U);
            tmp |= CMP_C0_FILTER_CNT(1U);
            tmp |= CMP_C0_FPR(filter_sample_period);
            break;
        case CMP_WINDOWED_FILTRED:
            tmp |= CMP_C0_EN(1U);
            tmp |= CMP_C0_WE(1U);
            tmp |= CMP_C0_FILTER_CNT(filter_sample_count);
            tmp |= CMP_C0_FPR(filter_sample_period);
            break;
        }
        baseAddr->C0 = tmp;
 }

/*FUNCTION**********************************************************************
 *
 * Function Name : CMP_HAL_GetFunctionalMode
 * Description   : This function return current CMP functional mode. CMP module cand be used in
 * one of this modes: CMP_DISABLED , CMP_CONTINUOUS, CMP_SAMPLED_NONFILTRED_INT_CLK, 
 * CMP_SAMPLED_NONFILTRED_EXT_CLK, CMP_SAMPLED_FILTRED_INT_CLK, CMP_SAMPLED_FILTRED_EXT_CLK,
 * CMP_WINDOWED, CMP_WINDOWED_RESAMPLED, CMP_WINDOWED_FILTRED
 *
 *END**************************************************************************/

 cmp_mode_t CMP_HAL_GetFunctionalMode(CMP_Type* const baseAddr)
{
    uint8_t state;
    uint32_t tmp = baseAddr->C0;
    cmp_mode_t mode;
    state = BITBAND_ACCESS32(&(baseAddr->C0), CMP_C0_EN_SHIFT) << 2;
    state |= BITBAND_ACCESS32(&(baseAddr->C0), CMP_C0_WE_SHIFT) << 1;
    state |= BITBAND_ACCESS32(&(baseAddr->C0), CMP_C0_SE_SHIFT);
    uint8_t filter_sample_period = (tmp & CMP_C0_FPR_MASK) >> CMP_C0_FPR_SHIFT;
    uint8_t filter_sample_count = (tmp & CMP_C0_FILTER_CNT_MASK) >> CMP_C0_FILTER_CNT_SHIFT;
    if ((state & 0x4) == 0)
        mode = CMP_DISABLED;
    switch(state)
    {
    case 4: 
        if ((filter_sample_period == 0) || (filter_sample_count == 0))
            mode = CMP_CONTINUOUS;
        if ((filter_sample_period > 0) || (filter_sample_count == 1))
            mode = CMP_SAMPLED_NONFILTRED_EXT_CLK;
        if ((filter_sample_period > 4) || (filter_sample_count > 1))
            mode = CMP_SAMPLED_FILTRED_EXT_CLK;
        break;
    case 5:
        if (filter_sample_count == 1)
            mode = CMP_SAMPLED_NONFILTRED_INT_CLK;
        if (filter_sample_count > 1)
            mode = CMP_SAMPLED_FILTRED_INT_CLK;
        break;
    case 6 :
        if ((filter_sample_period == 0) || (filter_sample_count == 0))
            mode = CMP_WINDOWED;
        if ((filter_sample_count == 1) || (filter_sample_period > 1))
            mode = CMP_WINDOWED_RESAMPLED;
        if ((filter_sample_count > 1) || (filter_sample_period > 1))
            mode = CMP_WINDOWED_FILTRED;
    }
    return mode;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
