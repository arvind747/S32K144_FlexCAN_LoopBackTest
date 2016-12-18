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

#include "fsl_lptmr_hal.h"

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : LPTMR_HAL_Init
 * Description   : This function configures the LPTMR instance to reset value.
 *
 *END**************************************************************************/
void LPTMR_HAL_Init(LPTMR_Type* const base)
{
    /* First, disable the module so we can write the registers */
    BITBAND_ACCESS32(&(base->CSR), LPTMR_CSR_TEN_SHIFT) = 0u;
    base->CSR = LPTMR_CSR_TEN(0u) | \
                LPTMR_CSR_TMS(0u) | \
                LPTMR_CSR_TFC(0u) | \
                LPTMR_CSR_TPP(0u) | \
                LPTMR_CSR_TPS(0u) | \
                LPTMR_CSR_TIE(0u) | \
                LPTMR_CSR_TCF(0u) | \
                LPTMR_CSR_TDRE(0u);

    base->PSR = LPTMR_PSR_PCS(0u) | \
                LPTMR_PSR_PBYP(0u) | \
                LPTMR_PSR_PRESCALE(0u);

    base->CMR = LPTMR_CMR_COMPARE(0u);
}
/*******************************************************************************
 * EOF
 ******************************************************************************/
