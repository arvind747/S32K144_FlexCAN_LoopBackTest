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
#include "fsl_trgmux_hal.h"

/*******************************************************************************
 * Definitions
 *******************************************************************************/
/* Number of possible outputs (target module) for TRGMUX IP */
#define TRGMUX_NUM_TARGET_MODULES            ((uint8_t)(sizeof(s_trgmuxTargetModule)/sizeof(uint8_t)))
/* Number of SEL bitfields in one TRGMUX register */
#define TRGMUX_NUM_SEL_BITFIELDS_PER_REG     (4U)

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : TRGMUX_HAL_Init
 * Description   : Initialize TRGMUX peripheral to workable state.
 *
 *END**************************************************************************/
void TRGMUX_HAL_Init(TRGMUX_Type * base)
{
    /* Constant array storing the value of all TRGMUX output(target module) identifiers */
    static const trgmux_target_module_t s_trgmuxTargetModule[] =
    {
        TRGMUX_TARGET_MODULE_DMA_CH0,
        TRGMUX_TARGET_MODULE_DMA_CH1,
        TRGMUX_TARGET_MODULE_DMA_CH2,
        TRGMUX_TARGET_MODULE_DMA_CH3,
        TRGMUX_TARGET_MODULE_TRGMUX_OUT0,
        TRGMUX_TARGET_MODULE_TRGMUX_OUT1,
        TRGMUX_TARGET_MODULE_TRGMUX_OUT2,
        TRGMUX_TARGET_MODULE_TRGMUX_OUT3,
        TRGMUX_TARGET_MODULE_TRGMUX_OUT4,
        TRGMUX_TARGET_MODULE_TRGMUX_OUT5,
        TRGMUX_TARGET_MODULE_TRGMUX_OUT6,
        TRGMUX_TARGET_MODULE_TRGMUX_OUT7,
        TRGMUX_TARGET_MODULE_ADC0_ADHWT_OR0,
        TRGMUX_TARGET_MODULE_ADC0_ADHWT_OR1,
        TRGMUX_TARGET_MODULE_ADC0_ADHWT_OR2,
        TRGMUX_TARGET_MODULE_ADC0_ADHWT_OR3,
        TRGMUX_TARGET_MODULE_ADC1_ADHWT_OR0,
        TRGMUX_TARGET_MODULE_ADC1_ADHWT_OR1,
        TRGMUX_TARGET_MODULE_ADC1_ADHWT_OR2,
        TRGMUX_TARGET_MODULE_ADC1_ADHWT_OR3,
        TRGMUX_TARGET_MODULE_CMP0_SAMPLE,
        TRGMUX_TARGET_MODULE_FTM0_HWTRIG,
        TRGMUX_TARGET_MODULE_FTM0_FAULT0,
        TRGMUX_TARGET_MODULE_FTM0_FAULT1,
        TRGMUX_TARGET_MODULE_FTM0_FAULT2,
        TRGMUX_TARGET_MODULE_FTM1_HW_TRIG,
        TRGMUX_TARGET_MODULE_FTM1_FAULT0,
        TRGMUX_TARGET_MODULE_FTM1_FAULT1,
        TRGMUX_TARGET_MODULE_FTM1_FAULT2,
        TRGMUX_TARGET_MODULE_FTM2_HW_TRIG,
        TRGMUX_TARGET_MODULE_FTM2_FAULT0,
        TRGMUX_TARGET_MODULE_FTM2_FAULT1,
        TRGMUX_TARGET_MODULE_FTM2_FAULT2,
        TRGMUX_TARGET_MODULE_FTM3_HW_TRIG,
        TRGMUX_TARGET_MODULE_FTM3_FAULT0,
        TRGMUX_TARGET_MODULE_FTM3_FAULT1,
        TRGMUX_TARGET_MODULE_FTM3_FAULT2,
        TRGMUX_TARGET_MODULE_PDB0_TRG_IN,
        TRGMUX_TARGET_MODULE_PDB1_TRG_IN,
        TRGMUX_TARGET_MODULE_FLEXIO_TRG_TIM0,
        TRGMUX_TARGET_MODULE_FLEXIO_TRG_TIM1,
        TRGMUX_TARGET_MODULE_FLEXIO_TRG_TIM2,
        TRGMUX_TARGET_MODULE_FLEXIO_TRG_TIM3,
        TRGMUX_TARGET_MODULE_LPIT_TRG_CH0,
        TRGMUX_TARGET_MODULE_LPIT_TRG_CH1,
        TRGMUX_TARGET_MODULE_LPIT_TRG_CH2,
        TRGMUX_TARGET_MODULE_LPIT_TRG_CH3,
        TRGMUX_TARGET_MODULE_LPUART0_TRG,
        TRGMUX_TARGET_MODULE_LPUART1_TRG,
        TRGMUX_TARGET_MODULE_LPI2C0_TRG,
        TRGMUX_TARGET_MODULE_LPI2C1_TRG,
        TRGMUX_TARGET_MODULE_LPSPI0_TRG,
        TRGMUX_TARGET_MODULE_LPSPI1_TRG,
        TRGMUX_TARGET_MODULE_LPTMR0_ALT0
    };
    uint8_t count, idxTrgmuxRegister, idxSelBitfield;
    /* Set all SEL bitfields of all TRGMUX registers to default value */
    for(count = 0U; count < TRGMUX_NUM_TARGET_MODULES; count++)
    {
        /* Get index of TRGMUX register to update */
        idxTrgmuxRegister = (uint8_t)s_trgmuxTargetModule[count] / TRGMUX_NUM_SEL_BITFIELDS_PER_REG;
        /* Get index of SEL bitfield inside TRGMUX register to update */
        idxSelBitfield = (uint8_t)s_trgmuxTargetModule[count] % TRGMUX_NUM_SEL_BITFIELDS_PER_REG;
        /* Write the TRGMUX register */
        base->TRGMUXn[idxTrgmuxRegister] &= ~(TRGMUX_TRGMUXn_SEL0_MASK << (TRGMUX_TRGMUXn_SEL1_SHIFT * idxSelBitfield));
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : TRGMUX_HAL_SetTrigSourceForTargetModule
 * Description   : Configures one of the 64 source triggers for a given target module .
 *
 *END**************************************************************************/
void TRGMUX_HAL_SetTrigSourceForTargetModule(
                                               TRGMUX_Type *            base,
                                               trgmux_trigger_source_t  triggerSource,
                                               trgmux_target_module_t   targetModule
                                            )
{
    uint8_t idxTrgmuxRegister, idxSelBitfield;
    uint32_t tmpReg;
    /* Get the index of the TRGMUX register that should be updated */
    idxTrgmuxRegister = (uint8_t)targetModule / TRGMUX_NUM_SEL_BITFIELDS_PER_REG;
    /* Get the index of the SEL bitfield inside TRGMUX register that should be updated */
    idxSelBitfield = (uint8_t)targetModule % TRGMUX_NUM_SEL_BITFIELDS_PER_REG;
    /* Read value of entire TRGMUX register in a temp variable */
    tmpReg = base->TRGMUXn[idxTrgmuxRegister];
    /* Clear first the SEL bitfield inside the TRGMUX register */
    tmpReg &= ~(TRGMUX_TRGMUXn_SEL0_MASK << (TRGMUX_TRGMUXn_SEL1_SHIFT * idxSelBitfield));
    /* Configure the SEL bitfield to the desired value */
    tmpReg |=  (((uint8_t)triggerSource) << (TRGMUX_TRGMUXn_SEL1_SHIFT * idxSelBitfield));
    /* Write back the TRGMUX register */
    base->TRGMUXn[idxTrgmuxRegister] = tmpReg;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : TRGMUX_HAL_GetTrigSourceForTargetModule
 * Description   : Returns the source trigger configured for a given target module .
 *
 *END**************************************************************************/
trgmux_trigger_source_t TRGMUX_HAL_GetTrigSourceForTargetModule(
                                                TRGMUX_Type *           base,
                                                trgmux_target_module_t  targetModule
                                                               )
{
    uint8_t idxTrgmuxRegister, idxSelBitfield;
    /* Get the index of the TRGMUX register that should be updated */
    idxTrgmuxRegister = (uint8_t)targetModule / TRGMUX_NUM_SEL_BITFIELDS_PER_REG;
    /* Get the index of the SEL bitfield inside TRGMUX register that should be updated */
    idxSelBitfield = (uint8_t)targetModule % TRGMUX_NUM_SEL_BITFIELDS_PER_REG;
    /* Perform the update operation */
    return (trgmux_trigger_source_t)((base->TRGMUXn[idxTrgmuxRegister] >> (TRGMUX_TRGMUXn_SEL1_SHIFT * idxSelBitfield)) & TRGMUX_TRGMUXn_SEL0_MASK);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : TRGMUX_HAL_SetLockForTargetModule
 * Description   : Sets the Lock bit in the TRGMUX register of a given target module .
 *
 *END**************************************************************************/
void TRGMUX_HAL_SetLockForTargetModule(
                                        TRGMUX_Type *           base,
                                        trgmux_target_module_t  targetModule
                                      )
{
    uint8_t idxTrgmuxRegister;
    /* Get the index of the TRGMUX register that should be updated */
    idxTrgmuxRegister = (uint8_t)targetModule / TRGMUX_NUM_SEL_BITFIELDS_PER_REG;
    /* Perform the update operation */
    BITBAND_ACCESS32(&(base->TRGMUXn[idxTrgmuxRegister]), TRGMUX_TRGMUXn_LK_SHIFT) = 1U;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : TRGMUX_HAL_GetLockForTargetModule
 * Description   : Gets the state of the Lock bit in the TRGMUX register for a given target module .
 *
 *END**************************************************************************/
bool TRGMUX_HAL_GetLockForTargetModule(
                                        TRGMUX_Type *           base,
                                        trgmux_target_module_t  targetModule
                                       )
{
    uint8_t idxTrgmuxRegister;
    /* Get the index of the TRGMUX register that should be updated */
    idxTrgmuxRegister = (uint8_t)targetModule / TRGMUX_NUM_SEL_BITFIELDS_PER_REG;
    /* Perform the update operation */
    return (bool)BITBAND_ACCESS32(&(base->TRGMUXn[idxTrgmuxRegister]), TRGMUX_TRGMUXn_LK_SHIFT);
}

/*******************************************************************************
 * EOF
 *******************************************************************************/
