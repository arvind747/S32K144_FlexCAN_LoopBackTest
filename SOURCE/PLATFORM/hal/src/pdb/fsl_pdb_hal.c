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

#include "fsl_pdb_hal.h"


/*FUNCTION*********************************************************************
 *
 * Function Name : PDB_HAL_Init
 * Description   : Reset PDB's registers to a known state. This state is
 * defined in Reference Manual, which is power on reset value.
 *
 *END*************************************************************************/
void PDB_HAL_Init(PDB_Type * base)
{
    uint32_t chn, preChn;

    base->SC    = 0U;
    PDB_HAL_Enable(base);
    base->MOD   = 0xFFFFU;
    base->IDLY  = 0xFFFFU;
    /* For ADC trigger. */
    for (chn = 0U; chn < FSL_FEATURE_PDB_ADC_CHANNEL_COUNT; chn++)
    {
        base->CH[chn].C1 = 0U;
        base->CH[chn].S = 0xFU;
        for (preChn = 0U; preChn < FSL_FEATURE_PDB_ADC_PRE_CHANNEL_COUNT; preChn++)
        {
            PDB_HAL_SetAdcPreTriggerDelayValue(base, chn, preChn, 0U);
        }
    }
    /* For Pulse out trigger. */
    base->POEN = 0U;
    for (chn = 0U; chn < FSL_FEATURE_PDB_PODLY_COUNT; chn++)
    {
         base->PODLY[chn] = 0U;
    }
    /* Load the setting value. */
    PDB_HAL_SetLoadValuesCmd(base);
    PDB_HAL_Disable(base);
}

/*FUNCTION*********************************************************************
 *
 * Function Name : PDB_HAL_ConfigTimer
 * Description   : Configure the PDB timer.
 *
 *END*************************************************************************/
pdb_status_t PDB_HAL_ConfigTimer(PDB_Type * base, const pdb_timer_config_t *configPtr)
{
    uint32_t sc;
    
    if(0U == configPtr)
    {
        return PDB_STATUS_INVALID_ARGUMENT;
    }
    
    sc = base->SC;
    sc &= ~(  (uint32_t)PDB_SC_LDMOD_MASK
            | (uint32_t)PDB_SC_PDBEIE_MASK
            | (uint32_t)PDB_SC_PRESCALER_MASK  
            | (uint32_t)PDB_SC_TRGSEL_MASK
            | (uint32_t)PDB_SC_MULT_MASK
            | (uint32_t)PDB_SC_CONT_MASK
            | (uint32_t)PDB_SC_DMAEN_MASK
            | (uint32_t)PDB_SC_PDBIE_MASK
    );
    
    sc |= PDB_SC_LDMOD((uint32_t)(configPtr->loadValueMode));
    if (configPtr->seqErrIntEnable)
    {
        sc |= PDB_SC_PDBEIE_MASK;
    }
    sc |= PDB_SC_PRESCALER((uint32_t)(configPtr->clkPreDiv));
    sc |= PDB_SC_TRGSEL((uint32_t)(configPtr->triggerInput));
    sc |= PDB_SC_MULT((uint32_t)(configPtr->clkPreMultFactor));
    if (configPtr->continuousModeEnable)
    {
        sc |= PDB_SC_CONT_MASK;
    }
    if (configPtr->dmaEnable)
    {
        sc |= PDB_SC_DMAEN_MASK;
    }
    if (configPtr->intEnable)
    {
        sc |= PDB_SC_PDBIE_MASK;
    }
    base->SC = sc;
    
    return PDB_STATUS_SUCCESS;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : PDB_HAL_SetAdcPreTriggerBackToBackEnable
 * Description   : Switch to enable pre-trigger's back to back mode.
 *
 *END*************************************************************************/
void PDB_HAL_SetAdcPreTriggerBackToBackEnable(PDB_Type * base, uint32_t chn, uint32_t preChnMask, bool enable)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(chn < FSL_FEATURE_PDB_ADC_CHANNEL_COUNT);
#endif

    uint32_t c1 = base->CH[chn].C1;    
    if (enable)
    {
        c1 |= PDB_C1_BB(preChnMask);
    }
    else
    {
        c1 &= ~PDB_C1_BB(preChnMask);
    }
    base->CH[chn].C1 = c1;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : PDB_HAL_SetAdcPreTriggerOutputEnable
 * Description   : Switch to enable pre-trigger's output.
 *
 *END*************************************************************************/
void PDB_HAL_SetAdcPreTriggerOutputEnable(PDB_Type * base, uint32_t chn, uint32_t preChnMask, bool enable)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(chn < FSL_FEATURE_PDB_ADC_CHANNEL_COUNT);
#endif

    uint32_t c1 = base->CH[chn].C1;    
    if (enable)
    {
        c1 |= PDB_C1_TOS(preChnMask);
    }
    else
    {
        c1 &= ~PDB_C1_TOS(preChnMask);
    }
    base->CH[chn].C1 = c1;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : PDB_HAL_SetAdcPreTriggerEnable
 * Description   : Switch to enable pre-trigger's.
 *
 *END*************************************************************************/
void PDB_HAL_SetAdcPreTriggerEnable(PDB_Type * base, uint32_t chn, uint32_t preChnMask, bool enable)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(chn < FSL_FEATURE_PDB_ADC_CHANNEL_COUNT);
#endif

    uint32_t c1 = base->CH[chn].C1;    
    if (enable)
    {
        c1 |= PDB_C1_EN(preChnMask);
    }
    else
    {
        c1 &= ~PDB_C1_EN(preChnMask);
    }
    base->CH[chn].C1 = c1;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : PDB_HAL_ClearAdcPreTriggerFlags
 * Description   : Clear the flag that the PDB counter reaches to the
 * pre-trigger's delay value.
 *
 *END*************************************************************************/
void PDB_HAL_ClearAdcPreTriggerFlags(PDB_Type * base, uint32_t chn, uint32_t preChnMask)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(chn < FSL_FEATURE_PDB_ADC_CHANNEL_COUNT);
#endif

    /* Write 0 to clear. */
    uint32_t s = base->CH[chn].S;
    s &= ~PDB_S_CF( preChnMask ); /* Update the change. */
    
    base->CH[chn].S = s;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : PDB_HAL_ClearAdcPreTriggerSeqErrFlags
 * Description   : Clear the flag that sequence error is detected.
 *
 *END*************************************************************************/
void PDB_HAL_ClearAdcPreTriggerSeqErrFlags(PDB_Type * base, uint32_t chn, uint32_t preChnMask)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(chn < FSL_FEATURE_PDB_ADC_CHANNEL_COUNT);
#endif

    /* Write 0 to clear. */
    uint32_t s = base->CH[chn].S;
    s &= ~PDB_S_ERR( preChnMask );
    
    base->CH[chn].S = s;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : PDB_HAL_SetAdcPreTriggerDelayValue
 * Description   : Set the delay value for pre-trigger.
 *
 *END*************************************************************************/
void PDB_HAL_SetAdcPreTriggerDelayValue(PDB_Type * base, uint32_t chn, uint32_t preChn, uint32_t value)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(chn < FSL_FEATURE_PDB_ADC_CHANNEL_COUNT);
    DEV_ASSERT(preChn < FSL_FEATURE_PDB_ADC_PRE_CHANNEL_COUNT);
#endif
    base->CH[chn].DLY[preChn] = value;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : PDB_HAL_SetCmpPulseOutEnable
 * Description   : Switch to enable the pulse-out trigger.
 *
 *END*************************************************************************/
void PDB_HAL_SetCmpPulseOutEnable(PDB_Type * base, uint32_t pulseChnMask, bool enable)
{   
    uint32_t poen = base->POEN;
    if (enable)
    {
        poen |= PDB_POEN_POEN(pulseChnMask);
    }
    else
    {
        poen &= ~PDB_POEN_POEN(pulseChnMask);
    }
    base->POEN = poen;
}

/******************************************************************************
 * EOF
 *****************************************************************************/
