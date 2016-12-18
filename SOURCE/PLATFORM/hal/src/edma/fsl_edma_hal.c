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
#include "fsl_edma_hal.h"

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_HAL_Init
 * Description   : Initializes eDMA module to known state.
 *
 *END**************************************************************************/
void EDMA_HAL_Init(DMA_Type * base)
{
    uint32_t i;

    /* Clear the bit of CR register */
    BITBAND_ACCESS32(&(base->CR), DMA_CR_CLM_SHIFT) = 0U;
    BITBAND_ACCESS32(&(base->CR), DMA_CR_CX_SHIFT) = 0U;
    BITBAND_ACCESS32(&(base->CR), DMA_CR_ECX_SHIFT) = 0U;
    BITBAND_ACCESS32(&(base->CR), DMA_CR_EDBG_SHIFT) = 0U;
    BITBAND_ACCESS32(&(base->CR), DMA_CR_EMLM_SHIFT) = 0U;
    BITBAND_ACCESS32(&(base->CR), DMA_CR_ERCA_SHIFT) = 0U;

    for (i = 0; i < FSL_FEATURE_EDMA_MODULE_CHANNELS; i++)
    {
        EDMA_HAL_TCDClearReg(base, i);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_HAL_CancelTransfer
 * Description   : Cancels the remaining data transfer.
 *
 *END**************************************************************************/
void EDMA_HAL_CancelTransfer(DMA_Type * base)
{
    BITBAND_ACCESS32(&(base->CR), DMA_CR_CX_SHIFT) = 1U;
    while (BITBAND_ACCESS32(&(base->CR), DMA_CR_CX_SHIFT) != 0)
    {}
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_HAL_CancelTransferWithError
 * Description   : Cancels the remaining data transfer and treat it as error.
 *
 *END**************************************************************************/
void EDMA_HAL_CancelTransferWithError(DMA_Type * base)
{
    BITBAND_ACCESS32(&(base->CR), DMA_CR_ECX_SHIFT) = 1U;
    while (BITBAND_ACCESS32(&(base->CR), DMA_CR_ECX_SHIFT) != 0)
    {}
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_HAL_SetErrorIntCmd
 * Description   : Enable/Disable error interrupt for channels.
 *
 *END**************************************************************************/
void EDMA_HAL_SetErrorIntCmd(DMA_Type * base, edma_channel_indicator_t channel, bool enable)
{
    if (enable)
    {
        base->SEEI = (uint8_t)channel;
    }
    else
    {
        base->CEEI = (uint8_t)channel;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_HAL_SetDmaRequestCmd
 * Description   : Enable/Disable dma request for channel or all channels.
 *
 *END**************************************************************************/
void EDMA_HAL_SetDmaRequestCmd(DMA_Type * base, edma_channel_indicator_t channel,bool enable)
{

    if (enable)
    {
        base->SERQ = (uint8_t)channel;
    }
    else
    {
        base->CERQ = (uint8_t)channel;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_HAL_TCDClearReg
 * Description   : Set registers to 0 for hardware TCD of eDMA channel.
 *
 *END**************************************************************************/
void EDMA_HAL_TCDClearReg(DMA_Type * base,uint32_t channel)
{
    base->TCD[channel].SADDR = 0U;
    base->TCD[channel].SOFF = 0U;
    base->TCD[channel].ATTR = 0U;
    base->TCD[channel].NBYTES.MLNO = 0U;
    base->TCD[channel].SLAST = 0U;
    base->TCD[channel].DADDR = 0U;
    base->TCD[channel].DOFF = 0U;
    base->TCD[channel].CITER.ELINKNO = 0U;
    base->TCD[channel].DLASTSGA = 0U;
    base->TCD[channel].CSR = 0U;
    base->TCD[channel].BITER.ELINKNO = 0U;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_HAL_TCDSetAttribute
 * Description   : Configures the transfer attribute for eDMA channel.
 *
 *END**************************************************************************/
void EDMA_HAL_TCDSetAttribute(
                DMA_Type * base, uint32_t channel,
                edma_modulo_t srcModulo, edma_modulo_t destModulo,
                edma_transfer_size_t srcTransferSize, edma_transfer_size_t destTransferSize)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_EDMA_MODULE_CHANNEL);
#endif
    uint16_t regValTemp;

    regValTemp = (srcModulo << DMA_TCD_ATTR_SMOD_SHIFT) | (srcTransferSize << DMA_TCD_ATTR_SSIZE_SHIFT);
    regValTemp |= (destModulo << DMA_TCD_ATTR_DMOD_SHIFT) | (destTransferSize << DMA_TCD_ATTR_DSIZE_SHIFT);
    base->TCD[channel].ATTR = regValTemp;
}
/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_HAL_TCDSetNbytes
 * Description   : Configures the nbytes for eDMA channel.
 *
 *END**************************************************************************/
void EDMA_HAL_TCDSetNbytes(DMA_Type * base, uint32_t channel, uint32_t nbytes)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_EDMA_MODULE_CHANNEL);
#endif

    if (BITBAND_ACCESS32(&(base->CR), DMA_CR_EMLM_SHIFT))
    {
        if (!(BITBAND_ACCESS32(&(base->TCD[channel].NBYTES.MLOFFNO), DMA_TCD_NBYTES_MLOFFNO_SMLOE_SHIFT) ||
                BITBAND_ACCESS32(&(base->TCD[channel].NBYTES.MLOFFNO), DMA_TCD_NBYTES_MLOFFNO_DMLOE_SHIFT)))
        {
            base->TCD[channel].NBYTES.MLOFFNO = (nbytes & DMA_TCD_NBYTES_MLOFFNO_NBYTES_MASK);
        }
        else
        {
            uint32_t regValTemp;
            
            regValTemp = base->TCD[channel].NBYTES.MLOFFYES;
            regValTemp &= ~(DMA_TCD_NBYTES_MLOFFYES_NBYTES_MASK);
            regValTemp |= DMA_TCD_NBYTES_MLOFFYES_NBYTES(nbytes);
            base->TCD[channel].NBYTES.MLOFFYES = regValTemp;
        }

    }
    else
    {       
        base->TCD[channel].NBYTES.MLNO = nbytes;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_HAL_TCDGetNbytes
 * Description   : Get nbytes configuration data.
 *
 *END**************************************************************************/
uint32_t EDMA_HAL_TCDGetNbytes(DMA_Type * base, uint32_t channel)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_EDMA_MODULE_CHANNEL);
#endif

    if (BITBAND_ACCESS32(&(base->CR), DMA_CR_EMLM_SHIFT))
    {
        if (BITBAND_ACCESS32(&(base->TCD[channel].NBYTES.MLOFFNO), DMA_TCD_NBYTES_MLOFFNO_SMLOE_SHIFT) ||
                BITBAND_ACCESS32(&(base->TCD[channel].NBYTES.MLOFFNO), DMA_TCD_NBYTES_MLOFFNO_DMLOE_SHIFT))
        {
            return ((base->TCD[channel].NBYTES.MLOFFYES & DMA_TCD_NBYTES_MLOFFYES_NBYTES_MASK) >> DMA_TCD_NBYTES_MLOFFYES_NBYTES_SHIFT);
        }
        else
        {
            return ((base->TCD[channel].NBYTES.MLOFFNO & DMA_TCD_NBYTES_MLOFFNO_NBYTES_MASK) >> DMA_TCD_NBYTES_MLOFFNO_NBYTES_SHIFT);
        }
    }
    else
    {
        return ((base->TCD[channel].NBYTES.MLNO & DMA_TCD_NBYTES_MLNO_NBYTES_MASK) >> DMA_TCD_NBYTES_MLNO_NBYTES_SHIFT);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_HAL_TCDSetMinorLoopOffset
 * Description   : Configures the minor loop offset for the TCD.
 *
 *END**************************************************************************/
void EDMA_HAL_TCDSetMinorLoopOffset(DMA_Type * base, uint32_t channel, uint32_t offset)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_EDMA_MODULE_CHANNEL);
#endif

    if (BITBAND_ACCESS32(&(base->CR), DMA_CR_EMLM_SHIFT))
    {
        if (BITBAND_ACCESS32(&(base->TCD[channel].NBYTES.MLOFFNO), DMA_TCD_NBYTES_MLOFFYES_SMLOE_SHIFT) ||
                BITBAND_ACCESS32(&(base->TCD[channel].NBYTES.MLOFFNO), DMA_TCD_NBYTES_MLOFFYES_DMLOE_SHIFT))
        {
            uint32_t regValTemp;

            regValTemp = base->TCD[channel].NBYTES.MLOFFYES;
            regValTemp &= ~(DMA_TCD_NBYTES_MLOFFYES_MLOFF_MASK);
            regValTemp |= DMA_TCD_NBYTES_MLOFFYES_MLOFF(offset);
            base->TCD[channel].NBYTES.MLOFFYES = regValTemp;
        }
    }
}
/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_HAL_TCDSetScatterGatherLink
 * Description   : Configures the memory address of the next TCD, in Scatter/Gather mode.
 *
 *END**************************************************************************/
void EDMA_HAL_TCDSetScatterGatherLink(DMA_Type * base, uint32_t channel, uint32_t nextTCDAddr)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_EDMA_MODULE_CHANNEL);
#endif
    base->TCD[channel].DLASTSGA = nextTCDAddr;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_HAL_TCDSetChannelMinorLink
 * Description   : Set Channel minor link for hardware TCD.
 *
 *END**************************************************************************/
void EDMA_HAL_TCDSetChannelMinorLink(
                DMA_Type * base, uint32_t channel, uint32_t linkChannel, bool enable)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_EDMA_MODULE_CHANNEL);
    DEV_ASSERT(linkChannel < FSL_FEATURE_EDMA_MODULE_CHANNEL);
#endif

    if (enable)
    {
        BITBAND_ACCESS32(&(base->TCD[channel].BITER.ELINKYES), DMA_TCD_BITER_ELINKYES_ELINK_SHIFT) = 1U;
        BITBAND_ACCESS32(&(base->TCD[channel].CITER.ELINKYES), DMA_TCD_CITER_ELINKYES_ELINK_SHIFT) = 1U;

        uint32_t regValTemp;

        regValTemp = base->TCD[channel].BITER.ELINKYES;
        regValTemp &= ~(DMA_TCD_BITER_ELINKYES_LINKCH_MASK);
        regValTemp |= DMA_TCD_BITER_ELINKYES_LINKCH(linkChannel);
        base->TCD[channel].BITER.ELINKYES = regValTemp;
        
        regValTemp = base->TCD[channel].CITER.ELINKYES;
        regValTemp &= ~(DMA_TCD_CITER_ELINKYES_LINKCH_MASK);
        regValTemp |= DMA_TCD_CITER_ELINKYES_LINKCH(linkChannel);
        base->TCD[channel].CITER.ELINKYES = regValTemp;
    }
    else
    {
        BITBAND_ACCESS32(&(base->TCD[channel].BITER.ELINKYES), DMA_TCD_BITER_ELINKYES_ELINK_SHIFT) = 0U;
        BITBAND_ACCESS32(&(base->TCD[channel].CITER.ELINKYES), DMA_TCD_CITER_ELINKYES_ELINK_SHIFT) = 0U;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_HAL_TCD_TCDSetMajorCount
 * Description   : Sets the major iteration count according to minor loop
 * channel link setting.
 *
 *END**************************************************************************/
void EDMA_HAL_TCDSetMajorCount(DMA_Type * base, uint32_t channel, uint32_t count)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_EDMA_MODULE_CHANNEL);
#endif

    uint32_t regValTemp;
    if (BITBAND_ACCESS32(&(base->TCD[channel].BITER.ELINKNO), DMA_TCD_BITER_ELINKNO_ELINK_SHIFT))
    {
        regValTemp = base->TCD[channel].BITER.ELINKYES;
        regValTemp &= ~(DMA_TCD_BITER_ELINKYES_BITER_MASK);
        regValTemp |= DMA_TCD_BITER_ELINKYES_BITER(count);
        base->TCD[channel].BITER.ELINKYES = regValTemp;
        
        regValTemp = base->TCD[channel].CITER.ELINKYES;
        regValTemp &= ~(DMA_TCD_CITER_ELINKYES_CITER_MASK);
        regValTemp |= DMA_TCD_CITER_ELINKYES_CITER(count);
        base->TCD[channel].CITER.ELINKYES = regValTemp;
    }
    else
    {
        regValTemp = base->TCD[channel].BITER.ELINKNO;
        regValTemp &= ~(DMA_TCD_BITER_ELINKNO_BITER_MASK);
        regValTemp |= DMA_TCD_BITER_ELINKNO_BITER(count);
        base->TCD[channel].BITER.ELINKNO = regValTemp;
        
        regValTemp = base->TCD[channel].CITER.ELINKNO;
        regValTemp &= ~(DMA_TCD_CITER_ELINKNO_CITER_MASK);
        regValTemp |= DMA_TCD_CITER_ELINKNO_CITER(count);
        base->TCD[channel].CITER.ELINKNO = regValTemp;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_HAL_TCDGetBeginMajorCount
 * Description   : Gets the begin major iteration count according to minor loop
 * channel link setting.
 *
 *END**************************************************************************/
uint32_t EDMA_HAL_TCDGetBeginMajorCount(DMA_Type * base, uint32_t channel)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_EDMA_MODULE_CHANNEL);
#endif

    if (BITBAND_ACCESS32(&(base->TCD[channel].BITER.ELINKNO), DMA_TCD_BITER_ELINKNO_ELINK_SHIFT))
    {
        return ((base->TCD[channel].BITER.ELINKYES & DMA_TCD_BITER_ELINKYES_BITER_MASK) >> DMA_TCD_BITER_ELINKYES_BITER_SHIFT);
    }
    else
    {
        return ((base->TCD[channel].BITER.ELINKNO & DMA_TCD_BITER_ELINKNO_BITER_MASK) >> DMA_TCD_BITER_ELINKNO_BITER_SHIFT);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_HAL_TCDGetCurrentMajorCount
 * Description   : Gets the current major iteration count according to minor
 * loop channel link setting.
 *
 *END**************************************************************************/
uint32_t EDMA_HAL_TCDGetCurrentMajorCount(DMA_Type * base, uint32_t channel)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_EDMA_MODULE_CHANNEL);
#endif

    if (BITBAND_ACCESS32(&(base->TCD[channel].BITER.ELINKNO), DMA_TCD_BITER_ELINKNO_ELINK_SHIFT))
    {
        return ((base->TCD[channel].CITER.ELINKYES & DMA_TCD_CITER_ELINKYES_CITER_MASK) >> DMA_TCD_CITER_ELINKYES_CITER_SHIFT);
    }
    else
    {
        return ((base->TCD[channel].CITER.ELINKNO & DMA_TCD_CITER_ELINKNO_CITER_MASK) >> DMA_TCD_CITER_ELINKNO_CITER_SHIFT);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_HAL_TCDGetUnfinishedBytes
 * Description   : Get the bytes number of bytes haven't been transferred for
 * this hardware TCD.
 *
 *END**************************************************************************/
uint32_t EDMA_HAL_TCDGetUnfinishedBytes(DMA_Type * base, uint32_t channel)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_EDMA_MODULE_CHANNEL);
#endif

    uint32_t nbytes;

    nbytes = EDMA_HAL_TCDGetNbytes(base, channel);

    if (BITBAND_ACCESS32(&(base->TCD[channel].BITER.ELINKNO), DMA_TCD_BITER_ELINKNO_ELINK_SHIFT))
    {
        return (((base->TCD[channel].CITER.ELINKYES & DMA_TCD_CITER_ELINKYES_CITER_MASK) >> DMA_TCD_CITER_ELINKYES_CITER_SHIFT) * nbytes);
    }
    else
    {
        return (((base->TCD[channel].CITER.ELINKNO & DMA_TCD_CITER_ELINKNO_CITER_MASK) >> DMA_TCD_CITER_ELINKNO_CITER_SHIFT) * nbytes);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_HAL_TCDGetFinishedBytes
 * Description   : Get the bytes number of bytes already be transferred for this
 * hardware TCD.
 *
 *END**************************************************************************/
uint32_t EDMA_HAL_TCDGetFinishedBytes(DMA_Type * base, uint32_t channel)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_EDMA_MODULE_CHANNEL);
#endif

    uint32_t nbytes, begin_majorcount, current_majorcount;

    nbytes = EDMA_HAL_TCDGetNbytes(base, channel);
    begin_majorcount = EDMA_HAL_TCDGetBeginMajorCount(base,channel);
    current_majorcount = EDMA_HAL_TCDGetCurrentMajorCount(base,channel);

    return ((begin_majorcount - current_majorcount) * nbytes);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
