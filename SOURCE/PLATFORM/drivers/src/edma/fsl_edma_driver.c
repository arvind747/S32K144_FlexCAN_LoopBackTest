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

#include "fsl_edma_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_interrupt_manager.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief EDMA global structure to maintain eDMA state */
static volatile edma_state_t *g_edma = NULL;

/*******************************************************************************
 * PROTOTYPES
 ******************************************************************************/
static edma_status_t EDMA_DRV_ClaimChannel(uint8_t channel, dma_request_source_t source, edma_chn_state_t *chn);
static void EDMA_DRV_ClearIntStatus(uint8_t channel);
static void EDMA_DRV_ClearSoftwareTCD(edma_software_tcd_t * stcd);
static bool EDMA_DRV_ValidTransferSize(edma_transfer_size_t size);

/*! @brief Macro for EDMA driver lock mechanism. */
#if (USE_RTOS)
    #define EDMA_DRV_LOCK()         OSA_MutexLock(&g_edma->lock, OSA_WAIT_FOREVER)
    #define EDMA_DRV_UNLOCK()       OSA_MutexUnlock(&g_edma->lock)
#else
    #define EDMA_DRV_LOCK()         do {}while (0)
    #define EDMA_DRV_UNLOCK()       do {}while (0)
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/
/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_DRV_Init
 * Description   : Initializes the eDMA module.
 *
 *END**************************************************************************/
edma_status_t EDMA_DRV_Init(edma_state_t *edmaState, const edma_user_config_t *userConfig)
{
    uint32_t i, dmaClkFreq, dmamuxClkFreq;
    DMA_Type * edmaRegBase;
    DMAMUX_Type * dmamuxRegBase;
    IRQn_Type irqNumber;

    CLOCK_SYS_GetFreq(dmaClockName, &dmaClkFreq);
    CLOCK_SYS_GetFreq(dmamuxClockName, &dmamuxClkFreq);

    /* Return an error if the clock is disabled for DMA operations */
    if ((dmaClkFreq == 0U) || (dmamuxClkFreq == 0U))
    {
        return EDMA_STATUS_CLOCK_GATED_OFF;
    }

    if (g_edma)
    {
        return EDMA_STATUS_SUCCESS;
    }

    g_edma = edmaState;

    /* Clear the state structure. */
    uint8_t *clearStructPtr = (uint8_t *)g_edma;
    uint8_t *lastByteAddr = (uint8_t *)g_edma + sizeof(edma_state_t);
    while (clearStructPtr < lastByteAddr)
    {
        *clearStructPtr = 0;
        clearStructPtr++;
    }

    edmaRegBase = g_edmaBase[0U];

    /* Init eDMA module on hardware level. */
    EDMA_HAL_Init(edmaRegBase);

    /* Set arbitration mode */
    EDMA_HAL_SetChannelArbitrationMode(edmaRegBase, userConfig->chnArbitration);

    /* Set 'Halt on error' configuration */
    EDMA_HAL_SetHaltOnErrorCmd(edmaRegBase, !userConfig->notHaltOnError);

#if defined FSL_FEATURE_EDMA_HAS_ERROR_IRQ
    /* Enable the error interrupt for eDMA module. */
    irqNumber = g_edmaErrIrqId[0U];
    INT_SYS_EnableIRQ(irqNumber);
#endif

    /* Register all edma channel interrupt handler into vector table. */
    for (i = 0U; i < FSL_FEATURE_EDMA_MODULE_CHANNELS; i++)
    {
        /* Enable channel interrupt ID. */
        irqNumber = g_edmaIrqId[i];
        INT_SYS_EnableIRQ(irqNumber);
    }

    /* Initialize DMAMUX */
    dmamuxRegBase = g_dmamuxBase[0U];
    DMAMUX_HAL_Init(dmamuxRegBase);

    return EDMA_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_DRV_Deinit
 * Description   : Deinitialize EDMA.
 *
 *END**************************************************************************/
edma_status_t EDMA_DRV_Deinit(void)
{
    uint32_t i;
    IRQn_Type irqNumber;
    edma_chn_state_t *chn;

    /* Release all edma channel. */
#if defined FSL_FEATURE_EDMA_HAS_ERROR_IRQ
    /* Disable the error interrupt for eDMA module. */
    irqNumber = g_edmaErrIrqId[0U];
    INT_SYS_DisableIRQ(irqNumber);
#endif

    for (i = 0U; i < FSL_FEATURE_EDMA_MODULE_CHANNELS; i++)
    {
        /* Release all channels. */
        chn = g_edma->chn[i];
        if (chn)
        {
            EDMA_DRV_ReleaseChannel(chn);
        }

        /* Disable channel interrupt ID. */
        irqNumber = g_edmaIrqId[i];
        INT_SYS_DisableIRQ(irqNumber);
    }

    g_edma = NULL;

    return EDMA_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_DRV_ChannelInit
 * Description   : Initialize EDMA channel.
 *
 *END**************************************************************************/
edma_status_t EDMA_DRV_ChannelInit(edma_chn_state_t *edmaChannelState, const edma_channel_config_t *edmaChannelConfig)
{
    uint8_t allocatedChannel;
    DMA_Type * edmaRegBase = g_edmaBase[0U];

    /* Check if the channel defined by user in the channel configuration structure is valid */
    if ((edmaChannelConfig->channel > FSL_FEATURE_EDMA_MODULE_CHANNELS) && (edmaChannelConfig->channel != EDMA_ANY_CHANNEL))
    {
        return EDMA_STATUS_INVALID_ARGUMENT;
    }

    /* Request the channel */
    allocatedChannel = EDMA_DRV_RequestChannel(edmaChannelConfig->channel, edmaChannelConfig->source, edmaChannelState);
    if (allocatedChannel == EDMA_INVALID_CHANNEL)
    {
        return EDMA_STATUS_FAIL;
    }

    /* Set the channel priority, as defined in the configuration structure, only if fixed arbitration mode is selected */
    if ((edmaChannelConfig->priority != EDMA_CHN_DEFAULT_PRIORITY) && !EDMA_HAL_GetChannelArbitrationMode(edmaRegBase))
    {
        EDMA_HAL_SetChannelPriority(edmaRegBase, allocatedChannel, edmaChannelConfig->priority);
    }

    /* Install the user callback */
    EDMA_DRV_InstallCallback(edmaChannelState, edmaChannelConfig->callback, edmaChannelConfig->callbackParam);

    return EDMA_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_DRV_InstallCallback
 * Description   : Register callback function and parameter.
 *
 *END**************************************************************************/
edma_status_t EDMA_DRV_InstallCallback(edma_chn_state_t *chn, edma_callback_t callback, void *parameter)
{
    chn->callback = callback;
    chn->parameter = parameter;

    return EDMA_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_DRV_RequestChannel
 * Description   : Request an eDMA channel.
 *
 *END**************************************************************************/
uint8_t EDMA_DRV_RequestChannel(uint8_t channel, dma_request_source_t source, edma_chn_state_t *chn)
{
    /*Check if dynamic allocation is requested */
    if (channel == EDMA_ANY_CHANNEL)
    {
        uint32_t i;

        for (i=0U; i < FSL_FEATURE_DMAMUX_MODULE_CHANNELS; i++)
        {
            EDMA_DRV_LOCK();
            if (!g_edma->chn[i])
            {
                g_edma->chn[i] = chn;
                EDMA_DRV_UNLOCK();
                EDMA_DRV_ClaimChannel(i, source, chn);
                return i;
            }
            EDMA_DRV_UNLOCK();
        }

        /* No available channel. */
        return EDMA_INVALID_CHANNEL;
    }

    /* Static allocation */
    EDMA_DRV_LOCK();
    if (!g_edma->chn[channel])
    {
        g_edma->chn[channel] = chn;
        EDMA_DRV_UNLOCK();
        EDMA_DRV_ClaimChannel(channel, source, chn);
        return channel;
    }
    EDMA_DRV_UNLOCK();

    return EDMA_INVALID_CHANNEL;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_DRV_ClaimChannel
 * Description   : Claim an edma channel.
 *
 *END**************************************************************************/
static edma_status_t EDMA_DRV_ClaimChannel(uint8_t channel, dma_request_source_t source, edma_chn_state_t *chn)
{
    DMA_Type * edmaRegBase = g_edmaBase[0U];
    DMAMUX_Type * dmamuxRegBase = g_dmamuxBase[0U];

    /* Reset the channel state structure to default value. */
    uint8_t *clearStructPtr = (uint8_t *)chn;
    uint8_t *lastByteAddr = (uint8_t *)chn + sizeof(edma_chn_state_t);
    while (clearStructPtr < lastByteAddr)
    {
        *clearStructPtr = 0;
        clearStructPtr++;
    }

    /* Init the channel state structure to the allocated channel number. */
    chn->channel = channel;

    /* Enable error interrupt for this channel */
    EDMA_HAL_SetErrorIntCmd(edmaRegBase, (edma_channel_indicator_t)channel, true);

    /* Configure the DMAMUX for edma channel */
    DMAMUX_HAL_SetChannelCmd(dmamuxRegBase, channel, false);
    DMAMUX_HAL_SetTriggerSource(dmamuxRegBase, channel, (uint8_t)source);
    DMAMUX_HAL_SetChannelCmd(dmamuxRegBase, channel, true);

    /* Clear the TCD registers for this channel */
    EDMA_HAL_TCDClearReg(edmaRegBase, channel);

    return EDMA_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_DRV_ReleaseChannel
 * Description   : Free eDMA channel's hardware and software resource.
 *
 *END**************************************************************************/
edma_status_t EDMA_DRV_ReleaseChannel(edma_chn_state_t *chn)
{
    uint8_t channel = chn->channel;
    DMA_Type * edmaRegBase = g_edmaBase[0U];

    if (!g_edma->chn[channel])
    {
        return EDMA_STATUS_INVALID_ARGUMENT;
    }

    /* Stop edma channel. */
    EDMA_HAL_SetDmaRequestCmd(edmaRegBase,(edma_channel_indicator_t)channel, false);

    /* Reset the channel state structure to default value. */
    uint8_t *clearStructPtr = (uint8_t *)chn;
    uint8_t *lastByteAddr = (uint8_t *)chn + sizeof(edma_chn_state_t);
    while (clearStructPtr < lastByteAddr)
    {
        *clearStructPtr = 0;
        clearStructPtr++;
    }

    EDMA_DRV_LOCK();
    g_edma->chn[channel] = NULL;
    EDMA_DRV_UNLOCK();
    return EDMA_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_DRV_ClearIntStatus
 * Description   : Clear done and interrupt status.
 *
 *END**************************************************************************/
static void EDMA_DRV_ClearIntStatus(uint8_t channel)
{
    DMA_Type * edmaRegBase = g_edmaBase[0U];

    EDMA_HAL_ClearDoneStatusFlag(edmaRegBase, (edma_channel_indicator_t)channel);
    EDMA_HAL_ClearIntStatusFlag(edmaRegBase, (edma_channel_indicator_t)channel);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_DRV_ClearSoftwareTCD
 * Description   : Clear the software tcd structure.
 *
 *END**************************************************************************/
static void EDMA_DRV_ClearSoftwareTCD(edma_software_tcd_t * stcd)
{
    uint8_t *byteAccess, *stcdAddr;
    stcdAddr = (uint8_t *)stcd;
    for (byteAccess = stcdAddr; byteAccess < stcdAddr + sizeof(edma_software_tcd_t); byteAccess++)
    {
        *byteAccess = 0;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_IRQ_HANDLER
 * Description   : EDMA IRQ handler.
 *
 *END**************************************************************************/
void EDMA_DRV_IRQHandler(uint8_t channel)
{
    edma_chn_state_t *chn = g_edma->chn[channel];

    EDMA_DRV_ClearIntStatus(channel);

    if (!chn)
    {
        return;
    }

    if (chn->callback)
    {
        chn->callback(chn->parameter, chn->status);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : DMA_ERR_IRQHandler
 * Description   : EDMA error handler
 *
 *END**************************************************************************/
void EDMA_DRV_ErrorIRQHandler(void)
{
    uint32_t channel = 0U, error;
    DMA_Type * edmaRegBase = g_edmaBase[0U];
    edma_chn_state_t *chn;

    error = EDMA_HAL_GetErrorIntStatusFlag(edmaRegBase);

    while (error && (channel < FSL_FEATURE_EDMA_MODULE_CHANNELS))
    {
        if (error & EDMA_ERR_LSB_MASK)
        {
            EDMA_HAL_SetDmaRequestCmd(edmaRegBase, (edma_channel_indicator_t)channel, false);
            chn = g_edma->chn[channel];
            if (chn)
            {
                EDMA_DRV_ClearIntStatus(channel);
                chn->status = EDMA_CHN_ERROR;
                if (chn->callback)
                {
                    chn->callback(chn->parameter, chn->status);
                }
            }
        }
        error = error >> 1U;
        channel++;
    }
    EDMA_HAL_SetHaltCmd(edmaRegBase, false);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_DRV_ConfigSingleBlockTransfer
 * Description   : Configures a DMA single block transfer.
 *
 *END**************************************************************************/
edma_status_t EDMA_DRV_ConfigSingleBlockTransfer(
                            edma_chn_state_t *chn, edma_transfer_type_t type,
                            uint32_t srcAddr, uint32_t destAddr,
                            edma_transfer_size_t transferSize, uint32_t dataBufferSize)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(chn != NULL);
#endif

    DMA_Type * edmaRegBase = g_edmaBase[0U];
    uint8_t transferOffset;

    /* Check if the value passed for 'transferSize' is valid */
    if (!EDMA_DRV_ValidTransferSize(transferSize))
    {
        return EDMA_STATUS_INVALID_ARGUMENT;
    }

    /* Compute the transfer offset, based on transfer size. 
     * The number of bytes transferred in each source read/destination write
     * is obtained with the following formula:
     *    source_read_size = 2^SSIZE
     *    destination_write_size = 2^DSIZE
     */
    transferOffset = 1U << ((uint8_t)transferSize);

    /* The number of bytes to be transferred (buffer size) must
     * be a multiple of the source read/destination write size
     */
    if (dataBufferSize % transferOffset != 0U)
    {
        return EDMA_STATUS_INVALID_ARGUMENT;
    }

    /* Clear transfer control descriptor for the current channel */
    EDMA_HAL_TCDClearReg(edmaRegBase, chn->channel);

    /* Configure source and destination addresses */
    EDMA_HAL_TCDSetSrcAddr(edmaRegBase, chn->channel, srcAddr);
    EDMA_HAL_TCDSetDestAddr(edmaRegBase, chn->channel, destAddr);

    /* Set transfer size (1B/2B/4B/16B/32B) */
    EDMA_HAL_TCDSetAttribute(edmaRegBase, chn->channel, EDMA_MODULO_OFF, EDMA_MODULO_OFF, transferSize, transferSize);

    /* Configure source/destination offset. */
    switch (type)
    {
        case EDMA_TRANSFER_PERIPH2MEM:
            EDMA_HAL_TCDSetSrcOffset(edmaRegBase, chn->channel, 0);
            EDMA_HAL_TCDSetDestOffset(edmaRegBase, chn->channel, transferOffset);
            break;
        case EDMA_TRANSFER_MEM2PERIPH:
            EDMA_HAL_TCDSetSrcOffset(edmaRegBase, chn->channel, transferOffset);
            EDMA_HAL_TCDSetDestOffset(edmaRegBase, chn->channel, 0);
            break;
        case EDMA_TRANSFER_MEM2MEM:
            EDMA_HAL_TCDSetSrcOffset(edmaRegBase, chn->channel, transferOffset);
            EDMA_HAL_TCDSetDestOffset(edmaRegBase, chn->channel, transferOffset);
            break;
    }

    /* Set the total number of bytes to be transfered */
    EDMA_HAL_TCDSetNbytes(edmaRegBase, chn->channel, dataBufferSize);

    /* Set major iteration count to 1 (single block mode) */
    EDMA_HAL_TCDSetMajorCount(edmaRegBase, chn->channel, 1U);

    /* Enable interrupt when the transfer completes */
    EDMA_HAL_TCDSetIntCmd(edmaRegBase, chn->channel, true);

    return EDMA_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_DRV_ConfigLoopTransfer
 * Description   : Configures the DMA transfer in a loop.
 *
 *END**************************************************************************/

edma_status_t EDMA_DRV_ConfigLoopTransfer(edma_chn_state_t *chn, edma_transfer_config_t *transferConfig)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(transferConfig != 0)
    DEV_ASSERT(chn != NULL);
#endif

    /* Check if the minor/major loop properties are defined */
    if (transferConfig->loopTransferConfig == NULL)
        return EDMA_STATUS_INVALID_ARGUMENT;

    /* Enable minor loop mapping */
    DMA_Type * edmaRegBase = g_edmaBase[0U];
    EDMA_HAL_SetMinorLoopMappingCmd(edmaRegBase, true);

    /* Write the configuration in the transfer control descriptor registers */
    EDMA_DRV_PushConfigToReg(chn, transferConfig);

    /* Enable interrupt when major loop count completes */
    EDMA_HAL_TCDSetIntCmd(edmaRegBase, chn->channel, true);

    return EDMA_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_DRV_ConfigScatterGatherTransfer
 * Description   : Configure eDMA for scatter/gather operation
 *
 *END**************************************************************************/
edma_status_t EDMA_DRV_ConfigScatterGatherTransfer(
                        edma_chn_state_t *chn, edma_software_tcd_t *stcd,
                        edma_transfer_size_t transferSize, uint32_t bytesOnEachRequest,
                        edma_scatter_gather_list_t *srcList, edma_scatter_gather_list_t *destList,
                        uint8_t tcdCount)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(stcd != 0)
    DEV_ASSERT(chn != NULL);
#endif

    uint8_t i, transferOffset;
    edma_software_tcd_t *stcdAddr = (edma_software_tcd_t *)STCD_ADDR(stcd);
    edma_loop_transfer_config_t loopConfig;
    edma_transfer_config_t config;

    /* Check if the value passed for 'transferSize' is valid */
    if (!EDMA_DRV_ValidTransferSize(transferSize))
    {
        return EDMA_STATUS_INVALID_ARGUMENT;
    }
    
    /* Compute the transfer offset, based on transfer size. 
     * The number of bytes transferred in each source read/destination write
     * is obtained with the following formula:
     *    source_read_size = 2^SSIZE
     *    destination_write_size = 2^DSIZE
     */
    transferOffset = 1U << ((uint8_t)transferSize);

    /* The number of bytes to be transferred on each request must
     * be a multiple of the source read/destination write size
     */
    if ((bytesOnEachRequest % transferOffset) != 0U)
    {
        return EDMA_STATUS_INVALID_ARGUMENT;
    }

    /* Clear the configuration structures before initializing them. */
    uint8_t *clearStructPtr = (uint8_t *)(&config);
    uint8_t *lastByteAddr = (uint8_t *)(&config) + sizeof(edma_transfer_config_t);
    while (clearStructPtr < lastByteAddr)
    {
        *clearStructPtr = 0;
        clearStructPtr++;
    }

    clearStructPtr = (uint8_t *)(&loopConfig);
    lastByteAddr = (uint8_t *)(&loopConfig) + sizeof(edma_loop_transfer_config_t);
    while (clearStructPtr < lastByteAddr)
    {
        *clearStructPtr = 0;
        clearStructPtr++;
    }

    /* Configure the transfer for scatter/gather mode. */
    config.srcLastAddrAdjust = 0U;
    config.destLastAddrAdjust = 0U;
    config.srcModulo = EDMA_MODULO_OFF;
    config.destModulo = EDMA_MODULO_OFF;
    config.srcTransferSize = transferSize;
    config.destTransferSize = transferSize;
    config.minorByteTransferCount = bytesOnEachRequest;
    config.interruptEnable = true;
    config.scatterGatherEnable = true;
    config.loopTransferConfig = &loopConfig;
    config.loopTransferConfig->srcOffsetEnable = false;
    config.loopTransferConfig->dstOffsetEnable = false;
    config.loopTransferConfig->minorLoopChnLinkEnable = false;
    config.loopTransferConfig->majorLoopChnLinkEnable = false;

    /* Copy scatter/gather lists to transfer configuration*/
    for (i = 0U; i < tcdCount; i++)
    {
        config.srcAddr = srcList[i].address;
        config.destAddr = destList[i].address;
        if ((srcList[i].length != destList[i].length) || (srcList[i].type != destList[i].type))
        {
            return EDMA_STATUS_INVALID_ARGUMENT;
        }
        config.loopTransferConfig->majorLoopIterationCount = srcList[i].length/bytesOnEachRequest;

        switch (srcList[i].type)
        {
            case EDMA_TRANSFER_PERIPH2MEM:
                /* Configure Source Read. */
                config.srcOffset = 0;
                /* Configure Dest Write. */
                config.destOffset = transferOffset;
                break;
            case EDMA_TRANSFER_MEM2PERIPH:
                /* Configure Source Read. */
                config.srcOffset = transferOffset;
                /* Configure Dest Write. */
                config.destOffset = 0;
                break;
            case EDMA_TRANSFER_MEM2MEM:
                /* Configure Source Read. */
                config.srcOffset = transferOffset;
                /* Configure Dest Write. */
                config.destOffset = transferOffset;
                break;
            default:
                return EDMA_STATUS_INVALID_ARGUMENT;
        }

        /* Configure the pointer to next software TCD structure; for the last one, this address should be 0 */
        if (i == tcdCount - 1)
        {
            config.scatterGatherNextDescAddr = 0U;
        }
        else
        {
            config.scatterGatherNextDescAddr = ((uint32_t) &stcdAddr[i+1]);
        }

        /* Copy configuration to software TCD structure */
        EDMA_DRV_PushConfigToSTCD(chn, &config, &stcdAddr[i]);

        /* Push the configuration for the first descriptor to registers */
        if (i == 0U)
        {
            EDMA_DRV_PushConfigToReg(chn, &config);
        }
    }

    return EDMA_STATUS_SUCCESS;

}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_DRV_StartChannel
 * Description   : Starts an eDMA channel.
 *
 *END**************************************************************************/
edma_status_t EDMA_DRV_StartChannel(edma_chn_state_t *chn)
{
    uint8_t channel = chn->channel;
    DMA_Type * edmaRegBase = g_edmaBase[0U];

    EDMA_HAL_SetDmaRequestCmd(edmaRegBase, (edma_channel_indicator_t)channel, true);

    return EDMA_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_DRV_StopChannel
 * Description   : Stops an eDMA channel.
 *
 *END**************************************************************************/
edma_status_t EDMA_DRV_StopChannel(edma_chn_state_t *chn)
{
    uint8_t channel = chn->channel;
    DMA_Type * edmaRegBase = g_edmaBase[0U];

    EDMA_HAL_SetDmaRequestCmd(edmaRegBase, (edma_channel_indicator_t)channel, false);

    return EDMA_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_DRV_PushConfigToSTCD
 * Description   : Copy the configuration to the software TCD structure.
 *
 *END**************************************************************************/
void EDMA_DRV_PushConfigToSTCD(edma_chn_state_t *chn, edma_transfer_config_t *config, edma_software_tcd_t *stcd)
{
    /* Clear the array of software TCDs passed by the user */
    EDMA_DRV_ClearSoftwareTCD(stcd);

    /* Set the software TCD fields */
    stcd->ATTR = (config->srcModulo << DMA_TCD_ATTR_SMOD_SHIFT) | (config->srcTransferSize << DMA_TCD_ATTR_SSIZE_SHIFT) |
                 (config->destModulo << DMA_TCD_ATTR_DMOD_SHIFT) | (config->destTransferSize << DMA_TCD_ATTR_DSIZE_SHIFT);
    stcd->SADDR = config->srcAddr;
    stcd->SOFF = config->srcOffset;
    stcd->NBYTES = config->minorByteTransferCount;
    stcd->SLAST = config->srcLastAddrAdjust;
    stcd->DADDR = config->destAddr;
    stcd->DOFF = config->destOffset;
    stcd->CITER = config->loopTransferConfig->majorLoopIterationCount;
    if (config->scatterGatherEnable)
    {
        stcd->DLAST_SGA = config->scatterGatherNextDescAddr;
    }
    else
    {
        stcd->DLAST_SGA = config->destLastAddrAdjust;
    }
    stcd->CSR = (config->interruptEnable << DMA_TCD_CSR_INTMAJOR_SHIFT) | (config->scatterGatherEnable << DMA_TCD_CSR_ESG_SHIFT);
    stcd->BITER = config->loopTransferConfig->majorLoopIterationCount;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_DRV_PushConfigToReg
 * Description   : Copy the configuration to the TCD registers.
 *
 *END**************************************************************************/
edma_status_t EDMA_DRV_PushConfigToReg(edma_chn_state_t *chn, edma_transfer_config_t *tcd)
{
    uint32_t channel = chn->channel;
    DMA_Type * edmaRegBase = g_edmaBase[0U];

    /* Clear TCD registers */
    EDMA_HAL_TCDClearReg(edmaRegBase, channel);

    /* Set source and destination addresses */
    EDMA_HAL_TCDSetSrcAddr(edmaRegBase, channel, tcd->srcAddr);
    EDMA_HAL_TCDSetDestAddr(edmaRegBase, channel, tcd->destAddr);

    /* Set source/destination modulo feature and transfer size */
    EDMA_HAL_TCDSetAttribute(edmaRegBase, channel, tcd->srcModulo, tcd->destModulo, tcd->srcTransferSize, tcd->destTransferSize);

    /* Set source/destination offset and last adjustment; for scatter/gather operation, destination
     * last adjustment is the address of the next TCD structure to be loaded by the eDMA engine */
    EDMA_HAL_TCDSetSrcOffset(edmaRegBase, channel, tcd->srcOffset);
    EDMA_HAL_TCDSetDestOffset(edmaRegBase, channel, tcd->destOffset);
    EDMA_HAL_TCDSetSrcLastAdjust(edmaRegBase, channel, tcd->srcLastAddrAdjust);
    if (tcd->scatterGatherEnable)
    {
        EDMA_HAL_TCDSetScatterGatherCmd(edmaRegBase, channel, true);
        EDMA_HAL_TCDSetScatterGatherLink(edmaRegBase, channel, tcd->scatterGatherNextDescAddr);
    }
    else
    {
        EDMA_HAL_TCDSetScatterGatherCmd(edmaRegBase, channel, false);
        EDMA_HAL_TCDSetDestLastAdjust(edmaRegBase, channel, tcd->destLastAddrAdjust);
    }

    /* Configure channel interrupt */
    EDMA_HAL_TCDSetIntCmd(edmaRegBase, channel, tcd->interruptEnable);

    /* If loop configuration is available, copy minor/major loop setup to registers */
    if (tcd->loopTransferConfig != NULL)
    {
        EDMA_HAL_TCDSetSrcMinorLoopOffsetCmd(edmaRegBase, channel, tcd->loopTransferConfig->srcOffsetEnable);
        EDMA_HAL_TCDSetDestMinorLoopOffsetCmd(edmaRegBase, channel, tcd->loopTransferConfig->dstOffsetEnable);
        EDMA_HAL_TCDSetMinorLoopOffset(edmaRegBase, channel, tcd->loopTransferConfig->minorLoopOffset);
        EDMA_HAL_TCDSetNbytes(edmaRegBase, channel, tcd->minorByteTransferCount);

        EDMA_HAL_TCDSetChannelMinorLink(edmaRegBase, channel, tcd->loopTransferConfig->minorLoopChnLinkNumber, tcd->loopTransferConfig->minorLoopChnLinkEnable);
        EDMA_HAL_TCDSetChannelMajorLink(edmaRegBase, channel, tcd->loopTransferConfig->majorLoopChnLinkNumber, tcd->loopTransferConfig->majorLoopChnLinkEnable);

        EDMA_HAL_TCDSetMajorCount(edmaRegBase, channel, tcd->loopTransferConfig->majorLoopIterationCount);
    }
    else
    {
        EDMA_HAL_TCDSetNbytes(edmaRegBase, channel, tcd->minorByteTransferCount);
    }

    return EDMA_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_DRV_ValidTransferSize
 * Description   : Check if the transfer size value is legal (0/1/2/4/5).
 *
 *END**************************************************************************/
static bool EDMA_DRV_ValidTransferSize(edma_transfer_size_t size)
{
    switch (size)
    {
        case EDMA_TRANSFER_SIZE_1B:
        case EDMA_TRANSFER_SIZE_2B:
        case EDMA_TRANSFER_SIZE_4B:
        case EDMA_TRANSFER_SIZE_16B:
        case EDMA_TRANSFER_SIZE_32B:
            return true;
        default:
            return false;
    }
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

