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

#include <string.h>
#include "fsl_lpspi_slave_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_interrupt_manager.h"



/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Pointer to runtime state structure.*/
extern void * g_lpspiStatePtr[LPSPI_INSTANCE_COUNT];

/* Table of SPI FIFO sizes per instance (this is calculated in the master init). */
extern uint32_t g_lpspiFifoSize[LPSPI_INSTANCE_COUNT];

/*******************************************************************************
 * Code
 ******************************************************************************/
lpspi_status_t LPSPI_DRV_SlaveInit(uint32_t instance,
                               lpspi_slave_state_t * lpspiState,
                               const lpspi_slave_config_t * slaveConfig)
{
    if (g_lpspiStatePtr[instance] != NULL)
    {
        return LPSPI_STATUS_INITIALIZED;
    }
    LPSPI_Type * base = g_lpspiBase[instance];
    /* Clear lpspi state */
    memset(lpspiState, 0, sizeof(lpspi_slave_state_t));
    lpspiState->bitsPerFrame = slaveConfig->bitcount;
    lpspiState->bytesPerFrame = slaveConfig->bitcount/8;
    lpspiState->clkPhase = slaveConfig->clkPhase;
    lpspiState->clkPolarity = slaveConfig->clkPolarity;
    lpspiState->whichPcs = slaveConfig->whichPcs;
    lpspiState->lsbFirst = slaveConfig->lsbFirst;
    lpspiState->rxWatermark = slaveConfig->rxWatermark;
    lpspiState->txWatermark = slaveConfig->txWatermark;
    g_lpspiStatePtr[instance] = lpspiState;

    /*Enable clock gate for LPSPI*/
    /*CLOCK_SYS_EnableLpspiClock(instance);*/

    /* Configure registers */
    LPSPI_HAL_Init(base);
    LPSPI_HAL_SetFlushFifoCmd(base, true, true);
    /* Configure lpspi to slave mode */
    LPSPI_HAL_SetMasterSlaveMode(base, LPSPI_SLAVE);
    /* Set Pin settings */
    LPSPI_HAL_SetPinConfigMode(base, LPSPI_SDI_IN_SDO_OUT, LPSPI_DATA_OUT_RETAINED, true);
    /* Calculate the FIFO size for the LPSPI */
    LPSPI_HAL_GetFifoSizes(base, &(g_lpspiFifoSize[instance]), NULL);

    /* Set watermark */
    LPSPI_HAL_SetFifoWatermarks(base, slaveConfig->txWatermark, slaveConfig->rxWatermark);
    LPSPI_HAL_SetPcsPolarityMode(base, LPSPI_PCS0,slaveConfig->pcsPolarity);

    /* Init the interrupt sync object.*/
    /*
    if (OSA_SemaCreate(&lpspiState->irqSync, 0) != kStatus_OSA_Success)
    {
        return LPSPI_STATUS_ERROR;
    }
    */
     /* Write the TCR for this transfer */
    lpspi_tx_cmd_config_t txCmdCfg = {
        .frameSize = lpspiState->bitsPerFrame,
        .width = LPSPI_SINGLE_BIT_XFER,
        .txMask = false,
        .rxMask = false,
        .byteSwap = false,
        .lsbFirst = lpspiState->lsbFirst,
        .clkPhase = lpspiState->clkPhase,
        .clkPolarity = lpspiState->clkPolarity,
        .whichPcs = lpspiState->whichPcs
    };

    /* Write to the TX CMD register */
    LPSPI_HAL_SetTxCommandReg(base, &txCmdCfg);
    LPSPI_HAL_Enable(base);
    /* Enable the interrupt source */
    INT_SYS_EnableIRQ(g_lpspiIrqId[instance]);

    return LPSPI_STATUS_SUCCESS;

}


lpspi_status_t LPSPI_DRV_SlaveDeinit(uint32_t instance)
{
    /* instantiate local variable of type lpspi_master_state_t and point to global state */
    lpspi_slave_state_t * lpspiState = (lpspi_slave_state_t *)g_lpspiStatePtr[instance];
    LPSPI_Type *base = g_lpspiBase[instance];

    /* Return error if a transfer is still in progress */
    if (lpspiState->isTransferInProgress == true)
    {
        return LPSPI_STATUS_TRANSFER_IN_PROGRESS;
    }

    /* Reset the LPSPI registers to their default state, inlcuding disabling the LPSPI */
    LPSPI_HAL_Init(base);

    /* destroy the interrupt sync object.*/
    /*OSA_SemaDestroy(&lpspiState->irqSync);*/

    /* disable the interrupt*/
    INT_SYS_DisableIRQ(g_lpspiIrqId[instance]);

    /* Gate the clock for LPSPI.*/
    /*CLOCK_SYS_DisableLpspiClock(instance);*/

    /* Clear the state pointer. */
    g_lpspiStatePtr[instance] = NULL;

    return LPSPI_STATUS_SUCCESS;
}


lpspi_status_t LPSPI_DRV_SlaveTransferBlocking(uint32_t instance,
                                           const uint8_t *sendBuffer,
                                           uint8_t *receiveBuffer,
                                           uint32_t transferByteCount,
                                           uint32_t timeout)
{
    LPSPI_Type * base = g_lpspiBase[instance];
    lpspi_slave_state_t * state = (lpspi_slave_state_t *)g_lpspiStatePtr[instance];
    uint32_t LPSPItimeout = timeout;
    if ((sendBuffer == NULL) && (receiveBuffer == NULL))
    {
        return LPSPI_STATUS_INVALID_PARAMETER;
    }
    /* The number of transferred bytes should be divisible by frame size */
    if (transferByteCount % state->bytesPerFrame != 0)
    {
        return LPSPI_STATUS_INVALID_PARAMETER;
    }
    if (state->isTransferInProgress == true)
    {
        return LPSPI_STATUS_BUSY;
    }

    state->isTransferInProgress = true;
    state->isTransferBlocking = true;
    state->rxBuff = receiveBuffer;
    state->txBuff = sendBuffer;
    state->bytesTransferred = 0;
    state->txCount = transferByteCount;
    state->rxCount = transferByteCount;

    /* Clear register */
    LPSPI_HAL_SetFlushFifoCmd(base, true, true);
    LPSPI_HAL_ClearStatusFlag(base, LPSPI_ALL_STATUS);

    /* Enable interrupts for RX and TX only if it's necessary */
    if(state->txBuff != NULL)
    {
      LPSPI_HAL_SetIntMode(base,LPSPI_TX_DATA_FLAG , true);
    }
    else
    {
      state->txCount = 0;
    }
    if(state->rxBuff != NULL)
    {
        LPSPI_HAL_SetIntMode(base, LPSPI_RX_DATA_FLAG, true);
    }
    else
    {
       state->rxCount = 0;
    }

    do
    {
        LPSPItimeout --;
    }while(0 < LPSPItimeout && LPSPI_HAL_GetStatusFlag(base, LPSPI_MODULE_BUSY));

    return LPSPI_STATUS_SUCCESS;
}


lpspi_status_t LPSPI_DRV_SlaveTransfer(uint32_t instance,
                                   const uint8_t *sendBuffer,
                                   uint8_t *receiveBuffer,
                                   uint32_t transferByteCount)
{
    LPSPI_Type * base = g_lpspiBase[instance];
    lpspi_slave_state_t * state = (lpspi_slave_state_t *)g_lpspiStatePtr[instance];
    if (state->isTransferInProgress == true)
    {
        return LPSPI_STATUS_BUSY;
    }
    if ((sendBuffer == NULL) && (receiveBuffer == NULL))
    {
        return LPSPI_STATUS_INVALID_PARAMETER;
    }
    /* The number of transferred bytes should be divisible by frame size */
    if (transferByteCount % state->bytesPerFrame != 0)
    {
        return LPSPI_STATUS_INVALID_PARAMETER;
    }
    state->isTransferInProgress = true;
    state->isTransferBlocking = false;
    state->rxBuff = receiveBuffer;
    state->txBuff = sendBuffer;
    state->bytesTransferred = 0;
    state->txCount = transferByteCount;
    state->rxCount = transferByteCount;

    /* Clear register */
    LPSPI_HAL_SetFlushFifoCmd(base, true, true);
    LPSPI_HAL_ClearStatusFlag(base, LPSPI_ALL_STATUS);

    /* Enable interrupts for RX and TX only if it's necessary */
    if(state->txBuff != NULL)
    {
        LPSPI_HAL_SetIntMode(base,LPSPI_TX_DATA_FLAG , true);
    }
    else
    {
        state->txCount = 0;
    }
    if(state->rxBuff != NULL)
    {
        LPSPI_HAL_SetIntMode(base, LPSPI_RX_DATA_FLAG, true);
    }
    else
    {
        state->rxCount = 0;
    }
    return LPSPI_STATUS_SUCCESS;
}

void LPSPI_DRV_SlaveIRQHandler(uint32_t instance)
{
    LPSPI_Type * base = g_lpspiBase[instance];
    uint32_t i,j,bytes, data = 0, numberOfWords;
    uint8_t rxFifoCount, txFifoCount;
    uint8_t wordsRemained;
    lpspi_slave_state_t * state = (lpspi_slave_state_t *)g_lpspiStatePtr[instance];
    /* If it is FIFO error, clear the fifo error flag */
    if (LPSPI_HAL_GetStatusFlag(base, LPSPI_TRANSMIT_ERROR))
    {
        LPSPI_HAL_ClearStatusFlag(base, LPSPI_TRANSMIT_ERROR);
    }
    if (LPSPI_HAL_GetStatusFlag(base, LPSPI_RECEIVE_ERROR))
    {
        LPSPI_HAL_ClearStatusFlag(base, LPSPI_RECEIVE_ERROR);
    }
    bytes = (state->bytesPerFrame > 4U) ? 4U: state->bytesPerFrame;
    /* Receive data */
    if (LPSPI_HAL_GetStatusFlag(base, LPSPI_RX_DATA_FLAG) &&(state->rxCount))
    {
        rxFifoCount = LPSPI_HAL_ReadRxCount(base);
        wordsRemained = (state->rxCount+bytes-1)/bytes;
        numberOfWords = (wordsRemained > (rxFifoCount)? rxFifoCount: wordsRemained);
        for (i = 0; i < numberOfWords; i++)
        {
            data = LPSPI_HAL_ReadData(base);
            for (j = 0; ((j < bytes)&&(state->rxCount != 0)); j ++)
            {
                * state->rxBuff = (data >> (j * 8)) & 0xFFU;
                state->rxCount --;
                state->rxBuff ++;
            }
        }
    }

    /* If need to transfer, write data or receive data */
    if (LPSPI_HAL_GetStatusFlag(base, LPSPI_TX_DATA_FLAG) && (state->txCount))
    {
        txFifoCount = LPSPI_HAL_ReadTxCount(base);
        /* Next instruction will get the number of words which will be added in
         * tx FIFO. The size of the hardware rx buffer is 4 words. The max number
         * of words transferred from software buffer to hardware buffers is
         * 4 - current number of words (hardware buffer).
         */
        wordsRemained = (state->txCount+bytes-1)/bytes;
        numberOfWords = (wordsRemained > (g_lpspiFifoSize[instance]-txFifoCount)? g_lpspiFifoSize[instance]-txFifoCount: wordsRemained);
        for (i = 0; i < numberOfWords; i ++)
        {
            if (state->txBuff)
            {
                data = 0;
                for (j = 0; ((j < bytes)&&(state->txCount != 0)); j ++ )
                {
                    data |= (uint32_t)(*state->txBuff) << (8 * j);
                    state->bytesTransferred ++;
                    state->txCount --;
                    state->txBuff ++;
                }
                LPSPI_HAL_WriteData(base, data);
            }
        }
    }
    /* If all bytes are sent disable interrupt TDF */
    if (state->txCount == 0)
    {
        LPSPI_HAL_SetIntMode(base, LPSPI_TX_DATA_FLAG, false);
    }
    /* If all bytes are received disable interrupt RDF */
    if (state->rxCount == 0)
    {
        LPSPI_HAL_SetIntMode(base, LPSPI_RX_DATA_FLAG, false);
    }
    if ((state->rxCount == 0) && (state->txCount == 0))
    {
        state->isTransferInProgress = false;
    }
    /* OLD VERSION */
    /* If transfer finished, post the semaphore */
    #if 0
    if ((state->txCount == 0) && (state->rxCount == 0))
    {
        if (state->isTransferBlocking)
        {
            OSA_SemaPost(&state->irqSync);
        }
        state->isTransferInProgress = false;
        LPSPI_HAL_SetIntMode(base, LPSPI_TX_DATA_FLAG, false);
        LPSPI_HAL_SetIntMode(base, LPSPI_RX_DATA_FLAG, false);
    }
    #endif
}

lpspi_status_t LPSPI_DRV_SlaveAbortTransfer(uint32_t instance)
{
    LPSPI_Type * base = g_lpspiBase[instance];
    lpspi_slave_state_t * state = (lpspi_slave_state_t *)g_lpspiStatePtr[instance];

    /*OSA_SemaPost(&state->irqSync);*/
    /* Disable interrupts */
    state->isTransferInProgress = false;
    LPSPI_HAL_SetIntMode(base, LPSPI_TX_DATA_FLAG, false);
    LPSPI_HAL_SetIntMode(base, LPSPI_RX_DATA_FLAG, false);
    /* Clear FIFOs */
    LPSPI_HAL_SetFlushFifoCmd(base, true, true);
    return LPSPI_STATUS_SUCCESS;
}

lpspi_status_t LPSPI_DRV_SlaveGetTransferStatus(uint32_t instance,uint32_t * bytesTransferred)
{
    /* instantiate local variable of type lpspi_master_state_t and point to global state */
    lpspi_slave_state_t * lpspiState = (lpspi_slave_state_t *)g_lpspiStatePtr[instance];

    /* Fill in the bytes transferred.*/
    if (bytesTransferred)
    {
        *bytesTransferred = lpspiState->bytesTransferred;
    }

    return (lpspiState->isTransferInProgress ? LPSPI_STATUS_BUSY : LPSPI_STATUS_SUCCESS);

}



