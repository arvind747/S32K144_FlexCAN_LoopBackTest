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
#include <string.h>
#include "fsl_lpspi_master_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_interrupt_manager.h"



/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Pointer to runtime state structure.*/
extern void * g_lpspiStatePtr[LPSPI_INSTANCE_COUNT];

/* Table of SPI FIFO sizes per instance (this is calculated in the master init). */
extern uint32_t g_lpspiFifoSize[LPSPI_INSTANCE_COUNT];

/* Flag needed when continuous PCS mode is used and an extra TCR write is needed in the ISR to
 * de-assert PCS.
 */
uint32_t s_writeTcrInIsr = 0;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static lpspi_status_t LPSPI_DRV_MasterStartTransfer(uint32_t instance,
                                                    const lpspi_master_config_t * spiConfig,
                                                    const uint8_t * sendBuffer,
                                                    uint8_t * receiveBuffer,
                                                    size_t transferByteCount);

static void LPSPI_DRV_MasterCompleteTransfer(uint32_t instance);

static void LPSPI_DRV_MasterFillupTxFifo(uint32_t instance);

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_DRV_MasterInit
 * Description   : Initializes a LPSPI instance for interrupt driven master mode operation.
 *
 * This function uses an interrupt-driven method for transferring data.
 * In this function, the term "spiConfig" is used to indicate the SPI device for which the LPSPI
 * master is communicating.
 * This function initializes the run-time state structure to track the ongoing
 * transfers, un-gates the clock to the LPSPI module, resets the LPSPI module,
 * configures the IRQ state structure, enables the module-level interrupt to the core, and
 * enables the LPSPI module.
 * This is an example to set up the lpspi_master_state_t and call the
 * LPSPI_DRV_MasterInit function by passing in these parameters:
 *    lpspi_master_state_t lpspiMasterState; <- the user  allocates memory for this structure
 *    lpspi_master_config_t spiConfig; Can declare more configs for use in transfer functions
 *    spiConfig.bitsPerSec = 500000;
 *    spiConfig.whichPcs = LPSPI_PCS0;
 *    spiConfig.pcsPolarity = LPSPI_ACTIVE_LOW;
 *    spiConfig.isPcsContinuous = false;
 *    spiConfig.bitCount = 16;
 *    spiConfig.clkPhase = LPSPI_CLOCK_PHASE_1ST_EDGE;
 *    spiConfig.clkPolarity = LPSPI_ACTIVE_HIGH;
 *    spiConfig.lsbFirst= false;
 *    spiConfig.rxWatermark = 2;
 *    spiConfig.txWatermark = 2;
 *    LPSPI_DRV_MasterInit(masterInstance, &lpspiMasterState, &spiConfig);
 *
 *END**************************************************************************/
lpspi_status_t LPSPI_DRV_MasterInit(uint32_t instance, lpspi_master_state_t * lpspiState,
                                    const lpspi_master_config_t * spiConfig)
{
    lpspi_status_t errorCode = LPSPI_STATUS_SUCCESS;
    LPSPI_Type *base = g_lpspiBase[instance];

     /* Check to make sure parameter pointer is not NULL or tx Buffer is NULL*/
    if ((lpspiState == NULL) || (spiConfig == NULL))
    {
        return LPSPI_STATUS_INVALID_PARAMETER;
    }

    /* Check to see if the LPSPI master instance is already initialized */
    if (g_lpspiStatePtr[instance])
    {
        return LPSPI_STATUS_INITIALIZED;
    }

    /* Save runtime structure pointers so irq handler can point to the correct state structure */
    g_lpspiStatePtr[instance] = lpspiState;

    /* Reset the LPSPI registers to their default state */
    LPSPI_HAL_Init(base);

    /* Set for master or slave mode */
    LPSPI_HAL_SetMasterSlaveMode(base, LPSPI_MASTER);

    /* Set Pin configuration such that SDO=out and SDI=in */
    LPSPI_HAL_SetPinConfigMode(base, LPSPI_SDI_IN_SDO_OUT, LPSPI_DATA_OUT_RETAINED, true);

    /* Calculate the FIFO size for the LPSPI */
    LPSPI_HAL_GetFifoSizes(base, &(g_lpspiFifoSize[instance]), NULL);

    /* Configure the SPI bus including the baud rate and return if there is an error */
    errorCode = LPSPI_DRV_MasterConfigureBus(instance, spiConfig, NULL);
    if (errorCode != LPSPI_STATUS_SUCCESS)
    {
        return errorCode;
    }

    /* Init the interrupt sync object.*/
    /*
    if (OSA_SemaCreate(&lpspiState->irqSync, 0) != kStatus_OSA_Success)
    {
        errorCode = LPSPI_STATUS_ERROR;
    }*/

    /* enable the interrupt */
    INT_SYS_EnableIRQ(g_lpspiIrqId[instance]);

    /* Finally, enable LPSPI */
    LPSPI_HAL_Enable(base);

    return errorCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_DRV_MasterDeinit
 * Description   : Shuts down a LPSPI instance.
 *
 * This function resets the LPSPI peripheral, gates its clock, and disables the interrupt to
 * the core.  It first checks to see if a transfer is in progress and if so returns an error
 * status.
 *
 *END**************************************************************************/
lpspi_status_t LPSPI_DRV_MasterDeinit(uint32_t instance)
{
    /* instantiate local variable of type lpspi_master_state_t and point to global state */
    lpspi_master_state_t * lpspiState = (lpspi_master_state_t *)g_lpspiStatePtr[instance];
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

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_DRV_MasterSetDelay
 * Description   : Configures the LPSPI master mode bus timing delay options.
 *
 * This function involves the LPSPI module's delay options to
 * "fine tune" some of the signal timings and match the timing needs of a slower peripheral device.
 * This is an optional function that can be called after the LPSPI module has been initialized for
 * master mode. The timings are adjusted in terms of cycles of the baud rate clock.
 * The bus timing delays that can be adjusted are listed below:
 *
 * SCK to PCS Delay: Adjustable delay option between the last edge of SCK to the de-assertion
 *                   of the PCS signal.
 *
 * PCS to SCK Delay: Adjustable delay option between the assertion of the PCS signal to the
 *                   first SCK edge.
 *
 * Delay between Transfers: Adjustable delay option between the de-assertion of the PCS signal for
 *                          a frame to the assertion of the PCS signal for the next frame.
 *
 *END**************************************************************************/
lpspi_status_t LPSPI_DRV_MasterSetDelay(uint32_t instance, lpspi_delay_type_t whichDelay,
                                        uint32_t delayInNanoSec, uint32_t * calculatedDelay)
{
    /* instantiate local variable of type lpspi_master_state_t and point to global state */
    lpspi_master_state_t * lpspiState = (lpspi_master_state_t *)g_lpspiStatePtr[instance];
    LPSPI_Type *base = g_lpspiBase[instance];
    lpspi_status_t errorCode = LPSPI_STATUS_SUCCESS;

    /* First, per the spec, we need to disable the LPSPI module before setting the delay */
    if (LPSPI_HAL_Disable(base) != LPSPI_STATUS_SUCCESS)
    {
        /* If error is returned, the LPSPI is busy */
        return LPSPI_STATUS_ERROR;
    }

    if (LPSPI_HAL_CalculateDelay(base, whichDelay, lpspiState->lpspiSrcClk, delayInNanoSec,
                                 calculatedDelay) != LPSPI_STATUS_SUCCESS)
    {
        /* If error is returned, the LPSPI is busy */
        return LPSPI_STATUS_ERROR;
    }

    /* Now, re-enable the LPSPI module */
    LPSPI_HAL_Enable(base);

    /* If the desired delay exceeds the capability of the device, alert the user */
    if (*calculatedDelay < delayInNanoSec)
    {
        errorCode = LPSPI_STATUS_OUT_OF_RANGE;
    }

    return errorCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_DRV_MasterConfigureBus
 * Description   : Configures the LPSPI port physical parameters to access a device on the bus when
 *                 the LSPI instance is configured for interrupt operation.
 *
 * In this function, the term "spiConfig" is used to indicate the SPI device for which the LPSPI
 * master is communicating. This is an optional function as the spiConfig parameters are
 * normally configured in the initialization function or the transfer functions, where these various
 * functions would call the configure bus function.
 * Hence the user has three options to configure the SPI device parameters: pass in the initial
 * spiConfig parameters to the init function then pass in any changes to the spiConfig to the
 * desired transfer function (see LPSPI_DRV_MasterTransferBlocking or LPSPI_DRV_MasterTransfer) or
 * pass in changes to the spiConfig to the LPSPI_DRV_MasterConfigureBus function.
 * The user can pass in a different spiConfig structure to the transfer function which contains
 * the parameters for the SPI bus to allow for communication to a different SPI device
 * (the transfer function then calls this function). However, the user also has the option to call
 * this function directly especially to get the calculated baud rate, at which point they may pass
 * in NULL for the spiConfig structure in the transfer function (assuming they have called this
 * configure bus function first). This is an example to set up the lpspi_master_config_t structure
 * to call the LPSPI_DRV_MasterConfigureBus function by passing in these parameters:
 *    lpspi_master_config_t spiConfig1;  You can also declare spiConfig2, spiConfig3, etc
 *    spiConfig1.bitsPerSec = 500000;
 *    spiConfig1.whichPcs = LPSPI_PCS0;
 *    spiConfig1.pcsPolarity = LPSPI_ACTIVE_LOW;
 *    spiConfig1.isPcsContinuous = false;
 *    spiConfig1.bitCount = 16;
 *    spiConfig1.clkPhase = LPSPI_CLOCK_PHASE_1ST_EDGE;
 *    spiConfig1.clkPolarity = LPSPI_ACTIVE_HIGH;
 *    spiConfig1.lsbFirst= false;
 *    spiConfig.rxWatermark = 2;
 *    spiConfig.txWatermark = 2;
 *
 *END**************************************************************************/
lpspi_status_t LPSPI_DRV_MasterConfigureBus(uint32_t instance,
                                            const lpspi_master_config_t * spiConfig,
                                            uint32_t * calculatedBaudRate)
{
    /* instantiate local variable of type lpspi_master_state_t and point to global state */
    lpspi_master_state_t * lpspiState = (lpspi_master_state_t *)g_lpspiStatePtr[instance];
    LPSPI_Type *base = g_lpspiBase[instance];
    uint32_t baudRate;

    /* The Transmit Command Register (TCR) Prescale value is calculated as part of the baud rate
       calculation. The value is stored in the run-time state structure for later programming
       in the TCR. */
    uint32_t tcrPrescaleValue;

    /* Check the bitcount to make sure it falls within the boundary conditions */
    if ((spiConfig->bitcount < 8) || (spiConfig->bitcount > 4096))
    {
        return LPSPI_STATUS_BITCOUNT_OUT_OF_RANGE;
    }

    /* Check the FIFO watermark settings, if watermark >= FIFO size, report out-of-range error */
    if ((spiConfig->txWatermark >= g_lpspiFifoSize[instance]) ||
        (spiConfig->rxWatermark >= g_lpspiFifoSize[instance]))
    {
        return LPSPI_STATUS_WATERMARK_OUT_OF_RANGE;
    }

    /* Configure the run-time state struct with the spiConfig parameters. Note that these
       parameters will be used later in the transfer function when programming the transmit
       command register */
    lpspiState->bitsPerFrame = spiConfig->bitcount;
    lpspiState->whichPcs = spiConfig->whichPcs;
    lpspiState->isPcsContinuous = spiConfig->isPcsContinuous;
    lpspiState->lpspiSrcClk = spiConfig->lpspiSrcClk;
    lpspiState->clkPhase = spiConfig->clkPhase;
    lpspiState->clkPolarity = spiConfig->clkPolarity;
    lpspiState->lsbFirst = spiConfig->lsbFirst;
    lpspiState->rxWatermark = spiConfig->rxWatermark;

    /* Configure the desired PCS polarity */
    LPSPI_HAL_SetPcsPolarityMode(base, spiConfig->whichPcs, spiConfig->pcsPolarity);

    /* Set the TX and RX FIFO watermarks */
    LPSPI_HAL_SetFifoWatermarks(base, spiConfig->txWatermark, spiConfig->rxWatermark);

    /* First, per the spec, we need to disable the LPSPI module before setting the delay */
    if (LPSPI_HAL_Disable(base) != LPSPI_STATUS_SUCCESS)
    {
        /* If error is returned, the LPSPI is busy */
        return LPSPI_STATUS_ERROR;
    }

    /* Set up the baud rate */
    baudRate = LPSPI_HAL_SetBaudRate(base, spiConfig->bitsPerSec, lpspiState->lpspiSrcClk,
                                     &tcrPrescaleValue);

    /* Now, re-enable the LPSPI module */
    LPSPI_HAL_Enable(base);

    /* If the baud rate return is "0", it means there was an error */
    if (baudRate == 0)
    {
        return LPSPI_STATUS_ERROR;
    }

    /* If the user wishes to know the calculated baud rate, then pass it back */
    if (calculatedBaudRate != NULL)
    {
        *calculatedBaudRate = baudRate;
    }

    /* Store the TCR prescale value to the run-time state struct for later programming the TCR */
    lpspiState->tcrPrescaler = tcrPrescaleValue;

    return LPSPI_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_DRV_MasterTransferBlocking
 * Description   : Performs an interrupt driven blocking SPI master mode transfer.
 *
 * This function simultaneously sends and receives data on the SPI bus, as SPI is naturally
 * a full-duplex bus. The function does not return until the transfer is complete.
 * This function allows the user to optionally pass in a SPI configuration structure which
 * allows the user to change the SPI bus attributes in conjunction with initiating a SPI transfer.
 * The difference between passing in the SPI configuration structure here as opposed to the
 * configure bus function is that the configure bus function returns the calculated baud rate where
 * this function does not. The user can also call the configure bus function prior to the transfer
 * in which case the user would simply pass in a NULL to the transfer function's device structure
 * parameter.
 *
 *END**************************************************************************/
lpspi_status_t LPSPI_DRV_MasterTransferBlocking(uint32_t instance,
                                                const lpspi_master_config_t * spiConfig,
                                                const uint8_t * sendBuffer,
                                                uint8_t * receiveBuffer,
                                                size_t transferByteCount,
                                                uint32_t timeout)
{
    /* instantiate local variable of type lpspi_master_state_t and point to global state */
    lpspi_master_state_t * lpspiState = (lpspi_master_state_t *)g_lpspiStatePtr[instance];
    LPSPI_Type *base = g_lpspiBase[instance];
    lpspi_status_t error = LPSPI_STATUS_SUCCESS;
    uint32_t LPSPItimeout = timeout;

    /* If the transfer count is zero, then return immediately.*/
    if (transferByteCount == 0)
    {
        return error;
    }

    /* As this is a synchronous transfer, set up the sync status variable*/
   /* osa_status_t syncStatus;*/

    /* fill in members of the run-time state struct*/
    lpspiState->isTransferBlocking = true; /* Indicates this is a blocking transfer */

    /* start the transfer process, if it returns an error code, return this back to user */
    error = LPSPI_DRV_MasterStartTransfer(instance, spiConfig, sendBuffer, receiveBuffer,
                                          transferByteCount);
    if (error != LPSPI_STATUS_SUCCESS)
    {
        /* The transfer is complete.*/
        lpspiState->isTransferInProgress = false;
        /* Disable interrupt requests*/
        /*LPSPI_CLR_IER(base, LPSPI_IER_TDIE_MASK|LPSPI_IER_RDIE_MASK);*/
        LPSPI_HAL_SetIntMode(base, LPSPI_TX_DATA_FLAG, false);
        LPSPI_HAL_SetIntMode(base, LPSPI_RX_DATA_FLAG, false);

        return error;
    }

    /* As this is a synchronous transfer, wait until the transfer is complete.*/
    do
    {
        LPSPItimeout --;
    }while(0 < LPSPItimeout && LPSPI_HAL_GetStatusFlag(base, LPSPI_MODULE_BUSY));

    /* If a timeout occurs, stop the transfer by setting the isTransferInProgress to false and
     * disabling interrupts, then return the timeout error status.
     */
    if (0 == LPSPItimeout)
    {
        /* The transfer is complete.*/
        lpspiState->isTransferInProgress = false;
        /* Disable interrupt requests*/
        /*LPSPI_CLR_IER(base, LPSPI_IER_TDIE_MASK|LPSPI_IER_RDIE_MASK);*/
        LPSPI_HAL_SetIntMode(base, LPSPI_TX_DATA_FLAG, false);
        LPSPI_HAL_SetIntMode(base, LPSPI_RX_DATA_FLAG, false);

        error = LPSPI_STATUS_TIMEOUT;
    }
    /* Transfer completed - clear blocking flag */
    lpspiState->isTransferBlocking = false;
    return error;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_DRV_MasterTransfer
 * Description   : Performs an interrupt driven non-blocking SPI master mode transfer.
 *
 * This function simultaneously sends and receives data on the SPI bus, as SPI is naturally
 * a full-duplex bus. The function returns immediately after initiating the transfer. The user
 * needs to check whether the transfer is complete using the LPSPI_DRV_MasterGetTransferStatus
 * function.
 * This function allows the user to optionally pass in a SPI configuration structure which
 * allows the user to change the SPI bus attributes in conjunction with initiating a SPI transfer.
 * The difference between passing in the SPI configuration structure here as opposed to the
 * configure bus function is that the configure bus function returns the calculated baud rate where
 * this function does not. The user can also call the configure bus function prior to the transfer
 * in which case the user would simply pass in a NULL to the transfer function's device structure
 * parameter.
 *
 *END**************************************************************************/
lpspi_status_t LPSPI_DRV_MasterTransfer(uint32_t instance,
                                        const lpspi_master_config_t * spiConfig,
                                        const uint8_t * sendBuffer,
                                        uint8_t * receiveBuffer,
                                        size_t transferByteCount)
{
    /* instantiate local variable of type lpspi_master_state_t and point to global state */
    lpspi_master_state_t * lpspiState = (lpspi_master_state_t *)g_lpspiStatePtr[instance];
    lpspi_status_t error = LPSPI_STATUS_SUCCESS;

    /* If the transfer count is zero, then return immediately.*/
    if (transferByteCount == 0)
    {
        return LPSPI_STATUS_SUCCESS;
    }

    /* fill in members of the run-time state struct*/
    lpspiState->isTransferBlocking = false; /* Indicates this is not a blocking transfer */

    /* start the transfer process, if it returns an error code, return this back to user */
    error = LPSPI_DRV_MasterStartTransfer(instance, spiConfig, sendBuffer, receiveBuffer,
                                          transferByteCount);
    if (error != LPSPI_STATUS_SUCCESS)
    {
        return error;
    }

    /* Else, return immediately as this is an async transfer */
    return LPSPI_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_DRV_MasterGetTransferStatus
 * Description   : Returns whether the previous interrupt driven transfer is completed.
 *
 * When performing an a-sync (non-blocking) transfer, the user can call this function to ascertain
 * the state of the current transfer: in progress (or busy) or complete (success).
 * In addition, if the transfer is still in progress, the user can get the number of words that
 * have been transferred up to now.
 *
 *END**************************************************************************/
lpspi_status_t LPSPI_DRV_MasterGetTransferStatus(uint32_t instance, uint32_t * bytesTransferred)
{
    /* instantiate local variable of type lpspi_master_state_t and point to global state */
    lpspi_master_state_t * lpspiState = (lpspi_master_state_t *)g_lpspiStatePtr[instance];

    /* Fill in the bytes transferred.*/
    if (bytesTransferred)
    {
        *bytesTransferred = lpspiState->bytesTransferred;
    }

    return (lpspiState->isTransferInProgress ? LPSPI_STATUS_BUSY : LPSPI_STATUS_SUCCESS);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_DRV_MasterAbortTransfer
 * Description   : Terminates an interrupt driven asynchronous transfer early.
 *
 * During an a-sync (non-blocking) transfer, the user has the option to terminate the transfer early
 * if the transfer is still in progress.
 *
 *END**************************************************************************/
lpspi_status_t LPSPI_DRV_MasterAbortTransfer(uint32_t instance)
{
    /* instantiate local variable of type lpspi_master_state_t and point to global state */
    lpspi_master_state_t * lpspiState = (lpspi_master_state_t *)g_lpspiStatePtr[instance];

    /* Check if a transfer is running.*/
    if (!lpspiState->isTransferInProgress)
    {
        return LPSPI_STATUS_NO_TRANSFER_IN_PROGRESS;
    }

    /* Stop the running transfer.*/
    LPSPI_DRV_MasterCompleteTransfer(instance);

    return LPSPI_STATUS_SUCCESS;
}

/*!
 * @brief Initiate (start) a transfer. This is not a public API as it is called from other
 *  driver functions
 */
static lpspi_status_t LPSPI_DRV_MasterStartTransfer(uint32_t instance,
                                                    const lpspi_master_config_t * spiConfig,
                                                    const uint8_t * sendBuffer,
                                                    uint8_t * receiveBuffer,
                                                    size_t transferByteCount)
{
    /* instantiate local variable of type dspi_master_state_t and point to global state */
    lpspi_master_state_t * lpspiState = (lpspi_master_state_t *)g_lpspiStatePtr[instance];
    LPSPI_Type *base = g_lpspiBase[instance];
    lpspi_status_t error;

    /* Check that we're not busy.*/
    if (LPSPI_HAL_GetStatusFlag(base, LPSPI_MODULE_BUSY))
    {
        return LPSPI_STATUS_BUSY;
    }
    /* Check if send buffer is not NULL */
    if(sendBuffer == NULL)
    {
    	return LPSPI_STATUS_INVALID_PARAMETER;
    }
    /* Configure bus for this device. If NULL is passed, we assume the caller has
     * preconfigured the bus and doesn't wish to re-configure it again for this transfer.
     * Do nothing for calculatedBaudRate. If the user wants to know the calculatedBaudRate
     * then they can call this function separately.
     */
    if (spiConfig)
    {
        error = LPSPI_DRV_MasterConfigureBus(instance, spiConfig, NULL);
        if (error != LPSPI_STATUS_SUCCESS)
        {
            return error;
        }
    }

    /* Save information about the transfer for use by the ISR.*/
    lpspiState->isTransferInProgress = true;

    LPSPI_HAL_SetFlushFifoCmd(base, true, true);

    /* No need to check for error return since we know we are not passing an invalid parameter */
    LPSPI_HAL_ClearStatusFlag(base, LPSPI_ALL_STATUS);

    /* Calculate the bytes/frame for lpspiState->bytesPerFrame */
    if (lpspiState->bitsPerFrame % 8)
    {
        lpspiState->bytesPerFrame = (lpspiState->bitsPerFrame)/8 + 1;
    }
    else
    {
        lpspiState->bytesPerFrame = (lpspiState->bitsPerFrame)/8;
    }

    /* Align the transferByteCount to the bytesPerFrame as follows:
     * If TransferByteCount is less than the bytePerFrame, then set the transferByteCount
     * to be the number of bytesPerFrame.
     * If the transferByteCount is not a multiple of the bytesPerFrame, increase the
     * transfeByteCount to the next multiple of the bytesPerFrame.
     */
    if (transferByteCount < lpspiState->bytesPerFrame)
    {
        transferByteCount = lpspiState->bytesPerFrame;
    }
    if (transferByteCount%(lpspiState->bytesPerFrame))
    {
        transferByteCount = transferByteCount + lpspiState->bytesPerFrame -
                            transferByteCount%(lpspiState->bytesPerFrame);
    }

    /* If the bytes/frame > 4, then the frame count needs to be calculated so that we can
     * transfer the data on a frame-by-frame basis. The total transfer byte count is
     * the bytes/frame multiplied by the frame count.  Therefore, the driver uses a combination
     * of the frame count and tx/rx byte counts (which are equated to the bytes/frame) to keep
     * track of the transfer.
     * If the bytes/frame is <= 4, then the frame count is zeroed out the same as the driver
     * uses the tx/rx byte counts to keep track of the transfer.
     */
    if (lpspiState->bytesPerFrame > 4)
    {
        /* The transfer count = #frames * #bytes/frame */
        lpspiState->txFrameCnt = transferByteCount/(lpspiState->bytesPerFrame);
        lpspiState->rxFrameCnt = lpspiState->txFrameCnt;
        /* Set TX and RX Count to the frame size since we're transferring on a frame basis */
        lpspiState->txCount = lpspiState->bytesPerFrame;
        lpspiState->rxCount = lpspiState->bytesPerFrame;
    }
    else
    {
        lpspiState->txFrameCnt = 0;  /* Set to 0 since this is not used for the transfer */
        lpspiState->rxFrameCnt = 0;  /* Set to 0 since this is not used for the transfer */
        /* Set TX and RX Count to the transfer byte count */
        lpspiState->txCount = transferByteCount;
        lpspiState->rxCount = transferByteCount;
    }

    /* Fill out the other members of the run-time state structure */
    lpspiState->txBuff = (const uint8_t *)sendBuffer;
    lpspiState->rxBuff = (uint8_t *)receiveBuffer;
    lpspiState->bytesTransferred = 0;

    /* Write the TCR for this transfer */
    lpspi_tx_cmd_config_t txCmdCfg = {
        .frameSize = lpspiState->bitsPerFrame,
        .width = LPSPI_SINGLE_BIT_XFER,
        .txMask = false,
        .rxMask = false,
        .contCmd = false,
        .contTransfer = lpspiState->isPcsContinuous,
        .byteSwap = false,
        .lsbFirst = lpspiState->lsbFirst,
        .whichPcs = lpspiState->whichPcs,
        .preDiv = lpspiState->tcrPrescaler,
        .clkPhase = lpspiState->clkPhase,
        .clkPolarity = lpspiState->clkPolarity
    };

    /* Mask the RX if no buffer is passed in */
    if (lpspiState->rxBuff == NULL)
    {
        txCmdCfg.rxMask = true;
        lpspiState->rxCount = 0; /* Since we're not receiving, set rxCount to 0 */
    }
    else
    {
    /* Enable RDF interrupt if RX buffer is not NULL */
        LPSPI_HAL_SetIntMode(base, LPSPI_RX_DATA_FLAG, true);
    }
    /* Write to the TX CMD register */
    LPSPI_HAL_SetTxCommandReg(base, &txCmdCfg);

    /* Fill up the TX FIFO */
    LPSPI_DRV_MasterFillupTxFifo(instance);

    /* Enable the TDF and RDF interrupt. */
    LPSPI_HAL_SetIntMode(base, LPSPI_TX_DATA_FLAG, true);
    return LPSPI_STATUS_SUCCESS;
}

/*!
 * @brief Fill up the TX FIFO with data.
 * This function fills up the TX FIFO with data based on the bytes/frame.
 * This is not a public API as it is called from other driver functions.
 */
static void LPSPI_DRV_MasterFillupTxFifo(uint32_t instance)
{
    /* instantiate local variable of type dspi_master_state_t and point to global state */
    lpspi_master_state_t * lpspiState = (lpspi_master_state_t *)g_lpspiStatePtr[instance];
    LPSPI_Type *base = g_lpspiBase[instance];
    uint32_t wordToSend = 0;
    uint32_t txCnt, rxCnt;

    /* Architectural note: When developing the TX FIFO fill functionality, it was found that to
     * achieve more efficient run-time performance, it was better to first check the bits/frame
     * setting and then proceed with the FIFO fill management process, rather than to clutter the
     * FIFO fill process with continual checks of the bits/frame setting.
     */

    /* Fill the TX FIFO based on the size of the frame. For frame sizes > 4, this needs to be
     * handled differently.
     */
    if (lpspiState->bytesPerFrame <= 4U)
    {
        /* Get the current remaining tx and rx byte counts */
        txCnt = lpspiState->txCount;
        /* If no RX Buffer, equate rxCnt to txCnt to keep the tx while loop going */
        if (lpspiState->rxBuff == NULL)
        {
            rxCnt = txCnt;
        }
        else
        {
            rxCnt = lpspiState->rxCount;
        }

        /* For frame size of 4 bytes */
        /* Make sure the difference in remaining TX and RX byte counts does not exceed FIFO depth
         * and that the number of TX FIFO entries does not exceed the FIFO depth
         */
        if (lpspiState->bytesPerFrame == 4)
        {
            while(((rxCnt - txCnt)/4 < g_lpspiFifoSize[instance]) &&
                  (LPSPI_HAL_ReadTxCount(base) < g_lpspiFifoSize[instance]))
            {
                if (lpspiState->txBuff)
                {
                    wordToSend = *(lpspiState->txBuff);
                    ++lpspiState->txBuff;
                    wordToSend |= (unsigned)(*(lpspiState->txBuff)) << 8U;
                    ++lpspiState->txBuff;
                    wordToSend |= (unsigned)(*(lpspiState->txBuff)) << 16U;
                    ++lpspiState->txBuff;
                    wordToSend |= (unsigned)(*(lpspiState->txBuff)) << 24U;
                    ++lpspiState->txBuff;
                }
                lpspiState->txCount -= 4; /* Decrease the remaining TX byte count by 4 */
                lpspiState->bytesTransferred += 4;

                /* Write the data word entry to TX FIFO */
                LPSPI_HAL_WriteData(base, wordToSend);

                /* If there is no more data to send then break */
                if (lpspiState->txCount == 0)
                {
                    /* If PCS is continuous, update TCR to de-assert PCS */
                    if (lpspiState->isPcsContinuous)
                    {
                        /* Only write to the TCR if the FIFO has room */
                        if (LPSPI_HAL_ReadTxCount(base) < g_lpspiFifoSize[instance])
                        {
                            LPSPI_HAL_ClearContCBit(base);
                        }
                        /* Else, set a global flag to tell the ISR to do write to the TCR */
                        else
                        {
                            s_writeTcrInIsr = 1;
                        }
                    }
                    break;
                }
                else
                {
                    /* Store the LPSPI state struct volatile member variables into temporary
                     * non-volatile variables to allow for MISRA compliant calculations
                     */
                    txCnt = lpspiState->txCount;
                    /* If no RX Buffer, re-equate rxCnt to txCnt to keep the tx while loop going */
                    if (lpspiState->rxBuff == NULL)
                    {
                        rxCnt = txCnt;
                    }
                }
            }
        }
        /* For frame size of 3 bytes */
        /* Make sure the difference in remaining TX and RX byte counts does not exceed FIFO depth
         * and that the number of TX FIFO entries does not exceed the FIFO depth
         */
        else if (lpspiState->bytesPerFrame == 3)
        {
            while(((rxCnt - txCnt)/3 < g_lpspiFifoSize[instance]) &&
                  (LPSPI_HAL_ReadTxCount(base) < g_lpspiFifoSize[instance]))
            {
                if (lpspiState->txBuff)
                {
                    wordToSend = *(lpspiState->txBuff);
                    ++lpspiState->txBuff;
                    wordToSend |= (unsigned)(*(lpspiState->txBuff)) << 8U;
                    ++lpspiState->txBuff;
                    wordToSend |= (unsigned)(*(lpspiState->txBuff)) << 16U;
                    ++lpspiState->txBuff;
                }
                lpspiState->txCount -= 3; /* Decrease the remaining TX byte count by 3 */
                lpspiState->bytesTransferred += 3;

                /* Write the data word entry to TX FIFO */
                LPSPI_HAL_WriteData(base, wordToSend);

                /* If there is no more data to send then break */
                if (lpspiState->txCount == 0)
                {
                    /* If PCS is continuous, update TCR to de-assert PCS */
                    if (lpspiState->isPcsContinuous)
                    {
                        /* Only write to the TCR if the FIFO has room */
                        if (LPSPI_HAL_ReadTxCount(base) < g_lpspiFifoSize[instance])
                        {
                            LPSPI_HAL_ClearContCBit(base);
                        }
                        /* Else, set a global flag to tell the ISR to do write to the TCR */
                        else
                        {
                            s_writeTcrInIsr = 1;
                        }
                    }
                    break;
                }
                else
                {
                    /* Store the LPSPI state struct volatile member variables into temporary
                     * non-volatile variables to allow for MISRA compliant calculations
                     */
                    txCnt = lpspiState->txCount;
                    /* If no RX Buffer, re-equate rxCnt to txCnt to keep the tx while loop going */
                    if (lpspiState->rxBuff == NULL)
                    {
                        rxCnt = txCnt;
                    }
                }
            }
        }
        /* For frame size of 2 bytes */
        /* Make sure the difference in remaining TX and RX byte counts does not exceed FIFO depth
         * and that the number of TX FIFO entries does not exceed the FIFO depth
         */
        else if (lpspiState->bytesPerFrame == 2)
        {
            while(((rxCnt - txCnt)/2 < g_lpspiFifoSize[instance]) &&
                  (LPSPI_HAL_ReadTxCount(base) < g_lpspiFifoSize[instance]))
            {
                if (lpspiState->txBuff)
                {
                    wordToSend = *(lpspiState->txBuff);
                    ++lpspiState->txBuff;
                    wordToSend |= (unsigned)(*(lpspiState->txBuff)) << 8U;
                    ++lpspiState->txBuff;
                }
                lpspiState->txCount -= 2; /* Decrease the remaining TX byte count by 2 */
                lpspiState->bytesTransferred += 2;

                /* Write the data word entry to TX FIFO */
                LPSPI_HAL_WriteData(base, wordToSend);

                /* If there is no more data to send then break */
                if (lpspiState->txCount == 0)
                {
                    /* If PCS is continuous, update TCR to de-assert PCS */
                    if (lpspiState->isPcsContinuous)
                    {
                        /* Only write to the TCR if the FIFO has room */
                        if (LPSPI_HAL_ReadTxCount(base) < g_lpspiFifoSize[instance])
                        {
                            LPSPI_HAL_ClearContCBit(base);
                        }
                        /* Else, set a global flag to tell the ISR to do write to the TCR */
                        else
                        {
                            s_writeTcrInIsr = 1;
                        }
                    }
                    break;
                }
                else
                {
                    /* Store the LPSPI state struct volatile member variables into temporary
                     * non-volatile variables to allow for MISRA compliant calculations
                     */
                    txCnt = lpspiState->txCount;
                    /* If no RX Buffer, re-equate rxCnt to txCnt to keep the tx while loop going */
                    if (lpspiState->rxBuff == NULL)
                    {
                        rxCnt = txCnt;
                    }
                }
            }
        }
        /* For frame size of 1 byte */
        /* Make sure the difference in remaining TX and RX byte counts does not exceed FIFO depth
         * and that the number of TX FIFO entries does not exceed the FIFO depth
         */
        else
        {
            while(((rxCnt - txCnt) < g_lpspiFifoSize[instance]) &&
                  (LPSPI_HAL_ReadTxCount(base) < g_lpspiFifoSize[instance]))
            {
                if (lpspiState->txBuff)
                {
                    wordToSend = *(lpspiState->txBuff);
                    ++lpspiState->txBuff;
                }
                lpspiState->txCount -= 1; /* Decrease the remaining TX byte count by 1 */
                lpspiState->bytesTransferred += 1;

                /* Write the data word entry to TX FIFO */
                LPSPI_HAL_WriteData(base, wordToSend);

                /* If there is no more data to send then break */
                if (lpspiState->txCount == 0)
                {
                    /* If PCS is continuous, update TCR to de-assert PCS */
                    if (lpspiState->isPcsContinuous)
                    {
                        /* Only write to the TCR if the FIFO has room */
                        if (LPSPI_HAL_ReadTxCount(base) < g_lpspiFifoSize[instance])
                        {
                            LPSPI_HAL_ClearContCBit(base);
                        }
                        /* Else, set a global flag to tell the ISR to do write to the TCR */
                        else
                        {
                            s_writeTcrInIsr = 1;
                        }
                    }
                    break;
                }
                else
                {
                    /* Store the LPSPI state struct volatile member variables into temporary
                     * non-volatile variables to allow for MISRA compliant calculations
                     */
                    txCnt = lpspiState->txCount;
                    /* If no RX Buffer, re-equate rxCnt to txCnt to keep the tx while loop going */
                    if (lpspiState->rxBuff == NULL)
                    {
                        rxCnt = txCnt;
                    }
                }
            }
        }
    }
    else /* FOR HANDLING FRAMES SIZES > 32 BITS */
    {
        /* Set txCnt to the FIFO size minus any data that may be in the RXFIFO and rxCnt to the
         * RXFIFO size, this way we don't overrun the RX FIFO.
         * If RxBUFF=NULL, we can leave this as is as RXCOUNT should always be 0 in this case.
         */
        txCnt = g_lpspiFifoSize[instance] - LPSPI_HAL_ReadRxCount(base);
        rxCnt = g_lpspiFifoSize[instance];

        /* Make sure the difference in remaining TX and RX word counts does not exceed FIFO depth
         * and that the number of TX FIFO entries does not exceed the FIFO depth
         */
        while(((rxCnt - txCnt) < g_lpspiFifoSize[instance]) &&
              (LPSPI_HAL_ReadTxCount(base) < g_lpspiFifoSize[instance]))
        {
            /* If there is 1 byte left in the frame */
            if (lpspiState->txCount == 1)
            {
                if (lpspiState->txBuff)
                {
                    wordToSend = *(lpspiState->txBuff);
                    ++lpspiState->txBuff;
                }
                lpspiState->txCount -= 1; /* Decrease the remaining TX byte count by 1 */
                lpspiState->bytesTransferred += 1;
            }
            /* If there are 2 bytes left in the frame */
            else if (lpspiState->txCount == 2)
            {
                if (lpspiState->txBuff)
                {
                    wordToSend = *(lpspiState->txBuff);
                    ++lpspiState->txBuff;
                    wordToSend |= (unsigned)(*(lpspiState->txBuff)) << 8U;
                    ++lpspiState->txBuff;
                }
                lpspiState->txCount -= 2; /* Decrease the remaining TX byte count by 2 */
                lpspiState->bytesTransferred += 2;
            }
            /* If there are 3 bytes left in the frame */
            else if (lpspiState->txCount == 3)
            {
                if (lpspiState->txBuff)
                {
                    wordToSend = *(lpspiState->txBuff);
                    ++lpspiState->txBuff;
                    wordToSend |= (unsigned)(*(lpspiState->txBuff)) << 8U;
                    ++lpspiState->txBuff;
                    wordToSend |= (unsigned)(*(lpspiState->txBuff)) << 16U;
                    ++lpspiState->txBuff;
                }
                lpspiState->txCount -= 3; /* Decrease the remaining TX byte count by 3 */
                lpspiState->bytesTransferred += 3;
            }
            /* If writing all four bytes to the TX FIFO */
            else
            {
                if (lpspiState->txBuff)
                {
                    wordToSend = *(lpspiState->txBuff);
                    ++lpspiState->txBuff;
                    wordToSend |= (unsigned)(*(lpspiState->txBuff)) << 8U;
                    ++lpspiState->txBuff;
                    wordToSend |= (unsigned)(*(lpspiState->txBuff)) << 16U;
                    ++lpspiState->txBuff;
                    wordToSend |= (unsigned)(*(lpspiState->txBuff)) << 24U;
                    ++lpspiState->txBuff;
                }
                lpspiState->txCount -= 4; /* Decrease the remaining TX byte count by 4 */
                lpspiState->bytesTransferred += 4;
            }

            /* Write the data word entry to TX FIFO */
            LPSPI_HAL_WriteData(base, wordToSend);

            /* If there is no more data to send, decrease frame count */
            if (lpspiState->txCount == 0)
            {
                lpspiState->txFrameCnt--;
                /* If there are still more frames to send, then reset the TX byte count */
                if (lpspiState->txFrameCnt)
                {
                    lpspiState->txCount = lpspiState->bytesPerFrame;
                }
                /* Else break if no more frames to send (note the case of continuous PCS) */
                else
                {
                    /* If PCS is continuous, update TCR to de-assert PCS */
                    if (lpspiState->isPcsContinuous)
                    {
                        /* Only write to the TCR if the FIFO has room */
                        if (LPSPI_HAL_ReadTxCount(base) < g_lpspiFifoSize[instance])
                        {
                            LPSPI_HAL_ClearContCBit(base);
                        }
                        /* Else, set global flag for ISR to write the TCR when FIFO has room */
                        else
                        {
                            s_writeTcrInIsr = 1;
                        }
                    }
                    break;
                }
            }

            /* Decrement txCnt for the while loop */
            txCnt--;

            /* If no RX Buffer, equate rxCnt to txCnt to keep the tx while loop going */
            if (lpspiState->rxBuff == NULL)
            {
                rxCnt = txCnt;
            }
        } /* while loop for filling tx fifo for frame sizes > 32-bits */
    } /* End if-else for filling tx fifo based on frame sizes < 32-bits or > 32-bits */
}

/*!
 * @brief Finish up a transfer.
 * Cleans up after a transfer is complete. Interrupts are disabled, and the LPSPI module
 * is disabled. This is not a public API as it is called from other driver functions.
 */
static void LPSPI_DRV_MasterCompleteTransfer(uint32_t instance)
{
    /* instantiate local variable of type dspi_master_state_t and point to global state */
    lpspi_master_state_t * lpspiState = (lpspi_master_state_t *)g_lpspiStatePtr[instance];
    LPSPI_Type *base = g_lpspiBase[instance];

    /* The transfer is complete.*/
    lpspiState->isTransferInProgress = false;

    /* Disable (clear) interrupt requests */
    LPSPI_HAL_SetRxDmaCmd(base, false);
    LPSPI_HAL_SetTxDmaCmd(base, false);
    LPSPI_HAL_SetIntMode(base, LPSPI_RX_DATA_FLAG, false);
    LPSPI_HAL_SetIntMode(base, LPSPI_TX_DATA_FLAG, false);
    LPSPI_HAL_SetIntMode(base, LPSPI_TRANSFER_COMPLETE, false);
    LPSPI_HAL_ClearStatusFlag(base, LPSPI_TRANSFER_COMPLETE);
    /*
    if (lpspiState->isTransferBlocking)
    {
    */
        /* Signal the synchronous completion object */
    /*
         OSA_SemaPost(&lpspiState->irqSync);
    }
    */
}

/*!
 * @brief Interrupt handler for LPSPI master mode.
 * This handler uses the buffers stored in the lpspi_master_state_t structs to transfer data.
 * This is not a public API as it is called whenever an interrupt occurs.
 */
void LPSPI_DRV_MasterIRQHandler(uint32_t instance)
{
    /* Instantiate local variable of type dspi_master_state_t and point to global state */
    lpspi_master_state_t * lpspiState = (lpspi_master_state_t *)g_lpspiStatePtr[instance];
    LPSPI_Type *base = g_lpspiBase[instance];
    uint32_t readData;  /* variable to store word read from RX FIFO */

    /* RECEIVE IRQ handler: Check read buffer only if there are remaining bytes to read. */
    if((lpspiState->rxCount) && (lpspiState->rxBuff != NULL))
    {
        /* Read the RXFIFO while there is data in the FIFO or until the RX byte count reaches 0 */
        while((LPSPI_HAL_ReadRxCount(base)) && (lpspiState->rxCount))
        {
            readData = LPSPI_HAL_ReadData(base);

            if (lpspiState->bytesPerFrame <= 4U)
            {
                if (lpspiState->bytesPerFrame == 4)
                {
                    *lpspiState->rxBuff = readData; /* Write first data byte to rxBuff */
                    ++lpspiState->rxBuff;     /* increment to next data byte */
                    *lpspiState->rxBuff = readData >> 8; /* Write second data byte */
                    ++lpspiState->rxBuff;    /* increment to next data byte */
                    *lpspiState->rxBuff = readData >> 16; /* Write third data byte */
                    ++lpspiState->rxBuff;    /* increment to next data byte */
                    *lpspiState->rxBuff = readData >> 24; /* Write fourth data byte */
                    ++lpspiState->rxBuff;    /* increment to next data byte */
                    lpspiState->rxCount -= 4;
                }
                else if (lpspiState->bytesPerFrame == 3)
                {
                    *lpspiState->rxBuff = readData; /* Write first data byte to rxBuff */
                    ++lpspiState->rxBuff;     /* increment to next data byte */
                    *lpspiState->rxBuff = readData >> 8; /* Write second data byte */
                    ++lpspiState->rxBuff;    /* increment to next data byte */
                    *lpspiState->rxBuff = readData >> 16; /* Write third data byte */
                    ++lpspiState->rxBuff;    /* increment to next data byte */
                    lpspiState->rxCount -= 3;
                }
                else if (lpspiState->bytesPerFrame == 2)
                {
                    *lpspiState->rxBuff = readData; /* Write first data byte to rxBuff */
                    ++lpspiState->rxBuff;     /* increment to next data byte */
                    *lpspiState->rxBuff = readData >> 8; /* Write second data byte */
                    ++lpspiState->rxBuff;    /* increment to next data byte */
                    lpspiState->rxCount -= 2;
                }
                else
                {
                    *lpspiState->rxBuff = readData; /* Write first data byte to rxBuff */
                    ++lpspiState->rxBuff;     /* increment to next data byte */
                    lpspiState->rxCount -= 1;
                }
            }
            else /* FOR HANDLING FRAMES SIZES > 32 BITS */
            {
                /* If there is 1 byte left in the frame */
                if (lpspiState->rxCount == 1)
                {
                    *lpspiState->rxBuff = readData;
                    ++lpspiState->rxBuff;
                    lpspiState->rxCount -= 1;
                }
                /* If there are 2 bytes left in the frame */
                else if (lpspiState->rxCount == 2)
                {
                    *lpspiState->rxBuff = readData;
                    ++lpspiState->rxBuff;
                    *lpspiState->rxBuff = readData >> 8;
                    ++lpspiState->rxBuff;
                    lpspiState->rxCount -= 2;
                }
                /* If there are 3 bytes left in the frame */
                else if (lpspiState->rxCount == 3)
                {
                    *lpspiState->rxBuff = readData;
                    ++lpspiState->rxBuff;
                    *lpspiState->rxBuff = readData >> 8;
                    ++lpspiState->rxBuff;
                    *lpspiState->rxBuff = readData >> 16;
                    ++lpspiState->rxBuff;
                    lpspiState->rxCount -= 3;
                }
                /* If reading all four bytes from the RX FIFO */
                else
                {
                    *lpspiState->rxBuff = readData;
                    ++lpspiState->rxBuff;
                    *lpspiState->rxBuff = readData >> 8;
                    ++lpspiState->rxBuff;
                    *lpspiState->rxBuff = readData >> 16;
                    ++lpspiState->rxBuff;
                    *lpspiState->rxBuff = readData >> 24;
                    ++lpspiState->rxBuff;
                    lpspiState->rxCount -= 4;
                }

                /* If rxCount is 0, then check to see if there are more bytes to transfer by
                 * checking the rxFrameCnt. If there are more bytes to transfer, reset the rxCount
                 * to the original bytes/frame else leave it as 0 where we'll break out of while
                 * loop.
                 */
                if (lpspiState->rxCount == 0)
                {
                    lpspiState->rxFrameCnt--;
                    if (lpspiState->rxFrameCnt)
                    {
                        lpspiState->rxCount = lpspiState->bytesPerFrame;
                    }
                }
            }
        }
    }  /* End of RECEIVE IRQ handler */

    /* TRANSMIT IRQ handler: if there is more data to send, then fill the TX FIFO. Else, check to
     * see if this is a continuous PCS transfer, and if so see if conditions are met to de-assert
     * PCS here in the ISR.
     */
    if (lpspiState->txCount)
    {
        LPSPI_DRV_MasterFillupTxFifo(instance);
    }
    else
    {
        /* Disable TX flag. Software buffer is empty.*/
        LPSPI_HAL_SetIntMode(base, LPSPI_TX_DATA_FLAG, false);
        LPSPI_HAL_SetIntMode(base, LPSPI_TRANSFER_COMPLETE, true);
        /* If PCS is continuous, update TCR to de-assert PCS */
        if ((lpspiState->isPcsContinuous) && (s_writeTcrInIsr))
        {
            /* Only write the TCR if there is room in the TX FIFO */
            if (LPSPI_HAL_ReadTxCount(base) < g_lpspiFifoSize[instance])
            {
                LPSPI_HAL_ClearContCBit(base);
                s_writeTcrInIsr = 0; /* Clear this so that we don't keep doing this needlessly */
            }
        }
        /* Check if we're done with this transfer.*/
        if ((lpspiState->txCount == 0) && (lpspiState->rxCount == 0) && LPSPI_HAL_GetStatusFlag(base, LPSPI_TRANSFER_COMPLETE))
        {
            LPSPI_DRV_MasterCompleteTransfer(instance);
        }
    }
}


/*******************************************************************************
 * EOF
 ******************************************************************************/

