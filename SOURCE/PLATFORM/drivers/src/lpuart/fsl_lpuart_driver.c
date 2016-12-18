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

#include "fsl_lpuart_driver.h"
#include "fsl_interrupt_manager.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* Pointer to lpuart runtime state structure */
extern void * g_lpuartStatePtr[LPUART_INSTANCE_COUNT];

/*******************************************************************************
 * Private Functions
 ******************************************************************************/
static lpuart_status_t LPUART_DRV_StartSendData(uint32_t instance,
                                                const uint8_t * txBuff,
                                                uint32_t txSize);
static void LPUART_DRV_CompleteSendData(uint32_t instance);
static lpuart_status_t LPUART_DRV_StartReceiveData(uint32_t instance,
                                                   uint8_t * rxBuff,
                                                   uint32_t rxSize);
static void LPUART_DRV_CompleteReceiveData(uint32_t instance);
/*******************************************************************************
 * Code
 ******************************************************************************/
/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_DRV_Init
 * Description   : This function initializes a LPUART instance for operation.
 * This function will initialize the run-time state structure to keep track of
 * the on-going transfers, ungate the clock to the LPUART module, initialize the
 * module to user defined settings and default settings, configure the IRQ state
 * structure and enable the module-level interrupt to the core, and enable the
 * LPUART module transmitter and receiver.
 * The following is an example of how to set up the lpuart_state_t and the
 * lpuart_user_config_t parameters and how to call the LPUART_DRV_Init function
 * by passing in these parameters:
 *    lpuart_user_config_t lpuartConfig;
 *    lpuartConfig.baudRate = 9600;
 *    lpuartConfig.bitCountPerChar = LPUART_8_BITS_PER_CHAR;
 *    lpuartConfig.parityMode = LPUART_PARITY_DISABLED;
 *    lpuartConfig.stopBitCount = LPUART_ONE_STOP_BIT;
 *    lpuart_state_t lpuartState;
 *    LPUART_DRV_Init(instance, &lpuartState, &lpuartConfig);
 *
 *END**************************************************************************/
lpuart_status_t LPUART_DRV_Init(uint32_t instance, lpuart_state_t * lpuartStatePtr,
                                const lpuart_user_config_t * lpuartUserConfig)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(lpuartStatePtr && lpuartUserConfig);
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    uint32_t lpuartSourceClock;
    clock_names_t instanceClkName = g_lpuartClkNames[instance];
    LPUART_Type * base = g_lpuartBase[instance];

    /* Get the LPUART clock as configured in the clock manager */
    CLOCK_SYS_GetFreq(instanceClkName, &lpuartSourceClock);

    /* Exit if current instance is clock gated. */
    if (lpuartSourceClock == 0)
    {
        return LPUART_STAT_CLOCK_GATED_OFF;
    }

    /* Exit if current instance is already initialized. */
    if (g_lpuartStatePtr[instance])
    {
        return LPUART_STAT_INITIALIZED;
    }

    /* Clear the state struct for this instance. */
    uint8_t *clearStructPtr = (uint8_t *)lpuartStatePtr;
    uint8_t *lastByteAddr = (uint8_t *)lpuartStatePtr + sizeof(lpuart_state_t);
    while(clearStructPtr < lastByteAddr)
    {
        *clearStructPtr = 0;
        clearStructPtr++;
    }

    /* Save runtime structure pointer.*/
    g_lpuartStatePtr[instance] = lpuartStatePtr;

    /* initialize the LPUART instance */
    LPUART_HAL_Init(base);

    /* initialize the parameters of the LPUART config structure with desired data */
    LPUART_HAL_SetBaudRate(base, lpuartSourceClock, lpuartUserConfig->baudRate);
    LPUART_HAL_SetBitCountPerChar(base, lpuartUserConfig->bitCountPerChar);
    LPUART_HAL_SetParityMode(base, lpuartUserConfig->parityMode);
    LPUART_HAL_SetStopBitCount(base, lpuartUserConfig->stopBitCount);

    /* finally, enable the LPUART transmitter and receiver */
    LPUART_HAL_SetTransmitterCmd(base, true);
    LPUART_HAL_SetReceiverCmd(base, true);

    /* Install LPUART irq handler */
    INT_SYS_InstallHandler(g_lpuartRxTxIrqId[instance], g_lpuartIsr[instance], (isr_t*) 0);

    /* Enable LPUART interrupt. */
    INT_SYS_EnableIRQ(g_lpuartRxTxIrqId[instance]);

    return LPUART_STAT_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_DRV_Deinit
 * Description   : This function shuts down the UART by disabling interrupts and
 *                 transmitter/receiver.
 *
 *END**************************************************************************/
lpuart_status_t LPUART_DRV_Deinit(uint32_t instance)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    clock_names_t instanceClkName = g_lpuartClkNames[instance];
    uint32_t lpuartSourceClock;
    CLOCK_SYS_GetFreq(instanceClkName, &lpuartSourceClock);

    /* Exit if current instance is already de-initialized or is gated.*/
    if ((!g_lpuartStatePtr[instance]) || (lpuartSourceClock == 0))
    {
        return LPUART_STAT_FAIL;
    }

    LPUART_Type * base = g_lpuartBase[instance];

    /* Wait until the data is completely shifted out of shift register */
    while (!LPUART_HAL_GetStatusFlag(base, LPUART_TX_COMPLETE)) {}

    /* Disable LPUART interrupt. */
    INT_SYS_DisableIRQ(g_lpuartRxTxIrqId[instance]);

    /* Restore default handler. */
    INT_SYS_InstallHandler(g_lpuartRxTxIrqId[instance], DefaultISR, (isr_t*) 0);

    /* disable tx and rx */
    LPUART_HAL_SetTransmitterCmd(base, false);
    LPUART_HAL_SetReceiverCmd(base, false);

    /* Clear our saved pointer to the state structure */
    g_lpuartStatePtr[instance] = NULL;

    return LPUART_STAT_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_DRV_InstallRxCallback
 * Description   : Install receive data callback function.
 *
 *END**************************************************************************/
lpuart_rx_callback_t LPUART_DRV_InstallRxCallback(uint32_t instance,
                                                lpuart_rx_callback_t function,
                                                uint8_t * rxBuff,
                                                void * callbackParam,
                                                bool alwaysEnableRxIrq)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    LPUART_Type * base = g_lpuartBase[instance];
    lpuart_state_t * lpuartState = (lpuart_state_t *)g_lpuartStatePtr[instance];

    lpuart_rx_callback_t currentCallback = lpuartState->rxCallback;
    lpuartState->rxCallback = function;
    lpuartState->rxCallbackParam = callbackParam;
    lpuartState->rxBuff = rxBuff;

    /* Enable/Disable the receive data full interrupt */
    lpuartState->isRxBusy = true;
    LPUART_HAL_SetIntMode(base, LPUART_INT_RX_DATA_REG_FULL, alwaysEnableRxIrq);

    return currentCallback;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_DRV_InstallTxCallback
 * Description   : Install transmit data callback function, pass in NULL pointer
 * as callback will uninstall.
 *
 *END**************************************************************************/
lpuart_tx_callback_t LPUART_DRV_InstallTxCallback(uint32_t instance,
                                                  lpuart_tx_callback_t function,
                                                  uint8_t * txBuff,
                                                  void * callbackParam)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    lpuart_state_t * lpuartState = (lpuart_state_t *)g_lpuartStatePtr[instance];

    lpuart_tx_callback_t currentCallback = lpuartState->txCallback;
    lpuartState->txCallback = function;
    lpuartState->txCallbackParam = callbackParam;
    lpuartState->txBuff = txBuff;

    return currentCallback;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_DRV_SendDataBlocking
 * Description   : This function sends data out through the LPUART module using
 * blocking method. The function does not return until the transmit is complete.
 *
 *END**************************************************************************/
lpuart_status_t LPUART_DRV_SendDataBlocking(uint32_t instance,
                                            const uint8_t * txBuff,
                                            uint32_t txSize,
                                            uint32_t timeout)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(txBuff);
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    lpuart_state_t * lpuartState = (lpuart_state_t *)g_lpuartStatePtr[instance];
    LPUART_Type * base = g_lpuartBase[instance];
    lpuart_status_t retVal = LPUART_STAT_SUCCESS;

    /* Indicates this is a blocking transaction. */
    lpuartState->isTxBlocking = true;

    /* Start the transmission process */
    retVal = LPUART_DRV_StartSendData(instance, txBuff, txSize);

    if (retVal == LPUART_STAT_SUCCESS)
    {
        /* Wait until the transmit is complete. */
        do
        {
            timeout--;
        } while(timeout > 0 && lpuartState->isTxBusy);

        if (timeout == 0)
        {
            /* Disable transmission complete interrupt */
            LPUART_HAL_SetIntMode(base, LPUART_INT_TX_DATA_REG_EMPTY, false);

            /* Update the information of the module driver state */
            lpuartState->isTxBusy = false;

            retVal = LPUART_STAT_TIMEOUT;
        }
    }

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_DRV_SendData
 * Description   : This function sends data out through the LPUART module using
 * non-blocking method. The function will return immediately after calling this
 * function.
 *
 *END**************************************************************************/
lpuart_status_t LPUART_DRV_SendData(uint32_t instance,
                                    const uint8_t * txBuff,
                                    uint32_t txSize)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(txBuff);
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    lpuart_status_t retVal = LPUART_STAT_SUCCESS;
    lpuart_state_t * lpuartState = (lpuart_state_t *)g_lpuartStatePtr[instance];

    /* Indicates this is a non-blocking transaction. */
    lpuartState->isTxBlocking = false;

    /* Start the transmission process */
    retVal = LPUART_DRV_StartSendData(instance, txBuff, txSize);

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_DRV_GetTransmitStatus
 * Description   : This function returns whether the previous LPUART transmit has
 * finished. When performing non-blocking transmit, the user can call this
 * function to ascertain the state of the current transmission:
 * in progress (or busy) or complete (success). In addition, if the transmission
 * is still in progress, the user can obtain the number of words that have been
 * currently transferred.
 *
 *END**************************************************************************/
lpuart_status_t LPUART_DRV_GetTransmitStatus(uint32_t instance, uint32_t * bytesRemaining)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    lpuart_state_t * lpuartState = (lpuart_state_t *)g_lpuartStatePtr[instance];
    lpuart_status_t retVal = LPUART_STAT_SUCCESS;
    uint32_t txSize = lpuartState->txSize;

    /* Fill in the bytes transferred. */
    if (bytesRemaining)
    {
        *bytesRemaining = txSize;
    }

    if (txSize)
    {
        retVal = LPUART_STAT_TX_BUSY;
    }

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_DRV_AbortSendingData
 * Description   : This function terminates an non-blocking LPUART transmission
 * early. During a non-blocking LPUART transmission, the user has the option to
 * terminate the transmission early if the transmission is still in progress.
 *
 *END**************************************************************************/
lpuart_status_t LPUART_DRV_AbortSendingData(uint32_t instance)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    lpuart_state_t * lpuartState = (lpuart_state_t *)g_lpuartStatePtr[instance];

    /* Check if a transfer is running. */
    if (!lpuartState->isTxBusy)
    {
        return LPUART_STAT_NO_TRANSMIT_IN_PROGRESS;
    }

    /* Stop the running transfer. */
    LPUART_DRV_CompleteSendData(instance);

    return LPUART_STAT_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_DRV_ReceiveDataBlocking
 * Description   : This function receives data from LPUART module using blocking
 * method, the function does not return until the receive is complete.
 *
 *END**************************************************************************/
lpuart_status_t LPUART_DRV_ReceiveDataBlocking(uint32_t instance,
                                               uint8_t * rxBuff,
                                               uint32_t rxSize,
                                               uint32_t timeout)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(rxBuff);
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    lpuart_state_t * lpuartState = (lpuart_state_t *)g_lpuartStatePtr[instance];
    LPUART_Type * base = g_lpuartBase[instance];
    lpuart_status_t retVal = LPUART_STAT_SUCCESS;

    /* Indicates this is a blocking transaction. */
    lpuartState->isRxBlocking = true;

    retVal = LPUART_DRV_StartReceiveData(instance, rxBuff, rxSize);

    if (retVal == LPUART_STAT_SUCCESS)
    {
        /* Wait until the receive is complete. */
        do
        {
            timeout--;
        } while(timeout > 0 && lpuartState->isRxBusy);

        if (timeout == 0)
        {
            /* Disable receive data full and rx overrun interrupt. */
            LPUART_HAL_SetIntMode(base, LPUART_INT_RX_DATA_REG_FULL, false);
            LPUART_HAL_SetIntMode(base, LPUART_INT_RX_OVERRUN, false);

            /* Update the information of the module driver state */
            lpuartState->isRxBusy = false;

            retVal = LPUART_STAT_TIMEOUT;
        }
    }

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_DRV_ReceiveData
 * Description   : This function receives data from LPUART module using
 * non-blocking method.  This function returns immediately after initiating the
 * receive function. The application has to get the receive status to see when
 * the receive is complete. In other words, after calling non-blocking get
 * function, the application must get the receive status to check if receive
 * is completed or not.
 *
 *END**************************************************************************/
lpuart_status_t LPUART_DRV_ReceiveData(uint32_t instance,
                                       uint8_t * rxBuff,
                                       uint32_t rxSize)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(rxBuff);
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    lpuart_status_t retVal = LPUART_STAT_SUCCESS;
    lpuart_state_t * lpuartState = (lpuart_state_t *)g_lpuartStatePtr[instance];

    /* Indicates this is a non-blocking transaction. */
    lpuartState->isRxBlocking = false;

    retVal = LPUART_DRV_StartReceiveData(instance, rxBuff, rxSize);

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_DRV_GetReceiveStatus
 * Description   : This function returns whether the previous LPUART receive is
 * complete. When performing a non-blocking receive, the user can call this
 * function to ascertain the state of the current receive progress: in progress
 * or complete. In addition, if the receive is still in progress, the user can
 * obtain the number of words that have been currently received.
 *
 *END**************************************************************************/
lpuart_status_t LPUART_DRV_GetReceiveStatus(uint32_t instance,
                                            uint32_t * bytesRemaining)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    lpuart_state_t * lpuartState = (lpuart_state_t *)g_lpuartStatePtr[instance];
    lpuart_status_t retVal = LPUART_STAT_SUCCESS;
    uint32_t rxSize = lpuartState->rxSize;

    /* Fill in the bytes transferred. */
    if (bytesRemaining)
    {
        *bytesRemaining = rxSize;
    }

    if (rxSize)
    {
        retVal = LPUART_STAT_RX_BUSY;
    }

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_DRV_AbortReceivingData
 * Description   : Terminates a non-blocking receive early.
 *
 *END**************************************************************************/
lpuart_status_t LPUART_DRV_AbortReceivingData(uint32_t instance)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    lpuart_state_t * lpuartState = (lpuart_state_t *)g_lpuartStatePtr[instance];

    /* Check if a transfer is running. */
    if (!lpuartState->isRxBusy)
    {
        return LPUART_STAT_NO_RECEIVE_IN_PROGRESS;
    }

    /* Stop the running transfer. */
    LPUART_DRV_CompleteReceiveData(instance);

    return LPUART_STAT_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_DRV_IRQHandler
 * Description   : Interrupt handler for LPUART.
 * This handler uses the buffers stored in the lpuart_state_t structs to transfer
 * data. This is not a public API as it is called by IRQ whenever an interrupt
 * occurs.
 *
 *END**************************************************************************/
void LPUART_DRV_IRQHandler(uint32_t instance)
{
    lpuart_state_t * lpuartState = (lpuart_state_t *)g_lpuartStatePtr[instance];
    LPUART_Type * base = g_lpuartBase[instance];

    /* Exit the ISR if no transfer is happening for this instance. */
    if ((!lpuartState->isTxBusy) && (!lpuartState->isRxBusy))
    {
        return;
    }

    /* Handle receive data full interrupt */
    if((LPUART_HAL_GetIntMode(base, LPUART_INT_RX_DATA_REG_FULL)) && (LPUART_HAL_GetStatusFlag(base, LPUART_RX_DATA_REG_FULL)))
    {
        /* Get data and put in receive buffer  */
        LPUART_HAL_Getchar(base, lpuartState->rxBuff);

        /* Invoke callback if there is one */
        if (lpuartState->rxCallback != NULL)
        {
            lpuartState->rxCallback(instance, lpuartState);
        }
        else
        {
            ++lpuartState->rxBuff;
            --lpuartState->rxSize;

            /* Check and see if this was the last byte received */
            if (lpuartState->rxSize == 0)
            {
                LPUART_DRV_CompleteReceiveData(instance);
            }
        }
    }

    /* Handle transmitter data register empty interrupt */
    if((LPUART_HAL_GetIntMode(base, LPUART_INT_TX_DATA_REG_EMPTY)) && (LPUART_HAL_GetStatusFlag(base, LPUART_TX_DATA_REG_EMPTY)))
    {
        /* check to see if there are any more bytes to send */
        if (lpuartState->txSize)
        {
            /* Transmit the data */
            LPUART_HAL_Putchar(base, *(lpuartState->txBuff));

            /* Invoke callback if there is one */
            if (lpuartState->txCallback != NULL)
            {
                /* The callback MUST set the txSize to 0 if the
                 * transmit is ended.*/
                lpuartState->txCallback(instance, lpuartState);
            }
            else
            {
                ++lpuartState->txBuff;
                --lpuartState->txSize;
            }

            /* Check and see if this was the last byte */
            if (lpuartState->txSize == 0)
            {
                /* Complete transfer, will disable tx interrupt */
                LPUART_DRV_CompleteSendData(instance);
            }
        }
    }

    /* Handle receive overrun interrupt */
    if (LPUART_HAL_GetStatusFlag(base, LPUART_RX_OVERRUN))
    {
        /* Clear the flag, OR the rxDataRegFull will not be set any more */
        LPUART_HAL_ClearStatusFlag(base, LPUART_RX_OVERRUN);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_DRV_StartSendData
 * Description   : Initiate (start) a transmit by beginning the process of
 * sending data and enabling the interrupt.
 * This is not a public API as it is called from other driver functions.
 *
 *END**************************************************************************/
static lpuart_status_t LPUART_DRV_StartSendData(uint32_t instance,
                                                const uint8_t * txBuff,
                                                uint32_t txSize)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    LPUART_Type * base = g_lpuartBase[instance];
    lpuart_state_t * lpuartState = (lpuart_state_t *)g_lpuartStatePtr[instance];

    /* Check it's not busy transmitting data from a previous function call */
    if (lpuartState->isTxBusy)
    {
        return LPUART_STAT_TX_BUSY;
    }

    if (txSize == 0U)
    {
        return LPUART_STAT_NO_DATA_TO_DEAL;
    }

    /* initialize the module driver state structure */
    lpuartState->txBuff = txBuff;
    lpuartState->txSize = txSize;
    lpuartState->isTxBusy = true;

    /* enable transmission complete interrupt */
    LPUART_HAL_SetIntMode(base, LPUART_INT_TX_DATA_REG_EMPTY, true);

    return LPUART_STAT_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_DRV_CompleteSendData
 * Description   : Finish up a transmit by completing the process of sending
 * data and disabling the interrupt.
 * This is not a public API as it is called from other driver functions.
 *
 *END**************************************************************************/

static void LPUART_DRV_CompleteSendData(uint32_t instance)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    LPUART_Type * base = g_lpuartBase[instance];
    lpuart_state_t * lpuartState = (lpuart_state_t *)g_lpuartStatePtr[instance];

    /* Disable transmission complete interrupt */
    LPUART_HAL_SetIntMode(base, LPUART_INT_TX_DATA_REG_EMPTY, false);

    /* Update the information of the module driver state */
    lpuartState->isTxBusy = false;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_DRV_StartReceiveData
 * Description   : Initiate (start) a receive by beginning the process of
 * receiving data and enabling the interrupt.
 * This is not a public API as it is called from other driver functions.
 *
 *END**************************************************************************/
static lpuart_status_t LPUART_DRV_StartReceiveData(uint32_t instance,
                                                   uint8_t * rxBuff,
                                                   uint32_t rxSize)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    lpuart_state_t * lpuartState = (lpuart_state_t *)g_lpuartStatePtr[instance];
    LPUART_Type * base = g_lpuartBase[instance];

    /* Check it's not busy receiving data from a previous function call */
    if ((lpuartState->isRxBusy) && (!lpuartState->rxCallback))
    {
        return LPUART_STAT_RX_BUSY;
    }

    if (rxSize == 0U)
    {
        return LPUART_STAT_NO_DATA_TO_DEAL;
    }

    /* Initialize the module driver state struct to indicate transfer in progress
     * and with the buffer and byte count data. */
    lpuartState->isRxBusy = true;
    lpuartState->rxBuff = rxBuff;
    lpuartState->rxSize = rxSize;

    /* Enable the receive data overrun interrupt */
    LPUART_HAL_SetIntMode(base, LPUART_INT_RX_OVERRUN, true);

    /* Enable receive data full interrupt */
    LPUART_HAL_SetIntMode(base, LPUART_INT_RX_DATA_REG_FULL, true);

    return LPUART_STAT_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_DRV_CompleteReceiveData
 * Description   : Finish up a receive by completing the process of receiving data
 * and disabling the interrupt.
 * This is not a public API as it is called from other driver functions.
 *
 *END**************************************************************************/
static void LPUART_DRV_CompleteReceiveData(uint32_t instance)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    lpuart_state_t * lpuartState = (lpuart_state_t *)g_lpuartStatePtr[instance];
    LPUART_Type * base = g_lpuartBase[instance];

    /* disable receive data full and rx overrun interrupt. */
    LPUART_HAL_SetIntMode(base, LPUART_INT_RX_DATA_REG_FULL, false);
    LPUART_HAL_SetIntMode(base, LPUART_INT_RX_OVERRUN, false);

    /* Update the information of the module driver state */
    lpuartState->isRxBusy = false;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
