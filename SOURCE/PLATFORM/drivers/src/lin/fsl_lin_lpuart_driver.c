/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modif ication,
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
 *   software without specif ic prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES,
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fsl_lin_lpuart_driver.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief Table of base addresses for LPUART instances. */
LPUART_Type * const g_linLpuartBase[LPUART_INSTANCE_COUNT] = LPUART_BASE_PTRS;

/*! @brief Table to save LPUART IRQ enumeration numbers defined in the CMSIS header file */
const IRQn_Type g_linLpuartRxTxIrqId[LPUART_INSTANCE_COUNT] = LPUART_RX_TX_IRQS;

/*! @brief Table to save LPUART state structure pointers */
lin_state_t * g_linStatePtr[LPUART_INSTANCE_COUNT] = {0};

/*! @brief Table to save LIN user config structure pointers */
lin_user_config_t * g_linUserconfigPtr[LPUART_INSTANCE_COUNT] = {0};

/*! @brief Table to save ISR pointers for LPUART instances */
extern isr_t g_linLpuartIsrs[LPUART_INSTANCE_COUNT];

/*******************************************************************************
 * Static function prototypes
 ******************************************************************************/

static lin_status_t LIN_LPUART_DRV_WaitComplete(uint32_t instance);

static void LIN_LPUART_DRV_AutobaudTimerValEval(uint32_t instance,
                                                lin_user_config_t * linUserConfig);

static void LIN_LPUART_DRV_PROCESS_DATA(uint32_t instance,
                                        lin_user_config_t * linUserConfig,
                                        lin_state_t * linCurrentState,
                                        uint8_t tmpByte);

/*******************************************************************************
 * Code
 ******************************************************************************/
/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_LPUART_DRV_Init
 * Description   : This function initializes a LPUART instance for operation.
 * This function will initialize the run-time state structure to keep track of
 * the on-going transfers, ungate the clock to the LPUART module, initialize the
 * module to user defined settings and default settings, configure the IRQ state
 * structure and enable the module-level interrupt to the core, and enable the
 * LPUART module transmitter and receiver.
 *
 *END**************************************************************************/
lin_status_t LIN_LPUART_DRV_Init(uint32_t instance,
                                 lin_user_config_t * linUserConfig,
                                 lin_state_t * linCurrentState)
{
    /* Assert parameters. */
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(linUserConfig && linCurrentState);
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    /*! @brief Table to save LPUART clock names as defined in clock manager. */
    clock_names_t g_linLpuartClkNames[LPUART_INSTANCE_COUNT] = {PCC_LPUART0_CLOCK, PCC_LPUART1_CLOCK, PCC_LPUART2_CLOCK, PCC_LPUART3_CLOCK};

    lin_status_t retVal = LIN_SUCCESS;

    /* Get base address of the LPUART instance. */
    LPUART_Type * base = g_linLpuartBase[instance];

    /* Check if the instance is already initialized. */
    if (g_linStatePtr[instance])
    {
        retVal = LIN_INITIALIZED;
    }

    else
    {
         /* Save runtime structure pointer. */
        g_linStatePtr[instance] = linCurrentState;

        /* Save LIN user config structure pointer. */
        g_linUserconfigPtr[instance] = linUserConfig;

        /* Get the LPUART clock as configured in the clock manager */
        (void)CLOCK_SYS_GetFreq(g_linLpuartClkNames[instance], &linCurrentState->linSourceClockFreq);

        /* Exit if current instance is clock gated. */
        if (linCurrentState->linSourceClockFreq == 0U)
        {
            retVal = LIN_LPUART_STAT_CLOCK_GATED_OFF;
        }
        else
        {
            /* Init LPUART */
            LPUART_HAL_Init(base);

            /* if autobaud is enabled */
            if ((linUserConfig->autobaudEnable) && (linUserConfig->nodeFunction == (bool)SLAVE))
            {
                /* Setting Slave's baudrate to 20000 will help Slave node */
                /* always detect LIN Break from Master */
                linUserConfig->baudRate = 20000U;
                linCurrentState->fallingEdgeInterruptCount = 0U;
                linCurrentState->baudrateEvalEnable = true;
            }
            /* Set baudrate to User's value */
            (void)LPUART_HAL_SetBaudRate(base, linCurrentState->linSourceClockFreq, linUserConfig->baudRate);

            /* Set 8 bit counts per char */
            LPUART_HAL_SetBitCountPerChar(base, LPUART_8_BITS_PER_CHAR);

            /* Set no parity mode */
            LPUART_HAL_SetParityMode(base, LPUART_PARITY_DISABLED);

            /* One stop bit */
            LPUART_HAL_SetStopBitCount(base, LPUART_ONE_STOP_BIT);

            /* Set Break char length as 13 bits minimum */
            LPUART_HAL_SetBreakCharTransmitLength(base, LPUART_BREAK_CHAR_13_BIT_MINIMUM);

            /* Enable the LPUART transmitter and receiver */
            LPUART_HAL_SetTransmitterCmd(base, true);
            LPUART_HAL_SetReceiverCmd(base, true);

            /* Set Break char detect length as 13 bits minimum */
            LPUART_HAL_SetBreakCharDetectLength(base, LPUART_BREAK_CHAR_13_BIT_MINIMUM);

            /* Enable RX complete interrupt */
            LPUART_HAL_SetIntMode(base, LPUART_INT_RX_DATA_REG_FULL, true);

            /* Enable frame error interrupt */
            LPUART_HAL_SetIntMode(base, LPUART_INT_FRAME_ERR_FLAG, true);

            /* Enable LIN break detect interrupt */
            LPUART_HAL_SetIntMode(base, LPUART_INT_LIN_BREAK_DETECT, true);

            /* Instal LIN ISR for LPUART instance */
            (void)INT_SYS_InstallHandler(g_linLpuartRxTxIrqId[instance], g_linLpuartIsrs[instance], (isr_t*) 0);

            /* Enable LPUART interrupt. */
            INT_SYS_EnableIRQ(g_linLpuartRxTxIrqId[instance]);

            /* Change node's current state to IDLE */
            linCurrentState->currentNodeState = LIN_NODE_STATE_IDLE;

            /* Clear flags in current LIN state structure */
            linCurrentState->isTxBusy = false;
            linCurrentState->isRxBusy = false;
            linCurrentState->isBusBusy = false;
            linCurrentState->isRxBlocking = false;
            linCurrentState->isTxBlocking = false;
            linCurrentState->timeoutCounterFlag = false;
        } /* End of if (!linCurrentState->linSourceClockFreq) */
    } /* End of if (g_linStatePtr[instance]) */

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_LPUART_DRV_Deinit
 * Description   : This function shuts down the LPUART by disabling interrupts and
 *                 transmitter/receiver.
 *
 *END**************************************************************************/
lin_status_t LIN_LPUART_DRV_Deinit(uint32_t instance)
{
    /* Assert parameters. */
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    lin_status_t retVal = LIN_SUCCESS;

    /* Get base address of the LPUART instance. */
    LPUART_Type * base = g_linLpuartBase[instance];

    /* Get the current LIN state of this LPUART instance. */
    lin_state_t *linCurrentState = g_linStatePtr[instance];

    /* Check if the LIN state pointer is 0. */
    if ((!linCurrentState) || (!linCurrentState->linSourceClockFreq))
    {
        retVal = LIN_ERROR;
    }
    else
    {
        /* Wait until the data is completely shifted out of shift register */
        while (!LPUART_HAL_GetStatusFlag(base, LPUART_TX_COMPLETE))
        {}

        /* Disable the LPUART transmitter and receiver */
        LPUART_HAL_SetTransmitterCmd(base, false);
        LPUART_HAL_SetReceiverCmd(base, false);

        /* Disable LPUART interrupt. */
        INT_SYS_DisableIRQ(g_linLpuartRxTxIrqId[instance]);

        /* Change node's current state to UNINIT */
        linCurrentState->currentNodeState = LIN_NODE_STATE_UNINIT;

        /* Clear our saved pointer to the LIN state structure */
        g_linStatePtr[instance] = 0U;
    }

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_LPUART_DRV_InstallCallback
 * Description   : This function installs the callback function that is used for LIN_LPUART_DRV_IRQHandler.
 * Pass in Null pointer as callback will uninstall.
 *
 *END**************************************************************************/
lin_callback_t LIN_LPUART_DRV_InstallCallback(uint32_t instance,
                                              lin_callback_t function)
{
    /* Assert parameters. */
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    /* Get the current LIN state of this LPUART instance. */
    lin_state_t *linCurrentState = g_linStatePtr[instance];

    /* Get the current callback function. */
    lin_callback_t currentCallback = linCurrentState->Callback;

    /* Install new callback function. */
    linCurrentState->Callback = function;

    return currentCallback;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_LPUART_DRV_SendFrameDataBlocking
 * Description   : This function sends data out through the LPUART module using
 * blocking method. This function will calculate the checksum byte and send it
 * with the frame data.  The function does not return until the transmission
 * is complete.
 *
 *END**************************************************************************/
lin_status_t LIN_LPUART_DRV_SendFrameDataBlocking(uint32_t instance,
                                                  const uint8_t * txBuff,
                                                  uint8_t txSize,
                                                  uint32_t timeout)
{
    /* Assert parameters. */
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(txBuff);
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    /* Get base address of the LPUART instance. */
    LPUART_Type * base = g_linLpuartBase[instance];

    /* Get the current LIN state of this LPUART instance. */
    lin_state_t *linCurrentState = g_linStatePtr[instance];
    lin_status_t retVal = LIN_SUCCESS;

    /* Check if the LIN Bus is busy */
    if (linCurrentState->isBusBusy)
    {
        retVal = LIN_BUS_BUSY;
    }
    else
    {
        /* Make the checksum byte. */
        linCurrentState->checkSum = LIN_DRV_MakeChecksumByte(txBuff, txSize, linCurrentState->currentPid);

        /* Update the LIN state structure. */
        linCurrentState->txBuff = txBuff;
        /* Add a place for checksum byte */
        linCurrentState->txSize = txSize + 1U;
        linCurrentState->cntByte = 0U;
        linCurrentState->currentEventId = LIN_NO_EVENT;
        linCurrentState->isBusBusy = true;
        linCurrentState->isTxBusy = true;
        linCurrentState->isTxBlocking = true;

        /* Set Break char detect length as 10 bits minimum */
        LPUART_HAL_SetBreakCharDetectLength(base, LPUART_BREAK_CHAR_10_BIT_MINIMUM);
        linCurrentState->timeoutCounterFlag = false;
        linCurrentState->timeoutCounter = timeout;

        /* Set node's current state to SEND_DATA */
        linCurrentState->currentNodeState = LIN_NODE_STATE_SEND_DATA;

        /* Start sending data */
        LPUART_HAL_Putchar(base, *linCurrentState->txBuff);

        /* Wait until the transmission is complete. */
        retVal = LIN_LPUART_DRV_WaitComplete(instance);

        /* Update Bus Busy flag. */
        linCurrentState->isBusBusy = false;
        linCurrentState->isTxBusy = false;

        /* Update node's current state to IDLE. */
        linCurrentState->currentNodeState = LIN_NODE_STATE_IDLE;
    }

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_LPUART_DRV_SendFrameData
 * Description   : This function sends data out through the LPUART module using
 * non-blocking method. This function will calculate the checksum byte and send
 * it with the frame data. The function will return immediately after calling
 * this function.
 *
 *END**************************************************************************/
lin_status_t LIN_LPUART_DRV_SendFrameData(uint32_t instance,
                                          const uint8_t * txBuff,
                                          uint8_t txSize)
{
    /* Assert parameters. */
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(txBuff);
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    lin_status_t retVal = LIN_SUCCESS;

    /* Get base address of the LPUART instance. */
    LPUART_Type * base = g_linLpuartBase[instance];

    /* Get the current LIN state of this LPUART instance. */
    lin_state_t *linCurrentState = g_linStatePtr[instance];

    /* Check if the LIN Bus is busy */
    if (linCurrentState->isBusBusy)
    {
        retVal = LIN_BUS_BUSY;
    }
    else
    {
        /* Make the checksum byte. */
        linCurrentState->checkSum = LIN_DRV_MakeChecksumByte(txBuff, txSize, linCurrentState->currentPid);

        /* Update the LIN state structure. */
        linCurrentState->txBuff = txBuff;
        /* Add a place for checksum byte */
        linCurrentState->txSize = txSize + 1U;
        linCurrentState->cntByte = 0U;
        linCurrentState->currentNodeState = LIN_NODE_STATE_SEND_DATA;
        linCurrentState->currentEventId = LIN_NO_EVENT;
        linCurrentState->isBusBusy = true;
        linCurrentState->isTxBusy = true;
        linCurrentState->isTxBlocking = false;

        /* Set Break char detect length as 10 bits minimum */
        LPUART_HAL_SetBreakCharDetectLength(base, LPUART_BREAK_CHAR_10_BIT_MINIMUM);

        /* Start sending data */
        LPUART_HAL_Putchar(base, *linCurrentState->txBuff);
    }

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_LPUART_DRV_GetTransmitStatus
 * Description   : This function returns whether the previous LPUART transmit has
 * finished. When performing non-blocking transmit, the user can call this
 * function to ascertain the state of the current transmission:
 * in progress (or busy that is LIN_TX_BUSY) or timeout (if timeout has occurred that is
 * LIN_TIMEOUT) or complete (success that is LIN_SUCCESS).
 * In addition, if the transmission is still in progress, the user can obtain the number
 * of bytes that still needed to transmit.
 *
 *END**************************************************************************/
lin_status_t LIN_LPUART_DRV_GetTransmitStatus(uint32_t instance,
                                              uint8_t * bytesRemaining)
{
    /* Assert parameters. */
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    lin_status_t retVal = LIN_SUCCESS;

    /* Get the current LIN state of this LPUART instance. */
    lin_state_t *linCurrentState = g_linStatePtr[instance];

    /* Get the number of bytes that is still needed to transmit */
    *bytesRemaining = linCurrentState->txSize - linCurrentState->cntByte;

    /* Return status of the on-going transmission */
    if ((linCurrentState->currentEventId == LIN_NO_EVENT) && (*bytesRemaining != 0U))
    {
        if (linCurrentState->timeoutCounterFlag == false)
        {
            retVal = LIN_TX_BUSY;
        }
        else
        {
            retVal = LIN_TIMEOUT;
        }
    }

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_DRV_ReceiveDataBlocking
 * Description   : This function receives data from LPUART module using blocking
 * method, the function does not return until the receive is complete. This
 * function will check the checksum byte. If the checksum is correct, it will
 * receive the frame data.
 *
 *END**************************************************************************/
lin_status_t LIN_LPUART_DRV_ReceiveFrameDataBlocking(uint32_t instance,
                                                     uint8_t * rxBuff,
                                                     uint8_t rxSize,
                                                     uint32_t timeout)
{
    /* Assert parameters. */
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(rxBuff);
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    /* Get base address of the LPUART instance. */
    LPUART_Type * base = g_linLpuartBase[instance];

    /* Get the current LIN state of this LPUART instance. */
    lin_state_t *linCurrentState = g_linStatePtr[instance];
    lin_status_t retVal = LIN_SUCCESS;

    /* Check if the LIN Bus is busy */
    if (linCurrentState->isBusBusy)
    {
        retVal = LIN_BUS_BUSY;
    }
    else
    {
        /* Update the LIN state structure. */
        linCurrentState->rxBuff = rxBuff;
        /* Add a place for checksum byte */
        linCurrentState->rxSize = rxSize + 1U;
        linCurrentState->cntByte = 0U;
        linCurrentState->timeoutCounterFlag = false;
        linCurrentState->timeoutCounter = timeout;

        /* Start receiving data */
        linCurrentState->currentNodeState = LIN_NODE_STATE_RECV_DATA;
        linCurrentState->currentEventId = LIN_NO_EVENT;
        linCurrentState->isBusBusy = true;
        linCurrentState->isRxBlocking = true;

        /* Set Break char detect length as 10 bits minimum */
        LPUART_HAL_SetBreakCharDetectLength(base, LPUART_BREAK_CHAR_10_BIT_MINIMUM);

        /* Wait until the reception is complete. */
        retVal = LIN_LPUART_DRV_WaitComplete(instance);

        /* Update node's current state to IDLE. */
        linCurrentState->currentNodeState = LIN_NODE_STATE_IDLE;

        /* Update Bus Busy flag. */
        linCurrentState->isBusBusy = false;
    }

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_LPUART_DRV_ReceiveFrameData
 * Description   : This function receives data from LPUART module using
 * non-blocking method.  This function returns immediately after initiating the
 * receive function. The application has to get the receive status to see when
 * the receive is complete. In other words, after calling non-blocking get
 * function, the application must get the receive status to check if receive
 * is completed or not.  This function will check the checksum byte. If the
 * checksum is correct, it will receive the frame data.
 *
 *END**************************************************************************/
lin_status_t LIN_LPUART_DRV_ReceiveFrameData(uint32_t instance,
                                             uint8_t * rxBuff,
                                             uint8_t rxSize)
{
    /* Assert parameters. */
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(rxBuff);
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    lin_status_t retVal = LIN_SUCCESS;

    /* Get base address of the LPUART instance. */
    LPUART_Type * base = g_linLpuartBase[instance];

    /* Get the current LIN state of this LPUART instance. */
    lin_state_t *linCurrentState = g_linStatePtr[instance];

    /* Check if the LIN Bus is busy */
    if (linCurrentState->isBusBusy)
    {
        retVal = LIN_BUS_BUSY;
    }
    else
    {
        /* Update the LIN state structure. */
        linCurrentState->rxBuff = rxBuff;
        /* Add a place for checksum byte */
        linCurrentState->rxSize = rxSize + 1U;
        linCurrentState->cntByte = 0U;

        /* Start receiving data */
        linCurrentState->currentNodeState = LIN_NODE_STATE_RECV_DATA;
        linCurrentState->currentEventId = LIN_NO_EVENT;
        linCurrentState->isBusBusy = true;
        linCurrentState->isRxBlocking = false;

        /* Set Break char detect length as 10 bits minimum */
        LPUART_HAL_SetBreakCharDetectLength(base, LPUART_BREAK_CHAR_10_BIT_MINIMUM);
    }

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_DRV_AbortTransferData
 * Description   : Aborts an on-going non-blocking transmission/reception.
 * While performing a non-blocking transferring data, users can call this
 * function to terminate immediately the transferring.
 *
 *END**************************************************************************/
lin_status_t LIN_LPUART_DRV_AbortTransferData(uint32_t instance)
{
    /* Assert parameters. */
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    lin_status_t retVal = LIN_SUCCESS;

    /* Get the current LIN state of this LPUART instance. */
    lin_state_t *linCurrentState = g_linStatePtr[instance];

    /* Check if no transfer is running. */
    if (linCurrentState->isBusBusy)
    {
        /* Change node's current state to IDLE */
        (void)LIN_LPUART_DRV_GotoIdleState(instance);

        /* Clear LIN Bus Busy flag */
        linCurrentState->isTxBusy = false;
        linCurrentState->isRxBusy = false;
        linCurrentState->isBusBusy = false;
    }
    else
    {
        retVal = LIN_NO_TRANSFER_IN_PROGRESS;
    }

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_LPUART_DRV_GetReceiveStatus
 * Description   : This function returns whether the previous LPUART receive is
 * complete. When performing a non-blocking receive, the user can call this
 * function to ascertain the state of the current receive progress: in progress
 * or complete. In addition, if the receive is still in progress, the user can
 * obtain the number of words that is still needed to receive.
 *
 *END**************************************************************************/
lin_status_t LIN_LPUART_DRV_GetReceiveStatus(uint32_t instance,
                                             uint8_t * bytesRemaining)
{
    /* Assert parameters. */
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    lin_status_t retVal = LIN_SUCCESS;

    /* Get the current LIN state of this LPUART instance. */
    lin_state_t *linCurrentState = g_linStatePtr[instance];

    /* Get the number of bytes that is still needed to receive */
    *bytesRemaining = linCurrentState->rxSize - linCurrentState->cntByte;

    /* Return status of the on-going reception */
    if ((linCurrentState->currentEventId == LIN_NO_EVENT) && (*bytesRemaining != 0U))
    {
        if (linCurrentState->timeoutCounterFlag == false)
        {
            retVal = LIN_RX_BUSY;
        }
        else
        {
            retVal = LIN_TIMEOUT;
        }
    }

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_LPUART_DRV_GoToSleepMode
 * Description   : This function puts current LIN node to sleep mode.
 * This function changes current node state to LIN_NODE_STATE_SLEEP_MODE.
 *
 *END**************************************************************************/
lin_status_t LIN_LPUART_DRV_GoToSleepMode(uint32_t instance)
{
    /* Assert parameters. */
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    /* Get base address of the LPUART instance. */
    LPUART_Type * base = g_linLpuartBase[instance];

    /* Get the current LIN state of this LPUART instance. */
    lin_state_t *linCurrentState = g_linStatePtr[instance];

    /* Update node's current state to SLEEP_MODE. */
    linCurrentState->currentNodeState = LIN_NODE_STATE_SLEEP_MODE;

    /* Set Receive data not inverted */
    LPUART_HAL_SetRxDataPolarity(base, false);

    /* Disable RX complete interrupt */
    LPUART_HAL_SetIntMode(base, LPUART_INT_RX_DATA_REG_FULL, false);

    /* Enable RX Input Active Edge interrupt */
    LPUART_HAL_SetIntMode(base, LPUART_INT_RX_ACTIVE_EDGE, true);

    /* Disable frame error interrupt */
    LPUART_HAL_SetIntMode(base, LPUART_INT_FRAME_ERR_FLAG, false);

    /* Disable LIN break detect interrupt */
    LPUART_HAL_SetIntMode(base, LPUART_INT_LIN_BREAK_DETECT, false);

    return LIN_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_LPUART_DRV_SendWakeupSignal
 * Description   : This function sends a wakeup signal through the LPUART interface.
 *
 *END**************************************************************************/
lin_status_t LIN_LPUART_DRV_SendWakeupSignal(uint32_t instance)
{
    /* DEV_ASSERT parameters. */
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    /* Get base address of the LPUART instance. */
    LPUART_Type * base = g_linLpuartBase[instance];

    /* Send a wakeup signal */
    LPUART_HAL_Putchar(base, 0x80);

    return LIN_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_LPUART_DRV_GetCurrentNodeState
 * Description   : This function gets the current LIN node state.
 *
 *END**************************************************************************/
lin_node_state_t LIN_LPUART_DRV_GetCurrentNodeState(uint32_t instance)
{
    /* Assert parameters. */
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    /* Get the current LIN state of this LPUART instance. */
    lin_state_t *linCurrentState = g_linStatePtr[instance];

    /* Return LIN node's current state */
    return linCurrentState->currentNodeState;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_LPUART_DRV_TimeoutService
 * Description   : This is callback function for Timer Interrupt Handler.
 * Users shall initialize a timer (for example FTM) in Output compare mode
 * with period of about 500 micro seconds. In timer IRQ handler, call this function.
 *
 *END**************************************************************************/
void LIN_LPUART_DRV_TimeoutService(uint32_t instance)
{
    /* Assert parameters. */
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    /* Get the current LIN state of this LPUART instance. */
    lin_state_t *linCurrentState = g_linStatePtr[instance];

    /* Get LIN node's current state */
    lin_node_state_t state = linCurrentState->currentNodeState;

    switch (state)
    {
        /* If the node is SENDING DATA */
        case LIN_NODE_STATE_SEND_DATA:
            /* Check if timeout Counter is 0 */
            if (linCurrentState->timeoutCounter == 0U)
            {
                /* Set timeout Counter flag */
                linCurrentState->timeoutCounterFlag = true;
                /* Check if the transmission is non-blocking */
                if (linCurrentState->isTxBlocking == false)
                {
                    /* Callback to handle timeout Counter flag */
                    linCurrentState->Callback(instance, linCurrentState);

                    /* Clear Bus busy flag */
                    linCurrentState->isBusBusy = false;
                    linCurrentState->isTxBusy = false;

                    /* Change the node's current state to IDLE */
                    (void)LIN_LPUART_DRV_GotoIdleState(instance);
                }
            }
            else /* If timeout Counter is not 0, then decrease timeout Counter by one */
            {
                linCurrentState->timeoutCounter--;
            }
            break;
        /* If the node is RECEIVING DATA */
        case LIN_NODE_STATE_RECV_DATA:
            /* Check if timeout Counter is 0 */
            if (linCurrentState->timeoutCounter == 0U)
            {
                /* Set timeout Counter flag */
                linCurrentState->timeoutCounterFlag = true;
                /* Check if the reception is non-blocking */
                if (linCurrentState->isRxBlocking == false)
                {
                    /* Callback to handle timeout Counter flag */
                    linCurrentState->Callback(instance, linCurrentState);    /* Callback timeout */
                    /* Clear Bus busy flag */
                    linCurrentState->isBusBusy = false;
                    linCurrentState->isRxBusy = false;
                    /* Change the node's current state to IDLE */
                    (void)LIN_LPUART_DRV_GotoIdleState(instance);
                }
            }
            /* If timeout Counter is not 0, then decrease timeout Counter by one */
            else
            {
                linCurrentState->timeoutCounter--;
            }
            break;
        default:
            /* The node state is not SENDING nor RECEIVING data */
            break;
    }

}

/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_LPUART_DRV_SetTimeoutCounter
 * Description   : This function sets value for timeout counter that is used in
 * LIN_LPUART_DRV_TimeoutService
 *
 *END**************************************************************************/
void LIN_LPUART_DRV_SetTimeoutCounter(uint32_t instance,
                                      uint32_t timeoutValue)
{
    /* Assert parameters. */
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    /* Get the current LIN state of this LPUART instance. */
    lin_state_t *linCurrentState = g_linStatePtr[instance];

    /* Clear Timeout Counter Flag */
    linCurrentState->timeoutCounterFlag = false;

    /* Set new value for Timeout Counter */
    linCurrentState->timeoutCounter = timeoutValue;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_LPUART_DRV_MasterSendHeader
 * Description   : This function sends frame header out through the LPUART module
 * using a non-blocking method. Non-blocking  means that the function returns
 * immediately. This function sends LIN Break field, sync field then the ID with
 * correct parity.
 *
 *END**************************************************************************/
lin_status_t LIN_LPUART_DRV_MasterSendHeader(uint32_t instance,
                                             uint8_t id)
{
    /* Assert parameters. */
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    lin_status_t retVal = LIN_SUCCESS;

    /* Get the current LIN user config structure of this LPUART instance. */
    lin_user_config_t *linUserConfig = g_linUserconfigPtr[instance];

    /* Get base address of the LPUART instance. */
    LPUART_Type * base = g_linLpuartBase[instance];

    /* Get the current LIN state of this LPUART instance. */
    lin_state_t *linCurrentState = g_linStatePtr[instance];

    /* Check if the current node is slave */
    if (linUserConfig->nodeFunction == (bool)SLAVE)
    {
        retVal = LIN_ERROR;
    }
    else
    {
        /* Check if the LIN bus is busy */
        if (linCurrentState->isBusBusy)
        {
            retVal = LIN_BUS_BUSY;
        }
        else
        {
            linCurrentState->currentId = id;

            /* Make parity for the current ID */
            linCurrentState->currentPid = LIN_DRV_ProcessParity(id, MAKE_PARITY);

            /* Send Break field */
            linCurrentState->currentNodeState = LIN_NODE_STATE_SEND_BREAK_FIELD;
            linCurrentState->currentEventId = LIN_NO_EVENT;
            linCurrentState->isBusBusy = true;

            /* Set Break char detect length as 10 bits minimum */
            LPUART_HAL_SetBreakCharDetectLength(base, LPUART_BREAK_CHAR_13_BIT_MINIMUM);
            LPUART_HAL_SetIntMode(base, LPUART_INT_LIN_BREAK_DETECT, true);

            /* Write 1 and then 0 to SBK queues a break character in the transmit data stream */
            LPUART_HAL_QueueBreakCharToSend(base, true);
            LPUART_HAL_QueueBreakCharToSend(base, false);
        }
    }

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_LPUART_DRV_EnableIRQ
 * Description   : This function enables LPUART hardware interrupts.
 *
 *END**************************************************************************/
lin_status_t LIN_LPUART_DRV_EnableIRQ(uint32_t instance)
{
    /* Assert parameters. */
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif
    lin_status_t retVal = LIN_SUCCESS;

    /* Get base address of the LPUART instance. */
    LPUART_Type * base = g_linLpuartBase[instance];

    /* Get the current LIN state of this LPUART instance. */
    lin_state_t *linCurrentState = g_linStatePtr[instance];

    if (linCurrentState->currentNodeState == LIN_NODE_STATE_SLEEP_MODE)
    {
        /* Enable RX Input Active Edge interrupt */
        LPUART_HAL_SetIntMode(base, LPUART_INT_RX_ACTIVE_EDGE, true);
    }
    else
    {
        /* Enable RX complete interrupt */
        LPUART_HAL_SetIntMode(base, LPUART_INT_RX_DATA_REG_FULL, true);

        /* Enable frame error interrupt */
        LPUART_HAL_SetIntMode(base, LPUART_INT_FRAME_ERR_FLAG, true);

        /* Enable LIN break detect interrupt */
        LPUART_HAL_SetIntMode(base, LPUART_INT_LIN_BREAK_DETECT, true);

    }
    /* Enable LPUART interrupt. */
    INT_SYS_EnableIRQ(g_linLpuartRxTxIrqId[instance]);
    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_LPUART_DRV_DisableIRQ
 * Description   : This function disables LPUART hardware interrupts.
 *
 *END**************************************************************************/
lin_status_t LIN_LPUART_DRV_DisableIRQ(uint32_t instance)
{
    /* Assert parameters. */
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif
    lin_status_t retVal = LIN_SUCCESS;

    /* Get base address of the LPUART instance. */
    LPUART_Type * base = g_linLpuartBase[instance];

    /* Get the current LIN state of this LPUART instance. */
    lin_state_t *linCurrentState = g_linStatePtr[instance];

    if (linCurrentState->currentNodeState == LIN_NODE_STATE_SLEEP_MODE)
    {
        /* Disable RX Input Active Edge interrupt */
        LPUART_HAL_SetIntMode(base, LPUART_INT_RX_ACTIVE_EDGE, false);
    }
    else
    {
        /* Disable RX complete interrupt */
        LPUART_HAL_SetIntMode(base, LPUART_INT_RX_DATA_REG_FULL, false);

        /* Disable frame error interrupt */
        LPUART_HAL_SetIntMode(base, LPUART_INT_FRAME_ERR_FLAG, false);

        /* Disable frame error interrupt */
        LPUART_HAL_SetIntMode(base, LPUART_INT_LIN_BREAK_DETECT, false);
    }

    /* Disable LPUART interrupt. */
    INT_SYS_DisableIRQ(g_linLpuartRxTxIrqId[instance]);
    return retVal;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_LPUART_DRV_IRQHandler
 * Description   : Interrupt handler for LPUART.
 * This handler uses the buffers stored in the lin_state_t structs to transfer
 * data. This is not a public API as it is called by IRQ whenever an interrupt
 * occurs.
 *
 *END**************************************************************************/
void LIN_LPUART_DRV_IRQHandler(uint32_t instance)
{
    /* Assert parameters. */
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);
#endif

    uint8_t tmpByte;
    uint32_t wakeupSignalLength = 0;

    /* Get the current LIN user config structure of this LPUART instance. */
    lin_user_config_t *linUserConfig = g_linUserconfigPtr[instance];

    /* Get base address of the LPUART instance. */
    LPUART_Type * base = g_linLpuartBase[instance];

    /* Get the current LIN state of this LPUART instance. */
    lin_state_t *linCurrentState = g_linStatePtr[instance];

    /* If LIN break character has been detected. */
    if (LPUART_HAL_GetStatusFlag(base, LPUART_LIN_BREAK_DETECT))
    {
        /* Clear LIN Break Detect Interrupt Flag */
        (void)LPUART_HAL_ClearStatusFlag(base, LPUART_LIN_BREAK_DETECT);

        /* Check if the current node is in SLEEP MODE */
        if (linCurrentState->currentNodeState == LIN_NODE_STATE_SLEEP_MODE)
        {
            /* Change the node's current state to IDLE */
            (void)LIN_LPUART_DRV_GotoIdleState(instance);
        }
        else
        {
            /* Set Break char detect length as 10 bits minimum */
            LPUART_HAL_SetBreakCharDetectLength(base, LPUART_BREAK_CHAR_10_BIT_MINIMUM);
            /* Disable LIN Break Detect Interrupt */
            LPUART_HAL_SetIntMode(base, LPUART_INT_LIN_BREAK_DETECT, false);

            /* Set flag LIN bus busy */
            linCurrentState->isBusBusy = true;

            /* Check if the current node is MASTER */
            if (linUserConfig->nodeFunction == (bool)MASTER)  /* Master */
            {
                /* Change the node's current state to SENDING PID */
                linCurrentState->currentNodeState = LIN_NODE_STATE_SEND_PID;
                /* Send Sync Field 0x55 */
                LPUART_HAL_Putchar(base, 0x55);    /* Send SYNC field */
            }
            /* If the current node is SLAVE */
            else
            {
                /* If the Autobaud feature is enabled */
                if (linUserConfig->autobaudEnable && linCurrentState->baudrateEvalEnable)
                {
                    /* Set Receive Data Not Inverted */
                    LPUART_HAL_SetRxDataPolarity(base, false);

                    /* Enable RXEDG interrupt */
                    LPUART_HAL_SetIntMode(base, LPUART_INT_RX_ACTIVE_EDGE, true);

                    /* Disable Receiver Interrupt */
                    LPUART_HAL_SetIntMode(base, LPUART_INT_RX_DATA_REG_FULL, false);

                    /* Disable frame error interrupt */
                    LPUART_HAL_SetIntMode(base, LPUART_INT_FRAME_ERR_FLAG, false);

                    /* Start Autobaud Count */
                    linCurrentState->fallingEdgeInterruptCount = 0U;
                }

                /* Change the node's current state to RECEIVED BREAK FIELD */
                linCurrentState->currentEventId = LIN_RECV_BREAK_FIELD_OK;

                /* Callback function */
                linCurrentState->Callback(instance, linCurrentState);

                /* Change the node's current state to RECEIVING SYNC FIELD */
                linCurrentState->currentNodeState = LIN_NODE_STATE_RECV_SYNC;
            }
        }
    }
    else
    {
        /* If LPUART_RX Pin Active Edge has been detected. */
        if (LPUART_HAL_GetStatusFlag(base, LPUART_RX_ACTIVE_EDGE_DETECT))
        {
            /* Clear LPUART_RX Pin Active Edge Interrupt Flag. */
            (void)LPUART_HAL_ClearStatusFlag(base, LPUART_RX_ACTIVE_EDGE_DETECT);

            /* Check if the node is in SLEEP MODE */
            /* If yes, then check if a wakeup signal has been received */
            if (linCurrentState->currentNodeState == LIN_NODE_STATE_SLEEP_MODE)
            {
                /* if LPUART_HAL_GetRxDataPolarity is 0: Receive Data is not inverted */
                if (LPUART_HAL_GetRxDataPolarity(base) == 0U)
                {
                    /* Start measure time */
                    linUserConfig->timerStartCallback();
                    /* Set Receive Data Inverted */
                    LPUART_HAL_SetRxDataPolarity(base, true);
                }
                else
                {
                    /* Set Receive Data is Not Inverted */
                    LPUART_HAL_SetRxDataPolarity(base, false);

                   /* Calculate time interval between the falling and rising edge */
                    linUserConfig->timerGetUsCallback(&wakeupSignalLength);

                    /* If length of the dominant signal is from 150us to 5ms, it is a wakeup signal */
                    if ((wakeupSignalLength >= 150U) && (wakeupSignalLength <= 5000U))
                    {
                        linCurrentState->currentEventId = LIN_WAKEUP_SIGNAL;

                        /* Callback to handle event: Received a wakeup signal */
                        linCurrentState->Callback(instance, linCurrentState);

                        /* Change node's state to IDLE */
                        (void)LIN_LPUART_DRV_GotoIdleState(instance);
                    }
                }
            }
            /* If the node is not in SLEEP MODE */
            /* Then check sync byte if Resynchronization is enabled */
            else
            {
                /* if Autobaud feature is enabled */
                if (linUserConfig->autobaudEnable && linCurrentState->baudrateEvalEnable)
                {
                    if (linCurrentState->fallingEdgeInterruptCount == 0U)
                    {
                      /* Start time measurement */
                      linUserConfig->timerStartCallback();
                    }

                    linCurrentState->fallingEdgeInterruptCount += 1U;
                    /* Process evaluation */
                    if (linCurrentState->fallingEdgeInterruptCount == 0x05U)
                    {
                        /* Clear autobaud flag */
                        linCurrentState->fallingEdgeInterruptCount = 0U;

                        /* Evaluate Timer Value to set desired baudrate */
                        LIN_LPUART_DRV_AutobaudTimerValEval(instance, linUserConfig);

                        /* Enable Receiver Interrupt */
                        LPUART_HAL_SetIntMode(base, LPUART_INT_RX_DATA_REG_FULL, true);
                        /* Disable RXEDG interrupt */
                        LPUART_HAL_SetIntMode(base, LPUART_INT_RX_ACTIVE_EDGE, false);
                        /* Enable frame error interrupt */
                        LPUART_HAL_SetIntMode(base, LPUART_INT_FRAME_ERR_FLAG, true);
                    }
                }
            }
        }
        else
        {
            /* If Framing Error has been detected */
            if (LPUART_HAL_GetStatusFlag(base, LPUART_FRAME_ERR))
            {
                /* Clear Framing Error Interrupt Flag */
                (void)LPUART_HAL_ClearStatusFlag(base, LPUART_FRAME_ERR);

                /* Set current event id to LIN_FRAME_ERROR */
                linCurrentState->currentEventId = LIN_FRAME_ERROR;
                if ((linCurrentState->isTxBlocking == false) && (linCurrentState->currentNodeState == LIN_NODE_STATE_SEND_DATA))
                {
                    /* Callback function to handle Framing Error Event */
                    linCurrentState->Callback(instance, linCurrentState);
                }
                else
                {
                    if ((linCurrentState->isRxBlocking == false) && (linCurrentState->currentNodeState == LIN_NODE_STATE_RECV_DATA))
                    {
                        /* Callback function to handle Framing Error Event */
                        linCurrentState->Callback(instance, linCurrentState);
                    }
                }
                /* Clear Bus busy Flag */
                linCurrentState->isBusBusy = false;
                /* Change node's state to IDLE */
                (void)LIN_LPUART_DRV_GotoIdleState(instance);
            }
            else
            {

                if (LPUART_HAL_GetStatusFlag(base, LPUART_RX_DATA_REG_FULL))
                {
                    /* Get data from Data Register & Clear LPUART_RX_DATA_REG_FULL flag */
                    LPUART_HAL_Getchar(base, &tmpByte);

                    /* Process data in Data Register while receive, send data */
                    LIN_LPUART_DRV_PROCESS_DATA(instance, linUserConfig, linCurrentState, tmpByte);
                }
            } /* End else: if (LPUART_HAL_GetStatusFlag(base, LPUART_FRAME_ERR) == 0) */
        } /* End else: if (LPUART_HAL_GetStatusFlag(base, LPUART_RX_ACTIVE_EDGE_DETECT) == 0) */
    } /* End else: if (LPUART_HAL_GetStatusFlag(base, LPUART_LIN_BREAK_DETECT) == 0) */

    return;
} /* End void LIN_LPUART_DRV_IRQHandler(uint32_t instance) */

/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_LPUART_DRV_PROCESS_DATA
 * Description   : Part of Interrupt handler for receiving and sending data.
 * Receive Sync byte, PID, Frame data and Send PID, data.
 *END**************************************************************************/
static void LIN_LPUART_DRV_PROCESS_DATA(uint32_t instance,
                                        lin_user_config_t * linUserConfig,
                                        lin_state_t * linCurrentState,
                                        uint8_t tmpByte)
{
    /* Get base address of the LPUART instance. */
    LPUART_Type * base = g_linLpuartBase[instance];

    /* Check node's current state */
    switch (linCurrentState->currentNodeState)
    {
        /* if current state is RECEIVE SYNC FIELD */
        case LIN_NODE_STATE_RECV_SYNC:
            if (tmpByte == 0x55U)
            {
                linCurrentState->currentEventId = LIN_RECV_SYNC_OK;

                /* Change node's current state to RECEIVE PID */
                linCurrentState->currentNodeState = LIN_NODE_STATE_RECV_PID;
            }
            else
            {
                linCurrentState->currentEventId = LIN_RECV_SYNC_ERROR;

                /* Callback function to handle event RECEIVED SYNC FIELD ERROR */
                linCurrentState->Callback(instance, linCurrentState);

                /* Clear Bus busy flag */
                linCurrentState->isBusBusy = false;

                /* Change node's current state to IDLE */
                (void)LIN_LPUART_DRV_GotoIdleState(instance);
            }
            break;
        /* if current state is MASTER SENDING PID */
        case LIN_NODE_STATE_SEND_PID:
            /* Change node's current state to RECEIVING PID */
            linCurrentState->currentNodeState = LIN_NODE_STATE_RECV_PID;
            /* Send the current PID byte */
            LPUART_HAL_Putchar(base, linCurrentState->currentPid);
            break;
        /* if current state is RECEIVE PID */
        case LIN_NODE_STATE_RECV_PID:
            /* if the node is MASTER */
            if (linUserConfig->nodeFunction == (bool)MASTER)
            {
                /* Set current event ID to PID correct */
                linCurrentState->currentEventId = LIN_PID_OK;

                /* Clear Bus bus flag */
                linCurrentState->isBusBusy = false;

                /* Callback function to handle correct PID */
                linCurrentState->Callback(instance, linCurrentState);
            }
            /* If the node is SLAVE */
            else
            {
                /* Check the received PID */
                linCurrentState->currentId = LIN_DRV_ProcessParity(tmpByte, CHECK_PARITY);
                linCurrentState->currentPid = tmpByte;
                if (linCurrentState->currentId != 0xFFU)
                {
                    /* Set current event ID to PID correct */
                    linCurrentState->currentEventId = LIN_PID_OK;

                   /* Clear Bus bus flag */
                    linCurrentState->isBusBusy = false;

                    /* Callback function to handle correct PID */
                    linCurrentState->Callback(instance, linCurrentState);

                }
                else
                {
                    /* Set current event ID to PID ERROR */
                    linCurrentState->currentEventId = LIN_PID_ERROR;

                    /* Callback function to handle correct PID */
                    linCurrentState->Callback(instance, linCurrentState);

                    /* Clear Bus bus flag */
                    linCurrentState->isBusBusy = false;

                    /* Change node's current state to IDLE */
                    (void)LIN_LPUART_DRV_GotoIdleState(instance);
                }
            }
            break;
        /* if current state is RECEIVE DATA */
        case LIN_NODE_STATE_RECV_DATA:
            if ((linCurrentState->rxSize - linCurrentState->cntByte) > 1U)
            {
                *(linCurrentState->rxBuff) = tmpByte;
                linCurrentState->rxBuff++;
            }
            else
            {
                if ((linCurrentState->rxSize - linCurrentState->cntByte) == 1U)
                {
                    linCurrentState->checkSum = tmpByte;
                }
            }

            linCurrentState->cntByte++;
            if (linCurrentState->cntByte == linCurrentState->rxSize)
            {
                /* Restore rxBuffer pointer */
                linCurrentState->rxBuff -= linCurrentState->rxSize - 1U;
                if (LIN_DRV_MakeChecksumByte(linCurrentState->rxBuff, linCurrentState->rxSize - 1U, linCurrentState->currentPid) == linCurrentState->checkSum)
                {
                    linCurrentState->currentEventId = LIN_RX_COMPLETED;
                    if (linCurrentState->isRxBlocking == false)
                    {
                        linCurrentState->currentNodeState = LIN_NODE_STATE_CALLBACK;
                        /* callback function to handle RX COMPLETED */
                        linCurrentState->Callback(instance, linCurrentState);
                        /* Clear Bus bus flag */
                        linCurrentState->isBusBusy = false;
                        linCurrentState->isRxBusy = false;

                        /* In case of receiving a go to sleep request, after callback, node is in SLEEP MODE */
                        /* In this case, node is in SLEEP MODE state */
                        if (linCurrentState->currentNodeState != LIN_NODE_STATE_SLEEP_MODE)
                        {
                            (void)LIN_LPUART_DRV_GotoIdleState(instance);
                        }
                    }
                }
                else
                {
                    linCurrentState->currentEventId = LIN_CHECKSUM_ERROR;
                    if (linCurrentState->isRxBlocking == false)
                    {
                        /* callback function to handle checksum error */
                        linCurrentState->Callback(instance, linCurrentState);
                        /* Clear Bus bus flag */
                        linCurrentState->isBusBusy = false;
                        linCurrentState->isRxBusy = false;
                        /* Change node's current state to IDLE */
                        (void)LIN_LPUART_DRV_GotoIdleState(instance);
                    }
                }
            }
            break;
        /* if current state is SENDING DATA */
        case LIN_NODE_STATE_SEND_DATA:
            if (LPUART_HAL_GetStatusFlag(base, LPUART_TX_DATA_REG_EMPTY) == false)
            {
                linCurrentState->currentEventId = LIN_READBACK_ERROR;
                if (linCurrentState->isTxBlocking == false)
                {
                    /* callback function to handle Readback error */
                    linCurrentState->Callback(instance, linCurrentState);
                    /* Clear Bus bus flag */
                    linCurrentState->isBusBusy = false;
                    linCurrentState->isTxBusy = false;
                    /* Change node's current state to IDLE */
                    (void)LIN_LPUART_DRV_GotoIdleState(instance);
                }
                break;
            }
            else
            {
                if (((*linCurrentState->txBuff != tmpByte) && ((linCurrentState->txSize - linCurrentState->cntByte) != 1U)) || \
                    (((linCurrentState->txSize - linCurrentState->cntByte) == 1U) && (linCurrentState->checkSum != tmpByte)))
                {
                    linCurrentState->currentEventId = LIN_READBACK_ERROR;
                    if (linCurrentState->isTxBlocking == false)
                    {
                        /* callback function to handle Readback error */
                        linCurrentState->Callback(instance, linCurrentState);
                        /* Clear Bus bus flag */
                        linCurrentState->isBusBusy = false;
                        linCurrentState->isTxBusy = false;
                        /* Change node's current state to IDLE */
                        (void)LIN_LPUART_DRV_GotoIdleState(instance);
                    }
                    break;
                }
                else
                {
                    linCurrentState->txBuff++;
                    linCurrentState->cntByte++;
                }
            }

            if (linCurrentState->cntByte < linCurrentState->txSize)
            {
                /* Send checksum byte */
                if ((linCurrentState->txSize - linCurrentState->cntByte) == 1U)
                {
                    LPUART_HAL_Putchar(base, linCurrentState->checkSum);
                }
                /* Send data bytes */
                else
                {
                    LPUART_HAL_Putchar(base, *linCurrentState->txBuff);
                }
            }
            else
            {
                linCurrentState->currentEventId = LIN_TX_COMPLETED;
                if (linCurrentState->isTxBlocking == false)
                {
                    LPUART_HAL_SetIntMode(base, LPUART_INT_RX_DATA_REG_FULL, false);
                    linCurrentState->currentNodeState = LIN_NODE_STATE_CALLBACK;
                    /* callback function to handle event TX COMPLETED */
                    linCurrentState->Callback(instance, linCurrentState);
                    /* Clear Bus bus flag */
                    linCurrentState->isBusBusy = false;
                    linCurrentState->isTxBusy = false;
                    /* Change node's current state to IDLE */
                    (void)LIN_LPUART_DRV_GotoIdleState(instance);
                    LPUART_HAL_SetIntMode(base, LPUART_INT_RX_DATA_REG_FULL, true);
                }
            }
            break;
        default:
            /* Other node state */
            break;
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_LPUART_DRV_WaitComplete
 * Description   : This function waits until transmission/reception is complete and
 * returns status of the transaction.
 *
 *END**************************************************************************/
static lin_status_t LIN_LPUART_DRV_WaitComplete(uint32_t instance)
{
    /* Get the current LIN state of this LPUART instance. */
    lin_state_t *linCurrentState = g_linStatePtr[instance];
    lin_event_id_t eventId;
    lin_status_t retVal = LIN_ERROR;
    while(true)
    {
        /* Get the current event ID */
        eventId = linCurrentState->currentEventId;
        /* If the current event ID is dif ferent than LIN_NO_EVENT */
        if (eventId != LIN_NO_EVENT)
        {
            /* Check if the current event ID is LIN_TX_COMPLETED or LIN_RX_COMPLETED */
            if ((eventId == LIN_TX_COMPLETED) || (eventId == LIN_RX_COMPLETED))
            {
                retVal = LIN_SUCCESS;
                break;
            }
            /* Check if the TimeoutCounterFlag is false */
            else
            {
                if (linCurrentState->timeoutCounterFlag == false)
                {
                    retVal = LIN_ERROR;
                    break;
                }
            }
        }
        /* Check if the TimeoutCounterFlag has been set */
        if (linCurrentState->timeoutCounterFlag == true)
        {
            retVal = LIN_TIMEOUT;
            break;
        }
    }

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_LPUART_DRV_GotoIdleState
 * Description   : This function puts current node to Idle state.
 *
 *END**************************************************************************/
lin_status_t LIN_LPUART_DRV_GotoIdleState(uint32_t instance)
{
    /* Get base address of the LPUART instance. */
    LPUART_Type * base = g_linLpuartBase[instance];

    /* Get the current LIN state of this LPUART instance. */
    lin_state_t *linCurrentState = g_linStatePtr[instance];

    linCurrentState->currentEventId = LIN_NO_EVENT;
    linCurrentState->cntByte = 0U;

    /* Set Break char detect length as 13 bits minimum */
    LPUART_HAL_SetBreakCharDetectLength(base, LPUART_BREAK_CHAR_13_BIT_MINIMUM);

    /* Set Receive data not inverted */
    LPUART_HAL_SetRxDataPolarity(base, false);

    /* Enable RX complete interrupt */
    LPUART_HAL_SetIntMode(base, LPUART_INT_RX_DATA_REG_FULL, true);

    /* Disable RXEDG interrupt */
    LPUART_HAL_SetIntMode(base, LPUART_INT_RX_ACTIVE_EDGE, false);

    /* Enable frame error interrupt */
    LPUART_HAL_SetIntMode(base, LPUART_INT_FRAME_ERR_FLAG, true);

    /* Enable lin break detect interrupt */
    LPUART_HAL_SetIntMode(base, LPUART_INT_LIN_BREAK_DETECT, true);

    /* Change node's current state to IDLE */
    linCurrentState->currentNodeState = LIN_NODE_STATE_IDLE;

    return LIN_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_LPUART_DRV_AutobaudTimerValEval
 * Description   : This function calculate LIN bus baudrate and set slave's baudrate accordingly.
 * Autobaud process runs only once after reset. After setting slave's baudrate to LIN bus baudrate,
 * slave does not evaluate LIN bus baudrate anymore.
 * This is not a public API as it is called from other driver functions.
 *
 *END**************************************************************************/
static void LIN_LPUART_DRV_AutobaudTimerValEval(uint32_t instance,
                                                lin_user_config_t * linUserConfig)
{
    uint32_t bitTimeLength = 0U;
    uint32_t MasterBaudrate = 0U;

    /* Get base address of the LPUART instance. */
    LPUART_Type * base = g_linLpuartBase[instance];

     /* Get the current LIN state of this LPUART instance. */
    lin_state_t *linCurrentState = g_linStatePtr[instance];


    (void)linUserConfig->timerGetUsCallback(&bitTimeLength);

    /* Calculate the average bit time value */
    bitTimeLength = bitTimeLength >> 3U;

    /* Evaluate average value against baudrate */
    if ((bitTimeLength <= BIT_TIME_MAX_19200) && (bitTimeLength > BIT_TIME_MIN_19200))
    {
        MasterBaudrate = 19200U;
    }
    else
    {
        if ((bitTimeLength <= BIT_TIME_MAX_14400) && (bitTimeLength > BIT_TIME_MIN_14400))
        {
            MasterBaudrate = 14400U;
        }
        else
        {
            if ((bitTimeLength <= BIT_TIME_MAX_9600) && (bitTimeLength > BIT_TIME_MIN_9600))
            {
                MasterBaudrate = 9600U;
            }
            else
            {
                if ((bitTimeLength <= BIT_TIME_MAX_4800) && (bitTimeLength > BIT_TIME_MIN_4800))
                {
                    MasterBaudrate = 4800U;
                }
                else
                {
                    if ((bitTimeLength <= BIT_TIME_MAX_2400) && (bitTimeLength > BIT_TIME_MIN_2400))
                    {
                        MasterBaudrate = 2400U;
                    }
                }
            }
        }
    }

    /* Check Master Baudrate against node's current baudrate */
    if ((MasterBaudrate != 0U) && (linUserConfig->baudRate != MasterBaudrate))
    {
        linUserConfig->baudRate = MasterBaudrate;
        /* Set new baudrate */
        (void)LPUART_HAL_SetBaudRate(base, linCurrentState->linSourceClockFreq, linUserConfig->baudRate);
        linCurrentState->currentEventId = LIN_BAUDRATE_ADJUSTED;
        /* Disable baudrate evaluation process */
        linCurrentState->baudrateEvalEnable = false;
        /* Callback function to handle this event */
        linCurrentState->Callback(instance, linCurrentState);
    }
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
