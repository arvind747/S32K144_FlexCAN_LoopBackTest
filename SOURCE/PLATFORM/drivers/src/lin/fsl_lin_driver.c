/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retValain the above copyright notice, this list
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
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES,
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fsl_lin_driver.h"
#if (LPUART_INSTANCE_COUNT > 0U)
    #include "fsl_lin_lpuart_driver.h"
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/
/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_DRV_Init
 * Description   : This function initializes a LIN Hardware Interface for operation.
 * This function will initialize the run-time state structure to keep track of
 * the on-going transfers, ungate the clock to LIN Hardware Interface, initialize the
 * module to user defined settings and default settings, configure the IRQ state
 * structure and enable the module-level interrupt to the core, and enable the
 * LIN Hardware Interface transmitter and receiver.
 * The following is an example of how to set up the lin_state_t and the
 * lin_user_config_t parameters and how to call the LIN_DRV_Init function
 * by passing in these parameters:
 *    lin_user_config_t linUserConfig
 *    linUserConfig.baudRate = 9600
 *    linUserConfig.nodeFunction = SLAVE
 *    linUserConfig.autobaudEnable = true
 *    linUserConfig.timerStartCallback = (lin_timer_start_t) l_ifc_timerStartCallbackHandler
 *    linUserConfig.timerGetUsCallback = (lin_timer_get_us_t) l_ifc_timerGetUsCallbackHandler
 *    lin_state_t linState
 *    LIN_DRV_Init(instance, (lin_user_config_t *) &linUserConfig, (lin_state_t *) &linState)
 *
 *END**************************************************************************/
lin_status_t LIN_DRV_Init(uint32_t instance,
                          lin_user_config_t * linUserConfig,
                          lin_state_t * linCurrentState)
{
    lin_status_t retVal = LIN_IFC_NOT_SUPPORT;

#if (LPUART_INSTANCE_COUNT > 0U)
    retVal = LIN_LPUART_DRV_Init(instance, linUserConfig, linCurrentState);
#endif

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_DRV_Deinit
 * Description   : This function shuts down the LIN Hardware Interface by disabling interrupts and
 *                 transmitter/receiver.
 *
 *END**************************************************************************/
lin_status_t LIN_DRV_Deinit(uint32_t instance)
{
    lin_status_t retVal = LIN_IFC_NOT_SUPPORT;

#if (LPUART_INSTANCE_COUNT > 0U)
    retVal = LIN_LPUART_DRV_Deinit(instance);
#endif

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_DRV_InstallCallback
 * Description   : This function installs the callback function that is used for LIN_DRV_IRQHandler.
 * Pass in Null pointer as callback will uninstall.
 *
 *END**************************************************************************/
lin_callback_t LIN_DRV_InstallCallback(uint32_t instance,
                                       lin_callback_t function)
{
    lin_callback_t retVal = 0;

#if (LPUART_INSTANCE_COUNT > 0U)
    retVal = LIN_LPUART_DRV_InstallCallback(instance, function);
#endif

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_DRV_SendFrameDataBlocking
 * Description   : This function sends data out through the LIN Hardware Interface using
 * blocking method. This function will calculate the checksum byte and send it
 * with the frame data. The function does not return until the transmission
 * is complete.
 *
 *END**************************************************************************/
lin_status_t LIN_DRV_SendFrameDataBlocking(uint32_t instance,
                                           const uint8_t * txBuff,
                                           uint8_t txSize,
                                           uint32_t timeout)
{
    lin_status_t retVal = LIN_IFC_NOT_SUPPORT;

#if (LPUART_INSTANCE_COUNT > 0U)
    retVal = LIN_LPUART_DRV_SendFrameDataBlocking(instance, txBuff, txSize, timeout);
#endif

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_DRV_SendFrameData
 * Description   : This function sends data out through the LIN Hardware Interface using
 * non-blocking method. This function will calculate the checksum byte and send
 * it with the frame data. The function will return immediately after calling
 * this function. Before using this function, users have to set timeout counter
 * to an appropriate value by using LIN_DRV_SetTimeoutCounter
 *
 *END**************************************************************************/
lin_status_t LIN_DRV_SendFrameData(uint32_t instance,
                                   const uint8_t * txBuff,
                                   uint8_t txSize)
{
    lin_status_t retVal = LIN_IFC_NOT_SUPPORT;

#if (LPUART_INSTANCE_COUNT > 0U)
    retVal = LIN_LPUART_DRV_SendFrameData(instance, txBuff, txSize);
#endif

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_DRV_GetTransmitStatus
 * Description   : This function returns whether the previous transmission has
 * finished. When performing non-blocking transmit, the user can call this
 * function to ascertain the state of the current transmission:
 * in progress (or busy that LIN_TX_BUSY) or timeout (LIN_TIMEOUT) or complete (success that is LIN_SUCCESS).
 * In addition, if the transmission is still in progress, the user can obtain the number
 * of bytes that still needed to transmit.
 *
 *END**************************************************************************/
lin_status_t LIN_DRV_GetTransmitStatus(uint32_t instance,
                                       uint8_t * bytesRemaining)
{
    lin_status_t retVal = LIN_IFC_NOT_SUPPORT;

#if (LPUART_INSTANCE_COUNT > 0U)
    retVal = LIN_LPUART_DRV_GetTransmitStatus(instance, bytesRemaining);
#endif

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_DRV_ReceiveFrameDataBlocking
 * Description   : This function receives data from LIN Hardware Interface module using blocking
 * method, the function does not return until the receive is complete. This
 * function will check the checksum byte. If the checksum is correct, it will
 * receive the frame data
 *
 *END**************************************************************************/

lin_status_t LIN_DRV_ReceiveFrameDataBlocking(uint32_t instance,
                                              uint8_t * rxBuff,
                                              uint8_t rxSize,
                                              uint32_t timeout)
{
    lin_status_t retVal = LIN_IFC_NOT_SUPPORT;

#if (LPUART_INSTANCE_COUNT > 0U)
    retVal = LIN_LPUART_DRV_ReceiveFrameDataBlocking(instance, rxBuff, rxSize, timeout);
#endif

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_DRV_ReceiveFrameData
 * Description   : This function receives data from LIN Hardware Interface using
 * non-blocking method.  This function returns immediately after initiating the
 * receive function. The application has to get the receive status to see when
 * the receive is complete. In other words, after calling non-blocking get
 * function, the application must get the receive status to check if receive
 * is completed or not.  This function will check the checksum byte. If the
 * checksum is correct, it will receive the frame data
 *
 *END**************************************************************************/
lin_status_t LIN_DRV_ReceiveFrameData(uint32_t instance,
                                      uint8_t * rxBuff,
                                      uint8_t rxSize)
{
    lin_status_t retVal = LIN_IFC_NOT_SUPPORT;

#if (LPUART_INSTANCE_COUNT > 0U)
    retVal = LIN_LPUART_DRV_ReceiveFrameData(instance, rxBuff, rxSize);
#endif

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_DRV_AbortTransferData
 * Description   : Aborts an on-going non-blocking transmission/reception.
 * While performing a non-blocking transferring data, users can call this
 * function to terminate immediately the transferring.
 *
 *END**************************************************************************/
lin_status_t LIN_DRV_AbortTransferData(uint32_t instance)
{
    lin_status_t retVal = LIN_IFC_NOT_SUPPORT;

#if (LPUART_INSTANCE_COUNT > 0U)
    retVal = LIN_LPUART_DRV_AbortTransferData(instance);
#endif

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_DRV_GetReceiveStatus
 * Description   : This function returns whether the data reception is
 * complete. When performing non-blocking transmit, the user can call this
 * function to ascertain the state of the current transmission:
 * in progress (or busy that LIN_RX_BUSY) or timeout (LIN_TIMEOUT) or complete (success that is LIN_SUCCESS).
 * In addition, if the transmission is still in progress, the user can obtain the number
 * of bytes that still needed to receive.
 *
 *END**************************************************************************/
lin_status_t LIN_DRV_GetReceiveStatus(uint32_t instance,
                                      uint8_t * bytesRemaining)
{
    lin_status_t retVal = LIN_IFC_NOT_SUPPORT;

#if (LPUART_INSTANCE_COUNT > 0U)
    retVal = LIN_LPUART_DRV_GetReceiveStatus(instance, bytesRemaining);
#endif

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_DRV_GoToSleepMode
 * Description   : This function puts current LIN node to sleep mode.
 * This function changes current node state to LIN_NODE_STATE_SLEEP_MODE.
 *
 *END**************************************************************************/
lin_status_t LIN_DRV_GoToSleepMode(uint32_t instance)
{
    lin_status_t retVal = LIN_IFC_NOT_SUPPORT;

#if (LPUART_INSTANCE_COUNT > 0U)
    retVal = LIN_LPUART_DRV_GoToSleepMode(instance);
#endif

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_DRV_GotoIdleState
 * Description   : This function puts current node to Idle state.
 *
 *END**************************************************************************/
lin_status_t LIN_DRV_GotoIdleState(uint32_t instance)
{
    lin_status_t retVal = LIN_IFC_NOT_SUPPORT;

#if (LPUART_INSTANCE_COUNT > 0U)
    retVal = LIN_LPUART_DRV_GotoIdleState(instance);
#endif

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_DRV_SendWakeupSignal
 * Description   : This function sends a wakeup signal through the LPUART interface.
 *
 *END**************************************************************************/
lin_status_t LIN_DRV_SendWakeupSignal(uint32_t instance)
{
    lin_status_t retVal = LIN_IFC_NOT_SUPPORT;

#if (LPUART_INSTANCE_COUNT > 0U)
    retVal = LIN_LPUART_DRV_SendWakeupSignal(instance);
#endif

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_DRV_GetCurrentNodeState
 * Description   : This function gets the current LIN node state.
 *
 *END**************************************************************************/
lin_node_state_t LIN_DRV_GetCurrentNodeState(uint32_t instance)
{
    lin_node_state_t retVal = LIN_NODE_STATE_UNINIT;

#if (LPUART_INSTANCE_COUNT > 0U)
    retVal = LIN_LPUART_DRV_GetCurrentNodeState(instance);
#endif

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_DRV_TimeoutService
 * Description   : This is callback function for Timer Interrupt Handler.
 * Users shall initialize a timer (for example FTM) in Output compare mode
 * with period of 500 micro seconds. In timer IRQ handler, call this function.
 *
 *END**************************************************************************/
void LIN_DRV_TimeoutService(uint32_t instance)
{

#if (LPUART_INSTANCE_COUNT > 0U)
    LIN_LPUART_DRV_TimeoutService(instance);
#endif

}

/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_DRV_SetTimeoutCounter
 * Description   : This function sets value for timeout counter that is used in
 * LIN_DRV_TimeoutService
 *
 *END**************************************************************************/
void LIN_DRV_SetTimeoutCounter(uint32_t instance,
                               uint32_t timeoutValue)
{

#if (LPUART_INSTANCE_COUNT > 0U)
    LIN_LPUART_DRV_SetTimeoutCounter(instance, timeoutValue);
#endif

}

/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_DRV_MasterSendHeader
 * Description   : This function sends frame header out through the LIN Hardware Interface
 * using a non-blocking method. Non-blocking  means that the function returns
 * immediately. This function sends LIN Break field, sync field then the ID with
 * correct parity.
 *
 *END**************************************************************************/
lin_status_t LIN_DRV_MasterSendHeader(uint32_t instance,
                                      uint8_t id)
{
    lin_status_t retVal = LIN_IFC_NOT_SUPPORT;

#if (LPUART_INSTANCE_COUNT > 0U)
    retVal = LIN_LPUART_DRV_MasterSendHeader(instance, id);
#endif

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_DRV_EnableIRQ
 * Description   : This function enables LIN hardware interrupts.
 *
 *END**************************************************************************/
lin_status_t LIN_DRV_EnableIRQ(uint32_t instance)
{
    lin_status_t retVal = LIN_IFC_NOT_SUPPORT;

#if (LPUART_INSTANCE_COUNT > 0U)
    retVal = LIN_LPUART_DRV_EnableIRQ(instance);
#endif

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_DRV_DisableIRQ
 * Description   : This function disables LIN hardware interrupts.
 *
 *END**************************************************************************/
lin_status_t LIN_DRV_DisableIRQ(uint32_t instance)
{
    lin_status_t retVal = LIN_IFC_NOT_SUPPORT;

#if (LPUART_INSTANCE_COUNT > 0U)
    retVal = LIN_LPUART_DRV_DisableIRQ(instance);
#endif

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_DRV_IRQHandler
 * Description   : Interrupt handler for LIN Hardware Interface.
 * This is not a public API as it is called by IRQ whenever an interrupt
 * occurs.
 *
 *END**************************************************************************/
void LIN_DRV_IRQHandler(uint32_t instance)
{

#if (LPUART_INSTANCE_COUNT > 0U)
    LIN_LPUART_DRV_IRQHandler(instance);
#endif

}
/*******************************************************************************
 * EOF
 ******************************************************************************/
