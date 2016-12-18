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
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES,
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef FSL_LIN_LPUART_DRIVER_H_
#define FSL_LIN_LPUART_DRIVER_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fsl_lpuart_hal.h"
#include "fsl_clock_manager.h"
#include "fsl_lin_driver.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* LIN Bus baudrate calculation tolerance 5% */
#define AUTOBAUD_BAUDRATE_TOLERANCE 0.05

#define ONE_MILLION 1000000.0

#define BIT_TIME_MAX_19200  (uint32_t)(ONE_MILLION*(1.0+ AUTOBAUD_BAUDRATE_TOLERANCE)/19200.0)
#define BIT_TIME_MIN_19200  (uint32_t)(ONE_MILLION*(1.0- AUTOBAUD_BAUDRATE_TOLERANCE)/19200.0)
#define BIT_TIME_MAX_14400  (uint32_t)(ONE_MILLION*(1.0+ AUTOBAUD_BAUDRATE_TOLERANCE)/14400.0)
#define BIT_TIME_MIN_14400  (uint32_t)(ONE_MILLION*(1.0- AUTOBAUD_BAUDRATE_TOLERANCE)/14400.0)
#define BIT_TIME_MAX_9600   (uint32_t)(ONE_MILLION*(1.0+ AUTOBAUD_BAUDRATE_TOLERANCE)/9600.0)
#define BIT_TIME_MIN_9600   (uint32_t)(ONE_MILLION*(1.0- AUTOBAUD_BAUDRATE_TOLERANCE)/9600.0)
#define BIT_TIME_MAX_4800   (uint32_t)(ONE_MILLION*(1.0+ AUTOBAUD_BAUDRATE_TOLERANCE)/4800.0)
#define BIT_TIME_MIN_4800   (uint32_t)(ONE_MILLION*(1.0- AUTOBAUD_BAUDRATE_TOLERANCE)/4800.0)
#define BIT_TIME_MAX_2400   (uint32_t)(ONE_MILLION*(1.0+ AUTOBAUD_BAUDRATE_TOLERANCE)/2400.0)
#define BIT_TIME_MIN_2400   (uint32_t)(ONE_MILLION*(1.0- AUTOBAUD_BAUDRATE_TOLERANCE)/2400.0)

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initializes an LIN_LPUART instance for LIN Network.
 *
 * The caller provides memory for the driver state structures during initialization.
 * The user must select the LIN_LPUART clock source in the application to initialize the LIN_LPUART.
 *
 * @param instance LIN_LPUART instance number
 * @param linUserConfig user configuration structure of type #lin_user_config_t
 * @param linCurrentState pointer to the LIN_LPUART driver state structure
 * @return An error code or lin_status_t
 */
lin_status_t LIN_LPUART_DRV_Init(uint32_t instance,
                                 lin_user_config_t * linUserConfig,
                                 lin_state_t * linCurrentState);

/*!
 * @brief Shuts down the LIN_LPUART by disabling interrupts and transmitter/receiver.
 *
 * @param instance LIN_LPUART instance number
 * @return An error code or lin_status_t
 */
lin_status_t LIN_LPUART_DRV_Deinit(uint32_t instance);

/*!
 * @brief Installs callback function that is used for LIN_LPUART_DRV_IRQHandler.
 *
 * @note After a callback is installed, it bypasses part of the LIN_LPUART IRQHandler logic.
 * Therefore, the callback needs to handle the indexes of txBuff and txSize.
 *
 * @param instance The LIN_LPUART instance number.
 * @param function The LIN_LPUART receive callback function.
 * @return Former LIN callback function pointer.
 */
lin_callback_t LIN_LPUART_DRV_InstallCallback(uint32_t instance,
                                              lin_callback_t function);

/*!
 * @brief Sends Frame data out through the LIN_LPUART module using blocking method.
 *  This function will calculate the checksum byte and send it with the frame data.
 *  Blocking means that the function does not return until the transmission is complete.
 *
 * @param instance LIN_LPUART instance number
 * @param txBuff  source buffer containing 8-bit data chars to send
 * @param txSize the number of bytes to send
 * @param timeout timeout value for timer sync control
 * @return An error code or lin_status_t
 */
lin_status_t LIN_LPUART_DRV_SendFrameDataBlocking(uint32_t instance,
                                                  const uint8_t * txBuff,
                                                  uint8_t txSize,
                                                  uint32_t timeout);

/*!
 * @brief Sends frame data out through the LIN_LPUART module using non-blocking method.
 *  This enables an a-sync method for transmitting data.
 *  Non-blocking  means that the function returns immediately.
 *  The application has to get the transmit status to know when the transmit is complete.
 *  This function will calculate the checksum byte and send it with the frame data.
 *
 * @param instance LIN_LPUART instance number
 * @param txBuff  source buffer containing 8-bit data chars to send
 * @param txSize  the number of bytes to send
 * @return An error code or lin_status_t
 */
lin_status_t LIN_LPUART_DRV_SendFrameData(uint32_t instance,
                                          const uint8_t * txBuff,
                                          uint8_t txSize);

/*!
 * @brief Get status of an on-going non-blocking transmission
 *  While sending frame data using non-blocking method, users can
 *  use this function to get status of that transmission.
 *  This function return LIN_TX_BUSY while sending, or LIN_TIMEOUT
 *  if timeout has occurred, or return LIN_SUCCESS when the transmission is complete.
 *  The bytesRemaining shows number of bytes that still needed to transmit.
 *
 * @param instance LIN_LPUART instance number
 * @param bytesRemaining  Number of bytes still needed to transmit
 * @return lin_status_t LIN_TX_BUSY, LIN_SUCCESS or LIN_TIMEOUT
 */
lin_status_t LIN_LPUART_DRV_GetTransmitStatus(uint32_t instance,
                                              uint8_t * bytesRemaining);

/*!
 * @brief Receives frame data through the LIN_LPUART module using blocking method.
 *  This function will check the checksum byte. If the checksum is correct, it
 *  will receive the frame data. Blocking means that the function does
 *  not return until the reception is complete.
 *
 * @param instance LIN_LPUART instance number
 * @param rxBuff  buffer containing 8-bit received data
 * @param rxSize the number of bytes to receive
 * @param timeout timeout value for timer sync control
 * @return An error code or lin_status_t
 */
lin_status_t LIN_LPUART_DRV_ReceiveFrameDataBlocking(uint32_t instance,
                                                     uint8_t * rxBuff,
                                                     uint8_t rxSize,
                                                     uint32_t timeout);

/*!
 * @brief Receives frame data through the LIN_LPUART module using non- blocking method.
 *  This function will check the checksum byte. If the checksum is correct, it will receive it with the frame data.
 *  Non-blocking  means that the function returns immediately.
 *  The application has to get the receive status to know when the reception is complete.
 *
 * @param instance LIN_LPUART instance number
 * @param rxBuff  buffer containing 8-bit received data
 * @param rxSize the number of bytes to receive
 * @return An error code or lin_status_t
 */
 lin_status_t LIN_LPUART_DRV_ReceiveFrameData(uint32_t instance,
                                              uint8_t * rxBuff,
                                              uint8_t rxSize);

/*!
 * @brief Aborts an on-going non-blocking transmission/reception.
 *  While performing a non-blocking transferring data, users can call this function
 *  to terminate immediately the transferring.
 *
 * @param instance LIN_LPUART instance number
 * @return An error code or lin_status_t
 */
lin_status_t LIN_LPUART_DRV_AbortTransferData(uint32_t instance);

/*!
 * @brief Get status of an on-going non-blocking reception
 *  While receiving frame data using non-blocking method, users can
 *  use this function to get status of that receiving.
 *  This function return the current event ID, LIN_RX_BUSY while receiving
 *  and return LIN_SUCCESS, or timeout (LIN_TIMEOUT) when the reception is complete.
 *  The bytesRemaining shows number of bytes that still needed to receive.
 *
 * @param instance LIN_LPUART instance number
 * @param bytesRemaining  Number of bytes still needed to receive
 * @return lin_status_t LIN_RX_BUSY, LIN_TIMEOUT or LIN_SUCCESS
 */
lin_status_t LIN_LPUART_DRV_GetReceiveStatus(uint32_t instance,
                                             uint8_t * bytesRemaining);

/*!
 * @brief This function puts current node to sleep mode
 * This function changes current node state to LIN_NODE_STATE_SLEEP_MODE
 *
 * @param instance LIN_LPUART instance number
 * @return An error code or lin_status_t
 */
lin_status_t LIN_LPUART_DRV_GoToSleepMode(uint32_t instance);

/*!
 * @brief Puts current LIN node to Idle state
 * This function changes current node state to LIN_NODE_STATE_IDLE
 *
 * @param instance LIN_LPUART instance number
 * @return An error code or lin_status_t
 */
lin_status_t LIN_LPUART_DRV_GotoIdleState(uint32_t instance);

/*!
 * @brief Sends a wakeup signal through the LIN_LPUART interface
 *
 * @param instance LIN_LPUART instance number
 * @return An error code or lin_status_t
 */
lin_status_t LIN_LPUART_DRV_SendWakeupSignal(uint32_t instance);

/*!
 * @brief Get the current LIN node state
 *
 * @param instance LIN_LPUART instance number
 * @return current LIN node state
 */
lin_node_state_t LIN_LPUART_DRV_GetCurrentNodeState(uint32_t instance);

/*!
 * @brief Callback function for Timer Interrupt Handler
 * Users shall initialize a timer (for example FTM) in Output campare mode
 * with period of 500 micro seconds. In timer IRQ handler, call this function.
 *
 * @param instance LIN_LPUART instance number
 * @return void
 */
void LIN_LPUART_DRV_TimeoutService(uint32_t instance);

/*!
 * @brief Set Value for Timeout Counter that is used in LIN_LPUART_DRV_TimeoutService
 *
 * @param instance LPUART instance number
 * @param timeoutValue  Timeout Value to be set
 * @return void
 */
void LIN_LPUART_DRV_SetTimeoutCounter(uint32_t instance,
                                      uint32_t timeoutValue);

/*!
 * @brief Sends frame header out through the LIN_LPUART module using a non-blocking method.
 *  This function sends LIN Break field, sync field then the ID with
 *  correct parity.
 *
 * @param instance LIN_LPUART instance number
 * @param id  Frame Identifier
 * @return An error code or lin_status_t
 */
lin_status_t LIN_LPUART_DRV_MasterSendHeader(uint32_t instance,
                                             uint8_t id);

/*!
 * @brief Enables LIN_LPUART hardware interrupts.
 *
 * @param instance LIN_LPUART instance number
 * @return An error code or lin_status_t
 */
lin_status_t LIN_LPUART_DRV_EnableIRQ(uint32_t instance);

/*!
 * @brief Disables LIN_LPUART hardware interrupts.
 *
 * @param instance LIN_LPUART instance number
 * @return An error code or lin_status_t
 */
lin_status_t LIN_LPUART_DRV_DisableIRQ(uint32_t instance);

/*!
 * @brief LIN_LPUART RX TX interrupt handler
 *
 * @param instance LIN_LPUART instance number
 * @return void
 */
void LIN_LPUART_DRV_IRQHandler(uint32_t instance);

#if defined(__cplusplus)
}
#endif

#endif /* FSL_LIN_LPUART_DRIVER_H_ */
/******************************************************************************/
/* EOF */
/******************************************************************************/
