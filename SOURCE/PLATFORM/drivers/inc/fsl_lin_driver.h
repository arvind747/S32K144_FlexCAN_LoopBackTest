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
 * LOSS OF USE, DATA, OR PROFITS, OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef FSL_LIN_DRIVER_H_
#define FSL_LIN_DRIVER_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdbool.h>
#include "fsl_device_registers.h"
#include "fsl_interrupt_manager.h"

/*!
 * @addtogroup lin_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define SLAVE  0
#define MASTER 1
#define MAKE_PARITY 0U
#define CHECK_PARITY 1U

/*! @brief Callback function to start time measurement */
typedef uint32_t (* lin_timer_start_t) (void);

/*! @brief Callback function to get time interval in micro seconds */
typedef uint32_t (* lin_timer_get_us_t) (uint32_t *us);

/*! @brief LIN hardware configuration structure */
typedef struct LinUserCfg {
    uint32_t baudRate;                        /*!< baudrate of LIN Hardware Interface to configure */
    bool nodeFunction;                        /*!< Node function as Master or Slave */
    bool autobaudEnable;                      /*!< Enable Autobaud feature */
    lin_timer_start_t timerStartCallback;     /*!< Callback function to start time measurement */
    lin_timer_get_us_t timerGetUsCallback;    /*!< Callback function to get time interval in micro seconds */
} lin_user_config_t;

/*! @brief Defines types for an enumerating event related to an Identifier. */
typedef enum LinEventId {
    LIN_NO_EVENT             = 0x00U,    /*!< No event yet */
    LIN_WAKEUP_SIGNAL        = 0x01U,    /*!< Received a wakeup signal */
    LIN_BAUDRATE_ADJUSTED    = 0x02U,    /*!< Indicate that baudrate was adjusted to Master's baudrate */
    LIN_RECV_BREAK_FIELD_OK  = 0x03U,    /*!< Indicate that correct Break Field was received */
    LIN_RECV_SYNC_OK         = 0x04U,    /*!< Sync byte is correct */
    LIN_RECV_SYNC_ERROR      = 0x05U,    /*!< Sync byte is incorrect */
    LIN_PID_OK               = 0x06U,    /*!< PID correct */
    LIN_PID_ERROR            = 0x07U,    /*!< PID incorrect */
    LIN_FRAME_ERROR          = 0x08U,    /*!< Framing Error */
    LIN_READBACK_ERROR       = 0x09U,    /*!< Readback data is incorrect */
    LIN_CHECKSUM_ERROR       = 0x0AU,    /*!< Checksum byte is incorrect */
    LIN_TX_COMPLETED         = 0x0BU,    /*!< Sending data completed */
    LIN_RX_COMPLETED         = 0x0CU,    /*!< Receiving data completed */
    LIN_NO_DATA_TIMEOUT      = 0x0DU,    /*!< No data timeout */
    LIN_BUS_ACTIVITY_TIMEOUT = 0x0EU,    /*!< Bus activity timeout */
    LIN_TIMEOUT_ERROR        = 0x0FU     /*!< Indicate that timeout has occured */
} lin_event_id_t;

/*! @brief Defines Error codes of the LIN driver. */
typedef enum {
    LIN_IFC_NOT_SUPPORT         = 0x00U,     /*!< This interface is not supported */
    LIN_INITIALIZED             = 0x01U,    /*!< LIN Hardware has been initialized */
    LIN_SUCCESS                 = 0x02U,    /*!< Successfully done */
    LIN_ERROR                   = 0x03U,    /*!< Error */
    LIN_TX_BUSY                 = 0x04U,    /*!< Transmitter is busy */
    LIN_RX_BUSY                 = 0x05U,    /*!< Receiver is busy */
    LIN_BUS_BUSY                = 0x06U,    /*!< Bus is busy */
    LIN_NO_TRANSFER_IN_PROGRESS = 0x07U,    /*!< No data transfer is in progress */
    LIN_TIMEOUT                 = 0x08U,     /*!< Timeout */
    LIN_LPUART_STAT_CLOCK_GATED_OFF = 0x09U /*!< LPUART is gated from clock manager */
} lin_status_t;

/*! @brief Define type for an enumerating LIN Node state. */
typedef enum NodeCurrentState {
    LIN_NODE_STATE_UNINIT                 = 0x00U,     /*!< Uninitialized state */
    LIN_NODE_STATE_SLEEP_MODE             = 0x01U,    /*!< Sleep mode state */
    LIN_NODE_STATE_IDLE                   = 0x02U,    /*!< Idle state */
    LIN_NODE_STATE_SEND_BREAK_FIELD       = 0x03U,    /*!< Send break field state */
    LIN_NODE_STATE_RECV_SYNC              = 0x04U,    /*!< Receive the synchronization byte state */
    LIN_NODE_STATE_SEND_PID               = 0x05U,    /*!< Send PID state */
    LIN_NODE_STATE_RECV_PID               = 0x06U,    /*!< Receive PID state */
    LIN_NODE_STATE_RECV_DATA              = 0x07U,    /*!< Receive data state */
    LIN_NODE_STATE_RECV_DATA_COMPLETED    = 0x08U,    /*!< Receive data completed state */
    LIN_NODE_STATE_SEND_DATA              = 0x09U,    /*!< Send data state */
    LIN_NODE_STATE_SEND_DATA_COMPLETED    = 0x0AU,    /*!< Send data completed state */
    LIN_NODE_STATE_CALLBACK               = 0x0BU    /*!< Procedure callback state */
} lin_node_state_t;


/*! @brief LIN Driver callback function type */
typedef void (* lin_callback_t)(uint32_t instance, void * linState);

/*!
 * @brief Runtime state of the LIN driver.
 *
 * Note that the caller provides memory for the driver state structures during
 * initialization because the driver does not statically allocate memory.
 */
typedef struct LinState {
    const uint8_t * txBuff;                     /*!< The buffer of data being sent. */
    uint8_t * rxBuff;                           /*!< The buffer of received data. */
    uint8_t cntByte;                            /*!< To count number of bytes already transmitted or received. */
    volatile uint8_t txSize;                    /*!< The remaining number of bytes to be transmitted. */
    volatile uint8_t rxSize;                    /*!< The remaining number of bytes to be received. */
    uint8_t checkSum;                           /*!< Checksum byte. */
    volatile bool isTxBusy;                     /*!< True if there is an active frame data transmission. */
    volatile bool isRxBusy;                     /*!< True if there is an active frame data reception. */
    volatile bool isBusBusy;                    /*!< True if there are data being transferring in bus */
    volatile bool isTxBlocking;                 /*!< True if transmit is blocking transaction. */
    volatile bool isRxBlocking;                 /*!< True if receive is blocking transaction. */
    lin_callback_t Callback;                    /*!< Callback function to invoke after receiving a byte or transmitting a byte. */
    uint8_t currentId;                          /*!< Current ID */
    uint8_t currentPid;                         /*!< Current PID */
    volatile lin_event_id_t currentEventId;     /*!< Current ID Event */
    volatile lin_node_state_t currentNodeState; /*!< Current Node state */
    volatile uint32_t timeoutCounter;           /*!< Value of the timeout counter */
    volatile bool timeoutCounterFlag;           /*!< Timeout counter flag */
    volatile bool baudrateEvalEnable;           /*!< Baudrate Evaluation Process Enable */
    volatile uint8_t fallingEdgeInterruptCount; /*!< Falling Edge count of a sync byte */
    uint32_t linSourceClockFreq;                 /*!< Frequency of the source clock for LIN */
} lin_state_t;

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name LIN DRIVER
 * @{
 */

/*!
 * @brief Initializes an instance LIN Hardware Interface for LIN Network.
 *
 * The caller provides memory for the driver state structures during initialization.
 * The user must select the LIN Hardware Interface clock source in the application to initialize the LIN Hardware Interface.
 *
 * @param instance LIN Hardware Interface instance number
 * @param linUserConfig user configuration structure of type #lin_user_config_t
 * @param linCurrentState pointer to the LIN Hardware Interface driver state structure
 * @return An error code or lin_status_t
 */
lin_status_t LIN_DRV_Init(uint32_t instance,
                          lin_user_config_t * linUserConfig,
                          lin_state_t * linCurrentState);

/*!
 * @brief Shuts down the LIN Hardware Interface by disabling interrupts and transmitter/receiver.
 *
 * @param instance LIN Hardware Interface instance number
 * @return An error code or lin_status_t
 */
lin_status_t LIN_DRV_Deinit(uint32_t instance);

/*!
 * @brief Installs callback function that is used for LIN_DRV_IRQHandler.
 *
 * @note After a callback is installed, it bypasses part of the LIN Hardware Interface IRQHandler logic.
 * Therefore, the callback needs to handle the indexes of txBuff and txSize.
 *
 * @param instance LIN Hardware Interface instance number.
 * @param function the LIN receive callback function.
 * @return Former LIN callback function pointer.
 */
lin_callback_t LIN_DRV_InstallCallback(uint32_t instance,
                                       lin_callback_t function);

/*!
 * @brief Sends Frame data out through the LIN Hardware Interface using blocking method.
 *  This function will calculate the checksum byte and send it with the frame data.
 *  Blocking means that the function does not return until the transmission is complete.
 *
 * @param instance LIN Hardware Interface instance number
 * @param txBuff source buffer containing 8-bit data chars to send
 * @param txSize the number of bytes to send
 * @param timeout timeout value for timer sync control
 * @return An error code or lin_status_t
 */
lin_status_t LIN_DRV_SendFrameDataBlocking(uint32_t instance,
                                           const uint8_t * txBuff,
                                           uint8_t txSize,
                                           uint32_t timeout);

/*!
 * @brief Sends frame data out through the LIN Hardware Interface using non-blocking method.
 *  This enables an a-sync method for transmitting data.
 *  Non-blocking  means that the function returns immediately.
 *  The application has to get the transmit status to know when the transmit is complete.
 *  This function will calculate the checksum byte and send it with the frame data.
 * @note Before using this function, users have to set timeout counter to an appropriate value by
 *  using LIN_DRV_SetTimeoutCounter(instance, timeoutValue).
 *
 * @param instance LIN Hardware Interface instance number
 * @param txBuff  source buffer containing 8-bit data chars to send
 * @param txSize  the number of bytes to send
 * @return An error code or lin_status_t
 */
lin_status_t LIN_DRV_SendFrameData(uint32_t instance,
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
 * @param instance LIN Hardware Interface instance number
 * @param bytesRemaining Number of bytes still needed to transmit
 * @return lin_status_t LIN_TX_BUSY, LIN_SUCCESS or LIN_TIMEOUT
 */
lin_status_t LIN_DRV_GetTransmitStatus(uint32_t instance,
                                       uint8_t * bytesRemaining);

/*!
 * @brief Receives frame data through the LIN Hardware Interface using blocking method.
 *  This function will check the checksum byte. If the checksum is correct, it
 *  will receive the frame data. Blocking means that the function does
 *  not return until the reception is complete.
 *
 * @param instance LIN Hardware Interface instance number
 * @param rxBuff  buffer containing 8-bit received data
 * @param rxSize the number of bytes to receive
 * @param timeout timeout value for timer sync control
 * @return An error code or lin_status_t
 */
lin_status_t LIN_DRV_ReceiveFrameDataBlocking(uint32_t instance,
                                              uint8_t * rxBuff,
                                              uint8_t rxSize,
                                              uint32_t timeout);

/*!
 * @brief Receives frame data through the LIN Hardware Interface using non- blocking method.
 *  This function will check the checksum byte. If the checksum is correct, it
 *  will receive it with the frame data.
 *  Non-blocking  means that the function returns immediately.
 *  The application has to get the receive status to know when the reception is complete.
 * @note Before using this function, users have to set timeout counter to an appropriate value by
 *  using LIN_DRV_SetTimeoutCounter(instance, timeoutValue).
 *
 * @param instance LIN Hardware Interface instance number
 * @param rxBuff  buffer containing 8-bit received data
 * @param rxSize the number of bytes to receive
 * @return An error code or lin_status_t
 */
lin_status_t LIN_DRV_ReceiveFrameData(uint32_t instance,
                                      uint8_t * rxBuff,
                                      uint8_t rxSize);

/*!
 * @brief Aborts an on-going non-blocking transmission/reception.
 *  While performing a non-blocking transferring data, users can call this function
 *  to terminate immediately the transferring.
 *
 * @param instance LIN Hardware Interface instance number
 * @return An error code or lin_status_t
 */
lin_status_t LIN_DRV_AbortTransferData(uint32_t instance);

/*!
 * @brief Get status of an on-going non-blocking reception
 *  While receiving frame data using non-blocking method, users can
 *  use this function to get status of that receiving.
 *  This function return the current event ID, LIN_RX_BUSY while receiving
 *  and return LIN_SUCCESS, or timeout (LIN_TIMEOUT) when the reception is complete.
 *  The bytesRemaining shows number of bytes that still needed to receive.
 *
 * @param instance LIN Hardware Interface instance number
 * @param bytesRemaining Number of bytes still needed to receive
 * @return lin_status_t LIN_RX_BUSY, LIN_TIMEOUT or LIN_SUCCESS
 */
lin_status_t LIN_DRV_GetReceiveStatus(uint32_t instance,
                                      uint8_t * bytesRemaining);

/*!
 * @brief Puts current LIN node to sleep mode
 * This function changes current node state to LIN_NODE_STATE_SLEEP_MODE
 *
 * @param instance LIN Hardware Interface instance number
 * @return An error code or lin_status_t
 */
lin_status_t LIN_DRV_GoToSleepMode(uint32_t instance);

/*!
 * @brief Puts current LIN node to Idle state
 * This function changes current node state to LIN_NODE_STATE_IDLE
 *
 * @param instance LIN Hardware Interface instance number
 * @return An error code or lin_status_t
 */
lin_status_t LIN_DRV_GotoIdleState(uint32_t instance);

/*!
 * @brief Sends a wakeup signal through the LIN Hardware Interface
 *
 * @param instance LIN Hardware Interface instance number
 * @return An error code or lin_status_t
 */
lin_status_t LIN_DRV_SendWakeupSignal(uint32_t instance);

/*!
 * @brief Get the current LIN node state
 *
 * @param instance LIN Hardware Interface instance number
 * @return current LIN node state
 */
lin_node_state_t LIN_DRV_GetCurrentNodeState (uint32_t instance);

/*!
 * @brief Callback function for Timer Interrupt Handler
 * Users shall initialize a timer (for example FTM) in Output campare mode
 * with period of 50 micro seconds. In timer IRQ handler, call this function.
 *
 * @param instance LIN Hardware Interface instance number
 * @return void
 */
void LIN_DRV_TimeoutService(uint32_t instance);

/*!
 * @brief Set Value for Timeout Counter that is used in LIN_DRV_TimeoutService
 *
 * @param instance LIN Hardware Interface instance number
 * @param timeoutValue  Timeout Value to be set
 * @return void
 */
void LIN_DRV_SetTimeoutCounter(uint32_t instance,
                               uint32_t timeoutValue);

/*!
 * @brief Sends frame header out through the LIN Hardware Interface using a non-blocking method.
 *  This function sends LIN Break field, sync field then the ID with
 *  correct parity.
 * @note Before using this function, users have to set timeout counter to an appropriate value by
 * using LIN_DRV_SetTimeoutCounter(instance, timeoutValue).
 *
 * @param instance LIN Hardware Interface instance number
 * @param id  Frame Identifier
 * @return An error code or lin_status_t
 */
lin_status_t LIN_DRV_MasterSendHeader(uint32_t instance,
                                      uint8_t id);

/*!
 * @brief Enables LIN hardware interrupts.
 *
 * @param instance LIN Hardware Interface instance number
 * @return An error code or lin_status_t
 */
lin_status_t LIN_DRV_EnableIRQ(uint32_t instance);

/*!
 * @brief Disables LIN hardware interrupts.
 *
 * @param instance LIN Hardware Interface instance number
 * @return An error code or lin_status_t
 */
lin_status_t LIN_DRV_DisableIRQ(uint32_t instance);

/*!
 * @brief Interrupt handler for LIN Hardware Interface.
 *
 * @param instance LIN Hardware Interface instance number
 * @return void
 */
void LIN_DRV_IRQHandler(uint32_t instance);


/*!
 * @brief Makes or checks parity bits. If action is checking parity, the function
 * returns ID value if parity bits are correct or 0xFF if parity bits are incorrect. If action
 * is making parity bits, then from input value of ID, the function returns PID.
 * This is not a public API as it is called by other API functions.
 *
 * @param PID PID byte in case of checking parity bits or ID byte in case of making parity bits.
 * @param typeAction TRUE for Checking parity bits, FALSE for making parity bits
 * @return 0xFF if parity bits are incorrect, ID in case of checking parity bits and
 * they are correct. Function returns PID in case of making parity bits.
 */
uint8_t LIN_DRV_ProcessParity(uint8_t PID,
                              uint8_t typeAction);

/*!
 * @brief Makes the checksum byte for a frame
 *
 * @param buffer Pointer to Tx buffer
 * @param sizeBuffer Number of bytes that are contained in the buffer.
 * @param PID PID byte.
 * @return the checksum byte.
 */
uint8_t LIN_DRV_MakeChecksumByte(const uint8_t *buffer,
                                 uint8_t sizeBuffer,
                                 uint8_t PID);

/* @} */

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* FSL_LIN_DRIVER_H_ */
/******************************************************************************/
/* EOF */
/******************************************************************************/
