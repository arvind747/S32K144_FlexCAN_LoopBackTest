/*
 * Copyright (c) 2014 - 2015, Freescale Semiconductor, Inc.
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

#ifndef __FSL_LPI2C_DRIVER_H__
#define __FSL_LPI2C_DRIVER_H__

#include <stddef.h>
#include <stdbool.h>
#include "fsl_device_registers.h"
#include "fsl_lpi2c_hal.h"

/*!
 * @addtogroup lpi2c_drv
 * @{
 */

/*******************************************************************************
 * Enumerations.
 ******************************************************************************/

/*! @cond DRIVER_INTERNAL_USE_ONLY */
/* Size of the master command queue. Worst case: 5 commands in High-Speed receive with 10-bit address:
   START + master code, REP START + addr_1 + tx, addr_2, REP START + addr_1 + rx, receive command */
#define LPI2C_MASTER_CMD_QUEUE_SIZE   5U
/*! @endcond */

/*! @brief I2C operating modes */
typedef enum _lpi2c_mode{
    LPI2C_STANDARD_MODE      = 0x0U,   /*!< Standard-mode (Sm), bidirectional data transfers up to 100 kbit/s */
    LPI2C_FAST_MODE          = 0X1U,   /*!< Fast-mode (Fm), bidirectional data transfers up to 400 kbit/s */
    LPI2C_FASTPLUS_MODE      = 0x2U,   /*!< Fast-mode Plus (Fm+), bidirectional data transfers up to 1 Mbit/s */
    LPI2C_HIGHSPEED_MODE     = 0x3U,   /*!< High-speed Mode (Hs-mode), bidirectional data transfers up to 3.4 Mbit/s */
    LPI2C_ULTRAFAST_MODE     = 0x4U    /*!< Ultra Fast Mode (UFm), unidirectional data transfers up to 5 Mbit/s */
} lpi2c_mode_t;

/*! @brief LPI2C slave events */
typedef enum lpi2c_slave_event
{
    LPI2C_SLAVE_EVENT_TX_REQ   = 0x02U,    /*!< The I2C slave received a request to transmit data */
    LPI2C_SLAVE_EVENT_RX_REQ   = 0x04U,    /*!< The I2C slave received a request to receive data */
    LPI2C_SLAVE_EVENT_TX_EMPTY = 0x10U,    /*!< The I2C slave transmit buffer empty */
    LPI2C_SLAVE_EVENT_RX_FULL  = 0x20U,    /*!< The I2C slave receive buffer full */
    LPI2C_SLAVE_EVENT_STOP = 0x80U,        /*!< The I2C slave STOP signal detected */
} lpi2c_slave_event_t;

/*! @brief LPI2C driver status codes */
typedef enum lpi2c_status {
    LPI2C_STATUS_SUCCESS            = 0U,  /*!< Operation successful */
    LPI2C_STATUS_FAIL               = 1U,  /*!< Operation failed */
    LPI2C_STATUS_BUSY               = 2U,  /*!< Already busy with a transfer */
    LPI2C_STATUS_RECEIVED_NACK      = 3U,  /*!< NACK signal received  */
    LPI2C_STATUS_SLAVE_TX_UNDERRUN  = 4U,  /*!< Slave TX underrun error */
    LPI2C_STATUS_SLAVE_RX_OVERRUN   = 5U,  /*!< Slave RX overrun error */
    LPI2C_STATUS_ARIBTRATION_LOST   = 6U,  /*!< Arbitration lost */
    LPI2C_STATUS_ABORTED            = 7U,  /*!< A transfer was aborted */
    LPI2C_STATUS_NO_TRANSFER        = 8U,  /*!< Attempt to abort a transfer when no transfer was in progress */
} lpi2c_status_t;

/*******************************************************************************
* Definitions
******************************************************************************/

/*!
 * @brief Defines the example structure
 *
 * This structure is used as an example.
 */

/*!
 * @brief LPI2C slave callback function
 *
 * Callback functions are called by the LPI2C slave when relevant events are detected on the I2C bus.
 * See type lpi2c_slave_event_t for a list of events. The callback can then react to the event, for example
 * providing the buffers for transmission or reception.
 */
typedef void (*lpi2c_slave_callback_t)(uint8_t instance, lpi2c_slave_event_t slaveEvent, void *userData);

 /*!
 * @brief Master configuration structure
 *
 * This structure is used to provide configuration parameters for the LPI2C master at initialization time.
 */
typedef struct _lpi2c_master_user_config_t
{
    uint16_t slaveAddress;         /*!< Slave address, 7-bit or 10-bit */
    bool is10bitAddr;              /*!< Selects 7-bit or 10-bit slave address */
    lpi2c_mode_t operatingMode;    /*!< I2C Operating mode */
    uint8_t masterCode;            /*!< Master code for High-speed mode. Valid range: 0-7. Unused in other operating modes */
    uint32_t baudRate;             /*!< The baud rate in hertz to use with current slave device */
    uint32_t baudRateHS;           /*!< Baud rate for High-speed mode. Unused in other operating modes */
} lpi2c_master_user_config_t;

/*!
 * @brief Slave configuration structure
 *
 * This structure is used to provide configuration parameters for the LPI2C slave at initialization time.
 */
typedef struct _lpi2c_slave_user_config_t
{
    uint16_t slaveAddress;                   /*!< Slave address, 7-bit or 10-bit */
    bool is10bitAddr;                        /*!< Selects 7-bit or 10-bit slave address */
    lpi2c_mode_t operatingMode;              /*!< I2C Operating mode */
    bool slaveListening;                     /*!< Slave mode (always listening or on demand only) */
    lpi2c_slave_callback_t slaveCallback;    /*!< Slave callback function. Note that this function will be 
                                                  called from the interrupt service routine, so its 
                                                  execution time should be as small as possible. It can be 
                                                  NULL if the slave is not in listening mode 
                                                  (slaveListening = false) */
    void *callbackParam;                     /*!< Parameter for the slave callback function */
} lpi2c_slave_user_config_t;

/*! @cond DRIVER_INTERNAL_USE_ONLY */
/* Master software command queue */
typedef struct _lpi2c_master_cmd_queue_t
{
    lpi2c_master_command_t cmd[LPI2C_MASTER_CMD_QUEUE_SIZE];
    uint8_t data[LPI2C_MASTER_CMD_QUEUE_SIZE];
    uint8_t writeIdx;
    uint8_t readIdx;
} lpi2c_master_cmd_queue_t;
/*! @endcond */

/*!
 * @brief Master internal context structure
 *
 * This structure is used by the master-mode driver for its internal logic. It must
 * be provided by the application through the LPI2C_DRV_MasterInit() function, then
 * it cannot be freed until the driver is de-initialized using LPI2C_DRV_MasterDeinit().
 * The application should make no assumptions about the content of this structure.
 */
typedef struct _lpi2c_master_state_t
{
/*! @cond DRIVER_INTERNAL_USE_ONLY */
    lpi2c_master_cmd_queue_t cmdQueue;    /* Software queue for commands, when LPI2C FIFO is not big enough */
    uint8_t * rxBuff;                     /* Pointer to receive data buffer */
    uint32_t rxSize;                      /* Size of receive data buffer */
    const uint8_t * txBuff;               /* Pointer to transmit data buffer */
    uint32_t txSize;                      /* Size of transmit data buffer */
    volatile lpi2c_status_t status;       /* Status of last driver operation */
    lpi2c_mode_t operatingMode;           /* I2C Operating mode */
    uint16_t slaveAddress;                /* Slave address */
    uint8_t masterCode;                   /* Master code for High-speed mode */
    volatile bool i2cIdle;                /* Idle/busy state of the driver */
    bool highSpeedInProgress;             /* High-speed communication is in progress */
    bool isBlocking;                      /* Specifies if current transfer is blocking */
    bool sendStop;                        /* Specifies if STOP condition must be generated after current transfer */
    bool is10bitAddr;                     /* Selects 7-bit or 10-bit slave address */
/*! @endcond */
} lpi2c_master_state_t;

/*!
 * @brief Slave internal context structure
 *
 * This structure is used by the slave-mode driver for its internal logic. It must
 * be provided by the application through the LPI2C_DRV_SlaveInit() function, then
 * it cannot be freed until the driver is de-initialized using LPI2C_DRV_SlaveDeinit().
 * The application should make no assumptions about the content of this structure.
 */
typedef struct _lpi2c_slave_state_t
{
/*! @cond DRIVER_INTERNAL_USE_ONLY */
    lpi2c_status_t status;                 /* The I2C slave status */
    volatile bool txBusy;                  /* Transmitter is busy */
    volatile bool rxBusy;                  /* Receiver is busy */
    uint32_t txSize;                       /* Size of the TX buffer*/
    uint32_t rxSize;                       /* Size of the RX buffer*/
    const uint8_t * txBuff;                /* Pointer to Tx Buffer*/
    uint8_t * rxBuff;                      /* Pointer to Rx Buffer*/
    lpi2c_mode_t operatingMode;            /* I2C Operating mode */
    bool slaveListening;                   /* Slave mode (always listening or on demand only) */
    bool txUnderrunWarning;                /* Possible slave tx underrun */
    lpi2c_slave_callback_t slaveCallback;  /* Slave callback function */
    void *callbackParam;                   /* Parameter for the slave callback function */
/*! @endcond */
} lpi2c_slave_state_t;

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @name LPI2C Driver
 * @{
 */

#if defined(__cplusplus)
extern "C" {
#endif


/*!
 * @brief Initialize the LPI2C master mode driver
 *
 * This function initializes the LPI2C driver in master mode.
 *
 * @param instance  LPI2C peripheral instance number
 * @param userConfigPtr    Pointer to the LPI2C master user configuration structure. The function
 *                         reads configuration data from this structure and initializes the
 *                         driver accordingly. The application may free this structure after
 *                         the function returns.
 * @param master    Pointer to the LPI2C master driver context structure. The driver uses
 *                  this memory area for its internal logic. The application must make no
 *                  assumptions about the content of this structure, and must not free this
 *                  memory until the driver is de-initialized using LPI2C_DRV_MasterDeinit().
 * @return    Error or success status returned by API
 */
lpi2c_status_t LPI2C_DRV_MasterInit(uint32_t instance,
                                    const lpi2c_master_user_config_t * userConfigPtr,
                                    lpi2c_master_state_t * master);


/*!
 * @brief De-initialize the LPI2C master mode driver
 *
 * This function de-initializes the LPI2C driver in master mode. The driver can't be used
 * again until reinitialized. The context structure is no longer needed by the driver and
 * can be freed after calling this function.
 *
 * @param instance  LPI2C peripheral instance number
 * @return    Error or success status returned by API
 */
lpi2c_status_t LPI2C_DRV_MasterDeinit(uint32_t instance);


/*!
 * @brief Get the currently configured baud rate
 *
 * This function returns the currently configured baud rate.
 *
 * @param instance  LPI2C peripheral instance number
 * @param baudRate  the current baud rate in hertz
 * @param baudRateHS  the baud rate in hertz for High-speed mode. Unused in other modes (can be NULL)
 * @return    Error or success status returned by API
 */
lpi2c_status_t LPI2C_DRV_MasterGetBaudRate(uint32_t instance, uint32_t *baudRate, uint32_t *baudRateHS);


/*!
 * @brief Set the baud rate for any subsequent I2C communication
 *
 * This function sets the baud rate (SCL frequency) for the I2C master. It can also 
 * change the operating mode. If the operating mode is High-Speed, a second baud rate 
 * must be provided for high-speed communication.
 * Note that due to module limitation not any baud rate can be achieved. The driver 
 * will set a baud rate as close as possible to the requested baud rate, but there may 
 * still be substantial differences, for example if requesting a high baud rate while 
 * using a low-frequency protocol clock for the LPI2C module. The application should 
 * call LPI2C_DRV_MasterGetBaudRate() after LPI2C_DRV_MasterSetBaudRate() to check 
 * what baud rate was actually set.
 *
 * @param instance  LPI2C peripheral instance number
 * @param operatingMode  I2C operating mode
 * @param baudRate  the baud rate in hertz to use by current slave device
 * @param baudRateHS  the baud rate in hertz for High-speed mode. Unused in other modes
 * @return    Error or success status returned by API
 */
lpi2c_status_t LPI2C_DRV_MasterSetBaudRate(uint32_t instance,
                                           const lpi2c_mode_t operatingMode,
                                           const uint32_t baudRate,
                                           const uint32_t baudRateHS);


/*!
 * @brief Set the slave address for any subsequent I2C communication
 *
 * This function sets the slave address which will be used for any future 
 * transfer initiated by the LPI2C master.
 *
 * @param instance  LPI2C peripheral instance number
 * @param address   slave address, 7-bit or 10-bit
 * @param is10bitAddr   specifies if provided address is 10-bit
 */
void LPI2C_DRV_MasterSetSlaveAddr(uint32_t instance, const uint16_t address, const bool is10bitAddr);


/*!
 * @brief Perform a blocking send transaction on the I2C bus
 *
 * This function sends a block of data to the currently configured slave address, and 
 * only returns when the transmission is complete.
 *
 * @param instance  LPI2C peripheral instance number
 * @param txBuff    pointer to the data to be transferred
 * @param txSize    length in bytes of the data to be transferred
 * @param sendStop    specifies whether or not to generate stop condition after the transmission
 * @return    Error or success status returned by API
 */
lpi2c_status_t LPI2C_DRV_MasterSendDataBlocking(uint32_t instance,
                                            const uint8_t * txBuff,
                                            uint32_t txSize,
                                            bool sendStop);


/*!
 * @brief Perform a non-blocking send transaction on the I2C bus
 *
 * This function starts the transmission of a block of data to the currently 
 * configured slave address and returns immediately. 
 * The rest of the transmission is handled by the interrupt service routine. 
 * Use LPI2C_DRV_MasterGetSendStatus() to check the progress of the transmission.
 *
 * @param instance  LPI2C peripheral instance number
 * @param txBuff    pointer to the data to be transferred
 * @param txSize    length in bytes of the data to be transferred
 * @param sendStop    specifies whether or not to generate stop condition after the transmission
 * @return    Error or success status returned by API
 */
lpi2c_status_t LPI2C_DRV_MasterSendData(uint32_t instance,
                                    const uint8_t * txBuff,
                                    uint32_t txSize,
                                    bool sendStop);


/*!
 * @brief Return the current status of the I2C master transmit
 *
 * This function can be called during a non-blocking transmission to check the 
 * status of the transfer.
 *
 * @param instance  LPI2C peripheral instance number
 * @param bytesRemaining   the number of remaining bytes in the active I2C transmits
 * @return    Error or success status returned by API
 */
lpi2c_status_t LPI2C_DRV_MasterGetSendStatus(uint32_t instance, uint32_t *bytesRemaining);


/*!
 * @brief Abort a non-blocking I2C Master transmission or reception
 *
 * @param instance  LPI2C peripheral instance number
 * @return    Error or success status returned by API
 */
lpi2c_status_t LPI2C_DRV_MasterAbortTransferData(uint32_t instance);


/*!
 * @brief Perform a blocking receive transaction on the I2C bus
 *
 * This function receives a block of data from the currently configured slave address, 
 * and only returns when the transmission is complete.
 *
 * @param instance  LPI2C peripheral instance number
 * @param rxBuff    pointer to the buffer where to store received data
 * @param rxSize    length in bytes of the data to be transferred
 * @param sendStop    specifies whether or not to generate stop condition after the reception
 * @return    Error or success status returned by API
 */
lpi2c_status_t LPI2C_DRV_MasterReceiveDataBlocking(uint32_t instance,
                                               uint8_t * rxBuff,
                                               uint32_t rxSize,
                                               bool sendStop);


/*!
 * @brief Perform a non-blocking receive transaction on the I2C bus
 *
 * This function starts the reception of a block of data from the currently 
 * configured slave address and returns immediately. 
 * The rest of the reception is handled by the interrupt service routine. 
 * Use LPI2C_DRV_MasterGetReceiveStatus() to check the progress of the reception.
 *
 * @param instance  LPI2C peripheral instance number
 * @param rxBuff    pointer to the buffer where to store received data
 * @param rxSize    length in bytes of the data to be transferred
 * @param sendStop    specifies whether or not to generate stop condition after the reception
 * @return    Error or success status returned by API
 */
lpi2c_status_t LPI2C_DRV_MasterReceiveData(uint32_t  instance,
                                       uint8_t * rxBuff,
                                       uint32_t rxSize,
                                       bool sendStop);


/*!
 * @brief Return the current status of the I2C master receive
 *
 * This function can be called during a non-blocking reception to check the 
 * status of the transfer.
 *
 * @param instance  LPI2C peripheral instance number
 * @param bytesRemaining   the number of remaining bytes to be received
 * @return    Error or success status returned by API
 */
lpi2c_status_t LPI2C_DRV_MasterGetReceiveStatus(uint32_t instance,
                                            uint32_t *bytesRemaining);


/*!
 * @brief Handle master operation when I2C interrupt occurs
 *
 * This is the interrupt service routine for the LPI2C master mode driver. It 
 * handles the rest of the transfer started by one of the send/receive functions.
 *
 * @param instance  LPI2C peripheral instance number
 */
void LPI2C_DRV_MasterIRQHandler(uint32_t instance);


/*!
 * @brief Initialize the I2C slave mode driver
 *
 * @param instance  LPI2C peripheral instance number
 * @param userConfigPtr    Pointer to the LPI2C slave user configuration structure. The function
 *                         reads configuration data from this structure and initializes the
 *                         driver accordingly. The application may free this structure after
 *                         the function returns.
 * @param slave     Pointer to the LPI2C slave driver context structure. The driver uses
 *                  this memory area for its internal logic. The application must make no
 *                  assumptions about the content of this structure, and must not free this
 *                  memory until the driver is de-initialized using LPI2C_DRV_SlaveDeinit().
 * @return    Error or success status returned by API
 */
lpi2c_status_t LPI2C_DRV_SlaveInit(uint32_t instance,
                               const lpi2c_slave_user_config_t * userConfigPtr,
                               lpi2c_slave_state_t * slave);


/*!
 * @brief De-initialize the I2C slave mode driver
 *
 * This function de-initializes the LPI2C driver in slave mode. The driver can't be used
 * again until reinitialized. The context structure is no longer needed by the driver and
 * can be freed after calling this function.
 *
 * @param instance  LPI2C peripheral instance number
 * @return    Error or success status returned by API
 */
lpi2c_status_t LPI2C_DRV_SlaveDeinit(uint32_t instance);


/*!
 * @brief Return the I2C slave run-time context structure
 *
 * This function returns a pointer to the context structure used by the LPI2C 
 * slave-mode driver.
 *
 * @param instance  LPI2C peripheral instance number
 * @return    pointer to the I2C slave run-time context structure
 */
lpi2c_slave_state_t * LPI2C_DRV_SlaveGetHandler(uint32_t instance);


/*!
 * @brief Provide a buffer for transmitting data
 *
 * This function provides a buffer from which the LPI2C slave-mode driver can 
 * transmit data. It can be called for example from the user callback provided at 
 * initialization time, when the driver reports events LPI2C_SLAVE_EVENT_TX_REQ or 
 * LPI2C_SLAVE_EVENT_TX_EMPTY.
 *
 * @param instance  LPI2C peripheral instance number
 * @param txBuff    pointer to the data to be transferred
 * @param txSize    length in bytes of the data to be transferred
 * @return    Error or success status returned by API
 */
lpi2c_status_t LPI2C_DRV_SlaveSetTxBuffer(uint32_t instance,
                                            const uint8_t * txBuff,
                                            uint32_t txSize);


/*!
 * @brief Provide a buffer for receiving data.
 *
 * This function provides a buffer in which the LPI2C slave-mode driver can 
 * store received data. It can be called for example from the user callback provided at 
 * initialization time, when the driver reports events LPI2C_SLAVE_EVENT_RX_REQ or 
 * LPI2C_SLAVE_EVENT_RX_FULL.
 *
 * @param instance  LPI2C peripheral instance number
 * @param rxBuff    pointer to the data to be transferred
 * @param rxSize    length in bytes of the data to be transferred
 * @return    Error or success status returned by API
 */
lpi2c_status_t LPI2C_DRV_SlaveSetRxBuffer(uint32_t instance,
                                            uint8_t * rxBuff,
                                            uint32_t  rxSize);


/*!
 * @brief Perform a non-blocking send transaction on the I2C bus
 *
 * Performs a non-blocking send transaction on the I2C bus when the slave is 
 * not in listening mode (initialized with slaveListening = false). It starts 
 * the transmission and returns immediately. The rest of the transmission is 
 * handled by the interrupt service routine. 
 * Use LPI2C_DRV_SlaveGetTransmitStatus() to check the progress of the transmission.
 *
 * @param instance  LPI2C peripheral instance number
 * @param txBuff    pointer to the data to be transferred
 * @param txSize    length in bytes of the data to be transferred
 * @return    Error or success status returned by API
 */
lpi2c_status_t LPI2C_DRV_SlaveSendData(uint32_t instance,
                                   const uint8_t * txBuff,
                                   uint32_t txSize);


/*!
 * @brief Perform a blocking send transaction on the I2C bus
 *
 * Performs a blocking send transaction on the I2C bus when the slave is 
 * not in listening mode (initialized with slaveListening = false). It sets 
 * up the transmission and then waits for the transfer to complete before 
 * returning.
 *
 * @param instance  LPI2C peripheral instance number
 * @param txBuff    pointer to the data to be transferred
 * @param txSize    length in bytes of the data to be transferred
 * @return    Error or success status returned by API
 */
lpi2c_status_t LPI2C_DRV_SlaveSendDataBlocking(uint32_t    instance,
                                           const uint8_t *  txBuff,
                                           uint32_t   txSize);


/*!
 * @brief Perform a non-blocking receive transaction on the I2C bus
 *
 * Performs a non-blocking receive transaction on the I2C bus when the slave is 
 * not in listening mode (initialized with slaveListening = false). It starts 
 * the reception and returns immediately. The rest of the reception is 
 * handled by the interrupt service routine. 
 * Use LPI2C_DRV_SlaveGetReceiveStatus() to check the progress of the reception.
 *
 * @param instance  LPI2C peripheral instance number
 * @param rxBuff    pointer to the buffer where to store received data
 * @param rxSize    length in bytes of the data to be transferred
 * @return    Error or success status returned by API
 */
lpi2c_status_t LPI2C_DRV_SlaveReceiveData(uint32_t   instance,
                                       uint8_t * rxBuff,
                                       uint32_t  rxSize);


/*!
 * @brief Perform a blocking receive transaction on the I2C bus
 *
 * Performs a blocking receive transaction on the I2C bus when the slave is 
 * not in listening mode (initialized with slaveListening = false). It sets 
 * up the reception and then waits for the transfer to complete before 
 * returning.
 *
 * @param instance  LPI2C peripheral instance number
 * @param rxBuff    pointer to the buffer where to store received data
 * @param rxSize    length in bytes of the data to be transferred
 * @return    Error or success status returned by API
 */
lpi2c_status_t LPI2C_DRV_SlaveReceiveDataBlocking(uint32_t instance,
                                       uint8_t  * rxBuff,
                                       uint32_t   rxSize);


/*!
 * @brief Return the current status of the I2C slave receive
 *
 * This function can be called during a non-blocking reception to check the 
 * status of the transfer.
 *
 * @param instance  LPI2C peripheral instance number
 * @param bytesRemaining   the number of remaining bytes in the active I2C transmit
 * @return    Error or success status returned by API
 */
lpi2c_status_t LPI2C_DRV_SlaveGetReceiveStatus(uint32_t instance,
                                            uint32_t *bytesRemaining);


/*!
 * @brief Return the current status of the I2C slave transmit
 *
 * This function can be called during a non-blocking transmission to check the 
 * status of the transfer.
 *
 * @param instance  LPI2C peripheral instance number
 * @param bytesRemaining   the number of remaining bytes in the active I2C receive
 * @return    Error or success status returned by API
 */
lpi2c_status_t LPI2C_DRV_SlaveGetTransmitStatus(uint32_t instance,
                                            uint32_t *bytesRemaining);


/*!
 * @brief Abort a non-blocking I2C Master transmission or reception
 *
 * @param instance  LPI2C peripheral instance number
 * @return    Error or success status returned by API
 */
lpi2c_status_t LPI2C_DRV_SlaveAbortTransferData(uint32_t instance);


/*!
 * @brief Handle slave operation when I2C interrupt occurs
 *
 * This is the interrupt service routine for the LPI2C slave mode driver. It 
 * handles any transfer initiated by an I2C master and notifies the application 
 * via the provided callback when relevant events occur.
 *
 * @param instance  LPI2C peripheral instance number
 */
void LPI2C_DRV_SlaveIRQHandler(uint32_t instance);


/*!
 * @brief Handle non-blocking slave or master operation when I2C interrupt occurs
 *
 * This interrupt service routine handles any I2C interrupt by calling either 
 * LPI2C_DRV_MasterIRQHandler() or LPI2C_DRV_SlaveIRQHandler(), as appropriate.
 *
 * @param instance  LPI2C peripheral instance number
 */
void LPI2C_DRV_IRQHandler(uint32_t instance);


/*! @}*/
#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __FSL_LPI2C_DRIVER_H__ */
/*******************************************************************************
 * EOF
 ******************************************************************************/
