/*
 * Copyright (c) 2014 - 2016, Freescale Semiconductor, Inc.
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

#ifndef __FSL_FLEXIO_I2C_DRIVER_H__
#define __FSL_FLEXIO_I2C_DRIVER_H__

#include <stddef.h>
#include <stdbool.h>
#include "fsl_device_registers.h"
#include "fsl_flexio_hal.h"

/*!
 * @addtogroup flexio_i2c_drv
 * @{
 */

/*******************************************************************************
 * Enumerations.
 ******************************************************************************/


/*! @brief Maximum size of a transfer. The restriction is that the total number of SCL 
   edges must not exceed 8 bits, such that it can be programmed in the upper part of the 
   timer compare register. There are 2 SCL edges per bit, 9 bits per byte (including ACK).
   The extra 1 is for the STOP condition. */
#define FLEXIO_I2C_MAX_SIZE   (((uint32_t)((0xFFU - 1U) / 18U)) - 1U)

/*! @brief FLEXIO_I2C driver status codes */
typedef enum _flexio_i2c_status {
    FLEXIO_I2C_STATUS_SUCCESS            = 0U,  /*!< Operation successful */
    FLEXIO_I2C_STATUS_FAIL               = 1U,  /*!< Operation failed */
    FLEXIO_I2C_STATUS_SIZE               = 2U,  /*!< Transfer size is not in admissible range */
    FLEXIO_I2C_STATUS_BUSY               = 3U,  /*!< Already busy with a transfer */
    FLEXIO_I2C_STATUS_BUS_BUSY           = 4U,  /*!< I2C bus is busy, cannot start transfer */
    FLEXIO_I2C_STATUS_ABORTED            = 5U,  /*!< Transfer aborted */
    FLEXIO_I2C_STATUS_NACK               = 6U,  /*!< Received NACK */
    FLEXIO_I2C_STATUS_TX_UNDERFLOW       = 7U,  /*!< Transmitter underflow */
    FLEXIO_I2C_STATUS_RX_OVERFLOW        = 8U,  /*!< Receiver overflow */
} flexio_i2c_status_t;

/*! @brief Driver type: interrupts/polling/DMA */
typedef enum _flexio_i2c_driver_type {
    FLEXIO_I2C_DRIVER_TYPE_INTERRUPTS    = 0U,  /*!< Driver uses interrupts for data transfers */
    FLEXIO_I2C_DRIVER_TYPE_POLLING       = 1U,  /*!< Driver is based on polling */
    FLEXIO_I2C_DRIVER_TYPE_DMA           = 2U,  /*!< Driver uses DMA for data transfers */
} flexio_i2c_driver_type_t;


/*******************************************************************************
* Definitions
******************************************************************************/

 /*!
 * @brief Master configuration structure
 *
 * This structure is used to provide configuration parameters for the flexio_i2c master at initialization time.
 */
typedef struct _flexio_i2c_master_user_config_t
{
    uint16_t slaveAddress;                  /*!< Slave address, 7-bit */
    flexio_i2c_driver_type_t driverType;    /*!< Driver type: interrupts/polling/DMA */
    uint32_t baudRate;                      /*!< Baud rate in hertz */
    uint8_t sdaPin;                         /*!< Flexio pin to use as I2C SDA pin */
    uint8_t sclPin;                         /*!< Flexio pin to use as I2C SCL pin */
} flexio_i2c_master_user_config_t;


/*!
 * @brief Master internal context structure
 *
 * This structure is used by the driver for its internal logic. It must
 * be provided by the application through the FLEXIO_I2C_DRV_MasterInit() function, then
 * it cannot be freed until the driver is de-initialized using FLEXIO_I2C_DRV_MasterDeinit().
 * The application should make no assumptions about the content of this structure.
 */
typedef struct _flexio_i2c_master_state_t
{
/*! @cond DRIVER_INTERNAL_USE_ONLY */
    uint8_t *data;                         /* Transmit/Receive buffer. */
    uint32_t txRemainingBytes;             /* Number of remaining bytes to be transmitted. */
    uint32_t rxRemainingBytes;             /* Number of remaining bytes to be received. */
    uint32_t baudRate;                     /* Baud rate in hertz */
    uint16_t slaveAddress;                 /* Slave address */
    flexio_i2c_driver_type_t driverType;   /* Driver type: interrupts/polling/DMA */
    flexio_i2c_status_t status;            /* Current status of the driver */
    bool receive;                          /* Transfer direction, true = receive, false = transmit */
    bool addrReceived;                     /* Indicated start of receive (after address transmission) */
    volatile bool driverIdle;              /* Idle/busy state of the driver */
    bool sendStop;                         /* Specifies if STOP condition must be generated after current transfer */
    bool needReinit;                       /* Re-initialization needed after a forced stop */
    uint8_t sdaPin;                        /* Flexio pin to use as I2C SDA pin */
    uint8_t sclPin;                        /* Flexio pin to use as I2C SCL pin */
/*! @endcond */
} flexio_i2c_master_state_t;


/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @name FLEXIO_I2C Driver
 * @{
 */

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initialize the FLEXIO_I2C master mode driver
 *
 * This function initializes the FLEXIO_I2C driver in master mode.
 *
 * @param instance  FLEXIO peripheral instance number
 * @param userConfigPtr    Pointer to the FLEXIO_I2C master user configuration structure. The function
 *                         reads configuration data from this structure and initializes the
 *                         driver accordingly. The application may free this structure after
 *                         the function returns.
 * @param master    Pointer to the FLEXIO_I2C master driver context structure. The driver uses
 *                  this memory area for its internal logic. The application must make no
 *                  assumptions about the content of this structure, and must not free this
 *                  memory until the driver is de-initialized using FLEXIO_I2C_DRV_MasterDeinit().
 * @return    Error or success status returned by API
 */
flexio_i2c_status_t FLEXIO_I2C_DRV_MasterInit(uint32_t instance,
                                              const flexio_i2c_master_user_config_t * userConfigPtr,
                                              flexio_i2c_master_state_t * master);

                                              
/*!
 * @brief De-initialize the FLEXIO_I2C master mode driver
 *
 * This function de-initializes the FLEXIO_I2C driver in master mode. The driver can't be used
 * again until reinitialized. The context structure is no longer needed by the driver and
 * can be freed after calling this function.
 *
 * @param instance  FLEXIO peripheral instance number
 * @return    Error or success status returned by API
 */
flexio_i2c_status_t FLEXIO_I2C_DRV_MasterDeinit(uint32_t instance);


/*!
 * @brief Set the baud rate for any subsequent I2C communication
 *
 * This function sets the baud rate (SCL frequency) for the I2C master.
 * Note that due to module limitation not any baud rate can be achieved. The driver 
 * will set a baud rate as close as possible to the requested baud rate, but there may 
 * still be substantial differences, for example if requesting a high baud rate while 
 * using a low-frequency FlexIO clock. The application should call 
 * FLEXIO_I2C_DRV_MasterGetBaudRate() after FLEXIO_I2C_DRV_MasterSetBaudRate() to check 
 * what baud rate was actually set.
 *
 * @param instance  FLEXIO peripheral instance number
 * @param baudRate  the desired baud rate in hertz
 * @return    Error or success status returned by API
 */
flexio_i2c_status_t FLEXIO_I2C_DRV_MasterSetBaudRate(uint32_t instance, uint32_t baudRate);


/*!
 * @brief Get the currently configured baud rate
 *
 * This function returns the currently configured I2C baud rate.
 *
 * @param instance  FLEXIO peripheral instance number
 * @param baudRate  the current baud rate in hertz
 * @return    Error or success status returned by API
 */
flexio_i2c_status_t FLEXIO_I2C_DRV_MasterGetBaudRate(uint32_t instance, uint32_t *baudRate);


/*!
 * @brief Set the slave address for any subsequent I2C communication
 *
 * This function sets the slave address which will be used for any future transfer.
 *
 * @param instance  FLEXIO peripheral instance number
 * @param address   slave address, 7-bit
 * @return    Error or success status returned by API
 */
flexio_i2c_status_t FLEXIO_I2C_DRV_MasterSetSlaveAddr(uint32_t instance, const uint16_t address);


/*!
 * @brief Perform a non-blocking send transaction on the I2C bus
 *
 * This function starts the transmission of a block of data to the currently 
 * configured slave address and returns immediately. 
 * The rest of the transmission is handled by the interrupt service routine (if the driver 
 * is initialized in interrupt mode) or by the FLEXIO_I2C_DRV_MasterGetStatus function (if 
 * the driver is initialized in polling mode). 
 * Use FLEXIO_I2C_DRV_MasterGetStatus() to check the progress of the transmission.
 *
 * @param instance  FLEXIO peripheral instance number
 * @param txBuff    pointer to the data to be transferred
 * @param txSize    length in bytes of the data to be transferred
 * @param sendStop    specifies whether or not to generate stop condition after the transmission
 * @return    Error or success status returned by API
 */
flexio_i2c_status_t FLEXIO_I2C_DRV_MasterSendData(uint32_t instance,
                                                  const uint8_t * txBuff,
                                                  uint32_t txSize,
                                                  bool sendStop);



/*!
 * @brief Perform a blocking send transaction on the I2C bus
 *
 * This function sends a block of data to the currently configured slave address, and 
 * only returns when the transmission is complete.
 *
 * @param instance  FLEXIO peripheral instance number
 * @param txBuff    pointer to the data to be transferred
 * @param txSize    length in bytes of the data to be transferred
 * @param sendStop    specifies whether or not to generate stop condition after the transmission
 * @return    Error or success status returned by API
 */
flexio_i2c_status_t FLEXIO_I2C_DRV_MasterSendDataBlocking(uint32_t instance,
                                                          const uint8_t * txBuff,
                                                          uint32_t txSize,
                                                          bool sendStop);


/*!
 * @brief Perform a non-blocking receive transaction on the I2C bus
 *
 * This function starts the reception of a block of data from the currently 
 * configured slave address and returns immediately. 
 * The rest of the transmission is handled by the interrupt service routine (if the driver 
 * is initialized in interrupt mode) or by the FLEXIO_I2C_DRV_MasterGetStatus function (if 
 * the driver is initialized in polling mode). 
 * Use FLEXIO_I2C_DRV_MasterGetStatus() to check the progress of the reception.
 *
 * @param instance  FLEXIO peripheral instance number
 * @param rxBuff    pointer to the buffer where to store received data
 * @param rxSize    length in bytes of the data to be transferred
 * @param sendStop    specifies whether or not to generate stop condition after the reception
 * @return    Error or success status returned by API
 */
flexio_i2c_status_t FLEXIO_I2C_DRV_MasterReceiveData(uint32_t  instance,
                                                     uint8_t * rxBuff,
                                                     uint32_t rxSize,
                                                     bool sendStop);


/*!
 * @brief Perform a blocking receive transaction on the I2C bus
 *
 * This function receives a block of data from the currently configured slave address, 
 * and only returns when the transmission is complete.
 *
 * @param instance  FLEXIO peripheral instance number
 * @param rxBuff    pointer to the buffer where to store received data
 * @param rxSize    length in bytes of the data to be transferred
 * @param sendStop    specifies whether or not to generate stop condition after the reception
 * @return    Error or success status returned by API
 */
flexio_i2c_status_t FLEXIO_I2C_DRV_MasterReceiveDataBlocking(uint32_t instance,
                                                        uint8_t * rxBuff,
                                                        uint32_t rxSize,
                                                        bool sendStop);


/*!
 * @brief Aborts a non-blocking I2C master transaction
 *
 * This function aborts a non-blocking I2C transfer.
 *
 * @param instance  FLEXIO peripheral instance number
 * @return    Error or success status returned by API
 */
flexio_i2c_status_t FLEXIO_I2C_DRV_MasterTransferAbort(uint32_t instance);


/*!
 * @brief Get the status of the current non-blocking I2C master transaction
 *
 * This function returns the current status of a non-blocking I2C master transaction.
 * A return code of FLEXIO_I2C_STATUS_BUSY means the transfer is still in progress.
 * Otherwise the function returns a status reflecting the outcome of the last transfer.
 * When the driver is initialized in polling mode this function also advances the transfer
 * by checking and handling the transmit and receive events, so it must be called
 * frequently to avoid overflows or underflows.
 *
 * @param instance        FLEXIO peripheral instance number
 * @param bytesRemaining  the remaining number of bytes to be transferred
 * @return    Error or success status returned by API
 */
flexio_i2c_status_t FLEXIO_I2C_DRV_MasterGetStatus(uint32_t instance, uint32_t *bytesRemaining);


/*! @}*/
#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __FSL_FLEXIO_I2C_DRIVER_H__ */
/*******************************************************************************
 * EOF
 ******************************************************************************/
