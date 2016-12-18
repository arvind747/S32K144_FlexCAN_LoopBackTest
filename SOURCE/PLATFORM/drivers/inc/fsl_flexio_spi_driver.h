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

#ifndef __FSL_FLEXIO_SPI_DRIVER_H__
#define __FSL_FLEXIO_SPI_DRIVER_H__

#include <stddef.h>
#include <stdbool.h>
#include "fsl_device_registers.h"
#include "fsl_flexio_hal.h"

/*!
 * @addtogroup flexio_spi_drv
 * @{
 */

/*******************************************************************************
 * Enumerations.
 ******************************************************************************/


/*! @brief FLEXIO_SPI driver status codes */
typedef enum _flexio_spi_status {
    FLEXIO_SPI_STATUS_SUCCESS            = 0U,  /*!< Operation successful */
    FLEXIO_SPI_STATUS_FAIL               = 1U,  /*!< Operation failed */
    FLEXIO_SPI_STATUS_BUSY               = 2U,  /*!< Already busy with a transfer */
    FLEXIO_SPI_STATUS_ABORTED            = 3U,  /*!< Transfer aborted */
    FLEXIO_SPI_STATUS_TX_UNDERFLOW       = 4U,  /*!< Transmitter underflow */
    FLEXIO_SPI_STATUS_RX_OVERFLOW        = 5U,  /*!< Receiver overflow */
} flexio_spi_status_t;

/*! @brief Transfer type: blocking/non-blocking */
typedef enum _flexio_spi_transfer_type {
    FLEXIO_SPI_TRANSFER_BLOCKING         = 0U,  /*!< Blocking transfer */
    FLEXIO_SPI_TRANSFER_NON_BLOCKING     = 1U,  /*!< Non-Blocking transfer */
} flexio_spi_transfer_type_t;

/*! @brief Order in which the data bits are transferred */
typedef enum _flexio_spi_transfer_bit_order {
    FLEXIO_SPI_TRANSFER_MSB_FIRST        = 0U,  /*!< Transmit data starting with most significant bit */
    FLEXIO_SPI_TRANSFER_LSB_FIRST        = 1U,  /*!< Transmit data starting with least significant bit */
} flexio_spi_transfer_bit_order_t;

/*! @brief Size of transferred data in bytes */
typedef enum _flexio_spi_transfer_size {
    FLEXIO_SPI_TRANSFER_1BYTE            = 1U,  /*!< Data size is 1-byte  */
    FLEXIO_SPI_TRANSFER_2BYTE            = 2U,  /*!< Data size is 2-bytes */
    FLEXIO_SPI_TRANSFER_4BYTE            = 4U,  /*!< Data size is 4-bytes */
} flexio_spi_transfer_size_t;

/*! @brief Driver type: interrupts/polling/DMA */
typedef enum _flexio_spi_driver_type {
    FLEXIO_SPI_DRIVER_TYPE_INTERRUPTS    = 0U,  /*!< Driver uses interrupts for data transfers */
    FLEXIO_SPI_DRIVER_TYPE_POLLING       = 1U,  /*!< Driver is based on polling */
    FLEXIO_SPI_DRIVER_TYPE_DMA           = 2U,  /*!< Driver uses DMA for data transfers */
} flexio_spi_driver_type_t;


/*******************************************************************************
* Definitions
******************************************************************************/


 /*!
 * @brief FlexIO SPI transfer structure
 *
 * This structure is used to provide the parameters for an SPI transfer.
 */
typedef struct _flexio_spi_transfer
{
    uint8_t *txData; /*!< Transmit buffer. */
    uint8_t *rxData; /*!< Receive buffer. */
    uint32_t dataSize; /*!< Number of bytes to transfer. */
    flexio_spi_transfer_type_t transfer_type;  /*!< Transfer type: blocking / non-blocking */
    flexio_spi_transfer_bit_order_t bit_order; /*!< Bit order: LSB-first / MSB-first */
    flexio_spi_transfer_size_t transfer_size;  /*!< Transfer size in bytes: 1/2/4 */
} flexio_spi_transfer_t;

 /*!
 * @brief Master configuration structure
 *
 * This structure is used to provide configuration parameters for the flexio_spi master at initialization time.
 */
typedef struct _flexio_spi_master_user_config_t
{
    flexio_spi_driver_type_t driverType; /*!< Driver type: interrupts/polling/DMA */
    uint8_t clockPolarity;     /*!< Clock Polarity (CPOL) 0 = active-high clock; 1 = active-low clock */
    uint8_t clockPhase;        /*!< Clock Phase (CPHA) 0 = sample on leading clock edge; 1 = sample on trailing clock edge */
    uint32_t baudRate;         /*!< Baud rate in hertz */
    uint8_t mosiPin;           /*!< Flexio pin to use as MOSI pin */
    uint8_t misoPin;           /*!< Flexio pin to use as MISO pin */
    uint8_t sckPin;            /*!< Flexio pin to use as SCK pin  */
    uint8_t ssPin;             /*!< Flexio pin to use as SS pin   */
} flexio_spi_master_user_config_t;

/*!
 * @brief Slave configuration structure
 *
 * This structure is used to provide configuration parameters for the flexio_spi slave at initialization time.
 */
typedef struct _flexio_spi_slave_user_config_t
{
    flexio_spi_driver_type_t driverType; /*!< Driver type: interrupts/polling/DMA */
    uint8_t clockPolarity;     /*!< Clock Polarity (CPOL) 0 = active-low clock; 1 = active-high clock */
    uint8_t clockPhase;        /*!< Clock Phase (CPHA) 0 = sample on leading clock edge; 1 = sample on trailing clock edge */
    uint8_t mosiPin;           /*!< Flexio pin to use as MOSI pin */
    uint8_t misoPin;           /*!< Flexio pin to use as MISO pin */
    uint8_t sckPin;            /*!< Flexio pin to use as SCK pin  */
    uint8_t ssPin;             /*!< Flexio pin to use as SS pin   */
} flexio_spi_slave_user_config_t;


/*!
 * @brief Master internal context structure
 *
 * This structure is used by the master-mode driver for its internal logic. It must
 * be provided by the application through the FLEXIO_SPI_DRV_MasterInit() function, then
 * it cannot be freed until the driver is de-initialized using FLEXIO_SPI_DRV_MasterDeinit().
 * The application should make no assumptions about the content of this structure.
 */
typedef struct _flexio_spi_master_state_t
{
/*! @cond DRIVER_INTERNAL_USE_ONLY */
    uint8_t *txData;                           /* Transmit buffer. */
    uint8_t *rxData;                           /* Receive buffer. */
    uint32_t txRemainingBytes;                 /* Number of remaining bytes to be transmitted. */
    uint32_t rxRemainingBytes;                 /* Number of remaining bytes to be received. */
    flexio_spi_driver_type_t driverType;       /* Driver type: interrupts/polling/DMA */
    uint8_t clockPolarity;                     /* Clock Polarity (CPOL) 0 = active-high clock; 1 = active-low clock */
    uint8_t clockPhase;                        /* Clock Phase (CPHA) 0 = sample on leading clock edge; 1 = sample on trailing clock edge */
    uint32_t baudRate;                         /* Baud rate in hertz */
    uint8_t mosiPin;                           /* Flexio pin to use as MOSI pin */
    uint8_t misoPin;                           /* Flexio pin to use as MISO pin */
    uint8_t sckPin;                            /* Flexio pin to use as SCK pin  */
    uint8_t ssPin;                             /* Flexio pin to use as SS pin   */
    flexio_spi_transfer_bit_order_t bit_order; /* Bit order: LSB-first / MSB-first */
    flexio_spi_transfer_size_t transfer_size;  /* Transfer size in bytes: 1/2/4 */
    flexio_spi_status_t status;                /* Current status of the driver */
    volatile bool driverIdle;                  /* Idle/busy state of the driver */
    bool needReinit;                           /* Re-initialization needed after a forced stop */
    bool master;                               /* Specifies if the current instance was initialized as master */
/*! @endcond */
} flexio_spi_master_state_t;

/*!
 * @brief Slave internal context structure
 *
 * This structure is used by the slave-mode driver for its internal logic. It must
 * be provided by the application through the FLEXIO_SPI_DRV_SlaveInit() function, then
 * it cannot be freed until the driver is de-initialized using FLEXIO_SPI_DRV_SlaveDeinit().
 * The application should make no assumptions about the content of this structure.
 */
typedef flexio_spi_master_state_t flexio_spi_slave_state_t;



/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @name FLEXIO_SPI Driver
 * @{
 */

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initialize the FLEXIO_SPI master mode driver
 *
 * This function initializes the FLEXIO_SPI driver in master mode.
 *
 * @param instance  FLEXIO peripheral instance number
 * @param userConfigPtr    Pointer to the FLEXIO_SPI master user configuration structure. The function
 *                         reads configuration data from this structure and initializes the
 *                         driver accordingly. The application may free this structure after
 *                         the function returns.
 * @param master    Pointer to the FLEXIO_SPI master driver context structure. The driver uses
 *                  this memory area for its internal logic. The application must make no
 *                  assumptions about the content of this structure, and must not free this
 *                  memory until the driver is de-initialized using FLEXIO_SPI_DRV_MasterDeinit().
 * @return    Error or success status returned by API
 */
flexio_spi_status_t FLEXIO_SPI_DRV_MasterInit(uint32_t instance,
                                              const flexio_spi_master_user_config_t * userConfigPtr,
                                              flexio_spi_master_state_t * master);

                                              
/*!
 * @brief De-initialize the FLEXIO_SPI master mode driver
 *
 * This function de-initializes the FLEXIO_SPI driver in master mode. The driver can't be used
 * again until reinitialized. The context structure is no longer needed by the driver and
 * can be freed after calling this function.
 *
 * @param instance  FLEXIO peripheral instance number
 * @return    Error or success status returned by API
 */
flexio_spi_status_t FLEXIO_SPI_DRV_MasterDeinit(uint32_t instance);


/*!
 * @brief Set the baud rate for any subsequent SPI communication
 *
 * This function sets the baud rate for the SPI master.
 * Note that due to module limitation not any baud rate can be achieved. The driver 
 * will set a baud rate as close as possible to the requested baud rate, but there may 
 * still be substantial differences, for example if requesting a high baud rate while 
 * using a low-frequency FlexIO clock. The application should call 
 * FLEXIO_SPI_DRV_MasterGetBaudRate() after FLEXIO_SPI_DRV_MasterSetBaudRate() to check 
 * what baud rate was actually set.
 *
 * @param instance  FLEXIO peripheral instance number
 * @param baudRate  the desired baud rate in hertz
 * @return    Error or success status returned by API
 */
flexio_spi_status_t FLEXIO_SPI_DRV_MasterSetBaudRate(uint32_t instance, uint32_t baudRate);


/*!
 * @brief Get the currently configured baud rate
 *
 * This function returns the currently configured SPI baud rate.
 *
 * @param instance  FLEXIO peripheral instance number
 * @param baudRate  the current baud rate in hertz
 * @return    Error or success status returned by API
 */
flexio_spi_status_t FLEXIO_SPI_DRV_MasterGetBaudRate(uint32_t instance, uint32_t *baudRate);


/*!
 * @brief Perform an SPI master transaction
 *
 * This function performs an SPI full-duplex transaction, transmit and receive in parallel.
 * If only transmit or receive are required, it is possible to provide NULL pointers for 
 * transfer->txData or transfer->rxData. Depending of the setting in transfer->transfer_type,
 * the transfer can be either blocking (the function only returns when the transfer is complete)
 * or non-blocking (the function only initiates the transfer and then returns, leaving the 
 * transfer to complete asynchronously). For non-blocking transfers 
 * FLEXIO_SPI_DRV_MasterGetStatus() must be called to check the status of the transfer.
 *
 * @param instance  FLEXIO peripheral instance number
 * @param transfer  SPI transfer structure for the current transfer
 * @return    Error or success status returned by API
 */
flexio_spi_status_t FLEXIO_SPI_DRV_MasterTransfer(uint32_t instance,
                                                  const flexio_spi_transfer_t *transfer);


                                                             
/*!
 * @brief Aborts a non-blocking SPI master transaction
 *
 * This function aborts a non-blocking SPI transfer.
 *
 * @param instance  FLEXIO peripheral instance number
 * @return    Error or success status returned by API
 */
flexio_spi_status_t FLEXIO_SPI_DRV_MasterTransferAbort(uint32_t instance);


/*!
 * @brief Get the status of the current non-blocking SPI master transaction
 *
 * This function returns the current status of a non-blocking SPI master transaction.
 * A return code of FLEXIO_SPI_STATUS_BUSY means the transfer is still in progress.
 * Otherwise the function returns a status reflecting the outcome of the last transfer.
 * When the driver is initialized in polling mode this function also advances the transfer
 * by checking and handling the transmit and receive events, so it must be called
 * frequently to avoid overflows or underflows.
 *
 * @param instance        FLEXIO peripheral instance number
 * @param bytesRemaining  the remaining number of bytes to be transferred
 * @return    Error or success status returned by API
 */
flexio_spi_status_t FLEXIO_SPI_DRV_MasterGetStatus(uint32_t instance, uint32_t *bytesRemaining);


/*!
 * @brief Initialize the FLEXIO_SPI slave mode driver
 *
 * This function initializes the FLEXIO_SPI driver in slave mode.
 *
 * @param instance  FLEXIO peripheral instance number
 * @param userConfigPtr    Pointer to the FLEXIO_SPI slave user configuration structure. The function
 *                         reads configuration data from this structure and initializes the
 *                         driver accordingly. The application may free this structure after
 *                         the function returns.
 * @param slave     Pointer to the FLEXIO_SPI slave driver context structure. The driver uses
 *                  this memory area for its internal logic. The application must make no
 *                  assumptions about the content of this structure, and must not free this
 *                  memory until the driver is de-initialized using FLEXIO_SPI_DRV_SlaveDeinit().
 * @return    Error or success status returned by API
 */
flexio_spi_status_t FLEXIO_SPI_DRV_SlaveInit(uint32_t instance,
                                             const flexio_spi_slave_user_config_t * userConfigPtr,
                                             flexio_spi_slave_state_t * slave);

/*!
 * @brief De-initialize the FLEXIO_SPI slave mode driver
 *
 * This function de-initializes the FLEXIO_SPI driver in slave mode. The driver can't be used
 * again until reinitialized. The context structure is no longer needed by the driver and
 * can be freed after calling this function.
 *
 * @param instance  FLEXIO peripheral instance number
 * @return    Error or success status returned by API
 */
static inline flexio_spi_status_t FLEXIO_SPI_DRV_SlaveDeinit(uint32_t instance)
{
    return FLEXIO_SPI_DRV_MasterDeinit(instance);
}


/*!
 * @brief Perform an SPI slave transaction
 *
 * This function performs an SPI full-duplex transaction, transmit and receive in parallel.
 * If only transmit or receive are required, it is possible to provide NULL pointers for 
 * transfer->txData or transfer->rxData. Depending of the setting in transfer->transfer_type,
 * the transfer can be either blocking (the function only returns when the transfer is complete)
 * or non-blocking (the function only initiates the transfer and then returns, leaving the 
 * transfer to complete asynchronously). For non-blocking transfers 
 * FLEXIO_SPI_DRV_SlaveGetStatus() must be called to check the status of the transfer.
 *
 * @param instance  FLEXIO peripheral instance number
 * @param transfer  SPI transfer structure for the current transfer
 * @return    Error or success status returned by API
 */
static inline flexio_spi_status_t FLEXIO_SPI_DRV_SlaveTransfer(uint32_t instance,
                                                 const flexio_spi_transfer_t *transfer)
{
    return FLEXIO_SPI_DRV_MasterTransfer(instance, transfer);
}


/*!
 * @brief Aborts a non-blocking SPI slave transaction
 *
 * This function aborts a non-blocking SPI transfer.
 *
 * @param instance  FLEXIO peripheral instance number
 * @return    Error or success status returned by API
 */
static inline flexio_spi_status_t FLEXIO_SPI_DRV_SlaveTransferAbort(uint32_t instance)
{
    return FLEXIO_SPI_DRV_MasterTransferAbort(instance);
}


/*!
 * @brief Get the status of the current non-blocking SPI slave transaction
 *
 * This function returns the current status of a non-blocking SPI slave transaction.
 * A return code of FLEXIO_SPI_STATUS_BUSY means the transfer is still in progress.
 * Otherwise the function returns a status reflecting the outcome of the last transfer.
 * When the driver is initialized in polling mode this function also advances the transfer
 * by checking and handling the transmit and receive events, so it must be called
 * frequently to avoid overflows or underflows.
 *
 * @param instance        FLEXIO peripheral instance number
 * @param bytesRemaining  the remaining number of bytes to be transferred
 * @return    Error or success status returned by API
 */
static inline flexio_spi_status_t FLEXIO_SPI_DRV_SlaveGetStatus(uint32_t instance, uint32_t *bytesRemaining)
{
    return FLEXIO_SPI_DRV_MasterGetStatus(instance, bytesRemaining);
}


/*! @}*/
#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __FSL_FLEXIO_SPI_DRIVER_H__ */
/*******************************************************************************
 * EOF
 ******************************************************************************/
