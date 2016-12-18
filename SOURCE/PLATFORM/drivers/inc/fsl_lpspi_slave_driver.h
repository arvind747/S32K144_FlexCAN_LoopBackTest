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
#if !defined(__FSL_LPSPI_SLAVE_DRIVER_H__)
#define __FSL_LPSPI_SLAVE_DRIVER_H__

#include "fsl_lpspi_hal.h"



/*!
 * @addtogroup lpspi_driver LPSPI Driver
 * @ingroup lpspi
 * @{
 */

/*! @brief Table of base pointers for SPI instances. */
extern LPSPI_Type * const g_lpspiBase[LPSPI_INSTANCE_COUNT];

/*! @brief Table to save LPSPI IRQ enumeration numbers defined in the CMSIS header file. */
extern const IRQn_Type g_lpspiIrqId[LPSPI_INSTANCE_COUNT];

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 *  @brief User configuration structure for the SPI slave driver.
 *  @internal gui name="Slave configuration" id="spiSlaveCfg"
 */
typedef struct LpspiSlaveConfig {
    lpspi_signal_polarity_t pcsPolarity; /*!< PCS polarity */
    uint32_t bitcount;                   /*!< Number of bits/frame, minimum is 8-bits */
    lpspi_clock_phase_t clkPhase;        /*!< Selects which phase of clock to capture data */
    lpspi_which_pcs_t whichPcs;
    lpspi_sck_polarity_t clkPolarity;    /*!< Selects clock polarity */
    bool lsbFirst;                       /*!< Option to transmit LSB first */
    uint32_t rxWatermark;                /*!< Rx watermark setting, must be less than fifo size */
    uint32_t txWatermark;                /*!< Tx watermark setting, must be less than fifo size */
} lpspi_slave_config_t;


/*!
 * @brief Runtime state of the LPSPI slave driver.
 *
 * This structure holds data that is used by the LPSPI slave peripheral driver to
 * communicate between the transfer function and the interrupt handler. The user
 * needs to pass in the memory for this structure and the driver fills out
 * the members.
 */
typedef struct LpspiSlaveState {
    uint32_t bitsPerFrame;               /*!< Number of bits per frame: 8- to 4096-bits; needed for
                                              TCR programming */
    uint32_t bytesPerFrame;              /*!< Number of bytes per frame: 1- to 512-bytes */

    lpspi_clock_phase_t clkPhase;        /*!< Selects which phase of clock to capture data; needed
                                              for TCR programming */
    lpspi_sck_polarity_t clkPolarity;    /*!< Selects clock polarity; needed for TCR programming */
    bool lsbFirst;                       /*!< Option to transmit LSB first; needed for
                                              TCR programming */
    lpspi_which_pcs_t whichPcs;
    uint32_t txWatermark;                /*!< Tx watermark setting, needed during transfer */
    uint32_t rxWatermark;                /*!< Rx watermark setting, needed during transfer */
    volatile bool isTransferInProgress;  /*!< True if there is an active transfer */
    const uint8_t * txBuff;              /*!< The buffer from which transmitted bytes are taken */
    uint8_t * rxBuff;                    /*!< The buffer into which received bytes are placed */
    volatile size_t txCount;             /*!< Number of bytes remaining to send (if bitcount>32
                                              then this is the number of bytes in a frame) */
    volatile size_t rxCount;             /*!< Number of bytes remaining to receive (if bitcount>32
                                              then this is the number of bytes in a frame) */
    volatile uint32_t txFrameCnt;        /*!< Number of frames remaining to transmit (only used
                                              when bitcount>32) */
    volatile uint32_t rxFrameCnt;        /*!< Number of frames remaining to receive (only used
                                              when bitcount>32) */
    uint32_t bytesTransferred;           /*!< Total number of bytes transferred */
    volatile bool isTransferBlocking;    /*!< True if transfer is a blocking transaction */
    /*semaphore_t irqSync;   */              /*!< Used to wait for ISR to complete its business */
} lpspi_slave_state_t;


/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Initialization and shutdown
 * @{
 */

/*!
 * @brief Initializes a LPSPI instance for a slave mode operation, using interrupt mechanism.
 *
 * This function un-gates the clock to the LPSPI module, initializes the LPSPI for
 * slave mode. After it is  initialized, the LPSPI module is configured in slave mode and the
 * user can start transmitting and receiving data by calling send, receive, and transfer functions.
 * This function indicates LPSPI slave uses an interrupt mechanism.
 *
 * @param instance The instance number of the LPSPI peripheral.
 * @param lpspiState The pointer to the LPSPI slave driver state structure.
 * @param slaveConfig The configuration structure lpspi_slave_user_config_t which
 *      configures the data bus format.
 *
 * @return An error code or LPSPI_STATUS_SUCCESS.
 */

lpspi_status_t LPSPI_DRV_SlaveInit(uint32_t instance,
                               lpspi_slave_state_t * lpspiState,
                               const lpspi_slave_config_t * slaveConfig);

/*!
 * @brief Shuts down an LPSPI instance interrupt mechanism.
 *
 * Disables the LPSPI module, gates its clock, and changes the LPSPI slave driver state to NonInit for the
 * LPSPI slave module which is initialized with interrupt mechanism. After de-initialization, the
 * user can re-initialize the LPSPI slave module with other mechanisms.
 *
 * @param instance The instance number of the LPSPI peripheral.
 * @return An error code or LPSPI_STATUS_SUCCESS.
 */
lpspi_status_t LPSPI_DRV_SlaveDeinit(uint32_t instance);

/*! @} */

/*!
 * @name Blocking transfers
 * @{
 */

/*!
 * @brief Transfers data on LPSPI bus using interrupt and a blocking call.
 *
 * This function checks the driver status and mechanism, and transmits/receives data through the LPSPI
 * bus. If the sendBuffer is NULL, the transmit process is ignored. If the receiveBuffer is NULL, the
 * receive process is ignored. If both the receiveBuffer and the sendBuffer are available, the transmit and the
 * receive progress is processed. If only the receiveBuffer is available, the receive is
 * processed. Otherwise, the transmit is processed. This function only returns when the
 * processes are completed. This function uses an interrupt mechanism.
 *
 * @param instance The instance number of LPSPI peripheral
 * @param sendBuffer The pointer to data that user wants to transmit.
 * @param receiveBuffer The pointer to data that user wants to store received data.
 * @param transferByteCount The number of bytes to send and receive.
 * @param timeout The maximum number of milliseconds that function waits before
 *              timed out reached.
 *
 * @return  LPSPI_STATUS_SUCCESS if driver starts to send/receive data successfully.
 *          LPSPI_STATUS_ERROR if driver is error and needs to clean error.
 *          LPSPI_STATUS_BUSY if driver is receiving/transmitting data and not available.
 *          LPSPI_STATUS_TIMEOUT if time out reached while transferring is in progress.
 */
lpspi_status_t LPSPI_DRV_SlaveTransferBlocking(uint32_t instance,
                                           const uint8_t *sendBuffer,
                                           uint8_t *receiveBuffer,
                                           uint32_t transferByteCount,
                                           uint32_t timeout);

/*@}*/

/*!
 * @name Non-blocking transfers
 * @{
 */

/*!
 * @brief Starts the transfer data on LPSPI bus using an interrupt and a non-blocking call.
 *
 * This function checks the driver status and sets buffer pointers to receive and transmit
 * SPI data. If the sendBuffer is NULL, the transmit process is ignored. If the receiveBuffer
 * is NULL, the receive process is ignored. If both the receiveBuffer and the sendBuffer are available,
 * the transfer is done when the LPSPI_TRANSFER_COMPLETE are set. If only the receiveBuffer is
 * available, the transfer is done when the kDspiRxDone flag is set. Otherwise, the transfer is done
 * when the kDspiTxDone was set. This function uses an interrupt mechanism.
 *
 * @param instance The instance number of LPSPI peripheral
 * @param sendBuffer The pointer to data that user wants to transmit.
 * @param receiveBuffer The pointer to data that user wants to store received data.
 * @param transferByteCount The number of bytes to send and receive.
 *
 * @return  kStatus_SPI_Success if driver starts to send/receive data successfully.
 *          kStatus_SPI_Error if driver is error and needs to clean error.
 *          kStatus_SPI_Busy if driver is receiving/transmitting data and not
 *                  available.
 */
lpspi_status_t LPSPI_DRV_SlaveTransfer(uint32_t instance,
                                   const uint8_t *sendBuffer,
                                   uint8_t *receiveBuffer,
                                   uint32_t transferByteCount);

/*!
 * @brief Aborts the transfer that started by a non-blocking call transfer function.
 *
 * This function stops the transfer which was started by the calling the SPI_DRV_SlaveTransfer() function.
 *
 * @param instance The instance number of SPI peripheral
 *
 * @return  LPSPI_STATUS_SUCCESS if everything is OK.
 *          kStatus_LPSPI_InvalidMechanism if the current transaction does not use
 *                      interrupt mechanism.
 */
lpspi_status_t LPSPI_DRV_SlaveAbortTransfer(uint32_t instance);

/*!
 * @brief Returns whether the previous transfer is finished.
 *
 * When performing an a-sync transfer, the user can call this function to ascertain
 * the state of the current transfer: in progress (or busy) or complete (success).
 * In addition, if the transfer is still in progress, the user can get the number
 * of words that have been transferred up to now.
 *
 * @param instance The instance number of the LPSPI peripheral.
 * @param framesTransferred Pointer to value that is filled in with the number of
 *  frames that have been sent in the active transfer. A frame is defined as the
 *  number of bits per frame.
 *
 * @return LPSPI_STATUS_SUCCESS The transfer has completed successfully, or
 *         LPSPI_STATUS_BUSY The transfer is still in progress. framesTransferred
 *         is filled with the number of words that have been transferred so far.
 */
lpspi_status_t LPSPI_DRV_SlaveGetTransferStatus(uint32_t instance,
                                            uint32_t * bytesTransferred);

/*!
 * @brief Interrupt handler for LPSPI slave mode.
 * This handler uses the buffers stored in the lpspi_master_state_t structs to transfer data.
 *
 * @param instance The instance number of the LPSPI peripheral.
 */
void LPSPI_DRV_SlaveIRQHandler(uint32_t instance);

/* @} */

#if defined(__cplusplus)
}
#endif


/*! @} */


#endif /* __FSL_LPSPI_SLAVE_DRIVER_H__ */


