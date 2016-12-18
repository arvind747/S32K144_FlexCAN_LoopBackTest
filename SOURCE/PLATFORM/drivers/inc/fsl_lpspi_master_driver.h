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
#if !defined(__FSL_LPSPI_MASTER_DRIVER_H__)
#define __FSL_LPSPI_MASTER_DRIVER_H__

#include "fsl_lpspi_hal.h"



/*!
 * @addtogroup lpspi_driver LPSPI Driver
 * @ingroup lpspi
 * @brief Low Power Serial Peripheral Interface Peripheral Driver
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
 * @brief Data structure containing information about a device on the SPI bus.
 *
 * The user must populate these members to set up the LPSPI master and
 * properly communicate with the SPI device.
 */
typedef struct LpspiConfig {
    uint32_t bitsPerSec;                 /*!< Baud rate in bits per second*/
    lpspi_which_pcs_t whichPcs;          /*!< Selects which PCS to use */
    lpspi_signal_polarity_t pcsPolarity; /*!< PCS polarity */
    bool isPcsContinuous;                /*!< Keeps PCS asserted until transfer complete */
    uint32_t bitcount;                   /*!< Number of bits/frame, minimum is 8-bits */
    uint32_t lpspiSrcClk;                /*!< Module source clock */
    lpspi_clock_phase_t clkPhase;        /*!< Selects which phase of clock to capture data */
    lpspi_sck_polarity_t clkPolarity;    /*!< Selects clock polarity */
    bool lsbFirst;                       /*!< Option to transmit LSB first */
    uint32_t rxWatermark;                /*!< Rx watermark setting, must be less than fifo size */
    uint32_t txWatermark;                /*!< Tx watermark setting, must be less than fifo size */
} lpspi_master_config_t;

/*!
 * @brief Runtime state structure for the LPSPI master driver.
 *
 * This structure holds data that is used by the LPSPI master peripheral driver to
 * communicate between the transfer function and the interrupt handler. The
 * interrupt handler also uses this information to keep track of its progress.
 * The user must pass  the memory for this run-time state structure. The
 * LPSPI master driver populates the members.
 */
typedef struct LpspiMasterState {
    uint32_t bitsPerFrame;               /*!< Number of bits per frame: 8- to 4096-bits; needed for
                                              TCR programming */
    uint32_t bytesPerFrame;              /*!< Number of bytes per frame: 1- to 512-bytes */
    lpspi_which_pcs_t whichPcs;          /*!< Peripheral Chip Select (PCS); needed for TCR
                                              programming */
    bool isPcsContinuous;                /*!< Option to keep chip select asserted until transfer
                                              complete; needed for TCR programming */
    uint32_t tcrPrescaler;               /*!< Stores the baud rate prescaler divider TCR bit
                                              setting; needed for TCR programming */
    lpspi_clock_phase_t clkPhase;        /*!< Selects which phase of clock to capture data; needed
                                              for TCR programming */
    lpspi_sck_polarity_t clkPolarity;    /*!< Selects clock polarity; needed for TCR programming */
    bool lsbFirst;                       /*!< Option to transmit LSB first; needed for
                                              TCR programming */
    uint32_t rxWatermark;                /*!< Rx watermark setting, needed during transfer */
    uint32_t lpspiSrcClk;                /*!< Module source clock */
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
    /*semaphore_t irqSync;*/                 /*!< Used to wait for ISR to complete its business */
} lpspi_master_state_t;

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
 * @brief Initializes a LPSPI instance for interrupt driven master mode operation.
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
   @code
    lpspi_master_state_t lpspiMasterState;  <- the user  allocates memory for this structure
    lpspi_master_config_t spiConfig;  Can declare more configs for use in transfer functions
    spiConfig.bitsPerSec = 500000;
    spiConfig.whichPcs = LPSPI_PCS0;
    spiConfig.pcsPolarity = LPSPI_ACTIVE_LOW;
    spiConfig.isPcsContinuous = false;
    spiConfig.bitCount = 16;
    spiConfig.clkPhase = LPSPI_CLOCK_PHASE_1ST_EDGE;
    spiConfig.clkPolarity = LPSPI_ACTIVE_HIGH;
    spiConfig.lsbFirst= false;
    spiConfig.rxWatermark = 2;
    spiConfig.txWatermark = 2;
    LPSPI_DRV_MasterInit(masterInstance, &lpspiMasterState, &spiConfig);
   @endcode
 *
 * @param instance The instance number of the LPSPI peripheral.
 * @param lpspiState The pointer to the LPSPI master driver state structure. The user
 *  passes the memory for this run-time state structure. The LPSPI master driver
 *  populates the members. This run-time state structure keeps track of the
 *  transfer in progress.
 * @param spiConfig The data structure containing information about a device on the SPI bus
 * @return An error code or LPSPI_STATUS_SUCCESS.
 */
lpspi_status_t LPSPI_DRV_MasterInit(uint32_t instance, lpspi_master_state_t * lpspiState,
                                    const lpspi_master_config_t * spiConfig);

/*!
 * @brief Shuts down a LPSPI instance.
 *
 * This function resets the LPSPI peripheral, gates its clock, and disables the interrupt to
 * the core.  It first checks to see if a transfer is in progress and if so returns an error
 * status.
 *
 * @param instance The instance number of the LPSPI peripheral.
 * @return An error code or kStatusLPSPI_Success.
 */
lpspi_status_t LPSPI_DRV_MasterDeinit(uint32_t instance);

/*!
 * @brief Configures the LPSPI master mode bus timing delay options.
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
 * @param instance The instance number of the LPSPI peripheral.
 * @param whichDelay The desired delay to configure, must be of type lpspi_delay_type_t
 * @param delayInNanoSec The desired delay value in nano-seconds.
 * @param calculatedDelay The calculated delay that best matches the desired
 *        delay (in nano-seconds).
 * @return Either LPSPI_STATUS_SUCCESS, LPSPI_STATUS_OUT_OF_RANGE if the desired delay exceeds
 *         the capability of the device, or LPSPI_STATUS_ERROR if an error is detected.
 */
lpspi_status_t LPSPI_DRV_MasterSetDelay(uint32_t instance, lpspi_delay_type_t whichDelay,
                                        uint32_t delayInNanoSec, uint32_t * calculatedDelay);


/*@}*/

/*!
 * @name Bus configuration
 * @{
 */

/*!
 * @brief Configures the LPSPI port physical parameters to access a device on the bus when the LSPI
 *        instance is configured for interrupt operation.
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
   @code
    lpspi_master_config_t spiConfig1;   You can also declare spiConfig2, spiConfig3, etc
    spiConfig1.bitsPerSec = 500000;
    spiConfig1.whichPcs = LPSPI_PCS0;
    spiConfig1.pcsPolarity = LPSPI_ACTIVE_LOW;
    spiConfig1.isPcsContinuous = false;
    spiConfig1.bitCount = 16;
    spiConfig1.clkPhase = LPSPI_CLOCK_PHASE_1ST_EDGE;
    spiConfig1.clkPolarity = LPSPI_ACTIVE_HIGH;
    spiConfig1.lsbFirst= false;
    spiConfig.rxWatermark = 2;
    spiConfig.txWatermark = 2;
   @endcode
 *
 * @param instance The instance number of the LPSPI peripheral.
 * @param spiConfig Pointer to the spiConfig structure. This structure contains the settings
 *  for the SPI bus configuration.  The SPI device parameters are the desired baud rate (in
 *  bits-per-sec), bits-per-frame, chip select attributes, clock attributes, and data shift
 *  direction.
 * @param calculatedBaudRate The calculated baud rate passed back to the user to determine
 *  if the calculated baud rate is close enough to meet the needs. The baud rate never exceeds
 *  the desired baud rate.
 * @return An error code or LPSPI_STATUS_SUCCESS.
 */
lpspi_status_t LPSPI_DRV_MasterConfigureBus(uint32_t instance,
                                            const lpspi_master_config_t * spiConfig,
                                            uint32_t * calculatedBaudRate);

/*@}*/

/*!
 * @name Blocking transfers
 * @{
 */

/*!
 * @brief Performs an interrupt driven blocking SPI master mode transfer.
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
 * @param instance The instance number of the LPSPI peripheral.
 * @param spiConfig Pointer to the SPI configuration structure. This structure contains the settings
 *  for the SPI bus configuration in this transfer. You may pass NULL for this
 *  parameter, in which case the current bus configuration is used unmodified. The device can be
 *  configured separately by calling the LPSPI_DRV_MasterConfigureBus function.
 * @param sendBuffer The pointer to the data buffer of the data to send. You may pass NULL for this
 *  parameter and  bytes with a value of 0 (zero) is sent.
 * @param receiveBuffer Pointer to the buffer where the received bytes are stored. If you pass NULL
 *  for this parameter, the received bytes are ignored.
 * @param transferByteCount The number of bytes to send and receive.
 * @param timeout A timeout for the transfer in milliseconds. If the transfer takes longer than
 *  this amount of time, the transfer is aborted and a LPSPI_STATUS_TIMEOUT error
 *  returned.
 * @return LPSPI_STATUS_SUCCESS The transfer was successful, or
 *         LPSPI_STATUS_BUSY Cannot perform transfer because a transfer is already in progress, or
 *         LPSPI_STATUS_TIMEOUT The transfer timed out and was aborted.
 */
lpspi_status_t LPSPI_DRV_MasterTransferBlocking(uint32_t instance,
                                                const lpspi_master_config_t * spiConfig,
                                                const uint8_t * sendBuffer,
                                                uint8_t * receiveBuffer,
                                                size_t transferByteCount,
                                                uint32_t timeout);

/*@}*/

/*!
 * @name Non-blocking transfers
 * @{
 */

/*!
 * @brief Performs an interrupt driven non-blocking SPI master mode transfer.
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
 * @param instance The instance number of the LPSPI peripheral.
 * @param spiConfig Pointer to the SPI configuration structure. This structure contains the settings
 *  for the SPI bus configuration in this transfer. You may pass NULL for this
 *  parameter, in which case the current bus configuration is used unmodified. The device can be
 *  configured separately by calling the LPSPI_DRV_MasterConfigureBus function.
 * @param sendBuffer The pointer to the data buffer of the data to send. You may pass NULL for this
 *  parameter and  bytes with a value of 0 (zero) is sent.
 * @param receiveBuffer Pointer to the buffer where the received bytes are stored. If you pass NULL
 *  for this parameter, the received bytes are ignored.
 * @param transferByteCount The number of bytes to send and receive.
 * @return LPSPI_STATUS_SUCCESS The transfer was successful, or
 *         LPSPI_STATUS_BUSY Cannot perform transfer because a transfer is already in progress, or
 *         LPSPI_STATUS_TIMEOUT The transfer timed out and was aborted.
 */
lpspi_status_t LPSPI_DRV_MasterTransfer(uint32_t instance,
                                        const lpspi_master_config_t * spiConfig,
                                        const uint8_t * sendBuffer,
                                        uint8_t * receiveBuffer,
                                        size_t transferByteCount);

/*!
 * @brief Returns whether the previous interrupt driven transfer is completed.
 *
 * When performing an a-sync (non-blocking) transfer, the user can call this function to ascertain
 * the state of the current transfer: in progress (or busy) or complete (success).
 * In addition, if the transfer is still in progress, the user can get the number of words that
 * have been transferred up to now.
 *
 * @param instance The instance number of the LPSPI peripheral.
 * @param bytesTransferred Pointer to a value that is filled in with the number of bytes that
 *      were sent in the active transfer.
 * @return LPSPI_STATUS_SUCCESS The transfer has completed successfully, or
 *         LPSPI_STATUS_BUSY The transfer is still in progress. framesTransferred is filled
 *         with the number of words that have been transferred so far.
 */
lpspi_status_t LPSPI_DRV_MasterGetTransferStatus(uint32_t instance, uint32_t * bytesTransferred);

/*!
 * @brief Terminates an interrupt driven asynchronous transfer early.
 *
 * During an a-sync (non-blocking) transfer, the user has the option to terminate the transfer early
 * if the transfer is still in progress.
 *
 * @param instance The instance number of the LPSPI peripheral.
 * @return LPSPI_STATUS_SUCCESS The transfer was successful, or
 *         LPSPI_STATUS_NO_TRANSFER_IN_PROGRESS No transfer is currently in progress.
 */
lpspi_status_t LPSPI_DRV_MasterAbortTransfer(uint32_t instance);

/*!
 * @brief Interrupt handler for LPSPI master mode.
 * This handler uses the buffers stored in the lpspi_master_state_t structs to transfer data.
 *
 * @param instance The instance number of the LPSPI peripheral.
 */
void LPSPI_DRV_MasterIRQHandler(uint32_t instance);

/* @}*/

#if defined(__cplusplus)
}
#endif

/*! @}*/


#endif /* __FSL_LPSPI_MASTER_DRIVER_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/

