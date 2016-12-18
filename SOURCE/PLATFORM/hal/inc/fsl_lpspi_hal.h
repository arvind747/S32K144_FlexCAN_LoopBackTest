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
#if !defined(__FSL_LPSPI_HAL_H__)
#define __FSL_LPSPI_HAL_H__

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "fsl_device_registers.h"



/*!
 * @addtogroup lpspi_hal LPSPI HAL
 * @ingroup lpspi
 * @brief Low Power Serial Peripheral Interface Hardware Abstraction Layer
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief Error codes for the LPSPI driver. */
typedef enum _lpspi_status
{
    LPSPI_STATUS_SUCCESS = 0,
    LPSPI_STATUS_SLAVE_TX_UNDERRUN,       /*!< LPSPI Slave Tx Under run error */
    LPSPI_STATUS_SLAVE_RX_OVERRUN,        /*!< LPSPI Slave Rx Overrun error */
    LPSPI_STATUS_TIMEOUT,               /*!< LPSPI transfer timed out */
    LPSPI_STATUS_BUSY,                  /*!< LPSPI instance is already busy performing a
                                              transfer */
    LPSPI_STATUS_NO_TRANSFER_IN_PROGRESS,  /*!< Attempt to abort a transfer when no
                                              transfer was in progress */
    LPSPI_STATUS_TRANSFER_IN_PROGRESS,    /*!< Transfer in progress */
    LPSPI_STATUS_INVALID_BIT_COUNT,       /*!< bits-per-frame value not valid */
    LPSPI_STATUS_INVALID_INSTANCE_NUMBER, /*!< LPSPI instance number does not match
                                              current count */
    LPSPI_STATUS_OUT_OF_RANGE,            /*!< LPSPI delay of divisor out-of-range error */
    LPSPI_STATUS_BITCOUNT_OUT_OF_RANGE,    /*!< LPSPI bitcount parameter out-of-range error */
    LPSPI_STATUS_WATERMARK_OUT_OF_RANGE,   /*!< LPSPI watermark parameter out-of-range error */
    LPSPI_STATUS_INVALID_PARAMETER,      /*!< LPSPI invalid parameter error */
    LPSPI_STATUS_NON_INIT,               /*!< LPSPI driver does not initialize, not ready */
    LPSPI_STATUS_INITIALIZED,           /*!< LPSPI driver has been initialized, could not
                                              initialize again */
    LPSPI_STATUS_DMA_CHANNEL_INVALID,     /*!< LPSPI driver could not request
                                              DMA channel(s) */
    LPSPI_STATUS_ERROR,                 /*!< LPSPI driver error */
} lpspi_status_t;

/*! @brief LPSPI status flags. */
typedef enum _lpspi_status_flag {
    LPSPI_TX_DATA_FLAG       = LPSPI_SR_TDF_SHIFT, /*!< TX data flag */
    LPSPI_RX_DATA_FLAG       = LPSPI_SR_RDF_SHIFT, /*!< RX data flag */
    LPSPI_WORD_COMPLETE     = LPSPI_SR_WCF_SHIFT, /*!< Word Complete flag */
    LPSPI_FRAME_COMPLETE    = LPSPI_SR_FCF_SHIFT, /*!< Frame Complete flag */
    LPSPI_TRANSFER_COMPLETE = LPSPI_SR_TCF_SHIFT, /*!< Transfer Complete flag */
    LPSPI_TRANSMIT_ERROR    = LPSPI_SR_TEF_SHIFT, /*!< Transmit Error flag (FIFO underrun) */
    LPSPI_RECEIVE_ERROR     = LPSPI_SR_REF_SHIFT, /*!< Receive Error flag (FIFO overrun) */
    LPSPI_DATA_MATCH        = LPSPI_SR_DMF_SHIFT, /*!< Data Match flag */
    LPSPI_MODULE_BUSY       = LPSPI_SR_MBF_SHIFT, /*!< Module Busy flag */
    LPSPI_ALL_STATUS        = 0x00003F00U         /*!< Used for clearing all w1c status flags */
} lpspi_status_flag_t;

/*! @brief LPSPI Clock Signal (SCK) Polarity configuration. */
typedef enum _lpspi_sck_polarity {
    LPSPI_SCK_ACTIVE_HIGH = 0U, /*!< Signal is Active High (idles low). */
    LPSPI_SCK_ACTIVE_LOW  = 1U  /*!< Signal is Active Low (idles high). */
} lpspi_sck_polarity_t;

/*! @brief LPSPI Signal (PCS and Host Request) Polarity configuration. */
typedef enum _lpspi_signal_polarity {
    LPSPI_ACTIVE_HIGH = 1U, /*!< Signal is Active High (idles low). */
    LPSPI_ACTIVE_LOW  = 0U  /*!< Signal is Active Low (idles high). */
} lpspi_signal_polarity_t;

/*! @brief LPSPI Host Request select configuration. */
typedef enum _lpspi_host_request_select {
    LPSPI_HOST_REQ_EXT_PIN           = 0U, /*!< Host Request is an ext pin. */
    LPSPI_HOST_REQ_INTERNAL_TRIGGER  = 1U  /*!< Host Request is an internal trigger. */
} lpspi_host_request_select_t;

/*! @brief LPSPI master or slave configuration. */
typedef enum _lpspi_master_slave_mode {
    LPSPI_MASTER = 1U,     /*!< LPSPI peripheral operates in master mode. */
    LPSPI_SLAVE  = 0U      /*!< LPSPI peripheral operates in slave mode. */
} lpspi_master_slave_mode_t;

/*! @brief LPSPI Peripheral Chip Select (PCS) configuration (which PCS to configure)*/
typedef enum _lpspi_which_pcs {
    LPSPI_PCS0 = 0U, /*!< PCS[0] */
    LPSPI_PCS1 = 1U, /*!< PCS[1] */
    LPSPI_PCS2 = 2U, /*!< PCS[2] */
    LPSPI_PCS3 = 3U  /*!< PCS[3] */
} lpspi_which_pcs_t;

/*! @brief LPSPI Match configuration options. */
typedef enum _lpspi_match_config {
    LPSPI_MATCH_DISABLED                   = 0x0U, /*!< LPSPI Match Disabled. */
    LPSPI_1ST_WRD_EQUALS_M0_OR_M1              = 0x2U, /*!< LPSPI Match Enabled. */
    LPSPI_ANY_WRD_EQUALS_M0_OR_M1              = 0x3U, /*!< LPSPI Match Enabled. */
    LPSPI_1ST_WRD_EQUALS_M0_AND_2ND_WRD_EQUALS_M1 = 0x4U, /*!< LPSPI Match Enabled. */
    LPSPI_ANY_WRD_EQUALS_M0_AND_NXT_WRD_EQUALS_M1 = 0x5U, /*!< LPSPI Match Enabled. */
    LPSPI_1ST_WRD_AND_M1_EQUALS_M0_AND_M1        = 0x6U, /*!< LPSPI Match Enabled. */
    LPSPI_ANY_WRD_AND_M1_EQUALS_M0_AND_M1        = 0x7U, /*!< LPSPI Match Enabled. */
} lpspi_match_config_t;

/*! @brief LPSPI pin (SDO and SDI) configuration. */
typedef enum _lpspi_pin_config {
    LPSPI_SDI_IN_SDO_OUT = 0U,     /*!< LPSPI SDI input, SDO output. */
    LPSPI_SDO_IN_SDO_OUT = 1U,     /*!< LPSPI SDO input, SDO output. */
    LPSPI_SDI_IN_SDI_OUT = 2U,     /*!< LPSPI SDI input, SDI output. */
    LPSPI_SDO_IN_SDI_OUT = 3U      /*!< LPSPI SDO input, SDI output. */
} lpspi_pin_config_t;

/*! @brief LPSPI clock phase configuration. */
typedef enum _lpspi_clock_phase{
    LPSPI_CLOCK_PHASE_1ST_EDGE = 0U, /*!< Data captured on SCK 1st edge, changed on 2nd. */
    LPSPI_CLOCK_PHASE_2ND_EDGE = 1U  /*!< Data changed on SCK 1st edge, captured on 2nd. */
} lpspi_clock_phase_t;

/*! @brief LPSPI data shift direction configuration. */
typedef enum _lpspi_data_direction{
    LPSPI_MSB = 0U, /*!< Data shifted out MSB first */
    LPSPI_LSB = 1U  /*!< Data shifted out LSB first */
} lpspi_data_direction_t;

/*! @brief LPSPI data output configuration. */
typedef enum _lpspi_data_out_config{
    LPSPI_DATA_OUT_RETAINED = 0U, /*!< Data out retains last value when chip select de-asserted */
    LPSPI_DATA_OUT_TRISTATE = 1U  /*!< Data out is tri-stated when chip select de-asserted */
} lpspi_data_out_config_t;

/*! @brief LPSPI transfer width configuration. */
typedef enum _lpspi_transfer_width{
    LPSPI_SINGLE_BIT_XFER = 0U, /*!< 1-bit shift at a time, data out on SDO, in on SDI (normal mode) */
    LPSPI_TWO_BIT_XFER = 1U,    /*!< 2-bits shift out on SDO/SDI and in on SDO/SDI */
    LPSPI_FOUR_BIT_XFER = 2U    /*!< 4-bits shift out on SDO/SDI/PCS[3:2] and in on SDO/SDI/PCS[3:2] */
} lpspi_transfer_width_t;

/*! @brief LPSPI Transmit Command Register configuration structure.
 *
 * This structure contains the Transmit Command Register (TCR) settings. Any writes
 * to the TCR will cause the entire TCR contents to be pushed to the TX FIFO.
 * Therefore any updates to the TCR should include updates to all of the register
 * bit fields to form a 32-bit write to the TCR.
 */
typedef struct LpspiTxCmdConfig {
    uint32_t frameSize;              /*!< Number of bits/frame, minimum is 8-bits. */
    lpspi_transfer_width_t width;    /*!< Transfer width, single, 2-bit, or 4-bit transfer. */
    bool txMask;                     /*!< Option to mask the transmit data (won't load to FIFO). */
    bool rxMask;                     /*!< Option to mask the receive data (won't store in FIFO). */
    bool contCmd;                    /*!< Master option to change cmd word within cont transfer. */
    bool contTransfer;               /*!< Master option for continuous transfer. */
    bool byteSwap;                   /*!< Option to invoke the byte swap option in the FIFOs. */
    bool lsbFirst;                   /*!< Option to transmit LSB first. */
    lpspi_which_pcs_t whichPcs;      /*!< Selects which PCS to use. */
    uint32_t preDiv;      /*!< Selects the baud rate prescaler divider TCR bit setting. */
    lpspi_clock_phase_t clkPhase;    /*!< Selects which phase of clock to capture data. */
    lpspi_sck_polarity_t clkPolarity; /*!< Selects clock polarity. */
} lpspi_tx_cmd_config_t;

/*! @brief LPSPI initialization configuration structure.
 *
 * This structure contains parameters for the user to fill in to configure the LPSPI.
 * The user passes this structure into the LPSPI init function to configure it to
 * their desired parameters.
 * Example user code for:
    - 60MHz assumed, check ref manual for exact value
    - baudrate 500KHz
    - master mode
    - PCS is active low
   @code
    lpspi_init_config_t lpspiCfg;
    lpspiCfg.lpspiSrcClk = 60000000;
    lpspiCfg.baudRate = 500000;
    lpspiCfg.lpspiMode = LPSPI_MASTER;
    lpspiCfg.pcsPol = LPSPI_ACTIVE_LOW;
   @endcode
 */
typedef struct LpspiInitConfig {
    uint32_t lpspiSrcClk;                /*!< LPSPI module clock */
    uint32_t baudRate;                   /*!< LPSPI baudrate */
    lpspi_master_slave_mode_t lpspiMode; /*!< LPSPI master/slave mode */
    lpspi_signal_polarity_t pcsPol;      /*!< LPSPI PCS polarity */
} lpspi_init_config_t;

/*! @brief LPSPI delay type selection*/
typedef enum _lpspi_delay_type {
	LPSPI_SCK_TO_PCS = LPSPI_CCR_SCKPCS_SHIFT,     /*!< SCK to PCS Delay */
	LPSPI_PCS_TO_SCK = LPSPI_CCR_PCSSCK_SHIFT,     /*!< PCS to SCK Delay */
	LPSPI_BETWEEN_TRANSFER = LPSPI_CCR_DBT_SHIFT  /*!< Delay between transfers */
} lpspi_delay_type_t ;

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Defines constant value arrays for the baud rate pre-scalar values.*/
static const uint32_t s_baudratePrescaler[] = { 1, 2, 4, 8, 16, 32, 64, 128 };

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Configuration
 * @{
 */

/*!
 * @brief Resets the LPSPI internal logic and registers to their default settings.
 *
 * This function first performs a software reset of the LPSPI module which resets the
 * internal LPSPI logic and most registers, then proceeds to manually reset all of the
 * LPSPI registers to their default setting to ensuring these registers at programmed to
 * their default value which includes disabling the module.
 *
 * @param Module base pointer of type LPSPI_Type.
 */
void LPSPI_HAL_Init(LPSPI_Type * base);

/*!
 * @brief Configures the LPSPI registers to a user defined configuration.
 *
 * Note, the LPSPI module must first be disabled prior to calling this function. It is recommended
 * to first call the LPSPI_HAL_Init function prior to calling this function.
 * This function configures the LPSPI based on the configuration passed in by the user
 * for normal SPI mode operation. Recommend single bit transfer: txCmd.width = LPSPI_SINGLE_BIT_XFER,
 * otherwise you will have to call function LPSPI_HAL_SetPinConfigMode to change the pin config.
 * This function sets the TX and RX FIFO watermarks to 0 such that the write blocking and read
 * blocking functions can be used following the init.
 *
 * @param Module base pointer of type LPSPI_Type.
 * @param config User configuration of type lpspi_init_config_t. Members of this struct are not
                 modifiable by the function.
 * @param actualBaudRate The actual, calculated baud rate passed back to the user
 * @param txCmdCfgSet Structure that contains the Transmit Command Register (TCR)
 *                    settings of type lpspi_tx_cmd_config_t. There is a member of this struct that
 *                    is modifiable by the function.
 * @return This function returns LPSPI_STATUS_ERROR if it is detected that an error occured
 *         during the LPSPI setup, otherwise, if success, it returns LPSPI_STATUS_SUCCESS.
 */
lpspi_status_t LPSPI_HAL_Config(LPSPI_Type * base, const lpspi_init_config_t * config,
                               lpspi_tx_cmd_config_t * txCmdCfgSet, uint32_t * actualBaudRate);

/*!
 * @brief Gets the Major, Minor and Feature ID of the LPSPI module.
 *
 * @ param Module base pointer of type LPSPI_Type.
 * @ param major The Major version number passed back to the user
 * @ param minor The Minor version number passed back to the user
 * @ param feature The Feature set number passed back to the user
 */
void LPSPI_HAL_GetVersionId(LPSPI_Type * base, uint32_t * major, uint32_t * minor,
                            uint32_t * feature);

/*!
 * @brief Enables the LPSPI module.
 *
 * @param Module base pointer of type LPSPI_Type.
 */
static inline void LPSPI_HAL_Enable(LPSPI_Type * base)
{
    BITBAND_ACCESS32(&(base->CR), LPSPI_CR_MEN_SHIFT) = (uint32_t)1U;
}

/*!
 * @brief Disables the LPSPI module.
 *
 * @param Module base pointer of type LPSPI_Type.
 * @return This function returns LPSPI_STATUS_BUSY if it is detected that the Module Busy Flag
 *         (MBF) is set, otherwise, if success, it returns LPSPI_STATUS_SUCCESS.
 */
lpspi_status_t LPSPI_HAL_Disable(LPSPI_Type * base);

/*!
 * @brief Configures the LPSPI for master or slave.
 *
 * Note that the LPSPI module must first be disabled before configuring this.
 *
 * @param Module base pointer of type LPSPI_Type.
 * @param mode Mode setting (master or slave) of type lpspi_master_slave_mode_t
 * @return This function returns the error condition LPSPI_STATUS_ERROR if the module is not
 *         disabled else it returns LPSPI_STATUS_SUCCESS.
 */
lpspi_status_t LPSPI_HAL_SetMasterSlaveMode(LPSPI_Type * base, lpspi_master_slave_mode_t mode);

/*!
 * @brief Returns whether the LPSPI module is in master mode.
 *
 * @param Module base pointer of type LPSPI_Type.
 * @return Returns true if LPSPI in master mode or false if in slave mode.
 */
static inline bool LPSPI_HAL_IsMaster(LPSPI_Type * base)
{
	return (bool)BITBAND_ACCESS32(&(base->CFGR1), LPSPI_CFGR1_MASTER_SHIFT);
}

/*!
 * @brief Gets the TX and RX FIFO sizes of the LPSPI module.
 *
 * @ param Module base pointer of type LPSPI_Type.
 * @ param txFifoSize The TX FIFO size passed back to the user
 * @ param rxFifoSize The RX FIFO size passed back to the user
 */
void LPSPI_HAL_GetFifoSizes(LPSPI_Type * base, uint32_t * txFifoSize, uint32_t * rxFifoSize);

/*!
 * @brief Flushes the LPSPI FIFOs.
 *
 * @param Module base pointer of type LPSPI_Type.
 * @param flushTxFifo Flushes (true) the Tx FIFO, else do not flush (false) the Tx FIFO
 * @param flushRxFifo Flushes (true) the Rx FIFO, else do not flush (false) the Rx FIFO
 */
void LPSPI_HAL_SetFlushFifoCmd(LPSPI_Type * base, bool flushTxFifo, bool flushRxFifo);

/*!
 * @brief Sets the TX and RX FIFO watermark values.
 *
 * This function allows the user to set the RX and TX FIFO watermarks. The function
 * does not compare the watermark setting to the FIFO size.  The FIFO watermark should not be
 * equal to or greater than the FIFO size.  It is up to the higher level driver to make this check.
 *
 * @param Module base pointer of type LPSPI_Type.
 * @param txWater The TX FIFO watermark value
 * @param rxWater The RX FIFO watermark value
 */
static inline void LPSPI_HAL_SetFifoWatermarks(LPSPI_Type * base, uint32_t txWater,
                                               uint32_t rxWater)
{
    uint32_t lpspi_tmp = base->FCR;
    lpspi_tmp &= ~(LPSPI_FCR_RXWATER_MASK | LPSPI_FCR_TXWATER_MASK);
    lpspi_tmp |= (rxWater << LPSPI_FCR_RXWATER_SHIFT)|(txWater);
	base->FCR = lpspi_tmp;

}

/*@}*/

/*!
 * @name Status flags and Interrupt configuration
 * @{
 */

/*!
 * @brief Gets the LPSPI status flag state.
 *
 * This function returns the state of one of the LPSPI status flags as requested by
 * the user.
 *
 * @param Module base pointer of type LPSPI_Type.
 * @param statusFlag The status flag, of type lpspi_status_flag_t
 * @return State of the status flag: asserted (true) or not-asserted (false)
 */
static inline bool LPSPI_HAL_GetStatusFlag(LPSPI_Type * base,
                                           lpspi_status_flag_t statusFlag)
{
    return (bool)(((base->SR) >> statusFlag) & 1U);
}

/*!
 * @brief Clears the LPSPI status flag.
 *
 * This function clears the state of one of the LPSPI status flags as requested by
 * the user. Note, the flag must be w1c capable, if not the function returns an error.
 * w1c capable flags are:
 *   LPSPI_WORD_COMPLETE
 *   LPSPI_FRAME_COMPLETE
 *   LPSPI_TRANSFER_COMPLETE
 *   LPSPI_TRANSMIT_ERROR
 *   LPSPI_RECEIVE_ERROR
 *   LPSPI_DATA_MATCH
 *
 * @param Module base pointer of type LPSPI_Type.
 * @param statusFlag The status flag, of type lpspi_status_flag_t
 * @return LPSPI_STATUS_SUCCESS or LPSPI_STATUS_INVALID_PARAMETER
 */
lpspi_status_t LPSPI_HAL_ClearStatusFlag(LPSPI_Type * base, lpspi_status_flag_t statusFlag);

/*!
 * @brief Configures the LPSPI interrupts.
 *
 * @param Module base pointer of type LPSPI_Type.
 * @param interruptSrc The interrupt source, of type lpspi_status_flag_t
 * @param enable Enable (true) or disable (false) the interrupt source
 */
static inline void LPSPI_HAL_SetIntMode(LPSPI_Type * base,
                                        lpspi_status_flag_t interruptSrc, bool enable)
{
    if (enable == true)
    {
        base->IER |= (uint32_t)1U << interruptSrc;
    }
    else
    {
        base->IER &= ~((uint32_t)1U << interruptSrc);
    }
}

/*!
 * @brief Returns if the LPSPI interrupt request is enabled or disabled.
 *
 * @param Module base pointer of type LPSPI_Type.
 * @param interruptSrc The interrupt source, of type lpspi_status_flag_t
 * @return Returns if the interrupt source is enabled (true) or disabled (false)
 */
static inline bool LPSPI_HAL_GetIntMode(LPSPI_Type * base,
                                        lpspi_status_flag_t interruptSrc)
{
    return (bool)(((base->IER) >> interruptSrc) & 1U);
}

/*@}*/

/*!
 * @name DMA configuration
 * @{
 */

/*!
 * @brief Sets the LPSPI Transmit Data DMA configuration (enable or disable).
 *
 * @param Module base pointer of type LPSPI_Type.
 * @param enable Enable (true) or disable (false) the TX DMA request
 */
static inline void LPSPI_HAL_SetTxDmaCmd(LPSPI_Type * base, bool enable)
{
    BITBAND_ACCESS32(&(base->DER), LPSPI_DER_TDDE_SHIFT) = (uint32_t)(enable == true);
}

/*!
 * @brief Sets the LPSPI Receive Data DMA configuration (enable or disable).
 *
 * @param Module base pointer of type LPSPI_Type.
 * @param enable Enable (true) or disable (false) the RX DMA request
 */
static inline void LPSPI_HAL_SetRxDmaCmd(LPSPI_Type * base, bool enable)
{
    BITBAND_ACCESS32(&(base->DER), LPSPI_DER_RDDE_SHIFT) = (uint32_t)(enable == true);
}

/*@}*/

/*!
 * @name SPI Bus Configuration
 * @{
 */

/*!
 * @brief Configures the LPSPI Host Request input.
 *
 * This function allows the user to configure the host request input pin as follows:
 *  Enable or disable the host request functionality.
 *  Select the polarity of the host request signal.
 *  Select the source of the host request (external signal or internal trigger).
 *
 * @param Module base pointer of type LPSPI_Type.
 * @param hostReqInput Host request input source of type lpspi_host_request_select_t
 * @param hostReqPol Host request polarity of type lpspi_signal_polarity_t
 * @param enable Enable (true) or disable (false) the Host request feature
 */
void LPSPI_HAL_SetHostRequestMode(LPSPI_Type * base,
                                  lpspi_host_request_select_t hostReqInput,
                                  lpspi_signal_polarity_t hostReqPol,
                                  bool enable);

/*!
 * @brief Configures the desired LPSPI PCS polarity.
 *
 * This function allows the user to configure the polarity of a particular PCS signal.
 * Note that the LPSPI module must first be disabled before configuring this.
 *
 * @param Module base pointer of type LPSPI_Type.
 * @param whichPcs Select which PCS to program, of type lpspi_which_pcs_t
 * @param pcsPolarity Set PCS as active high or low, of type lpspi_signal_polarity_t
 * @return This function returns the error condition LPSPI_STATUS_ERROR if the module is not
 *         disabled else it returns LPSPI_STATUS_SUCCESS.
 */
lpspi_status_t LPSPI_HAL_SetPcsPolarityMode(LPSPI_Type * base, lpspi_which_pcs_t whichPcs,
                                            lpspi_signal_polarity_t pcsPolarity);

/*!
 * @brief Configures the LPSPI data match configuration mode.
 *
 * When enabled and configured to the desired condition of type lpspi_match_config_t,
 * the LPSPI will assert the DMF status flag if the data match condition is met.
 * Note that the LPSPI module must first be disabled before configuring this.
 *
 * @param Module base pointer of type LPSPI_Type.
 * @param matchCondition Select condition for the data match (see lpspi_match_config_t)
 * @param rxDataMatchOnly When enabled, all received data that does not cause RMF to set
 *        is discarded.
 * @param match0 Setting for Match0 value
 * @param match1 Setting for Match1 value
 * @return This function returns the error condition LPSPI_STATUS_ERROR if the module is not
 *         disabled else it returns LPSPI_STATUS_SUCCESS.
 */
lpspi_status_t LPSPI_HAL_SetMatchConfigMode(LPSPI_Type * base,
                                            lpspi_match_config_t matchCondition,
                                            bool rxDataMatchOnly,
                                            uint32_t match0,
                                            uint32_t match1);

/*!
 * @brief Configures the LPSPI SDO/SDI pin configuration mode.
 *
 * This function configures the pin mode of the LPSPI.
 * For the SDI and SDO pins, the user can configure these pins as follows:
 *  SDI is used for input data and SDO for output data.
 *  SDO is used for input data and SDO for output data.
 *  SDI is used for input data and SDI for output data.
 *  SDO is used for input data and SDI for output data.
 *
 * The user has the option to configure the output data as:
 *  Output data retains last value when chip select is de-asserted (default setting).
 *  Output data is tristated when chip select is de-asserted.
 *
 * Finally, the user has the option to configure the PCS[3:2] pins as:
 *  Enabled for PCS operation (default setting).
 *  Disabled - this is need if the user wishes to configure the LPSPI mode for 4-bit transfers
 *             where these pins will be used as I/O data pins.
 *
 * Note that the LPSPI module must first be disabled before configuring this.
 *
 * @param Module base pointer of type LPSPI_Type.
 * @param pinCfg Select configuration for the SDO/SDI pins (see lpspi_pin_config_t)
 * @param dataOutConfig Select data output config after chip select de-assertion
 * @param pcs3and2Enable Enable or disable PCS[3:2]
 * @return This function returns the error condition LPSPI_STATUS_ERROR if the module is not
 *         disabled else it returns LPSPI_STATUS_SUCCESS.
 */
lpspi_status_t LPSPI_HAL_SetPinConfigMode(LPSPI_Type * base,
                                          lpspi_pin_config_t pinCfg,
                                          lpspi_data_out_config_t dataOutConfig,
                                          bool pcs3and2Enable);

/*!
 * @brief Sets the LPSPI baud rate in bits per second.
 *
 * This function takes in the desired bitsPerSec (baud rate) and calculates the nearest
 * possible baud rate without exceeding the desired baud rate, and returns the
 * calculated baud rate in bits-per-second. It requires that the caller also provide
 * the frequency of the module source clock (in Hertz). Also note that the baud rate
 * does not take into affect until the Transmit Control Register (TCR) is programmed
 * with the PRESCALE value. Hence, this function returns the PRESCALE tcrPrescaleValue
 * parameter for later programming in the TCR.  It is up to the higher level
 * peripheral driver to alert the user of an out of range baud rate input.
 *
 * Note that the LPSPI module must first be disabled before configuring this.
 * Note that the LPSPI module must be configure for master mode before configuring this.
 *
 * @param Module base pointer of type LPSPI_Type.
 * @param bitsPerSec The desired baud rate in bits per second
 * @param sourceClockInHz Module source input clock in Hertz
 * @param tcrPrescaleValue The TCR PRESCALE value, needed by user to program the TCR
 * @return  The actual calculated baud rate. This function may also return a "0" if the
 *          LPSPI is not configued for master mode or if the LPSPI module is not disabled.
 */
uint32_t LPSPI_HAL_SetBaudRate(LPSPI_Type * base, uint32_t bitsPerSec,
                               uint32_t sourceClockInHz, uint32_t * tcrPrescaleValue);

/*!
 * @brief Configures the baud rate divisor manually (only the LPSPI_CCR[SCKDIV]).
 *
 * This function allows the caller to manually set the baud rate divisor in the event
 * that this divider is known and the caller does not wish to call the
 * LPSPI_HAL_SetBaudRate function. Note that this only affects the LPSPI_CCR[SCKDIV]).
 * The Transmit Control Register (TCR) is programmed separately with the PRESCALE value.
 * The valid range is 0x00 to 0xFF, if the user inputs outside of this range, an error
 * is returned.
 *
 * Note that the LPSPI module must first be disabled before configuring this.
 * Note that the LPSPI module must be configure for master mode before configuring this.
 *
 * @param Module base pointer of type LPSPI_Type.
 * @param divisor Desired baud rate divisor setting (0x00 to 0xFF)
 * @return LPSPI_STATUS_SUCCESS or LPSPI_STATUS_OUT_OF_RANGE if divisor > 0xFF
 */
lpspi_status_t LPSPI_HAL_SetBaudRateDivisor(LPSPI_Type * base, uint32_t divisor);

/*!
 * @brief Manually configures a specific LPSPI delay parameter (module must be disabled to
 *        change the delay values).
 *
 * This function configures the:
 * SCK to PCS delay, or
 * PCS to SCK delay, or
 * Between transfer delay.
 *
 * These delay names are available in type lpspi_delay_type_t.
 *
 * The user passes which delay they want to configure along with the delay value.
 * This allows the user to directly set the delay values if they have
 * pre-calculated them or if they simply wish to manually increment the value.
 *
 * Note that the LPSPI module must first be disabled before configuring this.
 * Note that the LPSPI module must be configure for master mode before configuring this.
 *
 * @param base Module base pointer of type LPSPI_Type.
 * @param whichDelay The desired delay to configure, must be of type lpspi_delay_type_t
 * @param delay The 8-bit delay value 0x00 to 0xFF (255)
 * @return Either LPSPI_STATUS_SUCCESS, LPSPI_STATUS_OUT_OF_RANGE, or LPSPI_STATUS_ERROR if
 *         LPSPI is not disabled or if is not set for master mode.
 */
lpspi_status_t LPSPI_HAL_SetDelay(LPSPI_Type * base, lpspi_delay_type_t whichDelay, uint32_t delay);

/*!
 * @brief Calculates the delay based on the desired delay input in nanoseconds (module must be
 *        disabled to change the delay values).
 *
 * This function calculates the values for:
 * SCK to PCS delay, or
 * PCS to SCK delay, or
 * Between transfer delay.
 *
 * These delay names are available in type lpspi_delay_type_t.
 *
 * The user passes which delay they want to configure along with the desired delay value in
 * nano-seconds.  The function calculates the value needed for the desired delay parameter
 * and returns the actual calculated delay as an exact delay match may not be possible. In this
 * case, the closest match is calculated without going below the desired delay value input.
 * It is possible to input a very large delay value that exceeds the capability of the part, in
 * which case the maximum supported delay will be returned. It is up to the higher level
 * peripheral driver to alert the user of an out of range delay input.
 *
 * Note that the LPSPI module must first be disabled before configuring this.
 * Note that the LPSPI module must be configure for master mode before configuring this.
 *
 * @param base Module base pointer of type LPSPI_Type.
 * @param whichDelay The desired delay to configure, must be of type lpspi_delay_type_t
 * @param sourceClockInHz Module source input clock in Hertz
 * @param delayInNanoSec The desired delay value in nano-seconds.
 * @param actualDelay The actual calculated delay value in nano-seconds passed back to user.
 * @return Either LPSPI_STATUS_SUCCESS or LPSPI_STATUS_ERROR if LPSPI is not disabled or if it
 *         is not set for master mode.
 */
lpspi_status_t LPSPI_HAL_CalculateDelay(LPSPI_Type * base,
                                        lpspi_delay_type_t whichDelay,
                                        uint32_t sourceClockInHz,
                                        uint32_t delayInNanoSec,
                                        uint32_t * actualDelay);

/*@}*/

/*!
 * @name Data transfer
 * @{
 */

/*!
 * @brief Sets the Transmit Command Register (TCR) parameters.
 *
 * The Transmit Command Register (TCR) contains multiple parameters that affect
 * the transmission of data, such as clock phase and polarity, which PCS to use,
 * whether or not the PCS remains asserted at the completion of a frame, etc.
 * Any writes to this register results in an immediate push of the entire register
 * and its contents to the TX FIFO.  Hence, writes to this register should include
 * all of the desired parameters written to the register at once. Hence, the user
 * should fill in the members of the lpspi_tx_cmd_config_t data structure and pass
 * this to the function.
 *
 * @param Module base pointer of type LPSPI_Type.
 * @param txCmdCfgSet Structure that contains the Transmit Command Register (TCR)
 *                    settings of type lpspi_tx_cmd_config_t
 */
void LPSPI_HAL_SetTxCommandReg(LPSPI_Type * base, lpspi_tx_cmd_config_t * txCmdCfgSet);

/*!
 * @brief Writes data into the TX data buffer.
 *
 * This function writes data passed in by the user to the Transmit Data Register (TDR).
 * The user can pass up to 32-bits of data to load into the TDR. If the frame size exceeds 32-bits,
 * the user will have to manage sending the data one 32-bit word at a time.
 * Any writes to the TDR will result in an immediate push to the TX FIFO.
 * This function can be used for either master or slave mode.
 *
 * @param Module base pointer of type LPSPI_Type.
 * @param data The data word to be sent
 */
static inline void LPSPI_HAL_WriteData(LPSPI_Type * base, uint32_t data)
{
    base->TDR = data;
}

/*!
 * @brief Writes a data into the TX data buffer and waits till complete to return.
 *
 * This function writes the data to the Transmit Data Register (TDR) and waits for completion
 * before returning. If the frame size exceeds 32-bits, the user will have to manage sending
 * the data one 32-bit word at a time.
 * This function can be used for either master or slave mode.
 * Note that it is required that the TX FIFO watermark be set to 0.
 *
 * @param Module base pointer of type LPSPI_Type.
 * @param data The data word to be sent
 */
void LPSPI_HAL_WriteDataBlocking(LPSPI_Type * base, uint32_t data);

/*!
 * @brief Reads data from the data buffer.
 *
 * This function reads the data from the Receive Data Register (RDR).
 * This function can be used for either master or slave mode.
 *
 * @param Module base pointer of type LPSPI_Type.
 * @return The data read from the data buffer
 */
static inline uint32_t LPSPI_HAL_ReadData(LPSPI_Type * base)
{
    return (uint32_t)base->RDR;
}

/*!
 * @brief Reads data from the data buffer but first waits till data is ready.
 *
 * This function reads the data from the Receive Data Register (RDR).
 * However, before reading the data, it first waits till the read data ready status
 * indicates the data is ready to be read.
 * This function can be used for either master or slave mode.
 * Note that it is required that the RX FIFO watermark be set to 0.
 *
 * @param Module base pointer of type LPSPI_Type.
 * @return The data read from the data buffer
 */
uint32_t LPSPI_HAL_ReadDataBlocking(LPSPI_Type * base);

/*!
 * @brief Gets the LPSPI Transmit Data Register address for DMA operation.
 *
 * This function gets the LPSPI Transmit Data Register address as this value is needed
 * for DMA operation.
 * This function can be used for either master or slave mode.
 *
 * @param Module base pointer of type LPSPI_Type.
 * @return The LPSPI Transmit Data Register address
 */
static inline uint32_t LPSPI_HAL_GetTxDataRegAddr(LPSPI_Type * base)
{
	return (uint32_t)(base->TDR);
}

/*!
 * @brief Gets the LPSPI Receive Data Register address for DMA operation.
 *
 * This function gets the LPSPI Receive Data Register address as this value is needed
 * for DMA operation.
 * This function can be used for either master or slave mode.
 *
 * @param Module base pointer of type LPSPI_Type.
 * @return The LPSPI Receive Data Register address
 */
static inline uint32_t LPSPI_HAL_GetRxDataRegAddr(LPSPI_Type * base)
{
	return (uint32_t)(base->RDR);
}

/*!
 * @brief Reads TX COUNT form the FIFO Status Register.
 *
 * This function reads the TX COUNT field  from the FIFO Status Register (FSR).
 *
 * @param Module base pointer of type LPSPI_Type.
 * @return The data read from the FIFO Status Register
 */
static inline uint32_t LPSPI_HAL_ReadTxCount(LPSPI_Type * base)
{
    return (uint32_t)(((uint32_t)(base->FSR & LPSPI_FSR_TXCOUNT_MASK)) >> LPSPI_FSR_TXCOUNT_SHIFT);
}

/*!
 * @brief Reads RX COUNT form the FIFO Status Register.
 *
 * This function reads the RX COUNT field  from the FIFO Status Register (FSR).
 *
 * @param Module base pointer of type LPSPI_Type.
 * @return The data read from the FIFO Status Register
 */
static inline uint32_t LPSPI_HAL_ReadRxCount(LPSPI_Type * base)
{
    return (uint32_t)(((uint32_t)(base->FSR & LPSPI_FSR_RXCOUNT_MASK)) >> LPSPI_FSR_RXCOUNT_SHIFT);
}

/*!
 * @brief Clear CONTC bit form TCR Register.
 *
 * This function clears the CONTC bit from the Transmit Command Register (TCR).
 *
 * @param Module base pointer of type LPSPI_Type.
 */
static inline void LPSPI_HAL_ClearContCBit(LPSPI_Type * base)
{
	BITBAND_ACCESS32(&(base->TCR), LPSPI_TCR_CONTC_SHIFT) = (uint32_t)0;
}

/*@}*/

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __FSL_LPSPI_HAL_H__*/

/*******************************************************************************
 * EOF
 ******************************************************************************/

