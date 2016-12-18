/*
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
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
#ifndef __FSL_LPUART_HAL_H__
#define __FSL_LPUART_HAL_H__

#include "fsl_device_registers.h"
#include <stdbool.h>

/*!
 * @addtogroup lpuart_hal
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define LPUART_SHIFT (16U)
#define LPUART_BAUD_REG_ID (0U)
#define LPUART_STAT_REG_ID (1U)
#define LPUART_CTRL_REG_ID (2U)
#define LPUART_DATA_REG_ID (3U)
#define LPUART_MATCH_REG_ID (4U)
#define LPUART_MODIR_REG_ID (5U)
#define LPUART_FIFO_REG_ID (6U)
#define LPUART_WATER_REG_ID (7U)

/*! @brief Error codes for the LPUART driver.*/
typedef enum _lpuart_status
{
    LPUART_STAT_SUCCESS                     = 0x00U,
    LPUART_STAT_FAIL                        = 0x01U,
    LPUART_STAT_BAUD_RATE_CALCULATION_ERROR = 0x02U,
    LPUART_STAT_RX_STAND_BY_MODE_ERROR      = 0x03U,
    LPUART_STAT_CLEAR_STATUS_FLAG_ERROR     = 0x04U,
    LPUART_STAT_TX_NOT_DISABLED             = 0x05U,
    LPUART_STAT_RX_NOT_DISABLED             = 0x06U,
    LPUART_STAT_TX_BUSY                     = 0x07U,
    LPUART_STAT_RX_BUSY                     = 0x08U,
    LPUART_STAT_NO_TRANSMIT_IN_PROGRESS     = 0x09U,
    LPUART_STAT_NO_RECEIVE_IN_PROGRESS      = 0x0AU,
    LPUART_STAT_TIMEOUT                     = 0x0BU,
    LPUART_STAT_INITIALIZED                 = 0x0CU,
    LPUART_STAT_NO_DATA_TO_DEAL             = 0x0DU,
    LPUART_STAT_RX_OVERRUN                  = 0x0EU,
    LPUART_STAT_CLOCK_GATED_OFF             = 0x0FU
} lpuart_status_t;

/*! @brief LPUART number of stop bits*/
typedef enum _lpuart_stop_bit_count {
    LPUART_ONE_STOP_BIT = 0x0U, /*!< one stop bit @internal gui name="1" */
    LPUART_TWO_STOP_BIT = 0x1U, /*!< two stop bits @internal gui name="2" */
} lpuart_stop_bit_count_t;

/*! @brief LPUART parity mode*/
typedef enum _lpuart_parity_mode {
    LPUART_PARITY_DISABLED = 0x0U, /*!< parity disabled @internal gui name="Disabled" */
    LPUART_PARITY_EVEN     = 0x2U, /*!< parity enabled, type even, bit setting: PE|PT = 10 @internal gui name="Even" */
    LPUART_PARITY_ODD      = 0x3U, /*!< parity enabled, type odd,  bit setting: PE|PT = 11 @internal gui name="Odd" */
} lpuart_parity_mode_t;

/*! @brief LPUART number of bits in a character*/
typedef enum  _lpuart_bit_count_per_char {
    LPUART_8_BITS_PER_CHAR  = 0x0U, /*!< 8-bit data characters @internal gui name="8" */
    LPUART_9_BITS_PER_CHAR  = 0x1U, /*!< 9-bit data characters @internal gui name="9" */
    LPUART_10_BITS_PER_CHAR = 0x2U, /*!< 10-bit data characters @internal gui name="10" */
} lpuart_bit_count_per_char_t;

/*! @brief LPUART operation configuration constants*/
typedef enum _lpuart_operation_config {
    LPUART_OPERATES = 0x0U, /*!< LPUART continues to operate normally.*/
    LPUART_STOPS    = 0x1U, /*!< LPUART stops operation. */
} lpuart_operation_config_t;

/*! @brief LPUART wakeup from standby method constants*/
typedef enum _lpuart_wakeup_method {
    LPUART_IDLE_LINE_WAKE = 0x0U, /*!< Idle-line wakes the LPUART receiver from standby. */
    LPUART_ADDR_MARK_WAKE = 0x1U, /*!< Addr-mark wakes LPUART receiver from standby.*/
} lpuart_wakeup_method_t;

/*! @brief LPUART idle line detect selection types*/
typedef enum _lpuart_idle_line_select {
    LPUART_IDLE_LINE_AFTER_START_BIT = 0x0U, /*!< LPUART idle character bit count start after start bit */
    LPUART_IDLE_LINE_AFTER_STOP_BIT  = 0x1U, /*!< LPUART idle character bit count start after stop bit */
} lpuart_idle_line_select_t;

/*!
 * @brief LPUART break character length settings for transmit/detect.
 *
 * The actual maximum bit times may vary depending on the LPUART instance.
 */
typedef enum _lpuart_break_char_length {
    LPUART_BREAK_CHAR_10_BIT_MINIMUM = 0x0U, /*!< LPUART break char length 10 bit times (if M = 0, SBNS = 0)
                                      or 11 (if M = 1, SBNS = 0 or M = 0, SBNS = 1) or 12 (if M = 1,
                                      SBNS = 1 or M10 = 1, SNBS = 0) or 13 (if M10 = 1, SNBS = 1) */
    LPUART_BREAK_CHAR_13_BIT_MINIMUM = 0x1U, /*!< LPUART break char length 13 bit times (if M = 0, SBNS = 0
                                           or M10 = 0, SBNS = 1) or 14 (if M = 1, SBNS = 0 or M = 1,
                                           SBNS = 1) or 15 (if M10 = 1, SBNS = 1 or M10 = 1, SNBS = 0) */
} lpuart_break_char_length_t;

/*! @brief LPUART single-wire mode TX direction*/
typedef enum _lpuart_singlewire_txdir {
    LPUART_SINGLE_WIRE_TX_DIR_IN  = 0x0U, /*!< LPUART Single Wire mode TXDIR input*/
    LPUART_SINGLE_WIRE_TX_DIR_OUT = 0x1U, /*!< LPUART Single Wire mode TXDIR output*/
} lpuart_singlewire_txdir_t;

/*! @brief LPUART Configures the match addressing mode used.*/
typedef enum _lpuart_match_config {
    LPUART_ADDRESS_MATCH_WAKEUP     = 0x0U, /*!< Address Match Wakeup*/
    LPUART_IDLE_MATCH_WAKEUP        = 0x1U, /*!< Idle Match Wakeup*/
    LPUART_MATCH_ON_AND_MATCH_OFF   = 0x2U, /*!< Match On and Match Off*/
    LPUART_ENABLE_RWU_ON_DATA_MATCH = 0x3U, /*!< Enables RWU on Data Match and Match On/Off for transmitter CTS input*/
} lpuart_match_config_t;

/*! @brief LPUART infra-red transmitter pulse width options*/
typedef enum _lpuart_ir_tx_pulsewidth {
    LPUART_IR_THREE_SIXTEENTH_WIDTH   = 0x0U, /*!< 3/16 pulse*/
    LPUART_IR_ONE_SIXTEENTH_WIDTH     = 0x1U, /*!< 1/16 pulse*/
    LPUART_IR_ONE_THIRTYSECOND_WIDTH  = 0x2U, /*!< 1/32 pulse*/
    LPUART_IR_ONE_FOURTH_WIDTH        = 0x3U, /*!< 1/4 pulse*/
} lpuart_ir_tx_pulsewidth_t;

/*!
 * @brief LPUART Configures the number of idle characters that must be received
 * before the IDLE flag is set.
 */
typedef enum _lpuart_idle_char {
    LPUART_1_IDLE_CHAR   = 0x0U, /*!< 1 idle character*/
    LPUART_2_IDLE_CHAR   = 0x1U, /*!< 2 idle character*/
    LPUART_4_IDLE_CHAR   = 0x2U, /*!< 4 idle character*/
    LPUART_8_IDLE_CHAR   = 0x3U, /*!< 8 idle character*/
    LPUART_16_IDLE_CHAR  = 0x4U, /*!< 16 idle character*/
    LPUART_32_IDLE_CHAR  = 0x5U, /*!< 32 idle character*/
    LPUART_64_IDLE_CHAR  = 0x6U, /*!< 64 idle character*/
    LPUART_128_IDLE_CHAR = 0x7U, /*!< 128 idle character*/
} lpuart_idle_char_t;

/*! @brief LPUART Transmits the CTS Configuration. Configures the source of the CTS input.*/
typedef enum _lpuart_cts_source {
    LPUART_CTS_SOURCE_PIN                     = 0x0U,  /*!< CTS input is the LPUART_CTS pin.*/
    LPUART_CTS_SOURCE_INVERTED_RECEIVER_MATCH = 0x1U, /*!< CTS input is the inverted Receiver Match result.*/
} lpuart_cts_source_t;

/*!
 * @brief LPUART Transmits CTS Source.Configures if the CTS state is checked at
 * the start of each character or only when the transmitter is idle.
 */
typedef enum _lpuart_cts_config {
    LPUART_CTS_SAMPLED_ON_EACH_CHAR = 0x0U, /*!< CTS input is sampled at the start of each character.*/
    LPUART_CTS_SAMPLED_ON_IDLE      = 0x1U, /*!< CTS input is sampled when the transmitter is idle.*/
} lpuart_cts_config_t;

/*! @brief Structure for idle line configuration settings*/
typedef struct LpuartIdleLineConfig {
    unsigned idleLineType : 1; /*!< ILT, Idle bit count start: 0 - after start bit (default),
                                    1 - after stop bit */
    unsigned rxWakeIdleDetect : 1; /*!< RWUID, Receiver Wake Up Idle Detect. IDLE status bit
                                        operation during receive standbyControls whether idle
                                        character that wakes up receiver will also set
                                        IDLE status bit 0 - IDLE status bit doesn't
                                        get set (default), 1 - IDLE status bit gets set*/
} lpuart_idle_line_config_t;

/*!
 * @brief LPUART status flags.
 *
 * This provides constants for the LPUART status flags for use in the UART functions.
 */
typedef enum _lpuart_status_flag {
    LPUART_TX_DATA_REG_EMPTY          = LPUART_STAT_REG_ID << LPUART_SHIFT | LPUART_STAT_TDRE_SHIFT,    /*!< Tx data register empty flag, sets when Tx buffer is empty */
    LPUART_TX_COMPLETE                = LPUART_STAT_REG_ID << LPUART_SHIFT | LPUART_STAT_TC_SHIFT,      /*!< Transmission complete flag, sets when transmission activity complete */
    LPUART_RX_DATA_REG_FULL           = LPUART_STAT_REG_ID << LPUART_SHIFT | LPUART_STAT_RDRF_SHIFT,    /*!< Rx data register full flag, sets when the receive data buffer is full */
    LPUART_IDLE_LINE_DETECT           = LPUART_STAT_REG_ID << LPUART_SHIFT | LPUART_STAT_IDLE_SHIFT,    /*!< Idle line detect flag, sets when idle line detected */
    LPUART_RX_OVERRUN                 = LPUART_STAT_REG_ID << LPUART_SHIFT | LPUART_STAT_OR_SHIFT,      /*!< Rx Overrun, sets when new data is received before data is read from receive register */
    LPUART_NOISE_DETECT               = LPUART_STAT_REG_ID << LPUART_SHIFT | LPUART_STAT_NF_SHIFT,      /*!< Rx takes 3 samples of each received bit.  If any of these samples differ, noise flag sets */
    LPUART_FRAME_ERR                  = LPUART_STAT_REG_ID << LPUART_SHIFT | LPUART_STAT_FE_SHIFT,      /*!< Frame error flag, sets if logic 0 was detected where stop bit expected */
    LPUART_PARITY_ERR                 = LPUART_STAT_REG_ID << LPUART_SHIFT | LPUART_STAT_PF_SHIFT,      /*!< If parity enabled, sets upon parity error detection */
    LPUART_LIN_BREAK_DETECT           = LPUART_STAT_REG_ID << LPUART_SHIFT | LPUART_STAT_LBKDIF_SHIFT,  /*!< LIN break detect interrupt flag, sets when LIN break char detected and LIN circuit enabled */
    LPUART_RX_ACTIVE_EDGE_DETECT      = LPUART_STAT_REG_ID << LPUART_SHIFT | LPUART_STAT_RXEDGIF_SHIFT, /*!< Rx pin active edge interrupt flag, sets when active edge detected */
    LPUART_RX_ACTIVE                  = LPUART_STAT_REG_ID << LPUART_SHIFT | LPUART_STAT_RAF_SHIFT,     /*!< Receiver Active Flag (RAF), sets at beginning of valid start bit */
    LPUART_NOISE_IN_CURRENT_WORD      = LPUART_DATA_REG_ID << LPUART_SHIFT | LPUART_DATA_NOISY_SHIFT,   /*!< NOISY bit, sets if noise detected in current data word */
    LPUART_PARITY_ERR_IN_CURRENT_WORD = LPUART_DATA_REG_ID << LPUART_SHIFT | LPUART_DATA_PARITYE_SHIFT, /*!< PARITYE bit, sets if noise detected in current data word */
#if FSL_FEATURE_LPUART_HAS_ADDRESS_MATCHING
    LPUART_MATCH_ADDR_ONE             = LPUART_STAT_REG_ID << LPUART_SHIFT | LPUART_STAT_MA1F_SHIFT,    /*!< Address one match flag */
    LPUART_MATCH_ADDR_TWO             = LPUART_STAT_REG_ID << LPUART_SHIFT | LPUART_STAT_MA2F_SHIFT,    /*!< Address two match flag */
#endif
#if FSL_FEATURE_LPUART_FIFO_SIZE
    LPUART_FIFO_TX_OF                 = LPUART_FIFO_REG_ID << LPUART_SHIFT | LPUART_FIFO_TXOF_SHIFT,    /*!< Transmitter FIFO buffer overflow */
    LPUART_FIFO_RX_UF                 = LPUART_FIFO_REG_ID << LPUART_SHIFT | LPUART_FIFO_RXUF_SHIFT,    /*!< Receiver FIFO buffer underflow */
#endif
} lpuart_status_flag_t;

/*! @brief LPUART interrupt configuration structure, default settings are 0 (disabled)*/
typedef enum _lpuart_interrupt {
    LPUART_INT_LIN_BREAK_DETECT  = LPUART_BAUD_REG_ID << LPUART_SHIFT | LPUART_BAUD_LBKDIE_SHIFT,  /*!< LIN break detect. */
    LPUART_INT_RX_ACTIVE_EDGE    = LPUART_BAUD_REG_ID << LPUART_SHIFT | LPUART_BAUD_RXEDGIE_SHIFT, /*!< RX Active Edge. */
    LPUART_INT_TX_DATA_REG_EMPTY = LPUART_CTRL_REG_ID << LPUART_SHIFT | LPUART_CTRL_TIE_SHIFT,     /*!< Transmit data register empty. */
    LPUART_INT_TX_COMPLETE       = LPUART_CTRL_REG_ID << LPUART_SHIFT | LPUART_CTRL_TCIE_SHIFT,    /*!< Transmission complete. */
    LPUART_INT_RX_DATA_REG_FULL  = LPUART_CTRL_REG_ID << LPUART_SHIFT | LPUART_CTRL_RIE_SHIFT,     /*!< Receiver data register full. */
    LPUART_INT_IDLE_LINE         = LPUART_CTRL_REG_ID << LPUART_SHIFT | LPUART_CTRL_ILIE_SHIFT,    /*!< Idle line. */
    LPUART_INT_RX_OVERRUN        = LPUART_CTRL_REG_ID << LPUART_SHIFT | LPUART_CTRL_ORIE_SHIFT,    /*!< Receiver Overrun. */
    LPUART_INT_NOISE_ERR_FLAG    = LPUART_CTRL_REG_ID << LPUART_SHIFT | LPUART_CTRL_NEIE_SHIFT,    /*!< Noise error flag. */
    LPUART_INT_FRAME_ERR_FLAG    = LPUART_CTRL_REG_ID << LPUART_SHIFT | LPUART_CTRL_FEIE_SHIFT,    /*!< Framing error flag. */
    LPUART_INT_PARITY_ERR_FLAG   = LPUART_CTRL_REG_ID << LPUART_SHIFT | LPUART_CTRL_PEIE_SHIFT,    /*!< Parity error flag. */
#if FSL_FEATURE_LPUART_HAS_ADDRESS_MATCHING
    LPUART_INT_MATCH_ADDR_ONE    = LPUART_CTRL_REG_ID << LPUART_SHIFT | LPUART_CTRL_MA1IE_SHIFT,   /*!< Match address one flag. */
    LPUART_INT_MATCH_ADDR_TWO    = LPUART_CTRL_REG_ID << LPUART_SHIFT | LPUART_CTRL_MA2IE_SHIFT,   /*!< Match address two flag. */
#endif
#if FSL_FEATURE_LPUART_FIFO_SIZE
    LPUART_INT_FIFO_TXOF         = LPUART_FIFO_REG_ID << LPUART_SHIFT | LPUART_FIFO_TXOFE_SHIFT,    /*!< Transmitter FIFO buffer interrupt */
    LPUART_INT_FIFO_RXUF         = LPUART_FIFO_REG_ID << LPUART_SHIFT | LPUART_FIFO_RXUFE_SHIFT,    /*!< Receiver FIFO buffer interrupt */
#endif
} lpuart_interrupt_t;


/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name LPUART Common Configurations
 * @{
 */

/*!
 * @brief Initializes the LPUART controller.
 *
 * This function Initializes the LPUART controller to known state.
 *
 *
 * @param base LPUART base pointer.
 */
void LPUART_HAL_Init(LPUART_Type * base);

/*!
 * @brief Enable/Disable the LPUART transmitter.
 *
 * This function enables or disables the LPUART transmitter, based on the
 * parameter received.
 *
 *
 * @param base LPUART base pointer.
 * @param enable Enable(true) or disable(false) transmitter.
 */
static inline void LPUART_HAL_SetTransmitterCmd(LPUART_Type * base, bool enable)
{
    BITBAND_ACCESS32(&(base->CTRL), LPUART_CTRL_TE_SHIFT) = (uint32_t)enable;
}

/*!
 * @brief Gets the LPUART transmitter enabled/disabled configuration.
 *
 * This function returns true if the LPUART transmitter is enabled, or
 * false, when the transmitter is disabled.
 *
 *
 * @param base LPUART base pointer
 * @return State of LPUART transmitter enable(true)/disable(false)
 */
static inline bool LPUART_HAL_GetTransmitterCmd(LPUART_Type * base)
{
    return (bool)BITBAND_ACCESS32(&(base->CTRL), LPUART_CTRL_TE_SHIFT);
}

/*!
 * @brief Enable/Disable the LPUART receiver.
 *
 * This function enables or disables the LPUART receiver, based on the
 * parameter received.
 *
 *
 * @param base LPUART base pointer
 * @param enable Enable(true) or disable(false) receiver.
 */
static inline void LPUART_HAL_SetReceiverCmd(LPUART_Type * base, bool enable)
{
    BITBAND_ACCESS32(&(base->CTRL), LPUART_CTRL_RE_SHIFT) = (uint32_t)enable;
}

/*!
 * @brief Gets the LPUART receiver enabled/disabled configuration.
 *
 * This function returns true if the LPUART receiver is enabled, or
 * false, when the transmitter is disabled.
 *
 *
 * @param base LPUART base pointer
 * @return State of LPUART receiver enable(true)/disable(false)
 */
static inline bool LPUART_HAL_GetReceiverCmd(LPUART_Type * base)
{
    return (bool)BITBAND_ACCESS32(&(base->CTRL), LPUART_CTRL_RE_SHIFT);
}

/*!
 * @brief Configures the LPUART baud rate.
 *
 * This function configures the LPUART baud rate.
 * In some LPUART instances the user must disable the transmitter/receiver
 * before calling this function.
 * Generally, this may be applied to all LPUARTs to ensure safe operation.
 *
 * @param base LPUART base pointer.
 * @param sourceClockInHz LPUART source input clock in Hz.
 * @param desiredBaudRate LPUART desired baud rate.
 * @return  An error code or kStatus_Success
 */
lpuart_status_t LPUART_HAL_SetBaudRate(LPUART_Type * base,
                                       uint32_t sourceClockInHz,
                                       uint32_t desiredBaudRate);

/*!
 * @brief Sets the LPUART baud rate modulo divisor.
 *
 * This function sets the LPUART baud rate modulo divisor.
 *
 *
 * @param base LPUART base pointer.
 * @param baudRateDivisor The baud rate modulo division "SBR"
 */
static inline void LPUART_HAL_SetBaudRateDivisor(LPUART_Type * base, uint32_t baudRateDivisor)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT((baudRateDivisor < 0x1FFF) && (baudRateDivisor > 1));
#endif
    uint32_t baudRegValTemp;

    baudRegValTemp = base->BAUD;
    baudRegValTemp &= ~(LPUART_BAUD_SBR_MASK);
    baudRegValTemp |= LPUART_BAUD_SBR(baudRateDivisor);
    base->BAUD = baudRegValTemp;
}

#if FSL_FEATURE_LPUART_HAS_BAUD_RATE_OVER_SAMPLING_SUPPORT
/*!
 * @brief Sets the LPUART baud rate oversampling ratio
 *
 * This function sets the LPUART baud rate oversampling ratio.
 * (Note: Feature available on select LPUART instances used together with baud rate programming)
 * The oversampling ratio should be set between 4x (00011) and 32x (11111). Writing
 * an invalid oversampling ratio results in an error and is set to a default
 * 16x (01111) oversampling ratio.
 * Disable the transmitter/receiver before calling this function.
 *
 *
 * @param base LPUART base pointer.
 * @param overSamplingRatio The oversampling ratio "OSR"
 */
static inline void LPUART_HAL_SetOversamplingRatio(LPUART_Type * base, uint32_t overSamplingRatio)
{

/* NOTE: DEV_ASSERT macro is to be implemented; the syntax used for parameters validation may suffer changes */
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(overSamplingRatio < 0x1F);
#endif
    uint32_t baudRegValTemp;

    baudRegValTemp = base->BAUD;
    baudRegValTemp &= ~(LPUART_BAUD_OSR_MASK);
    baudRegValTemp |= LPUART_BAUD_OSR(overSamplingRatio);
    base->BAUD = baudRegValTemp;
}
#endif

#if FSL_FEATURE_LPUART_HAS_BOTH_EDGE_SAMPLING_SUPPORT
/*!
 * @brief Configures the LPUART baud rate both edge sampling
 *
 * This function configures the LPUART baud rate both edge sampling.
 * (Note: Feature available on select LPUART instances used with baud rate programming)
 * When enabled, the received data is sampled on both edges of the baud rate clock.
 * This must be set when the oversampling ratio is between 4x and 7x.
 * This function should only be called when the receiver is disabled.
 *
 *
 * @param base LPUART base pointer.
 * @param enable   Enable (1) or Disable (0) Both Edge Sampling
 * @return An error code or kStatus_Success
 */
static inline void LPUART_HAL_SetBothEdgeSamplingCmd(LPUART_Type * base, bool enable)
{
    BITBAND_ACCESS32(&(base->BAUD), LPUART_BAUD_BOTHEDGE_SHIFT) = (uint32_t)enable;
}
#endif

/*!
 * @brief Returns whether the recevie data is inverted or not.
 *
 * This function returns the polarity of the receive data.
 *
 * @param base LPUART base pointer.
 * @return Rx data polarity; true: inverted, false: not inverted.
 */
static inline bool LPUART_HAL_GetRxDataPolarity(LPUART_Type * base)
{
    return (bool)BITBAND_ACCESS32(&(base->STAT), LPUART_STAT_RXINV_SHIFT);
}

/*!
 * @brief Sets whether the recevie data is inverted or not.
 *
 * This function sets the polarity of the receive data.
 *
 * @param base LPUART base pointer.
 * @param polarity  Rx Data polarity; true: inverted, false: not inverted.
 */
static inline void LPUART_HAL_SetRxDataPolarity(LPUART_Type * base, bool polarity)
{
    BITBAND_ACCESS32(&(base->STAT), LPUART_STAT_RXINV_SHIFT) = (uint32_t)polarity;
}

/*!
 * @brief Configures the number of bits per character in the LPUART controller.
 *
 * This function configures the number of bits per character in the LPUART controller.
 * In some LPUART instances, the user should disable the transmitter/receiver
 * before calling this function.
 * Generally, this may be applied to all LPUARTs to ensure safe operation.
 *
 * @param base LPUART base pointer.
 * @param bitCountPerChar  Number of bits per char (8, 9, or 10, depending on the LPUART instance)
 */
void LPUART_HAL_SetBitCountPerChar(LPUART_Type * base, lpuart_bit_count_per_char_t bitCountPerChar);

/*!
 * @brief Configures parity mode in the LPUART controller.
 *
 * This function configures parity mode in the LPUART controller.
 * In some LPUART instances, the user should disable the transmitter/receiver
 * before calling this function.
 * Generally, this may be applied to all LPUARTs to ensure safe operation.
 *
 * @param base LPUART base pointer.
 * @param parityModeType  Parity mode (enabled, disable, odd, even - see parity_mode_t struct)
 */
void LPUART_HAL_SetParityMode(LPUART_Type * base, lpuart_parity_mode_t parityModeType);

/*!
 * @brief Configures the number of stop bits in the LPUART controller.
 *
 * This function configures the number of stop bits in the LPUART controller.
 * In some LPUART instances, the user should disable the transmitter/receiver
 * before calling this function.
 * Generally, this may be applied to all LPUARTs to ensure safe operation.
 *
 * @param base LPUART base pointer.
 * @param stopBitCount Number of stop bits (1 or 2 - see lpuart_stop_bit_count_t struct)
 * @return  An error code (an unsupported setting in some LPUARTs) or kStatus_Success
 */
static inline void LPUART_HAL_SetStopBitCount(LPUART_Type * base, lpuart_stop_bit_count_t stopBitCount)
{
    BITBAND_ACCESS32(&(base->BAUD), LPUART_BAUD_SBNS_SHIFT) = (uint32_t)stopBitCount;
}

/*!
 * @brief  Get LPUART tx/rx data register address.
 *
 * This function returns LPUART tx/rx data register address.
 *
 *
 * @param base LPUART base pointer.
 * @return  LPUART tx/rx data register address.
 */
static inline uint32_t LPUART_HAL_GetDataRegAddr(LPUART_Type * base)
{
    return (uint32_t)(&(base->DATA));
}

/*@}*/

/*!
 * @name LPUART Interrupts and DMA
 * @{
 */

/*!
 * @brief Configures the LPUART module interrupts.
 *
 * This function configures the LPUART module interrupts to enable/disable various interrupt sources.
 *
 *
 * @param   base LPUART module base pointer.
 * @param   intSrc LPUART interrupt configuration data.
 * @param   enable   true: enable, false: disable.
 */
void LPUART_HAL_SetIntMode(LPUART_Type * base, lpuart_interrupt_t intSrc, bool enable);

/*!
 * @brief Returns LPUART module interrupts state.
 *
 * This function returns whether a certain LPUART module interrupt is enabled or disabled.
 *
 *
 * @param   base LPUART module base pointer.
 * @param   intSrc LPUART interrupt configuration data.
 * @return  true: enable, false: disable.
 */
bool LPUART_HAL_GetIntMode(LPUART_Type * base, lpuart_interrupt_t intSrc);

#if FSL_FEATURE_LPUART_HAS_DMA_ENABLE
/*!
 * @brief Configures DMA requests.
 *
 * This function configures DMA requests for LPUART Transmitter.
 *
 *
 * @param base LPUART base pointer
 * @param enable Transmit DMA request configuration (enable:1 /disable: 0)
 */
static inline void LPUART_HAL_SetTxDmaCmd(LPUART_Type * base, bool enable)
{
    BITBAND_ACCESS32(&(base->BAUD), LPUART_BAUD_TDMAE_SHIFT) = (uint32_t)enable;
}

/*!
 * @brief Configures DMA requests.
 *
 * This function configures DMA requests for LPUART Receiver.
 *
 *
 * @param base LPUART base pointer
 * @param enable Receive DMA request configuration (enable: 1/disable: 0)
 */
static inline void LPUART_HAL_SetRxDmaCmd(LPUART_Type * base, bool enable)
{
    BITBAND_ACCESS32(&(base->BAUD), LPUART_BAUD_RDMAE_SHIFT) = (uint32_t)enable;
}

/*!
 * @brief Gets the LPUART DMA request configuration.
 *
 * This function returns the LPUART Transmit DMA request configuration.
 *
 *
 * @param base LPUART base pointer
 * @return Transmit DMA request configuration (enable: 1/disable: 0)
 */
static inline bool LPUART_HAL_IsTxDmaEnabled(LPUART_Type * base)
{
    return (bool)BITBAND_ACCESS32(&(base->BAUD), LPUART_BAUD_TDMAE_SHIFT);
}

/*!
 * @brief Gets the LPUART DMA request configuration.
 *
 * This function returns the LPUART Receive DMA request configuration.
 *
 *
 * @param base LPUART base pointer
 * @return Receives the DMA request configuration (enable: 1/disable: 0).
 */
static inline bool LPUART_HAL_IsRxDmaEnabled(LPUART_Type * base)
{
    return (bool)BITBAND_ACCESS32(&(base->BAUD), LPUART_BAUD_RDMAE_SHIFT);
}

#endif

/*@}*/

/*!
 * @name LPUART Transfer Functions
 * @{
 */

/*!
 * @brief Sends the LPUART 8-bit character.
 *
 * This functions sends an 8-bit character.
 *
 *
 * @param base LPUART Instance
 * @param data     data to send (8-bit)
 */
static inline void LPUART_HAL_Putchar(LPUART_Type * base, uint8_t data)
{
    uint8_t * dataRegBytes = (uint8_t *)(&(base->DATA));
    dataRegBytes[0] = data;
}

/*!
 * @brief Sends the LPUART 9-bit character.
 *
 * This functions sends a 9-bit character.
 *
 *
 * @param base LPUART Instance
 * @param data     data to send (9-bit)
 */
void LPUART_HAL_Putchar9(LPUART_Type * base, uint16_t data);

/*!
 * @brief Sends the LPUART 10-bit character (Note: Feature available on select LPUART instances).
 *
 * This functions sends a 10-bit character.
 *
 *
 * @param base LPUART Instance
 * @param data   data to send (10-bit)
 * @return An error code or kStatus_Success
 */
lpuart_status_t LPUART_HAL_Putchar10(LPUART_Type * base, uint16_t data);

/*!
 * @brief Gets the LPUART 8-bit character.
 *
 * This functions receives an 8-bit character.
 *
 *
 * @param base LPUART base pointer
 * @param readData Data read from receive (8-bit)
 */
static inline void LPUART_HAL_Getchar(LPUART_Type * base, uint8_t *readData)
{
    *readData = (uint8_t)base->DATA;
}

/*!
 * @brief Gets the LPUART 9-bit character.
 *
 * This functions receives a 9-bit character.
 *
 *
 * @param base LPUART base pointer
 * @param readData Data read from receive (9-bit)
 */
void LPUART_HAL_Getchar9(LPUART_Type * base, uint16_t *readData);

/*!
 * @brief Gets the LPUART 10-bit character.
 *
 * This functions receives a 10-bit character.
 *
 *
 * @param base LPUART base pointer
 * @param readData Data read from receive (10-bit)
 */
void LPUART_HAL_Getchar10(LPUART_Type * base, uint16_t *readData);

/*!
 * @brief Send out multiple bytes of data using polling method.
 *
 * This function only supports 8-bit transaction.
 *
 * @param   base LPUART module base pointer.
 * @param   txBuff The buffer pointer which saves the data to be sent.
 * @param   txSize Size of data to be sent in unit of byte.
 */
void LPUART_HAL_SendDataPolling(LPUART_Type * base, const uint8_t *txBuff, uint32_t txSize);

/*!
 * @brief Receive multiple bytes of data using polling method.
 *
 * This function only supports 8-bit transaction.
 *
 * @param   base LPUART module base pointer.
 * @param   rxBuff The buffer pointer which saves the data to be received.
 * @param   rxSize Size of data need to be received in unit of byte.
 * @return  Whether the transaction is success or rx overrun.
 */
lpuart_status_t LPUART_HAL_ReceiveDataPolling(LPUART_Type * base, uint8_t *rxBuff, uint32_t rxSize);

/*@}*/

/*!
 * @name LPUART Status Flags
 * @{
 */

/*!
 * @brief  LPUART get status flag
 *
 * This function returns the state of a status flag.
 *
 *
 * @param base LPUART base pointer
 * @param statusFlag  The status flag to query
 * @return Whether the current status flag is set(true) or not(false).
 */
bool LPUART_HAL_GetStatusFlag(LPUART_Type * base, lpuart_status_flag_t statusFlag);

/*!
 * @brief LPUART clears an individual status flag.
 *
 * This function clears an individual status flag (see lpuart_status_flag_t for list of status bits).
 *
 *
 * @param base LPUART base pointer
 * @param statusFlag  Desired LPUART status flag to clear
 * @return An error code or kStatus_Success
 */
lpuart_status_t LPUART_HAL_ClearStatusFlag(LPUART_Type * base, lpuart_status_flag_t statusFlag);

/*@}*/

/*!
 * @name LPUART Special Feature Configurations
 * @{
 */

/*!
 * @brief Configures the number of idle characters.
 *
 * This function Configures the number of idle characters that must be
 * received before the IDLE flag is set.
 *
 *
 * @param base LPUART base pointer
 * @param idleConfig Idle characters configuration
 */
static inline void LPUART_HAL_SetIdleChar(LPUART_Type * base, lpuart_idle_char_t idleConfig)
{
    uint32_t ctrlRegValTemp;

    ctrlRegValTemp = base->CTRL;
    ctrlRegValTemp &= ~(LPUART_CTRL_IDLECFG_MASK);
    ctrlRegValTemp |= LPUART_CTRL_IDLECFG(idleConfig);
    base->CTRL = ctrlRegValTemp;
}

/*!
 * @brief Gets the number of idle characters for IDLE flag.
 *
 * This function returns the number of idle characters that must be received
 * before the IDLE flag is set.
 *
 *
 * @param base LPUART base pointer
 * @return  idle characters configuration
 */
static inline lpuart_idle_char_t LPUART_HAL_GetIdleChar(LPUART_Type * base)
{
    uint32_t ctrlRegVal = base->CTRL;
    ctrlRegVal = (ctrlRegVal & LPUART_CTRL_IDLECFG_MASK) >> LPUART_CTRL_IDLECFG_SHIFT;
    return (lpuart_idle_char_t)ctrlRegVal;
}

/*!
 * @brief Checks whether the current data word was received with noise.
 *
 * This function returns whether the current data word was received with noise.
 *
 *
 * @param base LPUART base pointer.
 * @return The status of the NOISY bit in the LPUART extended data register
 */
static inline bool LPUART_HAL_IsCurrentDataWithNoise(LPUART_Type * base)
{
    return (bool)BITBAND_ACCESS32(&(base->DATA), LPUART_DATA_NOISY_SHIFT);
}

/*!
 * @brief Checks whether the current data word was received with frame error.
 *
 * This function returns whether the current data word was received with frame error.
 *
 *
 * @param base LPUART base pointer
 * @return The status of the FRETSC bit in the LPUART extended data register
 */
static inline bool LPUART_HAL_IsCurrentDataWithFrameError(LPUART_Type * base)
{
    return (bool)BITBAND_ACCESS32(&(base->DATA), LPUART_DATA_FRETSC_SHIFT);
}

/*!
 * @brief Indicates a special character is to be transmitted.
 *
 * This function sets this bit to indicate a break or idle character is to be transmitted
 * instead of the contents in DATA[T9:T0].
 *
 *
 * @param base LPUART base pointer
 * @param specialChar T9 is used to indicate a break character when 0 an idle
 * character when 1, the contents of DATA[T8:T0] should be zero.
 */
static inline void LPUART_HAL_SetTxSpecialChar(LPUART_Type * base, uint8_t specialChar)
{
    BITBAND_ACCESS32(&(base->DATA), LPUART_DATA_FRETSC_SHIFT) = (uint32_t)specialChar;
}

/*!
 * @brief Checks whether the current data word was received with parity error.
 *
 * This function returns whether the current data word was received with parity error.
 *
 *
 * @param base LPUART base pointer
 * @return The status of the PARITYE bit in the LPUART extended data register
 */
static inline bool LPUART_HAL_IsCurrentDataWithParityError(LPUART_Type * base)
{
    return (bool)BITBAND_ACCESS32(&(base->DATA), LPUART_DATA_PARITYE_SHIFT);
}

/*!
 * @brief Checks whether the receive buffer is empty.
 *
 * This function returns whether the receive buffer is empty.
 *
 *
 * @param base LPUART base pointer
 * @return TRUE if the receive-buffer is empty, else FALSE.
 */
static inline bool LPUART_HAL_IsReceiveBufferEmpty(LPUART_Type * base)
{
    return (bool)BITBAND_ACCESS32(&(base->DATA), LPUART_DATA_RXEMPT_SHIFT);
}

/*!
 * @brief Checks whether the previous BUS state was idle before this byte is received.
 *
 * This function returns whether the previous BUS state was idle before this byte is received.
 *
 *
 * @param base LPUART base pointer
 * @return TRUE if the previous BUS state was IDLE, else FALSE.
 */
static inline bool LPUART_HAL_WasPreviousReceiverStateIdle(LPUART_Type * base)
{
    return (bool)BITBAND_ACCESS32(&(base->DATA), LPUART_DATA_IDLINE_SHIFT);
}

/*!
 * @brief Configures the LPUART operation in wait mode (operates or stops operations in wait mode).
 *
 * This function configures the LPUART operation in wait mode (operates or stops operations in wait mode).
 * In some LPUART instances, the user should disable the transmitter/receiver
 * before calling this function.
 * Generally, this may be applied to all LPUARTs to ensure safe operation.
 *
 * @param base LPUART base pointer
 * @param mode     LPUART wait mode operation - operates or stops to operate in wait mode.
 */
static inline void  LPUART_HAL_SetWaitModeOperation(LPUART_Type * base, lpuart_operation_config_t mode)
{
    /* In CPU wait mode: 0 - lpuart clocks continue to run; 1 - lpuart clocks freeze */
    BITBAND_ACCESS32(&(base->CTRL), LPUART_CTRL_DOZEEN_SHIFT) = (uint32_t)mode;
}

/*!
 * @brief Gets the LPUART operation in wait mode.
 *
 * This function returns the LPUART operation in wait mode
 * (operates or stops operations in wait mode).
 *
 *
 * @param base LPUART base pointer
 * @return LPUART wait mode operation configuration
 *         - kLpuartOperates or KLpuartStops in wait mode
 */
static inline lpuart_operation_config_t LPUART_HAL_GetWaitModeOperation(LPUART_Type * base)
{
    /* In CPU wait mode: 0 - lpuart clocks continue to run; 1 - lpuart clocks freeze  */
    return (lpuart_operation_config_t)BITBAND_ACCESS32(&(base->CTRL), LPUART_CTRL_DOZEEN_SHIFT);
}

/*!
 * @brief Configures the LPUART loopback operation (enable/disable loopback operation)
 *
 * This function configures the LPUART loopback operation (enable/disable loopback operation).
 * In some LPUART instances, the user should disable the transmitter/receiver
 * before calling this function.
 * Generally, this may be applied to all LPUARTs to ensure safe operation.
 *
 * @param base LPUART base pointer
 * @param enable   LPUART loopback mode - disabled (0) or enabled (1)
 */
void LPUART_HAL_SetLoopbackCmd(LPUART_Type * base, bool enable);

/*!
 * @brief Configures the LPUART single-wire operation (enable/disable single-wire mode).
 *
 * This function configures the LPUART single-wire operation (enable/disable single-wire mode).
 * In some LPUART instances, the user should disable the transmitter/receiver
 * before calling this function.
 * Generally, this may be applied to all LPUARTs to ensure safe operation.
 *
 * @param base LPUART base pointer
 * @param enable   LPUART loopback mode - disabled (0) or enabled (1)
 */
void LPUART_HAL_SetSingleWireCmd(LPUART_Type * base, bool enable);

/*!
 * @brief Configures the LPUART transmit direction while in single-wire mode.
 *
 * This function configures the LPUART transmit direction while in single-wire mode.
 *
 *
 * @param base LPUART base pointer
 * @param direction LPUART single-wire transmit direction - input or output
 */
static inline void LPUART_HAL_SetTxdirInSinglewireMode(LPUART_Type * base,
                                                 lpuart_singlewire_txdir_t direction)
{
    BITBAND_ACCESS32(&(base->CTRL), LPUART_CTRL_TXDIR_SHIFT) = (uint32_t)direction;
}

/*!
 * @brief  Places the LPUART receiver in standby mode.
 *
 * This function places the LPUART receiver in standby mode.
 *
 *
 * @param base LPUART base pointer
 * @return Error code or kStatus_Success
 */
lpuart_status_t LPUART_HAL_SetReceiverInStandbyMode(LPUART_Type * base);

/*!
 * @brief  Places the LPUART receiver in a normal mode.
 *
 * This function places the LPUART receiver in a normal mode (disable standby mode operation).
 *
 *
 * @param base LPUART base pointer
 */
static inline void LPUART_HAL_PutReceiverInNormalMode(LPUART_Type * base)
{
    BITBAND_ACCESS32(&(base->CTRL), LPUART_CTRL_RWU_SHIFT) = (uint32_t)0;
}

/*!
 * @brief  Checks whether the LPUART receiver is in a standby mode.
 *
 * This function returns whether the LPUART receiver is in a standby mode.
 *
 *
 * @param base LPUART base pointer
 * @return LPUART in normal more (0) or standby (1)
 */
static inline bool LPUART_HAL_IsReceiverInStandby(LPUART_Type * base)
{
    return (bool)BITBAND_ACCESS32(&(base->CTRL), LPUART_CTRL_RWU_SHIFT);
}

/*!
 * @brief  Sets the LPUART receiver wakeup method from standby mode.
 *
 * This function sets the LPUART receiver wakeup method (idle line or addr-mark)
 * from standby mode.
 *
 *
 * @param base LPUART base pointer
 * @param method   LPUART wakeup method: 0 - Idle-line wake (default), 1 - addr-mark wake
 */
static inline void LPUART_HAL_SetReceiverWakeupMode(LPUART_Type * base,
                                                    lpuart_wakeup_method_t method)
{
    BITBAND_ACCESS32(&(base->CTRL), LPUART_CTRL_WAKE_SHIFT) = (uint32_t)method;
}

/*!
 * @brief  Gets the LPUART receiver wakeup method from standby mode.
 *
 * This function returns the LPUART receiver wakeup method (idle line or addr-mark)
 * from standby mode.
 *
 *
 * @param base LPUART base pointer
 * @return  LPUART wakeup method: kLpuartIdleLineWake: 0 - Idle-line wake (default),
 *          kLpuartAddrMarkWake: 1 - addr-mark wake
 */
static inline lpuart_wakeup_method_t LPUART_HAL_GetReceiverWakeupMode(LPUART_Type * base)
{
    return (lpuart_wakeup_method_t)BITBAND_ACCESS32(&(base->CTRL), LPUART_CTRL_WAKE_SHIFT);
}

/*!
 * @brief  LPUART idle-line detect operation configuration.
 *
 * This function configures idle-line detect operation configuration
 * (idle line bit-count start and wake up affect on IDLE status bit).
 * In some LPUART instances, the user should disable the transmitter/receiver
 * before calling this function.
 * Generally, this may be applied to all LPUARTs to ensure safe operation.
 *
 * @param base LPUART base pointer
 * @param config   LPUART configuration data for idle line detect operation
 */
void LPUART_HAL_SetIdleLineDetect(LPUART_Type * base,
                                  const lpuart_idle_line_config_t *config);

/*!
 * @brief  LPUART break character transmit length configuration
 *
 * This function configures the break char length.
 * In some LPUART instances, the user should disable the transmitter before calling
 * this function. Generally, this may be applied to all LPUARTs to ensure safe operation.
 *
 * @param base LPUART base pointer
 * @param length   LPUART break character length setting: 0 - minimum 10-bit times (default),
 *                   1 - minimum 13-bit times
 */
static inline void LPUART_HAL_SetBreakCharTransmitLength(LPUART_Type * base,
                                             lpuart_break_char_length_t length)
{
    BITBAND_ACCESS32(&(base->STAT), LPUART_STAT_BRK13_SHIFT) = (uint32_t)length;
}

/*!
 * @brief  LPUART break character detect length configuration
 *
 * This function sets the LPUART detectable break character length.
 *
 *
 * @param base LPUART base pointer
 * @param length  LPUART break character length setting: 0 - minimum 10-bit times (default),
 *                  1 - minimum 13-bit times
 */
static inline void LPUART_HAL_SetBreakCharDetectLength(LPUART_Type * base,
                                           lpuart_break_char_length_t length)
{
    BITBAND_ACCESS32(&(base->STAT), LPUART_STAT_LBKDE_SHIFT) = (uint32_t)length;
}

/*!
 * @brief  LPUART transmit sends break character configuration.
 *
 * This function sets break character transmission mode (normal/queue).
 *
 *
 * @param base LPUART base pointer
 * @param enable LPUART normal/queue break char - disabled (normal mode, default: 0) or
 *                 enabled (queue break char: 1)
 */
static inline void LPUART_HAL_QueueBreakCharToSend(LPUART_Type * base, bool enable)
{
    BITBAND_ACCESS32(&(base->CTRL), LPUART_CTRL_SBK_SHIFT) = (uint32_t)enable;
}

/*!
 * @brief Configures match address mode control.
 *
 * This function configures match address mode control.
 *
 *
 * @param base LPUART base pointer
 * @param config MATCFG: Configures the match addressing mode used.
 */
static inline void LPUART_HAL_SetMatchAddressMode(LPUART_Type * base, lpuart_match_config_t config)
{
    uint32_t baudRegValTemp;

    baudRegValTemp = base->BAUD;
    baudRegValTemp &= ~(LPUART_BAUD_MATCFG_MASK);
    baudRegValTemp |= LPUART_BAUD_MATCFG(config);
    base->BAUD = baudRegValTemp;
}

/*!
 * @brief Configures address match register 1.
 *
 * This function configures address match register 1.
 * The MAEN bit must be cleared before configuring MA value, so the enable/disable
 * and set value must be included inside one function.
 *
 * @param base LPUART base pointer
 * @param enable Match address model enable (true)/disable (false)
 * @param value Match address value to program into match address register 1
 */
void LPUART_HAL_SetMatchAddressReg1(LPUART_Type * base, bool enable, uint8_t value);

/*!
 * @brief Configures address match register 2.
 *
 * This function configures address match register 2.
 * The MAEN bit must be cleared before configuring MA value, so the enable/disable
 * and set value must be included inside one function.
 *
 * @param base LPUART base pointer
 * @param enable Match address model enable (true)/disable (false)
 * @param value Match address value to program into match address register 2
 */
void LPUART_HAL_SetMatchAddressReg2(LPUART_Type * base, bool enable, uint8_t value);

/*!
 * @brief LPUART sends the MSB first configuration
 *
 * This function configures whether MSB is sent first.
 * Note: In some LPUART instances, the user should disable the transmitter/receiver
 * before calling this function.
 * Generally, this may be applied to all LPUARTs to ensure safe operation.
 *
 * @param base LPUART base pointer
 * @param enable  false - LSB (default, disabled), true - MSB (enabled)
 */
static inline void LPUART_HAL_SetSendMsbFirstCmd(LPUART_Type * base, bool enable)
{
    BITBAND_ACCESS32(&(base->STAT), LPUART_STAT_MSBF_SHIFT) = (uint32_t)enable;
}

/*!
 * @brief  LPUART enable/disable re-sync of received data configuration.
 *
 * This function enables or disables re-sync of received data, based on the
 * parameter received.
 *
 *
 * @param base LPUART base pointer
 * @param enable  re-sync of received data word configuration:
 *                true - re-sync of received data word (default)
 *                false - disable the re-sync
 */
static inline void LPUART_HAL_SetReceiveResyncCmd(LPUART_Type * base, bool enable)
{
    /* When disabled, the resynchronization of the received data word when a data
     * one followed by data zero transition is detected. This bit should only be
     * changed when the receiver is disabled. */
    BITBAND_ACCESS32(&(base->BAUD), LPUART_BAUD_RESYNCDIS_SHIFT) = (uint32_t)enable;
}

#if FSL_FEATURE_LPUART_HAS_MODEM_SUPPORT
/*!
 * @brief  Transmits the CTS source configuration.
 *
 * This function transmits the CTS source configuration.
 *
 *
 * @param base LPUART base pointer
 * @param source   LPUART CTS source
 */
static inline void LPUART_HAL_SetCtsSource(LPUART_Type * base,
                                           lpuart_cts_source_t source)
{
    BITBAND_ACCESS32(&(base->MODIR), LPUART_MODIR_TXCTSSRC_SHIFT) = (uint32_t)source;
}

/*!
 * @brief  Transmits the CTS configuration.
 *
 * This function transmits the CTS configuration.
 * Note: configures if the CTS state is checked at the start of each character
 * or only when the transmitter is idle.
 *
 *
 * @param base LPUART base pointer
 * @param mode     LPUART CTS configuration
 */
static inline void LPUART_HAL_SetCtsMode(LPUART_Type * base, lpuart_cts_config_t mode)
{
    BITBAND_ACCESS32(&(base->MODIR), LPUART_MODIR_TXCTSC_SHIFT) = (uint32_t)mode;
}

/*!
 * @brief Enable/Disable the transmitter clear-to-send.
 *
 * This function controls the transmitter clear-to-send, based on the parameter
 * received.
 *
 *
 * @param base LPUART base pointer
 * @param enable  disable(0)/enable(1) transmitter CTS.
 */
static inline void LPUART_HAL_SetTxCtsCmd(LPUART_Type * base, bool enable)
{
    BITBAND_ACCESS32(&(base->MODIR), LPUART_MODIR_TXCTSE_SHIFT) = (uint32_t)enable;
}

/*!
 * @brief  Enable/Disable the receiver request-to-send.
 *
 * This function enables  or disables the receiver request-to-send, based 
 * on the parameter received.
 * Note: do not enable both Receiver RTS (RXRTSE) and Transmit RTS (TXRTSE).
 *
 *
 * @param base LPUART base pointer
 * @param enable  disable(0)/enable(1) receiver RTS.
 */
static inline void LPUART_HAL_SetRxRtsCmd(LPUART_Type * base, bool enable)
{
    BITBAND_ACCESS32(&(base->MODIR), LPUART_MODIR_RXRTSE_SHIFT) = (uint32_t)enable;
}

/*!
 * @brief  Enable/Disable the transmitter request-to-send.
 *
 * This function enables  or disables the transmitter request-to-send, based 
 * on the parameter received.
 * Note: do not enable both Receiver RTS (RXRTSE) and Transmit RTS (TXRTSE).
 *
 *
 * @param base LPUART base pointer
 * @param enable  disable(0)/enable(1) transmitter RTS.
 */
static inline void LPUART_HAL_SetTxRtsCmd(LPUART_Type * base, bool enable)
{
    BITBAND_ACCESS32(&(base->MODIR), LPUART_MODIR_TXRTSE_SHIFT) = (uint32_t)enable;
}

/*!
 * @brief Configures the transmitter RTS polarity.
 *
 * This function configures the transmitter RTS polarity.
 *
 *
 * @param base LPUART base pointer
 * @param polarity Settings to choose RTS polarity (0=active low, 1=active high)
 */
static inline void LPUART_HAL_SetTxRtsPolarityMode(LPUART_Type * base, bool polarity)
{
    BITBAND_ACCESS32(&(base->MODIR), LPUART_MODIR_TXRTSPOL_SHIFT) = (uint32_t)polarity;
}

#endif  /* FSL_FEATURE_LPUART_HAS_MODEM_SUPPORT */

#if FSL_FEATURE_LPUART_FIFO_SIZE
/*!
 * @brief  Enable/Disable the transmitter FIFO.
 *
 * This function enables or disables the transmitter FIFO structure, based 
 * on the parameter received.
 * Note: The size of the FIFO structure is indicated by TXFIFOSIZE.
 *
 *
 * @param base LPUART base pointer
 * @param enable  disable(0)/enable(1) transmitter FIFO.
 */
static inline void LPUART_HAL_SetTxFIFOCmd(LPUART_Type * base, bool enable)
{
    BITBAND_ACCESS32(&(base->FIFO), LPUART_FIFO_TXFE_SHIFT) = (uint32_t)enable;
}

/*!
 * @brief  Enable/Disable the receiver FIFO.
 *
 * This function enables or disables the receiver FIFO structure, based 
 * on the parameter received.
 * Note: The size of the FIFO structure is indicated by RXFIFOSIZE.
 *
 *
 * @param base LPUART base pointer
 * @param enable  disable(0)/enable(1) receiver FIFO.
 */
static inline void LPUART_HAL_SetRxFIFOCmd(LPUART_Type * base, bool enable)
{
    BITBAND_ACCESS32(&(base->FIFO), LPUART_FIFO_RXFE_SHIFT) = (uint32_t)enable;
}

/*!
 * @brief Enables the assertion of RDRF when the receiver is idle.
 *
 * This function enables the assertion of RDRF when the receiver is idle for 
 * a number of idle characters and the FIFO is not empty.
 *
 *
 * @param base LPUART base pointer.
 * @param duration  The number of characters the receiver must be empty before RDRF assertion
 *        0 - disabled, >0 - rx must be idle for 2^(duration-1) characters before RDRF assertion
 */
static inline void LPUART_HAL_SetRxIdleEmptyDuration(LPUART_Type * base, uint8_t duration)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(duration < ((uint8_t)(1U << LPUART_FIFO_RXIDEN_WIDTH)));
#endif
    uint32_t fifoRegValTemp;

    fifoRegValTemp = base->FIFO;
    fifoRegValTemp &= ~(LPUART_FIFO_RXIDEN_MASK);
    fifoRegValTemp |= LPUART_FIFO_RXIDEN(duration);
    base->FIFO = fifoRegValTemp;
}

/*!
 * @brief  Flush tx FIFO buffer.
 *
 * This function flushes the tx FIFO buffer.
 * Note: This does not affect data that is in the transmit shift register.
 *
 *
 * @param base LPUART base pointer
 */
static inline void LPUART_HAL_FlushTxFifoBuffer(LPUART_Type * base)
{
    BITBAND_ACCESS32(&(base->FIFO), LPUART_FIFO_TXFLUSH_SHIFT) = (uint32_t)1U;
}

/*!
 * @brief  Flush rx FIFO buffer.
 *
 * This function flushes the rx FIFO buffer.
 * Note: This does not affect data that is in the receive shift register.
 *
 *
 * @param base LPUART base pointer
 */
static inline void LPUART_HAL_FlushRxFifoBuffer(LPUART_Type * base)
{
    BITBAND_ACCESS32(&(base->FIFO), LPUART_FIFO_RXFLUSH_SHIFT) = (uint32_t)1U;
}

/*!
 * @brief  Sets the tx watermark.
 *
 * This function sets the tx FIFO watermark.
 * When the number of datawords in the transmit FIFO/buffer is equal to or less than the value in 
 * this register field, an interrupt or a DMA request is generated.
 * Note: For proper operation, the value in TXWATER must be set to be less than the 
 * transmit FIFO/buffer size and greater than 0.
 *
 *
 * @param base LPUART base pointer
 * @param txWater Tx FIFO Watermark
 */
static inline void LPUART_HAL_SetTxWatermark(LPUART_Type * base, uint8_t txWater)
{
#ifdef DEV_ERROR_DETECT
    uint8_t txFifoSize = ((uint8_t)((base->FIFO >> LPUART_FIFO_TXFIFOSIZE_SHIFT) & LPUART_FIFO_TXFIFOSIZE_MASK));
    DEV_ASSERT(txFifoSize > 0U);
    DEV_ASSERT(txWater < ((uint8_t)(1U << (txFifoSize + 1U)));
#endif
    uint32_t waterRegValTemp;

    waterRegValTemp = base->WATER;
    waterRegValTemp &= ~(LPUART_WATER_TXWATER_MASK);
    waterRegValTemp |= LPUART_WATER_TXWATER(txWater);
    base->WATER = waterRegValTemp;
}

/*!
 * @brief  Sets the rx watermark.
 *
 * This function sets the rx FIFO watermark.
 * When the number of datawords in the receive FIFO/buffer is greater than the value in 
 * this register field, an interrupt or a DMA request is generated.
 * Note: For proper operation, the value in RXWATER must be set to be less than the 
 * receive FIFO/buffer size and greater than 0.
 *
 *
 * @param base LPUART base pointer
 * @param rxWater Rx FIFO Watermark
 */
static inline void LPUART_HAL_SetRxWatermark(LPUART_Type * base, uint8_t rxWater)
{
#ifdef DEV_ERROR_DETECT
    uint8_t rxFifoSize = ((uint8_t)((base->FIFO >> LPUART_FIFO_RXFIFOSIZE_SHIFT) & LPUART_FIFO_RXFIFOSIZE_MASK));
    DEV_ASSERT(rxFifoSize > 0U);
    DEV_ASSERT(rxWater < ((uint8_t)(1U << (rxFifoSize + 1U)));
#endif
    uint32_t waterRegValTemp;

    waterRegValTemp = base->WATER;
    waterRegValTemp &= ~(LPUART_WATER_RXWATER_MASK);
    waterRegValTemp |= LPUART_WATER_RXWATER(rxWater);
    base->WATER = waterRegValTemp;
}

#endif /* FSL_FEATURE_LPUART_FIFO_SIZE */

#if FSL_FEATURE_LPUART_HAS_IR_SUPPORT
/*!
 * @brief  Configures the LPUART infrared operation.
 *
 * This function configures the LPUART infrared operation.
 *
 *
 * @param base LPUART base pointer
 * @param enable    Enable (1) or disable (0) the infrared operation
 * @param pulseWidth The transmit narrow pulse width of type lpuart_ir_tx_pulsewidth_t
 */
void LPUART_HAL_SetInfrared(LPUART_Type * base, bool enable,
                            lpuart_ir_tx_pulsewidth_t pulseWidth);
#endif  /* FSL_FEATURE_LPUART_HAS_IR_SUPPORT */

/*@}*/

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __FSL_LPUART_HAL_H__ */
/*******************************************************************************
 * EOF
 ******************************************************************************/

