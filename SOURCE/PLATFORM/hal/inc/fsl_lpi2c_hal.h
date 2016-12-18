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
#ifndef __FSL_LPI2C_HAL_H__
#define __FSL_LPI2C_HAL_H__

#include <stdbool.h>
#include "fsl_device_registers.h"

/*!
 * @addtogroup lpi2c_hal
 * @{
 */

/*!
 * @file
 *
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*!
 * LPI2C master interrupts
 */

#define   LPI2C_HAL_MASTER_DATA_MATCH_INT           0x4000UL   /*!< Data Match Interrupt       */
#define   LPI2C_HAL_MASTER_PIN_LOW_TIMEOUT_INT      0x2000UL   /*!< Pin Low Timeout Interrupt  */
#define   LPI2C_HAL_MASTER_FIFO_ERROR_INT           0x1000UL   /*!< FIFO Error Interrupt       */
#define   LPI2C_HAL_MASTER_ARBITRATION_LOST_INT      0x800UL   /*!< Arbitration Lost Interrupt */
#define   LPI2C_HAL_MASTER_NACK_DETECT_INT           0x400UL   /*!< NACK Detect Interrupt      */
#define   LPI2C_HAL_MASTER_STOP_DETECT_INT           0x200UL   /*!< STOP Detect Interrupt      */
#define   LPI2C_HAL_MASTER_END_PACKET_INT            0x100UL   /*!< End Packet Interrupt       */
#define   LPI2C_HAL_MASTER_RECEIVE_DATA_INT            0x2UL   /*!< Receive Data Interrupt     */
#define   LPI2C_HAL_MASTER_TRANSMIT_DATA_INT           0x1UL   /*!< Transmit Data Interrupt    */
                                     
/*!
 * LPI2C slave interrupts
 */
#define   LPI2C_HAL_SLAVE_SMBUS_ALERT_RESPONSE_INT  0x8000UL   /*!< SMBus Alert Response Interrupt */
#define   LPI2C_HAL_SLAVE_GENERAL_CALL_INT          0x4000UL   /*!< General Call Interrupt         */
#define   LPI2C_HAL_SLAVE_ADDRESS_MATCH_1_INT       0x2000UL   /*!< Address Match 1 Interrupt      */
#define   LPI2C_HAL_SLAVE_ADDRESS_MATCH_0_INT       0x1000UL   /*!< Address Match 0 Interrupt      */
#define   LPI2C_HAL_SLAVE_FIFO_ERROR_INT             0x800UL   /*!< FIFO Error Interrupt           */
#define   LPI2C_HAL_SLAVE_BIT_ERROR_INT              0x400UL   /*!< Bit Error Interrupt            */
#define   LPI2C_HAL_SLAVE_STOP_DETECT_INT            0x200UL   /*!< STOP Detect Interrupt          */
#define   LPI2C_HAL_SLAVE_REPEATED_START_INT         0x100UL   /*!< Repeated Start Interrupt       */
#define   LPI2C_HAL_SLAVE_TRANSMIT_ACK_INT             0x8UL   /*!< Transmit ACK Interrupt         */
#define   LPI2C_HAL_SLAVE_ADDRESS_VALID_INT            0x4UL   /*!< Address Valid Interrupt        */
#define   LPI2C_HAL_SLAVE_RECEIVE_DATA_INT             0x2UL   /*!< Receive Data Interrupt         */
#define   LPI2C_HAL_SLAVE_TRANSMIT_DATA_INT            0x1UL   /*!< Transmit Data Interrupt        */

/*! @brief LPI2C module version number */
typedef struct
{
    uint8_t  majorNumber;       /**< Major Version Number */
    uint8_t  minorNumber;       /**< Minor Version Number */
    uint16_t featureNumber;     /**< Feature Specification Number */
} lpi2c_version_info_t;

/*! @brief Non_matching data discard options */
typedef enum lpi2c_rx_data_match {
    LPI2C_RX_DATA_KEEP_ALL           = 0U,   /*!< Received data is stored in the receive FIFO as normal */
    LPI2C_RX_DATA_DROP_NON_MATCHING  = 1U,   /*!< Received data is discarded unless the RMF is set */
} lpi2c_rx_data_match_t;

/*! @brief Host request input source selection */
typedef enum lpi2c_hreq_source {
    LPI2C_HREQ_EXTERNAL_PIN      = 0U,   /*!< Host request input is external pin */
    LPI2C_HREQ_INTERNAL_TRIGGER  = 1U,   /*!< Host request input is internal trigger */
} lpi2c_hreq_source_t;

/*! @brief Host request input polarity selection */
typedef enum lpi2c_hreq_polarity {
    LPI2C_HREQ_POL_ACTIVE_HIGH  = 0U,   /*!< Host request active low */
    LPI2C_HREQ_POL_ACTIVE_LOW   = 1U,   /*!< Host request active high */
} lpi2c_hreq_polarity_t;

/*! @brief Pin configuration selection */
typedef enum lpi2c_pin_config {
    LPI2C_CFG_2PIN_OPEN_DRAIN             = 0U,  /*!< 2-pin open drain mode */
    LPI2C_CFG_2PIN_OUTPUT_ONLY            = 1U,  /*!< 2-pin output only mode (ultra-fast mode) */
    LPI2C_CFG_2PIN_PUSH_PULL              = 2U,  /*!< 2-pin push-pull mode */
    LPI2C_CFG_4PIN_PUSH_PULL              = 3U,  /*!< 4-pin push-pull mode */
    LPI2C_CFG_2PIN_OPEN_DRAIN_SLAVE       = 4U,  /*!< 2-pin open drain mode with separate LPI2C slave */
    LPI2C_CFG_2PIN_OUTPUT_ONLY_SLAVE      = 5U,  /*!< 2-pin output only mode (ultra-fast mode) with separate LPI2C slave */
    LPI2C_CFG_2PIN_PUSH_PULL_SLAVE        = 6U,  /*!< 2-pin push-pull mode with separate LPI2C slave */
    LPI2C_CFG_4PIN_PUSH_PULL_INVERTED     = 7U,  /*!< 4-pin push-pull mode (inverted outputs) */
} lpi2c_pin_config_t;

/*! @brief Data match selection */
typedef enum lpi2c_match_config {
    LPI2C_MATCH_DISABLED       = 0U,  /*!< Match disabled */
    LPI2C_MATCH_OR_FIRST       = 2U,  /*!< Match enabled (1st data word equals MATCH0 OR MATCH1) */
    LPI2C_MATCH_OR_ANY         = 3U,  /*!< Match enabled (any data word equals MATCH0 OR MATCH1) */
    LPI2C_MATCH_AND_FIRST      = 4U,  /*!< Match enabled (1st data word equals MATCH0 AND 2nd data word equals MATCH1) */
    LPI2C_MATCH_AND_ANY        = 5U,  /*!< Match enabled (any data word equals MATCH0 AND next data word equals MATCH1) */
    LPI2C_MATCH_MASK_FIRST     = 6U,  /*!< Match enabled (1st data word AND MATCH1 equals MATCH0 AND MATCH1) */
    LPI2C_MATCH_MASK_ANY       = 7U,  /*!< Match enabled (any data word AND MATCH1 equals MATCH0 AND MATCH1) */
} lpi2c_match_config_t;

/*! @brief SCL/SDA low time-out configuration */
typedef enum lpi2c_timeout_config {
    LPI2C_TIMEOUT_SCL          = 0U,  /*!< Pin Low Timeout Flag will set if SCL is low for longer than the configured timeout */
    LPI2C_TIMEOUT_SCL_OR_SDA   = 1U,  /*!< Pin Low Timeout Flag will set if either SCL or SDA is low for longer than the configured timeout */
} lpi2c_timeout_config_t;

/*! @brief Master NACK reaction configuration */
typedef enum lpi2c_nack_config {
    LPI2C_NACK_RECEIVE    = 0U,  /*!< Receive ACK and NACK normally */
    LPI2C_NACK_IGNORE     = 1U,  /*!< Treat a received NACK as if it was an ACK */
} lpi2c_nack_config_t;

/*! @brief LPI2C master prescaler options */
typedef enum lpi2c_master_prescaler {
    LPI2C_MASTER_PRESC_DIV_1    = 0U,  /*!< Divide by 1   */
    LPI2C_MASTER_PRESC_DIV_2    = 1U,  /*!< Divide by 2   */
    LPI2C_MASTER_PRESC_DIV_4    = 2U,  /*!< Divide by 4   */
    LPI2C_MASTER_PRESC_DIV_8    = 3U,  /*!< Divide by 8   */
    LPI2C_MASTER_PRESC_DIV_16   = 4U,  /*!< Divide by 16  */
    LPI2C_MASTER_PRESC_DIV_32   = 5U,  /*!< Divide by 32  */
    LPI2C_MASTER_PRESC_DIV_64   = 6U,  /*!< Divide by 64  */
    LPI2C_MASTER_PRESC_DIV_128  = 7U,  /*!< Divide by 128 */
} lpi2c_master_prescaler_t;

/*! @brief LPI2C master commands */
typedef enum lpi2c_master_command {
    LPI2C_MASTER_COMMAND_TRANSMIT         = 0U,  /*!< Transmit DATA[7:0] */
    LPI2C_MASTER_COMMAND_RECEIVE          = 1U,  /*!< Receive (DATA[7:0] + 1) bytes */
    LPI2C_MASTER_COMMAND_STOP             = 2U,  /*!< Generate STOP condition */
    LPI2C_MASTER_COMMAND_RECEIVE_DISCARD  = 3U,  /*!< Receive and discard (DATA[7:0] + 1) bytes */
    LPI2C_MASTER_COMMAND_START            = 4U,  /*!< Generate START and transmit address in DATA[7:0] */
    LPI2C_MASTER_COMMAND_START_NACK       = 5U,  /*!< Generate START and transmit address in DATA[7:0], expect a NACK to be returned */
    LPI2C_MASTER_COMMAND_START_HS         = 6U,  /*!< Generate START and transmit address in DATA[7:0] in high speed mode */
    LPI2C_MASTER_COMMAND_START_NACK_HS    = 7U,  /*!< Generate START and transmit address in DATA[7:0] in high speed mode, expect a NACK to be returned */
} lpi2c_master_command_t;

/*! @brief Slave address configuration */
typedef enum lpi2c_slave_addr_config {
    LPI2C_SLAVE_ADDR_MATCH_0_7BIT              = 0U,  /*!< Address match 0 (7-bit) */
    LPI2C_SLAVE_ADDR_MATCH_0_10BIT             = 1U,  /*!< Address match 0 (10-bit) */
    LPI2C_SLAVE_ADDR_MATCH_0_7BIT_OR_1_7BIT    = 2U,  /*!< Address match 0 (7-bit) or Address match 1 (7-bit) */
    LPI2C_SLAVE_ADDR_MATCH_0_10BIT_OR_1_10BIT  = 3U,  /*!< Address match 0 (10-bit) or Address match 1 (10-bit) */
    LPI2C_SLAVE_ADDR_MATCH_0_7BIT_OR_1_10BIT   = 4U,  /*!< Address match 0 (7-bit) or Address match 1 (10-bit) */
    LPI2C_SLAVE_ADDR_MATCH_0_10BIT_OR_1_7BIT   = 5U,  /*!< Address match 0 (10-bit) or Address match 1 (7-bit) */
    LPI2C_SLAVE_ADDR_MATCH_RANGE_7BIT          = 6U,  /*!< From Address match 0 (7-bit) to Address match 1 (7-bit) */
    LPI2C_SLAVE_ADDR_MATCH_RANGE_10BIT         = 7U,  /*!< From Address match 0 (10-bit) to Address match 1 (10-bit) */
} lpi2c_slave_addr_config_t;

/*! @brief Slave NACK reaction configuration */
typedef enum lpi2c_slave_nack_config {
    LPI2C_SLAVE_NACK_END_TRANSFER       = 0U,  /*!< Slave will end transfer when NACK detected */
    LPI2C_SLAVE_NACK_CONTINUE_TRANSFER  = 1U,  /*!< Slave will not end transfer when NACK detected */
} lpi2c_slave_nack_config_t;

/*! @brief Slave receive data register function configuration */
typedef enum lpi2c_slave_rxdata_config {
    LPI2C_SLAVE_RXDATA_DATA_ONLY     = 0U,  /*!< Reading the receive data register will return only data */
    LPI2C_SLAVE_RXDATA_DATA_OR_ADDR  = 1U,  /*!< Reading the receive data register can return data or address */
} lpi2c_slave_rxdata_config_t;

/*! @brief Slave Transmit Data Flag function control */
typedef enum lpi2c_slave_txflag_config {
    LPI2C_SLAVE_TXFLAG_TRANSFER_ONLY  = 0U,  /*!< only assert during a slave-transmit transfer */
    LPI2C_SLAVE_TXFLAG_ALWAYS         = 1U,  /*!< assert whenever the transmit data register is empty */
} lpi2c_slave_txflag_config_t;

/*! @brief Slave received address validity */
typedef enum lpi2c_slave_addr_valid {
    LPI2C_SLAVE_ADDR_VALID      = 0U,  /*!< RADDR is valid */
    LPI2C_SLAVE_ADDR_NOT_VALID  = 1U,  /*!< RADDR is not valid */
} lpi2c_slave_addr_valid_t;

/*! @brief Slave ACK transmission options */
typedef enum lpi2c_slave_nack_transmit {
    LPI2C_SLAVE_TRANSMIT_ACK   = 0U,  /*!< Transmit ACK for received word  */
    LPI2C_SLAVE_TRANSMIT_NACK  = 1U,  /*!< Transmit NACK for received word */
} lpi2c_slave_nack_transmit_t;


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
 * @brief Get the version of the LPI2C module
 * 
 * This function reads the version number of the LPI2C hardware module
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param versionInfo  Device Version Number
 */
static inline void  LPI2C_HAL_GetVersion(LPI2C_Type *baseAddr, lpi2c_version_info_t *versionInfo)
{
    uint32_t tmp = baseAddr->VERID;
    versionInfo->majorNumber = (tmp & LPI2C_VERID_MAJOR_MASK) >> LPI2C_VERID_MAJOR_SHIFT;
    versionInfo->minorNumber = (tmp & LPI2C_VERID_MINOR_MASK) >> LPI2C_VERID_MINOR_SHIFT;
    versionInfo->featureNumber = (tmp & LPI2C_VERID_FEATURE_MASK) >> LPI2C_VERID_FEATURE_SHIFT;
    return;
}


/*!
 * @brief Get the size of the Master Receive FIFO
 * 
 * This function returns the size of the Master Receive FIFO, always a power of 2.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  Master Receive FIFO Size
 */
static inline uint16_t  LPI2C_HAL_GetMasterRxFIFOSize(LPI2C_Type *baseAddr)
{
    uint32_t tmp = baseAddr->PARAM;
    tmp = (tmp & LPI2C_PARAM_MRXFIFO_MASK) >> LPI2C_PARAM_MRXFIFO_SHIFT;
    tmp = 1U << tmp;     /* RX FIFO size = 2^MRXFIFO */
    return (uint16_t)tmp;
}


/*!
 * @brief Get the size of the Master Transmit FIFO
 * 
 * This function returns the size of the Master Transmit FIFO, always a power of 2.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  Master Transmit FIFO Size
 */
static inline uint16_t  LPI2C_HAL_GetMasterTxFIFOSize(LPI2C_Type *baseAddr)
{
    uint32_t tmp = baseAddr->PARAM;
    tmp = (tmp & LPI2C_PARAM_MTXFIFO_MASK) >> LPI2C_PARAM_MTXFIFO_SHIFT;
    tmp = 1U << tmp;      /* TX FIFO size = 2^MTXFIFO */
    return (uint16_t)tmp;
}


/*!
 * @brief Reset the master receive FIFO
 * 
 * This function empties the receive FIFO of the LPI2C master.
 * 
 * @param baseAddr  base address of the LPI2C module
 */
static inline void LPI2C_HAL_MasterRxFIFOResetCmd(LPI2C_Type *baseAddr)
{
    BITBAND_ACCESS32(&(baseAddr->MCR), LPI2C_MCR_RRF_SHIFT) = (uint32_t)1u;
}


/*!
 * @brief Reset the master transmit FIFO
 * 
 * This function empties the transmit FIFO of the LPI2C master. 
 * 
 * @param baseAddr  base address of the LPI2C module
 */
static inline void LPI2C_HAL_MasterTxFIFOResetCmd(LPI2C_Type *baseAddr)
{
    BITBAND_ACCESS32(&(baseAddr->MCR), LPI2C_MCR_RTF_SHIFT) = (uint32_t)1u;
}


/*!
 * @brief Set the master behaviour in Debug mode 
 * 
 * This function enables of disables master functionality when the CPU is in debug mode.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param enable  specifies whether to enable or disable the LPI2C master in debug mode
 */
static inline void LPI2C_HAL_MasterSetDebugMode(LPI2C_Type *baseAddr, bool enable)
{
    BITBAND_ACCESS32(&(baseAddr->MCR), LPI2C_MCR_DBGEN_SHIFT) = (uint32_t)enable;
}


/*!
 * @brief Set the master behaviour in Doze mode 
 * 
 * This function enables of disables master functionality in doze mode.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param enable  specifies whether to enable or disable the LPI2C master in doze mode
 */
static inline void LPI2C_HAL_MasterSetDozeMode(LPI2C_Type *baseAddr, bool enable)
{
    BITBAND_ACCESS32(&(baseAddr->MCR), LPI2C_MCR_DOZEN_SHIFT) = (uint32_t)enable;
}


/*!
 * @brief Set/clear the master reset command
 *
 * Calling this function with enable parameter set to true resets all internal 
 * master logic and registers, except the Master Control Register. The reset state 
 * persists until this function is called with enable parameter set to false.
 *
 * @param baseAddr  base address of the LPI2C module
 * @param enable  specifies the reset state of the LPI2C master logic
 */
static inline void LPI2C_HAL_MasterSetSoftwareReset(LPI2C_Type *baseAddr, bool enable)
{
    BITBAND_ACCESS32(&(baseAddr->MCR), LPI2C_MCR_RST_SHIFT) = (uint32_t)enable;
}


/*!
 * @brief Enable or disable the LPI2C master
 * 
 * This function enables or disables the LPI2C module in master mode. If the module 
 * is enabled, the transmit FIFO  is not empty and the I2C bus is idle, then 
 * the LPI2C master will immediately initiate a transfer on the I2C bus.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param enable  specifies whether to enable or disable the LPI2C master
 */
static inline void LPI2C_HAL_MasterSetEnable(LPI2C_Type *baseAddr, bool enable)
{
    BITBAND_ACCESS32(&(baseAddr->MCR), LPI2C_MCR_MEN_SHIFT) = (uint32_t)enable;
}


/*!
 * @brief Return the debug mode setting for the LPI2C master
 * 
 * This function returns the current debug mode setting for the LPI2C master.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  the debug mode setting for the LPI2C master
 */
static inline bool LPI2C_HAL_MasterGetDebugMode(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->MCR), LPI2C_MCR_DBGEN_SHIFT);
}


/*!
 * @brief Return the doze mode setting for the LPI2C master
 * 
 * This function returns the current doze mode setting for the LPI2C master
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  the doze mode setting for the LPI2C master
 */
static inline bool LPI2C_HAL_MasterGetDozeMode(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->MCR), LPI2C_MCR_DOZEN_SHIFT);
}


/*!
 * @brief Return the reset setting for the LPI2C master
 * 
 * This function returns the state of the LPI2C master software reset bit.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  the reset setting for the LPI2C master
 */
static inline bool LPI2C_HAL_MasterGetSoftwareReset(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->MCR), LPI2C_MCR_RST_SHIFT);
}


/*!
 * @brief Return the enable/disable setting for the LPI2C master
 * 
 * This function checks whether or not the LPI2C master is enabled.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  the enable/disable setting for the LPI2C master
 */
static inline bool LPI2C_HAL_MasterGetEnable(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->MCR), LPI2C_MCR_MEN_SHIFT);
}


/*!
 * @brief Return the idle/busy state of the I2C bus
 * 
 * This function returns true if the I2C bus is busy and false if the bus is idle.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  the idle/busy state of the I2C bus
 */
static inline bool LPI2C_HAL_MasterGetBusBusyEvent(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->MSR), LPI2C_MSR_BBF_SHIFT);
}


/*!
 * @brief Return the idle/busy state of the LPI2C master
 * 
 * This function returns true if the LPI2C master is busy and false if 
 * the LPI2C master is idle.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  the idle/busy state of the LPI2C master
 */
static inline bool LPI2C_HAL_MasterGetMasterBusyEvent(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->MSR), LPI2C_MSR_MBF_SHIFT);
}


/*!
 * @brief Indicate the availability of receive data
 * 
 * This function returns true when the number of words in the receive FIFO is greater 
 * than the receive FIFO watermark. See function LPI2C_HAL_MasterSetRxFIFOWatermark()
 * for configuring the receive FIFO watermark.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  receive data ready/not ready
 */
static inline bool LPI2C_HAL_MasterGetReceiveDataReadyEvent(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->MSR), LPI2C_MSR_RDF_SHIFT);
}


/*!
 * @brief Indicate if the LPI2C master requests more data
 * 
 * This function returns true when the number of words in the transmit FIFO is equal 
 * or less than the transmit FIFO watermark. See function LPI2C_HAL_MasterSetTxFIFOWatermark()
 * for configuring the transmit FIFO watermark.
 *
 * @param baseAddr  base address of the LPI2C module
 * @return  transmit data requested/not requested
 */
static inline bool LPI2C_HAL_MasterGetTransmitDataRequestEvent(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->MSR), LPI2C_MSR_TDF_SHIFT);
}


/*!
 * @brief Check the occurrence of a data match event
 * 
 * This function returns true if there was a match on the received data according
 * to the current data match option. Received data that is discarded due to commands
 * does not cause this flag to set. See function LPI2C_HAL_MasterSetMatchConfig()
 * for more information on data match configuration.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  indication of a data match event
 */
static inline bool LPI2C_HAL_MasterGetDataMatchEvent(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->MSR), LPI2C_MSR_DMF_SHIFT);
}


/*!
 * @brief Check the occurrence of a pin low timeout event
 * 
 * This function returns true if SCL and/or SDA input is low for more than PINLOW cycles
 * This event can occur even when the LPI2C master is idle.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  indication of a pin low timeout event
 */
static inline bool LPI2C_HAL_MasterGetPinLowTimeoutEvent(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->MSR), LPI2C_MSR_PLTF_SHIFT);
}


/*!
 * @brief Check the occurrence of a FIFO error event
 * 
 * This function returns true if the LPI2C master detects an attempt to send or 
 * receive data without first generating a (repeated) START condition. This can 
 * occur if the transmit FIFO underflows when the AUTOSTOP bit is set. When this 
 * flag is set, the LPI2C master will send a STOP condition (if busy) and will 
 * not initiate a new START condition until this flag has been cleared.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  indication of a FIFO error event
 */
static inline bool LPI2C_HAL_MasterGetFIFOErrorEvent(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->MSR), LPI2C_MSR_FEF_SHIFT);
}


/*!
 * @brief Check the occurrence of an arbitration lost event
 * 
 * This function returns true if the LPI2C master detects an arbitration lost
 * condition, as defined by the I2C standard. When this flag sets, the LPI2C 
 * master will release the bus (go idle) and will not initiate a new START 
 * condition until this flag has been cleared.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  indication of an arbitration lost event
 */
static inline bool LPI2C_HAL_MasterGetArbitrationLostEvent(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->MSR), LPI2C_MSR_ALF_SHIFT);
}


/*!
 * @brief Check the occurrence of an unexpected NACK event
 * 
 * This function returns true if the LPI2C master detects a NACK when 
 * transmitting an address or data. If a NACK is expected for a given address 
 * (as configured by the command word) then the flag will set if a NACK is not
 * generated. When set, the master will transmit a STOP condition and will not 
 * initiate a new START condition until this flag has been cleared.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  indication of an unexpected NACK event
 */
static inline bool LPI2C_HAL_MasterGetNACKDetectEvent(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->MSR), LPI2C_MSR_NDF_SHIFT);
}


/*!
 * @brief Check the occurrence of a STOP detect event
 * 
 * This function returns true if the LPI2C master has generated a STOP condition.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  indication of a STOP detect event
 */
static inline bool LPI2C_HAL_MasterGetSTOPDetectEvent(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->MSR), LPI2C_MSR_SDF_SHIFT);
}


/*!
 * @brief Check the occurrence of an end packet event
 * 
 * This function returns true if the LPI2C master has generated a 
 * repeated START or a STOP condition. It does not return true when 
 * the master first generates a START condition.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  indication of an end packet event
 */
static inline bool LPI2C_HAL_MasterGetEndPacketEvent(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->MSR), LPI2C_MSR_EPF_SHIFT);
}


/*!
 * @brief Clear the data match event flag
 * 
 * This function clears the data match event.
 *
 * @param baseAddr  base address of the LPI2C module
 */
static inline void LPI2C_HAL_MasterClearDataMatchEvent(LPI2C_Type *baseAddr)
{
    baseAddr->MSR = (uint32_t)(1u << LPI2C_MSR_DMF_SHIFT);
}


/*!
 * @brief Clear the pin low timeout event flag
 * 
 * This function clears the pin low timeout event. This event cannot be cleared for
 * as long as the pin low timeout condition continues, and must be cleared before 
 * the LPI2C master can initiate a START condition.
 * 
 * @param baseAddr  base address of the LPI2C module
 */
static inline void LPI2C_HAL_MasterClearPinLowTimeoutEvent(LPI2C_Type *baseAddr)
{
    baseAddr->MSR = (uint32_t)(1u << LPI2C_MSR_PLTF_SHIFT);
}


/*!
 * @brief Clear the FIFO error event flag
 * 
 * This function clears the FIFO error event. This event must be cleared before 
 * the LPI2C master can initiate a START condition.
 * 
 * @param baseAddr  base address of the LPI2C module
 */
static inline void LPI2C_HAL_MasterClearFIFOErrorEvent(LPI2C_Type *baseAddr)
{
    baseAddr->MSR = (uint32_t)(1u << LPI2C_MSR_FEF_SHIFT);
}


/*!
 * @brief Clear the arbitration lost event flag
 * 
 * This function clears the arbitration lost event. This event must be cleared 
 * before the LPI2C master can initiate a START condition.
 * 
 * @param baseAddr  base address of the LPI2C module
 */
static inline void LPI2C_HAL_MasterClearArbitrationLostEvent(LPI2C_Type *baseAddr)
{
    baseAddr->MSR = (uint32_t)(1u << LPI2C_MSR_ALF_SHIFT);
}


/*!
 * @brief Clear the unexpected NACK event flag
 * 
 * This function clears the unexpected NACK event. This event must be cleared 
 * before the LPI2C master can initiate a START condition.
 * 
 * @param baseAddr  base address of the LPI2C module
 */
static inline void LPI2C_HAL_MasterClearNACKDetectEvent(LPI2C_Type *baseAddr)
{
    baseAddr->MSR = (uint32_t)(1u << LPI2C_MSR_NDF_SHIFT);
}


/*!
 * @brief Clear the STOP detect event flag
 * 
 * This function clears the STOP detect event.
 * 
 * @param baseAddr  base address of the LPI2C module
 */
static inline void LPI2C_HAL_MasterClearSTOPDetectEvent(LPI2C_Type *baseAddr)
{
    baseAddr->MSR = (uint32_t)(1u << LPI2C_MSR_SDF_SHIFT);
}


/*!
 * @brief Clear the end packet event flag
 * 
 * This function clears the end packet event.
 * 
 * @param baseAddr  base address of the LPI2C module
 */
static inline void LPI2C_HAL_MasterClearEndPacketEvent(LPI2C_Type *baseAddr)
{
    baseAddr->MSR = (uint32_t)(1u << LPI2C_MSR_EPF_SHIFT);
}


/*!
 * @brief Enable/disable receive data DMA requests
 * 
 * This function enables or disables generation of Rx DMA requests when data
 * can be read from the receive FIFO, as configured by the receive FIFO watermark.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param enable  specifies whether to enable or disable DMA requests
 */
static inline void LPI2C_HAL_MasterSetRxDMA(LPI2C_Type *baseAddr, bool enable)
{
    BITBAND_ACCESS32(&(baseAddr->MDER), LPI2C_MDER_RDDE_SHIFT) = (uint32_t)enable;
}


/*!
 * @brief Enable/disable transmit data DMA requests
 * 
 * This function enables or disables generation of Tx DMA requests when data
 * can be written to the transmit FIFO, as configured by the transmit FIFO watermark.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param enable  specifies whether to enable or disable DMA requests
 */
static inline void LPI2C_HAL_MasterSetTxDMA(LPI2C_Type *baseAddr, bool enable)
{
    BITBAND_ACCESS32(&(baseAddr->MDER), LPI2C_MDER_TDDE_SHIFT) = (uint32_t)enable;
}


/*!
 * @brief Check if receive data DMA requests are enabled
 * 
 * This function returns true if Rx DMA requests are enabled.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  enabled/disabled status of receive data DMA requests
 */
static inline bool LPI2C_HAL_MasterGetRxDMA(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->MDER), LPI2C_MDER_RDDE_SHIFT);
}


/*!
 * @brief Check if transmit data DMA requests are enabled
 * 
 * This function returns true if Tx DMA requests are enabled.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  enabled/disabled status of transmit data DMA requests
 */
static inline bool LPI2C_HAL_MasterGetTxDMA(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->MDER), LPI2C_MDER_TDDE_SHIFT);
}


/*!
 * @brief Enable or disable specified LPI2C master interrupts
 * 
 * This function can enable or disable one or more master interrupt sources 
 * specified by the interrupts parameter.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param interrupts  interrupts to be enabled or disabled; 
 *  must be a bitwise or between one or more of the following constants: 
 *  - LPI2C_HAL_MASTER_DATA_MATCH_INT          - Data Match Interrupt
 *  - LPI2C_HAL_MASTER_PIN_LOW_TIMEOUT_INT     - Pin Low Timeout Interrupt
 *  - LPI2C_HAL_MASTER_FIFO_ERROR_INT          - FIFO Error Interrupt
 *  - LPI2C_HAL_MASTER_ARBITRATION_LOST_INT    - Arbitration Lost Interrupt
 *  - LPI2C_HAL_MASTER_NACK_DETECT_INT         - NACK Detect Interrupt
 *  - LPI2C_HAL_MASTER_STOP_DETECT_INT         - STOP Detect Interrupt
 *  - LPI2C_HAL_MASTER_END_PACKET_INT          - End Packet Interrupt
 *  - LPI2C_HAL_MASTER_RECEIVE_DATA_INT        - Receive Data Interrupt
 *  - LPI2C_HAL_MASTER_TRANSMIT_DATA_INT       - Transmit Data Interrupt
 * @param enable  specifies whether to enable or disable specified interrupts
 */
static inline void LPI2C_HAL_MasterSetInt(LPI2C_Type *baseAddr, uint32_t interrupts, bool enable)
{
    uint32_t tmp = baseAddr->MIER;

    if (enable == true)
    {
        tmp |= interrupts;
    }
    else
    {
        tmp &= ~interrupts;
    }
    baseAddr->MIER = tmp;
}


/*!
 * @brief Return the state of the specified LPI2C master interrupt
 * 
 * This function returns the enabled/disabled state of the master interrupt 
 * source specified by the interrupt parameter.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param interrupts  interrupt for which the check is made;
 *  must be one of the following constants:
 *  - LPI2C_HAL_MASTER_DATA_MATCH_INT          - Data Match Interrupt
 *  - LPI2C_HAL_MASTER_PIN_LOW_TIMEOUT_INT     - Pin Low Timeout Interrupt
 *  - LPI2C_HAL_MASTER_FIFO_ERROR_INT          - FIFO Error Interrupt
 *  - LPI2C_HAL_MASTER_ARBITRATION_LOST_INT    - Arbitration Lost Interrupt
 *  - LPI2C_HAL_MASTER_NACK_DETECT_INT         - NACK Detect Interrupt
 *  - LPI2C_HAL_MASTER_STOP_DETECT_INT         - STOP Detect Interrupt
 *  - LPI2C_HAL_MASTER_END_PACKET_INT          - End Packet Interrupt
 *  - LPI2C_HAL_MASTER_RECEIVE_DATA_INT        - Receive Data Interrupt
 *  - LPI2C_HAL_MASTER_TRANSMIT_DATA_INT       - Transmit Data Interrupt
 * @return  enable/disable state of specified interrupt
 */
static inline bool LPI2C_HAL_MasterGetInt(LPI2C_Type *baseAddr, uint32_t interrupts)
{
    uint32_t tmp = baseAddr->MIER;

    if (tmp | interrupts)
    {
        return true;
    }
    else
    {
        return false;
    }
}


/*!
 * @brief Control the discarding of data that does not match the configured criteria
 * 
 * This function configures the policy for handling data that does not match the configuration 
 * criteria. Received data can be stored in the receive FIFO either unconditionally or based 
 * on the data match event. See function LPI2C_HAL_MasterGetDataMatchEvent() for more information on the 
 * data match event.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param rxDataMatch  specifies whether or not to drop non-matching data
 */
static inline void LPI2C_HAL_MasterSetRxDataMatch(LPI2C_Type *baseAddr, lpi2c_rx_data_match_t rxDataMatch)
{
    BITBAND_ACCESS32(&(baseAddr->MCFGR0), LPI2C_MCFGR0_RDMO_SHIFT) = (uint32_t)rxDataMatch;
}


/*!
 * @brief Return the current data match discarding policy
 * 
 * This function returns the currently configured policy for data that does not 
 * match the configured criteria
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  the current policy for non-matching data
 */
static inline lpi2c_rx_data_match_t LPI2C_HAL_MasterGetRxDataMatch(LPI2C_Type *baseAddr)
{
    return (lpi2c_rx_data_match_t)BITBAND_ACCESS32(&(baseAddr->MCFGR0), LPI2C_MCFGR0_RDMO_SHIFT);
}


/*!
 * @brief Set the circular FIFO mode for the transmit FIFO
 * 
 * This function enables or disables the circular FIFO feature of the module. When enabled, 
 * the transmit FIFO read pointer is saved to a temporary register. The transmit FIFO will 
 * be emptied as normal, but once the LPI2C master is idle and the transmit FIFO is empty, 
 * then the read pointer value will be restored from the temporary register. This will 
 * cause the contents of the transmit FIFO to be cycled through repeatedly. If AUTOSTOP 
 * is set, a STOP condition will be sent whenever the transmit FIFO is empty and the read 
 * pointer is restored.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param enable  enable/disable circular FIFO mode for the transmit FIFO
 */
static inline void LPI2C_HAL_MasterSetCircularFIFO(LPI2C_Type *baseAddr, bool enable)
{
    BITBAND_ACCESS32(&(baseAddr->MCFGR0), LPI2C_MCFGR0_CIRFIFO_SHIFT) = (uint32_t)enable;
}


/*!
 * @brief Return the circular FIFO mode for the transmit FIFO
 * 
 * This function returns the current setting for the circular FIFO feature of the module.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  circular FIFO mode for the transmit FIFO
 */
static inline bool LPI2C_HAL_MasterGetCircularFIFO(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->MCFGR0), LPI2C_MCFGR0_CIRFIFO_SHIFT);
}


/*!
 * @brief Set the source of the host request input
 * 
 * This function selects the source of the host request input.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param source  selected source of the host request input
 */
static inline void LPI2C_HAL_MasterSetHreqSelect(LPI2C_Type *baseAddr, lpi2c_hreq_source_t source)
{
    BITBAND_ACCESS32(&(baseAddr->MCFGR0), LPI2C_MCFGR0_HRSEL_SHIFT) = (uint32_t)source;
}


/*!
 * @brief Set the polarity of the host request input pin
 * 
 * This function configures the polarity of the host request input pin.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param polarity  selected polarity of the host request input
 */
static inline void LPI2C_HAL_MasterSetHreqPolarity(LPI2C_Type *baseAddr, lpi2c_hreq_polarity_t polarity)
{
    BITBAND_ACCESS32(&(baseAddr->MCFGR0), LPI2C_MCFGR0_HRPOL_SHIFT) = (uint32_t)polarity;
}


/*!
 * @brief Enable/disable the host request feature
 * 
 * This function enables or disables the host request signal. When enabled, the LPI2C 
 * master will only initiate a START condition if the host request input is asserted and 
 * the bus is idle. A repeated START is not affected by the host request.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param enable  enable/disable the host request feature
 */
static inline void LPI2C_HAL_MasterSetHreqEnable(LPI2C_Type *baseAddr, bool enable)
{
    BITBAND_ACCESS32(&(baseAddr->MCFGR0), LPI2C_MCFGR0_HREN_SHIFT) = (uint32_t)enable;
}


/*!
 * @brief Return the source of the host request input
 * 
 * This function returns the currently configured source for the host request input.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  source of the host request input
 */
static inline lpi2c_hreq_source_t LPI2C_HAL_MasterGetHreqSelect(LPI2C_Type *baseAddr)
{
    return (lpi2c_hreq_source_t)BITBAND_ACCESS32(&(baseAddr->MCFGR0), LPI2C_MCFGR0_HRSEL_SHIFT);
}


/*!
 * @brief Return the polarity of the host request input pin
 * 
 * This function returns the currently configured polarity for the host request input.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  polarity of the host request input
 */
static inline lpi2c_hreq_polarity_t LPI2C_HAL_MasterGetHreqPolarity(LPI2C_Type *baseAddr)
{
    return (lpi2c_hreq_polarity_t)BITBAND_ACCESS32(&(baseAddr->MCFGR0), LPI2C_MCFGR0_HRPOL_SHIFT);
}


/*!
 * @brief Return the enable/disable state the host request feature
 * 
 * This function returns true if the host request signal is enabled, and false if it is disabled.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  enable/disable state the host request feature
 */
static inline bool LPI2C_HAL_MasterGetHreqEnable(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->MCFGR0), LPI2C_MCFGR0_HREN_SHIFT);
}


/*!
 * @brief Set the pin mode of the module
 * 
 * This function sets the pin mode of the module. See type lpi2c_pin_config_t for 
 * a description of available modes.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param configuration  pin mode of the module
 */
static inline void LPI2C_HAL_MasterSetPinConfig(LPI2C_Type *baseAddr, lpi2c_pin_config_t configuration)
{
    uint32_t tmp = baseAddr->MCFGR1;
    tmp &= ~(LPI2C_MCFGR1_PINCFG_MASK);
    tmp |= LPI2C_MCFGR1_PINCFG(configuration);
    baseAddr->MCFGR1 = tmp;
}


/*!
 * @brief Return the pin mode of the module
 * 
 * This function returns the currently configured pin mode for the module.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  pin mode of the module
 */
static inline lpi2c_pin_config_t LPI2C_HAL_MasterGetPinConfig(LPI2C_Type *baseAddr)
{
    uint32_t tmp = baseAddr->MCFGR1;
    tmp = (tmp & LPI2C_MCFGR1_PINCFG_MASK) >> LPI2C_MCFGR1_PINCFG_SHIFT;
    return (lpi2c_pin_config_t)tmp;
}


/*!
 * @brief Set the match mode of the module
 * 
 * This function configures the rules for matching incoming data against the two match 
 * values, MATCH0 and MATCH1. A successful match will trigger a data match event. 
 * See type lpi2c_match_config_t for a description of all available match options.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param configuration  match mode of the module
 */
static inline void LPI2C_HAL_MasterSetMatchConfig(LPI2C_Type *baseAddr, lpi2c_match_config_t configuration)
{
    uint32_t tmp = baseAddr->MCFGR1;
    tmp &= ~(LPI2C_MCFGR1_MATCFG_MASK);
    tmp |= LPI2C_MCFGR1_MATCFG(configuration);
    baseAddr->MCFGR1 = tmp;
}


/*!
 * @brief Return the match mode of the module
 * 
 * This function returns the currently configured match mode for the module.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  match mode of the module
 */
static inline lpi2c_match_config_t LPI2C_HAL_MasterGetMatchConfig(LPI2C_Type *baseAddr)
{
    uint32_t tmp = baseAddr->MCFGR1;
    tmp = (tmp & LPI2C_MCFGR1_MATCFG_MASK) >> LPI2C_MCFGR1_MATCFG_SHIFT;
    return (lpi2c_match_config_t)tmp;
}


/*!
 * @brief Set the timeout configuration of the module
 * 
 * This function configures the condition for triggering a pin low timeout event. 
 * Selects between monitoring only SCL for timeout, or both SCL and SDA.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param configuration  timeout configuration of the module
 */
static inline void LPI2C_HAL_MasterSetTimeoutConfig(LPI2C_Type *baseAddr, lpi2c_timeout_config_t configuration)
{
    BITBAND_ACCESS32(&(baseAddr->MCFGR1), LPI2C_MCFGR1_TIMECFG_SHIFT) = (uint32_t)configuration;
}


/*!
 * @brief Return the timeout configuration of the module
 * 
 * This function returns the current pin low timeout configuration for the module.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  timeout configuration of the module
 */
static inline lpi2c_timeout_config_t LPI2C_HAL_MasterGetTimeoutConfig(LPI2C_Type *baseAddr)
{
    return (lpi2c_timeout_config_t)BITBAND_ACCESS32(&(baseAddr->MCFGR1), LPI2C_MCFGR1_TIMECFG_SHIFT);
}


/*!
 * @brief Configure the reaction of the module on NACK reception
 * 
 * This function configures how the LPI2C master reacts when receiving a NACK. NACK responses can 
 * be treated normally or ignored. In Ultra-Fast mode it is necessary to configure the module to 
 * ignore NACK responses.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param configuration  set reaction of the module on NACK reception
 */
static inline void LPI2C_HAL_MasterSetNACKConfig(LPI2C_Type *baseAddr, lpi2c_nack_config_t configuration)
{
    BITBAND_ACCESS32(&(baseAddr->MCFGR1), LPI2C_MCFGR1_IGNACK_SHIFT) = (uint32_t)configuration;
}


/*!
 * @brief Return the reaction of the module on NACK reception
 * 
 * This function returns the currently configured setting for handling NACK reception.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  configured reaction of the module on NACK reception
 */
static inline lpi2c_nack_config_t LPI2C_HAL_MasterGetNACKConfig(LPI2C_Type *baseAddr)
{
    return (lpi2c_nack_config_t)BITBAND_ACCESS32(&(baseAddr->MCFGR1), LPI2C_MCFGR1_IGNACK_SHIFT);
}


/*!
 * @brief Configure the automatic generation of STOP condition
 * 
 * This function can enable or disable the automatic generation of STOP condition. When enabled, 
 * a STOP condition is generated whenever the LPI2C master is busy and the transmit FIFO is 
 * empty. The STOP condition can also be generated using a transmit FIFO command.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param enable  enable/disable automatic generation of STOP condition
 */
static inline void LPI2C_HAL_MasterSetAutoStopConfig(LPI2C_Type *baseAddr, bool enable)
{
    BITBAND_ACCESS32(&(baseAddr->MCFGR1), LPI2C_MCFGR1_AUTOSTOP_SHIFT) = (uint32_t)enable;
}


/*!
 * @brief Return the current setting for automatic generation of STOP condition
 * 
 * This function returns the currently configured setting for automatic generation of 
 * STOP condition.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  current setting for automatic generation of STOP condition
 */
static inline bool LPI2C_HAL_MasterGetAutoStopConfig(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->MCFGR1), LPI2C_MCFGR1_AUTOSTOP_SHIFT);
}


/*!
 * @brief Configure the LPI2C master prescaler
 * 
 * This function configures the clock prescaler used for all LPI2C master logic, 
 * except the digital glitch filters.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param prescaler  LPI2C master prescaler
 */
static inline void LPI2C_HAL_MasterSetPrescaler(LPI2C_Type *baseAddr, lpi2c_master_prescaler_t prescaler)
{
    uint32_t tmp = baseAddr->MCFGR1;
    tmp &= ~(LPI2C_MCFGR1_PRESCALE_MASK);
    tmp |= LPI2C_MCFGR1_PRESCALE(prescaler);
    baseAddr->MCFGR1 = tmp;
}


/*!
 * @brief Return the LPI2C master prescaler
 * 
 * This function returns the currently configured clock prescaler.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  LPI2C master prescaler
 */
static inline lpi2c_master_prescaler_t LPI2C_HAL_MasterGetPrescaler(LPI2C_Type *baseAddr)
{
    uint32_t tmp = baseAddr->MCFGR1;
    tmp = (tmp & LPI2C_MCFGR1_PRESCALE_MASK) >> LPI2C_MCFGR1_PRESCALE_SHIFT;
    return (lpi2c_master_prescaler_t)tmp;
}


/*!
 * @brief Configure the LPI2C SDA glitch filter
 * 
 * This function configures the I2C master digital glitch filters for SDA input. 
 * A configuration of 0 will disable the glitch filter. Glitches equal to or less than 
 * FILTSDA cycles long will be filtered out and ignored. The latency through the glitch 
 * filter is equal to FILTSDA cycles and must be configured less than the minimum SCL 
 * low or high period.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param cycles  number of cycles for the LPI2C SDA glitch filter
 */
static inline void LPI2C_HAL_MasterSetSDAGlitchFilter(LPI2C_Type *baseAddr, uint8_t cycles)
{
    uint32_t tmp = baseAddr->MCFGR2;
    tmp &= ~(LPI2C_MCFGR2_FILTSDA_MASK);
    tmp |= LPI2C_MCFGR2_FILTSDA(cycles);
    baseAddr->MCFGR2 = tmp;
}


/*!
 * @brief Return the LPI2C SDA glitch filter configuration
 * 
 * This function returns the currently configured master SDA glitch filter.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  number of cycles for the LPI2C SDA glitch filter
 */
static inline uint8_t LPI2C_HAL_MasterGetSDAGlitchFilter(LPI2C_Type *baseAddr)
{
    uint32_t tmp = baseAddr->MCFGR2;
    tmp = (tmp & LPI2C_MCFGR2_FILTSDA_MASK) >> LPI2C_MCFGR2_FILTSDA_SHIFT;
    return (uint8_t)tmp;
}


/*!
 * @brief Configure the LPI2C SCL glitch filter
 * 
 * This function configures the I2C master digital glitch filters for SCL input,
 * a configuration of 0 will disable the glitch filter. Glitches equal to or less 
 * than FILTSCL cycles long will be filtered out and ignored. The latency through 
 * the glitch filter is equal to FILTSCL cycles and must be configured less than 
 * the minimum SCL low or high period.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param cycles  number of cycles for the LPI2C SCL glitch filter
 */
static inline void LPI2C_HAL_MasterSetSCLGlitchFilter(LPI2C_Type *baseAddr, uint8_t cycles)
{
    uint32_t tmp = baseAddr->MCFGR2;
    tmp &= ~(LPI2C_MCFGR2_FILTSCL_MASK);
    tmp |= LPI2C_MCFGR2_FILTSCL(cycles);
    baseAddr->MCFGR2 = tmp;
}


/*!
 * @brief Return the LPI2C SCL glitch filter configuration
 * 
 * This function returns the currently configured SCL glitch filter.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  number of cycles for the LPI2C SCL glitch filter
 */
static inline uint8_t LPI2C_HAL_MasterGetSCLGlitchFilter(LPI2C_Type *baseAddr)
{
    uint32_t tmp = baseAddr->MCFGR2;
    tmp = (tmp & LPI2C_MCFGR2_FILTSCL_MASK) >> LPI2C_MCFGR2_FILTSCL_SHIFT;
    return (uint8_t)tmp;
}


/*!
 * @brief Configure the bus idle timeout
 *
 * This function configures the bus idle timeout period in clock cycles. If both 
 * SCL and SDA are high for longer than the configured timeout in cycles, then the 
 * I2C bus is assumed to be idle and the master can generate a START condition. 
 * When set to zero, this feature is disabled.
 *
 * @param baseAddr  base address of the LPI2C module
 * @param cycles  number of cycles for the bus idle timeout
 */
static inline void LPI2C_HAL_MasterSetBusIdleTimeout(LPI2C_Type *baseAddr, uint16_t cycles)
{
    uint32_t tmp = baseAddr->MCFGR2;
    tmp &= ~(LPI2C_MCFGR2_BUSIDLE_MASK);
    tmp |= LPI2C_MCFGR2_BUSIDLE(cycles);
    baseAddr->MCFGR2 = tmp;
}


/*!
 * @brief Return the bus idle timeout configuration
 * 
 * This function returns the currently configured bus idle timeout.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  number of cycles for the bus idle timeout
 */
static inline uint16_t LPI2C_HAL_MasterGetBusIdleTimeout(LPI2C_Type *baseAddr)
{
    uint32_t tmp = baseAddr->MCFGR2;
    tmp = (tmp & LPI2C_MCFGR2_BUSIDLE_MASK) >> LPI2C_MCFGR2_BUSIDLE_SHIFT;
    return (uint16_t)tmp;
}


/*!
 * @brief Configure the pin low timeout
 * 
 * This function configures the pin low timeout period in clock cycles. If SCL and/or SDA 
 * is low for longer than the pin low timeout cycles then PLTF is set. Timeout value will 
 * be truncated to a multiple of 256. When set to zero, this feature is disabled.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param cycles  number of cycles for the pin low timeout; will be truncated to a multiple of 256
 */
static inline void LPI2C_HAL_MasterSetPinLowTimeout(LPI2C_Type *baseAddr, uint32_t cycles)
{
    baseAddr->MCFGR3 = cycles;
}


/*!
 * @brief Return the pin low timeout configuration
 * 
 * This function returns the currently configured pin low timeout.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  number of cycles for the pin low timeout
 */
static inline uint32_t LPI2C_HAL_MasterGetPinLowTimeout(LPI2C_Type *baseAddr)
{
    return (uint32_t)(baseAddr->MCFGR3);
}


/*!
 * @brief Set the MATCH0 value for the data match feature
 * 
 * This function sets the MATCH0 value for comparing against the received data 
 * when receive data match is enabled. See function LPI2C_HAL_MasterSetMatchConfig()
 * for details on how this value is used.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param value  MATCH0 value
 */
static inline void LPI2C_HAL_MasterSetMatch0(LPI2C_Type *baseAddr, uint8_t value)
{
    uint32_t tmp = baseAddr->MDMR;
    tmp &= ~(LPI2C_MDMR_MATCH0_MASK);
    tmp |= LPI2C_MDMR_MATCH0(value);
    baseAddr->MDMR = tmp;
}


/*!
 * @brief Return the MATCH0 value for the data match feature
 * 
 * This function returns the currently configured value for MATCH0.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  MATCH0 value
 */
static inline uint8_t LPI2C_HAL_MasterGetMatch0(LPI2C_Type *baseAddr)
{
    uint32_t tmp = baseAddr->MDMR;
    tmp = (tmp & LPI2C_MDMR_MATCH0_MASK) >> LPI2C_MDMR_MATCH0_SHIFT;
    return (uint8_t)tmp;
}


/*!
 * @brief Set the MATCH1 value for the data match feature
 * 
 * This function sets the MATCH1 value for comparing against the received data 
 * when receive data match is enabled. See function LPI2C_HAL_MasterSetMatchConfig()
 * for details on how this value is used.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param value  MATCH1 value
 */
static inline void LPI2C_HAL_MasterSetMatch1(LPI2C_Type *baseAddr, uint8_t value)
{
    uint32_t tmp = baseAddr->MDMR;
    tmp &= ~(LPI2C_MDMR_MATCH1_MASK);
    tmp |= LPI2C_MDMR_MATCH1(value);
    baseAddr->MDMR = tmp;
}


/*!
 * @brief Return the MATCH1 value for the data match feature
 * 
 * This function returns the currently configured value for MATCH1.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  MATCH1 value
 */
static inline uint8_t LPI2C_HAL_MasterGetMatch1(LPI2C_Type *baseAddr)
{
    uint32_t tmp = baseAddr->MDMR;
    tmp = (tmp & LPI2C_MDMR_MATCH1_MASK) >> LPI2C_MDMR_MATCH1_SHIFT;
    return (uint8_t)tmp;
}


/*!
 * @brief Set the data hold time for SDA
 * 
 * This function sets the minimum number of cycles (minus one) that is used as the 
 * data hold time for SDA. Must be configured less than the minimum SCL low period.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param value  value of the data hold time for SDA
 */
static inline void LPI2C_HAL_MasterSetDataValidDelay(LPI2C_Type *baseAddr, uint8_t value)
{
    uint32_t tmp = baseAddr->MCCR0;
    tmp &= ~(LPI2C_MCCR0_DATAVD_MASK);
    tmp |= LPI2C_MCCR0_DATAVD(value);
    baseAddr->MCCR0 = tmp;
}


/*!
 * @brief Return the configured data hold time for SDA
 * 
 * This function returns the currently configured value for SDA data hold time.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  value of the data hold time for SDA
 */
static inline uint8_t LPI2C_HAL_MasterGetDataValidDelay(LPI2C_Type *baseAddr)
{
    uint32_t tmp = baseAddr->MCCR0;
    tmp = (tmp & LPI2C_MCCR0_DATAVD_MASK) >> LPI2C_MCCR0_DATAVD_SHIFT;
    return (uint8_t)tmp;
}


/*!
 * @brief Set the setup and hold delay for a START / STOP condition
 * 
 * This function configures the Minimum number of cycles (minus one) that is used 
 * by the master as the setup and hold time for a (repeated) START condition and setup 
 * time for a STOP condition. The setup time is extended by the time it takes to detect 
 * a rising edge on the external SCL pin. Ignoring any additional board delay due to 
 * external loading, this is equal to (2 + FILTSCL) / 2^PRESCALE cycles.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param value  setup and hold time for a START / STOP condition
 */
static inline void LPI2C_HAL_MasterSetSetupHoldDelay(LPI2C_Type *baseAddr, uint8_t value)
{
    uint32_t tmp = baseAddr->MCCR0;
    tmp &= ~(LPI2C_MCCR0_SETHOLD_MASK);
    tmp |= LPI2C_MCCR0_SETHOLD(value);
    baseAddr->MCCR0 = tmp;
}


/*!
 * @brief Return the configured setup and hold time 
 * 
 * This function returns the currently configured value for setup and hold delay 
 * for a START / STOP condition.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  setup and hold time for a START / STOP condition
 */
static inline uint8_t LPI2C_HAL_MasterGetSetupHoldDelay(LPI2C_Type *baseAddr)
{
    uint32_t tmp = baseAddr->MCCR0;
    tmp = (tmp & LPI2C_MCCR0_SETHOLD_MASK) >> LPI2C_MCCR0_SETHOLD_SHIFT;
    return (uint8_t)tmp;
}


/*!
 * @brief Set the minimum clock high period
 * 
 * This function configures the minimum number of cycles (minus one) that the 
 * SCL clock is driven high by the master. The SCL high time is extended by the 
 * time it takes to detect a rising edge on the external SCL pin. Ignoring any 
 * additional board delay due to external loading, this is equal to 
 * (2 + FILTSCL) / 2^PRESCALE cycles.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param value  minimum clock high period
 */
static inline void LPI2C_HAL_MasterSetClockHighPeriod(LPI2C_Type *baseAddr, uint8_t value)
{
    uint32_t tmp = baseAddr->MCCR0;
    tmp &= ~(LPI2C_MCCR0_CLKHI_MASK);
    tmp |= LPI2C_MCCR0_CLKHI(value);
    baseAddr->MCCR0 = tmp;
}


/*!
 * @brief Return the configured minimum clock high period
 * 
 * This function returns the currently configured value for clock high period.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  minimum clock high period
 */
static inline uint8_t LPI2C_HAL_MasterGetClockHighPeriod(LPI2C_Type *baseAddr)
{
    uint32_t tmp = baseAddr->MCCR0;
    tmp = (tmp & LPI2C_MCCR0_CLKHI_MASK) >> LPI2C_MCCR0_CLKHI_SHIFT;
    return (uint8_t)tmp;
}


/*!
 * @brief Set the minimum clock low period
 * 
 * This function configures the minimum number of cycles (minus one) that the 
 * SCL clock is driven low by the master. This value is also used for the 
 * minimum bus free time between a STOP and a START condition.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param value  minimum clock low period
 */
static inline void LPI2C_HAL_MasterSetClockLowPeriod(LPI2C_Type *baseAddr, uint8_t value)
{
    uint32_t tmp = baseAddr->MCCR0;
    tmp &= ~(LPI2C_MCCR0_CLKLO_MASK);
    tmp |= LPI2C_MCCR0_CLKLO(value);
    baseAddr->MCCR0 = tmp;
}


/*!
 * @brief Return the configured minimum clock low period
 * 
 * This function returns the currently configured value for clock low period.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  minimum clock low period
 */
static inline uint8_t LPI2C_HAL_MasterGetClockLowPeriod(LPI2C_Type *baseAddr)
{
    uint32_t tmp = baseAddr->MCCR0;
    tmp = (tmp & LPI2C_MCCR0_CLKLO_MASK) >> LPI2C_MCCR0_CLKLO_SHIFT;
    return (uint8_t)tmp;
}


/*!
 * @brief Set the data hold time for SDA in high-speed mode
 * 
 * This function sets the minimum number of cycles (minus one) that is used as the 
 * data hold time for SDA in High-Speed mode. Must be configured less than the 
 * minimum SCL low period.
 * This setting only has effect during High-Speed mode transfers.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param value  value of the data hold time for SDA
 */
static inline void LPI2C_HAL_MasterSetDataValidDelayHS(LPI2C_Type *baseAddr, uint8_t value)
{
    uint32_t tmp = baseAddr->MCCR1;
    tmp &= ~(LPI2C_MCCR1_DATAVD_MASK);
    tmp |= LPI2C_MCCR1_DATAVD(value);
    baseAddr->MCCR1 = tmp;
}


/*!
 * @brief Return the configured data hold time for SDA in high-speed mode
 * 
 * This function returns the currently configured value for SDA data hold time
 * in high-speed mode.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  value of the data hold time for SDA
 */
static inline uint8_t LPI2C_HAL_MasterGetDataValidDelayHS(LPI2C_Type *baseAddr)
{
    uint32_t tmp = baseAddr->MCCR1;
    tmp = (tmp & LPI2C_MCCR1_DATAVD_MASK) >> LPI2C_MCCR1_DATAVD_SHIFT;
    return (uint8_t)tmp;
}


/*!
 * @brief Set the setup and hold time for a START / STOP condition in high-speed mode
 * 
 * This function configures the Minimum number of cycles (minus one) that is used 
 * by the master as the setup and hold time for a (repeated) START condition and setup 
 * time for a STOP condition. The setup time is extended by the time it takes to detect 
 * a rising edge on the external SCL pin. Ignoring any additional board delay due to 
 * external loading, this is equal to (2 + FILTSCL) / 2^PRESCALE cycles.
 * This setting only has effect during High-Speed mode transfers.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param value  setup and hold time for a START / STOP condition
 */
static inline void LPI2C_HAL_MasterSetSetupHoldDelayHS(LPI2C_Type *baseAddr, uint8_t value)
{
    uint32_t tmp = baseAddr->MCCR1;
    tmp &= ~(LPI2C_MCCR1_SETHOLD_MASK);
    tmp |= LPI2C_MCCR1_SETHOLD(value);
    baseAddr->MCCR1 = tmp;
}


/*!
 * @brief Return the configured setup and hold time in high-speed mode
 * 
 * This function returns the currently configured value for setup and hold delay
 * for a START / STOP condition in high-speed mode.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  setup and hold time for a START / STOP condition
 */
static inline uint8_t LPI2C_HAL_MasterGetSetupHoldDelayHS(LPI2C_Type *baseAddr)
{
    uint32_t tmp = baseAddr->MCCR1;
    tmp = (tmp & LPI2C_MCCR1_SETHOLD_MASK) >> LPI2C_MCCR1_SETHOLD_SHIFT;
    return (uint8_t)tmp;
}


/*!
 * @brief Set the minimum clock high period in high-speed mode
 * 
 * This function configures the minimum number of cycles (minus one) that the 
 * SCL clock is driven high by the master. The SCL high time is extended by the 
 * time it takes to detect a rising edge on the external SCL pin. Ignoring any 
 * additional board delay due to external loading, this is equal to 
 * (2 + FILTSCL) / 2^PRESCALE cycles.
 * This setting only has effect during High-Speed mode transfers.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param value  minimum clock high period
 */
static inline void LPI2C_HAL_MasterSetClockHighPeriodHS(LPI2C_Type *baseAddr, uint8_t value)
{
    uint32_t tmp = baseAddr->MCCR1;
    tmp &= ~(LPI2C_MCCR1_CLKHI_MASK);
    tmp |= LPI2C_MCCR1_CLKHI(value);
    baseAddr->MCCR1 = tmp;
}


/*!
 * @brief Return the configured minimum clock high period in high-speed mode
 * 
 * This function returns the currently configured value for clock high period
 * in high-speed mode.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  minimum clock high period
 */
static inline uint8_t LPI2C_HAL_MasterGetClockHighPeriodHS(LPI2C_Type *baseAddr)
{
    uint32_t tmp = baseAddr->MCCR1;
    tmp = (tmp & LPI2C_MCCR1_CLKHI_MASK) >> LPI2C_MCCR1_CLKHI_SHIFT;
    return (uint8_t)tmp;
}


/*!
 * @brief Set the minimum clock low period in high-speed mode
 * 
 * This function configures the minimum number of cycles (minus one) that the 
 * SCL clock is driven low by the master. This value is also used for the 
 * minimum bus free time between a STOP and a START condition.
 * This setting only has effect during High-Speed mode transfers.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param value  minimum clock low period
 */
static inline void LPI2C_HAL_MasterSetClockLowPeriodHS(LPI2C_Type *baseAddr, uint8_t value)
{
    uint32_t tmp = baseAddr->MCCR1;
    tmp &= ~(LPI2C_MCCR1_CLKLO_MASK);
    tmp |= LPI2C_MCCR1_CLKLO(value);
    baseAddr->MCCR1 = tmp;
}


/*!
 * @brief Return the configured minimum clock low period in high-speed mode
 * 
 * This function returns the currently configured value for clock low period
 * in high-speed mode.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  minimum clock low period
 */
static inline uint8_t LPI2C_HAL_MasterGetClockLowPeriodHS(LPI2C_Type *baseAddr)
{
    uint32_t tmp = baseAddr->MCCR1;
    tmp = (tmp & LPI2C_MCCR1_CLKLO_MASK) >> LPI2C_MCCR1_CLKLO_SHIFT;
    return (uint8_t)tmp;
}


/*!
 * @brief Set the receive FIFO watermark
 * 
 * This function configures the receive FIFO watermark. Whenever the number of words in the receive 
 * FIFO is greater than the receive FIFO watermark, a receive data ready event is generated.
 * Writing a value equal or greater than the FIFO size will be truncated.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param value  number of words in the receive FIFO that will cause the receive data flag to be set
 */
static inline void LPI2C_HAL_MasterSetRxFIFOWatermark(LPI2C_Type *baseAddr, uint8_t value)
{
    uint32_t tmp = baseAddr->MFCR;
    tmp &= ~(LPI2C_MFCR_RXWATER_MASK);
    tmp |= LPI2C_MFCR_RXWATER(value);
    baseAddr->MFCR = tmp;
}


/*!
 * @brief Return the configured receive FIFO watermark
 * 
 * This function returns the currently configured value for receive FIFO watermark
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  number of words in the receive FIFO that will cause the receive data flag to be set
 */
static inline uint8_t LPI2C_HAL_MasterGetRxFIFOWatermark(LPI2C_Type *baseAddr)
{
    uint32_t tmp = baseAddr->MFCR;
    tmp = (tmp & LPI2C_MFCR_RXWATER_MASK) >> LPI2C_MFCR_RXWATER_SHIFT;
    return (uint8_t)tmp;
}


/*!
 * @brief Set the transmit FIFO watermark
 * 
 * This function configures the transmit FIFO watermark. Whenever the number of words in the transmit 
 * FIFO is greater than the transmit FIFO watermark, a transmit data request event is generated.
 * Writing a value equal or greater than the FIFO size will be truncated.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param value  number of words in the transmit FIFO that will cause the transmit data flag to be set
 */
static inline void LPI2C_HAL_MasterSetTxFIFOWatermark(LPI2C_Type *baseAddr, uint8_t value)
{
    uint32_t tmp = baseAddr->MFCR;
    tmp &= ~(LPI2C_MFCR_TXWATER_MASK);
    tmp |= LPI2C_MFCR_TXWATER(value);
    baseAddr->MFCR = tmp;
}


/*!
 * @brief Return the configured transmit FIFO watermark
 * 
 * This function returns the currently configured value for transmit FIFO watermark
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  number of words in the transmit FIFO that will cause the transmit data flag to be set
 */
static inline uint8_t LPI2C_HAL_MasterGetTxFIFOWatermark(LPI2C_Type *baseAddr)
{
    uint32_t tmp = baseAddr->MFCR;
    tmp = (tmp & LPI2C_MFCR_TXWATER_MASK) >> LPI2C_MFCR_TXWATER_SHIFT;
    return (uint8_t)tmp;
}


/*!
 * @brief Return the number of words in the receive FIFO
 * 
 * This function returns the number of words currently available in the receive FIFO.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  the number of words in the receive FIFO
 */
static inline uint8_t LPI2C_HAL_MasterGetRxFIFOCount(LPI2C_Type *baseAddr)
{
    uint32_t tmp = baseAddr->MFSR;
    tmp = (tmp & LPI2C_MFSR_RXCOUNT_MASK) >> LPI2C_MFSR_RXCOUNT_SHIFT;
    return (uint8_t)tmp;
}


/*!
 * @brief Return the number of words in the transmit FIFO
 * 
 * This function returns the number of words currently available in the transmit FIFO.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  the number of words in the transmit FIFO
 */
static inline uint8_t LPI2C_HAL_MasterGetTxFIFOCount(LPI2C_Type *baseAddr)
{
    uint32_t tmp = baseAddr->MFSR;
    tmp = (tmp & LPI2C_MFSR_TXCOUNT_MASK) >> LPI2C_MFSR_TXCOUNT_SHIFT;
    return (uint8_t)tmp;
}


/*!
 * @brief Provide commands and data for the LPI2C master
 * 
 * This function stores commands and data in the transmit FIFO and increments the FIFO 
 * write pointer.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param cmd  command for the LPI2C master
 * @param data  data for the LPI2C master
 */
static inline void LPI2C_HAL_MasterTransmitCmd(LPI2C_Type *baseAddr, lpi2c_master_command_t cmd, uint8_t data)
{
    baseAddr->MTDR = ((uint32_t)cmd << 8U) + (uint32_t)data;
}


/*!
 * @brief Return the received data
 * 
 * This function returns data received by the I2C master that has not been discarded 
 * due to data match settings or active command, and increments the FIFO read pointer.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  data received by the LPI2C master
 */
static inline uint8_t LPI2C_HAL_MasterGetRxData(LPI2C_Type *baseAddr)
{
    uint32_t tmp = baseAddr->MRDR;
    tmp = (tmp & LPI2C_MRDR_DATA_MASK) >> LPI2C_MRDR_DATA_SHIFT;
    return (uint8_t)tmp;
}


/*!
 * @brief Check if the master receive FIFO is empty
 * 
 * This function checks if the master receive FIFO is empty.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  status of the master receive FIFO
 */
static inline bool LPI2C_HAL_MasterGetRxEmpty(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->MRDR), LPI2C_MRDR_RXEMPTY_SHIFT);
}


/*!
 * @brief Reset the slave receive FIFO
 * 
 * This function empties the receive FIFO of the LPI2C slave.
 * 
 * @param baseAddr  base address of the LPI2C module
 */
static inline void LPI2C_HAL_SlaveRxFIFOResetCmd(LPI2C_Type *baseAddr)
{
    BITBAND_ACCESS32(&(baseAddr->SCR), LPI2C_SCR_RRF_SHIFT) = (uint32_t)1u;
}


/*!
 * @brief Reset the slave transmit FIFO
 * 
 * This function empties the transmit FIFO of the LPI2C slave. 
 * 
 * @param baseAddr  base address of the LPI2C module
 */
static inline void LPI2C_HAL_SlaveTxFIFOResetCmd(LPI2C_Type *baseAddr)
{
    BITBAND_ACCESS32(&(baseAddr->SCR), LPI2C_SCR_RTF_SHIFT) = (uint32_t)1u;
}


/*!
 * @brief Configure the slave filter in doze mode
 * 
 * This function enables or disables the digital filter in doze mode.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param enable  enable/disable the slave filter in doze mode
 */
static inline void LPI2C_HAL_SlaveSetFilterDoze(LPI2C_Type *baseAddr, bool enable)
{
    BITBAND_ACCESS32(&(baseAddr->SCR), LPI2C_SCR_FILTDZ_SHIFT) = (uint32_t)enable;
}


/*!
 * @brief Return the slave filter configuration in doze mode
 * 
 * This function returns the currently configured settings for the digital filter 
 * in Doze mode.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  slave filter configuration in doze mode
 */
static inline bool LPI2C_HAL_SlaveGetFilterDoze(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->SCR), LPI2C_SCR_FILTDZ_SHIFT);
}


/*!
 * @brief Enable/disable the slave filter
 * 
 * This function enables or disables the digital filter and output delay counter 
 * for slave mode.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param enable  enable/disable the slave filter
 */
static inline void LPI2C_HAL_SlaveSetFilterEnable(LPI2C_Type *baseAddr, bool enable)
{
    BITBAND_ACCESS32(&(baseAddr->SCR), LPI2C_SCR_FILTEN_SHIFT) = (uint32_t)enable;
}


/*!
 * @brief Check if the slave filter is enabled or disabled
 * 
 * This function returns the currently configured settings for the digital filter.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  slave filter configuration
 */
static inline bool LPI2C_HAL_SlaveGetFilterEnable(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->SCR), LPI2C_SCR_FILTEN_SHIFT);
}


/*!
 * @brief Set/clear the slave reset command
 * 
 * Calling this function with enable parameter set to true will perform a software 
 * reset of the LPI2C slave.
 *
 * @param baseAddr  base address of the LPI2C module
 * @param enable  specifies the reset state of the LPI2C slave logic
 */
static inline void LPI2C_HAL_SlaveSetSoftwareReset(LPI2C_Type *baseAddr, bool enable)
{
    BITBAND_ACCESS32(&(baseAddr->SCR), LPI2C_SCR_RST_SHIFT) = (uint32_t)enable;
}


/*!
 * @brief Return the reset setting for the LPI2C slave
 * 
 * This function returns the state of the LPI2C slave software reset bit.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  the reset state of the LPI2C slave logic
 */
static inline bool LPI2C_HAL_SlaveGetSoftwareReset(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->SCR), LPI2C_SCR_RST_SHIFT);
}


/*!
 * @brief Enable or disable the LPI2C slave
 * 
 * This function enables or disables the LPI2C module in slave mode.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param enable  specifies whether to enable or disable the LPI2C slave
 */
static inline void LPI2C_HAL_SlaveSetEnable(LPI2C_Type *baseAddr, bool enable)
{
    BITBAND_ACCESS32(&(baseAddr->SCR), LPI2C_SCR_SEN_SHIFT) = (uint32_t)enable;
}


/*!
 * @brief Return the enable/disable setting for the LPI2C slave
 * 
 * This function checks whether or not the LPI2C slave is enabled.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  the enable/disable setting for the LPI2C slave
 */
static inline bool LPI2C_HAL_SlaveGetEnable(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->SCR), LPI2C_SCR_SEN_SHIFT);
}


/*!
 * @brief Check the busy state of the bus
 * 
 * This function returns true if the I2C bus is busy and false if the bus is idle.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  busy state of the bus
 */
static inline bool LPI2C_HAL_SlaveGetBusBusyEvent(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->SSR), LPI2C_SSR_BBF_SHIFT);
}


/*!
 * @brief Check the busy state of the slave
 * 
 * This function returns true if the LPI2C slave is busy and false if 
 * the LPI2C slave is idle.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  busy state of the slave
 */
static inline bool LPI2C_HAL_SlaveGetSlaveBusyEvent(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->SSR), LPI2C_SSR_SBF_SHIFT);
}


/*!
 * @brief Check the occurrence of an SMBus alert response
 * 
 * This function checks for the detection of a SMBus Alert Response. This event is 
 * cleared by reading the received address - see function LPI2C_HAL_SlaveGetReceivedAddr().
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  indication of an SMBus alert response
 */
static inline bool LPI2C_HAL_SlaveGetSMBusAlertResponseEvent(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->SSR), LPI2C_SSR_SARF_SHIFT);
}


/*!
 * @brief Check the detection of the general call address
 * 
 * This function checks for the detection of a General Call Address. This event is 
 * cleared by reading the received address - see function LPI2C_HAL_SlaveGetReceivedAddr().
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  indication of a general call address detection
 */
static inline bool LPI2C_HAL_SlaveGetGeneralCallEvent(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->SSR), LPI2C_SSR_GCF_SHIFT);
}


/*!
 * @brief Check the detection of an ADDR1 address match
 * 
 * This function checks for a match of the ADDR1 address, or ADDR0 to ADDR1 range as
 * configured by function LPI2C_HAL_SlaveSetAddrConfig() This event is cleared 
 * by reading the received address - see function LPI2C_HAL_SlaveGetReceivedAddr().
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  indication of an ADDR1 address match
 */
static inline bool LPI2C_HAL_SlaveGetAddressMatch1Event(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->SSR), LPI2C_SSR_AM1F_SHIFT);
}


/*!
 * @brief Check the detection of an ADDR0 address match
 * 
 * This function checks for a match of the ADDR0 address, as
 * configured by function LPI2C_HAL_SlaveSetAddrConfig() This event is cleared 
 * by reading the received address - see function LPI2C_HAL_SlaveGetReceivedAddr().
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  indication of an ADDR0 address match
 */
static inline bool LPI2C_HAL_SlaveGetAddressMatch0Event(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->SSR), LPI2C_SSR_AM0F_SHIFT);
}


/*!
 * @brief Check the detection of a FIFO overflow or underflow
 * 
 * This function checks for the occurrence of a slave FIFO overflow or underflow. 
 * This event can only occur if clock stretching is disabled.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  indication of a FIFO overflow or underflow
 */
static inline bool LPI2C_HAL_SlaveGetFIFOErrorEvent(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->SSR), LPI2C_SSR_FEF_SHIFT);
}


/*!
 * @brief Check the detection of a bit error
 * 
 * This function checks for the occurrence of a bit error event. This event occurs
 * if the LPI2C slave transmits a logic one and detects a logic zero on the I2C bus. The
 * slave will ignore the rest of the transfer until the next (repeated) START condition.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  indication of a bit error
 */
static inline bool LPI2C_HAL_SlaveGetBitErrorEvent(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->SSR), LPI2C_SSR_BEF_SHIFT);
}


/*!
 * @brief Check the detection of a STOP condition
 * 
 * This function checks for the detection of a STOP condition, after the LPI2C slave 
 * matched the last address byte.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  indication of a STOP condition
 */
static inline bool LPI2C_HAL_SlaveGetSTOPDetectEvent(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->SSR), LPI2C_SSR_SDF_SHIFT);
}


/*!
 * @brief Check the detection of a repeated START condition
 * 
 * This function checks for the detection of a repeated START condition, after 
 * the LPI2C slave matched the last address byte. This event does not occur
 * when the slave first detects a START condition.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  indication of a repeated START condition
 */
static inline bool LPI2C_HAL_SlaveGetRepeatedStartEvent(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->SSR), LPI2C_SSR_RSF_SHIFT);
}


/*!
 * @brief Check if transmitting ACK response is required
 * 
 * This function checks if the LPI2C slave requests the software to provide
 * an ACK or NACK response. This event is cleared by calling function 
 * LPI2C_HAL_SlaveSetTransmitNACK().
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  indicates whether or not ACK response is required
 */
static inline bool LPI2C_HAL_SlaveGetTransmitACKEvent(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->SSR), LPI2C_SSR_TAF_SHIFT);
}


/*!
 * @brief Check the validity of the Address Status Register
 * 
 * This function checks for the detection of a valid address. The event is 
 * cleared by reading the address - see function LPI2C_HAL_SlaveGetReceivedAddr().
 * It can also be cleared by reading the data register, when data register has 
 * been configured to allow address reads - see functions 
 * LPI2C_HAL_SlaveSetRxDataConfig() and LPI2C_HAL_SlaveGetData()
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  indication of the validity of the Address Status Register
 */
static inline bool LPI2C_HAL_SlaveGetAddressValidEvent(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->SSR), LPI2C_SSR_AVF_SHIFT);
}


/*!
 * @brief Check the availability of receive data
 * 
 * This function checks for the availability of data received by the I2C slave.
 * The event is cleared by reading the received data - see function 
 * LPI2C_HAL_SlaveGetData(). The event is not cleared by calling 
 * LPI2C_HAL_SlaveGetData() if the data register is configured to allow address 
 * reads and an address valid event is active.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  indication of receive data availability
 */
static inline bool LPI2C_HAL_SlaveGetReceiveDataEvent(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->SSR), LPI2C_SSR_RDF_SHIFT);
}


/*!
 * @brief Check if transmit data is requested
 * 
 * This function checks if the LPI2C slave requests data to transmit. The
 * event is cleared by providing transmit data - see function 
 * LPI2C_HAL_SlaveTransmitData(). The event can also be automatically cleared
 * if the LPI2C module detects a NACK or a repeated START or STOP condition -
 * see function LPI2C_HAL_SlaveSetTxFlagConfig()
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  indication of a transmit data request
 */
static inline bool LPI2C_HAL_SlaveGetTransmitDataEvent(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->SSR), LPI2C_SSR_TDF_SHIFT);
}


/*!
 * @brief Clear the FIFO overflow or underflow flag
 * 
 * This function clears the FIFO overflow or underflow event.
 * 
 * @param baseAddr  base address of the LPI2C module
 */
static inline void LPI2C_HAL_SlaveClearFIFOErrorEvent(LPI2C_Type *baseAddr)
{
    baseAddr->SSR = (uint32_t)(1u << LPI2C_SSR_FEF_SHIFT);
}


/*!
 * @brief Clear bit error flag
 * 
 * This function clears the bit error event.
 * 
 * @param baseAddr  base address of the LPI2C module
 */
static inline void LPI2C_HAL_SlaveClearBitErrorEvent(LPI2C_Type *baseAddr)
{
    baseAddr->SSR = (uint32_t)(1u << LPI2C_SSR_BEF_SHIFT);
}


/*!
 * @brief Clear the STOP detect flag
 * 
 * This function clears the STOP detect event.
 * 
 * @param baseAddr  base address of the LPI2C module
 */
static inline void LPI2C_HAL_SlaveClearSTOPDetectEvent(LPI2C_Type *baseAddr)
{
    baseAddr->SSR = (uint32_t)(1u << LPI2C_SSR_SDF_SHIFT);
}


/*!
 * @brief Clear the repeated START detect flag
 * 
 * This function clears the repeated START detect event.
 * 
 * @param baseAddr  base address of the LPI2C module
 */
static inline void LPI2C_HAL_SlaveClearRepeatedStartEvent(LPI2C_Type *baseAddr)
{
    baseAddr->SSR = (uint32_t)(1u << LPI2C_SSR_RSF_SHIFT);
}


/*!
 * @brief Enable or disable specified LPI2C slave interrupts
 * 
 * This function can enable or disable one or more slave interrupt sources 
 * specified by the interrupts parameter.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param interrupts  interrupts to be enabled or disabled; 
 *  must be a bitwise or between one or more of the following constants: 
 *  - LPI2C_HAL_SLAVE_SMBUS_ALERT_RESPONSE  - SMBus Alert Response Interrupt
 *  - LPI2C_HAL_SLAVE_GENERAL_CALL          - General Call Interrupt
 *  - LPI2C_HAL_SLAVE_ADDRESS_MATCH_1       - Address Match 1 Interrupt
 *  - LPI2C_HAL_SLAVE_ADDRESS_MATCH_0       - Address Match 0 Interrupt
 *  - LPI2C_HAL_SLAVE_FIFO_ERROR            - FIFO Error Interrupt
 *  - LPI2C_HAL_SLAVE_BIT_ERROR             - Bit Error Interrupt
 *  - LPI2C_HAL_SLAVE_STOP_DETECT           - STOP Detect Interrupt
 *  - LPI2C_HAL_SLAVE_REPEATED_START        - Repeated Start Interrupt
 *  - LPI2C_HAL_SLAVE_TRANSMIT_ACK          - Transmit ACK Interrupt
 *  - LPI2C_HAL_SLAVE_ADDRESS_VALID         - Address Valid Interrupt
 *  - LPI2C_HAL_SLAVE_RECEIVE_DATA          - Receive Data Interrupt
 *  - LPI2C_HAL_SLAVE_TRANSMIT_DATA         - Transmit Data Interrupt
 * @param enable  specifies whether to enable or disable specified interrupts
 */
static inline void LPI2C_HAL_SlaveSetInt(LPI2C_Type *baseAddr, uint32_t interrupts, bool enable)
{
    uint32_t tmp = baseAddr->SIER;

    if (enable == true)
    {
        tmp |= interrupts;
    }
    else
    {
        tmp &= ~interrupts;
    }
    baseAddr->SIER = tmp;
}


/*!
 * @brief Return the state of the specified LPI2C slave interrupt
 * 
 * This function returns the enabled/disabled state of the slave interrupt 
 * source specified by the interrupt parameter.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param interrupts  interrupt for which the check is made; 
 *  must be one of the following constants:
 *  - LPI2C_HAL_SLAVE_SMBUS_ALERT_RESPONSE  - SMBus Alert Response Interrupt
 *  - LPI2C_HAL_SLAVE_GENERAL_CALL          - General Call Interrupt
 *  - LPI2C_HAL_SLAVE_ADDRESS_MATCH_1       - Address Match 1 Interrupt
 *  - LPI2C_HAL_SLAVE_ADDRESS_MATCH_0       - Address Match 0 Interrupt
 *  - LPI2C_HAL_SLAVE_FIFO_ERROR            - FIFO Error Interrupt
 *  - LPI2C_HAL_SLAVE_BIT_ERROR             - Bit Error Interrupt
 *  - LPI2C_HAL_SLAVE_STOP_DETECT           - STOP Detect Interrupt
 *  - LPI2C_HAL_SLAVE_REPEATED_START        - Repeated Start Interrupt
 *  - LPI2C_HAL_SLAVE_TRANSMIT_ACK          - Transmit ACK Interrupt
 *  - LPI2C_HAL_SLAVE_ADDRESS_VALID         - Address Valid Interrupt
 *  - LPI2C_HAL_SLAVE_RECEIVE_DATA          - Receive Data Interrupt
 *  - LPI2C_HAL_SLAVE_TRANSMIT_DATA         - Transmit Data Interrupt
 * @return  enable/disable state of specified interrupt
 */
static inline bool LPI2C_HAL_SlaveGetInt(LPI2C_Type *baseAddr, uint32_t interrupts)
{
    uint32_t tmp = baseAddr->SIER;

    if (tmp | interrupts)
    {
        return true;
    }
    else
    {
        return false;
    }
}


/*!
 * @brief Enable/disable slave address valid DMA requests
 * 
 * This function enables or disables generation of Rx DMA requests when a valid
 * address is received. Note that this DMA request is shared with slave receive 
 * data requests, so if both are used the receive data register should be 
 * configured to allow address reads - see function LPI2C_HAL_SlaveSetRxDataConfig()
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param enable  specifies whether to enable or disable address valid DMA requests
 */
static inline void LPI2C_HAL_SlaveSetAddrDMA(LPI2C_Type *baseAddr, bool enable)
{
    BITBAND_ACCESS32(&(baseAddr->SDER), LPI2C_SDER_AVDE_SHIFT) = (uint32_t)enable;
}


/*!
 * @brief Enable/disable slave receive data DMA requests
 * 
 * This function enables or disables generation of Rx DMA requests when received
 * data is available.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param enable  specifies whether to enable or disable receive data DMA requests
 */
static inline void LPI2C_HAL_SlaveSetRxDMA(LPI2C_Type *baseAddr, bool enable)
{
    BITBAND_ACCESS32(&(baseAddr->SDER), LPI2C_SDER_RDDE_SHIFT) = (uint32_t)enable;
}


/*!
 * @brief Enable/disable slave transmit data DMA requests
 * 
 * This function enables or disables generation of Tx DMA requests when the module
 * requires more data to transmit.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param enable  specifies whether to enable or disable transmit data DMA requests
 */
static inline void LPI2C_HAL_SlaveSetTxDMA(LPI2C_Type *baseAddr, bool enable)
{
    BITBAND_ACCESS32(&(baseAddr->SDER), LPI2C_SDER_TDDE_SHIFT) = (uint32_t)enable;
}


/*!
 * @brief Check if slave address valid DMA requests are enabled
 * 
 * This function returns true if address valid DMA requests are enabled.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  enabled/disabled status of address valid DMA requests
 */
static inline bool LPI2C_HAL_SlaveGetAddrDMA(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->SDER), LPI2C_SDER_AVDE_SHIFT);
}


/*!
 * @brief Check if slave receive data DMA requests are enabled
 * 
 * This function returns true if receive data DMA requests are enabled.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  enabled/disabled status of receive data DMA requests
 */
static inline bool LPI2C_HAL_SlaveGetRxDMA(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->SDER), LPI2C_SDER_RDDE_SHIFT);
}


/*!
 * @brief Check if slave transmit data DMA requests are enabled
 * 
 * This function returns true if transmit data DMA requests are enabled.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  enabled/disabled status of transmit data DMA requests
 */
static inline bool LPI2C_HAL_SlaveGetTxDMA(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->SDER), LPI2C_SDER_TDDE_SHIFT);
}


/*!
 * @brief Control address match configuration
 * 
 * This function configures the condition that will cause an address match to 
 * occur. See type lpi2c_slave_addr_config_t for a description of available options.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param configuration  configures the condition that will cause an address to match
 */
static inline void LPI2C_HAL_SlaveSetAddrConfig(LPI2C_Type *baseAddr, lpi2c_slave_addr_config_t configuration)
{
    uint32_t tmp = baseAddr->SCFGR1;
    tmp &= ~(LPI2C_SCFGR1_ADDRCFG_MASK);
    tmp |= LPI2C_SCFGR1_ADDRCFG(configuration);
    baseAddr->SCFGR1 = tmp;
}


/*!
 * @brief Return the address match configuration
 * 
 * This function returns the currently configured option for address match.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  address match configuration
 */
static inline lpi2c_slave_addr_config_t LPI2C_HAL_SlaveGetAddrConfig(LPI2C_Type *baseAddr)
{
    uint32_t tmp = baseAddr->SCFGR1;
    tmp = (tmp & LPI2C_SCFGR1_ADDRCFG_MASK) >> LPI2C_SCFGR1_ADDRCFG_SHIFT;
    return (lpi2c_slave_addr_config_t)tmp;
}


/*!
 * @brief Control detection of the High-speed Mode master code
 * 
 * This function enables or disables the detection of the High-speed Mode 
 * master code of slave address 0000_1XX, but does not cause an address match 
 * on this code. When set and any Hs-mode master code is detected, the slave 
 * filter and ACK stalls are disabled until the next STOP condition is detected.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param enable  enable/disable the detection of the High-speed Mode master code
 */
static inline void LPI2C_HAL_SlaveSetHighSpeedModeDetect(LPI2C_Type *baseAddr, bool enable)
{
    BITBAND_ACCESS32(&(baseAddr->SCFGR1), LPI2C_SCFGR1_HSMEN_SHIFT) = (uint32_t)enable;
}


/*!
 * @brief Return the state of the High-speed Mode master code detection
 * 
 * This function returns the currently configured option for master code detection.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  enabled/disabled state of the High-speed Mode master code detection
 */
static inline bool LPI2C_HAL_SlaveGetHighSpeedModeDetect(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->SCFGR1), LPI2C_SCFGR1_HSMEN_SHIFT);
}


/*!
 * @brief Control slave behaviour when NACK is detected
 * 
 * This function controls the option to ignore received NACKs. When enabled, the 
 * LPI2C slave will continue transfers after a NACK is detected. This option is needed
 * for Ultra-Fast mode.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param nack_config  slave behaviour when NACK is detected
 */
static inline void LPI2C_HAL_SlaveSetIgnoreNACK(LPI2C_Type *baseAddr, lpi2c_slave_nack_config_t nack_config)
{
    BITBAND_ACCESS32(&(baseAddr->SCFGR1), LPI2C_SCFGR1_IGNACK_SHIFT) = (uint32_t)nack_config;
}


/*!
 * @brief Return the configured slave behaviour when NACK is detected
 * 
 * This function returns the currently configured option for handling NACKs.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  configured slave behaviour when NACK is detected
 */
static inline lpi2c_slave_nack_config_t LPI2C_HAL_SlaveGetIgnoreNACK(LPI2C_Type *baseAddr)
{
    return (lpi2c_slave_nack_config_t)BITBAND_ACCESS32(&(baseAddr->SCFGR1), LPI2C_SCFGR1_IGNACK_SHIFT);
}


/*!
 * @brief Control the functionality of the receive data register
 * 
 * This function can configure the receive data register to allow the address to be
 * read from it. When this option is enabled reading the data register when an 
 * address valid event is active will return the address and clear the address valid 
 * event. When the address valid event is not active data reads will behave normally
 * (return received data and clear the receive data event).
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param configuration  controls if address can be read from the receive data register
 */
static inline void LPI2C_HAL_SlaveSetRxDataConfig(LPI2C_Type *baseAddr, lpi2c_slave_rxdata_config_t configuration)
{
    BITBAND_ACCESS32(&(baseAddr->SCFGR1), LPI2C_SCFGR1_RXCFG_SHIFT) = (uint32_t)configuration;
}


/*!
 * @brief Return the configured functionality of the receive data register
 * 
 * This function returns the current configuration for the receive data register.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  specifies if address can be read from the receive data register
 */
static inline lpi2c_slave_rxdata_config_t LPI2C_HAL_SlaveGetRxDataConfig(LPI2C_Type *baseAddr)
{
    return (lpi2c_slave_rxdata_config_t)BITBAND_ACCESS32(&(baseAddr->SCFGR1), LPI2C_SCFGR1_RXCFG_SHIFT);
}


/*!
 * @brief Control the conditions for setting the transmit data flag
 * 
 * This function controls the conditions that will trigger transmit data events. If this option is
 * enabled the transmit data register is automatically emptied when a slave-transmit transfer is
 * detected. This cause the transmit data event to be triggered whenever a slave-transmit transfer 
 * is detected and negate at the end of the slave-transmit transfer.
 * If this option is disabled the transmit data event will assert whenever the transit data register 
 * is empty and negate when the transmit data register is full. This allows the transmit data register
 * to be filled before a slave transmit transfer is detected, but can cause the transmit data register
 * to be written before a NACK is detected on the last byte of a slave transmit transfer.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param configuration  controls if transmit data flag is automatically cleared when NACK is detected
 */
static inline void LPI2C_HAL_SlaveSetTxFlagConfig(LPI2C_Type *baseAddr, lpi2c_slave_txflag_config_t configuration)
{
    BITBAND_ACCESS32(&(baseAddr->SCFGR1), LPI2C_SCFGR1_TXCFG_SHIFT) = (uint32_t)configuration;
}


/*!
 * @brief Return the configured settings for the transmit data flag
 * 
 * This function returns the currently configured setting for the transmit data flag.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  specifies if transmit data flag is automatically cleared when NACK is detected
 */
static inline lpi2c_slave_txflag_config_t LPI2C_HAL_SlaveGetTxFlagConfig(LPI2C_Type *baseAddr)
{
    return (lpi2c_slave_txflag_config_t)BITBAND_ACCESS32(&(baseAddr->SCFGR1), LPI2C_SCFGR1_TXCFG_SHIFT);
}


/*!
 * @brief Enable or disable match on SMBus Alert
 * 
 * This function enables or disables match on SMBus Alert.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param enable  enable or disable match on SMBus Alert
 */
static inline void LPI2C_HAL_SlaveSetSMBusAlert(LPI2C_Type *baseAddr, bool enable)
{
    BITBAND_ACCESS32(&(baseAddr->SCFGR1), LPI2C_SCFGR1_SAEN_SHIFT) = (uint32_t)enable;
}


/*!
 * @brief Return the configured state for the SMBus Alert match
 * 
 * This function returns the currently configured setting for SMBus Alert match.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  specifies if match on SMBus Alert is enabled or disabled
 */
static inline bool LPI2C_HAL_SlaveGetSMBusAlert(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->SCFGR1), LPI2C_SCFGR1_SAEN_SHIFT);
}


/*!
 * @brief Enable or disable general call address
 * 
 * This function enables or disables match general call address.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param enable  enable or disable general call address
 */
static inline void LPI2C_HAL_SlaveSetGeneralCall(LPI2C_Type *baseAddr, bool enable)
{
    BITBAND_ACCESS32(&(baseAddr->SCFGR1), LPI2C_SCFGR1_GCEN_SHIFT) = (uint32_t)enable;
}


/*!
 * @brief Return the configured state for the general call address
 * 
 * This function returns the currently configured setting for general call address matching.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  specifies if general call address is enabled or disabled
 */
static inline bool LPI2C_HAL_SlaveGetGeneralCall(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->SCFGR1), LPI2C_SCFGR1_GCEN_SHIFT);
}


/*!
 * @brief Enable or disable clock stretching for the sending of the ACK bit
 * 
 * This function enables or disables SCL clock stretching during slave-transmit address 
 * byte(s) and slave-receiver address and data byte(s) to allow software to write the 
 * Transmit ACK Register before the ACK or NACK is transmitted. Clock stretching occurs 
 * when transmitting the 9th bit and is therefore not compatible with high speed mode.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param enable  enable or disable clock stretching
 */
static inline void LPI2C_HAL_SlaveSetACKStall(LPI2C_Type *baseAddr, bool enable)
{
    BITBAND_ACCESS32(&(baseAddr->SCFGR1), LPI2C_SCFGR1_ACKSTALL_SHIFT) = (uint32_t)enable;
}


/*!
 * @brief Return the configured state for clock stretching for the sending of the ACK bit
 * 
 * This function returns the currently configured setting for clock stretching before 
 * the ACK bit.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  indicates if clock stretching is enabled or disabled
 */
static inline bool LPI2C_HAL_SlaveGetACKStall(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->SCFGR1), LPI2C_SCFGR1_ACKSTALL_SHIFT);
}


/*!
 * @brief Enable or disable clock stretching for data transmission
 * 
 * This function enables or disables SCL clock stretching when the transmit data 
 * flag is set during a slave-transmit transfer. Clock stretching occurs following 
 * the 9th bit and is therefore compatible with high speed mode.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param enable  enable or disable clock stretching
 */
static inline void LPI2C_HAL_SlaveSetTXDStall(LPI2C_Type *baseAddr, bool enable)
{
    BITBAND_ACCESS32(&(baseAddr->SCFGR1), LPI2C_SCFGR1_TXDSTALL_SHIFT) = (uint32_t)enable;
}


/*!
 * @brief Return the configured state for clock stretching for data transmission
 * 
 * This function returns the currently configured setting for data transmission 
 * clock stretching.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  indicates if clock stretching is enabled or disabled
 */
static inline bool LPI2C_HAL_SlaveGetTXDStall(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->SCFGR1), LPI2C_SCFGR1_TXDSTALL_SHIFT);
}


/*!
 * @brief Enable or disable clock stretching for data reception
 * 
 * This function enables or disables SCL clock stretching when receive data flag 
 * is set during a slave-receive transfer. Clock stretching occurs following the 9th
 * bit and is therefore compatible with high speed mode.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param enable  enable or disable clock stretching
 */
static inline void LPI2C_HAL_SlaveSetRXStall(LPI2C_Type *baseAddr, bool enable)
{
    BITBAND_ACCESS32(&(baseAddr->SCFGR1), LPI2C_SCFGR1_RXSTALL_SHIFT) = (uint32_t)enable;
}


/*!
 * @brief Return the configured state for clock stretching for data reception
 * 
 * This function returns the currently configured setting for data reception 
 * clock stretching.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  indicates if clock stretching is enabled or disabled
 */
static inline bool LPI2C_HAL_SlaveGetRXStall(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->SCFGR1), LPI2C_SCFGR1_RXSTALL_SHIFT);
}


/*!
 * @brief Enable or disable clock stretching for valid address reception
 * 
 * This function enables or disables SCL clock stretching when the address valid 
 * flag is asserted. Clock stretching only occurs following the 9th bit and is 
 * therefore compatible with high speed mode.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param enable  enable or disable clock stretching
 */
static inline void LPI2C_HAL_SlaveSetAddrStall(LPI2C_Type *baseAddr, bool enable)
{
    BITBAND_ACCESS32(&(baseAddr->SCFGR1), LPI2C_SCFGR1_ADRSTALL_SHIFT) = (uint32_t)enable;
}


/*!
 * @brief Return the configured state for clock stretching for valid address reception
 * 
 * This function returns the currently configured setting for address valid 
 * clock stretching.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  indicates if clock stretching is enabled or disabled
 */
static inline bool LPI2C_HAL_SlaveGetAddrStall(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->SCFGR1), LPI2C_SCFGR1_ADRSTALL_SHIFT);
}


/*!
 * @brief Configure the LPI2C slave SDA glitch filter
 * 
 * This function configures the I2C slave digital glitch filters for SDA input. 
 * A configuration of 0 will disable the glitch filter. Glitches equal to or less 
 * than FILTSDA cycles long will be filtered out and ignored. The latency through 
 * the glitch filter is equal to FILTSDA+3 cycles and must be configured less than 
 * the minimum SCL low or high period. The glitch filter cycle count is not affected 
 * by the PRESCALE configuration, and is disabled in high speed mode.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param cycles  number of cycles for the LPI2C slave SDA glitch filter
 */
static inline void LPI2C_HAL_SlaveSetSDAGlitchFilter(LPI2C_Type *baseAddr, uint8_t cycles)
{
    uint32_t tmp = baseAddr->SCFGR2;
    tmp &= ~(LPI2C_SCFGR2_FILTSDA_MASK);
    tmp |= LPI2C_SCFGR2_FILTSDA(cycles);
    baseAddr->SCFGR2 = tmp;
}


/*!
 * @brief Return the LPI2C slave SDA glitch filter configuration
 * 
 * This function returns the currently configured slave SDA glitch filter.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  number of cycles for the LPI2C slave SDA glitch filter
 */
static inline uint8_t LPI2C_HAL_SlaveGetSDAGlitchFilter(LPI2C_Type *baseAddr)
{
    uint32_t tmp = baseAddr->SCFGR2;
    tmp = (tmp & LPI2C_SCFGR2_FILTSDA_MASK) >> LPI2C_SCFGR2_FILTSDA_SHIFT;
    return (uint8_t)tmp;
}


/*!
 * @brief Configure the LPI2C slave SCL glitch filter
 * 
 * This function configures the I2C slave digital glitch filters for SCL input, 
 * a configuration of 0 will disable the glitch filter. Glitches equal to or less 
 * than FILTSCL cycles long will be filtered out and ignored. The latency through 
 * the glitch filter is equal to FILTSCL+3 cycles and must be configured less than 
 * the minimum SCL low or high period. The glitch filter cycle count is not affected 
 * by the PRESCALE configuration, and is disabled in high speed mode.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param cycles  number of cycles for the LPI2C slave SCL glitch filter
 */
static inline void LPI2C_HAL_SlaveSetSCLGlitchFilter(LPI2C_Type *baseAddr, uint8_t cycles)
{
    uint32_t tmp = baseAddr->SCFGR2;
    tmp &= ~(LPI2C_SCFGR2_FILTSCL_MASK);
    tmp |= LPI2C_SCFGR2_FILTSCL(cycles);
    baseAddr->SCFGR2 = tmp;
}


/*!
 * @brief Return the LPI2C slave SCL glitch filter configuration
 * 
 * This function returns the currently configured slave SDA glitch filter.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  number of cycles for the LPI2C slave SCL glitch filter
 */
static inline uint8_t LPI2C_HAL_SlaveGetSCLGlitchFilter(LPI2C_Type *baseAddr)
{
    uint32_t tmp = baseAddr->SCFGR2;
    tmp = (tmp & LPI2C_SCFGR2_FILTSCL_MASK) >> LPI2C_SCFGR2_FILTSCL_SHIFT;
    return (uint8_t)tmp;
}


/*!
 * @brief Configure the SDA data valid delay time for the I2C slave
 * 
 * This function configures the SDA data valid delay time for the I2C slave 
 * equal to FILTSCL+DATAVD+3 cycles. This data valid delay must be configured 
 * to less than the minimum SCL low period. The I2C slave data valid delay time 
 * is not affected by the PRESCALE configuration, and is disabled in high speed 
 * mode.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param cycles  number of cycles for the SDA data valid delay time for the I2C slave
 */
static inline void LPI2C_HAL_SlaveSetDataValidDelay(LPI2C_Type *baseAddr, uint8_t cycles)
{
    uint32_t tmp = baseAddr->SCFGR2;
    tmp &= ~(LPI2C_SCFGR2_DATAVD_MASK);
    tmp |= LPI2C_SCFGR2_DATAVD(cycles);
    baseAddr->SCFGR2 = tmp;
}


/*!
 * @brief Return the SDA data valid delay time configuration
 * 
 * This function returns the currently configured slave data valid delay.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  number of cycles for the SDA data valid delay time for the I2C slave
 */
static inline uint8_t LPI2C_HAL_SlaveGetDataValidDelay(LPI2C_Type *baseAddr)
{
    uint32_t tmp = baseAddr->SCFGR2;
    tmp = (tmp & LPI2C_SCFGR2_DATAVD_MASK) >> LPI2C_SCFGR2_DATAVD_SHIFT;
    return (uint8_t)tmp;
}


/*!
 * @brief Configure the minimum clock hold time for the I2C slave
 * 
 * This function configures the minimum clock hold time for the I2C slave, when 
 * clock stretching is enabled. The minimum hold time is equal to CLKHOLD+3 cycles. 
 * The I2C slave clock hold time is not affected by the PRESCALE configuration, 
 * and is disabled in high speed mode.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param cycles  number of cycles for the minimum clock hold time for the I2C slave
 */
static inline void LPI2C_HAL_SlaveSetClockHoldTime(LPI2C_Type *baseAddr, uint8_t cycles)
{
    uint32_t tmp = baseAddr->SCFGR2;
    tmp &= ~(LPI2C_SCFGR2_CLKHOLD_MASK);
    tmp |= LPI2C_SCFGR2_CLKHOLD(cycles);
    baseAddr->SCFGR2 = tmp;
}


/*!
 * @brief Return the minimum clock hold time configuration
 * 
 * This function returns the currently configured slave clock hold time.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  number of cycles for the minimum clock hold time for the I2C slave
 */
static inline uint8_t LPI2C_HAL_SlaveGetClockHoldTime(LPI2C_Type *baseAddr)
{
    uint32_t tmp = baseAddr->SCFGR2;
    tmp = (tmp & LPI2C_SCFGR2_CLKHOLD_MASK) >> LPI2C_SCFGR2_CLKHOLD_SHIFT;
    return (uint8_t)tmp;
}


/*!
 * @brief Configure the ADDR1 address for slave address match
 * 
 * This function configures the ADDR1 value which is used to validate the received 
 * slave address. In 10-bit mode, the first address byte is compared to 
 * { 11110, ADDR1[10:9] } and the second address byte is compared to ADDR1[8:1]. 
 * In 7-bit mode, the address is compared to ADDR1[7:1]
 * The formula used for address validation is configured with function 
 * LPI2C_HAL_SlaveSetAddrConfig().
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param addr  ADDR1 address for slave address match
 */
static inline void LPI2C_HAL_SlaveSetAddr1(LPI2C_Type *baseAddr, uint16_t addr)
{
    uint32_t tmp = baseAddr->SAMR;
    tmp &= ~(LPI2C_SAMR_ADDR1_MASK);
    tmp |= LPI2C_SAMR_ADDR1(addr);
    baseAddr->SAMR = tmp;
}


/*!
 * @brief Return the ADDR1 address for slave address match
 * 
 * This function returns the currently configured value for ADDR1.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  ADDR1 address for slave address match
 */
static inline uint16_t LPI2C_HAL_SlaveGetAddr1(LPI2C_Type *baseAddr)
{
    uint32_t tmp = baseAddr->SAMR;
    tmp = (tmp & LPI2C_SAMR_ADDR1_MASK) >> LPI2C_SAMR_ADDR1_SHIFT;
    return (uint16_t)tmp;
}


/*!
 * @brief Configure the ADDR0 address for slave address match
 * 
 * This function configures the ADDR0 value which is used to validate the received 
 * slave address. In 10-bit mode, the first address byte is compared to 
 * { 11110, ADDR0[10:9] } and the second address byte is compared to ADDR0[8:1]. 
 * In 7-bit mode, the address is compared to ADDR0[7:1]
 * The formula used for address validation is configured with function 
 * LPI2C_HAL_SlaveSetAddrConfig().
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param addr  ADDR0 address for slave address match
 */
static inline void LPI2C_HAL_SlaveSetAddr0(LPI2C_Type *baseAddr, uint16_t addr)
{
    uint32_t tmp = baseAddr->SAMR;
    tmp &= ~(LPI2C_SAMR_ADDR0_MASK);
    tmp |= LPI2C_SAMR_ADDR0(addr);
    baseAddr->SAMR = tmp;
}


/*!
 * @brief Return the ADDR0 address for slave address match
 * 
 * This function returns the currently configured value for ADDR0.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  ADDR0 address for slave address match
 */
static inline uint16_t LPI2C_HAL_SlaveGetAddr0(LPI2C_Type *baseAddr)
{
    uint32_t tmp = baseAddr->SAMR;
    tmp = (tmp & LPI2C_SAMR_ADDR0_MASK) >> LPI2C_SAMR_ADDR0_SHIFT;
    return (uint16_t)tmp;
}


/*!
 * @brief Check the validity of received address
 * 
 * This function checks whether the received address register contains a 
 * valid address.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  validity of received address
 */
static inline lpi2c_slave_addr_valid_t LPI2C_HAL_SlaveGetAddrValid(LPI2C_Type *baseAddr)
{
    return (lpi2c_slave_addr_valid_t)BITBAND_ACCESS32(&(baseAddr->SASR), LPI2C_SASR_ANV_SHIFT);
}


/*!
 * @brief Return the received slave address
 * 
 * This function returns the received slave address. Reading the address clears 
 * the address valid event. The address can be 7-bit or 10-bit (10-bit addresses 
 * are prefixed by 11110) and includes the R/W bit in the least significant position.
 * Use function LPI2C_HAL_SlaveGetAddrValid() before calling this function to 
 * ensure a valid value will be read.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  received address
 */
static inline uint16_t LPI2C_HAL_SlaveGetReceivedAddr(LPI2C_Type *baseAddr)
{
    uint32_t tmp = baseAddr->SASR;
    tmp = (tmp & LPI2C_SASR_RADDR_MASK) >> LPI2C_SASR_RADDR_SHIFT;
    return (uint16_t)tmp;
}


/*!
 * @brief Configure the ACK/NACK transmission after a received byte
 * 
 * This function can be used to instruct the LPI2C slave whether to send an ACK or 
 * a NACK after receiving a byte. When ACK stall is enabled this function must be 
 * called after each matching address and after each received data byte. It can also 
 * be called when LPI2C Slave is disabled or idle to configure the default ACK/NACK.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param nack  specifies whether to transmit ACK or NACK
 */
static inline void LPI2C_HAL_SlaveSetTransmitNACK(LPI2C_Type *baseAddr, lpi2c_slave_nack_transmit_t nack)
{
    BITBAND_ACCESS32(&(baseAddr->STAR), LPI2C_STAR_TXNACK_SHIFT) = (uint32_t)nack;
}


/*!
 * @brief Return the configured ACK/NACK transmission setting
 * 
 * This function returns the currently configured setting for ACK/NACK response.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  specifies whether ACK or NACK is configured to be transmitted
 */
static inline lpi2c_slave_nack_transmit_t LPI2C_HAL_SlaveGetTransmitNACK(LPI2C_Type *baseAddr)
{
    return (lpi2c_slave_nack_transmit_t)BITBAND_ACCESS32(&(baseAddr->STAR), LPI2C_STAR_TXNACK_SHIFT);
}


/*!
 * @brief Provide data for the LPI2C slave transmitter
 * 
 * This function provides one byte of data for the LPI2C slave to transmit. 
 * Calling this function clears the transmit data event.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @param data  data for the LPI2C slave transmitter
 */
static inline void LPI2C_HAL_SlaveTransmitData(LPI2C_Type *baseAddr, uint8_t data)
{
    baseAddr->STDR = (uint32_t)data;
}


/*!
 * @brief Check if the current received data is the first in the current frame
 * 
 * This function checks if the currently received data byte is the first byte 
 * since a (repeated) START or STOP condition.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  specifies if the current received data is the first in the current frame
 */
static inline bool LPI2C_HAL_SlaveGetStartOfFrame(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->SRDR), LPI2C_SRDR_SOF_SHIFT);
}


/*!
 * @brief Check if the receive data register is empty
 * 
 * This function checks if the received data register is empty.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  specifies if the receive data register is empty
 */
static inline bool LPI2C_HAL_SlaveGetRXEmpty(LPI2C_Type *baseAddr)
{
    return (bool)BITBAND_ACCESS32(&(baseAddr->SRDR), LPI2C_SRDR_RXEMPTY_SHIFT);
}


/*!
 * @brief Return the data received by the LPI2C slave receiver
 * 
 * This function returns the data received by the I2C slave.
 * Calling this function clears the receive data event.
 * 
 * @param baseAddr  base address of the LPI2C module
 * @return  data received by the LPI2C slave receiver
 */
static inline uint8_t LPI2C_HAL_SlaveGetData(LPI2C_Type *baseAddr)
{
    uint32_t tmp = baseAddr->SRDR;
    tmp = (tmp & LPI2C_SRDR_DATA_MASK) >> LPI2C_SRDR_DATA_SHIFT;
    return (uint8_t)tmp;
}


/*!
 * @brief Initializes the LPI2C module to a known state.
 *
 * This function initializes all the registers of the LPI2C module to 
 * their reset value.
 *
 * @param baseAddr  base address of the LPI2C module
 */
void LPI2C_HAL_Init(LPI2C_Type *baseAddr);

/*! @}*/

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __FSL_LPI2C_HAL_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/

