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
#ifndef __FSL_FLEXCAN_HAL_H__
#define __FSL_FLEXCAN_HAL_H__

#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "fsl_device_registers.h"
//#include "S32K144.h"

/*!
 * @defgroup flexcan_hal FlexCAN HAL
 * @ingroup flexcan
 * @addtogroup flexcan_hal
 * @{
 */
  
/*******************************************************************************
 * Definitions
 ******************************************************************************/
enum _flexcan_brs_enable
{
    FLEXCAN_BRS_DISABLE = 0,    /*!< FlexCAN message buffer payload size in bytes*/
    FLEXCAN_BRS_ENABLE          /*!< FlexCAN message buffer payload size in bytes*/

};

enum _flexcan_fd_enable
{
    FLEXCAN_FD_DISABLE = 0,     /*!< FlexCAN message buffer payload size in bytes*/
    FLEXCAN_FD_ENABLE           /*!< FlexCAN message buffer payload size in bytes*/
};

/*! @brief FlexCAN constants*/
enum _flexcan_constants
{
    FLEXCAN_MESSAGE_SIZE = 8,   /*!< FlexCAN message buffer data size in bytes*/
};

/*! @brief The Status enum is used to report current status of the FlexCAN interface.*/
enum _flexcan_err_status
{
    FLEXCAN_RX_WRN   = 0x0080U, /*!< Reached warning level for RX errors*/
    FLEXCAN_TX_WRN   = 0x0100U, /*!< Reached warning level for TX errors*/
    FLEXCAN_STF_ERR  = 0x0200U, /*!< Stuffing Error*/
    FLEXCAN_FRM_ERR  = 0x0400U, /*!< Form Error*/
    FLEXCAN_CRC_ERR  = 0x0800U, /*!< Cyclic Redundancy Check Error*/
    FLEXCAN_ACK_ERR  = 0x1000U, /*!< Received no ACK on transmission*/
    FLEXCAN_BIT0_ERR = 0x2000U, /*!< Unable to send dominant bit*/
    FLEXCAN_BIT1_ERR = 0x4000U  /*!< Unable to send recessive bit*/
};

/*! @brief FlexCAN status return codes*/
typedef enum _flexcan_status
{
    FLEXCAN_STATUS_SUCCESS = 0,
    FLEXCAN_STATUS_OUT_OF_RANGE,
    FLEXCAN_STATUS_UNKNOWN_PROPERTY,
    FLEXCAN_STATUS_INVALID_ARGUMENT,
    FLEXCAN_STATUS_FAIL,
    FLEXCAN_STATUS_TIME_OUT,
    FLEXCAN_STATUS_TX_BUSY,
    FLEXCAN_STATUS_RX_BUSY,
    FLEXCAN_STATUS_NO_TRANSMIT_IN_PROGRESS,
    FLEXCAN_STATUS_NO_RECEIVE_IN_PROGRESS
} flexcan_status_t;


/*! @brief FlexCAN operation modes*/
typedef enum _flexcan_operation_modes {
    FLEXCAN_NORMAL_MODE,        /*!< Normal mode or user mode @internal gui name="Normal" */
    FLEXCAN_LISTEN_ONLY_MODE,   /*!< Listen-only mode @internal gui name="Listen-only" */
    FLEXCAN_LOOPBACK_MODE,      /*!< Loop-back mode @internal gui name="Loop back" */
    FLEXCAN_FREEZE_MODE,        /*!< Freeze mode @internal gui name="Freeze" */
    FLEXCAN_DISABLE_MODE        /*!< Module disable mode @internal gui name="Disabled" */
} flexcan_operation_modes_t;

/*! @brief FlexCAN message buffer CODE for Rx buffers*/
typedef enum _flexcan_msgbuff_code_rx {
    FLEXCAN_RX_INACTIVE  = 0x0, /*!< MB is not active.*/
    FLEXCAN_RX_FULL      = 0x2, /*!< MB is full.*/
    FLEXCAN_RX_EMPTY     = 0x4, /*!< MB is active and empty.*/
    FLEXCAN_RX_OVERRUN   = 0x6, /*!< MB is overwritten into a full buffer.*/
    FLEXCAN_RX_BUSY      = 0x8, /*!< FlexCAN is updating the contents of the MB.*/
                                /*!  The CPU must not access the MB.*/
    FLEXCAN_RX_RANSWER   = 0xA, /*!< A frame was configured to recognize a Remote Request Frame*/
                                /*!  and transmit a Response Frame in return.*/
    FLEXCAN_RX_NOT_USED   = 0xF /*!< Not used*/
} flexcan_msgbuff_code_rx_t;

/*! @brief FlexCAN message buffer CODE FOR Tx buffers*/
typedef enum _flexcan_msgbuff_code_tx {
    FLEXCAN_TX_INACTIVE  = 0x08, /*!< MB is not active.*/
    FLEXCAN_TX_ABORT     = 0x09, /*!< MB is aborted.*/
    FLEXCAN_TX_DATA      = 0x0C, /*!< MB is a TX Data Frame(MB RTR must be 0).*/
    FLEXCAN_TX_REMOTE    = 0x1C, /*!< MB is a TX Remote Request Frame (MB RTR must be 1).*/
    FLEXCAN_TX_TANSWER   = 0x0E, /*!< MB is a TX Response Request Frame from.*/
                                 /*!  an incoming Remote Request Frame.*/
    FLEXCAN_TX_NOT_USED   = 0xF  /*!< Not used*/
} flexcan_msgbuff_code_tx_t;

/*! @brief FlexCAN message buffer transmission types*/
typedef enum _flexcan_msgbuff_transmission_type {
    FLEXCAN_MB_STATUS_TYPE_TX,          /*!< Transmit MB*/
    FLEXCAN_MB_STATUS_TYPE_TX_REMOTE,   /*!< Transmit remote request MB*/
    FLEXCAN_MB_STATUS_TYPE_RX,          /*!< Receive MB*/
    FLEXCAN_MB_STATUS_TYPE_RX_REMOTE,   /*!< Receive remote request MB*/
    FLEXCAN_MB_STATUS_TYPE_RX_TX_REMOTE /*!< FlexCAN remote frame receives remote request and*/
                                        /*!  transmits MB.*/
} flexcan_msgbuff_transmission_type_t;

typedef enum _flexcan_fd_payload_size
{
    FLEXCAN_PAYLOAD_SIZE_8 = 0,  /*!< FlexCAN message buffer payload size in bytes*/
    FLEXCAN_PAYLOAD_SIZE_16 ,    /*!< FlexCAN message buffer payload size in bytes*/
    FLEXCAN_PAYLOAD_SIZE_32 ,    /*!< FlexCAN message buffer payload size in bytes*/
    FLEXCAN_PAYLOAD_SIZE_64      /*!< FlexCAN message buffer payload size in bytes*/
} flexcan_fd_payload_size_t;

typedef enum _flexcan_rx_fifo_id_element_format {
    FLEXCAN_RX_FIFO_ID_FORMAT_A, /*!< One full ID (standard and extended) per ID Filter Table*/
                                 /*!  element.*/
    FLEXCAN_RX_FIFO_ID_FORMAT_B, /*!< Two full standard IDs or two partial 14-bit (standard and*/
                                 /*!  extended) IDs per ID Filter Table element.*/
    FLEXCAN_RX_FIFO_ID_FORMAT_C, /*!< Four partial 8-bit Standard IDs per ID Filter Table*/
                                 /*!  element.*/
    FLEXCAN_RX_FIFO_ID_FORMAT_D  /*!< All frames rejected.*/
} flexcan_rx_fifo_id_element_format_t;

/*! @brief FlexCAN Rx FIFO filters number*/
typedef enum _flexcan_rx_fifo_id_filter_number {
    FLEXCAN_RX_FIFO_ID_FILTERS_8   = 0x0,         /*!<   8 Rx FIFO Filters. @internal gui name="8 Rx FIFO Filters" */
    FLEXCAN_RX_FIFO_ID_FILTERS_16  = 0x1,         /*!<  16 Rx FIFO Filters. @internal gui name="16 Rx FIFO Filters" */
    FLEXCAN_RX_FIFO_ID_FILTERS_24  = 0x2,         /*!<  24 Rx FIFO Filters. @internal gui name="24 Rx FIFO Filters" */
    FLEXCAN_RX_FIFO_ID_FILTERS_32  = 0x3,         /*!<  32 Rx FIFO Filters. @internal gui name="32 Rx FIFO Filters" */
    FLEXCAN_RX_FIFO_ID_FILTERS_40  = 0x4,         /*!<  40 Rx FIFO Filters. @internal gui name="40 Rx FIFO Filters" */
    FLEXCAN_RX_FIFO_ID_FILTERS_48  = 0x5,         /*!<  48 Rx FIFO Filters. @internal gui name="48 Rx FIFO Filters" */
    FLEXCAN_RX_FIFO_ID_FILTERS_56  = 0x6,         /*!<  56 Rx FIFO Filters. @internal gui name="56 Rx FIFO Filters" */
    FLEXCAN_RX_FIFO_ID_FILTERS_64  = 0x7,         /*!<  64 Rx FIFO Filters. @internal gui name="64 Rx FIFO Filters" */
    FLEXCAN_RX_FIFO_ID_FILTERS_72  = 0x8,         /*!<  72 Rx FIFO Filters. @internal gui name="72 Rx FIFO Filters" */
    FLEXCAN_RX_FIFO_ID_FILTERS_80  = 0x9,         /*!<  80 Rx FIFO Filters. @internal gui name="80 Rx FIFO Filters" */
    FLEXCAN_RX_FIFO_ID_FILTERS_88  = 0xA,         /*!<  88 Rx FIFO Filters. @internal gui name="88 Rx FIFO Filters" */
    FLEXCAN_RX_FIFO_ID_FILTERS_96  = 0xB,         /*!<  96 Rx FIFO Filters. @internal gui name="96 Rx FIFO Filters" */
    FLEXCAN_RX_FIFO_ID_FILTERS_104 = 0xC,         /*!< 104 Rx FIFO Filters. @internal gui name="104 Rx FIFO Filters" */
    FLEXCAN_RX_FIFO_ID_FILTERS_112 = 0xD,         /*!< 112 Rx FIFO Filters. @internal gui name="112 Rx FIFO Filters" */
    FLEXCAN_RX_FIFO_ID_FILTERS_120 = 0xE,         /*!< 120 Rx FIFO Filters. @internal gui name="120 Rx FIFO Filters" */
    FLEXCAN_RX_FIFO_ID_FILTERS_128 = 0xF          /*!< 128 Rx FIFO Filters. @internal gui name="128 Rx FIFO Filters" */
} flexcan_rx_fifo_id_filter_num_t;

/*! @brief FlexCAN RX FIFO ID filter table structure*/
typedef struct FLEXCANIdTable {
    bool isRemoteFrame;      /*!< Remote frame*/
    bool isExtendedFrame;    /*!< Extended frame*/
    uint32_t *idFilter;      /*!< Rx FIFO ID filter elements*/
} flexcan_id_table_t;

/*! @brief FlexCAN RX mask type.*/
typedef enum _flexcan_rx_mask_type {
    FLEXCAN_RX_MASK_GLOBAL,      /*!< Rx global mask*/
    FLEXCAN_RX_MASK_INDIVIDUAL   /*!< Rx individual mask*/
} flexcan_rx_mask_type_t;

/*! @brief FlexCAN Message Buffer ID type*/
typedef enum _flexcan_msgbuff_id_type {
    FLEXCAN_MSG_ID_STD,         /*!< Standard ID*/
    FLEXCAN_MSG_ID_EXT          /*!< Extended ID*/
} flexcan_msgbuff_id_type_t;

/*! @brief FlexCAN clock source*/
typedef enum _flexcan_clk_source {
    FLEXCAN_CLK_SOURCE_SOSCDIV2,  /*!< Clock divider 2 for System OSC */
    FLEXCAN_CLK_SOURCE_BUS        /*!< Bus clock */
} flexcan_clk_source_t;

/*! @brief FlexCAN error interrupt types*/
typedef enum _flexcan_int_type {
    FLEXCAN_INT_RX_WARNING = CAN_CTRL1_RWRNMSK_MASK,     /*!< RX warning interrupt*/
    FLEXCAN_INT_TX_WARNING = CAN_CTRL1_TWRNMSK_MASK,     /*!< TX warning interrupt*/
    FLEXCAN_INT_ERR = CAN_CTRL1_ERRMSK_MASK,             /*!< Error interrupt*/
    FLEXCAN_INT_BUSOFF = CAN_CTRL1_BOFFMSK_MASK,         /*!< Bus off interrupt*/
    FLEXCAN_INT_WAKEUP = CAN_MCR_WAKMSK_MASK             /*!< Wake up interrupt*/
} flexcan_int_type_t;

/*! @brief FlexCAN bus error counters*/
typedef struct FLEXCANBuserrCounter {
    uint16_t txerr;           /*!< Transmit error counter*/
    uint16_t rxerr;           /*!< Receive error counter*/
} flexcan_buserr_counter_t;

/*! @brief FlexCAN Message Buffer code and status for transmit and receive */
typedef struct FLEXCANMsgBuffCodeStatus {
    uint32_t code;                        /*!< MB code for TX or RX buffers.*/
                                          /*! Defined by flexcan_mb_code_rx_t and flexcan_mb_code_tx_t */
    flexcan_msgbuff_id_type_t msgIdType;  /*!< Type of message ID (standard or extended)*/
    uint32_t dataLen;                     /*!< Length of Data in Bytes*/
    uint8_t fd_enable;
    uint8_t fd_padding;
    uint8_t enable_brs;                   /* Enable bit rate switch*/    
} flexcan_msgbuff_code_status_t;

/*! @brief FlexCAN message buffer structure*/
typedef struct FLEXCANMsgBuff {
    uint32_t cs;                        /*!< Code and Status*/
    uint32_t msgId;                     /*!< Message Buffer ID*/
    uint8_t data[64];                   /*!< Bytes of the FlexCAN message*/
} flexcan_msgbuff_t;

/*! @brief FlexCAN timing related structures*/
typedef struct FLEXCANTimeSegment {
    uint32_t propSeg;         /*!< Propagation segment*/
    uint32_t phaseSeg1;       /*!< Phase segment 1*/
    uint32_t phaseSeg2;       /*!< Phase segment 2*/
    uint32_t preDivider;      /*!< Clock pre divider*/
    uint32_t rJumpwidth;      /*!< Resync jump width*/
} flexcan_time_segment_t;


/** Added Arvind */

typedef struct MAILBOX 
{
    // flexcan_msgbuff_t mb_info;
    uint32_t mb_msgId;
    uint8_t * mb_payload;  
    uint8_t mb_dlc;
}mailBox_t;

/** End */ 
  
  
/*! @brief FlexCAN timing related structures*/
typedef struct FLEXCANTimeSegmentFd {
    uint32_t propSegFd;       /*!< Propagation segment*/
    uint32_t phaseSeg1Fd;     /*!< Phase segment 1*/
    uint32_t phaseSeg2Fd;     /*!< Phase segment 2*/
    uint32_t preDividerFd;    /*!< Clock pre divider*/
    uint32_t rJumpwidthFd;    /*!< Resync jump width*/
} flexcan_time_segment_cbt_t;
#define RxFifoOcuppiedFirstMsgBuff      6U
#define RxFifoOcuppiedLastMsgBuff(x)    (5 + (x + 1) * 8 / 4)
#define RxFifoFilterElementNum(x)       ((x + 1) * 8)
#define RxFifoFilterTableOffset         0xE0U

#define FlexCanRxFifoAcceptRemoteFrame   1U
#define FlexCanRxFifoAcceptExtFrame      1U

#define CAN_ID_EXT_MASK                          0x3FFFFu
#define CAN_ID_EXT_SHIFT                         0
#define CAN_ID_EXT_WIDTH                         18

#define CAN_ID_STD_MASK                          0x1FFC0000u
#define CAN_ID_STD_SHIFT                         18
#define CAN_ID_STD_WIDTH                         11

#define CAN_ID_PRIO_MASK                         0xE0000000u
#define CAN_ID_PRIO_SHIFT                        29
#define CAN_ID_PRIO_WIDTH                        3
/* CS Bit Fields */
#define CAN_CS_TIME_STAMP_MASK                   0xFFFFu
#define CAN_CS_TIME_STAMP_SHIFT                  0
#define CAN_CS_TIME_STAMP_WIDTH                  16

#define CAN_CS_DLC_MASK                          0xF0000u
#define CAN_CS_DLC_SHIFT                         16
#define CAN_CS_DLC_WIDTH                         4

#define CAN_CS_RTR_MASK                          0x100000u
#define CAN_CS_RTR_SHIFT                         20
#define CAN_CS_RTR_WIDTH                         1

#define CAN_CS_IDE_MASK                          0x200000u
#define CAN_CS_IDE_SHIFT                         21
#define CAN_CS_IDE_WIDTH                         1

#define CAN_CS_SRR_MASK                          0x400000u
#define CAN_CS_SRR_SHIFT                         22
#define CAN_CS_SRR_WIDTH                         1

#define CAN_CS_CODE_MASK                         0xF000000u
#define CAN_CS_CODE_SHIFT                        24
#define CAN_CS_CODE_WIDTH                        4

#define CAN_MB_EDL_MASK                          0x80000000u    
#define CAN_MB_BRS_MASK                          0x40000000u    

/* CAN FD extended data length DLC encoding */
#define CAN_DLC_VALUE_12_BYTES                   9U
#define CAN_DLC_VALUE_16_BYTES                   10U
#define CAN_DLC_VALUE_20_BYTES                   11U
#define CAN_DLC_VALUE_24_BYTES                   12U
#define CAN_DLC_VALUE_32_BYTES                   13U
#define CAN_DLC_VALUE_48_BYTES                   14U
#define CAN_DLC_VALUE_64_BYTES                   15U
                                                 

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
 * @brief Enables FlexCAN controller.
 *
 * @param   base    The FlexCAN base address
 * @return  0 if successful; non-zero failed
 */
flexcan_status_t FLEXCAN_HAL_Enable(CAN_Type * base);

/*!
 * @brief Disables FlexCAN controller.
 *
 * @param   base    The FlexCAN base address
 * @return  0 if successful; non-zero failed
 */
flexcan_status_t FLEXCAN_HAL_Disable(CAN_Type * base);

/*!
 * @brief Selects the clock source for FlexCAN.
 *
 * @param   base The FlexCAN base address
 * @param   clk         The FlexCAN clock source
 * @return  0 if successful; non-zero failed
 */
flexcan_status_t FLEXCAN_HAL_SelectClock(CAN_Type * base, flexcan_clk_source_t clk);

/*!
 * @brief Reads the clock source for FlexCAN Protocol Engine (PE).
 *
 * @param   base The FlexCAN base address
 * @return  0: if clock source is oscillator clock, 1: if clock source is peripheral clock
 */
static inline bool FLEXCAN_HAL_GetClock(CAN_Type * base)
{
    return (BITBAND_ACCESS32(&(base->CTRL1), CAN_CTRL1_CLKSRC_SHIFT));
}

/*!
 * @brief Initializes the FlexCAN controller.
 *
 * @param   base  The FlexCAN base address
 * @return  0 if successful; non-zero failed
 */
flexcan_status_t FLEXCAN_HAL_Init(CAN_Type * base);

/*!
 * @brief Sets the FlexCAN time segments for setting up bit rate.
 *
 * @param   base The FlexCAN base address
 * @param   timeSeg    FlexCAN time segments, which need to be set for the bit rate.
 * @return  0 if successful; non-zero failed
 */
void FLEXCAN_HAL_SetTimeSegments(CAN_Type * base, flexcan_time_segment_t *timeSeg);


/*!
 * @brief Sets the FlexCAN time segments for setting up bit rate for FD BRS.
 *
 * @param   base The FlexCAN base address
 * @param   timeSeg    FlexCAN time segments, which need to be set for the bit rate.
 * @return  0 if successful; non-zero failed
 */
void FLEXCAN_HAL_SetTimeSegmentsCbt(CAN_Type * base, flexcan_time_segment_cbt_t *timeSeg);

/*!
 * @brief Gets the  FlexCAN time segments to calculate the bit rate.
 *
 * @param   base The FlexCAN base address
 * @param   timeSeg    FlexCAN time segments read for bit rate
 * @return  0 if successful; non-zero failed
 */
void FLEXCAN_HAL_GetTimeSegments(CAN_Type * base, flexcan_time_segment_t *timeSeg);

/*!
 * @brief Un freezes the FlexCAN module.
 *
 * @param   base     The FlexCAN base address
 * @return  0 if successful; non-zero failed.
 */
void FLEXCAN_HAL_ExitFreezeMode(CAN_Type * base);

/*!
 * @brief Freezes the FlexCAN module.
 *
 * @param   base     The FlexCAN base address
 */
void FLEXCAN_HAL_EnterFreezeMode(CAN_Type * base);

/*!
 * @brief Set operation mode.
 *
 * @param   base  The FlexCAN base address
 * @param   mode  Set an operation mode
 * @return  0 if successful; non-zero failed.
 */
flexcan_status_t FLEXCAN_HAL_SetOperationMode(
    CAN_Type * base,
    flexcan_operation_modes_t mode);

/*!
 * @brief Exit operation mode.
 *
 * @param   base  The FlexCAN base address
 * @param   mode  Exit An operation mode
 * @return  0 if successful; non-zero failed.
 */
flexcan_status_t FLEXCAN_HAL_ExitOperationMode(
    CAN_Type * base,
    flexcan_operation_modes_t mode);

/*!
 * @brief Enables/Disables Flexible Data rate (if supported).
 *
 * @param   base    The FlexCAN base address
 * @param   enable  true to enable; false to disable
 */
void FLEXCAN_HAL_SetFDEnabled(CAN_Type * base, bool enable);

/*!
 * @brief Checks if the Flexible Data rate feature is enabled.
 *
 * @param   base    The FlexCAN base address
 * @return  true if enabled; false if disabled
 */
bool FLEXCAN_HAL_IsFDEnabled(CAN_Type * base);

/*!
 * @brief Sets the payload size of the MBs.
 *
 * @param   base         The FlexCAN base address
 * @param   payloadSize  The payload size
 */
flexcan_status_t FLEXCAN_HAL_SetPayloadSize(
    CAN_Type * base,
    flexcan_fd_payload_size_t payloadSize);

/*!
 * @brief Gets the payload size of the MBs.
 *
 * @param   base         The FlexCAN base address
 * @return  The payload size in bytes
 */
uint8_t FLEXCAN_HAL_GetPayloadSize(CAN_Type * base);

/*@}*/

/*!
 * @name Data transfer
 * @{
 */

/*!
 * @brief Sets the FlexCAN message buffer fields for transmitting.
 *
 * @param   base  The FlexCAN base address
 * @param   msgBuffIdx       Index of the message buffer
 * @param   cs           CODE/status values (TX)
 * @param   msgId       ID of the message to transmit
 * @param   msgData      Bytes of the FlexCAN message
 * @return  0 if successful; non-zero failed
 */
flexcan_status_t FLEXCAN_HAL_SetTxMsgBuff(
    CAN_Type * base,
    uint32_t msgBuffIdx,
    flexcan_msgbuff_code_status_t *cs,
    uint32_t msgId,
    uint8_t *msgData);

/*!
 * @brief Sets the FlexCAN message buffer fields for receiving.
 *
 * @param   base  The FlexCAN base address
 * @param   msgBuffIdx       Index of the message buffer
 * @param   cs           CODE/status values (RX)
 * @param   msgId       ID of the message to receive
 * @return  0 if successful; non-zero failed
 */
flexcan_status_t FLEXCAN_HAL_SetRxMsgBuff(
    CAN_Type * base,
    uint32_t msgBuffIdx,
    flexcan_msgbuff_code_status_t *cs,
    uint32_t msgId);

/*!
 * @brief Gets the FlexCAN message buffer fields.
 *
 * @param   base  The FlexCAN base address
 * @param   msgBuffIdx       Index of the message buffer
 * @param   msgBuff           The fields of the message buffer
 * @return  0 if successful; non-zero failed
 */
flexcan_status_t FLEXCAN_HAL_GetMsgBuff(
    CAN_Type * base,
    uint32_t msgBuffIdx,
    flexcan_msgbuff_t *msgBuff);

/*!
 * @brief Locks the FlexCAN Rx message buffer.
 *
 * @param   base  The FlexCAN base address
 * @param   msgBuffIdx       Index of the message buffer
 * @return  0 if successful; non-zero failed
 */
flexcan_status_t FLEXCAN_HAL_LockRxMsgBuff(
    CAN_Type * base,
    uint32_t msgBuffIdx);

/*!
 * @brief Unlocks the FlexCAN Rx message buffer.
 *
 * @param   base     The FlexCAN base address
 * @return  0 if successful; non-zero failed
 */
static inline uint32_t FLEXCAN_HAL_UnlockRxMsgBuff(CAN_Type * base)
{
    uint32_t tmp;
    /* Unlock the mailbox */
    tmp =  (base->TIMER);
    return tmp;
}

/*!
 * @brief Enables the Rx FIFO.
 *
 * @param   base     The FlexCAN base address
 * @param   numOfFilters    The number of Rx FIFO filters
 */
void FLEXCAN_HAL_EnableRxFifo(CAN_Type * base, uint32_t numOfFilters);

/*!
 * @brief Disables the Rx FIFO.
 *
 * @param   base     The FlexCAN base address
 */
void FLEXCAN_HAL_DisableRxFifo(CAN_Type * base);

/*!
 * @brief Sets the number of the Rx FIFO filters.
 *
 * @param   base  The FlexCAN base address
 * @param   number       The number of Rx FIFO filters
 */
void FLEXCAN_HAL_SetRxFifoFilterNum(CAN_Type * base, uint32_t number);

/*!
 * @brief Sets  the maximum number of Message Buffers.
 *
 * @param   base  The FlexCAN base address
 * @param   maxMsgBuffNum     Maximum number of message buffers
 */
void FLEXCAN_HAL_SetMaxMsgBuffNum(
    CAN_Type * base,
    uint32_t maxMsgBuffNum);

/*!
 * @brief Sets the FlexCAN Rx FIFO fields.
 *
 * @param   base             The FlexCAN base address
 * @param   idFormat               The format of the Rx FIFO ID Filter Table Elements
 * @param   idFilterTable         The ID filter table elements which contain RTR bit, IDE bit,
 *                                  and RX message ID.
 * @return  0 if successful; non-zero failed.
 */
flexcan_status_t FLEXCAN_HAL_SetRxFifoFilter(
    CAN_Type * base,
    flexcan_rx_fifo_id_element_format_t idFormat,
    flexcan_id_table_t *idFilterTable);

/*!
 * @brief Gets the FlexCAN Rx FIFO data.
 *
 * @param   base  The FlexCAN base address
 * @param   rxFifo      The FlexCAN receive FIFO data
 * @return  0 if successful; non-zero failed.
 */
flexcan_status_t FLEXCAN_HAL_ReadRxFifo(
    CAN_Type * base,
    flexcan_msgbuff_t *rxFifo);

/*@}*/

/*!
 * @name Interrupts
 * @{
 */

/*!
 * @brief Enables/Disables the FlexCAN Message Buffer interrupt.
 *
 * @param   base  The FlexCAN base address
 * @param   msgBuffIdx       Index of the message buffer
 * @param   enable       choose enable or disable
 * @return  0 if successful; non-zero failed
 */
flexcan_status_t FLEXCAN_HAL_SetMsgBuffIntCmd(
    CAN_Type * base,
    uint32_t msgBuffIdx, bool enable);

/*!
 * @brief Enables error interrupt of the FlexCAN module.
 * @param   base     The FlexCAN base address
 * @param   errType     The interrupt type
 * @param   enable       choose enable or disable
 */
void FLEXCAN_HAL_SetErrIntCmd(CAN_Type * base, flexcan_int_type_t errType, bool enable);

/*@}*/

/*!
 * @name Status
 * @{
 */

/*!
 * @brief Gets the value of FlexCAN freeze ACK.
 *
 * @param   base     The FlexCAN base address
 * @return  freeze ACK state (1-freeze mode, 0-not in freeze mode).
 */
static inline uint32_t FLEXCAN_HAL_GetFreezeAck(CAN_Type * base)
{
    return (BITBAND_ACCESS32(&(base->MCR), CAN_MCR_FRZACK_SHIFT));
}

//Arvind
volatile uint32_t* FLEXCAN_HAL_GetMsgBuffRegion(
        CAN_Type * base,
        uint32_t msgBuffIdx);
uint8_t FLEXCAN_HAL_ComputePayloadSize(
        uint8_t dlcValue);
/*!
 * @brief Gets the individual FlexCAN MB interrupt flag.
 *
 * @param   base  The FlexCAN base address
 * @param   msgBuffIdx       Index of the message buffer
 * @return  the individual Message Buffer interrupt flag (0 and 1 are the flag value)
 */
uint8_t FLEXCAN_HAL_GetMsgBuffIntStatusFlag(
    CAN_Type * base,
    uint32_t msgBuffIdx);

/*!
 * @brief Gets all FlexCAN Message Buffer interrupt flags.
 *
 * @param   base     The FlexCAN base address
 * @return  all MB interrupt flags
 */
static inline uint32_t FLEXCAN_HAL_GetAllMsgBuffIntStatusFlag(CAN_Type * base)
{
    return (base->IFLAG1);
}

/*!
 * @brief Clears the interrupt flag of the message buffers.
 *
 * @param   base  The FlexCAN base address
 * @param   flag      The value to be written to the interrupt flag1 register.
 */
/* See fsl_flexcan_hal.h for documentation of this function.*/
static inline void FLEXCAN_HAL_ClearMsgBuffIntStatusFlag(
    CAN_Type * base,
    uint32_t flag)
{
    /* Clear the corresponding message buffer interrupt flag*/
    (base->IFLAG1) = (flag);
}

/*!
 * @brief Gets the transmit error counter and receives the error counter.
 *
 * @param   base  The FlexCAN base address
 * @param   errCount      Transmit error counter and receive error counter
 */
void FLEXCAN_HAL_GetErrCounter(
    CAN_Type * base,
    flexcan_buserr_counter_t *errCount);

/*!
 * @brief Gets error and status.
 *
 * @param   base     The FlexCAN base address
 * @return  The current error and status
 */
static inline uint32_t FLEXCAN_HAL_GetErrStatus(CAN_Type * base)
{
    return (base->ESR1);
}

/*!
 * @brief Clears all other interrupts in ERRSTAT register (Error, Busoff, Wakeup).
 *
 * @param   base     The FlexCAN base address
 */
void FLEXCAN_HAL_ClearErrIntStatusFlag(CAN_Type * base);

/*@}*/

/*!
 * @name Mask
 * @{
 */

/*!
 * @brief Sets the Rx masking type.
 *
 * @param   base  The FlexCAN base address
 * @param   type         The FlexCAN Rx mask type
 */
void FLEXCAN_HAL_SetRxMaskType(CAN_Type * base, flexcan_rx_mask_type_t type);

/*!
 * @brief Sets the FlexCAN RX FIFO global standard mask.
 *
 * @param   base  The FlexCAN base address
 * @param   stdMask     Standard mask
 */
void FLEXCAN_HAL_SetRxFifoGlobalStdMask(
    CAN_Type * base,
    uint32_t stdMask);

/*!
 * @brief Sets the FlexCAN Rx FIFO global extended mask.
 *
 * @param   base  The FlexCAN base address
 * @param   extMask     Extended mask
 */
void FLEXCAN_HAL_SetRxFifoGlobalExtMask(
    CAN_Type * base,
    uint32_t extMask);

/*!
 * @brief Sets the FlexCAN Rx individual standard mask for ID filtering in the Rx MBs and the Rx FIFO.
 *
 * @param   base  The FlexCAN base address
 * @param   msgBuffIdx       Index of the message buffer
 * @param   stdMask     Individual standard mask
 * @return  0 if successful; non-zero failed
 */
flexcan_status_t FLEXCAN_HAL_SetRxIndividualStdMask(
    CAN_Type * base,
    uint32_t msgBuffIdx,
    uint32_t stdMask);

/*!
 * @brief Sets the FlexCAN Rx individual extended mask for ID filtering in the Rx Message Buffers and the Rx FIFO.
 *
 * @param   base  The FlexCAN base address
 * @param   msgBuffIdx       Index of the message buffer
 * @param   extMask     Individual extended mask
 * @return  0 if successful; non-zero failed
 */
flexcan_status_t FLEXCAN_HAL_SetRxIndividualExtMask(
    CAN_Type * base,
    uint32_t msgBuffIdx,
    uint32_t extMask);

/*!
 * @brief Sets the FlexCAN Rx Message Buffer global standard mask.
 *
 * @param   base  The FlexCAN base address
 * @param   stdMask     Standard mask
 */
void FLEXCAN_HAL_SetRxMsgBuffGlobalStdMask(
    CAN_Type * base,
    uint32_t stdMask);

/*!
 * @brief Sets the FlexCAN RX Message Buffer BUF14 standard mask.
 *
 * @param   base  The FlexCAN base address
 * @param   stdMask     Standard mask
 */
void FLEXCAN_HAL_SetRxMsgBuff14StdMask(
    CAN_Type * base,
    uint32_t stdMask);

/*!
 * @brief Sets the FlexCAN Rx Message Buffer BUF15 standard mask.
 *
 * @param   base  The FlexCAN base address
 * @param   stdMask     Standard mask
 * @return  0 if successful; non-zero failed
 */
void FLEXCAN_HAL_SetRxMsgBuff15StdMask(
    CAN_Type * base,
    uint32_t stdMask);

/*!
 * @brief Sets the FlexCAN RX Message Buffer global extended mask.
 *
 * @param   base  The FlexCAN base address
 * @param   extMask     Extended mask
 */
void FLEXCAN_HAL_SetRxMsgBuffGlobalExtMask(
    CAN_Type * base,
    uint32_t extMask);

/*!
 * @brief Sets the FlexCAN RX Message Buffer BUF14 extended mask.
 *
 * @param   base  The FlexCAN base address
 * @param   extMask     Extended mask
 */
void FLEXCAN_HAL_SetRxMsgBuff14ExtMask(
    CAN_Type * base,
    uint32_t extMask);

/*!
 * @brief Sets the FlexCAN RX MB BUF15 extended mask.
 *
 * @param   base  The FlexCAN base address
 * @param   extMask     Extended mask
 */
void FLEXCAN_HAL_SetRxMsgBuff15ExtMask(
    CAN_Type * base,
    uint32_t extMask);

/*!
 * @brief Gets the FlexCAN ID acceptance filter hit indicator on Rx FIFO.
 *
 * @param   base  The FlexCAN base address
 * @return  RX FIFO information
 */
static inline uint32_t  FLEXCAN_HAL_GetRxFifoHitIdAcceptanceFilter(CAN_Type * base)
{
    return ((((base)->RXFIR) & CAN_RXFIR_IDHIT_MASK) >> CAN_RXFIR_IDHIT_SHIFT);
}

/** Arvind added */
extern flexcan_status_t FLEXCAN_HAL_GetMailBox( CAN_Type * base, uint32_t msgBuffIdx, \
                        flexcan_msgbuff_t *msgBuff, mailBox_t *mailBox);
/** end */
/*@}*/

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif

/*******************************************************************************
 * EOF
 ******************************************************************************/
