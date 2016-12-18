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
#include "fsl_flexcan_hal.h"


/*******************************************************************************
 * Variables
 ******************************************************************************/
   extern uint8_t can_real_payload;
   extern uint8_t can_payload[4u];
    
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATAB_RTR_SHIFT  (31U)          /*!< FlexCAN RX FIFO ID filter*/
                                                                     /*! format A&B RTR mask.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATAB_IDE_SHIFT  (30U)          /*!< FlexCAN RX FIFO ID filter*/
                                                                     /*! format A&B IDE mask.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_RTR_SHIFT  (15U)           /*!< FlexCAN RX FIFO ID filter*/
                                                                     /*! format B RTR-2 mask.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_IDE_SHIFT  (14U)           /*!< FlexCAN RX FIFO ID filter*/
                                                                     /*! format B IDE-2 mask.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATA_EXT_MASK    (0x3FFFFFFFU)  /*!< FlexCAN RX FIFO ID filter*/
                                                                     /*! format A extended mask.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATA_EXT_SHIFT   (1U)           /*!< FlexCAN RX FIFO ID filter*/
                                                                     /*! format A extended shift.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATA_STD_MASK    (0x3FF80000U)  /*!< FlexCAN RX FIFO ID filter*/
                                                                     /*! format A standard mask.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATA_STD_SHIFT   (19U)          /*!< FlexCAN RX FIFO ID filter*/
                                                                     /*! format A standard shift.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_EXT_MASK    (0x3FFFU)      /*!< FlexCAN RX FIFO ID filter*/
                                                                     /*! format B extended mask.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_EXT_SHIFT1  (16U)          /*!< FlexCAN RX FIFO ID filter*/
                                                                     /*! format B extended mask.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_EXT_SHIFT2  (0U)           /*!< FlexCAN RX FIFO ID filter*/
                                                                     /*! format B extended mask.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_STD_MASK    (0x7FFU)       /*!< FlexCAN RX FIFO ID filter*/
                                                                     /*! format B standard mask.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_STD_SHIFT1  (19U)          /*!< FlexCAN RX FIFO ID filter*/
                                                                     /*! format B standard shift1.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_STD_SHIFT2  (3U)           /*!< FlexCAN RX FIFO ID filter*/
                                                                     /*! format B standard shift2.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_MASK        (0xFFU)        /*!< FlexCAN RX FIFO ID filter*/
                                                                     /*! format C mask.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_SHIFT1      (24U)          /*!< FlexCAN RX FIFO ID filter*/
                                                                     /*! format C shift1.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_SHIFT2      (16U)          /*!< FlexCAN RX FIFO ID filter*/
                                                                     /*! format C shift2.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_SHIFT3      (8U)           /*!< FlexCAN RX FIFO ID filter*/
                                                                     /*! format C shift3.*/
#define FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_SHIFT4      (0U)           /*!< FlexCAN RX FIFO ID filter*/
                                                                     /*! format C shift4.*/
#define FLEXCAN_ALL_INT                               (0x0007U)      /*!< Masks for wakeup, error, bus off*/
                                                                     /*! interrupts*/
#define FLEXCAN_BYTE_DATA_FIELD_MASK                  (0xFFU)        /*!< Masks for byte data field.*/

/* Converts the index in an array of bytes to an index in an array of bytes with 
 * reversed endianness. */
#define FLEXCAN_SWAP_BYTES_IN_WORD_INDEX(index)       (((index) & ~3U) + (3U - ((index) & 3U)))

/*FD constant with all available payload values*/

/*******************************************************************************
 * Private Functions
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_GetMsgBuffRegion
 * Description   : Returns the start of a MB area, based on its index.
 *
 *END**************************************************************************/
volatile uint32_t* FLEXCAN_HAL_GetMsgBuffRegion(
        CAN_Type * base,
        uint32_t msgBuffIdx)
{
    uint8_t payload_size = FLEXCAN_HAL_GetPayloadSize(base);
    uint8_t arbitration_field_size = 8U;

    uint8_t mb_size = payload_size + arbitration_field_size;
    /* Multiply the MB index by the MB size (in words) */
    uint8_t mb_index = msgBuffIdx * (mb_size >> 2);

    return &(base->RAMn[mb_index]);
}

/*FUNCTION**********************************************************************
 *
 * Function Name: FLEXCAN_HAL_ComputeDLCValue
 * Description  : Computes the DLC field value, given a payload size (in bytes).
 *
 *END**************************************************************************/
uint8_t FLEXCAN_HAL_ComputeDLCValue(
        uint8_t payloadSize)
{
    if (payloadSize <= 8)
        return payloadSize;
    else if ((payloadSize > 8) && (payloadSize <= 12))
        return CAN_DLC_VALUE_12_BYTES;
    else if ((payloadSize > 12) && (payloadSize <= 16))
        return CAN_DLC_VALUE_16_BYTES;
    else if ((payloadSize > 16) && (payloadSize <= 20))
        return CAN_DLC_VALUE_20_BYTES;
    else if ((payloadSize > 20) && (payloadSize <= 24))
        return CAN_DLC_VALUE_24_BYTES;
    else if ((payloadSize > 24) && (payloadSize <= 32))
        return CAN_DLC_VALUE_32_BYTES;
    else if ((payloadSize > 32) && (payloadSize <= 48))
        return CAN_DLC_VALUE_48_BYTES;
    else if ((payloadSize > 48) && (payloadSize <= 64))
        return CAN_DLC_VALUE_64_BYTES;

    /* The argument is not a valid payload size */
    return 0xFF;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_ComputePayloadSize
 * Description   : Computes the maximum payload size (in bytes), given a DLC
 * field value.
 *
 *END**************************************************************************/
uint8_t FLEXCAN_HAL_ComputePayloadSize(
        uint8_t dlcValue)
{
    if (dlcValue <= 8)
        return dlcValue;

    switch (dlcValue) {
    case CAN_DLC_VALUE_12_BYTES:
        return 12;
    case CAN_DLC_VALUE_16_BYTES:
        return 16;
    case CAN_DLC_VALUE_20_BYTES:
        return 20;
    case CAN_DLC_VALUE_24_BYTES:
        return 24;
    case CAN_DLC_VALUE_32_BYTES:
        return 32;
    case CAN_DLC_VALUE_48_BYTES:
        return 48;
    case CAN_DLC_VALUE_64_BYTES:
        return 64;
    default:
        /* The argument is not a valid DLC value */
        return 0xFF;
    }
}

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_Enable
 * Description   : Enable FlexCAN module.
 * This function will enable FlexCAN module.
 *
 *END**************************************************************************/
flexcan_status_t FLEXCAN_HAL_Enable(CAN_Type * base)
{
    /* Check for low power mode*/
    if((BITBAND_ACCESS32(&(base->MCR), CAN_MCR_LPMACK_SHIFT))==0x1)
    {
        /* Enable clock*/
         BITBAND_ACCESS32(&(base->MCR), CAN_MCR_MDIS_SHIFT) = (0x0);
         BITBAND_ACCESS32(&(base->MCR), CAN_MCR_FRZ_SHIFT) = (0x0);
         BITBAND_ACCESS32(&(base->MCR), CAN_MCR_HALT_SHIFT) = (0x0);
        /* Wait until enabled*/
        while (BITBAND_ACCESS32( &(base->MCR), CAN_MCR_LPMACK_SHIFT)!=0x0){}
    }

    return FLEXCAN_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_Disable
 * Description   : Disable FlexCAN module.
 * This function will disable FlexCAN module.
 *
 *END**************************************************************************/
flexcan_status_t FLEXCAN_HAL_Disable(CAN_Type * base)
{
    /* To access the memory mapped registers*/
    /* Entre disable mode (hard reset).*/
    if((BITBAND_ACCESS32(&(base->MCR), CAN_MCR_MDIS_SHIFT)) == 0x0)
    {
        /* Clock disable (module)*/
        BITBAND_ACCESS32(&(base->MCR), CAN_MCR_MDIS_SHIFT) = (0x1);

        /* Wait until disable mode acknowledged*/
        while (!(BITBAND_ACCESS32(&(base->MCR), CAN_MCR_LPMACK_SHIFT))){}
    }

    return FLEXCAN_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SelectClock
 * Description   : Select FlexCAN clock source.
 * This function will select either internal bus clock or external clock as
 * FlexCAN clock source.
 *
 *END**************************************************************************/
flexcan_status_t FLEXCAN_HAL_SelectClock(
    CAN_Type * base,
    flexcan_clk_source_t clk)
{
    BITBAND_ACCESS32(&(base->CTRL1), CAN_CTRL1_CLKSRC_SHIFT) = (clk);
    return FLEXCAN_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_Init
 * Description   : Initialize FlexCAN module.
 * This function will reset FlexCAN module, set maximum number of message
 * buffers, initialize all message buffers as inactive, enable RX FIFO
 * if needed, mask all mask bits, and disable all MB interrupts.
 *
 *END**************************************************************************/
flexcan_status_t FLEXCAN_HAL_Init(CAN_Type * base)
{

    /* Reset the FLEXCAN*/
    BITBAND_ACCESS32(&(base->MCR), CAN_MCR_SOFTRST_SHIFT) = (0x1);

    /* Wait for reset cycle to complete*/
    while (BITBAND_ACCESS32(&(base->MCR), CAN_MCR_SOFTRST_SHIFT)){}

    /* Set Freeze, Halt*/
    FLEXCAN_HAL_EnterFreezeMode(base);

    /* Rx global mask*/
    (base->RXMGMASK) = (((uint32_t)(((uint32_t)(CAN_RXMGMASK_MG_MASK)) << CAN_ID_EXT_SHIFT)) & (CAN_ID_STD_MASK | CAN_ID_EXT_MASK));

    /* Rx reg 14 mask*/
    (base->RX14MASK) =  (((uint32_t)(((uint32_t)(CAN_RX14MASK_RX14M_MASK)) << CAN_ID_EXT_SHIFT)) & (CAN_ID_STD_MASK | CAN_ID_EXT_MASK));

    /* Rx reg 15 mask*/
    (base->RX15MASK) = (((uint32_t)(((uint32_t)(CAN_RX15MASK_RX15M_MASK)) << CAN_ID_EXT_SHIFT)) & (CAN_ID_STD_MASK | CAN_ID_EXT_MASK));

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(base);

    /* Disable all MB interrupts*/
    (base->IMASK1) = 0x0;

    return FLEXCAN_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetTimeSegments
 * Description   : Set FlexCAN time segments.
 * This function will set all FlexCAN time segments which define the length of
 * Propagation Segment in the bit time, the length of Phase Buffer Segment 2 in
 * the bit time, the length of Phase Buffer Segment 1 in the bit time, the ratio
 * between the PE clock frequency and the Serial Clock (Sclock) frequency, and
 * the maximum number of time quanta that a bit time can be changed by one
 * resynchronization. (One time quantum is equal to the Sclock period.)
 *
 *END**************************************************************************/
void FLEXCAN_HAL_SetTimeSegments(
    CAN_Type * base,
    flexcan_time_segment_t *timeSeg)
{
    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(base);
    /* Set FlexCAN time segments*/
    (base->CTRL1) = ((base->CTRL1) & ~((CAN_CTRL1_PROPSEG_MASK | CAN_CTRL1_PSEG2_MASK |
                                CAN_CTRL1_PSEG1_MASK | CAN_CTRL1_PRESDIV_MASK) |
                                CAN_CTRL1_RJW_MASK));

    (base->CTRL1) = ((base->CTRL1) | (CAN_CTRL1_PROPSEG(timeSeg->propSeg) |
                                CAN_CTRL1_PSEG2(timeSeg->phaseSeg2) |
                                CAN_CTRL1_PSEG1(timeSeg->phaseSeg1) |
                                CAN_CTRL1_PRESDIV(timeSeg->preDivider) |
                                CAN_CTRL1_RJW(timeSeg->rJumpwidth)));

    /* De-assert Freeze mode*/
     FLEXCAN_HAL_ExitFreezeMode(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetTimeSegments
 * Description   : Set FlexCAN time segments.
 * This function will set all FlexCAN time segments which define the length of
 * Propagation Segment in the bit time, the length of Phase Buffer Segment 2 in
 * the bit time, the length of Phase Buffer Segment 1 in the bit time, the ratio
 * between the PE clock frequency and the Serial Clock (Sclock) frequency, and
 * the maximum number of time quanta that a bit time can be changed by one
 * resynchronization. (One time quantum is equal to the Sclock period.)
 *
 *END**************************************************************************/
void FLEXCAN_HAL_SetTimeSegmentsCbt(
    CAN_Type * base,
    flexcan_time_segment_cbt_t *timeSeg)
{
    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(base);
    /* Set FlexCAN time segments*/
    (base->FDCBT) = ((base->FDCBT) & ~((CAN_FDCBT_FPROPSEG_MASK | CAN_FDCBT_FPSEG2_MASK |
                                CAN_FDCBT_FPSEG1_MASK | CAN_FDCBT_FPRESDIV_MASK) |
                                CAN_FDCBT_FRJW_MASK));

    (base->FDCBT) = ((base->FDCBT) | (CAN_FDCBT_FPROPSEG(timeSeg->propSegFd) |
                                CAN_FDCBT_FPSEG2(timeSeg->phaseSeg2Fd) |
                                CAN_FDCBT_FPSEG1(timeSeg->phaseSeg1Fd) |
                                CAN_FDCBT_FPRESDIV(timeSeg->preDividerFd) |
                                CAN_FDCBT_FRJW(timeSeg->rJumpwidthFd)));

    /* De-assert Freeze mode*/
     FLEXCAN_HAL_ExitFreezeMode(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_GetTimeSegments
 * Description   : Get FlexCAN time segments.
 * This function will get all FlexCAN time segments defined.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_GetTimeSegments(
    CAN_Type * base,
    flexcan_time_segment_t *timeSeg)
{
    timeSeg->preDivider = ( (base->CTRL1) & CAN_CTRL1_PRESDIV_MASK) >> CAN_CTRL1_PRESDIV_SHIFT;
    timeSeg->propSeg = ((base->CTRL1) & CAN_CTRL1_PROPSEG_MASK) >> CAN_CTRL1_PROPSEG_SHIFT;
    timeSeg->phaseSeg1 = ((base->CTRL1) & CAN_CTRL1_PSEG1_MASK) >> CAN_CTRL1_PSEG1_SHIFT;
    timeSeg->phaseSeg2 = ((base->CTRL1) & CAN_CTRL1_PSEG2_MASK) >> CAN_CTRL1_PSEG2_SHIFT;
    timeSeg->rJumpwidth = ((base->CTRL1) & CAN_CTRL1_RJW_MASK) >> CAN_CTRL1_RJW_SHIFT;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetTxMsgBuff
 * Description   : Configure a message buffer for transmission.
 * This function will first check if RX FIFO is enabled. If RX FIFO is enabled,
 * the function will make sure if the MB requested is not occupied by RX FIFO
 * and ID filter table. Then this function will copy user's buffer into the
 * message buffer data area and configure the message buffer as required for
 * transmission.
 *
 *END**************************************************************************/
flexcan_status_t FLEXCAN_HAL_SetTxMsgBuff(
    CAN_Type * base,
    uint32_t msgBuffIdx,
    flexcan_msgbuff_code_status_t *cs,
    uint32_t msgId,
    uint8_t *msgData)
{
    uint32_t val1, val2 = 1;   
    uint32_t flexcan_mb_config = 0;    
    uint8_t databyte;
    uint8_t dlc_value;
    
    volatile uint32_t *flexcan_mb = FLEXCAN_HAL_GetMsgBuffRegion(base, msgBuffIdx);

    volatile uint32_t *flexcan_mb_id   = flexcan_mb + 1;
    volatile uint8_t  *flexcan_mb_data = (uint8_t *)(flexcan_mb + 2);
        
    if (msgBuffIdx >= (((base->MCR) & CAN_MCR_MAXMB_MASK) >> CAN_MCR_MAXMB_SHIFT) )
    {
        return FLEXCAN_STATUS_OUT_OF_RANGE;
    }

    /* Check if RX FIFO is enabled*/
    if (((base->MCR) & CAN_MCR_RFEN_MASK) >> CAN_MCR_RFEN_SHIFT)
    {
        /* Get the number of RX FIFO Filters*/
        val1 = (((base->CTRL2) & CAN_CTRL2_RFFN_MASK) >> CAN_CTRL2_RFFN_SHIFT);
        /* Get the number if MBs occupied by RX FIFO and ID filter table*/
        /* the Rx FIFO occupies the memory space originally reserved for MB0-5*/
        /* Every number of RFFN means 8 number of RX FIFO filters*/
        /* and every 4 number of RX FIFO filters occupied one MB*/
        val2 = RxFifoOcuppiedLastMsgBuff(val1);

        if (msgBuffIdx <= val2)
        {
            return FLEXCAN_STATUS_INVALID_ARGUMENT;
        }
    }

    /* Make sure the BRS bit will not be ignored */
    if (FLEXCAN_HAL_IsFDEnabled(base) && cs->enable_brs)
    {
       BITBAND_ACCESS32(&(base->FDCTRL), CAN_FDCTRL_FDRATE_SHIFT) = 1;
    }

    /* Compute the value of the DLC field */
    dlc_value = FLEXCAN_HAL_ComputeDLCValue(cs->dataLen);

    /* Copy user's buffer into the message buffer data area */
    if (msgData != NULL)
    {
        uint8_t payload_size = FLEXCAN_HAL_ComputePayloadSize(dlc_value);
        for (databyte = 0; databyte < cs->dataLen; databyte++)
        {
            flexcan_mb_data[FLEXCAN_SWAP_BYTES_IN_WORD_INDEX(databyte)] =  msgData[databyte];
        }
        /* Add padding, if needed */
        for (databyte = cs->dataLen; databyte < payload_size; databyte++)
        {
            flexcan_mb_data[FLEXCAN_SWAP_BYTES_IN_WORD_INDEX(databyte)] = cs->fd_padding;
        }
    }

    /* Clean up the arbitration field area */
    *flexcan_mb = 0;
    *flexcan_mb_id = 0;
    
    /* Set the ID according the format structure */
    if (cs->msgIdType == FLEXCAN_MSG_ID_EXT)
    {
        /* ID [28-0]*/
        *flexcan_mb_id &= ~(CAN_ID_STD_MASK | CAN_ID_EXT_MASK);
        *flexcan_mb_id |= (msgId & (CAN_ID_STD_MASK | CAN_ID_EXT_MASK));
      
        /* Set IDE*/
        flexcan_mb_config |= CAN_CS_IDE_MASK;

        /* Clear SRR bit*/
        flexcan_mb_config &= ~CAN_CS_SRR_MASK;
    }
    else if(cs->msgIdType == FLEXCAN_MSG_ID_STD)
    {
        /* ID[28-18]*/
        *flexcan_mb_id &= ~CAN_ID_STD_MASK;
        *flexcan_mb_id |= (msgId << CAN_ID_STD_SHIFT) & CAN_ID_STD_MASK;

        /* make sure IDE and SRR are not set*/
        flexcan_mb_config &= ~(CAN_CS_IDE_MASK | CAN_CS_SRR_MASK);
    }
    else
    {
        return FLEXCAN_STATUS_INVALID_ARGUMENT;
    }

    /* Set the length of data in bytes */
    flexcan_mb_config &= ~CAN_CS_DLC_MASK;
    flexcan_mb_config |= (dlc_value << CAN_CS_DLC_SHIFT) & CAN_CS_DLC_MASK;

    /* Set MB CODE */
    if (cs->code != FLEXCAN_TX_NOT_USED)
    {
        if (cs->code == FLEXCAN_TX_REMOTE)
        {
            /* Set RTR bit */
            flexcan_mb_config |= CAN_CS_RTR_MASK;
            cs->code = FLEXCAN_TX_DATA;
        }

        /* Reset the code */
        flexcan_mb_config &= ~CAN_CS_CODE_MASK;

        /* Set the code */
        if (cs->fd_enable == 1)
        {
            flexcan_mb_config |= ((cs->code << CAN_CS_CODE_SHIFT) & CAN_CS_CODE_MASK) | CAN_MB_EDL_MASK;
        }
        else
        {
            flexcan_mb_config |= (cs->code << CAN_CS_CODE_SHIFT) & CAN_CS_CODE_MASK;
        }  

        if (cs->enable_brs == 1)
        {
            flexcan_mb_config |= CAN_MB_BRS_MASK;
        }

        *flexcan_mb |= flexcan_mb_config;
    }

    return FLEXCAN_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetMbRx
 * Description   : Configure a message buffer for receiving.
 * This function will first check if RX FIFO is enabled. If RX FIFO is enabled,
 * the function will make sure if the MB requested is not occupied by RX FIFO
 * and ID filter table. Then this function will configure the message buffer as
 * required for receiving.
 *
 *END**************************************************************************/
flexcan_status_t FLEXCAN_HAL_SetRxMsgBuff(
    CAN_Type * base,
    uint32_t msgBuffIdx,
    flexcan_msgbuff_code_status_t *cs,
    uint32_t msgId)
{
    uint32_t val1, val2 = 1;

    volatile uint32_t *flexcan_mb = FLEXCAN_HAL_GetMsgBuffRegion(base, msgBuffIdx);
    volatile uint32_t *flexcan_mb_id   = flexcan_mb + 1;

    if (msgBuffIdx >= (((base->MCR) & CAN_MCR_MAXMB_MASK) >> CAN_MCR_MAXMB_SHIFT))
    {
        return FLEXCAN_STATUS_OUT_OF_RANGE;
    }

    /* Check if RX FIFO is enabled */
    if (BITBAND_ACCESS32(&(base->MCR), CAN_MCR_RFEN_SHIFT))
    {
        /* Get the number of RX FIFO Filters*/
        val1 = (((base->CTRL2) & CAN_CTRL2_RFFN_MASK) >> CAN_CTRL2_RFFN_SHIFT);
        /* Get the number if MBs occupied by RX FIFO and ID filter table*/
        /* the Rx FIFO occupies the memory space originally reserved for MB0-5*/
        /* Every number of RFFN means 8 number of RX FIFO filters*/
        /* and every 4 number of RX FIFO filters occupied one MB*/
        val2 = RxFifoOcuppiedLastMsgBuff(val1);

        if (msgBuffIdx <= val2)
        {
            return FLEXCAN_STATUS_INVALID_ARGUMENT;
        }
    }

    /* Clean up the arbitration field area */
    *flexcan_mb = 0;
    *flexcan_mb_id = 0;

    /* Set the ID according the format structure */
    if (cs->msgIdType == FLEXCAN_MSG_ID_EXT)
    {
        /* Set IDE */
        *flexcan_mb |= CAN_CS_IDE_MASK;

        /* Clear SRR bit */
        *flexcan_mb &= ~CAN_CS_SRR_MASK;

        /* ID [28-0] */
        *flexcan_mb_id &= ~(CAN_ID_STD_MASK | CAN_ID_EXT_MASK);
        *flexcan_mb_id |= (msgId & (CAN_ID_STD_MASK | CAN_ID_EXT_MASK));
    }
    else if(cs->msgIdType == FLEXCAN_MSG_ID_STD)
    {
        /* Make sure IDE and SRR are not set */
        *flexcan_mb &= ~(CAN_CS_IDE_MASK | CAN_CS_SRR_MASK);

        /* ID[28-18] */
        *flexcan_mb_id &= ~CAN_ID_STD_MASK;
        *flexcan_mb_id |= (msgId << CAN_ID_STD_SHIFT) & CAN_ID_STD_MASK;
    }
    else
    {
        return FLEXCAN_STATUS_INVALID_ARGUMENT;
    }
 
    /* Set MB CODE */
    if (cs->code != FLEXCAN_RX_NOT_USED)
    {
         *flexcan_mb &= ~CAN_CS_CODE_MASK;
         *flexcan_mb |= (cs->code << CAN_CS_CODE_SHIFT) & CAN_CS_CODE_MASK;
    }

    return FLEXCAN_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_GetMsgBuff
 * Description   : Get a message buffer field values.
 * This function will first check if RX FIFO is enabled. If RX FIFO is enabled,
 * the function will make sure if the MB requested is not occupied by RX FIFO
 * and ID filter table. Then this function will get the message buffer field
 * values and copy the MB data field into user's buffer.
 *
 *END**************************************************************************/
flexcan_status_t FLEXCAN_HAL_GetMsgBuff(
    CAN_Type * base,
    uint32_t msgBuffIdx,
    flexcan_msgbuff_t *msgBuff)
{
    uint32_t i;
    uint32_t val1, val2 = 1;

    volatile uint32_t *flexcan_mb = FLEXCAN_HAL_GetMsgBuffRegion(base, msgBuffIdx);
    volatile uint32_t *flexcan_mb_id   = flexcan_mb + 1;
    volatile uint8_t  *flexcan_mb_data = (uint8_t *)(flexcan_mb + 2);

    uint32_t flexcan_mb_dlc_value = ((*flexcan_mb) & CAN_CS_DLC_MASK) >> 16;
    uint8_t payload_size = FLEXCAN_HAL_ComputePayloadSize(flexcan_mb_dlc_value);

    if (msgBuffIdx >= (((base->MCR) & CAN_MCR_MAXMB_MASK) >> CAN_MCR_MAXMB_SHIFT))
    {
        return FLEXCAN_STATUS_OUT_OF_RANGE;
    }

    /* Check if RX FIFO is enabled */
    if (BITBAND_ACCESS32(&(base->MCR), CAN_MCR_RFEN_SHIFT))
    {
        /* Get the number of RX FIFO Filters*/
        val1 = (((base->CTRL2) & CAN_CTRL2_RFFN_MASK) >> CAN_CTRL2_RFFN_SHIFT);
        /* Get the number if MBs occupied by RX FIFO and ID filter table*/
        /* the Rx FIFO occupies the memory space originally reserved for MB0-5*/
        /* Every number of RFFN means 8 number of RX FIFO filters*/
        /* and every 4 number of RX FIFO filters occupied one MB*/
        val2 = RxFifoOcuppiedLastMsgBuff(val1);

        if (msgBuffIdx <= val2)
        {
            return FLEXCAN_STATUS_INVALID_ARGUMENT;
        }
    }

    /* Get a MB field values */
    msgBuff->cs = *flexcan_mb;
    if ((msgBuff->cs) & CAN_CS_IDE_MASK)
    {
        msgBuff->msgId = (*(volatile uint32_t*)(flexcan_mb_id));
    }
    else
    {
        msgBuff->msgId = (*(volatile uint32_t*)(flexcan_mb_id)) >> CAN_ID_STD_SHIFT;
    }

    /* Copy MB data field into user's buffer */
    for (i = 0 ; i < payload_size ; i++)
    {
        msgBuff->data[i] = flexcan_mb_data[FLEXCAN_SWAP_BYTES_IN_WORD_INDEX(i)];
    }

    return FLEXCAN_STATUS_SUCCESS;
}

/** Arvind added */
/*FUNCTION**********************************************************************
 * 
 * Function Name : FLEXCAN_HAL_GetMailBox
 * Description   : Get a message buffer field values.
 * This function will first check if RX FIFO is enabled. If RX FIFO is enabled,
 * the function will make sure if the MB requested is not occupied by RX FIFO
 * and ID filter table. Then this function will get the message buffer field
 * values and copy the MB data field into user's buffer.
 *
 *END**************************************************************************/
flexcan_status_t FLEXCAN_HAL_GetMailBox(
    CAN_Type * base,
    uint32_t msgBuffIdx,
    flexcan_msgbuff_t *msgBuff,
    mailBox_t *mailBox)
{
    uint32_t i;
    uint32_t val1, val2 = 1;

    volatile uint32_t *flexcan_mb = FLEXCAN_HAL_GetMsgBuffRegion(base, msgBuffIdx);
    volatile uint32_t *flexcan_mb_id   = flexcan_mb + 1;
    volatile uint8_t  *flexcan_mb_data = (uint8_t *)(flexcan_mb + 2);

    uint32_t flexcan_mb_dlc_value = ((*flexcan_mb) & CAN_CS_DLC_MASK) >> 16;
    uint8_t payload_size = FLEXCAN_HAL_ComputePayloadSize(flexcan_mb_dlc_value);

    if (msgBuffIdx >= (((base->MCR) & CAN_MCR_MAXMB_MASK) >> CAN_MCR_MAXMB_SHIFT))
    {
        return FLEXCAN_STATUS_OUT_OF_RANGE;
    }

    /* Check if RX FIFO is enabled */
    if (BITBAND_ACCESS32(&(base->MCR), CAN_MCR_RFEN_SHIFT))
    {
        /* Get the number of RX FIFO Filters*/
        val1 = (((base->CTRL2) & CAN_CTRL2_RFFN_MASK) >> CAN_CTRL2_RFFN_SHIFT);
        /* Get the number if MBs occupied by RX FIFO and ID filter table*/
        /* the Rx FIFO occupies the memory space originally reserved for MB0-5*/
        /* Every number of RFFN means 8 number of RX FIFO filters*/
        /* and every 4 number of RX FIFO filters occupied one MB*/
        val2 = RxFifoOcuppiedLastMsgBuff(val1);

        if (msgBuffIdx <= val2)
        {
            return FLEXCAN_STATUS_INVALID_ARGUMENT;
        }
    }

    /* Get paylod from MB */
    mailBox->mb_dlc = payload_size;
    
    /* Get a MB field values */
    msgBuff->cs = *flexcan_mb;
    
    if ((msgBuff->cs) & CAN_CS_IDE_MASK)
    {
       msgBuff->msgId = (*(volatile uint32_t*)(flexcan_mb_id));
       mailBox->mb_msgId = msgBuff->msgId;
    }
    else
    {
        msgBuff->msgId = (*(volatile uint32_t*)(flexcan_mb_id)) >> CAN_ID_STD_SHIFT;
        mailBox->mb_msgId = msgBuff->msgId;
    }

    /* Copy MB data field into user's buffer */
    for (i = 0 ; i < payload_size ; i++)
    {
       msgBuff->data[i] = flexcan_mb_data[FLEXCAN_SWAP_BYTES_IN_WORD_INDEX(i)];
    }

    mailBox->mb_payload = msgBuff->data;
    
    return FLEXCAN_STATUS_SUCCESS;
}

/** End */

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_LockRxMsgBuff
 * Description   : Lock the RX message buffer.
 * This function will the RX message buffer.
 *
 *END**************************************************************************/
flexcan_status_t FLEXCAN_HAL_LockRxMsgBuff(
    CAN_Type * base,
    uint32_t msgBuffIdx)
{
    volatile uint32_t *flexcan_mb = FLEXCAN_HAL_GetMsgBuffRegion(base, msgBuffIdx);
    
    uint32_t tmp = 1;

    if (msgBuffIdx >= (((base->MCR) & CAN_MCR_MAXMB_MASK) >> CAN_MCR_MAXMB_SHIFT))
    {
        return (FLEXCAN_STATUS_OUT_OF_RANGE);
    }

    /* Lock the mailbox */
    if(tmp)
    {
        tmp = *flexcan_mb;
    }

    return FLEXCAN_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_EnableRxFifo
 * Description   : Enable Rx FIFO feature.
 * This function will enable the Rx FIFO feature.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_EnableRxFifo(CAN_Type * base, uint32_t numOfFilters)
{
    uint32_t i;
    uint32_t maxNumMb;
    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(base);
    /* Enable RX FIFO*/
    BITBAND_ACCESS32(&(base->MCR), CAN_MCR_RFEN_SHIFT) = (0x1);
    /* Set the number of the RX FIFO filters needed*/
    (base->CTRL2) = (((base->CTRL2) & ~(CAN_CTRL2_RFFN_MASK)) | (((uint32_t)(((uint32_t)(numOfFilters))<<CAN_CTRL2_RFFN_SHIFT))&CAN_CTRL2_RFFN_MASK) );
    /* RX FIFO global mask*/
    (base->RXFGMASK) = (((uint32_t)(((uint32_t)(CAN_RXFGMASK_FGM_MASK)) << CAN_ID_EXT_SHIFT)) & (CAN_ID_STD_MASK | CAN_ID_EXT_MASK));
    maxNumMb = (((base->MCR) & CAN_MCR_MAXMB_MASK) >> CAN_MCR_MAXMB_SHIFT);
    for (i = 0; i < maxNumMb; i++)
    {
        /* RX individual mask*/
        ((base)->RXIMR[i]) = (((uint32_t)(((uint32_t)(CAN_RXIMR_MI_MASK)) << CAN_ID_EXT_SHIFT)) & (CAN_ID_STD_MASK | CAN_ID_EXT_MASK));
    }

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_DisableRxFifo
 * Description   : Disable Rx FIFO feature.
 * This function will disable the Rx FIFO feature.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_DisableRxFifo(CAN_Type * base)
{
    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(base);
    /* Disable RX FIFO*/
    BITBAND_ACCESS32(&(base->MCR), CAN_MCR_RFEN_SHIFT) = (0x0);

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetRxFifoFilterNum
 * Description   : Set the number of Rx FIFO filters.
 * This function will define the number of Rx FIFO filters.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_SetRxFifoFilterNum(
    CAN_Type * base,
    uint32_t number)
{
    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(base);

    /* Set the number of RX FIFO ID filters*/
    (base->CTRL2) = (((base->CTRL2) & ~(CAN_CTRL2_RFFN_MASK)) | (((uint32_t)(((uint32_t)(number))<<CAN_CTRL2_RFFN_SHIFT))&CAN_CTRL2_RFFN_MASK));

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetMaxMsgBuffNum
 * Description   : Set the number of the last Message Buffers.
 * This function will define the number of the last Message Buffers
 *
*END**************************************************************************/
void FLEXCAN_HAL_SetMaxMsgBuffNum(
    CAN_Type * base,
    uint32_t maxMsgBuffNum)
{
    uint8_t msgBuffIdx;
    uint32_t databyte;

    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(base);

    /* Set the maximum number of MBs*/
    base->MCR = (base->MCR & ~CAN_MCR_MAXMB_MASK) | ((maxMsgBuffNum << CAN_MCR_MAXMB_SHIFT) & CAN_MCR_MAXMB_MASK);

    /* Initialize all message buffers as inactive*/
    uint8_t can_real_payload = FLEXCAN_HAL_GetPayloadSize(base);
    for (msgBuffIdx = 0; msgBuffIdx < maxMsgBuffNum; msgBuffIdx++)
    {
        volatile uint32_t *flexcan_mb = FLEXCAN_HAL_GetMsgBuffRegion(base, msgBuffIdx);
        volatile uint32_t *flexcan_mb_id   = flexcan_mb + 1;
        volatile uint8_t  *flexcan_mb_data = (uint8_t *)(flexcan_mb + 2);
        
        *flexcan_mb = 0x0;
        *flexcan_mb_id = 0x0;
        for (databyte = 0; databyte < can_real_payload; databyte++)
        {
           flexcan_mb_data[databyte] = 0x0;
        }
    }

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetRxFifoFilter
 * Description   : Confgure RX FIFO ID filter table elements.
 *
 *END**************************************************************************/
flexcan_status_t FLEXCAN_HAL_SetRxFifoFilter(
    CAN_Type * base,
    flexcan_rx_fifo_id_element_format_t idFormat,
    flexcan_id_table_t *idFilterTable)
{
    /* Set RX FIFO ID filter table elements*/
    uint32_t i, j, numOfFilters;
    uint32_t val1 = 0, val2 = 0, val = 0;

    volatile uint32_t *filterTable = (uint32_t *)((uint32_t)base + RxFifoFilterTableOffset);

    numOfFilters = (((base->CTRL2) & CAN_CTRL2_RFFN_MASK) >> CAN_CTRL2_RFFN_SHIFT);

    switch(idFormat)
    {
        case (FLEXCAN_RX_FIFO_ID_FORMAT_A):
            /* One full ID (standard and extended) per ID Filter Table element.*/
            (base->MCR) = (((base->MCR) & ~(CAN_MCR_IDAM_MASK)) | ( (((uint32_t)(((uint32_t)(FLEXCAN_RX_FIFO_ID_FORMAT_A))<<CAN_MCR_IDAM_SHIFT))&CAN_MCR_IDAM_MASK)));
            if (idFilterTable->isRemoteFrame)
            {
                val = FlexCanRxFifoAcceptRemoteFrame << FLEXCAN_RX_FIFO_ID_FILTER_FORMATAB_RTR_SHIFT;
            }
            if (idFilterTable->isExtendedFrame)
            {
                val |= FlexCanRxFifoAcceptExtFrame << FLEXCAN_RX_FIFO_ID_FILTER_FORMATAB_IDE_SHIFT;
            }
            for (i = 0; i < RxFifoFilterElementNum(numOfFilters); i++)
            {
                if(idFilterTable->isExtendedFrame)
                {
                    filterTable[i] = val + ((*(idFilterTable->idFilter + i)) <<
                                             FLEXCAN_RX_FIFO_ID_FILTER_FORMATA_EXT_SHIFT &
                                             FLEXCAN_RX_FIFO_ID_FILTER_FORMATA_EXT_MASK);
                }else
                {
                    filterTable[i] = val + ((*(idFilterTable->idFilter + i)) <<
                                             FLEXCAN_RX_FIFO_ID_FILTER_FORMATA_STD_SHIFT &
                                             FLEXCAN_RX_FIFO_ID_FILTER_FORMATA_STD_MASK);
                }
            }
            break;
        case (FLEXCAN_RX_FIFO_ID_FORMAT_B):
            /* Two full standard IDs or two partial 14-bit (standard and extended) IDs*/
            /* per ID Filter Table element.*/
           (base->MCR) = (((base->MCR) & ~(CAN_MCR_IDAM_MASK)) | ( (((uint32_t)(((uint32_t)(FLEXCAN_RX_FIFO_ID_FORMAT_B))<<CAN_MCR_IDAM_SHIFT))&CAN_MCR_IDAM_MASK)));
            if (idFilterTable->isRemoteFrame)
            {
                val1 = FlexCanRxFifoAcceptRemoteFrame << FLEXCAN_RX_FIFO_ID_FILTER_FORMATAB_RTR_SHIFT;
                val2 = FlexCanRxFifoAcceptRemoteFrame << FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_RTR_SHIFT;
            }
            if (idFilterTable->isExtendedFrame)
            {
                val1 |= FlexCanRxFifoAcceptExtFrame << FLEXCAN_RX_FIFO_ID_FILTER_FORMATAB_IDE_SHIFT;
                val2 |= FlexCanRxFifoAcceptExtFrame << FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_IDE_SHIFT;
            }
            j = 0;
            for (i = 0; i < RxFifoFilterElementNum(numOfFilters); i++)
            {
                if (idFilterTable->isExtendedFrame)
                {
                    filterTable[i] = val1 + (((*(idFilterTable->idFilter + j)) &
                                              FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_EXT_MASK) <<
                                              FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_EXT_SHIFT1);
                    filterTable[i] |= val2 + (((*(idFilterTable->idFilter + j + 1)) &
                                              FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_EXT_MASK) <<
                                              FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_EXT_SHIFT2);
                }else
                {
                    filterTable[i] = val1 + (((*(idFilterTable->idFilter + j)) &
                                              FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_STD_MASK) <<
                                              FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_STD_SHIFT1);
                    filterTable[i] |= val2 + (((*(idFilterTable->idFilter + j + 1)) &
                                              FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_STD_MASK) <<
                                              FLEXCAN_RX_FIFO_ID_FILTER_FORMATB_STD_SHIFT2);
                }
                j = j + 2;
            }
            break;
        case (FLEXCAN_RX_FIFO_ID_FORMAT_C):
            /* Four partial 8-bit Standard IDs per ID Filter Table element.*/
            (base->MCR) = (((base->MCR) & ~(CAN_MCR_IDAM_MASK)) | ( (((uint32_t)(((uint32_t)(FLEXCAN_RX_FIFO_ID_FORMAT_C))<<CAN_MCR_IDAM_SHIFT))&CAN_MCR_IDAM_MASK)));
            j = 0;
            for (i = 0; i < RxFifoFilterElementNum(numOfFilters); i++)
            {
                filterTable[i] = (((*(idFilterTable->idFilter + j)) &
                                  FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_MASK) <<
                                  FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_SHIFT1);
                filterTable[i] = (((*(idFilterTable->idFilter + j + 1)) &
                                  FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_MASK) <<
                                  FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_SHIFT2);
                filterTable[i] = (((*(idFilterTable->idFilter + j + 2)) &
                                  FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_MASK) <<
                                  FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_SHIFT3);
                filterTable[i] = (((*(idFilterTable->idFilter + j + 3)) &
                                  FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_MASK) <<
                                  FLEXCAN_RX_FIFO_ID_FILTER_FORMATC_SHIFT4);
                j = j + 4;
            }
            break;
        case (FLEXCAN_RX_FIFO_ID_FORMAT_D):
            /* All frames rejected.*/
            (base->MCR) = (((base->MCR) & ~(CAN_MCR_IDAM_MASK)) | ( (((uint32_t)(((uint32_t)(FLEXCAN_RX_FIFO_ID_FORMAT_D))<<CAN_MCR_IDAM_SHIFT))&CAN_MCR_IDAM_MASK)));
            break;
        default:
            return FLEXCAN_STATUS_INVALID_ARGUMENT;
    }
    return FLEXCAN_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetMsgBuffIntCmd
 * Description   : Enable/Disable the corresponding Message Buffer interrupt.
 *
 *END**************************************************************************/
flexcan_status_t FLEXCAN_HAL_SetMsgBuffIntCmd(
    CAN_Type * base,
    uint32_t msgBuffIdx, bool enable)
{
    uint32_t temp;

    if (msgBuffIdx >= (((base->MCR) & CAN_MCR_MAXMB_MASK) >> CAN_MCR_MAXMB_SHIFT))
    {
        return FLEXCAN_STATUS_OUT_OF_RANGE;
    }

    /* Enable the corresponding message buffer Interrupt*/
    temp = 0x1 << msgBuffIdx;
    if(enable)
    {
        (base->IMASK1) = ((base ->IMASK1) |  (temp));
    }
    else
    {
        (base->IMASK1) = ((base->IMASK1) & ~(temp));
    }

    return FLEXCAN_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetErrIntCmd
 * Description   : Enable the error interrupts.
 * This function will enable Error interrupt.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_SetErrIntCmd(CAN_Type * base, flexcan_int_type_t errType, bool enable)
{
    uint32_t temp = errType;
    if(enable)
    {
        if((errType == FLEXCAN_INT_RX_WARNING)||(errType == FLEXCAN_INT_TX_WARNING))
        {
           BITBAND_ACCESS32(&(base->MCR), CAN_MCR_WRNEN_SHIFT) = (0x1);
        }
        if(errType == FLEXCAN_INT_WAKEUP)
        {
            BITBAND_ACCESS32(&(base->MCR), CAN_MCR_WAKMSK_SHIFT) = (0x1);
        }
        (base->CTRL1) = ((base->CTRL1) |  (temp));
    }
    else
    {
        if(errType == FLEXCAN_INT_WAKEUP)
        {
            (BITBAND_ACCESS32(&(base->MCR), CAN_MCR_WAKMSK_SHIFT) = (0x0));
        }
        (base->CTRL1) = ((base->CTRL1) & ~(~temp));
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_ExitFreezeMode
 * Description   : Exit of freeze mode.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_ExitFreezeMode(CAN_Type * base)
{
    BITBAND_ACCESS32(&(base->MCR), CAN_MCR_HALT_SHIFT) = (0x0);
    BITBAND_ACCESS32(&(base->MCR), CAN_MCR_FRZ_SHIFT) = (0x0);
    /* Wait till exit freeze mode*/
    while (BITBAND_ACCESS32(&(base->MCR), CAN_MCR_FRZACK_SHIFT)){}
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_EnterFreezeMode
 * Description   : Enter the freeze mode.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_EnterFreezeMode(CAN_Type * base)
{
    BITBAND_ACCESS32(&(base->MCR), CAN_MCR_FRZ_SHIFT) = (0x1);
    BITBAND_ACCESS32(&(base->MCR), CAN_MCR_HALT_SHIFT) = (0x1);


    /* Wait for entering the freeze mode*/
    while (!(BITBAND_ACCESS32(&(base->MCR), CAN_MCR_FRZACK_SHIFT))){}
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_GetMsgBuffIntStatusFlag
 * Description   : Get the corresponding message buffer interrupt flag.
 *
 *END**************************************************************************/
uint8_t FLEXCAN_HAL_GetMsgBuffIntStatusFlag(
    CAN_Type * base,
    uint32_t msgBuffIdx)
{
    uint32_t temp;

    /* Get the corresponding message buffer interrupt flag*/
    temp = 0x1 << msgBuffIdx;
    if ((base->IFLAG1) & temp)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_GetErrCounter
 * Description   : Get transmit error counter and receive error counter.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_GetErrCounter(
    CAN_Type * base,
    flexcan_buserr_counter_t *errCount)
{
    /* Get transmit error counter and receive error counter*/
    errCount->rxerr = (((base->ECR) & CAN_ECR_RXERRCNT_MASK) >> CAN_ECR_RXERRCNT_SHIFT);
    errCount->txerr = (((base->ECR) & CAN_ECR_TXERRCNT_MASK) >> CAN_ECR_TXERRCNT_SHIFT);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_ClearErrIntStatusFlag
 * Description   : Clear all error interrupt status.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_ClearErrIntStatusFlag(CAN_Type * base)
{
    if((base->ESR1) & FLEXCAN_ALL_INT)
    {
        (base->ESR1) = FLEXCAN_ALL_INT;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_ReadRxFifo
 * Description   : Read Rx FIFO data.
 * This function will copy MB[0] data field into user's buffer.
 *
 *END**************************************************************************/
flexcan_status_t FLEXCAN_HAL_ReadRxFifo(
    CAN_Type * base,
    flexcan_msgbuff_t *rxFifo)
{
    uint32_t databyte = 0;
    
    volatile uint32_t *flexcan_mb = base->RAMn;
    volatile uint32_t *flexcan_mb_id = &base->RAMn[1];
    volatile uint8_t *flexcan_mb_data = (volatile uint8_t *)(&base->RAMn[2]);
    uint8_t can_real_payload = FLEXCAN_HAL_GetPayloadSize(base);
    
    rxFifo->cs = *flexcan_mb;

    if ((rxFifo->cs) & CAN_CS_IDE_MASK)
    {
        rxFifo->msgId = *flexcan_mb_id;
    }
    else
    {
        rxFifo->msgId = (*flexcan_mb_id) >> CAN_ID_STD_SHIFT;
    }

    /* Copy MB[0] data field into user's buffer */
    for (databyte = 0; databyte < can_real_payload; databyte++ )
    {
        rxFifo->data[databyte] = *(flexcan_mb_data + databyte);
    }

    return FLEXCAN_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetMaskType
 * Description   : Set RX masking type.
 * This function will set RX masking type as RX global mask or RX individual
 * mask.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_SetRxMaskType(
    CAN_Type * base,
    flexcan_rx_mask_type_t type)
{
    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(base);

    /* Set RX masking type (RX global mask or RX individual mask)*/
    if (type == FLEXCAN_RX_MASK_GLOBAL)
    {
        /* Enable Global RX masking*/
        (BITBAND_ACCESS32(&(base->MCR), CAN_MCR_IRMQ_SHIFT) = (0x0));
    }
    else
    {
        /* Enable Individual Rx Masking and Queue*/
        (BITBAND_ACCESS32(&(base->MCR), CAN_MCR_IRMQ_SHIFT) = (0x1));
    }

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetRxFifoGlobalStdMask
 * Description   : Set Rx FIFO global mask as the 11-bit standard mask.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_SetRxFifoGlobalStdMask(
    CAN_Type * base,
    uint32_t stdMask)
{
    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(base);

    /* 11 bit standard mask*/
    (base->RXFGMASK) = (((uint32_t)(((uint32_t)(stdMask))<<CAN_ID_STD_SHIFT))&CAN_ID_STD_MASK);

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetRxFifoGlobalExtMask
 * Description   : Set Rx FIFO global mask as the 29-bit extended mask.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_SetRxFifoGlobalExtMask(
    CAN_Type * base,
    uint32_t extMask)
{
    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(base);

    /* 29-bit extended mask*/
    (base->RXFGMASK) = (((uint32_t)(((uint32_t)(extMask)) << CAN_ID_EXT_SHIFT)) & (CAN_ID_STD_MASK | CAN_ID_EXT_MASK));

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetRxIndividualStdMask
 * Description   : Set Rx individual mask as the 11-bit standard mask.
 *
 *END**************************************************************************/
flexcan_status_t FLEXCAN_HAL_SetRxIndividualStdMask(
    CAN_Type * base,
    uint32_t msgBuffIdx,
    uint32_t stdMask)
{
    if (msgBuffIdx >= ((((base->MCR) & CAN_MCR_MAXMB_MASK) >> CAN_MCR_MAXMB_SHIFT)))
    {
        return FLEXCAN_STATUS_OUT_OF_RANGE;
    }

    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(base);

    /* 11 bit standard mask*/
     (base->RXIMR[msgBuffIdx]) =  (((uint32_t)(((uint32_t)(stdMask))<<CAN_ID_STD_SHIFT))&CAN_ID_STD_MASK);

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(base);

    return FLEXCAN_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetRxIndividualExtMask
 * Description   : Set Rx individual mask as the 29-bit extended mask.
 *
 *END**************************************************************************/
flexcan_status_t FLEXCAN_HAL_SetRxIndividualExtMask(
    CAN_Type * base,
    uint32_t msgBuffIdx,
    uint32_t extMask)
{
    if (msgBuffIdx >= (((base->MCR) & CAN_MCR_MAXMB_MASK) >> CAN_MCR_MAXMB_SHIFT))
    {
        return FLEXCAN_STATUS_OUT_OF_RANGE;
    }

    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(base);

    /* 29-bit extended mask*/
    (base->RXIMR[msgBuffIdx]) =   (((uint32_t)(((uint32_t)(extMask)) << CAN_ID_EXT_SHIFT)) & (CAN_ID_STD_MASK | CAN_ID_EXT_MASK));
    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(base);

    return FLEXCAN_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetRxMbGlobalStdMask
 * Description   : Set Rx Message Buffer global mask as the 11-bit standard mask.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_SetRxMsgBuffGlobalStdMask(
    CAN_Type * base,
    uint32_t stdMask)
{
    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(base);

    /* 11 bit standard mask*/

    (base->RXMGMASK) = (((uint32_t)(((uint32_t)(stdMask))<<CAN_ID_STD_SHIFT))&CAN_ID_STD_MASK);

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetRxMbBuf14StdMask
 * Description   : Set Rx Message Buffer 14 mask as the 11-bit standard mask.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_SetRxMsgBuff14StdMask(
    CAN_Type * base,
    uint32_t stdMask)
{
    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(base);

    /* 11 bit standard mask*/
    (base->RX14MASK) = (((uint32_t)(((uint32_t)(stdMask))<<CAN_ID_STD_SHIFT))&CAN_ID_STD_MASK);

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetRxMbBuf15StdMask
 * Description   : Set Rx Message Buffer 15 mask as the 11-bit standard mask.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_SetRxMsgBuff15StdMask(
    CAN_Type * base,
    uint32_t stdMask)
{
    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(base);

    /* 11 bit standard mask*/
    (base->RX15MASK) = (((uint32_t)(((uint32_t)(stdMask))<<CAN_ID_STD_SHIFT))&CAN_ID_STD_MASK);

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetRxMbGlobalExtMask
 * Description   : Set Rx Message Buffer global mask as the 29-bit extended mask.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_SetRxMsgBuffGlobalExtMask(
    CAN_Type * base,
    uint32_t extMask)
{
    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(base);

    /* 29-bit extended mask*/
    (base->RXMGMASK) = (((uint32_t)(((uint32_t)(extMask)) << CAN_ID_EXT_SHIFT)) & (CAN_ID_STD_MASK | CAN_ID_EXT_MASK));

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetRxMbBuf14ExtMask
 * Description   : Set Rx Message Buffer 14 mask as the 29-bit extended mask.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_SetRxMsgBuff14ExtMask(
    CAN_Type * base,
    uint32_t extMask)
{
    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(base);

    /* 29-bit extended mask*/
    (base->RX14MASK) = (((uint32_t)(((uint32_t)(extMask)) << CAN_ID_EXT_SHIFT)) & (CAN_ID_STD_MASK | CAN_ID_EXT_MASK));

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetRxMbBuf15ExtMask
 * Description   : Set Rx Message Buffer 15 mask as the 29-bit extended mask.
 *
 *END**************************************************************************/
void FLEXCAN_HAL_SetRxMsgBuff15ExtMask(
    CAN_Type * base,
    uint32_t extMask)
{
    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(base);

    /* 29-bit extended mask*/
    (base->RX15MASK) = (((uint32_t)(((uint32_t)(extMask)) << CAN_ID_EXT_SHIFT)) & (CAN_ID_STD_MASK | CAN_ID_EXT_MASK));

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_EnableOperationMode
 * Description   : Enable a FlexCAN operation mode.
 * This function will enable one of the modes listed in flexcan_operation_modes_t.
 *
 *END**************************************************************************/
flexcan_status_t FLEXCAN_HAL_SetOperationMode(
    CAN_Type * base,
    flexcan_operation_modes_t mode)
{
    if (mode == FLEXCAN_FREEZE_MODE)
    {
        /* Debug mode, Halt and Freeze*/
        FLEXCAN_HAL_EnterFreezeMode(base);

        return FLEXCAN_STATUS_SUCCESS;
    }
    else if (mode == FLEXCAN_DISABLE_MODE)
    {
        /* Debug mode, Halt and Freeze*/
        (BITBAND_ACCESS32(&(base->MCR), CAN_MCR_MDIS_SHIFT) = (0x1));
        return FLEXCAN_STATUS_SUCCESS;
    }

    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(base);

    if (mode == FLEXCAN_NORMAL_MODE)
    {
        (BITBAND_ACCESS32(&(base->MCR), CAN_MCR_SUPV_SHIFT) = (0x0));
        (BITBAND_ACCESS32(&(base->CTRL1), CAN_CTRL1_LOM_SHIFT) = (0x0));
        (BITBAND_ACCESS32(&(base->CTRL1), CAN_CTRL1_LPB_SHIFT) = (0x0));
    }
    else if (mode == FLEXCAN_LISTEN_ONLY_MODE)
    {
        (BITBAND_ACCESS32(&(base->CTRL1), CAN_CTRL1_LOM_SHIFT) = (0x1));
    }
    else if (mode == FLEXCAN_LOOPBACK_MODE)
    {
         (BITBAND_ACCESS32(&(base->CTRL1), CAN_CTRL1_LPB_SHIFT) = (0x1));
         (BITBAND_ACCESS32(&(base->CTRL1), CAN_CTRL1_LOM_SHIFT) = (0x0));
    }
    else
    {
        return FLEXCAN_STATUS_INVALID_ARGUMENT;
    }

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(base);

    return FLEXCAN_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_ExitOperationMode
 * Description   : Disable a FlexCAN operation mode.
 * This function will disable one of the modes listed in flexcan_operation_modes_t.
 *
 *END**************************************************************************/
flexcan_status_t FLEXCAN_HAL_ExitOperationMode(
    CAN_Type * base,
    flexcan_operation_modes_t mode)
{
    if (mode == FLEXCAN_FREEZE_MODE)
    {
        /* De-assert Freeze Mode*/
        FLEXCAN_HAL_ExitFreezeMode(base);

        return FLEXCAN_STATUS_SUCCESS;
    }
    else if (mode == FLEXCAN_DISABLE_MODE)
    {
        /* Disable module mode*/
        (BITBAND_ACCESS32(&(base->MCR), CAN_MCR_MDIS_SHIFT) = (0x0));
        return FLEXCAN_STATUS_SUCCESS;
    }

    /* Set Freeze mode*/
    FLEXCAN_HAL_EnterFreezeMode(base);

    if (mode == FLEXCAN_NORMAL_MODE)
    {
        (BITBAND_ACCESS32(&(base->MCR), CAN_MCR_SUPV_SHIFT) = (0x1));
    }
    else if (mode == FLEXCAN_LISTEN_ONLY_MODE)
    {
        (BITBAND_ACCESS32(&(base->CTRL1), CAN_CTRL1_LOM_SHIFT) = (0x0));
    }
    else if (mode == FLEXCAN_LOOPBACK_MODE)
    {
        (BITBAND_ACCESS32(&(base->CTRL1), CAN_CTRL1_LPB_SHIFT) = (0x0));
    }
    else
    {
        return FLEXCAN_STATUS_INVALID_ARGUMENT;
    }

    /* De-assert Freeze Mode*/
    FLEXCAN_HAL_ExitFreezeMode(base);

    return FLEXCAN_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetFDEnabled
 * Description   : Enables/Disables Flexible Data rate (if supported).
 *
 *END**************************************************************************/
void FLEXCAN_HAL_SetFDEnabled(CAN_Type * base, bool enable)
{
    FLEXCAN_HAL_EnterFreezeMode(base);

    BITBAND_ACCESS32(&(base->MCR), CAN_MCR_FDEN_SHIFT) = (uint32_t)enable;

    FLEXCAN_HAL_ExitFreezeMode(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_IsFDEnabled
 * Description   : Checks if the Flexible Data rate feature is enabled.
 *
 *END**************************************************************************/
bool FLEXCAN_HAL_IsFDEnabled(CAN_Type * base)
{
    return (bool) BITBAND_ACCESS32(&(base->MCR), CAN_MCR_FDEN_SHIFT);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_SetPayloadSize
 * Description   : Sets the payload size of the MBs.
 *
 *END**************************************************************************/
flexcan_status_t FLEXCAN_HAL_SetPayloadSize(
    CAN_Type * base,
    flexcan_fd_payload_size_t payloadSize)
{
    uint32_t tmp;

    /* If FD is not enabled, only 8 bytes payload is supported */
    if (!FLEXCAN_HAL_IsFDEnabled(base) && (payloadSize == FLEXCAN_PAYLOAD_SIZE_8))
    {
        return FLEXCAN_STATUS_SUCCESS;
    }
    if (!FLEXCAN_HAL_IsFDEnabled(base) && (payloadSize != FLEXCAN_PAYLOAD_SIZE_8))
    {
        return FLEXCAN_STATUS_UNKNOWN_PROPERTY;
    }

    FLEXCAN_HAL_EnterFreezeMode(base);

    tmp = base->FDCTRL;
    tmp &= ~(CAN_FDCTRL_MBDSR0_MASK | CAN_FDCTRL_MBDSR1_MASK);
    tmp |= payloadSize << CAN_FDCTRL_MBDSR0_SHIFT;
    tmp |= payloadSize << CAN_FDCTRL_MBDSR1_SHIFT;

    base->FDCTRL = tmp;

    FLEXCAN_HAL_ExitFreezeMode(base);

    return FLEXCAN_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXCAN_HAL_GetPayloadSize
 * Description   : Returns the payload size of the MBs (in bytes).
 *
 *END**************************************************************************/
uint8_t FLEXCAN_HAL_GetPayloadSize(CAN_Type * base)
{
    flexcan_fd_payload_size_t payloadSize;

    /* The standard payload size is 8 bytes */
    if (!FLEXCAN_HAL_IsFDEnabled(base))
    {
        return 8U;
    }

    payloadSize = (flexcan_fd_payload_size_t) 
        ((base->FDCTRL & CAN_FDCTRL_MBDSR0_MASK) >> CAN_FDCTRL_MBDSR0_SHIFT);
    return (1 << (payloadSize + 3));
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
