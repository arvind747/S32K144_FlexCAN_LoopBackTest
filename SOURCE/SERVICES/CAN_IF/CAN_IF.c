#include <string.h>

/**************************************************************************************************/
/* User Header Files Section                                                                      */
/**************************************************************************************************/

#include "Includes.h"

#include "CAN_IF.h"
#include "CAN_IF_Priv.h"

#include "Cpu.h"
#include "clockMan1.h"
#include "canCom1.h"
#include "pin_mux.h"

/**************************************************************************************************/
/* Global Definitions Section                                                                     */
/**************************************************************************************************/

/**************************************************************************************************/
/* Global Variables Section                                                                       */
/**************************************************************************************************/

/* ISO TP Tx & Rx Handlers */
INT16U u16_gUdsReqID = 0x7E5;
INT16U u16_gUdsRespID = 0x7ED;
void (* fp_gIsoTpTxHandler)(INT16U u16_fMsgID); 
void (* fp_gIsoTpRxHandler)(void * p_fRxData);

/* Com Tx & Rx Handlers */
void (* fp_gComTxConformation)(INT16U u16_fMsgID);
void (* fp_gComRxIndication)(INT16U u16_fMsgID, void * p_fRxData);


/* Set information about the data to be sent */
flexcan_data_info_t g_dataInfo_rx;
flexcan_msgbuff_t recvBuff;

/**************************************************************************************************/
/* Function Name   : CAN_IF_MsgTxHandler                                                          */
/*                                                                                                */
/* Description     : Call back function called from CAN Low Level Driver                          */
/*                                                                                                */
/* In Params       : INT16U u16_fMsgID : Transmitted Message ID                                   */
/*                                                                                                */
/* Out Params      : None                                                                         */
/*                                                                                                */
/* Return Value    : void                                                                         */
/**************************************************************************************************/

void CAN_TX_Confirmation(mailBox_t * p_fMsgInfo)
{
    /* Check if the Mag trasmitted Belong to ISOTP or COM */
    if(p_fMsgInfo->mb_msgId == u16_gUdsRespID)
    {
        /* Call back to ISO TP Layers */
        if(PNULL != fp_gIsoTpTxHandler)
        {
            fp_gIsoTpTxHandler(p_fMsgInfo->mb_msgId);
        }
        else
        {
            /* No Action */
        }
    }
    else
    {
        /* Update to OSEK COM */
        if(PNULL != fp_gComTxConformation)
        {
            fp_gComTxConformation(p_fMsgInfo->mb_msgId);
        }
        else
        {
            /* No Action */
        }
    }
}

/**************************************************************************************************/
/* Function Name   : CAN_IF_MsgRxHandler                                                          */
/*                                                                                                */
/* Description     : Call back function called from CAN Low Level Driver                          */
/*                                                                                                */
/* In Params       : void * p_fRxMsgData : Receive CAN Message Data pointer                       */
/*                                                                                                */
/* Out Params      : None                                                                         */
/*                                                                                                */
/* Return Value    : void                                                                         */
/**************************************************************************************************/

void CAN_Rx_Notification(mailBox_t * p_fMsgInfo)
{
    ST_RX_PDU_t st_lRxMsg = {0, };
    
    /* Copy the Received Message */
    st_lRxMsg.u32_mArbitrationID = p_fMsgInfo->mb_msgId;
    memcpy((void *)&st_lRxMsg.u8_maDataBuff, \
                    (const void *)p_fMsgInfo->mb_payload, p_fMsgInfo->mb_dlc);
    st_lRxMsg.u8_mDataSize = p_fMsgInfo->mb_dlc;
    
    /* Check if the received message id UDS Request or COM Message */
    if(p_fMsgInfo->mb_msgId == u16_gUdsReqID)
    {   
        /* Call Back to ISO TP callback */
        if(PNULL != fp_gIsoTpRxHandler)
        {
            fp_gIsoTpRxHandler(&st_lRxMsg);
        }
        else
        {
            /* No Action */
        }
    }
    else
    {
        /* Send the Data to OSEK COM */
        if(PNULL != fp_gComRxIndication)
        {
            fp_gComRxIndication(p_fMsgInfo->mb_msgId, &st_lRxMsg);
        }
        else
        {
            /* No Action */
        }
    }  
}

/**************************************************************************************************/
/* Function Name   : CAN_IF_Init()                                                                */
/*                                                                                                */
/* Description     : Initializes the CAN Low Level Driver                                         */
/*                                                                                                */
/* In Params       : None                                                                         */
/*                                                                                                */
/* Out Params      : None                                                                         */
/*                                                                                                */
/* Return Value    : EN_CAN_IF_RESULT_t : returns CAN_IF_OK On success                            */
/**************************************************************************************************/

EN_CAN_IF_RESULT_t CAN_IF_Init(void)
{
    EN_CAN_IF_RESULT_t en_lResult = CAN_IF_OK;
 
    /* Initialize and configure clocks
     * 	see clock manager component for details */
    CLOCK_SYS_Init(g_clockManConfigsArr, FSL_CLOCK_MANAGER_CONFIG_CNT,
    					g_clockManCallbacksArr, FSL_CLOCK_MANAGER_CALLBACK_CNT);
    CLOCK_SYS_UpdateConfiguration(0U, CLOCK_MANAGER_POLICY_FORCIBLE); 

	/* Initialize pins See PinSettings component for more info */
    Pins_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);

    /* Initialization of CAN RX/TX  callback function handler */       
    CAN_Mgr_Init(CAN_TX_Confirmation , CAN_Rx_Notification);
    
    /* Initialize FlexCAN driver */
    FLEXCAN_DRV_Init(FSL_CANCOM1, &canCom1_State, &canCom1_InitConfig0);

    /* Set bit rate */
    flexcan_time_segment_t myBitRate = {0x04, 0x07, 0x01, 0x00, 1};
    FLEXCAN_DRV_SetBitrate(FSL_CANCOM1, &myBitRate);

    g_dataInfo_rx.data_length = 8U;
    g_dataInfo_rx.msg_id_type = FLEXCAN_MSG_ID_STD;
    g_dataInfo_rx.enable_brs = false;
    g_dataInfo_rx.fd_enable = false;
    g_dataInfo_rx.fd_padding = 0U;

    /* Configure Rx message buffer with index 1 and rx_mb_id = 256 */
    FLEXCAN_DRV_ConfigRxMb(FSL_CANCOM1, 1UL, &g_dataInfo_rx, 2);

    /* Define receive buffer */
	recvBuff.cs = 0;
	recvBuff.msgId = 2;

	/* Start receiving data in MB 1. */
	FLEXCAN_DRV_RxMessageBuffer(FSL_CANCOM1, 1UL, &recvBuff);

    
    return en_lResult;
}

/**************************************************************************************************/
/* Function Name   : CAN_IF_WriteMsg()                                                            */
/*                                                                                                */
/* Description     :                                                                              */
/*                                                                                                */
/* In Params       : INT32U u32_fCANID : Transmit Message ID                                      */
/*                   INT8U u8_fMONumber : Trasmit MO Number                                       */
/*                   INT8U * u8_fPtr : Transmit Data Pointer                                      */
/*                   INT8U u8_fDLC : Transmit Message Length                                      */
/*                                                                                                */
/* Out Params      : None                                                                         */
/*                                                                                                */
/* Return Value    : EN_CAN_IF_RESULT_t : returns CAN_IF_OK On success                            */
/**************************************************************************************************/

EN_CAN_IF_RESULT_t CAN_IF_WriteMsg(INT32U u32_fCANID, INT8U u8_fMONumber, \
                                                               INT8U * u8_fPtr, INT8U u8_fDLC)
{
    EN_CAN_IF_RESULT_t en_lResult = CAN_IF_OK;
        
    flexcan_data_info_t g_dataInfo_tx;
        
    g_dataInfo_tx.data_length = 8U;
    g_dataInfo_tx.msg_id_type = FLEXCAN_MSG_ID_STD;
    g_dataInfo_tx.enable_brs = false;
    g_dataInfo_tx.fd_enable = false;
    g_dataInfo_tx.fd_padding = 0U;
    
    /* Configure Tx message buffer with index 1 */
    FLEXCAN_DRV_ConfigTxMb(FSL_CANCOM1, u8_fMONumber, &g_dataInfo_tx, u32_fCANID);
	
	/* Execute send non-blocking */
    FLEXCAN_DRV_Send(FSL_CANCOM1, u8_fMONumber, &g_dataInfo_tx, u32_fCANID, u8_fPtr);

    
    return en_lResult;
}

/**************************************************************************************************/
/* Function Name   : CAN_IF_InitIsoTp()                                                           */
/*                                                                                                */
/* Description     : Initialize the ISOTP request and response IDs, and TX and Rx Handlers        */
/*                                                                                                */
/* In Params       : ST_ISOTP_CONFIG_PARAMS_t * p_stfIsoTpInitData : ISOTP init data Pointer      */
/*                                                                                                */
/* Out Params      : None                                                                         */
/*                                                                                                */
/* Return Value    : EN_CAN_IF_RESULT_t : returns CAN_IF_OK On success                            */
/**************************************************************************************************/

EN_CAN_IF_RESULT_t CAN_IF_InitIsoTp(ST_ISOTP_CONFIG_PARAMS_t * p_stfIsoTpInitData)  
{
    EN_CAN_IF_RESULT_t en_lResult = CAN_IF_OK;

    /* update CAN TX and RX Event Handlers */
    fp_gIsoTpTxHandler = p_stfIsoTpInitData->fp_mIsoTpTxHandler;
    fp_gIsoTpRxHandler = p_stfIsoTpInitData->fp_mIsoTpRxHandler; 
    
     /* Update Request & Respose ID */
    u16_gUdsReqID = p_stfIsoTpInitData->u16_mIsoTpReqID;
    u16_gUdsRespID = p_stfIsoTpInitData->u16_mIsoTpRespID;
    
    return en_lResult;
}

/**************************************************************************************************/
/* Function Name   : CAN_IF_UpdateComHandlers()                                                   */
/*                                                                                                */
/* Description     : Updates the OSEK COM TX and Rx Handlers To CAN IF                            */
/*                                                                                                */
/* In Params       : void (* fp_mComTxHandler)(INT16U) : COM Tx conformation Call back            */
/*                   void (* fp_mComRxHandler)(void * p_fRxData) : COM Rx Initication Call back   */
/*                                                                                                */
/* Out Params      : None                                                                         */
/*                                                                                                */
/* Return Value    : EN_CAN_IF_RESULT_t : returns CAN_IF_OK On success                            */
/**************************************************************************************************/

EN_CAN_IF_RESULT_t CAN_IF_UpdateComHandlers(void (* fp_mComTxHandler)(INT16U), \
                                  void (* fp_mComRxHandler)(INT16U u16_fMsgID, void * p_fRxData))  
{
    EN_CAN_IF_RESULT_t en_lResult = CAN_IF_OK;

    /* update CAN TX and RX Event Handlers */
    fp_gComRxIndication = fp_mComRxHandler;
    fp_gComTxConformation = fp_mComTxHandler; 
    
    return en_lResult;
}

/**************************************************************************************************/
/* Function Name   : CAN_IF_ISOTP_WriteMsg()                                                      */
/*                                                                                                */
/* Description     :                                                                              */
/*                                                                                                */
/* In Params       : INT32U u32_fCANID : Transmit Message ID                                      */
/*                   INT8U * u8_fPtr : Transmit Data Pointer                                      */
/*                   INT8U u8_fDLC : Transmit Message Length                                      */
/*                                                                                                */
/* Out Params      : None                                                                         */
/*                                                                                                */
/* Return Value    : EN_CAN_IF_RESULT_t : returns CAN_IF_OK On success                            */
/**************************************************************************************************/

EN_CAN_IF_RESULT_t CAN_IF_ISOTP_WriteMsg(INT32U u32_fCANID, INT8U * u8_fPtr, INT8U u8_fDLC)
{
    EN_CAN_IF_RESULT_t en_lResult = CAN_IF_OK;
#if 0    
    ST_CAN_MO_INIT_t st_lMOInit = {0,};

    st_lMOInit.u32_mCANID =  u32_fCANID;
    st_lMOInit.u8_mMONumber =  0;
    st_lMOInit.u8_mDLC = u8_fDLC;    
    st_lMOInit.en_mCANIDType = CAN_STANDARD_ID;	
    st_lMOInit.en_mMIDE = CAN_RECEIVE_MATCH_IDE;
    st_lMOInit.u8_mAcceptanceMask = 0x7FF;
    st_lMOInit.en_mCANFrameType = CAN_DATA_FRAME;
    st_lMOInit.en_mCANMOType = CAN_MO_TRANSMIT;
    st_lMOInit.en_mMsgObjStatus = CAN_ENABLE;

    memcpy((INT8U *)&(st_lMOInit.au8_mData[0]), &(u8_fPtr[0]), u8_fDLC );
    
     /* Configure the Mail Box and Transmit */
    CAN_ConfigMO(&st_lMOInit);
    CAN_Transmit_DataFrame(st_lMOInit.u8_mMONumber);
#endif    
    return en_lResult;
}


/**************************************************************************************************/
/* Function Name   : CAN_IF_ConfigRxMO()                                                          */
/*                                                                                                */
/* Description     : Configures given MO for given Direction                                      */
/*                                                                                                */
/* In Params       : INT32U u32_fCANID : Transmit Message ID                                      */
/*                   INT8U u8_fMONumber : Trasmit MO Number                                       */
/*                   EN_CAN_MO_TYPE_t en_fMoDirection : Direction is either Transmit or receive   */
/*                                                                                                */
/* Out Params      : None                                                                         */
/*                                                                                                */
/* Return Value    : EN_CAN_IF_RESULT_t : returns CAN_IF_OK On success                            */
/**************************************************************************************************/

EN_CAN_IF_RESULT_t CAN_IF_ConfigRxMO(INT32 u32_fCANID, INT8U u8_fMONumber, \
                                                           EN_CAN_MO_TYPE_t en_fMoDirection)  
{
    EN_CAN_IF_RESULT_t en_lResult = CAN_IF_OK;        
    ST_CAN_MO_INIT_t st_lMOInit = {0,};
    st_lMOInit.u32_mCANID = u32_fCANID;
    st_lMOInit.u8_mMONumber =  u8_fMONumber;
    st_lMOInit.u8_mDLC = 8;    
    st_lMOInit.en_mCANIDType = CAN_STANDARD_ID;	
    st_lMOInit.en_mMIDE = CAN_RECEIVE_MATCH_IDE;
    st_lMOInit.u8_mAcceptanceMask = 0x7FF;
    st_lMOInit.en_mCANFrameType = CAN_DATA_FRAME;
    st_lMOInit.en_mCANMOType = CAN_MO_RECEIVE;
    st_lMOInit.en_mMsgObjStatus = CAN_ENABLE;
    
    /* Configure the Mail Box */
    CAN_ConfigMO(&st_lMOInit);     
    return en_lResult;
}

/**************************************************************************************************/
/* Function Name   : CAN_IF_ReadMO()                                                              */
/*                                                                                                */
/* Description     :                                                                              */
/*                                                                                                */
/* In Params       : INT8U u8_fMONumber : MailBox Index                                           */
/*                                                                                                */
/* Out Params      : INT8U * u8_fPtr : CAN Message Data Pointer                                   */
/*                                                                                                */
/* Return Value    : EN_CAN_IF_RESULT_t : returns CAN_IF_OK On success                            */
/**************************************************************************************************/

EN_CAN_IF_RESULT_t CAN_IF_ReadMO(INT8U u8_fMONumber, INT8U * u8_fPtr)
{
    EN_CAN_IF_RESULT_t en_lResult = CAN_IF_OK;
    
    ST_CAN_MO_INIT_t st_lMOInit = {0,};
    
    CAN_ReadMsg(&st_lMOInit, u8_fMONumber);
    memcpy(&(u8_fPtr[0]), (INT8U *)&(st_lMOInit.au8_mData[0]), 8);
  
    return en_lResult;
}

/**************************************************************************************************/
/* End of CAN_IF.c                                                                               */
/**************************************************************************************************/
