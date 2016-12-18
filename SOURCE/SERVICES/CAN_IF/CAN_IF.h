#ifndef _CAN_IF_H_
#define _CAN_IF_H_

/**************************************************************************************************/
/* System Header Files Section                                                                    */
/**************************************************************************************************/
#include "Includes.h"
#include <stdint.h>

/**************************************************************************************************/
/* User Header Files Section                                                                      */
/**************************************************************************************************/
#include "fsl_interrupt_manager.h"
#include "fsl_flexcan_hal.h"
#include "fsl_flexcan_driver.h"


/**************************************************************************************************/
/* Export Global Definitions Section                                                              */
/**************************************************************************************************/

#define FLEXCAN_SWAP_BYTES_IN_WORD_INDEX(index)       (((index) & ~3U) + (3U - ((index) & 3U)))
/*CAN Channel Definetion*/
#define MBFM CAN1
#define TRANSMIT_TPMS CAN0

/**************************************************************************************************/
/* Export Global Variable Section                                                                 */
/**************************************************************************************************/

typedef enum
{
   CAN_MO_TRANSMIT = 0,
   CAN_MO_RECEIVE
}EN_CAN_MO_TYPE_t;

typedef enum
{
    CAN_IF_OK = 0x00,
    CAN_IF_ERROR
      
}EN_CAN_IF_RESULT_t;

#if 0
/* DEVICE SELECTION*/
typedef enum
{
	CAN0 = 0x00,
	CAN1,
	CAN2,
	CAN_MAX_DEVICES

}EN_CAN_NODEID_t;


/*  STRUCTURE For CAN Parameters */
typedef struct
{
	EN_CAN_NODEID_t en_mNodeID;
	INT32U u32_mCANID;
	INT8U u8_mMONumber;
	INT8U  au8_mData[8];

}ST_CAN_PARA_INIT_t;
#endif

/* DEVICE SELECTION*/

/* ENUM for CAN ID Type */
typedef enum
{
	CAN_STANDARD_ID = 0,
	CAN_EXTENDED_ID

}EN_CAN_ID_TYPE_t;

/* Acceptance Mask Bit for Message IDE Bit */
typedef enum
{
	CAN_RECEIVE_BOTH = 0x00,
	CAN_RECEIVE_MATCH_IDE,

}EN_CAN_RECEIVE_MESSAGE_IDE_t;

/* ENUM for CAN FRAME Type */
typedef enum
{
    EN_MAILBOX_TRANSMIT = 0x00,
    EN_MAILBOX_RECEIVE
  
}EN_MAILBOX_DIRECTION_t;


/* ENUM for CAN FRAME Type */
typedef enum
{
	CAN_DATA_FRAME = 0,
	CAN_REMOTE_FRAME

}EN_CAN_FRAME_TYPE_t;

/* enum to select the functional states   */
typedef enum
{
	CAN_DISABLE = 0,
	CAN_ENABLE

}EN_CAN_FUNCTIONALSTATE_t;


/*  STRUCTURE For CAN Message Object(MO) Initialization */
typedef struct
{
	/* CAN MO Number */
	INT8U u8_mMONumber;

	/* CAN ID Type i.e STD or EXTENDED */
	EN_CAN_ID_TYPE_t en_mCANIDType;

	/* CAN ID*/
	INT32U u32_mCANID;

	/* Acceptance Mask Bit for Message IDE Bit */
	EN_CAN_RECEIVE_MESSAGE_IDE_t en_mMIDE;

	/* Acceptance Mask for Message Identifier */
	INT32U u8_mAcceptanceMask;

	/* CAN Frame Type */
	EN_CAN_FRAME_TYPE_t en_mCANFrameType;

	/* CAN Data Length Code */
	INT8U  u8_mDLC;

	/* */
	INT8U  au8_mData[8];
	

	/* CAN Message Object(MO) Type*/
	EN_CAN_MO_TYPE_t en_mCANMOType;

	/* Enable or Disable the CAN MO */
	EN_CAN_FUNCTIONALSTATE_t en_mMsgObjStatus;

}ST_CAN_MO_INIT_t;


extern INT8U u8_gChannelNo;



/* CAN Return Status */
typedef enum
{
	CAN_SUCCESS = 0x00,
	CAN_ERROR

}EN_CAN_RESULT_t;

/* ENUM To Select CAN Mode */
typedef enum
{
	CAN_MODE_NORMAL = 0x00,
	CAN_MODE_LOOPBK,

}EN_CAN_MODE_t;

/* ENUM To Select CAN Synchronization Jump Width(SJW) */
typedef enum
{
	CAN_SJW_0TQ = 0,
	CAN_SJW_1TQ,
	CAN_SJW_2TQ,
	CAN_SJW_3TQ

}EN_CAN_SJW_t;

/* ENUM To Select CAN Time Segment Before Sample Point(TSEG1) */
typedef enum
{
	CAN_TSEG1_2TQ = 2,
	CAN_TSEG1_3TQ,
	CAN_TSEG1_4TQ,
	CAN_TSEG1_5TQ,
	CAN_TSEG1_6TQ,
	CAN_TSEG1_7TQ,
	CAN_TSEG1_8TQ,
	CAN_TSEG1_9TQ,
	CAN_TSEG1_10TQ,
	CAN_TSEG1_11TQ,
	CAN_TSEG1_12TQ,
	CAN_TSEG1_13TQ,
	CAN_TSEG1_14TQ,
	CAN_TSEG1_15TQ

}EN_CAN_TSEG1_t;

/* ENUM To Select CAN Time Segment After Sample Point(TSEG2) */
typedef enum
{
	CAN_TSEG2_1TQ = 1,
	CAN_TSEG2_2TQ,
	CAN_TSEG2_3TQ,
	CAN_TSEG2_4TQ,
	CAN_TSEG2_5TQ,
	CAN_TSEG2_6TQ,
	CAN_TSEG2_7TQ,

}EN_CAN_TSEG2_t;

/*  ENUM To Select CAN Clock PRESCALAR */
typedef enum
{
	CAN_PRESCALAR_1 = 0x00,
	CAN_PRESCALAR_2,
	CAN_PRESCALAR_3,
	CAN_PRESCALAR_4,
	CAN_PRESCALAR_8,
	CAN_PRESCALAR_16,
	CAN_PRESCALAR_32,

}EN_CAN_PRESCALAR_t;

typedef enum
{
    EN_FLEXCAN_MSG_ID_STD = 0,
    EN_FLEXCAN_MSG_ID_EXT
  
}EN_MSG_ID_TYPE_t;

typedef struct
{
    void (* fp_mIsoTpTxHandler)(INT16U); 
    void (* fp_mIsoTpRxHandler)(void * p_fRxData);
    INT16U u16_mIsoTpReqID;
    INT16U u16_mIsoTpRespID;  
    
}ST_ISOTP_CONFIG_PARAMS_t;

typedef struct
{
    /* */
    INT8U u8_mDataLength;
    /* */
    EN_MSG_ID_TYPE_t en_mMsgIdType;
    /* */
    INT8U u8_mEnableBrs;
    /* */
    INT8U u8_mFdEnable;
    /* */
    INT8U u8_mFdPadding;
    
}ST_FLEXCAN_DATA_INFO_t;

typedef struct
{
    /* */
    INT8U u8_mCanNode;
    /* */
    INT8U u8_mMailBoxIndex;
    /* */
    EN_MAILBOX_DIRECTION_t en_mMailBoxDirection;
    /* */
    ST_FLEXCAN_DATA_INFO_t st_mFlexCanDataInfo;
    /* */
    INT32U u32_mMsgID;  
    /* */
    INT8U u8_EOL;
  
}ST_MAILBOX_CONFIG_t;

typedef struct 
{
    INT32U u32_mArbitrationID;
    INT8U u8_maDataBuff[8];
    INT8U u8_mDataSize;
    
}ST_RX_PDU_t;



/**************************************************************************************************/
/* Export Functions Section                                                                       */
/**************************************************************************************************/

/* */
extern EN_CAN_IF_RESULT_t CAN_IF_Init(void);

/* */
extern EN_CAN_IF_RESULT_t CAN_IF_WriteMsg(INT32U u32_fCANID, INT8U u8_fMONumber, \
                                                        INT8U * u8_fPtr, INT8U u8_fDLC);

/* */
extern EN_CAN_IF_RESULT_t CAN_IF_ISOTP_WriteMsg(INT32U u32_fCANID, INT8U * u8_fPtr, \
                                                                        INT8U u8_fDLC);

/* */
extern EN_CAN_IF_RESULT_t CAN_IF_ConfigRxMO(INT32 u32_fCANID, INT8U u8_fMONumber, \
                                                      EN_CAN_MO_TYPE_t en_fMoDirection);

/* */
extern EN_CAN_IF_RESULT_t CAN_IF_InitIsoTp(ST_ISOTP_CONFIG_PARAMS_t * p_stfIsoTpInitData); 

/* */
extern EN_CAN_IF_RESULT_t CAN_IF_UpdateComHandlers(void (* fp_mComTxHandler)(INT16U), \
                             void (* fp_mComRxHandler)(INT16U u16_fMsgID, void * p_fRxData)); 
/* Function to read received message object data and identifier */
extern EN_CAN_RESULT_t CAN_ReadMsg(ST_CAN_MO_INIT_t * p_stfCANMOInit,  \
				INT8U u8_fMsgobjNo);

/*Function to transmit Data Frame */
extern EN_CAN_RESULT_t CAN_Transmit_DataFrame(INT8U u8_fMsgobjNo);

/* Configure Message Object(MO) */
extern EN_CAN_RESULT_t CAN_ConfigMO(ST_CAN_MO_INIT_t * p_stfCANMOInit);

extern flexcan_state_t * g_flexcanStatePtr[CAN_INSTANCE_COUNT];

extern void CAN_TX_Confirmation(mailBox_t *pValue);
extern void CAN_Rx_Notification(mailBox_t *pValue);

#endif

/**************************************************************************************************/
/* End of CAN_IF.h                                                                                */
/**************************************************************************************************/
