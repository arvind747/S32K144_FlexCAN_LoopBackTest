
#include "Includes.h"

#include "CAN_IF.h"
#include "CAN_IF_COnfig.h"

/* Mail Box Configuration structure */
ST_MAILBOX_CONFIG_t st_gMailBoxConfig[] = 
{
    /* Signal Group for SDM_STAT_CH, ID = 1268 */
    /* BO_ 1268 SDM_STAT_CH: 1 SDM */
    {
        .u8_mCanNode = 0,
        .u8_mMailBoxIndex = 0,
        .en_mMailBoxDirection = EN_MAILBOX_RECEIVE,
        .st_mFlexCanDataInfo = 
        {
            .u8_mDataLength = 8,
            .en_mMsgIdType = EN_FLEXCAN_MSG_ID_STD,
            .u8_mEnableBrs = 0,
            .u8_mFdEnable = 0,
            .u8_mFdPadding = 0
        },
        .u32_mMsgID = 0x7E5, //293, //1268,
    },

    /* Signal Group for EBCM_STA1_CH, ID = 536 */
    /* BO_ 536 EBCM_STA1_CH: 8 EBCM */    
    {
        .u8_mCanNode = 0,
        .u8_mMailBoxIndex = 1,
        .en_mMailBoxDirection = EN_MAILBOX_RECEIVE,
        .st_mFlexCanDataInfo = 
        {
            .u8_mDataLength = 8,
            .en_mMsgIdType = EN_FLEXCAN_MSG_ID_STD,
            .u8_mEnableBrs = 0,
            .u8_mFdEnable = 0,
            .u8_mFdPadding = 0
        },
        .u32_mMsgID = 536,
    },

    /* Signal Group for EBCM_STA3_CH, ID = 600 */
    /* BO_ 600 EBCM_STA3_CH: 8 EBCM */    
    {
        .u8_mCanNode = 0,
        .u8_mMailBoxIndex = 2,
        .en_mMailBoxDirection = EN_MAILBOX_RECEIVE,
        .st_mFlexCanDataInfo = 
        {
            .u8_mDataLength = 8,
            .en_mMsgIdType = EN_FLEXCAN_MSG_ID_STD,
            .u8_mEnableBrs = 0,
            .u8_mFdEnable = 0,
            .u8_mFdPadding = 0
        },
        .u32_mMsgID = 600,
    },

    /* Signal Group for EBCM_STA2_CH, ID = 520 */
    /* BO_ 520 EBCM_STA2_CH: 8 EBCM */
    {
        .u8_mCanNode = 0,
        .u8_mMailBoxIndex = 3,
        .en_mMailBoxDirection = EN_MAILBOX_RECEIVE,
        .st_mFlexCanDataInfo = 
        {
            .u8_mDataLength = 8,
            .en_mMsgIdType = EN_FLEXCAN_MSG_ID_STD,
            .u8_mEnableBrs = 0,
            .u8_mFdEnable = 0,
            .u8_mFdPadding = 0
        },
        .u32_mMsgID = 520,
    },
    
    /* Signal Group for EPCM_STA1_CH, ID = 545 */
    /* BO_ 545 EPCM_STA1_CH: 8 EPCM */
    {
        .u8_mCanNode = 0,
        .u8_mMailBoxIndex = 4,
        .en_mMailBoxDirection = EN_MAILBOX_RECEIVE,
        .st_mFlexCanDataInfo = 
        {
            .u8_mDataLength = 8,
            .en_mMsgIdType = EN_FLEXCAN_MSG_ID_STD,
            .u8_mEnableBrs = 0,
            .u8_mFdEnable = 0,
            .u8_mFdPadding = 0
        },
        .u32_mMsgID = 545,
    },
    
    /* Signal Group for AVAS_STAT_CH, ID = 256 */
    /* BO_ 256 AVAS_STAT_CH: 1 AVAS */
    {
        .u8_mCanNode = 0,
        .u8_mMailBoxIndex = 5,
        .en_mMailBoxDirection = EN_MAILBOX_TRANSMIT,
        .st_mFlexCanDataInfo = 
        {
            .u8_mDataLength = 8,
            .en_mMsgIdType = EN_FLEXCAN_MSG_ID_STD,
            .u8_mEnableBrs = 0,
            .u8_mFdEnable = 0,
            .u8_mFdPadding = 0
        },
        .u32_mMsgID = 256,
    },
    
    /* Signal Group for AVAS_CTRL_CH, ID = 257 */
    /* BO_ 257 AVAS_CTRL_CH: 1 CIU */
    {
        .u8_mCanNode = 0,
        .u8_mMailBoxIndex = 6,
        .en_mMailBoxDirection = EN_MAILBOX_TRANSMIT,
        .st_mFlexCanDataInfo = 
        {
            .u8_mDataLength = 8,
            .en_mMsgIdType = EN_FLEXCAN_MSG_ID_STD,
            .u8_mEnableBrs = 0,
            .u8_mFdEnable = 0,
            .u8_mFdPadding = 0
        },
        .u32_mMsgID = 257,
    },
    
    /* Signal Group for EMCM_STAT_CH, ID = 1278 */
    /* BO_ 1278 EMCM_STAT_CH: 8 CIU */
    {
        .u8_mCanNode = 0,
        .u8_mMailBoxIndex = 7,
        .en_mMailBoxDirection = EN_MAILBOX_RECEIVE,
        .st_mFlexCanDataInfo = 
        {
            .u8_mDataLength = 8,
            .en_mMsgIdType = EN_FLEXCAN_MSG_ID_STD,
            .u8_mEnableBrs = 0,
            .u8_mFdEnable = 0,
            .u8_mFdPadding = 0
        },
        .u32_mMsgID = 1278,
    },
    
    /* Signal Group for SAS_STAT_CH, ID = 801 */
    /* BO_ 801 SAS_STAT_CH: 5 SAS */
    {
        .u8_mCanNode = 0,
        .u8_mMailBoxIndex = 8,
        .en_mMailBoxDirection = EN_MAILBOX_RECEIVE,
        .st_mFlexCanDataInfo = 
        {
            .u8_mDataLength = 8,
            .en_mMsgIdType = EN_FLEXCAN_MSG_ID_STD,
            .u8_mEnableBrs = 0,
            .u8_mFdEnable = 0,
            .u8_mFdPadding = 0
        },
        .u32_mMsgID = 801,
    },
    
    /* Signal Group for EPCM_STA2_CH, ID = 517 */
    /* BO_ 517 EPCM_STA2_CH: 8 EPCM */
    {
        .u8_mCanNode = 0,
        .u8_mMailBoxIndex = 9,
        .en_mMailBoxDirection = EN_MAILBOX_RECEIVE,
        .st_mFlexCanDataInfo = 
        {
            .u8_mDataLength = 8,
            .en_mMsgIdType = EN_FLEXCAN_MSG_ID_STD,
            .u8_mEnableBrs = 0,
            .u8_mFdEnable = 0,
            .u8_mFdPadding = 0
        },
        .u32_mMsgID = 517,
    },
    
    /* Signal Group for EPCM_STA3_CH, ID = 597 */
    /* BO_ 597 EPCM_STA3_CH: 8 EPCM */
    {
        .u8_mCanNode = 0,
        .u8_mMailBoxIndex = 10,
        .en_mMailBoxDirection = EN_MAILBOX_RECEIVE,
        .st_mFlexCanDataInfo = 
        {
            .u8_mDataLength = 8,
            .en_mMsgIdType = EN_FLEXCAN_MSG_ID_STD,
            .u8_mEnableBrs = 0,
            .u8_mFdEnable = 0,
            .u8_mFdPadding = 0
        },
        .u32_mMsgID = 597,
    },
    
    /* Signal Group for EPCM_STA4_CH, ID = 613 */
    /* BO_ 613 EPCM_STA4_CH: 8 EPCM */
    {
        .u8_mCanNode = 0,
        .u8_mMailBoxIndex = 11,
        .en_mMailBoxDirection = EN_MAILBOX_RECEIVE,
        .st_mFlexCanDataInfo = 
        {
            .u8_mDataLength = 8,
            .en_mMsgIdType = EN_FLEXCAN_MSG_ID_STD,
            .u8_mEnableBrs = 0,
            .u8_mFdEnable = 0,
            .u8_mFdPadding = 0
        },
        .u32_mMsgID = 613,
    },
    
    /* Signal Group for EPCM_STA5_CH, ID = 592 */
    /* BO_ 616 EPCM_STA5_CH: 8 EPCM */
    {
        .u8_mCanNode = 0,
        .u8_mMailBoxIndex = 12,
        .en_mMailBoxDirection = EN_MAILBOX_RECEIVE,
        .st_mFlexCanDataInfo = 
        {
            .u8_mDataLength = 8,
            .en_mMsgIdType = EN_FLEXCAN_MSG_ID_STD,
            .u8_mEnableBrs = 0,
            .u8_mFdEnable = 0,
            .u8_mFdPadding = 0
        },
        .u32_mMsgID = 616,
    },
    
    /* Signal Group for GSM_STAT_CH, ID = 592 */
    /* BO_ 592 GSM_STAT_CH: 8 GSM */
    {
        .u8_mCanNode = 0,
        .u8_mMailBoxIndex = 13,
        .en_mMailBoxDirection = EN_MAILBOX_RECEIVE,
        .st_mFlexCanDataInfo = 
        {
            .u8_mDataLength = 8,
            .en_mMsgIdType = EN_FLEXCAN_MSG_ID_STD,
            .u8_mEnableBrs = 0,
            .u8_mFdEnable = 0,
            .u8_mFdPadding = 0
        },
        .u32_mMsgID = 616,
        .u8_EOL = 0,
    },
    
    /* Signal Group for GSM_CTRL_CH, ID = 458 */
    /* BO_ 458 GSM_CTRL_CH: 8 EPCM */
    {
        .u8_mCanNode = 0,
        .u8_mMailBoxIndex = 14,
        .en_mMailBoxDirection = EN_MAILBOX_RECEIVE,
        .st_mFlexCanDataInfo = 
        {
            .u8_mDataLength = 8,
            .en_mMsgIdType = EN_FLEXCAN_MSG_ID_STD,
            .u8_mEnableBrs = 0,
            .u8_mFdEnable = 0,
            .u8_mFdPadding = 0
        },
        .u32_mMsgID = 458,
        .u8_EOL = 0,
    },
    
    /* Signal Group for IPC_ILLM_CH, ID = 1120 */
    /* BO_ 1120 IPC_ILLM_CH: 3 CIU */
    {
        .u8_mCanNode = 0,
        .u8_mMailBoxIndex = 15,
        .en_mMailBoxDirection = EN_MAILBOX_RECEIVE,
        .st_mFlexCanDataInfo = 
        {
            .u8_mDataLength = 8,
            .en_mMsgIdType = EN_FLEXCAN_MSG_ID_STD,
            .u8_mEnableBrs = 0,
            .u8_mFdEnable = 0,
            .u8_mFdPadding = 0
        },
        .u32_mMsgID = 1120,
        .u8_EOL = 1,
    }
};