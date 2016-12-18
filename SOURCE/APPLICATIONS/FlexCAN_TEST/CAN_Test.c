
#include "CAN_Test.h"

/* Set information about the data to be sent */
flexcan_data_info_t g_dataInfo_tx;
flexcan_data_info_t g_dataInfo_rx;

/**************************************************************************************************/
/* Global Definitions Section                                                                     */
/**************************************************************************************************/

/**************************************************************************************************/
/* Global Variables Section                                                                       */
/**************************************************************************************************/

/**************************************************************************************************/
/* Function Name   : FlexCAN_TestInit                                                             */
/*                                                                                                */
/* Description     :                                                                              */
/*                                                                                                */
/* In Params       :                                                                              */
/*                                                                                                */
/*                                                                                                */
/* Out Params      : None                                                                         */
/*                                                                                                */
/* Return Value    : None                                                                         */
/**************************************************************************************************/

int FlexCAN_TestInit(void)
{
    CAN_Module_Init_Loopback();
    Loopback_Test();

    while(1);
    {
        
    }
}

/**************************************************************************************************/
/* Function Name   : CAN_Module_Init_Loopback                                                     */
/*                                                                                                */
/* Description     : initialize can module for loopback                                           */
/*  This function enable clock for CAN module, Initialize CAN driver,                             */
/*  set bit rate, configure message buffer for RX & TX.                                           */
/* In Params       :                                                                              */
/*                                                                                                */
/*                                                                                                */
/* Out Params      : None                                                                         */
/*                                                                                                */
/* Return Value    : None                                                                         */
/**************************************************************************************************/

void CAN_Module_Init_Loopback(void)
{

    /* Initialize and configure clocks
     * 	see clock manager component for details
     */
    CLOCK_SYS_Init(g_clockManConfigsArr, FSL_CLOCK_MANAGER_CONFIG_CNT,
    					g_clockManCallbacksArr, FSL_CLOCK_MANAGER_CALLBACK_CNT);
    CLOCK_SYS_UpdateConfiguration(0U, CLOCK_MANAGER_POLICY_FORCIBLE); 

	/* Initialize pins
	 *-See PinSettings component for more info
	 */
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
	
    g_dataInfo_tx.data_length = 8U;
    g_dataInfo_tx.msg_id_type = FLEXCAN_MSG_ID_STD;
    g_dataInfo_tx.enable_brs = false;
    g_dataInfo_tx.fd_enable = false;
    g_dataInfo_tx.fd_padding = 0U;

    /* Configure Rx message buffer with index 1 and rx_mb_id = 256 */
    FLEXCAN_DRV_ConfigRxMb(FSL_CANCOM1, 1UL, &g_dataInfo_rx, 0x7E5);
	
    /* Configure Tx message buffer with index 0 and tx_mb_id = 257 */
    FLEXCAN_DRV_ConfigTxMb(FSL_CANCOM1, 0UL, &g_dataInfo_tx, 0x7ED);
}

/** @fn void Loopback_Test(void)
*   @brief send and receive data in loopback mode
*
*   This function send some data via mailbox 1 and receive data from
*   mailbox 0.
*/
flexcan_msgbuff_t recvBuff;

/**************************************************************************************************/
/* Function Name   : Loopback_Test                                                                */
/*                                                                                                */
/* Description     : send and receive data in loopback mode                                       */
/*                   This function send some data via mailbox 1 and receive data from             */
/*                   mailbox 0.                                                                   */
/* In Params       :                                                                              */
/*                                                                                                */
/*                                                                                                */
/* Out Params      : None                                                                         */
/*                                                                                                */
/* Return Value    : None                                                                         */
/**************************************************************************************************/

void Loopback_Test(void)
{
    /* CAN TX data */
    uint8_t can_tx_data[8] = {1,2,3,9,4,9,7,8};
	
	/* Define receive buffer */
	recvBuff.cs = 0;
	recvBuff.msgId =0x7FF;
	
	/* Configure Tx message buffer with index 1 */
//	FLEXCAN_DRV_ConfigTxMb(FSL_CANCOM1, 0UL, &g_dataInfo_tx, 257UL);
	
	/* Execute send non-blocking */
//	FLEXCAN_DRV_Send(FSL_CANCOM1, 0UL, &g_dataInfo_tx, 257UL, can_tx_data);
	
	/* Start receiving data in MB 1. */
	FLEXCAN_DRV_RxMessageBuffer(FSL_CANCOM1, 1UL, &recvBuff);

	/* Wait until the previous FlexCAN receive is completed */
	//while(FLEXCAN_DRV_GetReceiveStatus(FSL_CANCOM1) == FLEXCAN_STATUS_RX_BUSY);
}

/**************************************************************************************************/
/* End of CAN_Test.c                                                                              */
/**************************************************************************************************/
