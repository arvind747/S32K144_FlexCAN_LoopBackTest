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

#include "fsl_lpi2c_hal.h"

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_HAL_Init
 * Description   : This function initializes the LPI2C module to a known
 * state (the register are written with their reset values from the Reference 
 * Manual).
 *
 *END**************************************************************************/
void LPI2C_HAL_Init(LPI2C_Type *baseAddr)
{
    /* Set all R/W registers to their reset value */
    baseAddr->MCR = 0x0U;
    baseAddr->MIER = 0x0U;
    baseAddr->MDER = 0x0U;
    baseAddr->MCFGR0 = 0x0U;
    baseAddr->MCFGR1 = 0x0U;
    baseAddr->MCFGR2 = 0x0U;
    baseAddr->MCFGR3 = 0x0U;
    baseAddr->MDMR = 0x0U;
    baseAddr->MCCR0 = 0x0U;
    baseAddr->MCCR1 = 0x0U;
    baseAddr->MFCR = 0x0U;
    baseAddr->SCR = 0x0U;
    baseAddr->SIER = 0x0U;
    baseAddr->SDER = 0x0U;
    baseAddr->SCFGR1 = 0x0U;
    baseAddr->SCFGR2 = 0x0U;
    baseAddr->SAMR = 0x0U;
    baseAddr->STAR = 0x0U;

    /* Reset interrupt flags */
    baseAddr->MSR = (LPI2C_HAL_MASTER_DATA_MATCH_INT | 
                     LPI2C_HAL_MASTER_PIN_LOW_TIMEOUT_INT | 
                     LPI2C_HAL_MASTER_FIFO_ERROR_INT | 
                     LPI2C_HAL_MASTER_ARBITRATION_LOST_INT | 
                     LPI2C_HAL_MASTER_NACK_DETECT_INT | 
                     LPI2C_HAL_MASTER_STOP_DETECT_INT | 
                     LPI2C_HAL_MASTER_END_PACKET_INT | 
                     LPI2C_HAL_MASTER_RECEIVE_DATA_INT | 
                     LPI2C_HAL_MASTER_TRANSMIT_DATA_INT);
    baseAddr->SSR = (LPI2C_HAL_SLAVE_SMBUS_ALERT_RESPONSE_INT | 
                     LPI2C_HAL_SLAVE_GENERAL_CALL_INT | 
                     LPI2C_HAL_SLAVE_ADDRESS_MATCH_1_INT | 
                     LPI2C_HAL_SLAVE_ADDRESS_MATCH_0_INT | 
                     LPI2C_HAL_SLAVE_FIFO_ERROR_INT | 
                     LPI2C_HAL_SLAVE_BIT_ERROR_INT | 
                     LPI2C_HAL_SLAVE_STOP_DETECT_INT | 
                     LPI2C_HAL_SLAVE_REPEATED_START_INT | 
                     LPI2C_HAL_SLAVE_TRANSMIT_ACK_INT | 
                     LPI2C_HAL_SLAVE_ADDRESS_VALID_INT | 
                     LPI2C_HAL_SLAVE_RECEIVE_DATA_INT | 
                     LPI2C_HAL_SLAVE_TRANSMIT_DATA_INT);
}
/*******************************************************************************
 * EOF
 ******************************************************************************/
