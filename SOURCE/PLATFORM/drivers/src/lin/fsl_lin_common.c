/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modif ication,
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
 *   software without specif ic prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES,
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fsl_lin_driver.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define BIT(A, B) (((A) >> (B)) & 0x01U) /*!< Return bit B in byte A */

/*******************************************************************************
 * Code
 ******************************************************************************/
/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_DRV_ProcessParity
 * Description   : Makes or checks parity bits. If action is checking parity, the function
 * returns ID value if parity bits are correct or 0xFF if parity bits are incorrect. If action
 * is making parity bits, then from input value of ID, the function returns PID.
 * This is not a public API as it is called by other API functions.
 *
 *END**************************************************************************/
uint8_t LIN_DRV_ProcessParity(uint8_t PID,
                              uint8_t typeAction)
{
    uint8_t parity;
    uint8_t retVal;

    parity = (((BIT(PID, 0U)^BIT(PID, 1U)^BIT(PID, 2U)^BIT(PID, 4U)) << 6U)|
             ((~(BIT(PID, 1U)^BIT(PID, 3U)^BIT(PID, 4U)^BIT(PID, 5U))) << 7U));

    /* Check if action is checking parity bits */
    if (CHECK_PARITY == typeAction)
    {
        /* If parity bits are incorrect */
        if ((PID & 0xC0U) != parity)
        {
            /* Return 0xFF if parity bits are incorrect */
            retVal = 0xFFU;
        }
        /* If parity bits are correct */
        else
        {
            /* Return ID if parity bits are correct */
            retVal = (uint8_t)(PID & 0x3FU);
        }
    }
    /* If action is making parity bits */
    else
    {
        /* Return PID in case of making parity bits */
        retVal = (uint8_t)(PID|parity);
    }

    return (retVal);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LIN_DRV_MakeChecksumByte
 * Description   : Makes the checksum byte for a frame
 *
 *END**************************************************************************/
uint8_t LIN_DRV_MakeChecksumByte(const uint8_t *buffer,
                                 uint8_t sizeBuffer,
                                 uint8_t PID)
{
    uint8_t length;
    uint16_t checksum;
    /* For PID is 0x3C or 0x7D apply classic checksum, and for other PID: Enhanced checksum */
    if ((0x3CU != PID) && (0x7DU != PID))
    {
        /* For PID other than 0x3c and 0x7D: Add PID in checksum calculation */
        checksum = PID;
    }
    else
    {
        /* For 0x3c and 0x7D: Do not add PID in checksum calculation */
        checksum = 0U;
    }

    for (length = sizeBuffer; 0U < length; length--)
    {
        checksum += *(buffer);
        buffer++;
        /* Deal with the carry */
        if (checksum > 0xFFU)
        {
            checksum -= 0xFFU;
        }
    }

    /* Return reversed checksum */
    return (uint8_t)(~checksum);
}
/*******************************************************************************
 * EOF
 ******************************************************************************/
