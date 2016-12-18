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
#include "fsl_lpuart_hal.h"


/*******************************************************************************
 * Code
 ******************************************************************************/
/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_Init
 * Description   : Initializes the LPUART controller to known state, using
 *                 register reset values defined in the reference manual.
 *
 *END**************************************************************************/
void LPUART_HAL_Init(LPUART_Type * base)
{
    /* Set the default oversampling ratio (16) and baud-rate divider (4) */
    base->BAUD = ((uint32_t)((FSL_FEATURE_LPUART_DEFAULT_OSR << LPUART_BAUD_OSR_SHIFT) | \
                 (FSL_FEATURE_LPUART_DEFAULT_SBR << LPUART_BAUD_SBR_SHIFT)));
    /* Clear the error/interrupt flags */
    base->STAT = FSL_FEATURE_LPUART_STAT_REG_FLAGS_MASK;
    /* Reset all features/interrupts by default */
    base->CTRL = 0x00000000;
    /* Reset match addresses */
    base->MATCH = 0x00000000;
#if FSL_FEATURE_LPUART_HAS_MODEM_SUPPORT
    /* Reset IrDA modem features */
    base->MODIR = 0x00000000;
#endif
#if FSL_FEATURE_LPUART_FIFO_SIZE
    /* Reset FIFO feature */
    base->FIFO = FSL_FEATURE_LPUART_FIFO_REG_FLAGS_MASK;
    /* Reset FIFO Watermark values */
    base->WATER = 0x00000000;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_SetBaudRate
 * Description   : Configures the LPUART baud rate.
 * In some LPUART instances the user must disable the transmitter/receiver
 * before calling this function.
 * Generally, this may be applied to all LPUARTs to ensure safe operation.
 *
 *END**************************************************************************/
lpuart_status_t LPUART_HAL_SetBaudRate(LPUART_Type * base,
                                       uint32_t sourceClockInHz,
                                       uint32_t desiredBaudRate)
{
    uint16_t sbr, sbrTemp, i;
    uint32_t osr, tempDiff, calculatedBaud, baudDiff, baudRegValTemp;

    /* This lpuart instantiation uses a slightly different baud rate calculation
     * The idea is to use the best OSR (over-sampling rate) possible
     * Note, osr is typically hard-set to 16 in other lpuart instantiations
     * First calculate the baud rate using the minimum OSR possible (4) */
    osr = 4;
    sbr = (sourceClockInHz/(desiredBaudRate * osr));
    calculatedBaud = (sourceClockInHz / (osr * sbr));

    if (calculatedBaud > desiredBaudRate)
    {
        baudDiff = calculatedBaud - desiredBaudRate;
    }
    else
    {
        baudDiff = desiredBaudRate - calculatedBaud;
    }

    /* loop to find the best osr value possible, one that generates minimum baudDiff
     * iterate through the rest of the supported values of osr */
    for (i = 5; i <= 32; i++)
    {
        /* calculate the temporary sbr value   */
        sbrTemp = (sourceClockInHz/(desiredBaudRate * i));
        /* calculate the baud rate based on the temporary osr and sbr values */
        calculatedBaud = (sourceClockInHz / (i * sbrTemp));

        if (calculatedBaud > desiredBaudRate)
        {
            tempDiff = calculatedBaud - desiredBaudRate;
        }
        else
        {
            tempDiff = desiredBaudRate - calculatedBaud;
        }

        if (tempDiff <= baudDiff)
        {
            baudDiff = tempDiff;
            osr = i;  /* update and store the best osr value calculated */
            sbr = sbrTemp;  /* update store the best sbr value calculated */
        }
    }

    /* Check to see if actual baud rate is within 3% of desired baud rate
     * based on the best calculate osr value */
    if (baudDiff < ((desiredBaudRate / 100) * 3))
    {
        /* Acceptable baud rate, check if osr is between 4x and 7x oversampling.
         * If so, then "BOTHEDGE" sampling must be turned on */
        if ((osr > 3) && (osr < 8))
        {
            BITBAND_ACCESS32(&(base->BAUD), LPUART_BAUD_BOTHEDGE_SHIFT) = (uint32_t)1U;
        }

        /* program the osr value (bit value is one less than actual value) */
        baudRegValTemp = base->BAUD;
        baudRegValTemp &= ~(LPUART_BAUD_OSR_MASK);
        baudRegValTemp |= LPUART_BAUD_OSR(osr-1);
        base->BAUD = baudRegValTemp;

        /* write the sbr value to the BAUD registers */
        baudRegValTemp = base->BAUD;
        baudRegValTemp &= ~(LPUART_BAUD_SBR_MASK);
        baudRegValTemp |= LPUART_BAUD_SBR(sbr);
        base->BAUD = baudRegValTemp;
    }
    else
    {
        /* Unacceptable baud rate difference of more than 3% */
        return LPUART_STAT_BAUD_RATE_CALCULATION_ERROR;
    }

    return LPUART_STAT_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_SetBitCountPerChar
 * Description   : Configures the number of bits per char in LPUART controller.
 * In some LPUART instances, the user should disable the transmitter/receiver
 * before calling this function.
 * Generally, this may be applied to all LPUARTs to ensure safe operation.
 *
 *END**************************************************************************/
void LPUART_HAL_SetBitCountPerChar(LPUART_Type * base,
                                   lpuart_bit_count_per_char_t bitCountPerChar)
{
    if (bitCountPerChar == LPUART_10_BITS_PER_CHAR)
    {
        BITBAND_ACCESS32(&(base->BAUD), LPUART_BAUD_M10_SHIFT) = (uint32_t)1U;
    }
    else
    {
        /* config 8-bit (M=0) or 9-bits (M=1) */
        BITBAND_ACCESS32(&(base->CTRL), LPUART_CTRL_M_SHIFT) = (uint32_t)bitCountPerChar;
        /* clear M10 to make sure not 10-bit mode */
        BITBAND_ACCESS32(&(base->BAUD), LPUART_BAUD_M10_SHIFT) = (uint32_t)0U;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_SetParityMode
 * Description   : Configures parity mode in the LPUART controller.
 * In some LPUART instances, the user should disable the transmitter/receiver
 * before calling this function.
 * Generally, this may be applied to all LPUARTs to ensure safe operation.
 *
 *END**************************************************************************/
void LPUART_HAL_SetParityMode(LPUART_Type * base, lpuart_parity_mode_t parityModeType)
{
    BITBAND_ACCESS32(&(base->CTRL), LPUART_CTRL_PE_SHIFT) = (uint32_t)(parityModeType >> 1U);
    BITBAND_ACCESS32(&(base->CTRL), LPUART_CTRL_PT_SHIFT) = (uint32_t)(parityModeType & 1U);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_Putchar9
 * Description   : Sends the LPUART 9-bit character.
 *
 *END**************************************************************************/
void LPUART_HAL_Putchar9(LPUART_Type * base, uint16_t data)
{
    uint8_t ninthDataBit;
    uint8_t * dataRegBytes = (uint8_t *)(&(base->DATA));
    

    ninthDataBit = (data >> 8U) & 0x1U;

    /* write to ninth data bit T8(where T[0:7]=8-bits, T8=9th bit) */
    BITBAND_ACCESS32(&(base->CTRL), LPUART_CTRL_R9T8_SHIFT) = (uint32_t)ninthDataBit;

    /* write 8-bits to the data register*/
    dataRegBytes[0] = data;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_Putchar10
 * Description   : Sends the LPUART 10-bit character.
 *
 *END**************************************************************************/
lpuart_status_t LPUART_HAL_Putchar10(LPUART_Type * base, uint16_t data)
{
    uint8_t ninthDataBit, tenthDataBit;
    uint8_t * dataRegBytes = (uint8_t *)(&(base->DATA));

    ninthDataBit = (data >> 8U) & 0x1U;
    tenthDataBit = (data >> 9U) & 0x1U;

    /* write to ninth/tenth data bit (T[0:7]=8-bits, T8=9th bit, T9=10th bit) */
    BITBAND_ACCESS32(&(base->CTRL), LPUART_CTRL_R9T8_SHIFT) = (uint32_t)ninthDataBit;
    BITBAND_ACCESS32(&(base->CTRL), LPUART_CTRL_R8T9_SHIFT) = (uint32_t)tenthDataBit;

    /* write to 8-bits to the data register */
    dataRegBytes[0] = data;

    return LPUART_STAT_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_Getchar9
 * Description   : Gets the LPUART 9-bit character.
 *
 *END**************************************************************************/
void LPUART_HAL_Getchar9(LPUART_Type * base, uint16_t *readData)
{
    /* get ninth bit from lpuart data register */
    *readData = (uint16_t)(BITBAND_ACCESS32(&(base->CTRL), LPUART_CTRL_R8T9_SHIFT) << 8);

    /* get 8-bit data from the lpuart data register */
    *readData |= (uint8_t)base->DATA;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_Getchar10
 * Description   : Gets the LPUART 10-bit character, available only on
 *                 supported lpuarts
 *
 *END**************************************************************************/
void LPUART_HAL_Getchar10(LPUART_Type * base, uint16_t *readData)
{
    /* read tenth data bit */
    *readData = (uint16_t)((uint32_t)(BITBAND_ACCESS32(&(base->CTRL), LPUART_CTRL_R9T8_SHIFT)) << 9U);

    /* read ninth data bit */
    *readData |= (uint16_t)((uint32_t)(BITBAND_ACCESS32(&(base->CTRL), LPUART_CTRL_R8T9_SHIFT)) << 8U);

    /* get 8-bit data */
    *readData |= (uint8_t)base->DATA;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_SendDataPolling
 * Description   : Send out multiple bytes of data using polling method.
 * This function only supports 8-bit transaction.
 *
 *END**************************************************************************/
void LPUART_HAL_SendDataPolling(LPUART_Type * base,
                                const uint8_t *txBuff,
                                uint32_t txSize)
{
    while (txSize--)
    {
        while (!BITBAND_ACCESS32(&(base->STAT), LPUART_STAT_TDRE_SHIFT))
        {}

        LPUART_HAL_Putchar(base, *txBuff++);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_ReceiveDataPolling
 * Description   : Receive multiple bytes of data using polling method.
 * This function only supports 8-bit transaction.
 *
 *END**************************************************************************/
lpuart_status_t LPUART_HAL_ReceiveDataPolling(LPUART_Type * base,
                                              uint8_t *rxBuff,
                                              uint32_t rxSize)
{
    lpuart_status_t retVal = LPUART_STAT_SUCCESS;

    while (rxSize--)
    {
        while (!BITBAND_ACCESS32(&(base->STAT), LPUART_STAT_RDRF_SHIFT))
        {}

        LPUART_HAL_Getchar(base, rxBuff++);

        /* Clear the Overrun flag since it will block receiving */
        if (BITBAND_ACCESS32(&(base->STAT), LPUART_STAT_OR_SHIFT))
        {
            BITBAND_ACCESS32(&(base->STAT), LPUART_STAT_OR_SHIFT) = (uint32_t)1U;
            retVal = LPUART_STAT_RX_OVERRUN;
        }
    }

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_SetIntMode
 * Description   : Configures the LPUART module interrupts to enable/disable
 * various interrupt sources.
 *
 *END**************************************************************************/
void LPUART_HAL_SetIntMode(LPUART_Type * base, lpuart_interrupt_t intSrc, bool enable)
{
    uint32_t reg = (uint32_t)(intSrc) >> LPUART_SHIFT;
    uint32_t intRegOffset = (uint16_t)(intSrc);

    switch ( reg )
    {
        case LPUART_BAUD_REG_ID:
            BITBAND_ACCESS32(&(base->BAUD), intRegOffset) = (uint32_t)enable;
            break;
        case LPUART_STAT_REG_ID:
            BITBAND_ACCESS32(&(base->STAT), intRegOffset) = (uint32_t)enable;
            break;
        case LPUART_CTRL_REG_ID:
            BITBAND_ACCESS32(&(base->CTRL), intRegOffset) = (uint32_t)enable;
            break;
        case LPUART_DATA_REG_ID:
            BITBAND_ACCESS32(&(base->DATA), intRegOffset) = (uint32_t)enable;
            break;
        case LPUART_MATCH_REG_ID:
            BITBAND_ACCESS32(&(base->MATCH), intRegOffset) = (uint32_t)enable;
            break;
#if FSL_FEATURE_LPUART_HAS_MODEM_SUPPORT
        case LPUART_MODIR_REG_ID:
            BITBAND_ACCESS32(&(base->MODIR), intRegOffset) = (uint32_t)enable;
            break;
#endif
#if FSL_FEATURE_LPUART_FIFO_SIZE
        case LPUART_FIFO_REG_ID:
            BITBAND_ACCESS32(&(base->FIFO), intRegOffset) = (uint32_t)enable;
            break;
#endif
        default :
            break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_GetIntMode
 * Description   : Returns whether LPUART module interrupt is enabled/disabled.
 *
 *END**************************************************************************/
bool LPUART_HAL_GetIntMode(LPUART_Type * base, lpuart_interrupt_t intSrc)
{
    uint32_t reg = (uint32_t)(intSrc) >> LPUART_SHIFT;
    bool retVal = false;

    switch ( reg )
    {
        case LPUART_BAUD_REG_ID:
            retVal = (bool)(base->BAUD >> (uint16_t)(intSrc) & 1U);
            break;
        case LPUART_STAT_REG_ID:
            retVal = (bool)(base->STAT >> (uint16_t)(intSrc) & 1U);
            break;
        case LPUART_CTRL_REG_ID:
            retVal = (bool)(base->CTRL >> (uint16_t)(intSrc) & 1U);
            break;
        case LPUART_DATA_REG_ID:
            retVal = (bool)(base->DATA >> (uint16_t)(intSrc) & 1U);
            break;
        case LPUART_MATCH_REG_ID:
            retVal = (bool)(base->MATCH >> (uint16_t)(intSrc) & 1U);
            break;
#if FSL_FEATURE_LPUART_HAS_MODEM_SUPPORT
        case LPUART_MODIR_REG_ID:
            retVal = (bool)(base->MODIR >> (uint16_t)(intSrc) & 1U);
            break;
#endif
        default :
            break;
    }

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_SetLoopbackCmd
 * Description   : Configures the LPUART loopback operation (enable/disable
 *                 loopback operation)
 * In some LPUART instances, the user should disable the transmitter/receiver
 * before calling this function.
 * Generally, this may be applied to all LPUARTs to ensure safe operation.
 *
 *END**************************************************************************/
void LPUART_HAL_SetLoopbackCmd(LPUART_Type * base, bool enable)
{
    /* configure LOOPS bit to enable(1)/disable(0) loopback mode, but also need
     * to clear RSRC */
    BITBAND_ACCESS32(&(base->CTRL), LPUART_CTRL_LOOPS_SHIFT) = (uint32_t)enable;

    /* clear RSRC for loopback mode, and if loopback disabled, */
    /* this bit has no meaning but clear anyway */
    /* to set it back to default value */
    BITBAND_ACCESS32(&(base->CTRL), LPUART_CTRL_RSRC_SHIFT) = (uint32_t)0U;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_SetSingleWireCmd
 * Description   : Configures the LPUART single-wire operation (enable/disable
 *                 single-wire mode)
 * In some LPUART instances, the user should disable the transmitter/receiver
 * before calling this function.
 * Generally, this may be applied to all LPUARTs to ensure safe operation.
 *
 *END**************************************************************************/
void LPUART_HAL_SetSingleWireCmd(LPUART_Type * base, bool enable)
{
    /* to enable single-wire mode, need both LOOPS and RSRC set,
     * to enable or clear both */
    BITBAND_ACCESS32(&(base->CTRL), LPUART_CTRL_LOOPS_SHIFT) = (uint32_t)enable;
    BITBAND_ACCESS32(&(base->CTRL), LPUART_CTRL_RSRC_SHIFT) = (uint32_t)enable;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_SetReceiverInStandbyMode
 * Description   : Places the LPUART receiver in standby mode.
 * In some LPUART instances, before placing LPUART in standby mode, first
 * determine whether the receiver is set to wake on idle or whether it is
 * already in idle state.
 *
 *END**************************************************************************/
lpuart_status_t LPUART_HAL_SetReceiverInStandbyMode(LPUART_Type * base)
{
    lpuart_wakeup_method_t rxWakeMethod;
    bool lpuart_current_rx_state;

    rxWakeMethod = LPUART_HAL_GetReceiverWakeupMode(base);
    lpuart_current_rx_state = LPUART_HAL_GetStatusFlag(base, LPUART_RX_ACTIVE);

    /* if both rxWakeMethod is set for idle and current rx state is idle,
     * don't put in standby */
    if ((rxWakeMethod == LPUART_IDLE_LINE_WAKE) && (lpuart_current_rx_state == 0))
    {
        return LPUART_STAT_RX_STAND_BY_MODE_ERROR;
    }
    else
    {
        /* set the RWU bit to place receiver into standby mode */
        BITBAND_ACCESS32(&(base->CTRL), LPUART_CTRL_RWU_SHIFT) = (uint32_t)1U;
        return LPUART_STAT_SUCCESS;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_SetIdleLineDetect
 * Description   : LPUART idle-line detect operation configuration (idle line
 * bit-count start and wake up affect on IDLE status bit).
 * In some LPUART instances, the user should disable the transmitter/receiver
 * before calling this function.
 * Generally, this may be applied to all LPUARTs to ensure safe operation.
 *
 *END**************************************************************************/
void LPUART_HAL_SetIdleLineDetect(LPUART_Type * base,
                                  const lpuart_idle_line_config_t *config)
{
    /* Configure the idle line detection configuration as follows:
     * configure the ILT to bit count after start bit or stop bit
     * configure RWUID to set or not set IDLE status bit upon detection of
     * an idle character when receiver in standby */
    BITBAND_ACCESS32(&(base->CTRL), LPUART_CTRL_ILT_SHIFT) = (uint32_t)config->idleLineType;
    BITBAND_ACCESS32(&(base->STAT), LPUART_STAT_RWUID_SHIFT) = (uint32_t)config->rxWakeIdleDetect;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_SetMatchAddressReg1
 * Description   : Configures match address register 1
 *
 *END**************************************************************************/
void LPUART_HAL_SetMatchAddressReg1(LPUART_Type * base, bool enable, uint8_t value)
{
    uint32_t matchRegValTemp;

    /* The MAEN bit must be cleared before configuring MA value */
    BITBAND_ACCESS32(&(base->BAUD), LPUART_BAUD_MAEN1_SHIFT) = (uint32_t)0U;
    if (enable)
    {
        matchRegValTemp = base->MATCH;
        matchRegValTemp &= ~(LPUART_MATCH_MA1_MASK);
        matchRegValTemp |= LPUART_MATCH_MA1(value);
        base->MATCH = matchRegValTemp;

        BITBAND_ACCESS32(&(base->BAUD), LPUART_BAUD_MAEN1_SHIFT) = (uint32_t)1U;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_SetMatchAddressReg2
 * Description   : Configures match address register 2
 *
 *END**************************************************************************/
void LPUART_HAL_SetMatchAddressReg2(LPUART_Type * base, bool enable, uint8_t value)
{
    uint32_t matchRegValTemp;

    /* The MAEN bit must be cleared before configuring MA value */
    BITBAND_ACCESS32(&(base->BAUD), LPUART_BAUD_MAEN2_SHIFT) = (uint32_t)0U;
    if (enable)
    {
        matchRegValTemp = base->MATCH;
        matchRegValTemp &= ~(LPUART_MATCH_MA2_MASK);
        matchRegValTemp |= LPUART_MATCH_MA2(value);
        base->MATCH = matchRegValTemp;

        BITBAND_ACCESS32(&(base->BAUD), LPUART_BAUD_MAEN2_SHIFT) = (uint32_t)1U;
    }
}

#if FSL_FEATURE_LPUART_HAS_IR_SUPPORT
/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_SetInfrared
 * Description   : Configures the LPUART infrared operation.
 *
 *END**************************************************************************/
void LPUART_HAL_SetInfrared(LPUART_Type * base, bool enable,
                            lpuart_ir_tx_pulsewidth_t pulseWidth)
{
    uint32_t modirRegValTemp;

    /* enable or disable infrared */
    BITBAND_ACCESS32(&(base->MODIR), LPUART_MODIR_IREN_SHIFT) = (uint32_t)enable;

    /* configure the narrow pulse width of the IR pulse */
    modirRegValTemp = base->MODIR;
    modirRegValTemp &= ~(LPUART_MODIR_TNP_MASK);
    modirRegValTemp |= LPUART_MODIR_TNP(pulseWidth);
    base->MODIR = modirRegValTemp;
}
#endif  /* FSL_FEATURE_LPUART_HAS_IR_SUPPORT */

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_GetStatusFlag
 * Description   : LPUART get status flag by passing flag enum.
 *
 *END**************************************************************************/
bool LPUART_HAL_GetStatusFlag(LPUART_Type * base, lpuart_status_flag_t statusFlag)
{
    uint32_t reg = (uint32_t)(statusFlag) >> LPUART_SHIFT;
    bool retVal = false;

    switch ( reg )
    {
        case LPUART_BAUD_REG_ID:
            retVal = (bool)(base->BAUD >> (uint32_t)(statusFlag) & 1U);
            break;
        case LPUART_STAT_REG_ID:
            retVal = (bool)(base->STAT >> (uint32_t)(statusFlag) & 1U);
            break;
        case LPUART_CTRL_REG_ID:
            retVal = (bool)(base->CTRL >> (uint32_t)(statusFlag) & 1U);
            break;
        case LPUART_DATA_REG_ID:
            retVal = (bool)(base->DATA >> (uint32_t)(statusFlag) & 1U);
            break;
        case LPUART_MATCH_REG_ID:
            retVal = (bool)(base->MATCH >> (uint32_t)(statusFlag) & 1U);
            break;
#if FSL_FEATURE_LPUART_HAS_MODEM_SUPPORT
        case LPUART_MODIR_REG_ID:
            retVal = (bool)(base->MODIR >> (uint32_t)(statusFlag) & 1U);
            break;
#endif
        default:
            break;
    }

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_ClearStatusFlag
 * Description   : LPUART clears an individual status flag
 * (see lpuart_status_flag_t for list of status bits).
 *
 *END**************************************************************************/
lpuart_status_t LPUART_HAL_ClearStatusFlag(LPUART_Type * base,
                                           lpuart_status_flag_t statusFlag)
{
    lpuart_status_t returnCode = LPUART_STAT_SUCCESS;

    switch(statusFlag)
    {
        /* These flags are cleared automatically by other lpuart operations
         * and cannot be manually cleared, return error code */
        case LPUART_TX_DATA_REG_EMPTY:
        case LPUART_TX_COMPLETE:
        case LPUART_RX_DATA_REG_FULL:
        case LPUART_RX_ACTIVE:
#if FSL_FEATURE_LPUART_HAS_EXTENDED_DATA_REGISTER_FLAGS
        case LPUART_NOISE_IN_CURRENT_WORD:
        case LPUART_PARITY_ERR_IN_CURRENT_WORD:
#endif
            returnCode = LPUART_STAT_CLEAR_STATUS_FLAG_ERROR;
            break;

        case LPUART_IDLE_LINE_DETECT:
            BITBAND_ACCESS32(&(base->STAT), LPUART_STAT_IDLE_SHIFT) = (uint32_t)1U;
            break;

        case LPUART_RX_OVERRUN:
            BITBAND_ACCESS32(&(base->STAT), LPUART_STAT_OR_SHIFT) = (uint32_t)1U;
            break;

        case LPUART_NOISE_DETECT:
            BITBAND_ACCESS32(&(base->STAT), LPUART_STAT_NF_SHIFT) = (uint32_t)1U;
            break;

        case LPUART_FRAME_ERR:
            BITBAND_ACCESS32(&(base->STAT), LPUART_STAT_FE_SHIFT) = (uint32_t)1U;
            break;

        case LPUART_PARITY_ERR:
            BITBAND_ACCESS32(&(base->STAT), LPUART_STAT_PF_SHIFT) = (uint32_t)1U;
            break;

        case LPUART_LIN_BREAK_DETECT:
            BITBAND_ACCESS32(&(base->STAT), LPUART_STAT_LBKDIF_SHIFT) = (uint32_t)1U;
            break;

        case LPUART_RX_ACTIVE_EDGE_DETECT:
            BITBAND_ACCESS32(&(base->STAT), LPUART_STAT_RXEDGIF_SHIFT) = (uint32_t)1U;
            break;

#if FSL_FEATURE_LPUART_HAS_ADDRESS_MATCHING
        case LPUART_MATCH_ADDR_ONE:
            BITBAND_ACCESS32(&(base->STAT), LPUART_STAT_MA1F_SHIFT) = (uint32_t)1U;
            break;
        case LPUART_MATCH_ADDR_TWO:
            BITBAND_ACCESS32(&(base->STAT), LPUART_STAT_MA2F_SHIFT) = (uint32_t)1U;
            break;
#endif
#if FSL_FEATURE_LPUART_FIFO_SIZE
        case LPUART_FIFO_TX_OF:
            BITBAND_ACCESS32(&(base->FIFO), LPUART_FIFO_TXOF_SHIFT) = (uint32_t)1U;
            break;
        case LPUART_FIFO_RX_UF:
            BITBAND_ACCESS32(&(base->FIFO), LPUART_FIFO_RXUF_SHIFT) = (uint32_t)1U;
            break;
#endif
        default:
            returnCode = LPUART_STAT_CLEAR_STATUS_FLAG_ERROR;
            break;
    }

    return (returnCode);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

