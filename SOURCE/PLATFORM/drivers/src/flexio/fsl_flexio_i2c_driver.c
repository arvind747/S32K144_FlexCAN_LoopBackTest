/*
 * Copyright (c) 2014 - 2016, Freescale Semiconductor, Inc.
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

#include "fsl_flexio_i2c_driver.h"
#include "fsl_interrupt_manager.h"
#include "fsl_flexio_hal.h"
#include "fsl_clock_manager.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @cond DRIVER_INTERNAL_USE_ONLY */

    /* Constraints used for baud rate computation */
#define DIVIDER_MIN_VALUE  1U
#define DIVIDER_MAX_VALUE  0xFFU

    /* Shifters/Timers used for I2C simulation */
#define TX_SHIFTER     0U
#define RX_SHIFTER     1U
#define SCL_TIMER      0U
#define CONTROL_TIMER  (SCL_TIMER+1U)

/* Table of base addresses for FLEXIO instances. */
static FLEXIO_Type * const g_flexioBase[FLEXIO_INSTANCE_COUNT] = FLEXIO_BASE_PTRS;

/* Pointer to runtime state structure.*/
void * g_flexioI2CStatePtr[FLEXIO_INSTANCE_COUNT] = {NULL};

/* Table for FLEXIO IRQ numbers */
static const IRQn_Type g_flexioIrqId[FLEXIO_INSTANCE_COUNT] = FLEXIO_IRQS;

/* PCC clock sources, for getting the input clock frequency */
static const clock_names_t g_flexioClock[FLEXIO_INSTANCE_COUNT] = {PCC_FLEXIO0_CLOCK};

/* Table to save FLEXIO ISRs - for installing interrupt handlers at runtime */
extern const isr_t g_flexioI2CIsr[FLEXIO_INSTANCE_COUNT];

/*******************************************************************************
 * Private Functions
 ******************************************************************************/


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2C_DRV_MasterComputeBaudRateDivider
 * Description   : Computes the baud rate divider for a target baud rate
 *
 *END**************************************************************************/
static flexio_i2c_status_t FLEXIO_I2C_DRV_MasterComputeBaudRateDivider(uint32_t instance, uint32_t baudRate, uint16_t *divider)
{
    uint32_t inputClock;
    uint32_t tmpDiv;
    clock_manager_error_code_t clkErr;

    /* Get the protocol clock frequency */
    clkErr = CLOCK_SYS_GetFreq(g_flexioClock[instance], &inputClock);
    if ((clkErr != CLOCK_MANAGER_SUCCESS) || (inputClock == 0U))
    {
        /* No clock configured for this module */
        return FLEXIO_I2C_STATUS_FAIL;
    }

    /* Compute divider: ((input_clock / baud_rate) / 2) - 1 - 1. The extra -1 is from the 
       timer reset setting used for clock stretching. Round to nearest integer */
    tmpDiv = (inputClock + baudRate) / (2U * baudRate) - 2U;
    /* Enforce upper/lower limits */
    if (tmpDiv < DIVIDER_MIN_VALUE)
    {
        tmpDiv = DIVIDER_MIN_VALUE;
    }
    if (tmpDiv > DIVIDER_MAX_VALUE)
    {
        tmpDiv = DIVIDER_MAX_VALUE;
    }
    
    *divider = (uint16_t)tmpDiv;
    return FLEXIO_I2C_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2C_DRV_MasterConfigure
 * Description   : configures the FLEXIO module as I2C master
 *
 *END**************************************************************************/
static flexio_i2c_status_t FLEXIO_I2C_DRV_MasterConfigure(uint32_t instance)
{
    FLEXIO_Type *baseAddr;
    flexio_i2c_master_state_t *master;
    flexio_i2c_status_t retCode;
    uint16_t divider;

    baseAddr = g_flexioBase[instance];
    master = (flexio_i2c_master_state_t *)g_flexioI2CStatePtr[instance];

    /* Compute divider.*/
    retCode = FLEXIO_I2C_DRV_MasterComputeBaudRateDivider(instance, master->baudRate, &divider);
    if (retCode != FLEXIO_I2C_STATUS_SUCCESS)
    {
        return retCode;
    }

    /* Configure SCL timer */
    FLEXIO_HAL_SetTimerCompare(baseAddr, SCL_TIMER, divider);
    FLEXIO_HAL_SetTimerInitialOutput(baseAddr, SCL_TIMER, FLEXIO_TIMER_INITOUT_ZERO);
    FLEXIO_HAL_SetTimerReset(baseAddr, SCL_TIMER, FLEXIO_TIMER_RESET_PIN_OUT);
    FLEXIO_HAL_SetTimerEnable(baseAddr, SCL_TIMER, FLEXIO_TIMER_ENABLE_TRG_HIGH);
    FLEXIO_HAL_SetTimerDisable(baseAddr, SCL_TIMER, FLEXIO_TIMER_DISABLE_TIM_CMP);
    FLEXIO_HAL_SetTimerStart(baseAddr, SCL_TIMER, FLEXIO_TIMER_START_BIT_ENABLED);
    FLEXIO_HAL_SetTimerStop(baseAddr, SCL_TIMER, FLEXIO_TIMER_STOP_BIT_TIM_DIS);
    FLEXIO_HAL_SetTimerTrigger(baseAddr, SCL_TIMER, (TX_SHIFTER << 2U)+1U, FLEXIO_TRIGGER_POLARITY_LOW, FLEXIO_TRIGGER_SOURCE_INTERNAL);
    FLEXIO_HAL_SetTimerPin(baseAddr, SCL_TIMER, master->sclPin, FLEXIO_PIN_POLARITY_HIGH, FLEXIO_PIN_CONFIG_OPEN_DRAIN);
    FLEXIO_HAL_SetTimerMode(baseAddr, SCL_TIMER, FLEXIO_TIMER_MODE_8BIT_BAUD);
    /* Configure control timer for shifters */
    FLEXIO_HAL_SetTimerCompare(baseAddr, CONTROL_TIMER, 0x000FU);
    FLEXIO_HAL_SetTimerDecrement(baseAddr, CONTROL_TIMER, FLEXIO_TIMER_DECREMENT_PIN_SHIFT_PIN);
    FLEXIO_HAL_SetTimerEnable(baseAddr, CONTROL_TIMER, FLEXIO_TIMER_ENABLE_TIM_ENABLE);
    FLEXIO_HAL_SetTimerDisable(baseAddr, CONTROL_TIMER, FLEXIO_TIMER_DISABLE_TIM_DISABLE);
    FLEXIO_HAL_SetTimerStart(baseAddr, CONTROL_TIMER, FLEXIO_TIMER_START_BIT_ENABLED);
    FLEXIO_HAL_SetTimerStop(baseAddr, CONTROL_TIMER, FLEXIO_TIMER_STOP_BIT_TIM_CMP);
    FLEXIO_HAL_SetTimerTrigger(baseAddr, CONTROL_TIMER, (TX_SHIFTER << 2U)+1U, FLEXIO_TRIGGER_POLARITY_LOW, FLEXIO_TRIGGER_SOURCE_INTERNAL);
    FLEXIO_HAL_SetTimerPin(baseAddr, CONTROL_TIMER, master->sclPin, FLEXIO_PIN_POLARITY_LOW, FLEXIO_PIN_CONFIG_DISABLED);
    FLEXIO_HAL_SetTimerMode(baseAddr, CONTROL_TIMER, FLEXIO_TIMER_MODE_16BIT);

    /* Configure tx shifter */
    FLEXIO_HAL_SetShifterStartBit(baseAddr, TX_SHIFTER, FLEXIO_SHIFTER_START_BIT_0);
    FLEXIO_HAL_SetShifterStopBit(baseAddr, TX_SHIFTER, FLEXIO_SHIFTER_STOP_BIT_1);
    FLEXIO_HAL_SetShifterTimer(baseAddr, TX_SHIFTER, CONTROL_TIMER, FLEXIO_TIMER_POLARITY_POSEDGE);
    FLEXIO_HAL_SetShifterMode(baseAddr, TX_SHIFTER, FLEXIO_SHIFTER_MODE_TRANSMIT);
    FLEXIO_HAL_SetShifterPin(baseAddr, TX_SHIFTER, master->sdaPin, FLEXIO_PIN_POLARITY_LOW, FLEXIO_PIN_CONFIG_OPEN_DRAIN);
    /* Configure rx shifter */
    FLEXIO_HAL_SetShifterStopBit(baseAddr, RX_SHIFTER, FLEXIO_SHIFTER_STOP_BIT_0);
    FLEXIO_HAL_SetShifterTimer(baseAddr, RX_SHIFTER, CONTROL_TIMER, FLEXIO_TIMER_POLARITY_NEGEDGE);
    FLEXIO_HAL_SetShifterPin(baseAddr, RX_SHIFTER, master->sdaPin, FLEXIO_PIN_POLARITY_HIGH, FLEXIO_PIN_CONFIG_DISABLED);
    FLEXIO_HAL_SetShifterMode(baseAddr, RX_SHIFTER, FLEXIO_SHIFTER_MODE_RECEIVE);

    return FLEXIO_I2C_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2C_DRV_MasterSetBytesNo
 * Description   : configures the number of SCL clocks needed for the entire transmission
 *
 *END**************************************************************************/
static void FLEXIO_I2C_DRV_MasterSetBytesNo(FLEXIO_Type *baseAddr, flexio_i2c_master_state_t *master)
{
    uint16_t timerCmp;
    uint16_t bytesNo;

    /* Compute number of SCL ticks, including address */
    bytesNo = master->txRemainingBytes;
    bytesNo = bytesNo * 18U + 1U;
    /* Set number of ticks in low part of timer compare register */
    timerCmp = FLEXIO_HAL_GetTimerCompare(baseAddr, SCL_TIMER);
    timerCmp = (timerCmp & 0x00FFU) | (uint16_t)((bytesNo & 0xFFU) << 8U);
    FLEXIO_HAL_SetTimerCompare(baseAddr, SCL_TIMER, timerCmp);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2C_DRV_MasterSendAddress
 * Description   : send address byte
 *
 *END**************************************************************************/
static void FLEXIO_I2C_DRV_MasterSendAddress(FLEXIO_Type *baseAddr, flexio_i2c_master_state_t *master)
{
    uint8_t addrByte;

    /* Address byte: slave 7-bit address + D = 0(transmit) or 1 (receive) */
    addrByte = (uint8_t)((master->slaveAddress << 1U) + (uint8_t)(master->receive));
    FLEXIO_HAL_WriteShifterBuffer(baseAddr, TX_SHIFTER, addrByte << 24U, FLEXIO_SHIFTER_RW_MODE_BIT_SWAP);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2C_DRV_MasterWaitTransferEnd
 * Description   : waits for the end of a blocking transfer
 *
 *END**************************************************************************/
static flexio_i2c_status_t FLEXIO_I2C_DRV_MasterWaitTransferEnd(uint32_t instance)
{
    flexio_i2c_master_state_t *master;

    master = (flexio_i2c_master_state_t *)g_flexioI2CStatePtr[instance];

    switch (master->driverType)
    {
        case FLEXIO_I2C_DRIVER_TYPE_INTERRUPTS:
            /* Wait for transfer to be completed by the IRQ */
            while (master->driverIdle == false);
            break;
        case FLEXIO_I2C_DRIVER_TYPE_POLLING:
            /* Call FLEXIO_I2C_DRV_MasterGetStatus() to do the transfer */
            while (FLEXIO_I2C_DRV_MasterGetStatus(instance, NULL) == FLEXIO_I2C_STATUS_BUSY);
            break;
        case FLEXIO_I2C_DRIVER_TYPE_DMA:
            /* TODO: add DMA support */
            break;
        default: 
            /* Impossible type - do nothing */
            break;
    }

    return master->status;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2C_DRV_ReadData
 * Description   : Handles data reception
 *
 *END**************************************************************************/
static void FLEXIO_I2C_DRV_ReadData(uint32_t instance)
{
    FLEXIO_Type *baseAddr;
    flexio_i2c_master_state_t *master;
    uint8_t data;

    baseAddr = g_flexioBase[instance];
    master = (flexio_i2c_master_state_t *)g_flexioI2CStatePtr[instance];

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(master->rxRemainingBytes > 0);
#endif

    /* Read data from rx shifter */
    data = (uint8_t)FLEXIO_HAL_ReadShifterBuffer(baseAddr, RX_SHIFTER, FLEXIO_SHIFTER_RW_MODE_BIT_SWAP);

    if (master->addrReceived == false)
    {
        /* This was the address byte */
        master->addrReceived = true;
        if (master->receive == true)
        {
            /* Send ACK from now on */
            FLEXIO_HAL_SetShifterStopBit(baseAddr, TX_SHIFTER, FLEXIO_SHIFTER_STOP_BIT_0);
        }
    }
    else
    {
        master->rxRemainingBytes--;
        if (master->receive == true)
        {
            /* Put data in user buffer */
            *(master->data) = data;
            master->data++;
        }
    }
    if ((master->receive == true) && (master->rxRemainingBytes == 1U))
    {
        /* Send NACK for last byte */
        FLEXIO_HAL_SetShifterStopBit(baseAddr, TX_SHIFTER, FLEXIO_SHIFTER_STOP_BIT_1);
        /* Also instruct rx shifter to expect NACK */
        FLEXIO_HAL_SetShifterStopBit(baseAddr, RX_SHIFTER, FLEXIO_SHIFTER_STOP_BIT_1);
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2C_DRV_WriteData
 * Description   : Handles data transmission
 *
 *END**************************************************************************/
static void FLEXIO_I2C_DRV_WriteData(uint32_t instance)
{
    FLEXIO_Type *baseAddr;
    flexio_i2c_master_state_t *master;
    uint32_t data;

    baseAddr = g_flexioBase[instance];
    master = (flexio_i2c_master_state_t *)g_flexioI2CStatePtr[instance];

    /* If txRemainingBytes == 0 the transmission is over */
    if (master->txRemainingBytes == 0) 
    {
        return;
    }

    master->txRemainingBytes--;

    if (master->txRemainingBytes == 0)
    {
        /* Done transmitting */
        if (master->sendStop == true)
        {
            /* Transmit stop condition */
            data = 0x0;
        }
        else
        {
            /* Do not transmit stop condition */
            data = 0xFFU;
        }
    }
    else if (master->receive == true)
    {
        /* Transmit 0xFF to leave the line free while receiving */
        data = 0xFFU;
    }
    else
    {
        /* Read data from user buffer */
        data =  *(master->data);
        master->data++;
    }

    /* Shift data before bit-swapping it to get the relevant bits in the lower part of the shifter */
    data <<= 24U;
    FLEXIO_HAL_WriteShifterBuffer(baseAddr, TX_SHIFTER, data, FLEXIO_SHIFTER_RW_MODE_BIT_SWAP);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2C_DRV_MasterStopTransfer
 * Description   : Forcefully stops the current transfer
 *
 *END**************************************************************************/
static void FLEXIO_I2C_DRV_MasterStopTransfer(uint32_t instance)
{
    FLEXIO_Type *baseAddr;
    flexio_i2c_master_state_t *master;

    baseAddr = g_flexioBase[instance];
    master = (flexio_i2c_master_state_t *)g_flexioI2CStatePtr[instance];

    master->driverIdle = true;
    /* Call HAL_Init to reset module */
    FLEXIO_HAL_Init(baseAddr);
    /* Will re-initialize module on next use (not here because it takes a 
       long time and we may be inside an interrupt handler) */
    master->needReinit = true;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2C_DRV_MasterEndTransfer
 * Description   : End the current transfer
 *
 *END**************************************************************************/
static void FLEXIO_I2C_DRV_MasterEndTransfer(uint32_t instance)
{
    FLEXIO_Type *baseAddr;
    flexio_i2c_master_state_t *master;

    baseAddr = g_flexioBase[instance];
    master = (flexio_i2c_master_state_t *)g_flexioI2CStatePtr[instance];

    /* Restore Rx stop bit, in case it was changed by a receive */
    FLEXIO_HAL_SetShifterStopBit(baseAddr, RX_SHIFTER, FLEXIO_SHIFTER_STOP_BIT_0);
    /* Clear Rx status in case there is a character left in the buffer */
    FLEXIO_HAL_ClearShifterStatus(baseAddr, RX_SHIFTER);

    /* Disable transfer engine */
    switch (master->driverType)
    {
        case FLEXIO_I2C_DRIVER_TYPE_INTERRUPTS:
            /* Disable interrupts for Rx and Tx shifters */
            FLEXIO_HAL_SetShifterInterrupt(baseAddr, 
                                           (1U << TX_SHIFTER) | (1U << RX_SHIFTER), 
                                           false);
            FLEXIO_HAL_SetShifterErrorInterrupt(baseAddr, 
                                                (1U << TX_SHIFTER) | (1U << RX_SHIFTER), 
                                                false);
            /* Disable interrupt for SCL timer */
            FLEXIO_HAL_SetTimerInterrupt(baseAddr, (1U << SCL_TIMER), false);
            break;
        case FLEXIO_I2C_DRIVER_TYPE_POLLING:
            /* Nothing to do here */
            break;
        case FLEXIO_I2C_DRIVER_TYPE_DMA:
            /* TODO: add DMA support */
            break;
        default: 
            /* Impossible type - do nothing */
            break;
    }

    master->driverIdle = true;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2C_DRV_MasterCheckNack
 * Description   : Checks if the current Rx shifter error is NACK or RX_OVERFLOW
 *                 
 *         If there is a Tx event active it is an indication that module was not 
 *         serviced for a long time - chances are this is an overflow.
 *         It is not certain, and it is even possible to have both NACK and overflow,
 *         but there is no way to tell, so we chose the safe option (if it is an 
 *         overflow and we abort the transfer we may block the I2C bus).
 *                 
 *
 *END**************************************************************************/
static inline bool FLEXIO_I2C_DRV_MasterCheckNack(FLEXIO_Type *baseAddr)
{
    return !(FLEXIO_HAL_GetShifterStatus(baseAddr, TX_SHIFTER));
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2C_DRV_MasterBusBusy
 * Description   : Check status of the I2C bus.
 *                 If either SDA or SCL is low, the bus is busy.
 *
 *END**************************************************************************/
static inline bool FLEXIO_I2C_DRV_MasterBusBusy(FLEXIO_Type *baseAddr, flexio_i2c_master_state_t *master)
{
    uint8_t pinMask = (1 << master->sdaPin) | (1 << master->sclPin);
    if ((FLEXIO_HAL_GetPinData(baseAddr) & pinMask) == pinMask)
    {
        /* both pins are high, bus is not busy */
        return false;
    }
    else
    {
        /* bus is busy */
        return true;
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2C_DRV_MasterCheckStatus
 * Description   : Check status of the I2C transfer. This function can be 
 *                 called either in an interrupt routine or directly in polling 
 *                 mode to advance the I2C transfer.
 *
 *END**************************************************************************/
void FLEXIO_I2C_DRV_MasterCheckStatus(uint32_t instance)
{
    FLEXIO_Type *baseAddr;
    flexio_i2c_master_state_t *master;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FLEXIO_INSTANCE_COUNT);
#endif

    baseAddr = g_flexioBase[instance];
    master = (flexio_i2c_master_state_t *)g_flexioI2CStatePtr[instance];

    /* Check for errors */
    if (FLEXIO_HAL_GetShifterErrorStatus(baseAddr, TX_SHIFTER))
    {
        master->status = FLEXIO_I2C_STATUS_TX_UNDERFLOW;
        FLEXIO_HAL_ClearShifterErrorStatus(baseAddr, TX_SHIFTER);
        /* don't stop the transfer, continue processing events */
    }
    if (FLEXIO_HAL_GetShifterErrorStatus(baseAddr, RX_SHIFTER))
    {
        /* Device limitation: not possible to tell the difference between NACK and receive overflow */
        FLEXIO_HAL_ClearShifterErrorStatus(baseAddr, RX_SHIFTER);
        if (FLEXIO_I2C_DRV_MasterCheckNack(baseAddr))
        {
            master->status = FLEXIO_I2C_STATUS_NACK;
            /* Force the ongoing transfer to stop */
            FLEXIO_I2C_DRV_MasterStopTransfer(instance);
            /* Stop processing events */
            return;
        }
        else
        {
            master->status = FLEXIO_I2C_STATUS_RX_OVERFLOW;
            /* don't stop the transfer, continue processing events */
        }
    }
    /* Check if data was received */
    if (FLEXIO_HAL_GetShifterStatus(baseAddr, RX_SHIFTER))
    {
        FLEXIO_I2C_DRV_ReadData(instance);
    }
    /* Check if transmitter needs more data */
    if (FLEXIO_HAL_GetShifterStatus(baseAddr, TX_SHIFTER))
    {
        FLEXIO_I2C_DRV_WriteData(instance);
        if (master->txRemainingBytes == 0) 
        {
            /* Done transmitting, disable Tx interrupt */
            FLEXIO_HAL_SetShifterInterrupt(baseAddr, (1U << TX_SHIFTER), false);
        }
    }
    /* Check if the transfer is over */
    if (FLEXIO_HAL_GetTimerStatus(baseAddr, SCL_TIMER))
    {
        /* Clear timer status */
        FLEXIO_HAL_ClearTimerStatus(baseAddr, SCL_TIMER);
        /* End transfer */
        FLEXIO_I2C_DRV_MasterEndTransfer(instance);
        /* Record success if there was no error */
        if (master->status == FLEXIO_I2C_STATUS_BUSY)
        {
            master->status = FLEXIO_I2C_STATUS_SUCCESS;
        }
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2C_DRV_MasterStartTransfer
 * Description   : Perform a send or receive transaction on the I2C bus
 *
 *END**************************************************************************/
static flexio_i2c_status_t FLEXIO_I2C_DRV_MasterStartTransfer(uint32_t instance,
                                                              uint8_t * buff,
                                                              uint32_t size,
                                                              bool sendStop,
                                                              bool receive)
{
    FLEXIO_Type *baseAddr;
    flexio_i2c_master_state_t *master;
    flexio_i2c_status_t retCode;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FLEXIO_INSTANCE_COUNT);
#endif

    baseAddr = g_flexioBase[instance];
    master = (flexio_i2c_master_state_t *)g_flexioI2CStatePtr[instance];

    /* Check if driver is busy */
    if (!master->driverIdle)
    {
        return FLEXIO_I2C_STATUS_BUSY;
    }
    /* Check if bus is busy */
    if (FLEXIO_I2C_DRV_MasterBusBusy(baseAddr, master))
    {
        return FLEXIO_I2C_STATUS_BUS_BUSY;
    }
    /* Check if transfer size is in admissible range */
    if (size > FLEXIO_I2C_MAX_SIZE)
    {
        return FLEXIO_I2C_STATUS_SIZE;
    }
    if (master->needReinit)
    {
        /* Re-initialize module after a forced stop */
        retCode = FLEXIO_I2C_DRV_MasterConfigure(instance);
        if (retCode != FLEXIO_I2C_STATUS_SUCCESS)
        {
            return retCode;
        }
        FLEXIO_HAL_SetEnable(baseAddr, true);
        master->needReinit = false;
    }

    /* Initialize transfer data */
    master->data = (uint8_t *)buff;
    /* Tx - one extra byte for stop condition */
    master->txRemainingBytes = size + 1U;
    master->rxRemainingBytes = size;
    master->status = FLEXIO_I2C_STATUS_BUSY;
    master->driverIdle = false;
    master->sendStop = sendStop;
    master->receive = receive;
    master->addrReceived = false;

    /* Configure device for requested number of bytes, keeping the existing baud rate */
    FLEXIO_I2C_DRV_MasterSetBytesNo(baseAddr, master);

    /* Send address */
    FLEXIO_I2C_DRV_MasterSendAddress(baseAddr, master);

    /* Enable transfer engine */
    switch (master->driverType)
    {
        case FLEXIO_I2C_DRIVER_TYPE_INTERRUPTS:
            /* Enable interrupt for Tx and Rx shifters */
            FLEXIO_HAL_SetShifterInterrupt(baseAddr, 
                                           (1U << TX_SHIFTER) | (1U << RX_SHIFTER), 
                                           true);
            FLEXIO_HAL_SetShifterErrorInterrupt(baseAddr, 
                                                (1U << TX_SHIFTER) | (1U << RX_SHIFTER), 
                                                true);
            /* Enable interrupt for SCL timer */
            FLEXIO_HAL_SetTimerInterrupt(baseAddr, (1U << SCL_TIMER), true);
            break;
        case FLEXIO_I2C_DRIVER_TYPE_POLLING:
            /* Nothing to do here, FLEXIO_I2C_DRV_MasterGetStatus() will handle the transfer */
            break;
        case FLEXIO_I2C_DRIVER_TYPE_DMA:
            /* TODO: add DMA support */
            break;
        default: 
            /* Impossible type - do nothing */
            break;
    }

    return FLEXIO_I2C_STATUS_SUCCESS;
}

/*! @endcond */

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2C_DRV_MasterInit
 * Description   : Initialize the FLEXIO_I2C master mode driver
 *
 *END**************************************************************************/
flexio_i2c_status_t FLEXIO_I2C_DRV_MasterInit(uint32_t instance,
                                              const flexio_i2c_master_user_config_t * userConfigPtr,
                                              flexio_i2c_master_state_t * master)
{
    FLEXIO_Type *baseAddr;
    interrupt_manager_error_code_t intErr;
    flexio_i2c_status_t retCode;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(master != NULL);
    DEV_ASSERT(instance < FLEXIO_INSTANCE_COUNT);
#endif

    if (g_flexioI2CStatePtr[instance])
    {
        return FLEXIO_I2C_STATUS_FAIL;
    }

    baseAddr = g_flexioBase[instance];
    g_flexioI2CStatePtr[instance] = master;

    /* Initialize driver context structure */
    master->driverType = userConfigPtr->driverType;
    master->slaveAddress = userConfigPtr->slaveAddress;
    master->sdaPin = userConfigPtr->sdaPin;
    master->sclPin = userConfigPtr->sclPin;
    master->baudRate = userConfigPtr->baudRate;
    master->driverIdle = true;
    master->needReinit = false;

    /* Initialize HAL */
    FLEXIO_HAL_Init(baseAddr);

    /* Configure device for I2C mode */
    retCode = FLEXIO_I2C_DRV_MasterConfigure(instance);
    if (retCode != FLEXIO_I2C_STATUS_SUCCESS)
    {
        g_flexioI2CStatePtr[instance] = NULL;
        return retCode;
    }

    /* Set up transfer engine */
    switch (master->driverType)
    {
        case FLEXIO_I2C_DRIVER_TYPE_INTERRUPTS:
            /* Install irq handler */
            intErr = INT_SYS_InstallHandler(g_flexioIrqId[instance], g_flexioI2CIsr[instance], NULL);
            if (intErr == INTERRUPT_MANAGER_ERROR)
            {
                return FLEXIO_I2C_STATUS_FAIL;
            }
            /* Enable FLEXIO interrupts in the interrupt manager */
            INT_SYS_EnableIRQ(g_flexioIrqId[instance]);
            break;
        case FLEXIO_I2C_DRIVER_TYPE_POLLING:
            /* Nothing to do here, FLEXIO_I2C_DRV_MasterGetStatus() will handle the transfer */
            break;
        case FLEXIO_I2C_DRIVER_TYPE_DMA:
            /* TODO: add DMA support */
            break;
        default: 
            /* Impossible type - do nothing */
            break;
    }

    /* Enable module */
    FLEXIO_HAL_SetEnable(baseAddr, true);

    return FLEXIO_I2C_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2C_DRV_MasterDeinit
 * Description   : De-initialize the FLEXIO_I2C master mode driver
 *
 *END**************************************************************************/
flexio_i2c_status_t FLEXIO_I2C_DRV_MasterDeinit(uint32_t instance)
{
    flexio_i2c_master_state_t *master;
    interrupt_manager_error_code_t intErr;
    FLEXIO_Type *baseAddr;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FLEXIO_INSTANCE_COUNT);
#endif

    baseAddr = g_flexioBase[instance];
    master = (flexio_i2c_master_state_t *)g_flexioI2CStatePtr[instance];

    /* Disable module */
    FLEXIO_HAL_SetEnable(baseAddr, false);

    /* Disable transfer engine */
    if (master != NULL)
    {
        switch (master->driverType)
        {
            case FLEXIO_I2C_DRIVER_TYPE_INTERRUPTS:
                /* Disable FLEXIO interrupt */
                INT_SYS_DisableIRQ(g_flexioIrqId[instance]);
                /* Restore default handler. */
                intErr = INT_SYS_InstallHandler(g_flexioIrqId[instance], DefaultISR, NULL);
                if (intErr == INTERRUPT_MANAGER_ERROR)
                {
                    return FLEXIO_I2C_STATUS_FAIL;
                }
                break;
            case FLEXIO_I2C_DRIVER_TYPE_POLLING:
                /* Nothing to do */
                break;
            case FLEXIO_I2C_DRIVER_TYPE_DMA:
                /* TODO: add DMA support */
                break;
            default: 
                /* Impossible type - do nothing */
                break;
        }
    }

    g_flexioI2CStatePtr[instance] = NULL;

    return FLEXIO_I2C_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2C_DRV_MasterSetBaudRate
 * Description   : Set the baud rate for any subsequent I2C communication
 *
 *END**************************************************************************/
flexio_i2c_status_t FLEXIO_I2C_DRV_MasterSetBaudRate(uint32_t instance, uint32_t baudRate)
{
    FLEXIO_Type *baseAddr;
    flexio_i2c_master_state_t *master;
    uint16_t divider;
    uint16_t timerCmp;
    flexio_i2c_status_t retCode;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FLEXIO_INSTANCE_COUNT);
#endif

    baseAddr = g_flexioBase[instance];
    master = (flexio_i2c_master_state_t *)g_flexioI2CStatePtr[instance];

    /* Check if driver is busy */
    if (!master->driverIdle)
    {
        return FLEXIO_I2C_STATUS_BUSY;
    }

    master->baudRate = baudRate;
    /* Compute divider */
    retCode = FLEXIO_I2C_DRV_MasterComputeBaudRateDivider(instance, baudRate, &divider);
    if (retCode != FLEXIO_I2C_STATUS_SUCCESS)
    {
        return retCode;
    }

    /* Configure timer divider in the lower 8 bits of TIMCMP[CMP] */
    timerCmp = FLEXIO_HAL_GetTimerCompare(baseAddr, SCL_TIMER);
    timerCmp = (timerCmp & 0xFF00U) | divider;
    FLEXIO_HAL_SetTimerCompare(baseAddr, SCL_TIMER, timerCmp);

    return FLEXIO_I2C_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2C_DRV_MasterGetBaudRate
 * Description   : Get the currently configured baud rate
 *
 *END**************************************************************************/
flexio_i2c_status_t FLEXIO_I2C_DRV_MasterGetBaudRate(uint32_t instance, uint32_t *baudRate)
{
    FLEXIO_Type *baseAddr;
    uint32_t inputClock;
    uint32_t divider;
    uint16_t timerCmp;
    clock_manager_error_code_t clkErr;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FLEXIO_INSTANCE_COUNT);
#endif

    baseAddr = g_flexioBase[instance];

    /* Get the protocol clock frequency */
    clkErr = CLOCK_SYS_GetFreq(g_flexioClock[instance], &inputClock);
    if ((clkErr != CLOCK_MANAGER_SUCCESS) || (inputClock == 0U))
    {
        /* No clock configured for this module */
        return FLEXIO_I2C_STATUS_FAIL;
    }

    /* Get the currently configured divider */
    timerCmp = FLEXIO_HAL_GetTimerCompare(baseAddr, SCL_TIMER);
    divider = (uint32_t)(timerCmp & 0x00FFU);

    /* Compute baud rate: input_clock / (2 * (divider + 2)). Round to nearest integer */
    *baudRate = (inputClock + divider + 2U) / (2U * (divider + 2U));

    return FLEXIO_I2C_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2C_DRV_MasterSetSlaveAddr
 * Description   : Set the slave address for any subsequent I2C communication
 *
 *END**************************************************************************/
flexio_i2c_status_t FLEXIO_I2C_DRV_MasterSetSlaveAddr(uint32_t instance, const uint16_t address)
{
    flexio_i2c_master_state_t *master;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FLEXIO_INSTANCE_COUNT);
#endif

    master = (flexio_i2c_master_state_t *)g_flexioI2CStatePtr[instance];

    /* Check if driver is busy */
    if (!master->driverIdle)
    {
        return FLEXIO_I2C_STATUS_BUSY;
    }

    master->slaveAddress = address;
    return FLEXIO_I2C_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2C_DRV_MasterSendData
 * Description   : Perform a non-blocking send transaction on the I2C bus
 *
 *END**************************************************************************/
flexio_i2c_status_t FLEXIO_I2C_DRV_MasterSendData(uint32_t instance,
                                                  const uint8_t * txBuff,
                                                  uint32_t txSize,
                                                  bool sendStop)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FLEXIO_INSTANCE_COUNT);
    DEV_ASSERT(txBuff != NULL);
    DEV_ASSERT(txSize > 0);
#endif
    return FLEXIO_I2C_DRV_MasterStartTransfer(instance, (uint8_t *)txBuff, txSize, sendStop, false);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2C_DRV_MasterSendDataBlocking
 * Description   : Perform a blocking send transaction on the I2C bus
 *
 *END**************************************************************************/
flexio_i2c_status_t FLEXIO_I2C_DRV_MasterSendDataBlocking(uint32_t instance,
                                                          const uint8_t * txBuff,
                                                          uint32_t txSize,
                                                          bool sendStop)
{
    flexio_i2c_status_t status;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FLEXIO_INSTANCE_COUNT);
    DEV_ASSERT(txBuff != NULL);
    DEV_ASSERT(txSize > 0);
#endif

    /* Call FLEXIO_I2C_DRV_MasterSendData to start transfer */
    status = FLEXIO_I2C_DRV_MasterSendData(instance, txBuff, txSize, sendStop);
    if (status != FLEXIO_I2C_STATUS_SUCCESS)
    {
        /* Transfer could not be started */
        return status; 
    }

    /* Wait for transfer to end */
    return FLEXIO_I2C_DRV_MasterWaitTransferEnd(instance);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2C_DRV_MasterReceiveData
 * Description   : Perform a non-blocking receive transaction on the I2C bus
 *
 *END**************************************************************************/
flexio_i2c_status_t FLEXIO_I2C_DRV_MasterReceiveData(uint32_t  instance,
                                                     uint8_t * rxBuff,
                                                     uint32_t rxSize,
                                                     bool sendStop)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FLEXIO_INSTANCE_COUNT);
    DEV_ASSERT(rxBuff != NULL);
    DEV_ASSERT(rxSize > 0);
#endif
    return FLEXIO_I2C_DRV_MasterStartTransfer(instance, rxBuff, rxSize, sendStop, true);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2C_DRV_MasterReceiveDataBlocking
 * Description   : Perform a blocking receive transaction on the I2C bus
 *
 *END**************************************************************************/
flexio_i2c_status_t FLEXIO_I2C_DRV_MasterReceiveDataBlocking(uint32_t instance,
                                                        uint8_t * rxBuff,
                                                        uint32_t rxSize,
                                                        bool sendStop)
{
    flexio_i2c_status_t status;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FLEXIO_INSTANCE_COUNT);
    DEV_ASSERT(rxBuff != NULL);
    DEV_ASSERT(rxSize > 0);
#endif

    /* Call FLEXIO_I2C_DRV_MasterReceiveData to start transfer */
    status = FLEXIO_I2C_DRV_MasterReceiveData(instance, rxBuff, rxSize, sendStop);
    if (status != FLEXIO_I2C_STATUS_SUCCESS)
    {
        /* Transfer could not be started */
        return status; 
    }

    /* Wait for transfer to end */
    return FLEXIO_I2C_DRV_MasterWaitTransferEnd(instance);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2C_DRV_MasterTransferAbort
 * Description   : Aborts a non-blocking I2C master transaction
 *
 *END**************************************************************************/
flexio_i2c_status_t FLEXIO_I2C_DRV_MasterTransferAbort(uint32_t instance)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FLEXIO_INSTANCE_COUNT);
#endif

    /* can't abort safely due to device limitation; there is no way to know the exact 
       stage of the transfer, and if we disable the module during the ACK bit (transmit) 
       or during a data bit (receive) the slave will hold the SDA line low forever and 
       block the I2C bus. NACK reception is the only exception - there is no slave 
       to hold the line low */
    
    return FLEXIO_I2C_STATUS_FAIL;
    
#if 0
    /* Check if driver is busy */
    if (master->driverIdle)
    {
        return FLEXIO_I2C_STATUS_FAIL;
    }

    master->status = FLEXIO_I2C_STATUS_ABORTED;
    /* Force the ongoing transfer to stop */
    FLEXIO_I2C_DRV_MasterStopTransfer(instance);

    return FLEXIO_I2C_STATUS_SUCCESS;
#endif
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_I2C_DRV_MasterGetStatus
 * Description   : Get the status of the current non-blocking I2C master transaction
 *
 *END**************************************************************************/
flexio_i2c_status_t FLEXIO_I2C_DRV_MasterGetStatus(uint32_t instance, uint32_t *bytesRemaining)
{
    flexio_i2c_master_state_t *master;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FLEXIO_INSTANCE_COUNT);
#endif

    master = (flexio_i2c_master_state_t *)g_flexioI2CStatePtr[instance];

    if ((!master->driverIdle) && (master->driverType == FLEXIO_I2C_DRIVER_TYPE_POLLING))
    {
        /* In polling mode advance the I2C transfer here */
        FLEXIO_I2C_DRV_MasterCheckStatus(instance);
    }

    if (bytesRemaining != NULL)
    {
        /* Use rxRemainingBytes even for transmit; byte is not transmitted 
           until rx shifter confirms the ACK */
        *bytesRemaining = master->rxRemainingBytes;
    }

    if (!master->driverIdle)
    {
        return FLEXIO_I2C_STATUS_BUSY;
    }
    else
    {
        return master->status;
    }
}


/*******************************************************************************
 * EOF
 ******************************************************************************/
