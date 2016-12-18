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

#include "fsl_flexio_spi_driver.h"
#include "fsl_interrupt_manager.h"
#include "fsl_flexio_hal.h"
#include "fsl_clock_manager.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @cond DRIVER_INTERNAL_USE_ONLY */

    /* Constraints used for baud rate computation */
#define DIVIDER_MIN_VALUE 1U
#define DIVIDER_MAX_VALUE 0xFFU

    /* Shifters/Timers used for SPI simulation */
#define TX_SHIFTER     0U
#define RX_SHIFTER     1U
#define SCK_TIMER        0U
#define SS_TIMER         1U

    /* Dummy data to send when user provides no data */
#define FLEXIO_SPI_DUMMYDATA (0xFFFFFFFFU)

/* Table of base addresses for FLEXIO instances. */
FLEXIO_Type * const g_flexioBase[FLEXIO_INSTANCE_COUNT] = FLEXIO_BASE_PTRS;

/* Pointer to runtime state structure.*/
void * g_flexioSPIStatePtr[FLEXIO_INSTANCE_COUNT] = {NULL};

/* Table for FLEXIO IRQ numbers */
const IRQn_Type g_flexioIrqId[FLEXIO_INSTANCE_COUNT] = FLEXIO_IRQS;

/* PCC clock sources, for getting the input clock frequency */
const clock_names_t g_flexioClock[FLEXIO_INSTANCE_COUNT] = {PCC_FLEXIO0_CLOCK};

/* Table to save FLEXIO ISRs - for installing interrupt handlers at runtime */
extern const isr_t g_flexioIsr[FLEXIO_INSTANCE_COUNT];

/*******************************************************************************
 * Private Functions
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_SPI_DRV_MasterComputeBaudRateDivider
 * Description   : Computes the baud rate divider for a target baud rate
 *
 *END**************************************************************************/
static flexio_spi_status_t FLEXIO_SPI_DRV_MasterComputeBaudRateDivider(uint32_t instance, uint32_t baudRate, uint16_t *divider)
{
    uint32_t inputClock;
    uint32_t tmpDiv;
    clock_manager_error_code_t clkErr;

    /* Get the protocol clock frequency */
    clkErr = CLOCK_SYS_GetFreq(g_flexioClock[instance], &inputClock);
    if ((clkErr != CLOCK_MANAGER_SUCCESS) || (inputClock == 0U))
    {
        /* No clock configured for this module */
        return FLEXIO_SPI_STATUS_FAIL;
    }

    /* Compute divider: ((input_clock / baud_rate) / 2) - 1. Round to nearest integer */
    tmpDiv = (inputClock + baudRate) / (2U * baudRate) - 1U;
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
    return FLEXIO_SPI_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_SPI_DRV_MasterConfigure
 * Description   : configures the FLEXIO module as SPI master
 *
 *END**************************************************************************/
static flexio_spi_status_t FLEXIO_SPI_DRV_MasterConfigure(uint32_t instance)
{
    FLEXIO_Type *baseAddr;
    flexio_spi_master_state_t *master;
    flexio_pin_polarity_t clockPolarity;        /* Polarity of clock signal */
    flexio_timer_polarity_t txShifterPolarity;  /* Polarity of MOSI shifter */
    flexio_timer_polarity_t rxShifterPolarity;  /* Polarity of MISO shifter */
    uint16_t divider;
    flexio_spi_status_t retCode;

    baseAddr = g_flexioBase[instance];
    master = (flexio_spi_master_state_t *)g_flexioSPIStatePtr[instance];
 
    /* Compute divider.*/
    retCode = FLEXIO_SPI_DRV_MasterComputeBaudRateDivider(instance, master->baudRate, &divider);
    if (retCode != FLEXIO_SPI_STATUS_SUCCESS)
    {
        return retCode;
    }

    if (master->clockPolarity == 0)
    {
        /* CPOL = 0 */
        clockPolarity = FLEXIO_PIN_POLARITY_HIGH;
    }
    else
    {
        /* CPOL = 1 */
        clockPolarity = FLEXIO_PIN_POLARITY_LOW;
    }

    if (master->clockPhase == 0)
    {
        /* CPHA = 0 */
        txShifterPolarity = FLEXIO_TIMER_POLARITY_NEGEDGE;
        rxShifterPolarity = FLEXIO_TIMER_POLARITY_POSEDGE;
    }
    else
    {
        /* CPHA = 1 */
        txShifterPolarity = FLEXIO_TIMER_POLARITY_POSEDGE;
        rxShifterPolarity = FLEXIO_TIMER_POLARITY_NEGEDGE;
    }
    /* Configure Tx shifter (MOSI) */
    FLEXIO_HAL_SetShifterTimer(baseAddr, TX_SHIFTER, SCK_TIMER, txShifterPolarity);
    FLEXIO_HAL_SetShifterPin(baseAddr, TX_SHIFTER, master->mosiPin, FLEXIO_PIN_POLARITY_HIGH, FLEXIO_PIN_CONFIG_OUTPUT);
    FLEXIO_HAL_SetShifterMode(baseAddr, TX_SHIFTER, FLEXIO_SHIFTER_MODE_TRANSMIT);

    if (master->clockPhase != 0U)
    {
        /* CPHA = 1 */
        FLEXIO_HAL_SetShifterStopBit(baseAddr, TX_SHIFTER, FLEXIO_SHIFTER_STOP_BIT_0);
        FLEXIO_HAL_SetShifterStartBit(baseAddr, TX_SHIFTER, FLEXIO_SHIFTER_START_BIT_DISABLED_SH);
    }

    /* Configure Rx shifter (MISO) */
    FLEXIO_HAL_SetShifterTimer(baseAddr, RX_SHIFTER, SCK_TIMER, rxShifterPolarity);
    FLEXIO_HAL_SetShifterPin(baseAddr, RX_SHIFTER, master->misoPin, FLEXIO_PIN_POLARITY_HIGH, FLEXIO_PIN_CONFIG_DISABLED);
    FLEXIO_HAL_SetShifterMode(baseAddr, RX_SHIFTER, FLEXIO_SHIFTER_MODE_RECEIVE);

    /* Configure sck timer */
    FLEXIO_HAL_SetTimerEnable(baseAddr, SCK_TIMER, FLEXIO_TIMER_ENABLE_TRG_HIGH);
    FLEXIO_HAL_SetTimerDisable(baseAddr, SCK_TIMER, FLEXIO_TIMER_DISABLE_TIM_CMP);
    FLEXIO_HAL_SetTimerStart(baseAddr, SCK_TIMER, FLEXIO_TIMER_START_BIT_ENABLED);
    FLEXIO_HAL_SetTimerStop(baseAddr, SCK_TIMER, FLEXIO_TIMER_STOP_BIT_TIM_DIS);
    FLEXIO_HAL_SetTimerInitialOutput(baseAddr, SCK_TIMER, FLEXIO_TIMER_INITOUT_ZERO);
    FLEXIO_HAL_SetTimerPin(baseAddr, SCK_TIMER, master->sckPin, clockPolarity, FLEXIO_PIN_CONFIG_OUTPUT);
    FLEXIO_HAL_SetTimerTrigger(baseAddr, SCK_TIMER, (TX_SHIFTER << 2U) + 1U, FLEXIO_TRIGGER_POLARITY_LOW, FLEXIO_TRIGGER_SOURCE_INTERNAL);
    FLEXIO_HAL_SetTimerMode(baseAddr, SCK_TIMER, FLEXIO_TIMER_MODE_8BIT_BAUD);
    FLEXIO_HAL_SetTimerCompare(baseAddr, SCK_TIMER, divider); /* set baud rate, number of bits will be updated later */

    /* Configure SS timer */
    FLEXIO_HAL_SetTimerEnable(baseAddr, SS_TIMER, FLEXIO_TIMER_ENABLE_TIM_ENABLE);
    FLEXIO_HAL_SetTimerDisable(baseAddr, SS_TIMER, FLEXIO_TIMER_DISABLE_TIM_DISABLE);
    FLEXIO_HAL_SetTimerCompare(baseAddr, SS_TIMER, 0xFFFFU);
    FLEXIO_HAL_SetTimerPin(baseAddr, SS_TIMER, master->ssPin, FLEXIO_PIN_POLARITY_LOW, FLEXIO_PIN_CONFIG_OUTPUT);
    FLEXIO_HAL_SetTimerMode(baseAddr, SS_TIMER, FLEXIO_TIMER_MODE_16BIT);

    return FLEXIO_SPI_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_SPI_DRV_MasterWaitTransferEnd
 * Description   : waits for the end of a blocking transfer
 *
 *END**************************************************************************/
flexio_spi_status_t FLEXIO_SPI_DRV_MasterWaitTransferEnd(uint32_t instance)
{
    flexio_spi_master_state_t *master;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FLEXIO_INSTANCE_COUNT);
#endif

    master = (flexio_spi_master_state_t *)g_flexioSPIStatePtr[instance];
    switch (master->driverType)
    {
        case FLEXIO_SPI_DRIVER_TYPE_INTERRUPTS:
            /* Wait for transfer to be completed by the IRQ */
            while (master->driverIdle == false);
            break;
        case FLEXIO_SPI_DRIVER_TYPE_POLLING:
            /* Call FLEXIO_SPI_DRV_MasterGetStatus() to do the transfer */
            while (FLEXIO_SPI_DRV_MasterGetStatus(instance, NULL) == FLEXIO_SPI_STATUS_BUSY);
            break;
        case FLEXIO_SPI_DRIVER_TYPE_DMA:
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
 * Function Name : FLEXIO_SPI_DRV_MasterStopTransfer
 * Description   : Forcefully stops the current transfer
 *
 *END**************************************************************************/
static void FLEXIO_SPI_DRV_MasterStopTransfer(uint32_t instance)
{
    FLEXIO_Type *baseAddr;
    flexio_spi_master_state_t *master;

    baseAddr = g_flexioBase[instance];
    master = (flexio_spi_master_state_t *)g_flexioSPIStatePtr[instance];

    master->driverIdle = true;
    /* Call HAL_Init to reset module */
    FLEXIO_HAL_Init(baseAddr);
    /* Will re-initialize module on next use (not here because it takes a 
       long time and we may be inside an interrupt handler) */
    master->needReinit = true;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_SPI_DRV_MasterEndTransfer
 * Description   : end a transfer
 *
 *END**************************************************************************/
void FLEXIO_SPI_DRV_MasterEndTransfer(uint32_t instance)
{
    FLEXIO_Type *baseAddr;
    flexio_spi_master_state_t *master;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FLEXIO_INSTANCE_COUNT);
#endif

    baseAddr = g_flexioBase[instance];
    master = (flexio_spi_master_state_t *)g_flexioSPIStatePtr[instance];

    /* Disable transfer engine */
    switch (master->driverType)
    {
        case FLEXIO_SPI_DRIVER_TYPE_INTERRUPTS:
            /* Disable interrupts for Rx and Tx shifters */
            FLEXIO_HAL_SetShifterInterrupt(baseAddr, 
                                           (1U << TX_SHIFTER) | (1U << RX_SHIFTER), 
                                           false);
            FLEXIO_HAL_SetShifterErrorInterrupt(baseAddr, 
                                                (1U << TX_SHIFTER) | (1U << RX_SHIFTER), 
                                                false);
            break;
        case FLEXIO_SPI_DRIVER_TYPE_POLLING:
            /* Nothing to do here */
            break;
        case FLEXIO_SPI_DRIVER_TYPE_DMA:
            /* TODO: add DMA support */
            break;
        default: 
            /* Impossible type - do nothing */
            break;
    }

    master->driverIdle = true;
    master->txRemainingBytes = 0;
    master->rxRemainingBytes = 0;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_SPI_DRV_SlaveConfigure
 * Description   : configures the FLEXIO module as SPI slave
 *
 *END**************************************************************************/
static flexio_spi_status_t FLEXIO_SPI_DRV_SlaveConfigure(uint32_t instance)
{
    FLEXIO_Type *baseAddr;
    flexio_spi_slave_state_t *slave;
    flexio_pin_polarity_t clockPolarity;        /* Polarity of clock signal */
    flexio_timer_polarity_t txShifterPolarity;  /* Polarity of MISO shifter */
    flexio_timer_polarity_t rxShifterPolarity;  /* Polarity of MOSI shifter */

    baseAddr = g_flexioBase[instance];
    slave = (flexio_spi_slave_state_t *)g_flexioSPIStatePtr[instance];

    if (slave->clockPolarity == 0)
    {
        /* CPOL = 0 */
        clockPolarity = FLEXIO_PIN_POLARITY_HIGH;
    }
    else
    {
        /* CPOL = 1 */
        clockPolarity = FLEXIO_PIN_POLARITY_LOW;
    }

    if (slave->clockPhase == 0)
    {
        /* CPHA = 0 */
        txShifterPolarity = FLEXIO_TIMER_POLARITY_NEGEDGE;
        rxShifterPolarity = FLEXIO_TIMER_POLARITY_POSEDGE;
    }
    else
    {
        /* CPHA = 1 */
        txShifterPolarity = FLEXIO_TIMER_POLARITY_POSEDGE;
        rxShifterPolarity = FLEXIO_TIMER_POLARITY_NEGEDGE;
    }

    /* Configure Slave Tx shifter (MISO) */
    FLEXIO_HAL_SetShifterTimer(baseAddr, TX_SHIFTER, SCK_TIMER, txShifterPolarity);
    FLEXIO_HAL_SetShifterPin(baseAddr, TX_SHIFTER, slave->misoPin, FLEXIO_PIN_POLARITY_HIGH, FLEXIO_PIN_CONFIG_OUTPUT);
    FLEXIO_HAL_SetShifterMode(baseAddr, TX_SHIFTER, FLEXIO_SHIFTER_MODE_TRANSMIT);

    if (slave->clockPhase != 0U)
    {
        /* CPHA = 1 */
        FLEXIO_HAL_SetShifterStartBit(baseAddr, TX_SHIFTER, FLEXIO_SHIFTER_START_BIT_DISABLED_SH);
    }

    /* Configure Slave Rx shifter (MOSI) */
    FLEXIO_HAL_SetShifterTimer(baseAddr, RX_SHIFTER, SCK_TIMER, rxShifterPolarity);
    FLEXIO_HAL_SetShifterPin(baseAddr, RX_SHIFTER, slave->mosiPin, FLEXIO_PIN_POLARITY_HIGH, FLEXIO_PIN_CONFIG_DISABLED);
    FLEXIO_HAL_SetShifterMode(baseAddr, RX_SHIFTER, FLEXIO_SHIFTER_MODE_RECEIVE);

    /* Configure sck timer  */
    FLEXIO_HAL_SetTimerInitialOutput(baseAddr, SCK_TIMER, FLEXIO_TIMER_INITOUT_ZERO);
    FLEXIO_HAL_SetTimerDisable(baseAddr, SCK_TIMER, FLEXIO_TIMER_DISABLE_TRG);
    FLEXIO_HAL_SetTimerEnable(baseAddr, SCK_TIMER, FLEXIO_TIMER_ENABLE_TRG_POSEDGE);
    FLEXIO_HAL_SetTimerDecrement(baseAddr, SCK_TIMER, FLEXIO_TIMER_DECREMENT_PIN_SHIFT_PIN);
    FLEXIO_HAL_SetTimerTrigger(baseAddr, SCK_TIMER, slave->ssPin << 1U, FLEXIO_TRIGGER_POLARITY_LOW, FLEXIO_TRIGGER_SOURCE_INTERNAL);
    FLEXIO_HAL_SetTimerPin(baseAddr, SCK_TIMER, slave->sckPin, clockPolarity, FLEXIO_PIN_CONFIG_DISABLED);
    FLEXIO_HAL_SetTimerMode(baseAddr, SCK_TIMER, FLEXIO_TIMER_MODE_16BIT);
    FLEXIO_HAL_SetTimerCompare(baseAddr, SCK_TIMER, 0x003FU);

    return FLEXIO_SPI_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_SPI_DRV_ReadData
 * Description   : reads data received by the module
 *
 *END**************************************************************************/
static void FLEXIO_SPI_DRV_ReadData(uint32_t instance)
{
    FLEXIO_Type *baseAddr;
    flexio_spi_master_state_t *master;
    uint32_t data;

    baseAddr = g_flexioBase[instance];
    master = (flexio_spi_master_state_t *)g_flexioSPIStatePtr[instance];

    /* Read data from shifter buffer */
    if (master->bit_order == FLEXIO_SPI_TRANSFER_LSB_FIRST)
    {
        /* For data size < 4 bytes our data is in the upper part of the buffer and must be shifted */
        data = FLEXIO_HAL_ReadShifterBuffer(baseAddr, RX_SHIFTER, FLEXIO_SHIFTER_RW_MODE_NORMAL);
        data >>= (32U - 8U * (uint32_t)(master->transfer_size));
    }
    else
    {
        data = FLEXIO_HAL_ReadShifterBuffer(baseAddr, RX_SHIFTER, FLEXIO_SHIFTER_RW_MODE_BIT_SWAP);
    }

    if ((master->rxRemainingBytes > 0) && (master->rxData != NULL))
    {
        switch (master->transfer_size)
        {
            case FLEXIO_SPI_TRANSFER_1BYTE:
                *(uint8_t *)master->rxData = (uint8_t)data;
                break;
            case FLEXIO_SPI_TRANSFER_2BYTE:
                *(uint16_t *)master->rxData = (uint16_t)data;
                break;
            case FLEXIO_SPI_TRANSFER_4BYTE:
                *(uint32_t *)master->rxData = (uint32_t)data;
                break;
            default:
                /* Not possible */
                break;
        }
        /* Update rx buffer pointer and remaining bytes count */
        master->rxData += (uint32_t)(master->transfer_size);
        master->rxRemainingBytes -= (uint32_t)(master->transfer_size);
    }
    else
    {
        /* No data to receive, just ignore the read data */
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_SPI_DRV_WriteData
 * Description   : writes data to be transmitted by the module
 *
 *END**************************************************************************/
static void FLEXIO_SPI_DRV_WriteData(uint32_t instance)
{
    FLEXIO_Type *baseAddr;
    flexio_spi_master_state_t *master;
    uint32_t data = FLEXIO_SPI_DUMMYDATA;

    baseAddr = g_flexioBase[instance];
    master = (flexio_spi_master_state_t *)g_flexioSPIStatePtr[instance];

    if (master->txRemainingBytes == 0)
    {
        /* Done transmitting */
        return;
    }

    if ((master->txRemainingBytes > 0) && (master->txData != NULL))
    {
        /* Read data from user buffer */
        switch (master->transfer_size)
        {
            case FLEXIO_SPI_TRANSFER_1BYTE:
                data = (uint32_t)(*(uint8_t *)master->txData);
                break;
            case FLEXIO_SPI_TRANSFER_2BYTE:
                data = (uint32_t)(*(uint16_t *)master->txData);
                break;
            case FLEXIO_SPI_TRANSFER_4BYTE:
                data = (uint32_t)(*(uint32_t *)master->txData);
                break;
            default:
                /* Not possible */
                break;
        }
        /* Update tx buffer pointer and remaining bytes count */
        master->txData += (uint32_t)(master->transfer_size);
        master->txRemainingBytes -= (uint32_t)(master->transfer_size);
        /* Write data to shifter buffer */
        if (master->bit_order == FLEXIO_SPI_TRANSFER_LSB_FIRST)
        {
            FLEXIO_HAL_WriteShifterBuffer(baseAddr, TX_SHIFTER, data, FLEXIO_SHIFTER_RW_MODE_NORMAL);
        }
        else
        {
            /* Shift data before bit-swapping it to get the relevant bits in the lower part of the shifter */
            data <<= 32U - 8U * (uint32_t)(master->transfer_size);
            FLEXIO_HAL_WriteShifterBuffer(baseAddr, TX_SHIFTER, data, FLEXIO_SHIFTER_RW_MODE_BIT_SWAP);
        }
    }
    else
    {
        /* Nothing to send, write dummy data in buffer */
        FLEXIO_HAL_WriteShifterBuffer(baseAddr, TX_SHIFTER, FLEXIO_SPI_DUMMYDATA, FLEXIO_SHIFTER_RW_MODE_NORMAL);
    }
}



/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_SPI_DRV_MasterCheckStatus
 * Description   : Check status of SPI master transfer. This function can be 
 *                 called either in an interrupt routine or directly in polling 
 *                 mode to advance the SPI transfer.
 *
 *END**************************************************************************/
void FLEXIO_SPI_DRV_MasterCheckStatus(uint32_t instance)
{
    FLEXIO_Type *baseAddr;
    flexio_spi_master_state_t *master;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FLEXIO_INSTANCE_COUNT);
#endif

    baseAddr = g_flexioBase[instance];
    master = (flexio_spi_master_state_t *)g_flexioSPIStatePtr[instance];

    /* Check for errors */
    if (FLEXIO_HAL_GetShifterErrorStatus(baseAddr, TX_SHIFTER))
    {
        master->status = FLEXIO_SPI_STATUS_TX_UNDERFLOW;
        FLEXIO_HAL_ClearShifterErrorStatus(baseAddr, TX_SHIFTER);
        /* Force the ongoing transfer to stop */
        FLEXIO_SPI_DRV_MasterStopTransfer(instance);
        /* Stop processing events */
        return;
    }
    if (FLEXIO_HAL_GetShifterErrorStatus(baseAddr, RX_SHIFTER))
    {
        master->status = FLEXIO_SPI_STATUS_RX_OVERFLOW;
        FLEXIO_HAL_ClearShifterErrorStatus(baseAddr, RX_SHIFTER);
        /* Force the ongoing transfer to stop */
        FLEXIO_SPI_DRV_MasterStopTransfer(instance);
        /* Stop processing events */
        return;
    }
    /* Check if data was received */
    if (FLEXIO_HAL_GetShifterStatus(baseAddr, RX_SHIFTER))
    {
        FLEXIO_SPI_DRV_ReadData(instance);
    }
    /* Check if transmitter needs more data */
    if (FLEXIO_HAL_GetShifterStatus(baseAddr, TX_SHIFTER))
    {
        FLEXIO_SPI_DRV_WriteData(instance);
        if (master->txRemainingBytes == 0)
        {
            /* No more data to transmit, disable tx interrupts */
            FLEXIO_HAL_SetShifterInterrupt(baseAddr, (1U << TX_SHIFTER), false);
            FLEXIO_HAL_SetShifterErrorInterrupt(baseAddr, (1U << TX_SHIFTER), false);
        }
    }

    /* Check there is any data left */
    if ((master->txRemainingBytes == 0) && (master->rxRemainingBytes == 0))
    {
        /* End transfer */
        FLEXIO_SPI_DRV_MasterEndTransfer(instance);
        /* Record success if there was no error */
        if (master->status == FLEXIO_SPI_STATUS_BUSY)
        {
            master->status = FLEXIO_SPI_STATUS_SUCCESS;
        }
    }
}

/*! @endcond */

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_SPI_DRV_MasterInit
 * Description   : Initialize the FLEXIO_SPI master mode driver
 *
 *END**************************************************************************/
flexio_spi_status_t FLEXIO_SPI_DRV_MasterInit(uint32_t instance,
                                              const flexio_spi_master_user_config_t * userConfigPtr,
                                              flexio_spi_master_state_t * master)
{
    FLEXIO_Type *baseAddr;
    interrupt_manager_error_code_t intErr;
    flexio_spi_status_t retCode;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(master != NULL);
    DEV_ASSERT(instance < FLEXIO_INSTANCE_COUNT);
#endif

    if (g_flexioSPIStatePtr[instance])
    {
        return FLEXIO_SPI_STATUS_FAIL;
    }

    baseAddr = g_flexioBase[instance];
    g_flexioSPIStatePtr[instance] = master;

    /* Initialize driver context structure */
    master->driverType = userConfigPtr->driverType;
    master->clockPolarity = userConfigPtr->clockPolarity;
    master->clockPhase = userConfigPtr->clockPhase;
    master->baudRate = userConfigPtr->baudRate;
    master->mosiPin = userConfigPtr->mosiPin;
    master->misoPin = userConfigPtr->misoPin;
    master->sckPin = userConfigPtr->sckPin;
    master->ssPin = userConfigPtr->ssPin;
    master->driverIdle = true;
    master->needReinit = false;
    master->master = true;

    /* Initialize HAL */
    FLEXIO_HAL_Init(baseAddr);

    /* Configure device for SPI mode */
    retCode = FLEXIO_SPI_DRV_MasterConfigure(instance);
    if (retCode != FLEXIO_SPI_STATUS_SUCCESS)
    {
        g_flexioSPIStatePtr[instance] = NULL;
        return retCode;
    }

    /* Set up transfer engine */
    switch (userConfigPtr->driverType)
    {
        case FLEXIO_SPI_DRIVER_TYPE_INTERRUPTS:
            /* Install irq handler */
            intErr = INT_SYS_InstallHandler(g_flexioIrqId[instance], g_flexioIsr[instance], NULL);
            if (intErr == INTERRUPT_MANAGER_ERROR)
            {
                return FLEXIO_SPI_STATUS_FAIL;
            }
            /* Enable FLEXIO interrupts in the interrupt manager */
            INT_SYS_EnableIRQ(g_flexioIrqId[instance]);
            break;
        case FLEXIO_SPI_DRIVER_TYPE_POLLING:
            /* Nothing to do here, FLEXIO_SPI_DRV_MasterGetStatus() will handle the transfer */
            break;
        case FLEXIO_SPI_DRIVER_TYPE_DMA:
            /* TODO: add DMA support */
            break;
        default: 
            /* Impossible type - do nothing */
            break;
    }

    /* Enable module */
    FLEXIO_HAL_SetEnable(baseAddr, true);

    return FLEXIO_SPI_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_SPI_DRV_MasterDeinit
 * Description   : De-initialize the FLEXIO_SPI master mode driver
 *
 *END**************************************************************************/

flexio_spi_status_t FLEXIO_SPI_DRV_MasterDeinit(uint32_t instance)
{
    flexio_spi_master_state_t *master;
    FLEXIO_Type *baseAddr;
    interrupt_manager_error_code_t intErr;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FLEXIO_INSTANCE_COUNT);
#endif

    baseAddr = g_flexioBase[instance];
    master = (flexio_spi_master_state_t *)g_flexioSPIStatePtr[instance];

    /* Disable module */
    FLEXIO_HAL_SetEnable(baseAddr, false);

    /* Disable transfer engine */
    if (master != NULL)
    {
        switch (master->driverType)
        {
            case FLEXIO_SPI_DRIVER_TYPE_INTERRUPTS:
                /* Disable FLEXIO interrupt */
                INT_SYS_DisableIRQ(g_flexioIrqId[instance]);
                /* Restore default handler. */
                intErr = INT_SYS_InstallHandler(g_flexioIrqId[instance], DefaultISR, NULL);
                if (intErr == INTERRUPT_MANAGER_ERROR)
                {
                    return FLEXIO_SPI_STATUS_FAIL;
                }
                break;
            case FLEXIO_SPI_DRIVER_TYPE_POLLING:
                /* Nothing to do */
                break;
            case FLEXIO_SPI_DRIVER_TYPE_DMA:
                /* TODO: add DMA support */
                break;
            default: 
                /* Impossible type - do nothing */
                break;
        }
    }

    g_flexioSPIStatePtr[instance] = NULL;

    return FLEXIO_SPI_STATUS_SUCCESS;
}




/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_SPI_DRV_MasterSetBaudRate
 * Description   : Set the baud rate for any subsequent SPI communication
 *
 *END**************************************************************************/
flexio_spi_status_t FLEXIO_SPI_DRV_MasterSetBaudRate(uint32_t instance, uint32_t baudRate)
{
    FLEXIO_Type *baseAddr;
    flexio_spi_master_state_t *master;
    uint16_t divider;
    uint16_t timerCmp;
    flexio_spi_status_t retCode;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FLEXIO_INSTANCE_COUNT);
#endif

    baseAddr = g_flexioBase[instance];
    master = (flexio_spi_master_state_t *)g_flexioSPIStatePtr[instance];

    /* Check if driver is busy */
    if (!master->driverIdle)
    {
        return FLEXIO_SPI_STATUS_BUSY;
    }

    master->baudRate = baudRate;
    /* Compute divider */
    retCode = FLEXIO_SPI_DRV_MasterComputeBaudRateDivider(instance, baudRate, &divider);
    if (retCode != FLEXIO_SPI_STATUS_SUCCESS)
    {
        return retCode;
    }

    /* Configure timer divider in the lower 8 bits of TIMCMP[CMP] */
    timerCmp = FLEXIO_HAL_GetTimerCompare(baseAddr, SCK_TIMER);
    timerCmp = (timerCmp & 0xFF00U) | divider;
    FLEXIO_HAL_SetTimerCompare(baseAddr, SCK_TIMER, timerCmp);

    return FLEXIO_SPI_STATUS_SUCCESS;
}



/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_SPI_DRV_MasterGetBaudRate
 * Description   : Get the currently configured baud rate
 *
 *END**************************************************************************/
flexio_spi_status_t FLEXIO_SPI_DRV_MasterGetBaudRate(uint32_t instance, uint32_t *baudRate)
{
    FLEXIO_Type *baseAddr;
    uint32_t divider;
    uint16_t timerCmp;
    uint32_t inputClock;
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
        return FLEXIO_SPI_STATUS_FAIL;
    }

    /* Get the currently configured divider */
    timerCmp = FLEXIO_HAL_GetTimerCompare(baseAddr, SCK_TIMER);
    divider = (uint32_t)(timerCmp & 0x00FFU);

    /* Compute baud rate: input_clock / (2 * (divider + 1)). Round to nearest integer */
    *baudRate = (inputClock + divider + 1U) / (2U * (divider + 1U));
    
    return FLEXIO_SPI_STATUS_SUCCESS;
}




/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_SPI_DRV_MasterTransfer
 * Description   : Perform an SPI master transaction
 *
 *END**************************************************************************/
flexio_spi_status_t FLEXIO_SPI_DRV_MasterTransfer(uint32_t instance,
                                                  const flexio_spi_transfer_t *transfer)
{
    FLEXIO_Type *baseAddr;
    flexio_spi_master_state_t *master;
    uint16_t timerCmp;
    uint16_t shiftCnt;
    flexio_spi_status_t retCode;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FLEXIO_INSTANCE_COUNT);
    DEV_ASSERT((transfer->transfer_size == FLEXIO_SPI_TRANSFER_1BYTE) ||
               (transfer->transfer_size == FLEXIO_SPI_TRANSFER_2BYTE) ||
               (transfer->transfer_size == FLEXIO_SPI_TRANSFER_4BYTE));
    DEV_ASSERT(transfer->dataSize % (uint32_t)(transfer->transfer_size) == 0);
#endif

    baseAddr = g_flexioBase[instance];
    master = (flexio_spi_master_state_t *)g_flexioSPIStatePtr[instance];

    /* Check if driver is busy */
    if (!master->driverIdle)
    {
        return FLEXIO_SPI_STATUS_BUSY;
    }

    if (master->needReinit)
    {
        /* Re-initialize module after a forced stop */
        if (master->master)
        {
            retCode = FLEXIO_SPI_DRV_MasterConfigure(instance);
        }
        else
        {
            retCode = FLEXIO_SPI_DRV_SlaveConfigure(instance);
        }
        if (retCode != FLEXIO_SPI_STATUS_SUCCESS)
        {
            return retCode;
        }
        FLEXIO_HAL_SetEnable(baseAddr, true);
    }

    /* Copy transfer data */
    master->txData = transfer->txData;
    master->rxData = transfer->rxData;
    master->txRemainingBytes = transfer->dataSize;
    master->rxRemainingBytes = transfer->dataSize;
    master->bit_order = transfer->bit_order;
    master->transfer_size = transfer->transfer_size;
    master->driverIdle = false;
    master->status = FLEXIO_SPI_STATUS_BUSY;

    if (master->master)
    {
        /* Configure device for requested number of bytes, keeping the existing baud rate */
        shiftCnt = ((uint16_t)(transfer->transfer_size) * 8U * 2U - 1U);
        timerCmp = FLEXIO_HAL_GetTimerCompare(baseAddr, SCK_TIMER);
        timerCmp = (timerCmp & 0x00FFU) | shiftCnt << 8U;
        FLEXIO_HAL_SetTimerCompare(baseAddr, SCK_TIMER, timerCmp);
    }

    /* Enable transfer engine */
    switch (master->driverType)
    {
        case FLEXIO_SPI_DRIVER_TYPE_INTERRUPTS:
            /* Enable interrupts for Rx and Tx shifters */
            FLEXIO_HAL_SetShifterInterrupt(baseAddr, 
                                           (1U << TX_SHIFTER) | (1U << RX_SHIFTER), 
                                           true);
            FLEXIO_HAL_SetShifterErrorInterrupt(baseAddr, 
                                                (1U << TX_SHIFTER) | (1U << RX_SHIFTER), 
                                                true);
            break;
        case FLEXIO_SPI_DRIVER_TYPE_POLLING:
            /* Call FLEXIO_SPI_DRV_MasterCheckStatus once to send the first byte */
            FLEXIO_SPI_DRV_MasterCheckStatus(instance);
            break;
        case FLEXIO_SPI_DRIVER_TYPE_DMA:
            /* TODO: add DMA support */
            break;
        default: 
            /* Impossible type - do nothing */
            break;
    }

    if (transfer->transfer_type == FLEXIO_SPI_TRANSFER_BLOCKING)
    {
        /* Wait for transfer to end */
        return FLEXIO_SPI_DRV_MasterWaitTransferEnd(instance);
    }

    return FLEXIO_SPI_STATUS_SUCCESS;
}



                                                             
/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_SPI_DRV_MasterTransferAbort
 * Description   : Aborts a non-blocking SPI master transaction
 *
 *END**************************************************************************/
flexio_spi_status_t FLEXIO_SPI_DRV_MasterTransferAbort(uint32_t instance)
{
    flexio_spi_master_state_t *master;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FLEXIO_INSTANCE_COUNT);
#endif

    master = (flexio_spi_master_state_t *)g_flexioSPIStatePtr[instance];

    /* Check if driver is busy */
    if (master->driverIdle)
    {
        return FLEXIO_SPI_STATUS_FAIL;
    }

    master->status = FLEXIO_SPI_STATUS_ABORTED;
    /* Force the ongoing transfer to stop */
    FLEXIO_SPI_DRV_MasterStopTransfer(instance);

    return FLEXIO_SPI_STATUS_SUCCESS;
}




/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_SPI_DRV_MasterGetStatus
 * Description   : Get the status of the current non-blocking SPI master transaction
 *
 *END**************************************************************************/
flexio_spi_status_t FLEXIO_SPI_DRV_MasterGetStatus(uint32_t instance, uint32_t *bytesRemaining)
{
    flexio_spi_master_state_t *master;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FLEXIO_INSTANCE_COUNT);
#endif

    master = (flexio_spi_master_state_t *)g_flexioSPIStatePtr[instance];

    if ((!master->driverIdle) && (master->driverType == FLEXIO_SPI_DRIVER_TYPE_POLLING))
    {
        /* In polling mode advance the SPI transfer here */
        FLEXIO_SPI_DRV_MasterCheckStatus(instance);
    }

    if (bytesRemaining != NULL)
    {
        *bytesRemaining = master->txRemainingBytes;
    }

    return master->status;
}



/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXIO_SPI_DRV_SlaveInit
 * Description   : Initialize the FLEXIO_SPI slave mode driver
 *
 *END**************************************************************************/
flexio_spi_status_t FLEXIO_SPI_DRV_SlaveInit(uint32_t instance,
                                             const flexio_spi_slave_user_config_t * userConfigPtr,
                                             flexio_spi_slave_state_t * slave)
{
    FLEXIO_Type *baseAddr;
    interrupt_manager_error_code_t intErr;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(slave != NULL);
    DEV_ASSERT(instance < FLEXIO_INSTANCE_COUNT);
#endif

    if (g_flexioSPIStatePtr[instance])
    {
        return FLEXIO_SPI_STATUS_FAIL;
    }

    baseAddr = g_flexioBase[instance];
    g_flexioSPIStatePtr[instance] = slave;

    /* Initialize driver context structure */
    slave->driverType = userConfigPtr->driverType;
    slave->clockPolarity = userConfigPtr->clockPolarity;
    slave->clockPhase = userConfigPtr->clockPhase;
    slave->mosiPin = userConfigPtr->mosiPin;
    slave->misoPin = userConfigPtr->misoPin;
    slave->sckPin = userConfigPtr->sckPin;
    slave->ssPin = userConfigPtr->ssPin;
    slave->driverIdle = true;
    slave->needReinit = false;
    slave->master = false;

    /* Initialize HAL */
    FLEXIO_HAL_Init(baseAddr);

    /* Configure device for SPI mode */
    (void)FLEXIO_SPI_DRV_SlaveConfigure(instance);

    /* Set up transfer engine */
    switch (userConfigPtr->driverType)
    {
        case FLEXIO_SPI_DRIVER_TYPE_INTERRUPTS:
            /* Install irq handler */
            intErr = INT_SYS_InstallHandler(g_flexioIrqId[instance], g_flexioIsr[instance], NULL);
            if (intErr == INTERRUPT_MANAGER_ERROR)
            {
                return FLEXIO_SPI_STATUS_FAIL;
            }
            /* Enable FLEXIO interrupts in the interrupt manager */
            INT_SYS_EnableIRQ(g_flexioIrqId[instance]);
            break;
        case FLEXIO_SPI_DRIVER_TYPE_POLLING:
            /* Nothing to do here, FLEXIO_SPI_DRV_MasterGetStatus() will handle the transfer */
            break;
        case FLEXIO_SPI_DRIVER_TYPE_DMA:
            /* TODO: add DMA support */
            break;
        default: 
            /* Impossible type - do nothing */
            break;
    }

    /* Enable module */
    FLEXIO_HAL_SetEnable(baseAddr, true);

    return FLEXIO_SPI_STATUS_SUCCESS;
}



/*******************************************************************************
 * EOF
 ******************************************************************************/
