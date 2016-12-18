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

#include "fsl_lpi2c_driver.h"
#include "fsl_interrupt_manager.h"
#include "fsl_lpi2c_hal.h"
#include "fsl_clock_manager.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @cond DRIVER_INTERNAL_USE_ONLY */

    /* Constraints used for baud rate computation */
#define CLKHI_MIN_VALUE 1U
#define CLKLO_MIN_VALUE 3U
#define CLKHI_MAX_VALUE (((1U << LPI2C_MCCR0_CLKHI_WIDTH) - 1U) >> 1U)
#define CLKLO_MAX_VALUE (CLKHI_MAX_VALUE << 1U)
#define DATAVD_MIN_VALUE 1U
#define SETHOLD_MIN_VALUE 2U

/* Table of base addresses for LPI2C instances. */
LPI2C_Type * const g_lpi2cBase[LPI2C_INSTANCE_COUNT] = LPI2C_BASE_PTRS;

/* Pointer to runtime state structure.*/
void * g_lpi2cStatePtr[LPI2C_INSTANCE_COUNT] = {NULL};

/* Table for lpi2c IRQ numbers */
const IRQn_Type g_lpi2cIrqId[LPI2C_INSTANCE_COUNT] = {LPI2C0_IRQn, LPI2C1_IRQn};

/* PCC clock sources, for getting the input clock frequency */
const clock_names_t g_lpi2cClock[LPI2C_INSTANCE_COUNT] = {PCC_LPI2C0_CLOCK, PCC_LPI2C1_CLOCK};


/*******************************************************************************
 * Private Functions
 ******************************************************************************/


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterCmdQueueEmpty
 * Description   : checks if there are any commands in the master software queue
 *
 *END**************************************************************************/
static inline bool LPI2C_DRV_MasterCmdQueueEmpty(lpi2c_master_state_t * master)
{
    return (master->cmdQueue.writeIdx == master->cmdQueue.readIdx);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterResetQueue
 * Description   : resets the master software queue
 *
 *END**************************************************************************/
static inline void LPI2C_DRV_MasterResetQueue(lpi2c_master_state_t * master)
{
    master->cmdQueue.readIdx = 0U;
    master->cmdQueue.writeIdx = 0U;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterQueueCmd
 * Description   : queues a command in the hardware FIFO or in the master software queue
 *
 *END**************************************************************************/
static inline void LPI2C_DRV_MasterQueueCmd(LPI2C_Type *baseAddr,
                                            lpi2c_master_state_t * master,
                                            lpi2c_master_command_t cmd,
                                            uint8_t data)
{
    /* Check if there is room in the hardware FIFO */
    if (LPI2C_HAL_MasterGetTxFIFOCount(baseAddr) < LPI2C_HAL_GetMasterTxFIFOSize(baseAddr))
    {
        LPI2C_HAL_MasterTransmitCmd(baseAddr, cmd, data);
    }
    else
    {
        /* Hardware FIFO full, use software FIFO */
#ifdef DEV_ERROR_DETECT
        DEV_ASSERT(master->cmdQueue.writeIdx < LPI2C_MASTER_CMD_QUEUE_SIZE);
#endif
        master->cmdQueue.cmd[master->cmdQueue.writeIdx] = cmd;
        master->cmdQueue.data[master->cmdQueue.writeIdx] = data;
        master->cmdQueue.writeIdx++;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterSendQueuedCmd
 * Description   : transfers commands from the master software queue to the hardware FIFO
 *
 *END**************************************************************************/
static inline void LPI2C_DRV_MasterSendQueuedCmd(LPI2C_Type *baseAddr, lpi2c_master_state_t * master)
{
    uint16_t txFifoSize = LPI2C_HAL_GetMasterTxFIFOSize(baseAddr);

    while ((!LPI2C_DRV_MasterCmdQueueEmpty(master)) && (LPI2C_HAL_MasterGetTxFIFOCount(baseAddr) < txFifoSize))
    {
        LPI2C_HAL_MasterTransmitCmd(baseAddr,
                                    master->cmdQueue.cmd[master->cmdQueue.readIdx],
                                    master->cmdQueue.data[master->cmdQueue.readIdx]);
        master->cmdQueue.readIdx++;
    }
    if (LPI2C_DRV_MasterCmdQueueEmpty(master))
    {
        /* Reset queue */
        LPI2C_DRV_MasterResetQueue(master);
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterSendAddress
 * Description   : send start event and slave address
 *                 parameter receive specifies the direction of the transfer
 *
 *END**************************************************************************/
static void LPI2C_DRV_MasterSendAddress(LPI2C_Type *baseAddr,
                                        lpi2c_master_state_t * master,
                                        bool receive)
{
    uint8_t addrByte;
    lpi2c_master_command_t startCommand;

    if ((master->operatingMode == LPI2C_HIGHSPEED_MODE) && (master->highSpeedInProgress == false))
    {
        /* Initiating High-speed mode - send master code first */
        LPI2C_DRV_MasterQueueCmd(baseAddr, master, LPI2C_MASTER_COMMAND_START_NACK, master->masterCode);
        master->highSpeedInProgress = true;
    }
    if (master->highSpeedInProgress == true)
    {
        /* Use high-speed settings after start event in High Speed mode */
        startCommand = LPI2C_MASTER_COMMAND_START_HS;
    }
    else
    {
        /* Normal START command */
        startCommand = LPI2C_MASTER_COMMAND_START;
    }

    if (master->is10bitAddr)
    {
        /* 10-bit addressing */
        /* First address byte: 1111 0XXD, where XX are bits 10 and 9 of address, and D = 0(transmit) */
        addrByte = (uint8_t)(0xF0U + ((master->slaveAddress >> 7U) & 0x6U) + (uint8_t)0U);
        LPI2C_DRV_MasterQueueCmd(baseAddr, master, startCommand, addrByte);
        /* Second address byte: Remaining 8 bits of address */
        addrByte = (uint8_t)(master->slaveAddress & 0xFFU);
        LPI2C_DRV_MasterQueueCmd(baseAddr, master, LPI2C_MASTER_COMMAND_TRANSMIT, addrByte);
        if (receive == true)
        {
            /* Receiving from 10-bit slave - must send repeated start and resend first address byte */
            /* First address byte: 1111 0XXD, where XX are bits 10 and 9 of address, and D = 1 (receive) */
            addrByte = (uint8_t)(0xF0U + ((master->slaveAddress >> 7U) & 0x6U) + (uint8_t)1U);
            LPI2C_DRV_MasterQueueCmd(baseAddr, master, startCommand, addrByte);
        }
    }
    else
    {
        /* 7-bit addressing */
        /* Address byte: slave 7-bit address + D = 0(transmit) or 1 (receive) */
        addrByte = (uint8_t)((master->slaveAddress << 1U) + (uint8_t)receive);
        LPI2C_DRV_MasterQueueCmd(baseAddr, master, startCommand, addrByte);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterQueueData
 * Description   : queues transmit data in the LPI2C tx fifo until it is full
 *
 *END**************************************************************************/
static void LPI2C_DRV_MasterQueueData(LPI2C_Type *baseAddr,
                                      lpi2c_master_state_t * master)
{
    uint16_t txFifoSize = LPI2C_HAL_GetMasterTxFIFOSize(baseAddr);

    /* Don't queue any data if there are commands in the software queue */
    if (LPI2C_DRV_MasterCmdQueueEmpty(master))
    {
        while ((master->txSize > 0U) && (LPI2C_HAL_MasterGetTxFIFOCount(baseAddr) < txFifoSize))
        {
            LPI2C_HAL_MasterTransmitCmd(baseAddr, LPI2C_MASTER_COMMAND_TRANSMIT, master->txBuff[0U]);
            master->txBuff++;
            master->txSize--;
        }
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterEndTransfer
 * Description   : ends current transmission or reception
 *
 *END**************************************************************************/
static void LPI2C_DRV_MasterEndTransfer(LPI2C_Type *baseAddr,
                                        lpi2c_master_state_t * master,
                                        bool sendStop,
                                        bool resetFIFO)
{
    /* Disable all events */
    LPI2C_HAL_MasterSetInt(baseAddr, LPI2C_HAL_MASTER_FIFO_ERROR_INT |
                                     LPI2C_HAL_MASTER_ARBITRATION_LOST_INT |
                                     LPI2C_HAL_MASTER_NACK_DETECT_INT |
                                     LPI2C_HAL_MASTER_TRANSMIT_DATA_INT |
                                     LPI2C_HAL_MASTER_RECEIVE_DATA_INT,
                           false);
    if (resetFIFO == true)
    {
        /* Reset FIFOs if requested */
        LPI2C_HAL_MasterTxFIFOResetCmd(baseAddr);
        LPI2C_HAL_MasterRxFIFOResetCmd(baseAddr);
    }
    /* Queue STOP command if requested */
    if (sendStop == true)
    {
        LPI2C_HAL_MasterTransmitCmd(baseAddr, LPI2C_MASTER_COMMAND_STOP, 0U);
        master->highSpeedInProgress = false; /* High-speed transfers end at STOP condition */
    }
    master->txBuff = NULL;
    master->rxBuff = NULL;
    master->i2cIdle = true;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterSend
 * Description   : perform a send transaction on the I2C bus
 *
 *END**************************************************************************/
static lpi2c_status_t LPI2C_DRV_MasterSend(uint32_t instance,
                                           const uint8_t * txBuff,
                                           uint32_t txSize,
                                           bool sendStop,
                                           bool blocking)
{
    LPI2C_Type *baseAddr;
    lpi2c_master_state_t * master;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPI2C_INSTANCE_COUNT);
#endif

    baseAddr = g_lpi2cBase[instance];
    master = (lpi2c_master_state_t *)g_lpi2cStatePtr[instance];

    /* Check if driver is busy */
    if (!master->i2cIdle)
    {
        return LPI2C_STATUS_BUSY;
    }

    /* Copy parameters to driver state structure */
    master->txBuff = txBuff;
    master->txSize = txSize;
    master->isBlocking = blocking;
    master->sendStop = sendStop;
    master->i2cIdle = false;
    master->status = LPI2C_STATUS_BUSY;

    /* Initiate communication */
    LPI2C_DRV_MasterSendAddress(baseAddr, master, false);

    /* Queue data bytes to fill tx fifo */
    LPI2C_DRV_MasterQueueData(baseAddr, master);

    /* Set tx FIFO watermark */
    LPI2C_HAL_MasterSetTxFIFOWatermark(baseAddr, 0U);

    /* Enable relevant events */
    if (master->operatingMode == LPI2C_ULTRAFAST_MODE)
    {
        /* Do not enable NACK event reporting in ultra-fast mode */
        LPI2C_HAL_MasterSetInt(baseAddr, LPI2C_HAL_MASTER_FIFO_ERROR_INT |
                                         LPI2C_HAL_MASTER_ARBITRATION_LOST_INT |
                                         LPI2C_HAL_MASTER_TRANSMIT_DATA_INT,
                               true);
    }
    else
    {
        LPI2C_HAL_MasterSetInt(baseAddr, LPI2C_HAL_MASTER_FIFO_ERROR_INT |
                                         LPI2C_HAL_MASTER_ARBITRATION_LOST_INT |
                                         LPI2C_HAL_MASTER_NACK_DETECT_INT |
                                         LPI2C_HAL_MASTER_TRANSMIT_DATA_INT,
                               true);
    }

    if (blocking)
    {
        while (master->i2cIdle == false);
        return master->status;
    }
    else
    {
        return LPI2C_STATUS_SUCCESS;
    }

}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterReceive
 * Description   : perform a receive transaction on the I2C bus
 *
 *END**************************************************************************/
static lpi2c_status_t LPI2C_DRV_MasterReceive(uint32_t instance,
                                              uint8_t * rxBuff,
                                              uint32_t rxSize,
                                              bool sendStop,
                                              bool blocking)
{
    LPI2C_Type *baseAddr;
    lpi2c_master_state_t * master;
    uint8_t rxBytes;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(rxBuff != NULL);
    DEV_ASSERT(rxSize > 0U);
    DEV_ASSERT(instance < LPI2C_INSTANCE_COUNT);
#endif

    baseAddr = g_lpi2cBase[instance];
    master = (lpi2c_master_state_t *)g_lpi2cStatePtr[instance];

    /* Check if driver is busy */
    if (!master->i2cIdle)
    {
        return LPI2C_STATUS_BUSY;
    }
    if (master->operatingMode == LPI2C_ULTRAFAST_MODE)
    {
        /* No reception possible in ultra-fast mode */
        return LPI2C_STATUS_FAIL;
    }

    /* Copy parameters to driver state structure */
    master->rxBuff = rxBuff;
    master->rxSize = rxSize;
    master->isBlocking = blocking;
    master->sendStop = sendStop;
    master->i2cIdle = false;
    master->status = LPI2C_STATUS_BUSY;

    /* Initiate communication */
    LPI2C_DRV_MasterSendAddress(baseAddr, master, true);

    /* Queue receive command for rxSize bytes */
    LPI2C_DRV_MasterQueueCmd(baseAddr, master, LPI2C_MASTER_COMMAND_RECEIVE, rxSize - 1U);

    /* Set rx FIFO watermark */
    rxBytes = LPI2C_HAL_GetMasterRxFIFOSize(baseAddr);
    if (rxBytes > rxSize)
    {
        rxBytes = rxSize;
    }
    LPI2C_HAL_MasterSetRxFIFOWatermark(baseAddr, rxBytes - 1U);

    /* Enable relevant events */
    if (!LPI2C_DRV_MasterCmdQueueEmpty(master))
    {
        /* Enable tx event too if there are commands in the software FIFO */
        LPI2C_HAL_MasterSetInt(baseAddr, LPI2C_HAL_MASTER_FIFO_ERROR_INT |
                                         LPI2C_HAL_MASTER_ARBITRATION_LOST_INT |
                                         LPI2C_HAL_MASTER_NACK_DETECT_INT |
                                         LPI2C_HAL_MASTER_TRANSMIT_DATA_INT |
                                         LPI2C_HAL_MASTER_RECEIVE_DATA_INT,
                               true);
    }
    else
    {
        LPI2C_HAL_MasterSetInt(baseAddr, LPI2C_HAL_MASTER_FIFO_ERROR_INT |
                                         LPI2C_HAL_MASTER_ARBITRATION_LOST_INT |
                                         LPI2C_HAL_MASTER_NACK_DETECT_INT |
                                         LPI2C_HAL_MASTER_RECEIVE_DATA_INT,
                               true);
    }

    if (blocking)
    {
        while (master->i2cIdle == false);
        return master->status;
    }
    else
    {
        return LPI2C_STATUS_SUCCESS;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterSetOperatingMode
 * Description   : sets the operating mode of the I2C master
 *
 *END**************************************************************************/
static void LPI2C_DRV_MasterSetOperatingMode(uint32_t instance, lpi2c_mode_t operatingMode)
{
    LPI2C_Type *baseAddr;
    lpi2c_master_state_t * master;

    baseAddr = g_lpi2cBase[instance];
    master = (lpi2c_master_state_t *)g_lpi2cStatePtr[instance];

    if (operatingMode == LPI2C_ULTRAFAST_MODE)
    {
        LPI2C_HAL_MasterSetPinConfig(baseAddr, LPI2C_CFG_2PIN_OUTPUT_ONLY);
        LPI2C_HAL_MasterSetNACKConfig(baseAddr, LPI2C_NACK_IGNORE);
    }
    else
    {
        LPI2C_HAL_MasterSetPinConfig(baseAddr, LPI2C_CFG_2PIN_OPEN_DRAIN);
        LPI2C_HAL_MasterSetNACKConfig(baseAddr, LPI2C_NACK_RECEIVE);
    }
    master->operatingMode = operatingMode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_SlaveSetOperatingMode
 * Description   : sets the operating mode of the I2C slave
 *
 *END**************************************************************************/
static void LPI2C_DRV_SlaveSetOperatingMode(uint32_t instance, lpi2c_mode_t operatingMode)
{
    LPI2C_Type *baseAddr;
    lpi2c_slave_state_t * slave;

    baseAddr = g_lpi2cBase[instance];
    slave = (lpi2c_slave_state_t *)g_lpi2cStatePtr[instance];

    if (operatingMode == LPI2C_ULTRAFAST_MODE)
    {
        LPI2C_HAL_SlaveSetIgnoreNACK(baseAddr, LPI2C_SLAVE_NACK_CONTINUE_TRANSFER);
        LPI2C_HAL_SlaveSetTransmitNACK(baseAddr, LPI2C_SLAVE_TRANSMIT_NACK);
        /* Disable all clock stretching in ultra-fast mode */
        LPI2C_HAL_SlaveSetACKStall(baseAddr, false);
        LPI2C_HAL_SlaveSetTXDStall(baseAddr, false);
        LPI2C_HAL_SlaveSetRXStall(baseAddr, false);
        LPI2C_HAL_SlaveSetAddrStall(baseAddr, false);
    }
    else
    {
        LPI2C_HAL_SlaveSetIgnoreNACK(baseAddr, LPI2C_SLAVE_NACK_END_TRANSFER);
        LPI2C_HAL_SlaveSetTransmitNACK(baseAddr, LPI2C_SLAVE_TRANSMIT_ACK);
        /* Enable clock stretching except ACKSTALL (we don't need to send ACK/NACK manually) */
        LPI2C_HAL_SlaveSetACKStall(baseAddr, false);
        LPI2C_HAL_SlaveSetTXDStall(baseAddr, true);
        LPI2C_HAL_SlaveSetRXStall(baseAddr, true);
        LPI2C_HAL_SlaveSetAddrStall(baseAddr, true);
    }
    if (operatingMode == LPI2C_HIGHSPEED_MODE)
    {
        /* Enable detection of the High-speed Mode master code */
        LPI2C_HAL_SlaveSetHighSpeedModeDetect(baseAddr, true);
    }
    else
    {
        /* Disable detection of the High-speed Mode master code */
        LPI2C_HAL_SlaveSetHighSpeedModeDetect(baseAddr, false);
    }

    slave->operatingMode = operatingMode;
}

/*! @endcond */

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterInit
 * Description   : initialize the I2C master mode driver
 *
 *END**************************************************************************/
lpi2c_status_t LPI2C_DRV_MasterInit(uint32_t instance,
                                    const lpi2c_master_user_config_t * userConfigPtr,
                                    lpi2c_master_state_t * master)
{
    LPI2C_Type *baseAddr;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(master != NULL);
    DEV_ASSERT(instance < LPI2C_INSTANCE_COUNT);
#endif

    if (g_lpi2cStatePtr[instance])
    {
        return LPI2C_STATUS_FAIL;
    }

    baseAddr = g_lpi2cBase[instance];
    g_lpi2cStatePtr[instance] = master;

    /* Initialize driver status structure */
    master->rxBuff = NULL;
    master->rxSize = 0U;
    master->txBuff = NULL;
    master->txSize = 0U;
    master->status = LPI2C_STATUS_SUCCESS;
    master->i2cIdle = true;
    master->masterCode = userConfigPtr->masterCode;
    master->slaveAddress = userConfigPtr->slaveAddress;
    master->is10bitAddr = userConfigPtr->is10bitAddr;
    master->highSpeedInProgress = false;
    LPI2C_DRV_MasterResetQueue(master);

    /* Enable lpi2c interrupt */
    INT_SYS_EnableIRQ(g_lpi2cIrqId[instance]);

    /* Initialize module */
    LPI2C_HAL_Init(baseAddr);

    /* Set baud rate */
    LPI2C_DRV_MasterSetBaudRate(instance, userConfigPtr->operatingMode, userConfigPtr->baudRate, userConfigPtr->baudRateHS);

    /* Set slave address */
    LPI2C_DRV_MasterSetSlaveAddr(instance, userConfigPtr->slaveAddress, userConfigPtr->is10bitAddr);

    /* Enable LPI2C master */
    LPI2C_HAL_MasterSetEnable(baseAddr, true);

    return LPI2C_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterDeinit
 * Description   : deinitialize the I2C master mode driver
 *
 *END**************************************************************************/
lpi2c_status_t LPI2C_DRV_MasterDeinit(uint32_t instance)
{
    LPI2C_Type *baseAddr;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPI2C_INSTANCE_COUNT);
#endif

    baseAddr = g_lpi2cBase[instance];

    g_lpi2cStatePtr[instance] = NULL;

    /* Disable master */
    LPI2C_HAL_MasterSetEnable(baseAddr, false);

    /* Disable i2c interrupt */
    INT_SYS_DisableIRQ(g_lpi2cIrqId[instance]);

    return LPI2C_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterGetBaudRate
 * Description   : returns the currently configured baud rate
 *
 *END**************************************************************************/
lpi2c_status_t LPI2C_DRV_MasterGetBaudRate(uint32_t instance, uint32_t *baudRate, uint32_t *baudRateHS)
{
    LPI2C_Type *baseAddr;
    lpi2c_master_state_t * master;
    clock_manager_error_code_t clkErr;
    uint32_t prescaler;
    uint32_t clkLo;
    uint32_t clkHi;
    uint32_t inputClock;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPI2C_INSTANCE_COUNT);
#endif

    baseAddr = g_lpi2cBase[instance];
    master = (lpi2c_master_state_t *)g_lpi2cStatePtr[instance];

    /* Get the protocol clock frequency */
    clkErr = CLOCK_SYS_GetFreq(g_lpi2cClock[instance], &inputClock);
    if ((clkErr != CLOCK_MANAGER_SUCCESS) || (inputClock == 0U))
    {
        /* No clock configured for this module */
        return LPI2C_STATUS_FAIL;
    }
    /* Ignoring the glitch filter, the baud rate formula is:
            SCL_freq = Input_freq / (2^PRESCALER * (CLKLO + CLKHI + 2))
    */
    prescaler = LPI2C_HAL_MasterGetPrescaler(baseAddr);
    clkHi = LPI2C_HAL_MasterGetClockHighPeriod(baseAddr);
    clkLo = LPI2C_HAL_MasterGetClockLowPeriod(baseAddr);

    *baudRate = inputClock / ((1U << prescaler) * (clkLo + clkHi + 2U));

    if (master->operatingMode == LPI2C_HIGHSPEED_MODE)
    {
        clkHi = LPI2C_HAL_MasterGetClockHighPeriodHS(baseAddr);
        clkLo = LPI2C_HAL_MasterGetClockLowPeriodHS(baseAddr);

        *baudRateHS = inputClock / ((1U << prescaler) * (clkLo + clkHi + 2U));
    }

    return LPI2C_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterSetBaudRate
 * Description   : set the baud rate for any subsequent I2C communication
 *
 *END**************************************************************************/
lpi2c_status_t LPI2C_DRV_MasterSetBaudRate(uint32_t instance,
                                           const lpi2c_mode_t operatingMode,
                                           const uint32_t baudRate,
                                           const uint32_t baudRateHS)
{
    LPI2C_Type *baseAddr;
    lpi2c_master_state_t * master;
    bool masterEnabled;
    clock_manager_error_code_t clkErr;
    uint32_t inputClock;
    uint32_t minPrescaler;
    uint32_t prescaler;
    uint32_t clkTotal;
    uint32_t clkLo;
    uint32_t clkHi;
    uint32_t setHold;
    uint32_t dataVd;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPI2C_INSTANCE_COUNT);
#endif

    baseAddr = g_lpi2cBase[instance];
    master = (lpi2c_master_state_t *)g_lpi2cStatePtr[instance];

    /* Check if driver is busy */
    if (!master->i2cIdle)
    {
        return LPI2C_STATUS_BUSY;
    }

    /* Get the protocol clock frequency */
    clkErr = CLOCK_SYS_GetFreq(g_lpi2cClock[instance], &inputClock);
    if ((clkErr != CLOCK_MANAGER_SUCCESS) || (inputClock == 0U))
    {
        /* No clock configured for this module */
        return LPI2C_STATUS_FAIL;
    }

    masterEnabled = LPI2C_HAL_MasterGetEnable(baseAddr);
    if (masterEnabled)
    {
        /* Disable master */
        LPI2C_HAL_MasterSetEnable(baseAddr, false);
    }
    /* Ignoring the glitch filter, the baud rate formula is:
            SCL_freq = Input_freq / (2^PRESCALER * (CLKLO + CLKHI + 2))
            Assume CLKLO = 2*CLKHI, SETHOLD = CLKHI, DATAVD = CLKHI/2
    */
    /* Compute minimum prescaler for which CLKLO and CLKHI values are in valid range. Always round up. */
    minPrescaler = (inputClock - 1U) / (baudRate * (CLKHI_MAX_VALUE + CLKLO_MAX_VALUE + 2U)) + 1U;
    for (prescaler = 0U; prescaler < 7U; prescaler++)
    {
        if ((1U << prescaler) >= minPrescaler) break;
    }
    /* Compute CLKLO and CLKHI values for this prescaler. Round to nearest integer. */
    clkTotal = (inputClock + ((baudRate << prescaler) >> 1U)) / (baudRate << prescaler);
    if (clkTotal > (CLKHI_MAX_VALUE + CLKLO_MAX_VALUE + 2U))
    {
        clkTotal = (CLKHI_MAX_VALUE + CLKLO_MAX_VALUE + 2U);
    }
    clkHi = (clkTotal - 2U) / 3U;
    clkLo = clkTotal - 2U - clkHi;
    if (clkHi < CLKHI_MIN_VALUE)
    {
        clkHi = CLKHI_MIN_VALUE;
    }
    if (clkLo < CLKLO_MIN_VALUE)
    {
        clkLo = CLKLO_MIN_VALUE;
    }
    /* Compute DATAVD and SETHOLD */
    setHold = clkHi;
    dataVd = clkHi >> 1U;
    if (setHold < SETHOLD_MIN_VALUE)
    {
        setHold = SETHOLD_MIN_VALUE;
    }
    if (dataVd < DATAVD_MIN_VALUE)
    {
        dataVd = DATAVD_MIN_VALUE;
    }

    /* Apply settings */
    LPI2C_HAL_MasterSetPrescaler(baseAddr, (lpi2c_master_prescaler_t)prescaler);
    LPI2C_HAL_MasterSetDataValidDelay(baseAddr, dataVd);
    LPI2C_HAL_MasterSetSetupHoldDelay(baseAddr, setHold);
    LPI2C_HAL_MasterSetClockHighPeriod(baseAddr, clkHi);
    LPI2C_HAL_MasterSetClockLowPeriod(baseAddr, clkLo);

    if (operatingMode == LPI2C_HIGHSPEED_MODE)
    {
        /* Compute settings for High-speed baud rate */
        /* Compute High-speed CLKLO and CLKHI values for the same prescaler. Round to nearest integer. */
        clkTotal = (inputClock + ((baudRateHS << prescaler) >> 1U)) / (baudRateHS << prescaler);
        if (clkTotal > (CLKHI_MAX_VALUE + CLKLO_MAX_VALUE + 2U))
        {
            clkTotal = (CLKHI_MAX_VALUE + CLKLO_MAX_VALUE + 2U);
        }
        clkHi = (clkTotal - 2U) / 3U;
        clkLo = clkTotal - 2U - clkHi;
        if (clkHi < CLKHI_MIN_VALUE)
        {
            clkHi = CLKHI_MIN_VALUE;
        }
        if (clkLo < CLKLO_MIN_VALUE)
        {
            clkLo = CLKLO_MIN_VALUE;
        }
        /* Compute High-speed DATAVD and SETHOLD */
        setHold = clkHi;
        dataVd = clkHi >> 1U;
        if (setHold < SETHOLD_MIN_VALUE)
        {
            setHold = SETHOLD_MIN_VALUE;
        }
        if (dataVd < DATAVD_MIN_VALUE)
        {
            dataVd = DATAVD_MIN_VALUE;
        }

        /* Apply High-speed settings */
        LPI2C_HAL_MasterSetDataValidDelayHS(baseAddr, dataVd);
        LPI2C_HAL_MasterSetSetupHoldDelayHS(baseAddr, setHold);
        LPI2C_HAL_MasterSetClockHighPeriodHS(baseAddr, clkHi);
        LPI2C_HAL_MasterSetClockLowPeriodHS(baseAddr, clkLo);
    }
    if (master->operatingMode != operatingMode)
    {
         /* Perform other settings related to the chosen operating mode */
        LPI2C_DRV_MasterSetOperatingMode(instance, operatingMode);
    }

    if (masterEnabled)
    {
        /* Re-enable master */
        LPI2C_HAL_MasterSetEnable(baseAddr, true);
    }

    return LPI2C_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterSetSlaveAddr
 * Description   : set the slave address for any subsequent I2C communication
 *
 *END**************************************************************************/
void LPI2C_DRV_MasterSetSlaveAddr(uint32_t instance, const uint16_t address, const bool is10bitAddr)
{
    lpi2c_master_state_t * master;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPI2C_INSTANCE_COUNT);
#endif

    master = (lpi2c_master_state_t *)g_lpi2cStatePtr[instance];

    master->slaveAddress = address;
    master->is10bitAddr = is10bitAddr;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterSendDataBlocking
 * Description   : perform a blocking send transaction on the I2C bus
 *
 *END**************************************************************************/
lpi2c_status_t LPI2C_DRV_MasterSendDataBlocking(uint32_t instance,
                                            const uint8_t * txBuff,
                                            uint32_t txSize,
                                            bool sendStop)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(txBuff != NULL);
    DEV_ASSERT(instance < LPI2C_INSTANCE_COUNT);
#endif

    return LPI2C_DRV_MasterSend(instance, txBuff, txSize, sendStop, true);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterSendData
 * Description   : perform a non-blocking send transaction on the I2C bus
 *
 *END**************************************************************************/
lpi2c_status_t LPI2C_DRV_MasterSendData(uint32_t instance,
                                    const uint8_t * txBuff,
                                    uint32_t txSize,
                                    bool sendStop)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(txBuff != NULL);
    DEV_ASSERT(instance < LPI2C_INSTANCE_COUNT);
#endif

    return LPI2C_DRV_MasterSend(instance, txBuff, txSize, sendStop, false);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterGetSendStatus
 * Description   : return the current status of the I2C master transmit
 *
 *END**************************************************************************/
lpi2c_status_t LPI2C_DRV_MasterGetSendStatus(uint32_t instance, uint32_t *bytesRemaining)
{
    LPI2C_Type *baseAddr;
    lpi2c_master_state_t * master;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPI2C_INSTANCE_COUNT);
#endif

    baseAddr = g_lpi2cBase[instance];
    master = (lpi2c_master_state_t *)g_lpi2cStatePtr[instance];

    if (bytesRemaining)
    {
        /* Remaining bytes = bytes in buffer + bytes in tx FIFO */
        *bytesRemaining = master->txSize + LPI2C_HAL_MasterGetTxFIFOCount(baseAddr);
    }

    return master->status;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterAbortTransferData
 * Description   : abort a non-blocking I2C Master transmission or reception
 *
 *END**************************************************************************/
lpi2c_status_t LPI2C_DRV_MasterAbortTransferData(uint32_t instance)
{
    LPI2C_Type *baseAddr;
    lpi2c_master_state_t * master;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPI2C_INSTANCE_COUNT);
#endif

    baseAddr = g_lpi2cBase[instance];
    master = (lpi2c_master_state_t *)g_lpi2cStatePtr[instance];

    if (master->i2cIdle == true)
    {
        /* No transfer in progress */
        return LPI2C_STATUS_NO_TRANSFER;
    }
    if (master->rxBuff != NULL)
    {
        /* Aborting a reception not supported because hardware will continue the 
           current command even if the FIFO is reset and this could last indefinitely */
        return LPI2C_STATUS_FAIL;
    }
    /* End transfer: force stop generation, reset FIFOs */
    LPI2C_DRV_MasterEndTransfer(baseAddr, master, true, true);
    master->status = LPI2C_STATUS_ABORTED;
    return LPI2C_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterReceiveDataBlocking
 * Description   : perform a blocking receive transaction on the I2C bus
 *
 *END**************************************************************************/
lpi2c_status_t LPI2C_DRV_MasterReceiveDataBlocking(uint32_t instance,
                                               uint8_t * rxBuff,
                                               uint32_t rxSize,
                                               bool sendStop)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(rxBuff != NULL);
    DEV_ASSERT(instance < LPI2C_INSTANCE_COUNT);
#endif

    return LPI2C_DRV_MasterReceive(instance, rxBuff, rxSize, sendStop, true);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterReceiveData
 * Description   : perform a non-blocking receive transaction on the I2C bus
 *
 *END**************************************************************************/
lpi2c_status_t LPI2C_DRV_MasterReceiveData(uint32_t  instance,
                                       uint8_t * rxBuff,
                                       uint32_t rxSize,
                                       bool sendStop)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(rxBuff != NULL);
    DEV_ASSERT(instance < LPI2C_INSTANCE_COUNT);
#endif

    return LPI2C_DRV_MasterReceive(instance, rxBuff, rxSize, sendStop, false);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterGetReceiveStatus
 * Description   : return the current status of the I2C master receive
 *
 *END**************************************************************************/
lpi2c_status_t LPI2C_DRV_MasterGetReceiveStatus(uint32_t instance,
                                            uint32_t *bytesRemaining)
{
    LPI2C_Type *baseAddr;
    lpi2c_master_state_t * master;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPI2C_INSTANCE_COUNT);
#endif

    baseAddr = g_lpi2cBase[instance];
    master = (lpi2c_master_state_t *)g_lpi2cStatePtr[instance];

    if (bytesRemaining)
    {
        /* Remaining bytes = free space in buffer - bytes in rx FIFO */
        *bytesRemaining = master->rxSize - LPI2C_HAL_MasterGetRxFIFOCount(baseAddr);
    }

    return master->status;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_MasterIRQHandler
 * Description   : handle non-blocking master operation when I2C interrupt occurs
 *
 *END**************************************************************************/
void LPI2C_DRV_MasterIRQHandler(uint32_t instance)
{
    LPI2C_Type *baseAddr;
    lpi2c_master_state_t * master;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPI2C_INSTANCE_COUNT);
#endif

    baseAddr = g_lpi2cBase[instance];
    master = (lpi2c_master_state_t *)g_lpi2cStatePtr[instance];

    /* Check which event caused the interrupt */
    if (LPI2C_HAL_MasterGetTransmitDataRequestEvent(baseAddr))
    {
        /* More data needed for transmission */
        if (!LPI2C_DRV_MasterCmdQueueEmpty(master))
        {
            /* If there are queued commands, send them */
            LPI2C_DRV_MasterSendQueuedCmd(baseAddr, master);
        }
        else if (master->txBuff != NULL)
        {
            /* A transmission is in progress */
            if (master->txSize == 0U)
            {
                /* There is no more data in buffer, the transmission is over */
                LPI2C_DRV_MasterEndTransfer(baseAddr, master, master->sendStop, false);
                master->status = LPI2C_STATUS_SUCCESS;
            }
            else
            {
                /* Queue data bytes to fill tx fifo */
                LPI2C_DRV_MasterQueueData(baseAddr, master);
            }
        }
        else
        {
            /* No more commands and no transmission in progress - disable tx event */
            LPI2C_HAL_MasterSetInt(baseAddr, LPI2C_HAL_MASTER_TRANSMIT_DATA_INT, false);
        }
    }
    if (LPI2C_HAL_MasterGetReceiveDataReadyEvent(baseAddr))
    {
        /* Received data ready */
#ifdef DEV_ERROR_DETECT
        DEV_ASSERT(master->rxBuff != NULL);
#endif
        /* Transfer received data to user buffer */
        while ((LPI2C_HAL_MasterGetRxFIFOCount(baseAddr) > 0U) && (master->rxSize > 0U))
        {
            master->rxBuff[0U] = LPI2C_HAL_MasterGetRxData(baseAddr);
            master->rxBuff++;
            master->rxSize--;
        }
        if (master->rxSize == 0U)
        {
            /* Done receiving */
            LPI2C_DRV_MasterEndTransfer(baseAddr, master, master->sendStop, false);
            master->status = LPI2C_STATUS_SUCCESS;
        }
        else if (master->rxSize <= LPI2C_HAL_MasterGetRxFIFOWatermark(baseAddr))
        {
            /* Reduce rx watermark to receive the last few bytes */
            LPI2C_HAL_MasterSetRxFIFOWatermark(baseAddr, master->rxSize - 1U);
        }
        else
        {
            /* Continue reception */
        }
    }
    if (LPI2C_HAL_MasterGetFIFOErrorEvent(baseAddr))
    {
        /* FIFO error */
        LPI2C_HAL_MasterClearFIFOErrorEvent(baseAddr);
        /* End transfer: no stop generation (the module will handle that by itself 
           if needed), reset FIFOs */
        LPI2C_DRV_MasterEndTransfer(baseAddr, master, false, true);
        master->highSpeedInProgress = false; /* High-speed transfers end at STOP condition */
        master->status = LPI2C_STATUS_FAIL;
    }
    if (LPI2C_HAL_MasterGetArbitrationLostEvent(baseAddr))
    {
        /* Arbitration lost */
        LPI2C_HAL_MasterClearArbitrationLostEvent(baseAddr);
        /* End transfer: no stop generation (the module will handle that by itself 
           if needed), reset FIFOs */
        LPI2C_DRV_MasterEndTransfer(baseAddr, master, false, true);
        master->status = LPI2C_STATUS_ARIBTRATION_LOST;
    }
    if (LPI2C_HAL_MasterGetNACKDetectEvent(baseAddr))
    {
        /* Received NACK */
        LPI2C_HAL_MasterClearNACKDetectEvent(baseAddr);
        /* Ignore NACK in Ultra Fast mode */
        if (master->operatingMode != LPI2C_ULTRAFAST_MODE)
        {
            /* End transfer: no stop generation (the module will handle that by itself 
               if needed), reset FIFOs */
            LPI2C_DRV_MasterEndTransfer(baseAddr, master, false, true);
            master->highSpeedInProgress = false; /* High-speed transfers end at STOP condition */
            master->status = LPI2C_STATUS_RECEIVED_NACK;
        }
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_SlaveInit
 * Description   : initialize the I2C slave mode driver
 *
 *END**************************************************************************/
lpi2c_status_t LPI2C_DRV_SlaveInit(uint32_t instance,
                                   const lpi2c_slave_user_config_t * userConfigPtr,
                                   lpi2c_slave_state_t * slave)
{
    LPI2C_Type *baseAddr;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(userConfigPtr != NULL);
    DEV_ASSERT(slave != NULL);
    DEV_ASSERT(instance < LPI2C_INSTANCE_COUNT);
#endif

    if (g_lpi2cStatePtr[instance])
    {
        return LPI2C_STATUS_FAIL;
    }

    baseAddr = g_lpi2cBase[instance];
    g_lpi2cStatePtr[instance] = slave;

    /* Initialize driver status structure */
    slave->status = LPI2C_STATUS_SUCCESS;
    slave->slaveListening = userConfigPtr->slaveListening;
    slave->slaveCallback = userConfigPtr->slaveCallback;
    slave->callbackParam = userConfigPtr->callbackParam;
    slave->txBuff = NULL;
    slave->rxBuff = NULL;
    slave->txSize = 0U;
    slave->rxSize = 0U;
    slave->txBusy = false;
    slave->rxBusy = false;

    /* Enable lpi2c interrupt */
    INT_SYS_EnableIRQ(g_lpi2cIrqId[instance]);

    /* Initialize module */
    LPI2C_HAL_Init(baseAddr);

    /* Configure slave address */
    LPI2C_HAL_SlaveSetAddr0(baseAddr, userConfigPtr->slaveAddress);
    if (userConfigPtr->is10bitAddr)
    {
        LPI2C_HAL_SlaveSetAddrConfig(baseAddr, LPI2C_SLAVE_ADDR_MATCH_0_10BIT);
    }
    else
    {
        LPI2C_HAL_SlaveSetAddrConfig(baseAddr, LPI2C_SLAVE_ADDR_MATCH_0_7BIT);
    }

    /* Configure operating mode */
    LPI2C_DRV_SlaveSetOperatingMode(instance, userConfigPtr->operatingMode);

    if (userConfigPtr->slaveListening)
    {
        /* Activate events */
        LPI2C_HAL_SlaveSetInt(baseAddr, LPI2C_HAL_SLAVE_BIT_ERROR_INT |
                                        LPI2C_HAL_SLAVE_FIFO_ERROR_INT |
                                        LPI2C_HAL_SLAVE_STOP_DETECT_INT |
                                        LPI2C_HAL_SLAVE_REPEATED_START_INT |
                                        LPI2C_HAL_SLAVE_ADDRESS_VALID_INT |
                                        LPI2C_HAL_SLAVE_RECEIVE_DATA_INT |
                                        LPI2C_HAL_SLAVE_TRANSMIT_DATA_INT,
                              true);
        /* Enable LPI2C slave */
        LPI2C_HAL_SlaveSetEnable(baseAddr, true);
    }


    return LPI2C_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_SlaveDeinit
 * Description   : de-initialize the I2C slave mode driver
 *
 *END**************************************************************************/
lpi2c_status_t LPI2C_DRV_SlaveDeinit(uint32_t instance)
{
    LPI2C_Type *baseAddr;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPI2C_INSTANCE_COUNT);
#endif

    baseAddr = g_lpi2cBase[instance];

    g_lpi2cStatePtr[instance] = NULL;

    /* Disable LPI2C slave */
    LPI2C_HAL_SlaveSetEnable(baseAddr, false);

    /* Disable i2c interrupt */
    INT_SYS_DisableIRQ(g_lpi2cIrqId[instance]);

    return LPI2C_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_SlaveGetHandler
 * Description   : return the I2C slave run-time state structure
 *
 *END**************************************************************************/
lpi2c_slave_state_t * LPI2C_DRV_SlaveGetHandler(uint32_t instance)
{
    lpi2c_slave_state_t * slave;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPI2C_INSTANCE_COUNT);
#endif

    slave = (lpi2c_slave_state_t *)g_lpi2cStatePtr[instance];

    return slave;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_SlaveSetTxBuffer
 * Description   : Provide a buffer for transmitting data.
 *
 *END**************************************************************************/
lpi2c_status_t LPI2C_DRV_SlaveSetTxBuffer(uint32_t instance,
                                                 const uint8_t * txBuff,
                                                 uint32_t txSize)
{
    lpi2c_slave_state_t * slave;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(txBuff != NULL);
    DEV_ASSERT(instance < LPI2C_INSTANCE_COUNT);
#endif

    slave = (lpi2c_slave_state_t *)g_lpi2cStatePtr[instance];

    slave->txBuff = txBuff;
    slave->txSize = txSize;

    return LPI2C_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_SlaveSetRxBuffer
 * Description   : Provide a buffer for receiving data.
 *
 *END**************************************************************************/
lpi2c_status_t LPI2C_DRV_SlaveSetRxBuffer(uint32_t instance,
                                                 uint8_t * rxBuff,
                                                 uint32_t  rxSize)
{
    lpi2c_slave_state_t * slave;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(rxBuff != NULL);
    DEV_ASSERT(instance < LPI2C_INSTANCE_COUNT);
#endif

    slave = (lpi2c_slave_state_t *)g_lpi2cStatePtr[instance];

    slave->rxBuff = rxBuff;
    slave->rxSize = rxSize;

    return LPI2C_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_SlaveSendData
 * Description   : perform a non-blocking send transaction on the I2C bus
 *
 *END**************************************************************************/
lpi2c_status_t LPI2C_DRV_SlaveSendData(uint32_t instance,
                                   const uint8_t * txBuff,
                                   uint32_t txSize)
{
    LPI2C_Type *baseAddr;
    lpi2c_slave_state_t * slave;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(txBuff != NULL);
    DEV_ASSERT(instance < LPI2C_INSTANCE_COUNT);
#endif

    baseAddr = g_lpi2cBase[instance];
    slave = (lpi2c_slave_state_t *)g_lpi2cStatePtr[instance];

    if (!slave->slaveListening)
    {
        if (slave->txBusy == true)
        {
            /* Slave is busy */
            return LPI2C_STATUS_BUSY;
        }
        slave->txBuff = txBuff;
        slave->txSize = txSize;
        slave->status = LPI2C_STATUS_BUSY;
        /* Activate events */
        LPI2C_HAL_SlaveSetInt(baseAddr, LPI2C_HAL_SLAVE_BIT_ERROR_INT |
                                        LPI2C_HAL_SLAVE_FIFO_ERROR_INT |
                                        LPI2C_HAL_SLAVE_STOP_DETECT_INT |
                                        LPI2C_HAL_SLAVE_REPEATED_START_INT |
                                        LPI2C_HAL_SLAVE_ADDRESS_VALID_INT |
                                        LPI2C_HAL_SLAVE_TRANSMIT_DATA_INT,
                              true);
        slave->txBusy = true;
        /* Enable LPI2C slave */
        LPI2C_HAL_SlaveSetEnable(baseAddr, true);
    }

    return LPI2C_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_SlaveSendDataBlocking
 * Description   : perform a blocking send transaction on the I2C bus
 *
 *END**************************************************************************/
lpi2c_status_t LPI2C_DRV_SlaveSendDataBlocking(uint32_t    instance,
                                           const uint8_t *  txBuff,
                                           uint32_t   txSize)
{
    LPI2C_Type *baseAddr;
    lpi2c_slave_state_t * slave;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(txBuff != NULL);
    DEV_ASSERT(instance < LPI2C_INSTANCE_COUNT);
#endif

    baseAddr = g_lpi2cBase[instance];
    slave = (lpi2c_slave_state_t *)g_lpi2cStatePtr[instance];
    if (!slave->slaveListening)
    {
        if (slave->txBusy == true)
        {
            /* Slave is busy */
            return LPI2C_STATUS_BUSY;
        }
        slave->txBuff = txBuff;
        slave->txSize = txSize;
        slave->status = LPI2C_STATUS_BUSY;
        /* Activate events */
        LPI2C_HAL_SlaveSetInt(baseAddr, LPI2C_HAL_SLAVE_BIT_ERROR_INT |
                                        LPI2C_HAL_SLAVE_FIFO_ERROR_INT |
                                        LPI2C_HAL_SLAVE_STOP_DETECT_INT |
                                        LPI2C_HAL_SLAVE_REPEATED_START_INT |
                                        LPI2C_HAL_SLAVE_ADDRESS_VALID_INT |
                                        LPI2C_HAL_SLAVE_TRANSMIT_DATA_INT,
                              true);
        slave->txBusy = true;
        /* Enable LPI2C slave */
        LPI2C_HAL_SlaveSetEnable(baseAddr, true);
        while (slave->txBusy == true)
        {
            /* Wait for transmission to be completed */
        }
    }

    return LPI2C_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_SlaveReceiveData
 * Description   : perform a non-blocking receive transaction on the I2C bus
 *
 *END**************************************************************************/
lpi2c_status_t LPI2C_DRV_SlaveReceiveData(uint32_t   instance,
                                       uint8_t * rxBuff,
                                       uint32_t  rxSize)
{
    LPI2C_Type *baseAddr;
    lpi2c_slave_state_t * slave;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(rxBuff != NULL);
    DEV_ASSERT(instance < LPI2C_INSTANCE_COUNT);
#endif

    baseAddr = g_lpi2cBase[instance];
    slave = (lpi2c_slave_state_t *)g_lpi2cStatePtr[instance];

    if (!slave->slaveListening)
    {
        if (slave->rxBusy == true)
        {
            /* Slave is busy */
            return LPI2C_STATUS_BUSY;
        }
        slave->rxBuff = rxBuff;
        slave->rxSize = rxSize;
        slave->status = LPI2C_STATUS_BUSY;
        /* Activate events */
        LPI2C_HAL_SlaveSetInt(baseAddr, LPI2C_HAL_SLAVE_BIT_ERROR_INT |
                                        LPI2C_HAL_SLAVE_FIFO_ERROR_INT |
                                        LPI2C_HAL_SLAVE_STOP_DETECT_INT |
                                        LPI2C_HAL_SLAVE_REPEATED_START_INT |
                                        LPI2C_HAL_SLAVE_ADDRESS_VALID_INT |
                                        LPI2C_HAL_SLAVE_RECEIVE_DATA_INT,
                              true);
        slave->rxBusy = true;
        /* Enable LPI2C slave */
        LPI2C_HAL_SlaveSetEnable(baseAddr, true);
    }

    return LPI2C_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_SlaveReceiveDataBlocking
 * Description   : perform a blocking receive transaction on the I2C bus
 *
 *END**************************************************************************/
lpi2c_status_t LPI2C_DRV_SlaveReceiveDataBlocking(uint32_t instance,
                                       uint8_t  * rxBuff,
                                       uint32_t   rxSize)
{
    LPI2C_Type *baseAddr;
    lpi2c_slave_state_t * slave;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(rxBuff != NULL);
    DEV_ASSERT(instance < LPI2C_INSTANCE_COUNT);
#endif

    baseAddr = g_lpi2cBase[instance];
    slave = (lpi2c_slave_state_t *)g_lpi2cStatePtr[instance];

    if (!slave->slaveListening)
    {
        if (slave->rxBusy == true)
        {
            /* Slave is busy */
            return LPI2C_STATUS_BUSY;
        }
        slave->rxBuff = rxBuff;
        slave->rxSize = rxSize;
        slave->status = LPI2C_STATUS_BUSY;
        /* Activate events */
        LPI2C_HAL_SlaveSetInt(baseAddr, LPI2C_HAL_SLAVE_BIT_ERROR_INT |
                                        LPI2C_HAL_SLAVE_FIFO_ERROR_INT |
                                        LPI2C_HAL_SLAVE_STOP_DETECT_INT |
                                        LPI2C_HAL_SLAVE_REPEATED_START_INT |
                                        LPI2C_HAL_SLAVE_ADDRESS_VALID_INT |
                                        LPI2C_HAL_SLAVE_RECEIVE_DATA_INT,
                              true);
        slave->rxBusy = true;
        /* Enable LPI2C slave */
        LPI2C_HAL_SlaveSetEnable(baseAddr, true);
        while (slave->rxBusy == true)
        {
            /* Wait for reception to be completed */
        }
    }

    return LPI2C_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_SlaveGetReceiveStatus
 * Description   : return the current status of the I2C slave receive
 *
 *END**************************************************************************/
lpi2c_status_t LPI2C_DRV_SlaveGetReceiveStatus(uint32_t instance,
                                               uint32_t *bytesRemaining)
{
    lpi2c_slave_state_t * slave;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPI2C_INSTANCE_COUNT);
#endif

    slave = (lpi2c_slave_state_t *)g_lpi2cStatePtr[instance];

    if (bytesRemaining)
    {
        *bytesRemaining = slave->rxSize;
    }

    return slave->status;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_SlaveGetTransmitStatus
 * Description   : return the current status of the I2C slave transmit
 *
 *END**************************************************************************/
lpi2c_status_t LPI2C_DRV_SlaveGetTransmitStatus(uint32_t instance,
                                                uint32_t *bytesRemaining)
{
    lpi2c_slave_state_t * slave;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPI2C_INSTANCE_COUNT);
#endif

    slave = (lpi2c_slave_state_t *)g_lpi2cStatePtr[instance];

    if (bytesRemaining)
    {
        *bytesRemaining = slave->txSize;
    }

    return slave->status;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_SlaveAbortTransferData
 * Description   : abort a non-blocking I2C Master transmission or reception
 *
 *END**************************************************************************/
lpi2c_status_t LPI2C_DRV_SlaveAbortTransferData(uint32_t instance)
{
    lpi2c_slave_state_t * slave;
    LPI2C_Type *baseAddr;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPI2C_INSTANCE_COUNT);
#endif

    slave = (lpi2c_slave_state_t *)g_lpi2cStatePtr[instance];
    baseAddr = g_lpi2cBase[instance];

    if (!slave->slaveListening)
    {
        /* Deactivate events */
        LPI2C_HAL_SlaveSetInt(baseAddr, LPI2C_HAL_SLAVE_BIT_ERROR_INT |
                                        LPI2C_HAL_SLAVE_FIFO_ERROR_INT |
                                        LPI2C_HAL_SLAVE_STOP_DETECT_INT |
                                        LPI2C_HAL_SLAVE_REPEATED_START_INT |
                                        LPI2C_HAL_SLAVE_ADDRESS_VALID_INT |
                                        LPI2C_HAL_SLAVE_RECEIVE_DATA_INT |
                                        LPI2C_HAL_SLAVE_TRANSMIT_DATA_INT,
                              false);
        slave->rxBusy = false;
        slave->txBusy = false;
        /* Disable LPI2C slave */
        LPI2C_HAL_SlaveSetEnable(baseAddr, false);
    }

    return LPI2C_STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_SlaveIRQHandler
 * Description   : handle non-blocking slave operation when I2C interrupt occurs
 *
 *END**************************************************************************/
void LPI2C_DRV_SlaveIRQHandler(uint32_t instance)
{
    LPI2C_Type *baseAddr;
    lpi2c_slave_state_t * slave;
    uint16_t receivedAddr;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPI2C_INSTANCE_COUNT);
#endif

    baseAddr = g_lpi2cBase[instance];
    slave = (lpi2c_slave_state_t *)g_lpi2cStatePtr[instance];

    /* Check which event caused the interrupt */
    if (LPI2C_HAL_SlaveGetAddressValidEvent(baseAddr))
    {
        receivedAddr = LPI2C_HAL_SlaveGetReceivedAddr(baseAddr);
        if (receivedAddr & 1U)
        {
            /* Request from master to transmit data */
            if (slave->slaveCallback != NULL)
            {
                slave->slaveCallback(instance, LPI2C_SLAVE_EVENT_TX_REQ, slave->callbackParam);
            }
            slave->txUnderrunWarning = false;
        }
        else
        {
            /* Request from master to receive data */
            if (slave->slaveCallback != NULL)
            {
                slave->slaveCallback(instance, LPI2C_SLAVE_EVENT_RX_REQ, slave->callbackParam);
            }
        }
        slave->status = LPI2C_STATUS_BUSY;
    }
    if (LPI2C_HAL_SlaveGetTransmitDataEvent(baseAddr))
    {
        if (slave->txUnderrunWarning == true)
        {
            /* Another Tx event after underflow warning means the dummy char was sent */
            slave->status = LPI2C_STATUS_SLAVE_TX_UNDERRUN;
        }
        if (slave->txSize == 0U)
        {
            /* Out of data, call callback to allow user to provide a new buffer */
            if (slave->slaveCallback != NULL)
            {
                slave->slaveCallback(instance, LPI2C_SLAVE_EVENT_TX_EMPTY, slave->callbackParam);
            }
        }
        if (slave->txSize == 0U)
        {
            /* Still no data, record tx underflow event and send dummy char */
            /* Special case after the last tx byte: the device will ask for more data 
               but the dummy char will not be sent if NACK and then STOP condition are 
               received from master. So only record a "warning" for now. */
            slave->txUnderrunWarning = true;
            LPI2C_HAL_SlaveTransmitData(baseAddr, (uint8_t)0xFFU);
        }
        else
        {
            LPI2C_HAL_SlaveTransmitData(baseAddr, slave->txBuff[0U]);
            slave->txBuff++;
            slave->txSize--;
        }
    }
    if (LPI2C_HAL_SlaveGetReceiveDataEvent(baseAddr))
    {
        if (slave->rxSize == 0U)
        {
            /* No more room for data, call callback to allow user to provide a new buffer */
            if (slave->slaveCallback != NULL)
            {
                slave->slaveCallback(instance, LPI2C_SLAVE_EVENT_RX_FULL, slave->callbackParam);
            }
        }
        if (slave->rxSize == 0U)
        {
            /* Still no room for data, record rx overrun event and dummy read data */
            slave->status = LPI2C_STATUS_SLAVE_RX_OVERRUN;
            LPI2C_HAL_SlaveGetData(baseAddr);
        }
        else
        {
            slave->rxBuff[0U] = LPI2C_HAL_SlaveGetData(baseAddr);
            slave->rxBuff++;
            slave->rxSize--;
        }
    }
    if (LPI2C_HAL_SlaveGetSTOPDetectEvent(baseAddr) || 
        LPI2C_HAL_SlaveGetRepeatedStartEvent(baseAddr))
    {
        /* Either STOP or repeated START have the same meaning here: the current transfer is over */
        LPI2C_HAL_SlaveClearSTOPDetectEvent(baseAddr);
        LPI2C_HAL_SlaveClearRepeatedStartEvent(baseAddr);

        if (slave->status == LPI2C_STATUS_BUSY)
        {
            /* Report success if no error was recorded */
            slave->status = LPI2C_STATUS_SUCCESS;
        }
        if (!slave->slaveListening)
        {
            /* Deactivate events */
            LPI2C_HAL_SlaveSetInt(baseAddr, LPI2C_HAL_SLAVE_BIT_ERROR_INT |
                                            LPI2C_HAL_SLAVE_FIFO_ERROR_INT |
                                            LPI2C_HAL_SLAVE_STOP_DETECT_INT |
                                            LPI2C_HAL_SLAVE_REPEATED_START_INT |
                                            LPI2C_HAL_SLAVE_ADDRESS_VALID_INT |
                                            LPI2C_HAL_SLAVE_RECEIVE_DATA_INT |
                                            LPI2C_HAL_SLAVE_TRANSMIT_DATA_INT,
                                  false);
            slave->txBusy = false;
            slave->rxBusy = false;
            /* Disable LPI2C slave */
            LPI2C_HAL_SlaveSetEnable(baseAddr, false);
        }
    }
    if (LPI2C_HAL_SlaveGetBitErrorEvent(baseAddr))
    {
        LPI2C_HAL_SlaveClearBitErrorEvent(baseAddr);
        slave->status = LPI2C_STATUS_FAIL;
    }
    if (LPI2C_HAL_SlaveGetFIFOErrorEvent(baseAddr))
    {
        /* In Ultra-Fast mode clock stretching is disabled, so it is possible to get 
           this event if the slave can't keep up */
        LPI2C_HAL_SlaveClearFIFOErrorEvent(baseAddr);
        slave->status = LPI2C_STATUS_SLAVE_RX_OVERRUN;
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPI2C_DRV_IRQHandler
 * Description   : handle non-blocking slave or master operation when I2C interrupt occurs
 *
 *END**************************************************************************/
void LPI2C_DRV_IRQHandler(uint32_t instance)
{
    LPI2C_Type *baseAddr;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < LPI2C_INSTANCE_COUNT);
#endif

    baseAddr = g_lpi2cBase[instance];
    if (LPI2C_HAL_MasterGetEnable(baseAddr))
    {
        LPI2C_DRV_MasterIRQHandler(instance);
    }
    else if (LPI2C_HAL_SlaveGetEnable(baseAddr))
    {
        LPI2C_DRV_SlaveIRQHandler(instance);
    }
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
