/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
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

#include "fsl_lpspi_hal.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_HAL_IsModuleEnabled
 * Description   : Returns if the module is enabled or not..
 *
 *END**************************************************************************/
static bool LPSPI_HAL_IsModuleEnabled(LPSPI_Type * base)
{
    return (bool)BITBAND_ACCESS32(&(base->CR), LPSPI_CR_MEN_SHIFT);
}
/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_HAL_Init
 * Description   : Resets the LPSPI internal logic and registers to their default settings.
 *
 * This function first performs a software reset of the LPSPI module which resets the
 * internal LPSPI logic and most registers, then proceeds to manually reset all of the
 * LPSPI registers to their default setting to ensuring these registers at programmed to
 * their default value which includes disabling the module.
 *
 *END**************************************************************************/
void LPSPI_HAL_Init(LPSPI_Type * base)
{
    /* Software reset the internal logic */
    BITBAND_ACCESS32(&(base->CR), LPSPI_CR_RST_SHIFT) = (uint32_t)1U;

    /* Now bring the LPSPI module out of reset and clear CR register */
    base->CR = (uint32_t)0x0;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_HAL_Config
 * Description   : Configures the LPSPI registers to a user defined configuration.
 *
 * Note, the LPSPI module must first be disabled prior to calling this function. It is recommended
 * to first call the LPSPI_HAL_Init function prior to calling this function.
 * This function configures the LPSPI based on the configuration passed in by the user
 * for normal SPI mode operation. Recommend single bit transfer: txCmd.width = LPSPI_SINGLE_BIT_XFER,
 * otherwise you will have to call function LPSPI_HAL_SetPinConfigMode to change the pin config.
 * This function sets the TX and RX FIFO watermarks to 0 such that the write blocking and read
 * blocking functions can be used following the init.
 *
 *END**************************************************************************/
lpspi_status_t LPSPI_HAL_Config(LPSPI_Type * base, const lpspi_init_config_t * config,
                               lpspi_tx_cmd_config_t * txCmdCfgSet, uint32_t * actualBaudRate)
{
    uint32_t tcrPre;

    /* Set for master or slave mode */
    if (LPSPI_HAL_SetMasterSlaveMode(base, config->lpspiMode) != LPSPI_STATUS_SUCCESS)
    {
        return LPSPI_STATUS_ERROR;
    }

    if (config->lpspiMode == LPSPI_MASTER)
    {
        /* First, set the baudrate and get the prescalar value */
        *actualBaudRate = LPSPI_HAL_SetBaudRate(base, config->baudRate, config->lpspiSrcClk,
                                                &tcrPre);
          if (actualBaudRate == 0)
        {
            return LPSPI_STATUS_ERROR;
        }

        /* Populate the TCR Prescaler */
        txCmdCfgSet->preDiv = tcrPre;

    }
    /* Configure the desired PCS polarity */
    LPSPI_HAL_SetPcsPolarityMode(base, txCmdCfgSet->whichPcs, config->pcsPol);

    /* Set Pin configuration for SDO-out and SDI-in */
    LPSPI_HAL_SetPinConfigMode(base, LPSPI_SDI_IN_SDO_OUT, LPSPI_DATA_OUT_RETAINED, true);

    /* Enable LPSPI */
    BITBAND_ACCESS32(&(base->CR), LPSPI_CR_MEN_SHIFT) = (uint32_t)1U;

    /* Program the transmit command register with all of the relevant parameters */
    LPSPI_HAL_SetTxCommandReg(base, txCmdCfgSet);

    /* Set FIFO watermarks to 0 */
    LPSPI_HAL_SetFifoWatermarks(base, 0U, 0U);

    return LPSPI_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_HAL_GetVersionId
 * Description   : Gets the Major, Minor and Feature ID of the LPSPI module.
 *
 *END**************************************************************************/
void LPSPI_HAL_GetVersionId(LPSPI_Type * base, uint32_t * major, uint32_t * minor,
                            uint32_t * feature)
{
    *major = (base->VERID & LPSPI_VERID_MAJOR_MASK) >> LPSPI_VERID_MAJOR_SHIFT;
    *minor = (base->VERID & LPSPI_VERID_MINOR_MASK) >> LPSPI_VERID_MINOR_SHIFT;
    *feature = (base->VERID & LPSPI_VERID_FEATURE_MASK) >> LPSPI_VERID_FEATURE_SHIFT;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_HAL_Disable
 * Description   : Disables the LPSPI module.
 *
 * Note that this function returns LPSPI_STATUS_BUSY if it is detected that the Module Busy Flag
 * (MBF) is set, otherwise, if success, it returns LPSPI_STATUS_SUCCESS.
 *
 *END**************************************************************************/
lpspi_status_t LPSPI_HAL_Disable(LPSPI_Type * base)
{
    uint32_t lpspi_tmp = base->SR;
    lpspi_tmp = (lpspi_tmp & LPSPI_SR_MBF_MASK) >> LPSPI_SR_MBF_SHIFT;

    if (lpspi_tmp == (uint32_t)1)
    {
        return LPSPI_STATUS_BUSY;
    }
    else
    {
        BITBAND_ACCESS32(&(base->CR), LPSPI_CR_MEN_SHIFT) = (uint32_t)0U;
        return LPSPI_STATUS_SUCCESS;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_HAL_GetVersionId
 * Description   : Configures the LPSPI for master or slave.
 *
 * Note that the LPSPI module must first be disabled before configuring this.
 *
 *END**************************************************************************/
lpspi_status_t LPSPI_HAL_SetMasterSlaveMode(LPSPI_Type * base, lpspi_master_slave_mode_t mode)
{

    /* If the module is enabled, return error */
    if (true == LPSPI_HAL_IsModuleEnabled(base))
    {
        return LPSPI_STATUS_ERROR;
    }
    else
    {
        BITBAND_ACCESS32(&(base->CFGR1), LPSPI_CFGR1_MASTER_SHIFT) = (uint32_t)mode;
        return LPSPI_STATUS_SUCCESS;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_HAL_GetFifoSizes
 * Description   : Gets the TX and RX FIFO sizes of the LPSPI module.
 *
 *END**************************************************************************/
void LPSPI_HAL_GetFifoSizes(LPSPI_Type * base, uint32_t * txFifoSize,
                            uint32_t * rxFifoSize)
{
    if (txFifoSize != NULL)
    {
    	*txFifoSize = 1U << ((base->PARAM & LPSPI_PARAM_TXFIFO_MASK) >> LPSPI_PARAM_TXFIFO_SHIFT);
    }
    if (rxFifoSize != NULL)
    {
        *rxFifoSize = 1U << ((base->PARAM & LPSPI_PARAM_RXFIFO_MASK) >> LPSPI_PARAM_RXFIFO_SHIFT);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_HAL_SetPinConfigMode
 * Description   : Flushes the LPSPI FIFOs.
 *
 *END**************************************************************************/
void LPSPI_HAL_SetFlushFifoCmd(LPSPI_Type * base, bool flushTxFifo, bool flushRxFifo)
{
    uint32_t crValue = 0;

    crValue = ((uint32_t)flushTxFifo << LPSPI_CR_RTF_SHIFT) |
              ((uint32_t)flushRxFifo << LPSPI_CR_RRF_SHIFT);

    base->CR |= crValue;

}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_HAL_ClearStatusFlag
 * Description   : Clears the LPSPI status flag.
 *
 * This function clears the state of one of the LPSPI status flags as requested by
 * the user. Note, the flag must be w1c capable, if not the function returns an error.
 * w1c capable flags are:
 *   LPSPI_WORD_COMPLETE
 *   LPSPI_FRAME_COMPLETE
 *   LPSPI_TRANSFER_COMPLETE
 *   LPSPI_TRANSMIT_ERROR
 *   LPSPI_RECEIVE_ERROR
 *   LPSPI_DATA_MATCH
 *
 *END**************************************************************************/
lpspi_status_t LPSPI_HAL_ClearStatusFlag(LPSPI_Type * base, lpspi_status_flag_t statusFlag)
{
    /* If the flag is not w1c capable, return invalid parameter error */
    if ((statusFlag == LPSPI_MODULE_BUSY) ||
        (statusFlag == LPSPI_RX_DATA_FLAG) ||
        (statusFlag == LPSPI_TX_DATA_FLAG))
    {
        return LPSPI_STATUS_INVALID_PARAMETER;
    }
    else
    {
        if (statusFlag == LPSPI_ALL_STATUS)
        {
            base->SR = LPSPI_ALL_STATUS;
        }
        else
        {
            base->SR = (1U << statusFlag);
        }
        return LPSPI_STATUS_SUCCESS;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_HAL_SetHostRequestMode
 * Description   : Configures the LPSPI Host Request input.
 *
 * This function allows the user to configure the host request input pin as follows:
 *  Enable or disable the host request functionality.
 *  Select the polarity of the host request signal.
 *  Select the source of the host request (external signal or internal trigger).
 *
 *END**************************************************************************/
void LPSPI_HAL_SetHostRequestMode(LPSPI_Type * base,
                                  lpspi_host_request_select_t hostReqInput,
                                  lpspi_signal_polarity_t hostReqPol,
                                  bool enable)
{
    uint32_t cfgr0Value = 0;

    cfgr0Value = base->CFGR0 &
                 ~(LPSPI_CFGR0_HREN_MASK|LPSPI_CFGR0_HRPOL_MASK|LPSPI_CFGR0_HRSEL_MASK);

    cfgr0Value |= (uint32_t)(enable) |
                  (uint32_t)(hostReqPol << LPSPI_CFGR0_HRPOL_SHIFT) |
                  (uint32_t)(hostReqInput << LPSPI_CFGR0_HRSEL_SHIFT);

    base->CFGR0 = cfgr0Value;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_HAL_SetPcsPolarityMode
 * Description   : Configures the desired LPSPI PCS polarity.
 *
 * This function allows the user to configure the polarity of a particular PCS signal.
 * Note that the LPSPI module must first be disabled before configuring this.
 *
 *END**************************************************************************/
lpspi_status_t LPSPI_HAL_SetPcsPolarityMode(LPSPI_Type * base, lpspi_which_pcs_t whichPcs,
                                            lpspi_signal_polarity_t pcsPolarity)
{
    uint32_t cfgr1Value = 0;

    /* If the module is enabled, return error */
    if (true == LPSPI_HAL_IsModuleEnabled(base))
    {
        return LPSPI_STATUS_ERROR;
    }
    else
    {
        /* Clear the PCS polarity bit */
        cfgr1Value = base->CFGR1 & ~(1U << (LPSPI_CFGR1_PCSPOL_SHIFT + whichPcs));

        /* Configure the PCS polarity bit according to the pcsPolarity setting */
        cfgr1Value |= (uint32_t)pcsPolarity << (LPSPI_CFGR1_PCSPOL_SHIFT + whichPcs);

        base->CFGR1 = cfgr1Value;

        return LPSPI_STATUS_SUCCESS;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_HAL_SetMatchConfigMode
 * Description   : Configures the LPSPI data match configuration mode.
 *
 * When enabled and configured to the desired condition of type lpspi_match_config_t,
 * the LPSPI will assert the DMF status flag if the data match condition is met.
 * Note that the LPSPI module must first be disabled before configuring this.
 *
 *END**************************************************************************/
lpspi_status_t LPSPI_HAL_SetMatchConfigMode(LPSPI_Type * base,
                                            lpspi_match_config_t matchCondition,
                                            bool rxDataMatchOnly,
                                            uint32_t match0,
                                            uint32_t match1)
{
    uint32_t cfgr1Value = 0;

    /* If the module is enabled, return error */
    if (true == LPSPI_HAL_IsModuleEnabled(base))
    {
        return LPSPI_STATUS_ERROR;
    }
    else
    {
        cfgr1Value = base->CFGR1 & ~(LPSPI_CFGR1_MATCFG_MASK);
        cfgr1Value |= ((uint32_t)matchCondition << LPSPI_CFGR1_MATCFG_SHIFT);
        base->CFGR1 = cfgr1Value;

        BITBAND_ACCESS32(&(base->CFGR0), LPSPI_CFGR0_RDMO_SHIFT) = (uint32_t)rxDataMatchOnly;

        base->DMR0 = match0;
        base->DMR1 = match1;

        return LPSPI_STATUS_SUCCESS;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_HAL_SetPinConfigMode
 * Description   : Configures the LPSPI SDO/SDI pin configuration mode.
 *
 * This function configures the pin mode of the LPSPI.
 * For the SDI and SDO pins, the user can configure these pins as follows:
 *  SDI is used for input data and SDO for output data.
 *  SDO is used for input data and SDO for output data.
 *  SDI is used for input data and SDI for output data.
 *  SDO is used for input data and SDI for output data.
 *
 * The user has the option to configure the output data as:
 *  Output data retains last value when chip select is de-asserted (default setting).
 *  Output data is tristated when chip select is de-asserted.
 *
 * Finally, the user has the option to configure the PCS[3:2] pins as:
 *  Enabled for PCS operation (default setting).
 *  Disabled - this is need if the user wishes to configure the LPSPI mode for 4-bit transfers
 *             where these pins will be used as I/O data pins.
 *
 * Note that the LPSPI module must first be disabled before configuring this.
 *
 *END**************************************************************************/
lpspi_status_t LPSPI_HAL_SetPinConfigMode(LPSPI_Type * base,
                                          lpspi_pin_config_t pinCfg,
                                          lpspi_data_out_config_t dataOutConfig,
                                          bool pcs3and2Enable)
{
    uint32_t cfgr1Value = 0;

    /* If the module is enabled, return error */
    if (true == LPSPI_HAL_IsModuleEnabled(base))
    {
        return LPSPI_STATUS_ERROR;
    }
    else
    {
        cfgr1Value = base->CFGR1 &
                     ~(LPSPI_CFGR1_PINCFG_MASK|LPSPI_CFGR1_OUTCFG_MASK|LPSPI_CFGR1_PCSCFG_MASK);

        cfgr1Value |= (uint32_t)(pinCfg << LPSPI_CFGR1_PINCFG_SHIFT) |
                      (uint32_t)(dataOutConfig << LPSPI_CFGR1_OUTCFG_SHIFT) |
                      (uint32_t)((!pcs3and2Enable) << LPSPI_CFGR1_PCSCFG_SHIFT);  /* enable = 0 */

        base->CFGR1 = cfgr1Value;

        return LPSPI_STATUS_SUCCESS;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_HAL_SetBaudRate
 * Description   : Sets the LPSPI baud rate in bits per second.
 *
 * This function takes in the desired bitsPerSec (baud rate) and calculates the nearest
 * possible baud rate without exceeding the desired baud rate, and returns the
 * calculated baud rate in bits-per-second. It requires that the caller also provide
 * the frequency of the module source clock (in Hertz). Also note that the baud rate
 * does not take into affect until the Transmit Control Register (TCR) is programmed
 * with the PRESCALE value. Hence, this function returns the PRESCALE tcrPrescaleValue
 * parameter for later programming in the TCR.  It is up to the higher level
 * peripheral driver to alert the user of an out of range baud rate input.
 *
 * Note that the LPSPI module must first be disabled before configuring this.
 * Note that the LPSPI module must be configure for master mode before configuring this.
 *
 *END**************************************************************************/
uint32_t LPSPI_HAL_SetBaudRate(LPSPI_Type * base, uint32_t bitsPerSec,
                               uint32_t sourceClockInHz, uint32_t * tcrPrescaleValue)
{
	uint32_t lpspi_tmp;

	/* For master mode configuration only, if slave mode detected, return 0.
     * Also, the LPSPI module needs to be disabled first, if enabled, return 0
     */
    if ((LPSPI_HAL_IsMaster(base) != true) || (true == LPSPI_HAL_IsModuleEnabled(base)))
    {
        return 0;
    }

    uint32_t prescaler, bestPrescaler;
    uint32_t scaler, bestScaler;
    uint32_t realBaudrate, bestBaudrate;
    uint32_t diff, min_diff;
    uint32_t desiredBaudrate = bitsPerSec;

    /* find combination of prescaler and scaler resulting in baudrate closest to the
     * requested value
     */
    min_diff = 0xFFFFFFFFU;

    /* Set to maximum divisor value bit settings so that if baud rate passed in is less
     * than the minimum possible baud rate, then the SPI will be configured to the lowest
     * possible baud rate
     */
    bestPrescaler = 7;
    bestScaler = 255;

    bestBaudrate = 0; /* required to avoid compilation warning */

    /* In all for loops, if min_diff = 0, the exit for loop*/
    for (prescaler = 0; (prescaler < 8) && min_diff; prescaler++)
    {
        for (scaler = 0; (scaler < 256) && min_diff; scaler++)
        {
            realBaudrate = (sourceClockInHz /
                            (s_baudratePrescaler[prescaler] * (scaler + 2U)));

            /* calculate the baud rate difference based on the conditional statement
             * that states that the calculated baud rate must not exceed the desired baud rate
             */
            if (desiredBaudrate >= realBaudrate)
            {
                diff = desiredBaudrate - realBaudrate;
                if (min_diff > diff)
                {
                    /* a better match found */
                    min_diff = diff;
                    bestPrescaler = prescaler;
                    bestScaler = scaler;
                    bestBaudrate = realBaudrate;
                }
            }
        }
    }

    /* Write the best baud rate scalar to the CCR.
     * Note, no need to check for error since we've already checked to make sure the module is
     * disabled and in master mode. Also, there is a limit on the maximum divider so we will not
     * exceed this.
     */
    lpspi_tmp = base->CCR;
    lpspi_tmp &= ~(LPSPI_CCR_SCKDIV_MASK);
    lpspi_tmp |= LPSPI_CCR_SCKDIV(bestScaler);
    base->CCR = lpspi_tmp;


    /* return the best prescaler value for user to use later */
    *tcrPrescaleValue = bestPrescaler;

    /* return the actual calculated baud rate */
    return bestBaudrate;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_HAL_SetBaudRateDivisor
 * Description   : Configures the baud rate divisor manually (only the LPSPI_CCR[SCKDIV]).
 *
 * This function allows the caller to manually set the baud rate divisor in the event
 * that this divider is known and the caller does not wish to call the
 * LPSPI_HAL_SetBaudRate function. Note that this only affects the LPSPI_CCR[SCKDIV]).
 * The Transmit Control Register (TCR) is programmed separately with the PRESCALE value.
 * The valid range is 0x00 to 0xFF (255), if the user inputs outside of this range, an error
 * is returned.
 *
 * Note that the LPSPI module must first be disabled before configuring this.
 * Note that the LPSPI module must be configure for master mode before configuring this.
 *
 *END**************************************************************************/
lpspi_status_t LPSPI_HAL_SetBaudRateDivisor(LPSPI_Type * base, uint32_t divisor)
{
	uint32_t lpspi_tmp;


    /* LPSPI must first be disabled before setting the SCKDIV value and set to master mode */
    if ((true == LPSPI_HAL_IsModuleEnabled(base)) || (LPSPI_HAL_IsMaster(base) != true))
    {
        return LPSPI_STATUS_ERROR;
    }
    else if (divisor > 255)
    {
        return LPSPI_STATUS_OUT_OF_RANGE;
    }
    else
    {
        lpspi_tmp = base->CCR;
        lpspi_tmp &= ~(LPSPI_CCR_SCKDIV_MASK);
        lpspi_tmp |= LPSPI_CCR_SCKDIV(divisor);
        base->CCR = lpspi_tmp;

        return LPSPI_STATUS_SUCCESS;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_HAL_SetDelay
 * Description   : Manually configures a specific LPSPI delay parameter (module must be disabled to
 *                 change the delay values).
 *
 * This function configures the:
 * SCK to PCS delay, or
 * PCS to SCK delay, or
 * Between transfer delay.
 *
 * These delay names are available in type lpspi_delay_type_t.
 *
 * The user passes which delay they want to configure along with the delay value.
 * This allows the user to directly set the delay values if they have
 * pre-calculated them or if they simply wish to manually increment the value.
 *
 * Note that the LPSPI module must first be disabled before configuring this.
 * Note that the LPSPI module must be configure for master mode before configuring this.
 *
 *END**************************************************************************/
lpspi_status_t LPSPI_HAL_SetDelay(LPSPI_Type * base, lpspi_delay_type_t whichDelay, uint32_t delay)
{
    uint32_t ccrValue = 0;

    /* LPSPI must first be disabled before setting the delay value and set to master mode */
    if ((true == LPSPI_HAL_IsModuleEnabled(base)) || (LPSPI_HAL_IsMaster(base) != true))
    {
        return LPSPI_STATUS_ERROR;
    }
    else if (delay > 255)
    {
        return LPSPI_STATUS_OUT_OF_RANGE;
    }
    else
    {
        ccrValue = base->CCR & ~(0xFFU << whichDelay);
        ccrValue |= delay << whichDelay;
        base->CCR = ccrValue;
        return LPSPI_STATUS_SUCCESS;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_HAL_CalculateDelay
 * Description   : Calculates the delay based on the desired delay input in nanoseconds
 *                 (module must be disabled to change the delay values).
 *
 * This function calculates the values for:
 * SCK to PCS delay, or
 * PCS to SCK delay, or
 * Between transfer delay.
 *
 * These delay names are available in type lpspi_delay_type_t.
 *
 * The user passes which delay they want to configure along with the desired delay value in
 * nano-seconds.  The function calculates the value needed for the desired delay parameter
 * and returns the actual calculated delay as an exact delay match may not be possible. In this
 * case, the closest match is calculated without going below the desired delay value input.
 * It is possible to input a very large delay value that exceeds the capability of the part, in
 * which case the maximum supported delay will be returned. It is up to the higher level
 * peripheral driver to alert the user of an out of range delay input.
 *
 * Note that the LPSPI module must first be disabled before configuring this.
 * Note that the LPSPI module must be configure for master mode before configuring this.
 *
 *END**************************************************************************/
lpspi_status_t LPSPI_HAL_CalculateDelay(LPSPI_Type * base,
                                        lpspi_delay_type_t whichDelay,
                                        uint32_t sourceClockInHz,
                                        uint32_t delayInNanoSec,
                                        uint32_t * actualDelay)
{
    lpspi_status_t returnStatus = LPSPI_STATUS_SUCCESS;

    /* LPSPI must first be disabled before setting the delay value and set to master mode */
    if ((true == LPSPI_HAL_IsModuleEnabled(base)) || (LPSPI_HAL_IsMaster(base) != true))
    {
        return LPSPI_STATUS_ERROR;
    }

    uint64_t realDelay, bestDelay;
    uint32_t scaler, bestScaler;
    uint32_t diff, min_diff;
    uint64_t initialDelayNanoSec;

    /* For delay between transfer, an additional scaler value is needed */
    uint32_t additionalScaler = 0;

    /* find combination of prescaler and scaler resulting in the delay closest to the
     * requested value
     */
    min_diff = 0xFFFFFFFFU;

    /* Initialize scaler to max value to generate the max delay */
    bestScaler = 0xFFU;

    /* Calculate the initial (min) delay and maximum possible delay based on the specific delay as
     * the delay divisors are slightly different based on which delay we are configuring.
     */
    if (whichDelay == LPSPI_BETWEEN_TRANSFER)
    {
        /* First calculate the initial, default delay, note min delay is 2 clock cycles
           Due to large size of calculated values (uint64_t), we need to break up the
           calculation into several steps to ensure accurate calculated results
         */
        initialDelayNanoSec = 1000000000U;
        initialDelayNanoSec *= 2U;
        initialDelayNanoSec /= sourceClockInHz;

        /* Calculate the maximum delay */
        bestDelay = 1000000000U;
        bestDelay *= 257U; /* based on DBT+2, or 255 + 2 */
        bestDelay /= sourceClockInHz;

        additionalScaler = 1U;
    }
    else
    {
        /* First calculate the initial, default delay, min delay is 1 clock cycle
           Due to large size of calculated values (uint64_t), we need to break up the
           calculation into several steps to ensure accurate calculated results
         */
        initialDelayNanoSec = 1000000000U;
        initialDelayNanoSec /= sourceClockInHz;


        /* Calculate the maximum delay */
        bestDelay = 1000000000U;
        bestDelay *= 256U; /* based on SCKPCS+1 or PCSSCK+1, or 255 + 1 */
        bestDelay /= sourceClockInHz;

        additionalScaler = 0;
    }

    /* If the initial, default delay is already greater than the desired delay, then
     * set the delay to their initial value (0) and return the delay. In other words,
     * there is no way to decrease the delay value further.
     */
    if (initialDelayNanoSec >= delayInNanoSec)
    {
        LPSPI_HAL_SetDelay(base, whichDelay, 0);
        *actualDelay = initialDelayNanoSec;
        return returnStatus;
    }

    /* If min_diff = 0, the exit for loop */
    for (scaler = 0; (scaler < 256U) && min_diff; scaler++)
    {
        /* Calculate the real delay value as we cycle through the scaler values.
           Due to large size of calculated values (uint64_t), we need to break up the
           calculation into several steps to ensure accurate calculated results
         */
        realDelay = 1000000000U;
        realDelay *= (scaler + 1 + additionalScaler);
        realDelay /= sourceClockInHz;

        /* calculate the delay difference based on the conditional statement
         * that states that the calculated delay must not be less then the desired delay
         */
        if (realDelay >= delayInNanoSec)
        {
            diff = realDelay-delayInNanoSec;
            if (min_diff > diff)
            {
                /* a better match found */
                min_diff = diff;
                bestScaler = scaler;
                bestDelay = realDelay;
            }
        }
    }

    /* write the best scaler value for the delay */
    returnStatus = LPSPI_HAL_SetDelay(base, whichDelay, bestScaler);

    /* return the actual calculated delay value (in ns) */
    *actualDelay = (uint32_t)bestDelay;

    return returnStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_HAL_SetTxCommandReg
 * Description   : Sets the Transmit Command Register (TCR) parameters.
 *
 * The Transmit Command Register (TCR) contains multiple parameters that affect
 * the transmission of data, such as clock phase and polarity, which PCS to use,
 * whether or not the PCS remains asserted at the completion of a frame, etc.
 * Any writes to this register results in an immediate push of the entire register
 * and its contents to the TX FIFO.  Hence, writes to this register should include
 * all of the desired parameters written to the register at once. Hence, the user
 * should fill in the members of the lpspi_tx_cmd_config_t data structure and pass
 * this to the function.
 *
 *END**************************************************************************/
void LPSPI_HAL_SetTxCommandReg(LPSPI_Type * base, lpspi_tx_cmd_config_t * txCmdCfgSet)
{
     base->TCR = ((uint32_t)txCmdCfgSet->clkPolarity << LPSPI_TCR_CPOL_SHIFT |
                         (uint32_t)txCmdCfgSet->clkPhase << LPSPI_TCR_CPHA_SHIFT |
                         (uint32_t)txCmdCfgSet->preDiv << LPSPI_TCR_PRESCALE_SHIFT |
                         (uint32_t)txCmdCfgSet->whichPcs << LPSPI_TCR_PCS_SHIFT |
                         (uint32_t)txCmdCfgSet->lsbFirst << LPSPI_TCR_LSBF_SHIFT |
                         (uint32_t)txCmdCfgSet->byteSwap<< LPSPI_TCR_BYSW_SHIFT |
                         (uint32_t)txCmdCfgSet->contTransfer << LPSPI_TCR_CONT_SHIFT |
                         (uint32_t)txCmdCfgSet->contCmd << LPSPI_TCR_CONTC_SHIFT |
                         (uint32_t)txCmdCfgSet->rxMask << LPSPI_TCR_RXMSK_SHIFT |
                         (uint32_t)txCmdCfgSet->txMask << LPSPI_TCR_TXMSK_SHIFT |
                         (uint32_t)txCmdCfgSet->width << LPSPI_TCR_WIDTH_SHIFT |
                         (uint32_t)(txCmdCfgSet->frameSize - 1) << LPSPI_TCR_FRAMESZ_SHIFT);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_HAL_WriteDataBlocking
 * Description   : Writes a data into the TX data buffer and waits till complete to return.
 *
 * This function writes the data to the Transmit Data Register (TDR) and waits for completion
 * before returning. If the frame size exceeds 32-bits, the user will have to manage sending
 * the data one 32-bit word at a time.
 * This function can be used for either master or slave mode.
 * Note that it is required that the TX FIFO watermark be set to 0.
 *
 *END**************************************************************************/
void LPSPI_HAL_WriteDataBlocking(LPSPI_Type * base, uint32_t data)
{
    /* Wait until the transmit data is requested */
    while(!LPSPI_HAL_GetStatusFlag(base, LPSPI_TX_DATA_FLAG)) { }

    base->TDR = data;

    /* Wait until the transmit data is requested */
    while(!LPSPI_HAL_GetStatusFlag(base, LPSPI_TX_DATA_FLAG)) { }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : LPSPI_HAL_ReadDataBlocking
 * Description   : Reads data from the data buffer but first waits till data is ready.
 *
 * This function reads the data from the Receive Data Register (RDR).
 * However, before reading the data, it first waits till the read data ready status
 * indicates the data is ready to be read.
 * This function can be used for either master or slave mode.
 * Note that it is required that the RX FIFO watermark be set to 0.
 *
 *END**************************************************************************/
uint32_t LPSPI_HAL_ReadDataBlocking(LPSPI_Type * base)
{
    /* Wait for Receive Data Flag to indicate receive data is ready */
    while(!LPSPI_HAL_GetStatusFlag(base, LPSPI_RX_DATA_FLAG)) { }

    /* Now read the received data  */
    return base->RDR;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
