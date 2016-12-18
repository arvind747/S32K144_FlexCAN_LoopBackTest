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

#include "fsl_clock_S32K144.h"
#include "fsl_clock_manager.h"
#include "fsl_device_registers.h"
/*
 * README:
 * This file should provide these APIs:
 * 1. APIs to get the frequency of output clocks in Reference Manual ->
 * Chapter Clock Distribution -> Figure Clocking diagram.
 * 2. APIs for IP modules listed in Reference Manual -> Chapter Clock Distribution
 * -> Module clocks.
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* This frequency values should be set by different boards. */
/* SIM */
uint32_t g_TClkFreq[NUMBER_OF_TCLK_INPUTS];      /* TCLKx clocks    */

/* RTC */
uint32_t g_RtcClkInFreq;                         /* RTC CLKIN clock */

/* SCG */
uint32_t g_xtal0ClkFreq;                         /* EXTAL0 clock    */

/*******************************************************************************
 * PROTOTYPES
 ******************************************************************************/

static clock_manager_error_code_t CLOCK_SYS_GetScgClockFreq(clock_names_t clockName, uint32_t * frequency);
static clock_manager_error_code_t CLOCK_SYS_GetSimClockFreq(clock_names_t clockName, uint32_t * frequency);
static clock_manager_error_code_t CLOCK_SYS_GetPccClockFreq(clock_names_t clockName, uint32_t * frequency);
static uint32_t CLOCK_SYS_GetPeripheralClock(clock_names_t clockName, scg_async_clock_type_t divider);
static scg_system_clock_mode_t CLOCK_SYS_GetCurrentRunMode(SMC_Type * smc_base);
static bool CLOCK_SYS_SwitchSystemClock(scg_system_clock_src_t to_clk);

/*******************************************************************************
 * Code
 ******************************************************************************/

 
/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_SetConfiguration
 * Description   : This function sets the system to target configuration, it
 * only sets the clock modules registers for clock mode change, but not send
 * notifications to drivers.
 *
 *END**************************************************************************/
clock_manager_error_code_t CLOCK_SYS_SetConfiguration(clock_manager_user_config_t const* config)
{
    clock_manager_error_code_t result;
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(config);
#endif

    /* Set SCG settings. */
    if(STATUS_SCG_SUCCESS != CLOCK_SYS_SetScgConfiguration(&config->scgConfig)){
        result = CLOCK_MANAGER_INVALID_PARAM;
    } else {
        result = CLOCK_MANAGER_SUCCESS;
    }

    /* Set PCC settings. */
    CLOCK_SYS_SetPccConfiguration(&config->pccConfig);

    /* Set SIM settings. */
    CLOCK_SYS_SetSimConfiguration(&config->simConfig);

    return result;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_SetScgConfiguration
 * Description   : This function configures the SCG blocks
 *
 *END**************************************************************************/
scg_status_t CLOCK_SYS_SetScgConfiguration(const scg_config_t *scgConfig)
{
    scg_status_t status;
    uint32_t timeout;
    scg_system_clock_config_t current_config;
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(scgConfig);
#endif

    status = STATUS_SCG_SUCCESS;

    if (scgConfig){

        /* Get CURRENT mode configuration */
        SCG_HAL_GetSystemClockConfig(SCG, SCG_SYSTEM_CLOCK_MODE_CURRENT, &current_config);

        /* Configure SIRC. */
        if (scgConfig->sircConfig.initialize) {

            /* Check and switch to alternate */
            if ( current_config.src == SCG_SYSTEM_CLOCK_SRC_SIRC ) {
                CLOCK_SYS_SwitchSystemClock(scgConfig->clockModeConfig.alternateClock);
            }

            /* Disable the SIRC. */
            status |= SCG_HAL_DeinitSirc(SCG);

            /* Setup SIRC. */
            status |= SCG_HAL_InitSirc(SCG, &(scgConfig->sircConfig));

            /* Wait for SIRC to initialize - typical 6us*/
            /* f core = max 112mhz=>9ns/cycle, ~26 cycles per loop=> 26 loops*/
            timeout = 26;
            while((SCG_HAL_GetSircFreq(SCG) == 0) && (timeout > 0)){
                timeout--;
            }

            /* Revert from alternate */
            if ( current_config.src == SCG_SYSTEM_CLOCK_SRC_SIRC ) {
                status |= CLOCK_SYS_SwitchSystemClock( SCG_SYSTEM_CLOCK_SRC_SIRC ) ?
                          STATUS_SCG_SUCCESS : STATUS_SCG_FAILED;
            }

            #ifdef DEV_ERROR_DETECT
                DEV_ASSERT(STATUS_SCG_SUCCESS == status);
            #endif
        }

        /* Configure FIRC. */
        if (scgConfig->fircConfig.initialize) {

            /* Check and switch to alternate */
            if ( current_config.src == SCG_SYSTEM_CLOCK_SRC_FIRC ) {
                CLOCK_SYS_SwitchSystemClock(scgConfig->clockModeConfig.alternateClock);
            }

            /* Disable the FIRC. */
            status |= SCG_HAL_DeinitFirc(SCG);

            /* Setup FIRC. */
            status |= SCG_HAL_InitFirc(SCG, &(scgConfig->fircConfig));

            /* Revert from alternate */
            if ( current_config.src == SCG_SYSTEM_CLOCK_SRC_FIRC ) {
                status |= CLOCK_SYS_SwitchSystemClock( SCG_SYSTEM_CLOCK_SRC_FIRC ) ?
                          STATUS_SCG_SUCCESS : STATUS_SCG_FAILED;
            }

            #ifdef DEV_ERROR_DETECT
                DEV_ASSERT(STATUS_SCG_SUCCESS == status);
            #endif
        }

        /* Configure SOSC. */
        if (scgConfig->soscConfig.initialize) {

            /* Check and switch to alternate */
            if ( current_config.src == SCG_SYSTEM_CLOCK_SRC_SYS_OSC ) {
                CLOCK_SYS_SwitchSystemClock(scgConfig->clockModeConfig.alternateClock);
            }

            /* Disable the SOSC. */
            status |= SCG_HAL_DeinitSysOsc(SCG);

            /* Setup System OSC. */
            status |= SCG_HAL_InitSysOsc(SCG, &(scgConfig->soscConfig));

            /* Wait for System OSC to initialize - max 750ms*/
            /* f core = max 112mhz=>9ns/cycle, ~26 cycles per loop=> 3205000 loops*/
            timeout = 3205000;
            while((SCG_HAL_GetSysOscFreq(SCG) == 0) && (timeout > 0)){
                timeout--;
            }

            /* Revert from alternate */
            if ( current_config.src == SCG_SYSTEM_CLOCK_SRC_SYS_OSC ) {
                status |= CLOCK_SYS_SwitchSystemClock( SCG_SYSTEM_CLOCK_SRC_SYS_OSC ) ?
                          STATUS_SCG_SUCCESS : STATUS_SCG_FAILED;
            }

            #ifdef DEV_ERROR_DETECT
                DEV_ASSERT(STATUS_SCG_SUCCESS == status);
            #endif
        }

        /* Configure SPLL. */
        if (scgConfig->spllConfig.initialize) {

            /* Check and switch to alternate */
            if ( current_config.src == SCG_SYSTEM_CLOCK_SRC_SYS_PLL ) {
                CLOCK_SYS_SwitchSystemClock(scgConfig->clockModeConfig.alternateClock);
            }

            /* Disable the SPLL. */
            status |= SCG_HAL_DeinitSysPll(SCG);

            /* Setup SPLL. */
            status |= SCG_HAL_InitSysPll(SCG, &(scgConfig->spllConfig));

            /* Revert from alternate */
            if ( current_config.src == SCG_SYSTEM_CLOCK_SRC_SYS_PLL ) {
                status |= CLOCK_SYS_SwitchSystemClock( SCG_SYSTEM_CLOCK_SRC_SYS_PLL ) ?
                          STATUS_SCG_SUCCESS : STATUS_SCG_FAILED;
            }

            #ifdef DEV_ERROR_DETECT
                DEV_ASSERT(STATUS_SCG_SUCCESS == status);
            #endif
        }

        /* Configure RTC. */
        if (scgConfig->rtcConfig.initialize) {
            /* RTC Clock settings. */
            SCG_HAL_SetRtcClkInFreq(SCG, scgConfig->rtcConfig.rtcClkInFreq);
        }

        /* Configure SCG clock modes. */
        if (scgConfig->clockModeConfig.initialize) {
            /* Configure SCG clock modes */
            SCG_HAL_SetSystemClockConfig(SCG, SCG_SYSTEM_CLOCK_MODE_RUN,   &(scgConfig->clockModeConfig.rccrConfig));
            SCG_HAL_SetSystemClockConfig(SCG, SCG_SYSTEM_CLOCK_MODE_VLPR,  &(scgConfig->clockModeConfig.vccrConfig));
            SCG_HAL_SetSystemClockConfig(SCG, SCG_SYSTEM_CLOCK_MODE_HSRUN, &(scgConfig->clockModeConfig.hccrConfig));
        }

    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_SetPccConfiguration
 * Description   : This function configures the PCC block
 *
 *END**************************************************************************/
void CLOCK_SYS_SetPccConfiguration(const pcc_config_t *peripheralClockConfig)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(peripheralClockConfig);
#endif
    PCC_HAL_SetPeripheralClockConfig(PCC, peripheralClockConfig);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_SetSimConfiguration
 * Description   : This function configures the PCC block
 *
 *END**************************************************************************/
void CLOCK_SYS_SetSimConfiguration(const sim_clock_config_t *simClockConfig)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(simClockConfig);
#endif
    uint8_t i;

    /* ClockOut settings. */
    if (simClockConfig->clockOutConfig.initialize){
        SIM_HAL_SetClkOutSel(SIM, simClockConfig->clockOutConfig.source);
    }

    /* Low Power Clock settings. */
    if (simClockConfig->lpoClockConfig.initialize){
        SIM_HAL_SetLpoClocks(SIM, simClockConfig->lpoClockConfig);
    }

    /* Platform Gate Clock settings. */
    if (simClockConfig->platGateConfig.initialize){
        SIM_HAL_SetMscmClockGate(SIM, simClockConfig->platGateConfig.enableMscm);
        SIM_HAL_SetMpuClockGate(SIM,  simClockConfig->platGateConfig.enableMpu);
        SIM_HAL_SetDmaClockGate(SIM,  simClockConfig->platGateConfig.enableDma);
    }

    /* TCLK Clock settings. */
    if (simClockConfig->tclkConfig.initialize){
        for( i = 0; i< NUMBER_OF_TCLK_INPUTS; i++) {
            SIM_HAL_SetTClkFreq(SIM, i, simClockConfig->tclkConfig.tclkFreq[i]);
        }
    }

    /* Debug trace Clock settings. */
    if (simClockConfig->traceClockConfig.initialize) {
        SIM_HAL_InitTraceClock(SIM, &(simClockConfig->traceClockConfig));
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_GetScgClockFreq
 * Description   : This function returns the frequency of a given clock from SCG
 *
 *END**************************************************************************/
static clock_manager_error_code_t CLOCK_SYS_GetScgClockFreq(clock_names_t clockName,
                                                            uint32_t *frequency)
{
    clock_manager_error_code_t returnCode = CLOCK_MANAGER_SUCCESS;
    *frequency = 0U;

    switch (clockName)
    {
        /* Main clocks */
        case CORE_CLOCK:
            *frequency = SCG_HAL_GetSystemClockFreq(SCG,SCG_SYSTEM_CLOCK_CORE);
            break;
        case PLATFORM_CLOCK:
            *frequency = SCG_HAL_GetSystemClockFreq(SCG,SCG_SYSTEM_CLOCK_PLAT);
            break;
        case BUS_CLOCK:
            *frequency = SCG_HAL_GetSystemClockFreq(SCG,SCG_SYSTEM_CLOCK_BUS);
            break;
        case SLOW_CLOCK:
            *frequency = SCG_HAL_GetSystemClockFreq(SCG,SCG_SYSTEM_CLOCK_SLOW);
            break;

        /* Other internal clocks used by peripherals. */
        case SIRC_CLOCK:
            *frequency = SCG_HAL_GetSircFreq(SCG);
            break;
        case FIRC_CLOCK:
            *frequency = SCG_HAL_GetFircFreq(SCG);
            break;
        case SOSC_CLOCK:
            *frequency = SCG_HAL_GetSysOscFreq(SCG);
            break;
        case SPLL_CLOCK:
            *frequency = SCG_HAL_GetSysPllFreq(SCG);
            break;
        case RTC_CLKIN_CLOCK:
            *frequency = SCG_HAL_GetRtcClkInFreq(SCG);
            break;
        default:
            returnCode = CLOCK_MANAGER_NO_SUCH_CLOCK_NAME;
            break;
    }
    return returnCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_GetSimClockFreq
 * Description   : This function returns the frequency of a given clock from SIM
 *
 *END**************************************************************************/
static clock_manager_error_code_t CLOCK_SYS_GetSimClockFreq(clock_names_t clockName,
                                                            uint32_t *frequency)
{
    clock_manager_error_code_t returnCode = CLOCK_MANAGER_SUCCESS;
    *frequency = 0U;

    switch (clockName)
    {
        /* SIM clocks */
        case SIM_FTM0_CLOCKSEL:
            *frequency = SIM_HAL_GetTClkFreq(SIM,(uint8_t)SIM_HAL_GetFtmExternalClkPinMode(SIM, 0U));
            break;
        case SIM_FTM1_CLOCKSEL:
            *frequency = SIM_HAL_GetTClkFreq(SIM,(uint8_t)SIM_HAL_GetFtmExternalClkPinMode(SIM, 1U));
            break;
        case SIM_FTM2_CLOCKSEL:
            *frequency = SIM_HAL_GetTClkFreq(SIM,(uint8_t)SIM_HAL_GetFtmExternalClkPinMode(SIM, 2U));
            break;
        case SIM_FTM3_CLOCKSEL:
            *frequency = SIM_HAL_GetTClkFreq(SIM,(uint8_t)SIM_HAL_GetFtmExternalClkPinMode(SIM, 3U));
            break;
        case SIM_CLKOUTSELL:
            /* Check CLKOUT Select */
            switch (SIM_HAL_GetClkOutSel(SIM))
            {
                case SIM_CLKOUT_SEL_SYSTEM_OSC_CLK:
                    *frequency = SCG_HAL_GetSysOscFreq(SCG);
                    break;
                case SIM_CLKOUT_SEL_LPO_CLK:
                    *frequency = SIM_HAL_GetLpoFreq(SIM);
                    break;
                default:
                    /* Invalid CLKOUT selection.*/
                    returnCode = CLOCK_MANAGER_ERROR;
                    break;
            }
            break;

        case SIM_CLK32K_CLOCK:
            /* Check CLK32KSEL Select */
            switch (SIM_HAL_GetClock32kSrc(SIM))
            {
                case SIM_OSC32K_SEL_SIRCDIV2_CLK:
                    *frequency = SCG_HAL_GetSircAsyncFreq(SCG,SCG_ASYNC_CLOCK_DIV2);
                    break;
                case SIM_OSC32K_SEL_LPO_32K:
                    *frequency = SIM_HAL_GetLpo32KFreq(SIM);
                    break;
                case SIM_OSC32K_SEL_RTC_CLKIN:
                    *frequency = SCG_HAL_GetRtcClkInFreq(SCG);
                    break;
                case SIM_OSC32K_SEL_FIRCDIV2_CLK:
                    *frequency = SCG_HAL_GetFircAsyncFreq(SCG,SCG_ASYNC_CLOCK_DIV2);
                    break;
                default:
                    /* Invalid CLK32KSEL selection.*/
                    returnCode = CLOCK_MANAGER_ERROR;
                    break;
            }
            break;
        case SIM_LPO_CLOCK:
            *frequency = SIM_HAL_GetLpoFreq(SIM);
            break;
        case SIM_LPO_1K_CLOCK:
            *frequency = SIM_HAL_GetLpo1KFreq(SIM);
            break;
        case SIM_LPO_32K_CLOCK:
            *frequency = SIM_HAL_GetLpo32KFreq(SIM);
            break;
        default:
            returnCode = CLOCK_MANAGER_NO_SUCH_CLOCK_NAME;
            break;
    }
    return returnCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_GetPccClockFreq
 * Description   : This function returns the clock frequency of peripheral functional clock 
 *                 where applicable, otherwise the clock frequency of peripheral interface clock.
 *END**************************************************************************/
static clock_manager_error_code_t CLOCK_SYS_GetPccClockFreq(clock_names_t clockName,
                                                            uint32_t *frequency)
{
    clock_manager_error_code_t returnCode = CLOCK_MANAGER_SUCCESS;
    *frequency = 0U;

    /* Invalid PCC clock names */
    if (clockName <= SIM_END_OF_CLOCKS ||
        clockName == PCC_END_OF_BUS_CLOCKS ||
        clockName == PCC_END_OF_SYS_CLOCKS ||
        clockName == PCC_END_OF_ASYNCH_DIV1_CLOCKS ||
        clockName == PCC_END_OF_ASYNCH_DIV2_CLOCKS)
    {
        returnCode = CLOCK_MANAGER_NO_SUCH_CLOCK_NAME;
    }
    /* PCC clocks that are provided from BUS_CLK. 
     * These are peripheral interface clocks. */
    else if (clockName < PCC_END_OF_BUS_CLOCKS)
    {
        if (PCC_HAL_GetClockMode(PCC,clockName))  {
            *frequency = SCG_HAL_GetSystemClockFreq(SCG,SCG_SYSTEM_CLOCK_BUS);
        }
    }
    /* PCC clocks that are provided from SYS_CLK
     * These are peripheral interface clocks. */
    else if (clockName < PCC_END_OF_SYS_CLOCKS)
    {
        if (PCC_HAL_GetClockMode(PCC,clockName))  {
            *frequency = SCG_HAL_GetSystemClockFreq(SCG,SCG_SYSTEM_CLOCK_CORE);
        }
    }
    /* PCC clocks that are provided from asynchronous clocks 1th divider
     * These are peripheral functional clocks. */
    else if (clockName < PCC_END_OF_ASYNCH_DIV1_CLOCKS)
    {
        *frequency = CLOCK_SYS_GetPeripheralClock(clockName,SCG_ASYNC_CLOCK_DIV1);
    }
    /* PCC clocks that are provided from asynchronous clocks 2th divider
     * These are peripheral functional clocks. */
    else if (clockName < PCC_END_OF_ASYNCH_DIV2_CLOCKS)
    {
        *frequency = CLOCK_SYS_GetPeripheralClock(clockName,SCG_ASYNC_CLOCK_DIV2);
    }
    /* Invalid PCC clock names */
    else
    {
        returnCode = CLOCK_MANAGER_NO_SUCH_CLOCK_NAME;
    }
    return returnCode;
}




/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_SYS_GetFreq
 * Description   : This function returns the frequency of a given clock
 *
 *END**************************************************************************/
clock_manager_error_code_t CLOCK_SYS_GetFreq(clock_names_t clockName,
                                             uint32_t *frequency)
{
    clock_manager_error_code_t returnCode;

    /* Frequency of the clock name from SCG */
    if (clockName < SCG_END_OF_CLOCKS)
    {
        returnCode = CLOCK_SYS_GetScgClockFreq(clockName, frequency);
    }
    /* Frequency of the clock name from SIM */
    else if (clockName < SIM_END_OF_CLOCKS)
    {
        returnCode = CLOCK_SYS_GetSimClockFreq(clockName, frequency);
    }
    /* Frequency of the clock name from PCC */
    else if (clockName < PCC_END_OF_CLOCKS)
    {
        returnCode = CLOCK_SYS_GetPccClockFreq(clockName, frequency);
    }
    /* Invalid clock name */
    else
    {
        returnCode = CLOCK_MANAGER_NO_SUCH_CLOCK_NAME;
    }
    return returnCode;
}

/*FUNCTION**********************************************************************
 * Function Name : CLOCK_SYS_GetPeripheralClock
 * Description   : Internal function used by CLOCK_SYS_GetFreq function
 *END**************************************************************************/

static uint32_t CLOCK_SYS_GetPeripheralClock(clock_names_t clockName, scg_async_clock_type_t divider)
{
    uint32_t frequency = 0;
    peripheral_clock_frac_t  fracValue = PCC_HAL_GetFracValueSel(PCC,clockName);
    peripheral_clock_divider_t divValue = PCC_HAL_GetDividerSel(PCC,clockName);

    /* Check division factor */
    if (((uint32_t)fracValue) <= ((uint32_t)divValue))
    {
        /* Check clock gate */
        if (PCC_HAL_GetClockMode(PCC,clockName))
        {
            /* Check clock source */
            switch (PCC_HAL_GetClockSourceSel(PCC,clockName))
            {
                case CLK_SRC_SOSC:
                    frequency = SCG_HAL_GetSysOscAsyncFreq(SCG,divider);
                    break;
                case CLK_SRC_SIRC:
                    frequency = SCG_HAL_GetSircAsyncFreq(SCG,divider);
                    break;
                case CLK_SRC_FIRC:
                    frequency = SCG_HAL_GetFircAsyncFreq(SCG,divider);
                    break;
                case CLK_SRC_SPLL:
                    frequency = SCG_HAL_GetSysPllAsyncFreq(SCG,divider);
                    break;
                default:
                    frequency = 0;
            }      
            frequency = frequency / (divValue+1);
            frequency = frequency * (fracValue+1);
        }
    }
    return frequency;
}

/*FUNCTION**********************************************************************
 * Function Name : CLOCK_SYS_GetCurrentRunMode
 * Description   : Internal function used by CLOCK_SYS_SetScgConfiguration function
 *END**************************************************************************/
static scg_system_clock_mode_t CLOCK_SYS_GetCurrentRunMode(SMC_Type * smc_base) {
    scg_system_clock_mode_t mode;

    /* Read and convert from SMC run mode to SCG defines*/
    switch (SMC_HAL_GetPowerModeStatus(smc_base))
    {
        /* High speed run mode */
        case STAT_HSRUN:
            mode = SCG_SYSTEM_CLOCK_MODE_HSRUN;
            break;
        /* Run mode */
        case STAT_RUN:
            mode = SCG_SYSTEM_CLOCK_MODE_RUN;
            break;
        /* Very low power run mode */
        case STAT_VLPR:
            mode = SCG_SYSTEM_CLOCK_MODE_VLPR;
            break;
        /* This should never happen - core has to be in some run mode to execute code */
        default:
            mode = SCG_SYSTEM_CLOCK_MODE_NONE;
            break;
    }

    return mode;
}

/*FUNCTION**********************************************************************
 * Function Name : CLOCK_SYS_SwitchSystemClock
 * Description   : Internal function used by CLOCK_SYS_SetScgConfiguration function
 *END**************************************************************************/
static bool CLOCK_SYS_SwitchSystemClock(scg_system_clock_src_t to_clk) {
    scg_system_clock_config_t config;
    scg_system_clock_mode_t run_mode;

    /* Check destination clock */
    if ( to_clk != SCG_SYSTEM_CLOCK_SRC_NONE ) {

        /* Get & Convert Run mode from SMC to SCG defines*/
        run_mode = CLOCK_SYS_GetCurrentRunMode(SMC);

        if ( run_mode != SCG_SYSTEM_CLOCK_MODE_NONE ) {

            /* Read system clock configuration*/
            SCG_HAL_GetSystemClockConfig(SCG, run_mode, &config);

            /* Use to_clk as system clock source*/
            config.src = to_clk;

            /* Update run mode configuration */
            SCG_HAL_SetSystemClockConfig(SCG, run_mode, &config);

            /* Read new system clock configuration*/
            SCG_HAL_GetSystemClockConfig(SCG, SCG_SYSTEM_CLOCK_MODE_CURRENT, &config);

            if ( config.src == to_clk ) {
                   return true;
            }
        }
    }

    return false;
}
/*******************************************************************************
 * EOF
 ******************************************************************************/
