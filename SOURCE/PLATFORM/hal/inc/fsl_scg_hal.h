/* * Copyright (c) 2014, Freescale Semiconductor, Inc.
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

#ifndef __FSL_SCG_HAL_H__
#define __FSL_SCG_HAL_H__

#include "fsl_device_registers.h"
#include <stdbool.h>

/*! @file fsl_scg_hal.h */
#if FSL_FEATURE_SOC_SCG_COUNT

/*!
 * @addtogroup fsl_scg_hal System Clock Generator (SCG) HAL
 * @ingroup clock_manager 
 * @brief System Clock Generator Hardware Abstraction Layer
 * @{
 */

/******************************************************************************
 * Definitions
 *****************************************************************************/

/* @brief System PLL base multiplier value, it is the multiplier value when SCG_SPLLCFG[MULT]=0. */
#define SCG_SPLL_MULT_BASE 16U

/*
 * @brief System PLL base divider value, it is the PLL reference clock divider value when
 * SCG_SPLLCFG[PREDIV]=0.
 */
#define SCG_SPLL_PREDIV_BASE 1U

/* @brief Max value of SCG_SPLLCFG[MULT]. */
#define SCG_SPLL_MULT_MAX_VALUE 31U

/* @brief Max value of SCG_SPLLCFG[PREDIV]. */
#define SCG_SPLL_PREDIV_MAX_VALUE 7U

/*
 * @brief System PLL reference clock after SCG_SPLLCFG[PREDIV] should be in the range of
 * SCG_SPLL_REF_MIN to SCG_SPLL_REF_MAX.
 */
#define SCG_SPLL_REF_MIN 8000000U

/*
 * @brief System PLL reference clock after SCG_SPLLCFG[PREDIV] should be in the range of
 * SCG_SPLL_REF_MIN to SCG_SPLL_REF_MAX.
 */
#define SCG_SPLL_REF_MAX 32000000U

/* @brief EXTAL0 clock frequency. */
extern uint32_t g_xtal0ClkFreq;

/* @brief RTC_CLKIN clock frequency. */
extern uint32_t g_RtcClkInFreq;

/*!
 * @brief SCG status return code.
 */
typedef enum _scg_status
{
    STATUS_SCG_SUCCESS           = 0x0U, /*!< Success.          */
    STATUS_SCG_BUSY              = 0x1U, /*!< Clock is busy.    */
    STATUS_SCG_INVALID_PARAMETER = 0x2U, /*!< Invalid argument. */
    STATUS_SCG_INVALID_SRC       = 0x4U, /*!< Invalid source.   */
    STATUS_SCG_FAILED            = 0x8U  /*!< Execution failed. */
} scg_status_t;

/*!
 * @name System Clock.
 * @{
 */

/*!
 * @brief SCG system clock type.
 */
typedef enum _scg_system_clock_type
{
    SCG_SYSTEM_CLOCK_CORE,  /*!< Core clock.        */
    SCG_SYSTEM_CLOCK_PLAT,  /*!< Platform clock.    */
    SCG_SYSTEM_CLOCK_BUS,   /*!< BUS clock.         */
    SCG_SYSTEM_CLOCK_SLOW,  /*!< System slow clock. */
    SCG_SYSTEM_CLOCK_MAX,   /*!< Max value.         */
} scg_system_clock_type_t;

/*!
 * @brief SCG system clock source.
 */
typedef enum _scg_system_clock_src
{
    SCG_SYSTEM_CLOCK_SRC_SYS_OSC = 1U,  /*!< System OSC. */
    SCG_SYSTEM_CLOCK_SRC_SIRC    = 2U,  /*!< Slow IRC.   */
    SCG_SYSTEM_CLOCK_SRC_FIRC    = 3U,  /*!< Fast IRC.   */
    SCG_SYSTEM_CLOCK_SRC_SYS_PLL = 6U,  /*!< System PLL. */
    SCG_SYSTEM_CLOCK_SRC_NONE           /*!< MAX value.  */
} scg_system_clock_src_t;

/*!
 * @brief SCG system clock modes.
 */
typedef enum _scg_system_clock_mode
{
    SCG_SYSTEM_CLOCK_MODE_CURRENT = 0U,  /*!< Current mode.            */
    SCG_SYSTEM_CLOCK_MODE_RUN     = 1U,  /*!< Run mode.                */
    SCG_SYSTEM_CLOCK_MODE_VLPR    = 2U,  /*!< Very Low Power Run mode. */
    SCG_SYSTEM_CLOCK_MODE_HSRUN   = 3U,  /*!< High Speed Run mode.     */
    SCG_SYSTEM_CLOCK_MODE_NONE           /*!< MAX value.               */
} scg_system_clock_mode_t;

/*!
 * @brief SCG system clock divider value.
 */
typedef enum scg_system_clock_div
{
    SCG_SYSTEM_CLOCK_DIV_BY_1   = 0U,   /*!< Divided by 1. */
    SCG_SYSTEM_CLOCK_DIV_BY_2   = 1U,   /*!< Divided by 2. */
    SCG_SYSTEM_CLOCK_DIV_BY_3   = 2U,   /*!< Divided by 3. */
    SCG_SYSTEM_CLOCK_DIV_BY_4   = 3U,   /*!< Divided by 4. */
    SCG_SYSTEM_CLOCK_DIV_BY_5   = 4U,   /*!< Divided by 5. */
    SCG_SYSTEM_CLOCK_DIV_BY_6   = 5U,   /*!< Divided by 6. */
    SCG_SYSTEM_CLOCK_DIV_BY_7   = 6U,   /*!< Divided by 7. */
    SCG_SYSTEM_CLOCK_DIV_BY_8   = 7U,   /*!< Divided by 8. */
    SCG_SYSTEM_CLOCK_DIV_BY_9   = 8U,   /*!< Divided by 9. */
    SCG_SYSTEM_CLOCK_DIV_BY_10  = 9U,   /*!< Divided by 10. */
    SCG_SYSTEM_CLOCK_DIV_BY_11  = 10U,  /*!< Divided by 11. */
    SCG_SYSTEM_CLOCK_DIV_BY_12  = 11U,  /*!< Divided by 12. */
    SCG_SYSTEM_CLOCK_DIV_BY_13  = 12U,  /*!< Divided by 13. */
    SCG_SYSTEM_CLOCK_DIV_BY_14  = 13U,  /*!< Divided by 14. */
    SCG_SYSTEM_CLOCK_DIV_BY_15  = 14U,  /*!< Divided by 15. */
    SCG_SYSTEM_CLOCK_DIV_BY_16  = 15U,  /*!< Divided by 16. */
} scg_system_clock_div_t;

/*!
 * @brief SCG system clock configuration.
 */
typedef struct ScgSystemClockConfig
{
    scg_system_clock_div_t divSlow;  /*!< Slow clock divider.      */
    scg_system_clock_div_t divBus;   /*!< BUS clock divider.       */
    scg_system_clock_div_t divPlat;  /*!< Platform clock divider.  */
    scg_system_clock_div_t divCore;  /*!< Core clock divider.      */
    scg_system_clock_src_t src;      /*!< System clock source.     */
} scg_system_clock_config_t;

/* @} */

/*!
 * @brief SCG asynchronous clock type.
 */
typedef enum _scg_async_clock_type
{
    SCG_ASYNC_CLOCK_DIV1,   /*!< Clock divider 1  */
    SCG_ASYNC_CLOCK_DIV2,   /*!< Clock divider 2  */
    SCG_ASYNC_CLOCK_MAX,    /*!< Max value.       */
} scg_async_clock_type_t;

/*!
 * @brief SCG asynchronous clock divider value.
 */
typedef enum scg_async_clock_div
{
    SCG_ASYNC_CLOCK_DISABLE     = 0U,  /*!< Clock output is disabled.  */
    SCG_ASYNC_CLOCK_DIV_BY_1    = 1U,  /*!< Divided by 1.              */
    SCG_ASYNC_CLOCK_DIV_BY_2    = 2U,  /*!< Divided by 2.              */
    SCG_ASYNC_CLOCK_DIV_BY_4    = 3U,  /*!< Divided by 4.              */
    SCG_ASYNC_CLOCK_DIV_BY_8    = 4U,  /*!< Divided by 8.              */
    SCG_ASYNC_CLOCK_DIV_BY_16   = 5U,  /*!< Divided by 16.             */
    SCG_ASYNC_CLOCK_DIV_BY_32   = 6U,  /*!< Divided by 32.             */
    SCG_ASYNC_CLOCK_DIV_BY_64   = 7U,  /*!< Divided by 64.             */
} scg_async_clock_div_t;

/*!
 * @brief SCG system OSC monitor mode.
 */
typedef enum _scg_sosc_monitor_mode
{
    SCG_SOSC_MONITOR_DISABLE = 0U,                         /*!< Monitor disable.                          */
    SCG_SOSC_MONITOR_INT     = SCG_SOSCCSR_SOSCCM_MASK,    /*!< Interrupt when system OSC error detected. */
    SCG_SOSC_MONITOR_RESET   = SCG_SOSCCSR_SOSCCM_MASK |
                               SCG_SOSCCSR_SOSCCMRE_MASK,  /*!< Reset when system OSC error detected.     */
} scg_sosc_monitor_mode_t;

/*!
 * @brief SCG OSC frequency range select
 */
typedef enum _scg_sosc_range
{
    SCG_SOSC_RANGE_LOW    = 1U,  /*!< Low frequency range selected for the crystal OSC (32 kHz to 40 kHz).   */
    SCG_SOSC_RANGE_MID    = 2U,  /*!< Medium frequency range selected for the crystal OSC (1 Mhz to 8 Mhz).  */
    SCG_SOSC_RANGE_HIGH   = 3U,  /*!< High frequency range selected for the crystal OSC (8 Mhz to 32 Mhz).   */
} scg_sosc_range_t;

/*!
 * @brief SCG OSC high gain oscillator select.
 */
typedef enum _scg_sosc_gain
{
    SCG_SOSC_GAIN_LOW,  /* Configure crystal oscillator for low-power operation */
    SCG_SOSC_GAIN_HIGH  /* Configure crystal oscillator for high-gain operation */
} scg_sosc_gain_t;

/*!
 * @brief SCG OSC external reference clock select.
 */
typedef enum _scg_sosc_ext_ref_t
{
    SCG_SOSC_REF_EXT,    /* External reference clock requested    */
    SCG_SOSC_REF_OSC     /* Internal oscillator of OSC requested. */
} scg_sosc_ext_ref_t;

/*!
 * @brief SCG system OSC configuration.
 */
typedef struct ScgSysOscConfig
{
    uint32_t  freq;                       /*!< System OSC frequency.                                 */

    scg_sosc_monitor_mode_t monitorMode;   /*!< System OSC Clock monitor mode.                       */

    scg_sosc_ext_ref_t extRef;             /*!< System OSC External Reference Select.                */
    scg_sosc_gain_t    gain;               /*!< System OSC high-gain operation.                      */
    
    scg_sosc_range_t   range;              /*!< System OSC frequency range.                          */
        
    scg_async_clock_div_t div1;            /*!< Divider for platform asynchronous clock.             */
    scg_async_clock_div_t div2;            /*!< Divider for bus asynchronous clock.                  */

    bool enableInStop;                     /*!< System OSC is enable or not in stop mode.            */
    bool enableInLowPower;                 /*!< System OSC is enable or not in low power mode.       */
    bool enableErClk;                      /*!< System OSC 3V ERCLK output clock.                    */
            
    bool locked;                           /*!< System OSC Control Register can be written.          */
            
    bool initialize;                       /*!< Initialize or not the System OSC module.             */
} scg_sosc_config_t;

/*!
 * @brief SCG slow IRC clock frequency range.
 */
typedef enum _scg_sirc_range
{
    SCG_SIRC_RANGE_LOW,   /*!< Slow IRC low range clock (2 MHz).  */
    SCG_SIRC_RANGE_HIGH,  /*!< Slow IRC high range clock (8 MHz). */
} scg_sirc_range_t;

/*!
 * @brief SCG SIRC enable mode.
 */
typedef enum _scg_sirc_enable_mode
{
    SCG_SIRC_ENABLE              = SCG_SIRCCSR_SIRCEN_MASK,       /*!< Enable SIRC.              */
    SCG_SIRC_ENABLE_IN_STOP      = SCG_SIRCCSR_SIRCSTEN_MASK,     /*!< Enable SIRC in stop mode. */
    SCG_SIRC_ENABLE_IN_LOW_POWER = SCG_SIRCCSR_SIRCLPEN_MASK,     /*!< Enable SIRC in VLP mode.  */
} scg_sirc_enable_mode_t;

/*!
 * @brief SCG slow IRC clock configuration.
 */
typedef struct ScgSircConfig
{
    scg_sirc_range_t range;         /*!< Slow IRC frequency range.                 */

    scg_async_clock_div_t div1;     /*!< Divider for platform asynchronous clock.  */
    scg_async_clock_div_t div2;     /*!< Divider for bus asynchronous clock.       */
    
    bool initialize;                /*!< Initialize or not the SIRC module.        */
    bool enableInStop;              /*!< SIRC is enable or not in stop mode.       */
    bool enableInLowPower;          /*!< SIRC is enable or not in low power mode.  */
                                
    bool locked;                    /*!< SIRC Control Register can be written.     */
} scg_sirc_config_t;

/*!
 * @brief SCG fast IRC clock frequency range.
 */
typedef enum _scg_firc_range
{
    SCG_FIRC_RANGE_48M,   /*!< Fast IRC is trimmed to 48MHz.  */
    SCG_FIRC_RANGE_52M,   /*!< Fast IRC is trimmed to 52MHz.  */
    SCG_FIRC_RANGE_56M,   /*!< Fast IRC is trimmed to 56MHz.  */
    SCG_FIRC_RANGE_60M,   /*!< Fast IRC is trimmed to 60MHz.  */
} scg_firc_range_t;

/*!
 * @brief SCG FIRC enable mode.
 */
typedef enum _scg_firc_enable_mode
{
    SCG_FIRC_ENABLE              = SCG_FIRCCSR_FIRCEN_MASK,       /*!< Enable FIRC.              */
    SCG_FIRC_ENABLE_IN_STOP      = SCG_FIRCCSR_FIRCSTEN_MASK,     /*!< Enable FIRC in stop mode. */
    SCG_FIRC_ENABLE_IN_LOW_POWER = SCG_FIRCCSR_FIRCLPEN_MASK,     /*!< Enable FIRC in VLP mode.  */
} scg_firc_enable_mode_t;

/*!
 * @brief SCG fast IRC clock configuration.
 */
typedef struct ScgFircConfig
{
    scg_firc_range_t range;            /*!< Fast IRC frequency range.                 */

    scg_async_clock_div_t div1;        /*!< Divider for platform asynchronous clock.  */
    scg_async_clock_div_t div2;        /*!< Divider for bus asynchronous clock.       */

    uint8_t trimCoar;                  /*!< Trim coarse value.                        */
    uint8_t trimFine;                  /*!< Trim fine value.                          */

    
    bool enableInStop;                 /*!< FIRC is enable or not in stop mode.       */
    bool enableInLowPower;             /*!< FIRC is enable or not in lowpower mode.   */
    bool regulator;                    /*!< FIRC regulator is enable or not.          */
    bool locked;                       /*!< FIRC Control Register can be written.     */
    bool enableTrim;                   /*!< FIRC is trimmed to an external clock.     */
    bool updateTrim;                   /*!< Enable FIRC trimming updates.             */
         
    bool initialize;                   /*!< Initialize or not the FIRC module.        */
} scg_firc_config_t;

/*!
 * @brief SCG system PLL monitor mode.
 */
typedef enum _scg_spll_monitor_mode
{
    SCG_SPLL_MONITOR_DISABLE = 0U,                         /*!< Monitor disable.                          */
    SCG_SPLL_MONITOR_INT     = SCG_SPLLCSR_SPLLCM_MASK,    /*!< Interrupt when system PLL error detected. */
    SCG_SPLL_MONITOR_RESET   = SCG_SPLLCSR_SPLLCM_MASK |
                               SCG_SPLLCSR_SPLLCMRE_MASK,  /*!< Reset when system PLL error detected.     */
} scg_spll_monitor_mode_t;


/*!
 * @brief SCG system PLL configuration.
 */
typedef struct ScgSysPllConfig
{
    scg_spll_monitor_mode_t monitorMode; /*!< Clock monitor mode selected.                    */

    uint8_t        prediv;               /*!< PLL reference clock divider.                    */
    uint8_t        mult;                 /*!< System PLL multiplier.                          */

    scg_async_clock_div_t div1;          /*!< Divider for platform asynchronous clock.        */
    scg_async_clock_div_t div2;          /*!< Divider for bus asynchronous clock.             */

    bool enableInStop;                   /*!< System PLL clock is enable or not in stop mode. */

    bool locked;                         /*!< System PLL Control Register can be written.     */
    bool initialize;                     /*!< Initialize or not the System PLL module.        */
} scg_spll_config_t;

/*!
 * @brief SCG RTC configuration.
 */
typedef struct ScgRtcConfig
{
    uint32_t  rtcClkInFreq;              /*!< RTC_CLKIN frequency.                            */
    bool      initialize;                /*!< Initialize or not the RTC.                      */
} scg_rtc_config_t;

/*!
 * @brief SCG Clock Mode Configuration structure.
 */
typedef struct ScgClockModeConfig
{
    scg_system_clock_config_t rccrConfig;      /*!< Run Clock Control configuration.                 */
    scg_system_clock_config_t vccrConfig;      /*!< VLPR Clock Control configuration.                */
    scg_system_clock_config_t hccrConfig;      /*!< HSRUN Clock Control configuration.               */
    scg_system_clock_src_t    alternateClock;  /*!< Alternate clock used during initialization       */
    bool                      initialize;      /*!< Initialize or not the Clock Mode Configuration.  */
} scg_clock_mode_config_t;

/*!
 * @brief SCG configure structure.
 */
typedef struct ScgConfig
{
    scg_sirc_config_t         sircConfig;      /*!< Slow internal reference clock configuration.     */
    scg_firc_config_t         fircConfig;      /*!< Fast internal reference clock configuration.     */
    scg_sosc_config_t         soscConfig;      /*!< System oscillator configuration.                 */
    scg_spll_config_t         spllConfig;      /*!< System Phase locked loop configuration.          */
    scg_rtc_config_t          rtcConfig;       /*!< Real Time Clock configuration.                   */
    scg_clock_mode_config_t   clockModeConfig; /*!< SCG Clock Mode Configuration.                    */
} scg_config_t;

#if defined(__cplusplus)
extern "C" {
#endif

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @name System Clock.
 * @{
 */

/*!
 * @brief Set the system clock configuration in specified mode.
 *
 * This function sets the system configuration in specified mode.
 *
 * @param base Register base address for the SCG instance.
 * @param mode specifies the mode.
 * @param config Pointer to the configuration.
 */
void SCG_HAL_SetSystemClockConfig(SCG_Type * base,
                                  scg_system_clock_mode_t mode,
                                  scg_system_clock_config_t const *config);

/*!
 * @brief Get the system clock configuration for specified mode.
 *
 * This function gets the system configuration for specified mode.
 *
 * @param base Register base address for the SCG instance.
 * @param mode specifies the mode.
 * @param config Pointer to the configuration.
 */
void SCG_HAL_GetSystemClockConfig(SCG_Type * base,
                                  scg_system_clock_mode_t mode,
                                  scg_system_clock_config_t *config);

/* @} */

/*!
 * @name System OSC Clock.
 * @{
 */

/*!
 * @brief Get the default system OSC configuration.
 *
 * This function gets the default SCG system OSC configuration.
 *
 * @param config Pointer to the configuration structure.
 */
void SCG_HAL_GetSysOscDefaultConfig(scg_sosc_config_t *config);

/*!
 * @brief Initialize SCG system OSC.
 *
 * This function enables the SCG system OSC clock according to the
 * configuration.
 *
 * @param base Register base address for the SCG instance.
 * @param config   Pointer to the configuration structure.
 * @return Status of module initialization
 * @retval STATUS_SCG_SUCCESS System OSC is initialized.
 * @retval STATUS_SCG_BUSY System OSC has been enabled and used by system clock.
 *
 * @note This function can not detect whether system OSC has been enabled and
 * used by some IPs.
 */
scg_status_t SCG_HAL_InitSysOsc(SCG_Type * base,
                                scg_sosc_config_t const *config);

/*!
 * @brief De-initialize SCG system OSC.
 *
 * This function disables the SCG system OSC clock.
 *
 * @param base Register base address for the SCG instance.
 * @return Status of module de-initialization
 * @retval STATUS_SCG_SUCCESS System OSC is deinitialized.
 * @retval STATUS_SCG_BUSY System OSC is used by system clock.
 *
 * @note This function can not detect whether system OSC is used by some IPs.
 */
scg_status_t SCG_HAL_DeinitSysOsc(SCG_Type * base);

/*!
 * @brief Get SCG system OSC clock frequency (SYSOSC).
 *
 * @param base Register base address for the SCG instance.
 * @return  Clock frequency, if clock is invalid, return 0.
 */
uint32_t SCG_HAL_GetSysOscFreq(SCG_Type * base);

/*!
 * @brief Get SCG asynchronous clock frequency from system OSC.
 *
 * @param base Register base address for the SCG instance.
 * @param type     The asynchronous clock type.
 * @return  Clock frequency, if clock is invalid, return 0.
 */
uint32_t SCG_HAL_GetSysOscAsyncFreq(SCG_Type * base,
                                  scg_async_clock_type_t type);

/* @} */

/*!
 * @name Slow IRC Clock.
 * @{
 */

/*!
 * @brief Get the default slow IRC clock configuration.
 *
 * This function gets the default slow IRC clock configuration (SIRC).
 *
 * @param config Pointer to the configuration structure.
 */
void SCG_HAL_GetSircDefaultConfig(scg_sirc_config_t *config);

/*!
 * @brief Initialize SCG slow IRC clock.
 *
 * This function enables the SCG slow IRC clock according to the
 * configuration.
 *
 * @param base Register base address for the SCG instance.
 * @param config   Pointer to the configuration structure.
 * @return Status of module initialization
 * @retval STATUS_SCG_SUCCESS SIRC is initialized.
 * @retval STATUS_SCG_BUSY SIRC has been enabled and used by system clock.
 *
 * @note This function can not detect whether SIRC is used by some IPs.
 */
scg_status_t SCG_HAL_InitSirc(SCG_Type * base,
                              const scg_sirc_config_t *config);

/*!
 * @brief De-initialize SCG slow IRC.
 *
 * This function disables the SCG slow IRC.
 *
 * @param base Register base address for the SCG instance.
 * @return Status of module de-initialization
 * @retval STATUS_SCG_SUCCESS SIRC is deinitialized.
 * @retval STATUS_SCG_BUSY SIRC is used by system clock.
 *
 * @note This function can not detect whether SIRC is used by some IPs.
 */
scg_status_t SCG_HAL_DeinitSirc(SCG_Type * base);

/*!
 * @brief Get SCG SIRC clock frequency.
 *
 * @param base Register base address for the SCG instance.
 * @return  Clock frequency, if clock is invalid, return 0.
 */
uint32_t SCG_HAL_GetSircFreq(SCG_Type * base);

/*!
 * @brief Get SCG asynchronous clock frequency from SIRC.
 *
 * @param base Register base address for the SCG instance.
 * @param type     The asynchronous clock type.
 * @return  Clock frequency, if clock is invalid, return 0.
 */
uint32_t SCG_HAL_GetSircAsyncFreq(SCG_Type * base,
                                scg_async_clock_type_t type);

/* @} */

/*!
 * @name Fast IRC Clock.
 * @{
 */

/*!
 * @brief Get the default fast IRC clock configuration.
 *
 * This function gets the default fast IRC clock configuration (FIRC).
 * The default trim coarse value and trim fine value are all 0. If updateTrim
 * is false, then trimCoar and trimFine must be set. If updateTrim is ture,
 * then it is not necessary to set trimCoar and trimFine.
 *
 * @param config Pointer to the configuration structure.
 */
void SCG_HAL_GetFircDefaultConfig(scg_firc_config_t *config);

/*!
 * @brief Initialize SCG fast IRC clock.
 *
 * This function enables the SCG fast IRC clock according to the
 * configuration.
 *
 * @param base Register base address for the SCG instance.
 * @param config   Pointer to the configuration structure.
 * @return Status of module initialization
 * @retval STATUS_SCG_SUCCESS FIRC is initialized.
 * @retval STATUS_SCG_BUSY FIRC has been enabled and used by system clock.
 * @retval STATUS_SCG_FAILED FIRC initialization routine detected an error
 *
 * @note This function can not detect whether system OSC has been enabled and
 * used by some IPs.
 */
scg_status_t SCG_HAL_InitFirc(SCG_Type * base,
                              const scg_firc_config_t *config);

/*!
 * @brief De-initialize SCG fast IRC.
 *
 * This function disables the SCG fast IRC.
 *
 * @param base Register base address for the SCG instance.
 * @return Status of module de-initialization
 * @retval STATUS_SCG_SUCCESS FIRC is deinitialized.
 * @retval STATUS_SCG_BUSY FIRC is used by system clock.
 *
 * @note This function can not detect whether FIRC is used by some IPs.
 */
scg_status_t SCG_HAL_DeinitFirc(SCG_Type * base);

/*!
 * @brief Get SCG FIRC clock frequency.
 *
 * @param base Register base address for the SCG instance.
 * @return  Clock frequency, if clock is invalid, return 0.
 */
uint32_t SCG_HAL_GetFircFreq(SCG_Type * base);

/*!
 * @brief Get SCG asynchronous clock frequency from FIRC.
 *
 * @param base Register base address for the SCG instance.
 * @param type     The asynchronous clock type.
 * @return  Clock frequency, if clock is invalid, return 0.
 */
uint32_t SCG_HAL_GetFircAsyncFreq(SCG_Type * base,
                                scg_async_clock_type_t type);


/*!
 * @brief Get SCG system clock source.
 *
 * This function gets the SCG system clock source, these clocks are used for
 * core, platform, external and bus clock domains.
 *
 * @param base Register base address for the SCG instance.
 * @return  Clock source.
 */
static inline scg_system_clock_src_t SCG_HAL_GetSystemClockSrc(SCG_Type * base)
{
    uint32_t regValue = base->CSR;
    regValue = (regValue & SCG_CSR_SCS_MASK) >> SCG_CSR_SCS_SHIFT;
    return (scg_system_clock_src_t)regValue;
}

/*!
 * @brief Get SCG system clock frequency.
 *
 * This function gets the SCG system clock frequency, these clocks are used for
 * core, platform, external and bus clock domains.
 *
 * @param base Register base address for the SCG instance.
 * @param type     Which type of clock to get, core clock or slow clock.
 * @return  Clock frequency.
 */
uint32_t SCG_HAL_GetSystemClockFreq(SCG_Type * base,
                                  scg_system_clock_type_t type);

/* @} */


/*!
 * @name System PLL Clock.
 * @{
 */

/*!
 * @brief Get the default system PLL configuration.
 *
 * This function gets the default SCG system PLL configuration.
 *
 * @param config Pointer to the configuration structure.
 */
void SCG_HAL_GetSysPllDefaultConfig(scg_spll_config_t *config);

/*!
 * @brief Initialize SCG system PLL.
 *
 * This function enables the SCG system PLL clock according to the
 * configuration. The system PLL could use system OSC or FIRC as
 * the clock source, please make sure the source clock is valid before
 * this function.
 *
 * @param base Register base address for the SCG instance.
 * @param config   Pointer to the configuration structure.
 * @return Status of module initialization
 * @retval STATUS_SCG_SUCCESS System PLL is initialized.
 * @retval STATUS_SCG_INVALID_SRC Clock source is invalid.
 * @retval STATUS_SCG_INVALID_PARAMETER The pre-divider is invalid.
 * @retval STATUS_SCG_BUSY System PLL has been enabled and used by system clock.
 *
 * @note This function can not detect whether system PLL has been enabled and
 * used by some IPs.
 */
scg_status_t SCG_HAL_InitSysPll(SCG_Type * base,
                                scg_spll_config_t const *config);

/*!
 * @brief De-initialize SCG system PLL.
 *
 * This function disables the SCG system PLL.
 *
 * @param base Register base address for the SCG instance.
 * @return Status of module de-initialization
 * @retval STATUS_SCG_SUCCESS system PLL is deinitialized.
 * @retval STATUS_SCG_BUSY system PLL is used by system clock.
 *
 * @note This function can not detect whether system PLL is used by some IPs.
 */
scg_status_t SCG_HAL_DeinitSysPll(SCG_Type * base);

/*!
 * @brief Get SCG system PLL clock frequency.
 *
 * @param base Register base address for the SCG instance.
 * @return  Clock frequency, if clock is invalid, return 0.
 */
uint32_t SCG_HAL_GetSysPllFreq(SCG_Type * base);

/*!
 * @brief Get SCG asynchronous clock frequency from system PLL.
 *
 * @param base Register base address for the SCG instance.
 * @param type     The asynchronous clock type.
 * @return  Clock frequency, if clock is invalid, return 0.
 */
uint32_t SCG_HAL_GetSysPllAsyncFreq(SCG_Type * base,
                                  scg_async_clock_type_t type);

/* @} */

/*!
 * @name RTC Clock.
 * @{
 */

/*!
 * @brief Set SCG RTC CLKIN clock frequency.
 *
 * @param base Register base address for the SCG instance.
 * @param frequency The frequency of the RTC_CLKIN
 */
void SCG_HAL_SetRtcClkInFreq(SCG_Type * base, uint32_t frequency);

/*!
 * @brief Get SCG RTC CLKIN clock frequency.
 *
 * @param base Register base address for the SCG instance.
 * @return  Clock frequency
 */
uint32_t SCG_HAL_GetRtcClkInFreq(SCG_Type * base);

/* @} */

#if defined(__cplusplus)
extern }
#endif

/*!
 * @}
 */

#endif /* FSL_FEATURE_SOC_SCG_COUNT */
#endif /* __FSL_SCG_HAL_H__ */

/******************************************************************************
 * EOF
 *****************************************************************************/

