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

#if !defined(__FSL_SIM_HAL_S32K144_H__)
#define __FSL_SIM_HAL_S32K144_H__

/*! @file fsl_sim_hal_S32K144.h */

/*!
 * @ingroup fsl_sim_hal
 * @defgroup fsl_sim_hal_s32k144
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* @brief TClk clock frequency. */
#define NUMBER_OF_TCLK_INPUTS 3
extern uint32_t g_TClkFreq[NUMBER_OF_TCLK_INPUTS];      /* TCLKx clocks */

/* @brief LPO fixed clock frequency. */
#define LPO_128K_FREQUENCY 128000
#define LPO_32K_FREQUENCY   32000
#define LPO_1K_FREQUENCY     1000


 /*! @brief Conversion Complete trigger source select */
typedef enum _coco_trigger_src_t
{
    COCO_TRIGGER_SRC_ADC1_COCO,         /*!< ADC1 Conversion Complete     */
    COCO_TRIGGER_SRC_ADC2_COCO          /*!< ADC2 Conversion Complete     */
} coco_trigger_src_t;
 
 /*! @brief Debug trace clock source select */
typedef enum _clock_trace_src_t
{
    CLOCK_TRACE_SRC_CORE_CLK,         /*!< core clock     */
    CLOCK_TRACE_SRC_PLATFORM_CLK      /*!< platform clock  */
} clock_trace_src_t;


 /*! @brief PDB back-to-back select */
typedef enum _sim_pdb_bb_src_t
{
    PDB_BACK_TO_BACK_OPTION_0,     /*!< PDBx ch0 back-to-back operation with ADCx COCO[7:0]     */
    PDB_BACK_TO_BACK_OPTION_1      /*!< Ch0 of PDBx back-to-back operation with COCO[7:0] of ADCx  */
} sim_pdb_bb_src_t;

/*! @brief SIM CLKOUT select */
typedef enum _sim_clkout_src
{
    SIM_CLKOUT_SEL_SYSTEM_OSC_CLK  = 0U,  /*!< System oscillator clock  */
    SIM_CLKOUT_SEL_RESERVED_1_CLK  = 1U,  /*!< Reserved                 */
    SIM_CLKOUT_SEL_LPO_CLK         = 2U,  /*!< LPO clock (128 kHz)      */
    SIM_CLKOUT_SEL_RESERVED_3_CLK  = 3U,  /*!< Reserved                 */
} sim_clkout_src_t;

#define  SIM_HAL_ADC_INTERLEAVE_BIT0_MASK   (1U) /*!< PTB0 to ADC0_SE4 and ADC1_SE14 MASK */
#define  SIM_HAL_ADC_INTERLEAVE_BIT1_MASK   (2U) /*!< PTB1 to ADC0_SE4 and ADC1_SE15 MASK */
#define  SIM_HAL_ADC_INTERLEAVE_BIT2_MASK   (4U) /*!< PTB13 to ADC1_SE8 and ADC2_SE8 MASK */
#define  SIM_HAL_ADC_INTERLEAVE_BIT3_MASK   (8U) /*!< PTB14 to ADC1_SE9 and ADC2_SE9 MASK */

/*! @brief SIM FlexTimer external clock select */
typedef enum _sim_ftm_clk_sel
{
    SIM_FTM_CLK_SEL_00 = 0U,                 /*!< FTM external clock driven by TCLK0 pin. */
    SIM_FTM_CLK_SEL_01 = 1U,                 /*!< FTM external clock driven by TCLK1 pin. */
    SIM_FTM_CLK_SEL_10 = 2U,                 /*!< FTM external clock driven by TCLK2 pin. */
    SIM_FTM_CLK_SEL_11 = 3U                  /*!< No clock input */
} sim_ftm_clk_sel_t;

/*! @brief SIM FlexTimer x Fault y select */
typedef enum _sim_ftm_flt_sel
{
    SIM_FTM_FLT_SEL_0,                 /*!< FlexTimer x fault y select 0 */
    SIM_FTM_FLT_SEL_1                  /*!< FlexTimer x fault y select 1 */
} sim_ftm_flt_sel_t;

#define  SIM_HAL_FTM_FLT0_MASK   (1) /*!< Bit value = 0: FTMx_FLT0 pin ; Bit value = 1: TRGMUX_FTMx out*/
#define  SIM_HAL_FTM_FLT1_MASK   (2) /*!< Bit value = 0: FTMx_FLT1 pin ; Bit value = 1: TRGMUX_FTMx out */
#define  SIM_HAL_FTM_FLT2_MASK   (4) /*!< Bit value = 0: FTMx_FLT2 pin ; Bit value = 1: TRGMUX_FTMx out */

/*! @brief SIM CLK32KSEL clock source select */
typedef enum _sim_clk32k_sel_src
{
    SIM_OSC32K_SEL_SIRCDIV2_CLK,      /* SIRCDIV2 clock          */
    SIM_OSC32K_SEL_LPO_32K,           /* 32 kHz LPO clock        */
    SIM_OSC32K_SEL_RTC_CLKIN,         /* RTC_CLKIN clock         */
    SIM_OSC32K_SEL_FIRCDIV2_CLK       /* FIRCDIV2 clock          */
} sim_clk32k_sel_src_t;

/*! @brief SIM LPOCLKSEL clock source select */
typedef enum _sim_lpoclk_sel_src
{
    SIM_LPO_CLK_SEL_LPO_128K,           /* 128 kHz LPO clock */
    SIM_LPO_CLK_SEL_NO_CLOCK,           /* No clock */
    SIM_LPO_CLK_SEL_LPO_32K,            /* 32 kHz LPO clock which is divided by the 128 kHz LPO clock */
    SIM_LPO_CLK_SEL_LPO_1K              /* 1 kHz LPO clock which is divided by the 128 kHz LPO clock */
} sim_lpoclk_sel_src_t;

/*! @brief SIM ADCx pre-trigger select */
typedef enum _sim_adc_pretrg_sel
{
    SIM_ADC_PRETRG_SEL_PDB,                 /*!< PDB pre-trigger */
    SIM_ADC_PRETRG_SEL_TRGMUX,              /*!< TRGMUX pre-trigger */
    SIM_ADC_PRETRG_SEL_SOFTWARE,            /*!< Software pre-trigger */
    SIM_ADC_PRETRG_SEL_RESERVED             /*!< Reserved */
} sim_adc_pretrg_sel_t;

/*! @brief SIM ADCx software pre-trigger select */
typedef enum _sim_adc_sw_pretrg_sel
{
    SIM_ADC_SW_PRETRG_SEL_DISABLED  = 0U,   /*!< Software pre-trigger disabled */
    SIM_ADC_SW_PRETRG_SEL_RESERVED0 = 1U,   /*!< Reserved */
    SIM_ADC_SW_PRETRG_SEL_RESERVED1 = 2U,   /*!< Reserved */
    SIM_ADC_SW_PRETRG_SEL_RESERVED2 = 3U,   /*!< Reserved */
    SIM_ADC_SW_PRETRG_SEL_0         = 4U,   /*!< Software pre-trigger 0 */
    SIM_ADC_SW_PRETRG_SEL_1         = 5U,   /*!< Software pre-trigger 1 */
    SIM_ADC_SW_PRETRG_SEL_2         = 6U,   /*!< Software pre-trigger 2 */
    SIM_ADC_SW_PRETRG_SEL_3         = 7U    /*!< Software pre-trigger 3 */
} sim_adc_sw_pretrg_sel_t;

/*! @brief SIM ADCx trigger select */
typedef enum _sim_adc_trg_sel
{
    SIM_ADC_TRG_SEL_PDB            = 0U,    /*!< PDB output     */
    SIM_ADC_TRG_SEL_TRGMUX         = 1U     /*!< TRGMUX output  */
} sim_adc_trg_sel_t;

/*! @brief SIM FlexTimer x channel y output source select */
typedef enum _sim_ftm_ch_out_src
{
    SIM_FTM_CH_OUT_SRC_0, /*!< FlexTimer x channel y output source 0. */
    SIM_FTM_CH_OUT_SRC_1, /*!< FlexTimer x channel y output source 1. */
} sim_ftm_ch_out_src_t;

/*! @brief SIM FlexTimer x channel y input source select */
typedef enum _sim_ftm_ch_src
{
    SIM_FTM_CH_SRC_0,    /*!< FlexTimer x channel y input source 0. */
    SIM_FTM_CH_SRC_1,    /*!< FlexTimer x channel y input source 1. */
    SIM_FTM_CH_SRC_2,    /*!< FlexTimer x channel y input source 2. */
    SIM_FTM_CH_SRC_3,    /*!< FlexTimer x channel y input source 3. */
} sim_ftm_ch_src_t;

/*! @brief SIM ADC configuration. */
typedef struct _sim_adc_config_t
{
    sim_pdb_bb_src_t         pdbBbSelect;                           /*!< PDB back-to-back select.          */
    uint8_t                  interleaveSelect;                      /*!< ADC interleave channel.           */
    sim_adc_pretrg_sel_t     pretrgSeelect[ADC_INSTANCE_COUNT];     /*!< ADCx pre-trigger select.          */
    sim_adc_sw_pretrg_sel_t  swPreTrgSelect[ADC_INSTANCE_COUNT];    /*!< ADCx software pre-trigger select. */
    sim_adc_trg_sel_t        triggerSeelect[ADC_INSTANCE_COUNT];    /*!< ADCx trigger select.              */
} sim_adc_config_t;

/*! @brief SIM ClockOut configuration. */
typedef struct _sim_clock_out_config_t
{
    bool              initialize;     /*!< Initialize or not the ClockOut clock.  */
    sim_clkout_src_t  source;         /*!< ClockOut source select.                */
} sim_clock_out_config_t;

/*! @brief SIM Trigger configuration. */
typedef struct _sim_trigger_config_t
{
    coco_trigger_src_t  cocoSource; /*!< Conversion Complete trigger source select. */
    bool              swTrigMux;  /*!< Software Trigger bit for TRGMUX. */
} sim_trigger_config_t;

/*! @brief SIM LPO Clocks configuration. */
typedef struct _sim_lpo_clock_config_t
{
    bool                  initialize;       /*!< Initialize or not the LPO clock.     */
    sim_clk32k_sel_src_t  sourceClk32k;     /*!< Clk32K source select.                */
    sim_lpoclk_sel_src_t  sourceLpoClk;     /*!< LPO clock source select.             */
    bool       enableLpo32k;                /*!< MSCM Clock Gating Control enable.    */
    bool       enableLpo1k;                 /*!< MSCM Clock Gating Control enable.    */
} sim_lpo_clock_config_t;

/*! @brief SIM FTM configuration. */
typedef struct _sim_ftm_config_t
{
    sim_ftm_clk_sel_t  extClkSource;           /*!< FlexTimer external clock select.    */
    uint8_t            faultSoruce;            /*!< FlexTimer Fault select.             */
    sim_ftm_ch_src_t   inputSource;            /*!< FlexTimer input source select.      */
    uint8_t            modulationSelect;       /*!< FlexTimer channel modulation select.*/
    bool               syncBit;                /*!< FlexTimer Sync Bit.                 */
} sim_ftm_config_t;

/*! @brief SIM  Platform Gate Clock configuration. */
typedef struct _sim_plat_gate_config_t
{
    bool    initialize;     /*!< Initialize or not the Trace clock.  */
    bool    enableMscm;     /*!< MSCM Clock Gating Control enable.         */
    bool    enableMpu;      /*!< MPU Clock Gating Control enable.          */
    bool    enableDma;      /*!< DMA Clock Gating Control enable.          */
} sim_plat_gate_config_t;

/*! @brief SIM  Platform Gate Clock configuration. */
typedef struct _sim_tclk_config_t
{
    bool      initialize;                         /*!< Initialize or not the Trace clock.  */
    uint32_t  tclkFreq[NUMBER_OF_TCLK_INPUTS];    /*!< TCLKx frequency.                    */
} sim_tclk_config_t;

/*! @brief SIM Flash configuration. */
typedef struct _sim_flash_config_t
{
    bool    disable;     /*!< Flash disable.          */
    bool    doze;        /*!< Flash doze.             */
} sim_flash_config_t;

/*! @brief SIM Debug Trace clock configuration. */
typedef struct _sim_trace_clock_config_t
{
    bool               initialize;    /*!< Initialize or not the Trace clock.  */
    bool               divEnable;     /*!< Trace clock divider enable.         */
    clock_trace_src_t  source;        /*!< Trace clock select.                 */
    uint8_t            divider;       /*!< Trace clock divider divisor.        */
    bool               divFraction;   /*!< Trace clock divider fraction.       */
} sim_trace_clock_config_t;

/*!
 * @brief SIM configure structure.
 */
typedef struct SimConfig
{
    sim_clock_out_config_t    clockOutConfig;                 /*!< Clock Out configuration.           */
    sim_lpo_clock_config_t    lpoClockConfig;                 /*!< Low Power Clock configuration.     */
    sim_tclk_config_t         tclkConfig;                     /*!< Platform Gate Clock configuration. */
    sim_plat_gate_config_t    platGateConfig;                 /*!< Platform Gate Clock configuration. */
    sim_trace_clock_config_t  traceClockConfig;               /*!< Trace clock configuration.         */
} sim_clock_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/


 /*!
 * @brief Set PDB back-to-back selection.
 *
 * This function sets PDB back-to-back selection.
 *
 * @param base Base address for current SIM instance.
 * @param setting  The value to set.
 */
static inline void SIM_HAL_SetPdbBackToBackSrc(SIM_Type * base, sim_pdb_bb_src_t setting)
{
    BITBAND_ACCESS32(&(base->CHIPCTL), SIM_CHIPCTL_PDB_BB_SEL_SHIFT) = setting;
}

/*!
 * @brief Get PDB back-to-back selection.
 *
 * This function gets PDB back-to-back selection.
 *
 * @param base Base address for current SIM instance.
 * @return Current selection.
 */
static inline sim_pdb_bb_src_t SIM_HAL_GetPdbBackToBackSrc(SIM_Type * base)
{
    return (sim_pdb_bb_src_t)BITBAND_ACCESS32(&(base->CHIPCTL), SIM_CHIPCTL_PDB_BB_SEL_SHIFT);
}

/*!
 * @brief Set CLKOUTSEL selection.
 *
 * This function sets the selection of the clock to output on the CLKOUT pin.
 *
 * @param base Base address for current SIM instance.
 * @param setting  The value to set.
 */
static inline void SIM_HAL_SetClkOutSel(SIM_Type * base, sim_clkout_src_t setting)
{
    uint32_t regValue = base->CHIPCTL;
    regValue &= ~(SIM_CHIPCTL_CLKOUTSEL_MASK);
    regValue |= SIM_CHIPCTL_CLKOUTSEL(setting);
    base->CHIPCTL = regValue;
}

/*!
 * @brief Get CLKOUTSEL selection.
 *
 * This function gets the selection of the clock to output on the CLKOUT pin.
 *
 * @param base Base address for current SIM instance.
 * @return Current selection.
 */
static inline sim_clkout_src_t SIM_HAL_GetClkOutSel(SIM_Type * base)
{
    uint32_t regValue = base->CHIPCTL;
    regValue = (regValue & SIM_CHIPCTL_CLKOUTSEL_MASK) >> SIM_CHIPCTL_CLKOUTSEL_SHIFT;
    return (sim_clkout_src_t)regValue;
}

/*!
 * @brief Set ADC interleave channel select.
 *
 * This function sets value of ADC interleave channel select.
 *
 * @param base Base address for current SIM instance.
 * @param setting  The value to set. See SIM_HAL_ADC_INTERLEAVE_BIT0_MASK
 */
static inline void SIM_HAL_SetAdcInterleaveSel(SIM_Type * base, uint8_t setting)
{
    uint32_t regValue;
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(setting < (1<<SIM_CHIPCTL_ADC_INTERLEAVE_EN_WIDTH));
#endif
    regValue = base->CHIPCTL;
    regValue &= ~(SIM_CHIPCTL_ADC_INTERLEAVE_EN_MASK);
    regValue |= SIM_CHIPCTL_ADC_INTERLEAVE_EN(setting);
    base->CHIPCTL = regValue;
}

/*!
 * @brief Get ADC interleave channel select.
 *
 * This function gets value of ADC interleave channel select.
 *
 * @param base Base address for current SIM instance.
 * @return Current value. See SIM_HAL_ADC_INTERLEAVE_BIT0_MASK
 */
static inline uint8_t SIM_HAL_GetAdcInterleaveSel(SIM_Type * base)
{
    uint32_t regValue = base->CHIPCTL;
    regValue = (regValue & SIM_CHIPCTL_ADC_INTERLEAVE_EN_MASK) >> SIM_CHIPCTL_ADC_INTERLEAVE_EN_SHIFT;
    return regValue;
}

/*!
 * @brief Sets the FlexTimer x external clock pin select setting.
 *
 * This function  selects the source of FTMx external clock pin select.
 *
 * @param base      Base address for current SIM instance.
 * @param instance  Device instance.
 * @param select    FTMx external clock pin select
 */
void SIM_HAL_SetFtmExternalClkPinMode(SIM_Type * base,
                                      uint32_t instance,
                                      sim_ftm_clk_sel_t select);

/*!
 * @brief Gets the FlexTimer x external clock pin select setting.
 *
 * This function gets the FlexTimer x external clock pin select setting.
 *
 * @param base      Base address for current SIM instance.
 * @param instance  Device instance.
 * @return select   FlexTimer x external clock pin select setting
 */
sim_ftm_clk_sel_t SIM_HAL_GetFtmExternalClkPinMode(SIM_Type * base,
                                                   uint32_t instance);

/*!
 * @brief Sets the FlexTimer x faults select settings.
 *
 * This function  sets the FlexTimer x faults select settings.
 *
 * @param base      Base address for current SIM instance.
 * @param instance  Device instance.
 * @param select    FlexTimer x faults select settings. See SIM_HAL_FTM_FLT0_MASK
 */
void SIM_HAL_SetFtmFaultSelMode(SIM_Type * base,
                                uint32_t instance,
                                uint8_t select);

/*!
 * @brief Gets the FlexTimer x faults select settings.
 *
 * This function  gets the FlexTimer x faults select settings.
 *
 * @param base      Base address for current SIM instance.
 * @param instance  Device instance.
 * @return select   FlexTimer x faults select settings. See SIM_HAL_FTM_FLT0_MASK
 */
uint8_t SIM_HAL_GetFtmFaultSelMode(SIM_Type * base,
                                   uint32_t instance);

/*!
 * @brief Get the clock selection of CLK32KSEL.
 *
 * This function gets the clock selection of CLK32KSEL.
 *
 * @param base Base address for current SIM instance.
 * @return Current selection.
 */
static inline sim_clk32k_sel_src_t SIM_HAL_GetClock32kSrc(SIM_Type* base)
{
    uint32_t regValue = base->LPOCLKS;
    regValue = (regValue & SIM_LPOCLKS_CLK32KSEL_MASK) >> SIM_LPOCLKS_CLK32KSEL_SHIFT;
    return (sim_clk32k_sel_src_t)regValue;
}


/*!
 * @brief Set the clock selection of LPOCLKSEL.
 *
 * This function sets the clock selection of LPOCLKSEL.
 *
 * @param base Base address for current SIM instance.
 * @param setting  The value to set.
 * @note This function ignores initialize member
 */
static inline void SIM_HAL_SetLpoClocks(SIM_Type* base, sim_lpo_clock_config_t setting)
{
    uint32_t regValue = base->LPOCLKS;

    regValue &= ~( SIM_LPOCLKS_LPO1KCLKEN_MASK  |
                   SIM_LPOCLKS_LPO32KCLKEN_MASK |
                   SIM_LPOCLKS_LPOCLKSEL_MASK   |
                   SIM_LPOCLKS_CLK32KSEL_MASK   );

    regValue |= SIM_LPOCLKS_LPO1KCLKEN(setting.enableLpo1k);
    regValue |= SIM_LPOCLKS_LPO32KCLKEN(setting.enableLpo32k);
    regValue |= SIM_LPOCLKS_LPOCLKSEL(setting.sourceLpoClk);
    regValue |= SIM_LPOCLKS_CLK32KSEL(setting.sourceClk32k);

    /* Write value to register. */
    base->LPOCLKS = regValue;
}

/*!
 * @brief Get the clock selection of LPOCLKSEL.
 *
 * This function gets the clock selection of LPOCLKSEL.
 *
 * @param base Base address for current SIM instance.
 * @return Current selection.
 */
static inline sim_lpoclk_sel_src_t SIM_HAL_GetLpoClkSrc(SIM_Type* base)
{
    uint32_t regValue = base->LPOCLKS;
    regValue = (regValue & SIM_LPOCLKS_LPOCLKSEL_MASK) >> SIM_LPOCLKS_LPOCLKSEL_SHIFT;
    return (sim_lpoclk_sel_src_t)regValue;
}

/*!
 * @brief Gets the 32 kHz LPO clock Control.
 *
 * This function  gets the 32 kHz LPO clock enable setting.
 *
 * @param base     Base address for current SIM instance.
 * @return Current selection.
 */
static inline bool SIM_HAL_GetLpo32kClkEnCmd(SIM_Type* base)
{
    return (bool)BITBAND_ACCESS32(&(base->LPOCLKS), SIM_LPOCLKS_LPO32KCLKEN_SHIFT);
}

/*!
 * @brief Gets the 1 kHz LPO clock Control.
 *
 * This function  gets the 1 kHz LPO clock enable setting.
 *
 * @param base     Base address for current SIM instance.
 * @return Current selection.
 */
static inline bool SIM_HAL_GetLpo1kClkEnCmd(SIM_Type* base)
{
    return (bool)BITBAND_ACCESS32(&(base->LPOCLKS), SIM_LPOCLKS_LPO1KCLKEN_SHIFT);
}

/*!
 * @brief Get SIM LPO clock frequency (LPO_CLOCK).
 *
 * @param base Register base address for the SIM instance.
 * @return  Clock frequency, if clock is invalid, return 0.
 */
uint32_t SIM_HAL_GetLpoFreq(SIM_Type * base);

/*!
 * @brief Get SIM LPO 32KHz clock frequency (LPO_32K_CLOCK).
 *
 * @param base Register base address for the SIM instance.
 * @return  Clock frequency, if clock is invalid, return 0.
 */
uint32_t SIM_HAL_GetLpo32KFreq(SIM_Type * base);

/*!
 * @brief Get SIM LPO 1KHz clock frequency (LPO_1K_CLOCK).
 *
 * @param base Register base address for the SIM instance.
 * @return  Clock frequency, if clock is invalid, return 0.
 */
uint32_t SIM_HAL_GetLpo1KFreq(SIM_Type * base);

/*!
 * @brief Sets the ADCx pre-trigger select setting.
 *
 * This function selects the ADCx pre-trigger source.
 *
 * @param base     Base address for current SIM instance.
 * @param instance device instance.
 * @param select   pre-trigger select setting for ADCx
 */
void SIM_HAL_SetAdcPreTriggerMode(SIM_Type * base,
                                  uint32_t instance,
                                  sim_adc_pretrg_sel_t select);

/*!
 * @brief Gets the ADCx pre-trigger select setting.
 *
 * This function  gets the ADCx pre-trigger select setting.
 *
 * @param base     Base address for current SIM instance.
 * @param instance device instance.
 * @return select  ADCax pre-trigger select setting
 */
sim_adc_pretrg_sel_t SIM_HAL_GetAdcPreTriggerMode(SIM_Type * base,
                                                  uint32_t instance);

/*!
 * @brief Sets the ADCx software pre-trigger select setting.
 *
 * This function selects the ADCx software pre-trigger source.
 *
 * @param base     Base address for current SIM instance.
 * @param instance device instance.
 * @param select   pre-trigger select setting for ADCx
 */
void SIM_HAL_SetAdcSwPreTriggerMode(SIM_Type * base,
                                    uint32_t instance,
                                    sim_adc_sw_pretrg_sel_t select);

/*!
 * @brief Gets the ADCx software pre-trigger select setting.
 *
 * This function  gets the ADCx software pre-trigger select setting.
 *
 * @param base     Base address for current SIM instance.
 * @param instance device instance.
 * @return select  ADCx pre-trigger select setting
 */
sim_adc_sw_pretrg_sel_t SIM_HAL_GetAdcSwPreTriggerMode(SIM_Type * base,
                                                    uint32_t instance);

/*!
 * @brief Sets the ADCx trigger select setting.
 *
 * This function  selects the ADCx trigger source
 *
 * @param base     Base address for current SIM instance.
 * @param instance device instance.
 * @param select   trigger select setting for ADCx
*/
void SIM_HAL_SetAdcTriggerMode(SIM_Type * base,
                               uint32_t instance,
                               sim_adc_trg_sel_t select);

/*!
 * @brief Gets the ADCx trigger select setting.
 *
 * This function  gets the ADCx trigger select setting.
 *
 * @param base     Base address for current SIM instance.
 * @param instance device instance.
 * @return ADCx trigger select setting
 */
sim_adc_trg_sel_t SIM_HAL_GetAdcTriggerMode(SIM_Type * base,
                                            uint32_t instance);
 
 /*!
 * @brief Sets the FlexTimer x channel y output source select setting.
 *
 * This function  selects the FlexTimer x channel y output source.
 *
 * @param base      Base address for current SIM instance.
 * @param instance  device instance.
 * @param channel   FlexTimer channel y
 * @param select    FlexTimer x channel y output source
 */
void SIM_HAL_SetFtmChOutSrcMode(SIM_Type * base,
                                uint32_t instance,
                                uint8_t channel,
                                sim_ftm_ch_out_src_t select);

/*!
 * @brief Gets the FlexTimer x channel y output source select setting.
 *
 * This function gets the FlexTimer x channel y output
 * source select setting.
 *
 * @param base      Base address for current SIM instance.
 * @param instance  device instance.
 * @param channel   FlexTimer channel y
 * @return select   FlexTimer x channel y output source select setting
 */
sim_ftm_ch_out_src_t SIM_HAL_GetFtmChOutSrcMode(SIM_Type * base,
                                                uint32_t instance,
                                                uint8_t channel);

/*!
 * @brief Sets the FlexTimer x channel y input source select setting.
 *
 * This function  selects the FlexTimer x channel y input source.
 *
 * @param base      Base address for current SIM instance.
 * @param instance  device instance.
 * @param channel   FlexTimer channel y
 * @param select    FlexTimer x channel y input source
 */
void SIM_HAL_SetFtmChSrcMode(SIM_Type * base,
                             uint32_t instance,
                             uint8_t channel,
                             sim_ftm_ch_src_t select);

/*!
 * @brief Gets the FlexTimer x channel y input source select setting.
 *
 * This function gets the FlexTimer x channel y input source select setting.
 *
 * @param base      Base address for current SIM instance.
 * @param instance  device instance.
 * @param channel   FlexTimer channel y
 * @return select   FlexTimer x channel y input source select setting
 */
sim_ftm_ch_src_t SIM_HAL_GetFtmChSrcMode(SIM_Type * base,
                                         uint32_t instance,
                                         uint8_t channel);
 
/*!
 * @brief Set FlexTimer x hardware trigger 0 software synchronization.
 *
 * This function sets FlexTimer x hardware trigger 0 software synchronization.
 * FTMxSYNCBIT.
 *
 * @param base      Base address for current SIM instance.
 * @param instance  device instance.
 * @param sync      Synchronize or not.
 */
void SIM_HAL_SetFtmSyncCmd(SIM_Type * base, uint32_t instance, bool sync);

/*!
 * @brief Get FlexTimer x hardware trigger software synchronization setting.
 *
 * This function gets FlexTimer x hardware trigger software synchronization.
 * FTMxSYNCBIT.
 *
 * @param base       Base address for current SIM instance.
 * @param instance   device instance.
 * @return enable    hardware trigger software synchronization setting
 */
static inline bool SIM_HAL_GetFtmSyncCmd(SIM_Type * base, uint32_t instance)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
#endif
    return (bool)(base->FTMOPT1 & (1U<<instance));
}

/*!
 * @brief Gets the Family ID in System Device ID register (SIM_SDID).
 *
 * This function  gets the Family ID in System Device ID register.
 *
 * @param base     Base address for current SIM instance.
 * @return id      Family ID
 */
static inline uint32_t SIM_HAL_GetFamId(SIM_Type * base)
{
    uint32_t regValue = base->SDID;
    regValue = (regValue & SIM_SDID_FAMILYID_MASK) >> SIM_SDID_FAMILYID_SHIFT;
    return regValue;
}

/*!
 * @brief Gets the Sub-Family ID in the System Device ID register (SIM_SDID).
 *
 * This function  gets the Sub-Family ID in System Device ID register.
 *
 * @param base     Base address for current SIM instance.
 * @return id      Sub-Family ID
 */
static inline uint32_t SIM_HAL_GetSubFamilyId(SIM_Type * base)
{
    uint32_t regValue = base->SDID;
    regValue = (regValue & SIM_SDID_SUBFAMID_MASK) >> SIM_SDID_SUBFAMID_SHIFT;
    return regValue;
}

/*!
 * @brief Gets the SeriesID in the System Device ID register (SIM_SDID).
 *
 * This function  gets the Series ID in System Device ID register.
 *
 * @param base  Base address for current SIM instance.
 * @return id   Series ID
 */
static inline uint32_t SIM_HAL_GetSeriesId(SIM_Type * base)
{
    uint32_t regValue = base->SDID;
    regValue = (regValue & SIM_SDID_SERIESID_MASK) >> SIM_SDID_SERIESID_SHIFT;
    return regValue;
}

/*!
 * @brief Gets RAM size.
 *
 * This function gets the RAM size. The field specifies the amount of system RAM
 * available on the device.
 *
 * @param base Base address for current SIM instance.
 * @return size  RAM size on the device
 */
static inline uint32_t SIM_HAL_GetRamSize(SIM_Type * base)
{
    uint32_t regValue = base->SDID;
    regValue = (regValue & SIM_SDID_RAMSIZE_MASK) >> SIM_SDID_RAMSIZE_SHIFT;
    return regValue;
}

/*!
 * @brief Gets the Revision ID in the System Device ID register (SIM_SDID).
 *
 * This function  gets the Revision ID in System Device ID register.
 *
 * @param base Base address for current SIM instance.
 * @return id  Revision ID
 */
static inline uint32_t SIM_HAL_GetRevId(SIM_Type * base)
{
    uint32_t regValue = base->SDID;
    regValue = (regValue & SIM_SDID_REVID_MASK) >> SIM_SDID_REVID_SHIFT;
    return regValue;
}

/*!
 * @brief Gets the Project Count ID in System Device ID register (SIM_SDID).
 *
 * This function  gets the Project Count ID in System Device ID register.
 *
 * @param base     Base address for current SIM instance.
 * @return id      Project Count ID
 */
static inline uint32_t SIM_HAL_GetProjectId(SIM_Type * base)
{
    uint32_t regValue = base->SDID;
    regValue = (regValue & SIM_SDID_PROJECTID_MASK) >> SIM_SDID_PROJECTID_SHIFT;
    return regValue;
}

/*!
 * @brief Gets the Pin Count ID in System Device ID register (SIM_SDID).
 *
 * This function  gets the Pin Count ID in System Device ID register.
 *
 * @param base     Base address for current SIM instance.
 * @return id      Pin Count ID
 */
static inline uint32_t SIM_HAL_GetPinCountId(SIM_Type * base)
{
    uint32_t regValue = base->SDID;
    regValue = (regValue & SIM_SDID_PINID_MASK) >> SIM_SDID_PINID_SHIFT;
    return regValue;
}

/*!
 * @brief Set the DMA Clock Gate from the Platform Clock Gating Control Register.
 *
 * This function configures the DMA Clock Gate in the Platform Clock Gating Control Register.
 *
 * @param base     Base address for current SIM instance.
 * @param enable   DMA clock gate enable setting
 */
static inline void SIM_HAL_SetDmaClockGate(SIM_Type* base, bool enable)
{
    BITBAND_ACCESS32(&(base->PLATCGC), SIM_PLATCGC_CGCDMA_SHIFT) = enable;
}

/*!
 * @brief Gets the DMA Clock Gate from the Platform Clock Gating Control Register.
 *
 * This function gets the DMA Clock Gate in the Platform Clock Gating Control Register.
 *
 * @param base     Base address for current SIM instance.
 * @return enable  DMA Clock Gating
 */
static inline bool SIM_HAL_GetDmaClockGate(SIM_Type* base)
{
    return (bool)BITBAND_ACCESS32(&(base->PLATCGC), SIM_PLATCGC_CGCDMA_SHIFT);
}

/*!
 * @brief Configure the MPU Clock Gating from the Platform Clock Gating Control Register.
 *
 * This function configures the MPU Clock Gating in the Platform Clock Gating Control Register.
 *
 * @param base     Base address for current SIM instance.
 * @param enable   MPU clock gate enable setting
 */
static inline void SIM_HAL_SetMpuClockGate(SIM_Type* base, bool enable)
{
    BITBAND_ACCESS32(&(base->PLATCGC), SIM_PLATCGC_CGCMPU_SHIFT) = enable;
}

/*!
 * @brief Gets the MPU Clock Gating from the Platform Clock Gating Control Register.
 *
 * This function gets the MPU Clock Gating in the Platform Clock Gating Control Register.
 *
 * @param base     Base address for current SIM instance.
 * @return enable  MPU Clock Gating
 */
static inline bool SIM_HAL_GetMpuClockGate(SIM_Type* base)
{
    return (bool)BITBAND_ACCESS32(&(base->PLATCGC), SIM_PLATCGC_CGCMPU_SHIFT);
}

/*!
 * @brief Configure the MSCM Clock Gating from the Platform Clock Gating Control Register.
 *
 * This function configures the MSCM Clock Gating in the Platform Clock Gating Control Register.
 *
 * @param base     Base address for current SIM instance.
 * @param enable   MPU clock gate enable setting
 */
static inline void SIM_HAL_SetMscmClockGate(SIM_Type* base, bool enable)
{
    BITBAND_ACCESS32(&(base->PLATCGC), SIM_PLATCGC_CGCMSCM_SHIFT) = enable;
}

/*!
 * @brief Gets the MSCM Clock Gating from the Platform Clock Gating Control Register.
 *
 * This function gets the MSCM Clock Gating in the Platform Clock Gating Control Register.
 *
 * @param base     Base address for current SIM instance.
 * @return enable  MSCM Clock Gating
 */
static inline bool SIM_HAL_GetMscmClockGate(SIM_Type* base)
{
    return (bool)BITBAND_ACCESS32(&(base->PLATCGC), SIM_PLATCGC_CGCMSCM_SHIFT);
}

/*!
 * @brief Gets the FlexNVM size in the Flash Configuration Register 1.
 *
 * This function  gets the FlexNVM size in the Flash Configuration Register 1.
 *
 * @param base     Base address for current SIM instance.
 * @return size FlexNVM Size
 */
static inline uint32_t SIM_HAL_GetFlexnvmSize(SIM_Type* base)
{
    uint32_t regValue = base->FCFG1;
    regValue = (regValue & SIM_FCFG1_NVMSIZE_MASK) >> SIM_FCFG1_NVMSIZE_SHIFT;
    return regValue;
}

/*!
 * @brief Gets the program flash size in the Flash Configuration Register 1 (SIM_FCFG).
 *
 * This function  gets the program flash size in the Flash Configuration Register 1.
 *
 * @param base     Base address for current SIM instance.
 * @return size Program flash Size
 */
static inline uint32_t SIM_HAL_GetProgramFlashSize(SIM_Type * base)
{
    uint32_t regValue = base->FCFG1;
    regValue = (regValue & SIM_FCFG1_PFSIZE_MASK) >> SIM_FCFG1_PFSIZE_SHIFT;
    return regValue;
}

/*!
 * @brief Gets the EEE SRAM size in the Flash Configuration Register 1.
 *
 * This function  gets the EEE SRAM size in the Flash Configuration Register 1.
 *
 * @param base   Base address for current SIM instance.
 * @return size  EEE SRAM size
 */
static inline uint32_t SIM_HAL_GetEepromSize(SIM_Type* base)
{
    uint32_t regValue = base->FCFG1;
    regValue = (regValue & SIM_FCFG1_EEERAMSIZE_MASK) >> SIM_FCFG1_EEERAMSIZE_SHIFT;
    return regValue;
}

/*!
 * @brief Gets the FlexNVM partition in the Flash Configuration Register 1.
 *
 * This function  gets the FlexNVM partition in the Flash Configuration Register 1
 *
 * @param base     Base address for current SIM instance.
 * @return setting FlexNVM partition setting
 */
static inline uint32_t SIM_HAL_GetFlexnvmPartition(SIM_Type* base)
{
    uint32_t regValue = base->FCFG1;
    regValue = (regValue & SIM_FCFG1_DEPART_MASK) >> SIM_FCFG1_DEPART_SHIFT;
    return regValue;
}

/*!
 * @brief Sets the Flash Doze in the Flash Configuration Register 1.
 *
 * This function  sets the Flash Doze in the Flash Configuration Register 1.
 *
 * @param base     Base address for current SIM instance.
 * @param setting Flash Doze setting
 */
static inline void SIM_HAL_SetFlashDoze(SIM_Type * base, bool setting)
{
    BITBAND_ACCESS32(&(base->FCFG1), SIM_FCFG1_FLASHDOZE_SHIFT) = setting;
}

/*!
 * @brief Gets the Flash Doze in the Flash Configuration Register 1.
 *
 * This function  gets the Flash Doze in the Flash Configuration Register 1.
 *
 * @param base     Base address for current SIM instance.
 * @return setting Flash Doze setting
 */
static inline bool SIM_HAL_GetFlashDoze(SIM_Type * base)
{
    return BITBAND_ACCESS32(&(base->FCFG1), SIM_FCFG1_FLASHDOZE_SHIFT);
}

/*!
 * @brief Sets the Flash disable setting.
 *
 * This function  sets the Flash disable setting in the
 * Flash Configuration Register 1.
 *
 * @param base     Base address for current SIM instance.
 * @param disable      Flash disable setting
 */
static inline void SIM_HAL_SetFlashDisableCmd(SIM_Type * base, bool disable)
{
    BITBAND_ACCESS32(&(base->FCFG1), SIM_FCFG1_FLASHDIS_SHIFT) = disable;
}

/*!
 * @brief Gets the Flash disable setting.
 *
 * This function  gets the Flash disable setting in the
 * Flash Configuration Register 1.
 *
 * @param base     Base address for current SIM instance.
 * @return setting Flash disable setting
 */
static inline bool SIM_HAL_GetFlashDisableCmd(SIM_Type * base)
{
    return (bool)BITBAND_ACCESS32(&(base->FCFG1), SIM_FCFG1_FLASHDIS_SHIFT);
}

/*!
 * @brief Gets the Flash maximum address block 0 in the Flash Configuration Register 1  (SIM_FCFG).
 *
 * This function gets the Flash maximum block 0 in Flash Configuration Register 2.
 *
 * @param base     Base address for current SIM instance.
 * @return address Flash maximum block 0 address
 */
static inline uint32_t SIM_HAL_GetFlashMaxAddrBlock0(SIM_Type * base)
{
    uint32_t regValue = base->FCFG2;
    regValue = (regValue & SIM_FCFG2_MAXADDR0_MASK) >> SIM_FCFG2_MAXADDR0_SHIFT;
    return regValue;
}

/*!
 * @brief Gets the Flash maximum address block 1 in Flash Configuration Register 2.
 *
 * This function  gets the Flash maximum block 1 in Flash Configuration Register 1.
 *
 * @param base     Base address for current SIM instance.
 * @return address Flash maximum block 0 address
 */
static inline uint32_t SIM_HAL_GetFlashMaxAddrBlock1(SIM_Type * base)
{
    uint32_t regValue = base->FCFG2;
    regValue = (regValue & SIM_FCFG2_MAXADDR1_MASK) >> SIM_FCFG2_MAXADDR1_SHIFT;
    return regValue;
}

/*!
 * @brief Gets the UID127_96 from Unique Identification Register High.
 *
 * This function gets the UID127_96 from Unique Identification Register High.
 *
 * @param base     Base address for current SIM instance.
 * @return setting UID127_96
 */
static inline uint32_t SIM_HAL_GetUniqueIdentificationHigh(SIM_Type * base)
{
    return base->UIDH;
}

/*!
 * @brief Gets the UID95_64 from Unique Identification Register Mid High.
 *
 * This function gets the UID95_64 from Unique Identification Register Mid High.
 *
 * @param base     Base address for current SIM instance.
 * @return setting UID95_64
 */
static inline uint32_t SIM_HAL_GetUniqueIdentificationMidHigh(SIM_Type * base)
{
    return base->UIDMH;
}

/*!
 * @brief Gets the UID63_32 from Unique Identification Register Mid Low.
 *
 * This function gets the UID63_32 from Unique Identification Register Mid Low.
 *
 * @param base     Base address for current SIM instance.
 * @return setting UID63_32
 */
static inline uint32_t SIM_HAL_GetUniqueIdentificationMidLow(SIM_Type * base)
{
    return base->UIDML;
}

/*!
 * @brief Gets the UID31_0 from Unique Identification Register Low.
 *
 * This function gets the UID31_0 from Unique Identification Register Low.
 *
 * @param base     Base address for current SIM instance.
 * @return setting UID31_0
 */
static inline uint32_t SIM_HAL_GetUniqueIdentificationLow(SIM_Type * base)
{
    return base->UIDL;
}

/*!
 * @brief Get the default Debug Trace clock configuration.
 *
 * This function gets the default Debug Trace clock configuration.
 *
 * @param config Pointer to the configuration structure.
 */
void SIM_HAL_GetTraceClockDefaultConfig(sim_trace_clock_config_t *config);

/*!
 * @brief Initialize SIM Debug Trace.
 *
 * This function enables the SIM Debug Trace clock according to the
 * configuration.
 *
 * @param base Register base address for the SIM instance.
 * @param config   Pointer to the configuration structure.
 *
 */
void SIM_HAL_InitTraceClock(SIM_Type * base,
                            const sim_trace_clock_config_t *config);

/*!
 * @brief De-initialize SIM Debug Trace.
 *
 * This function disables the SIM Debug Trace clock.
 *
 * @param base Register base address for the SIM instance.
 *
 */
static inline void SIM_HAL_DeinitTraceClock(SIM_Type * base)
{
    /* Disable divider. */
    BITBAND_ACCESS32(&(base->CLKDIV4), SIM_CLKDIV4_TRACEDIVEN_SHIFT) = false;
}

/*!
 * @brief Sets the Software Trigger bit to TRGMUX setting.
 *
 * This function sets the Software Trigger bit to TRGMUX in Miscellaneous Control register.
 *
 * @param base     Base address for current SIM instance.
 * @param disable  Software Trigger bit
 */
static inline void SIM_HAL_SetSwTriggerTrgmux(SIM_Type * base, bool disable)
{
    BITBAND_ACCESS32(&(base->MISCTRL), SIM_MISCTRL_SW_TRG_SHIFT) = disable;
}

/*!
 * @brief Gets the Software Trigger bit to TRGMUX.
 *
 * This function gets the Software Trigger bit to TRGMUX in Miscellaneous Control register.
 *
 * @param base     Base address for current SIM instance.
 * @return setting Software Trigger bit
 */
static inline uint32_t SIM_HAL_GetSwTriggerTrgmux(SIM_Type * base)
{
    return BITBAND_ACCESS32(&(base->MISCTRL), SIM_MISCTRL_SW_TRG_SHIFT);
}

/*!
 * @brief Sets the TClk Frequency
 *
 * This function sets the TClk Frequency.
 *
 * @param base      Base address for current SIM instance.
 * @param index     Index of the TClk.
 * @param frequency The frequency of the specified TClk
 */
static inline void SIM_HAL_SetTClkFreq(SIM_Type * base, uint8_t index, uint32_t frequency)
{
    (void) base;
    if(index <NUMBER_OF_TCLK_INPUTS) {
        g_TClkFreq[index] = frequency;
    }
}

/*!
 * @brief Gets the TClk Frequency
 *
 * This function gets the TClk Frequency.
 *
 * @param base       Base address for current SIM instance.
 * @param index      Index of the TClk.
 * @return frequency The configured frequency of the specified TClk
 */
static inline uint32_t SIM_HAL_GetTClkFreq(SIM_Type * base, uint8_t index)
{
    (void) base;
    if(index <NUMBER_OF_TCLK_INPUTS) {
        return g_TClkFreq[index];
    }

    return 0U;
}

#if defined(__cplusplus)
}
#endif /* __cplusplus*/


/*! @}*/

#endif /* __FSL_SIM_HAL_S32K144_H__ */
/*******************************************************************************
 * EOF
 ******************************************************************************/

