/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
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

#ifndef __FSL_LPIT_HAL_H__
#define __FSL_LPIT_HAL_H__

#include "fsl_device_registers.h"
#include <stdbool.h>

/*!
 * @addtogroup lpit_hal
 * @{
 */

/*******************************************************************************
 * Definitions
 *****************************************************************************/


/*! @brief Mode options available for the LPIT timer. */
typedef enum _lpit_timer_modes {
    LPIT_PERIODIC_COUNTER      = 0x00U,  /*!< Use the all 32-bits, counter loads and decrements to zero */
    LPIT_DUAL_PERIODIC_COUNTER = 0x01U,  /*!< Counter loads, lower 16-bits  decrement to zero, then
                                              upper 16-bits  decrement */
    LPIT_TRIGGER_ACCUMULATOR   = 0x02U,  /*!< Counter loads on first trigger and decrements on each trigger */
    LPIT_INPUT_CAPTURE         = 0x03U   /*!< Counter  loads with 0xFFFFFFFF, decrements to zero. It stores
                                              the inverse of the current value when a input trigger is detected */
} lpit_timer_modes_t;

/*!
 * @brief Trigger source options.
 *
 * This is used for both internal and external trigger sources. The actual trigger options
 * available is SoC specific, user should refer to the reference manual.
 */
typedef enum _lpit_trigger_source {
    LPIT_TRIGGER_SOURCE_EXTERNAL = 0x00U, /*!< Use external trigger input */
    LPIT_TRIGGER_SOURCE_INTERNAL = 0x01U  /*!< Use internal trigger */
}  lpit_trigger_source_t;

/*! @brief Informations hardware of LPIT module. */
typedef struct lpitModuleInformation {
    uint32_t majorVersionNumber;                /*!< The major version number for the LPIT module specification */
    uint32_t minorVersionNumber;                /*!< The minor version number for the LPIT module specification */
    uint32_t featureNumber;                     /*!< The feature set number */
    uint32_t numberOfExternalTriggerInputs;     /*!< Number of external triggers implemented */
    uint32_t numberOfTimerChannels;             /*!< Number of timer channels implemented.*/
} lpit_module_information_t;
/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
  @name Timer Initialization
 * @{
 */

/*!
 * @brief Get the information of LPIT module.
 *
 * This function return the information of LPIT module, the information consists major version number,
 * minor version number, feature number, number of external trigger inputs, number of timer channels.
 *
 * @param base LPIT peripheral base address
 * @param moduleInfomation Pointer to lpit_module_information_t structure.
 */
void LPIT_HAL_GetModuleInformation(LPIT_Type * base, lpit_module_information_t * moduleInfomation);

/*!
 * @brief Enables the LPIT module.
 *
 * This function enables the functional clock of LPIT module (Note: this function does not un-gate
 * the system clock gating control). It should be called before any other timer
 * related setup.
 *
 * @param base LPIT peripheral base address
 */
static inline void LPIT_HAL_Enable(LPIT_Type * base)
{
    BITBAND_ACCESS32(&(base->MCR), LPIT_MCR_M_CEN_SHIFT) =(uint32_t)0x1U;
}

/*!
 * @brief Disables the LPIT module.
 *
 * This function disables functional clock of LPIT module (Note: it does not affect the
 * SIM clock gating control).
 *
 * @param base LPIT peripheral base address
 */
static inline void LPIT_HAL_Disable(LPIT_Type * base)
{
    BITBAND_ACCESS32(&(base->MCR), LPIT_MCR_M_CEN_SHIFT) = (uint32_t)0x0U;
}

/*!
 * @brief Reset the LPIT module.
 *
 * This function Reset the timer channels and registers except the MCR register.
 *
 * @param base LPIT peripheral base address
 */
static inline void LPIT_HAL_Reset(LPIT_Type *base)
{
    BITBAND_ACCESS32(&(base->MCR), LPIT_MCR_SW_RST_SHIFT) = (uint32_t)0x1U;
    BITBAND_ACCESS32(&(base->MCR), LPIT_MCR_SW_RST_SHIFT) = (uint32_t)0x0U;
}

/*! @} */

/*!
  @name Timer Start and Stop
 * @{
 */

/*!
  @brief Starts the timer channel counting.
 *
 * This function allows starting timer channels simultaneously.
 * After calling this function, timer channels are going operate depend on mode of them.
 * For example: In compare modes, the timer channels load period value and decrement.
 * Each time a timer channel reaches 0,it generates a trigger pulse , timeout pulse and sets
 * the timeout interrupt flag.
 *
 * @param base LPIT peripheral base address
 * @param mask: Timer channels start mask, mask decides which channels to be started
 *             For example: with mask = 0x01U then channel 0 will be started
 *                          with mask = 0x02U then channel 1 will be started
 *                          with mask = 0x03U then channel 0 and channel 1 will be started
 */
static inline void LPIT_HAL_StartTimerChannels(LPIT_Type * base, uint32_t mask)
{
    base->SETTEN |= mask;
}

/*!
 * @brief Stops the timer channel from counting.
 *
 * This function allows stop timer channels simultaneously. Timer channels reload their periods
 * respectively after they call the LPIT_HAL_StartTimerChannels the next time.
 *
 * @param base LPIT peripheral base address
 * @param mask Timer channels stop mask, mask decides which channels to be stop
 *                For example: with mask = 0x01U then channel 0 will stop
 *                             with mask = 0x02U then channel 1 will stop
 *                             with mask = 0x03U then channel 0 and channel 1 will stop
 */
static inline void LPIT_HAL_StopTimerChannels(LPIT_Type * base, uint32_t mask)
{
    base->CLRTEN |= mask;
}

/*!
 * @brief Checks to see whether the current timer channel is running or not.
 *
 * @param base LPIT peripheral base address
 * @param channel Timer channel number
 * @return Current timer channel running status
 *         -true: Current timer channel is running
 *         -false: Current timer channel is not running
 */
static inline bool LPIT_HAL_IsTimerChannelRunning(LPIT_Type * base, uint32_t channel)
{
    return (bool)BITBAND_ACCESS32(&(base->SETTEN), channel);
}

/*! @} */

/*!
 * @name Timer Period
 * @{
 */

/*!
 * @brief Sets the timer channel period in count unit.
 *
 * Timer channels begin counting from the value set by this function.
 * The counter period of a running timer channel can be modified by first
 * setting a new load value, the value will be loaded after the timer expires.
 * To abort the current cycle and start a timer channel period with the new value,
 * the timer channel must be disabled and enabled again.
 *
 * @param base LPIT peripheral base address
 * @param channel Timer channel number
 * @param count Timer channel period in count unit
 */
static inline void LPIT_HAL_SetTimerPeriodByCount(LPIT_Type * base, uint32_t channel, uint32_t count)
{
    base->TVAL_CVAL_TCTRL[channel].TVAL = count;
}

/*!
 * @brief Returns the current timer channel period in count unit.
 *
 * @param base LPIT peripheral base address
 * @param channel Timer channel number
 * @return Timer channel period in count unit
 */
static inline uint32_t LPIT_HAL_GetTimerPeriodByCount(LPIT_Type * base, uint32_t channel)
{
    return (base->TVAL_CVAL_TCTRL[channel].TVAL);
}

/*!
 * @brief Reads the current timer channel counting value.
 *
 * This function returns the real-time timer counting value, in a range from 0 to a
 * timer channel period.
 *
 * @param base LPIT peripheral base address
 * @param channel Timer channel number
 * @return Current timer channel counting value
 */
static inline uint32_t LPIT_HAL_GetCurrentTimerCount(LPIT_Type * base, uint32_t channel)
{
    return (base->TVAL_CVAL_TCTRL[channel].CVAL);
}

/*! @} */

/*!
 * @name Timer Interrupt
 * @{
 */

/*!
 * @brief Enables the interrupt for timer channels.
 *
 * This function allows enabling interrupt for timer channels simultaneously.
 * @param base LPIT peripheral base address
 * @param mask The interrupts enable mask, mask decides which channel is be enabled interrupt
 *             For example: with mask = 0x01u then will enable interrupt for channel 0 only
 *                          with mask = 0x02u then will enable interrupt for channel 1 only
 *                          with mask = 0x03u then will enable interrupt for channel 0 and channel 1
 */
static inline void LPIT_HAL_EnableInterruptTimerChannels(LPIT_Type * base, uint32_t mask)
{
    base->MIER |= mask;
}

/*!
 * @brief Disables the interrupt for timer channels.
 *
 * This function allows disabling interrupts for timer channels simultaneously.
 * @param base LPIT peripheral base address
 * @param mask The interrupts disable mask, mask decides which channel is be disabled interrupt
 *             For example: with mask = 0x01u then will disable interrupt for channel 0 only
 *                          with mask = 0x02u then will disable interrupt for channel 1 only
 *                          with mask = 0x03u then will disable interrupt for channel 0 and channel 1
 */
static inline void LPIT_HAL_DisableInterruptTimerChannels(LPIT_Type * base, uint32_t mask)
{
    base->MIER &= ~mask;
}

/*!
 * @brief Gets the interrupt flag for timer channels.
 *
 * This function reads current interrupt flag of timer channels.
 * @param base LPIT peripheral base address
 * @param mask The interrupt flag get mask, mask decides which channels be get interrupt flag
 *             For example: with mask = 0x01u then the interrupt flag of channel 0 only will be get
 *                          with mask = 0x02u then the interrupt flag of channel 1 only will be get
 *                          with mask = 0x03u then the interrupt flags of channel 0 and channel 1 will be get
 * @return The interrupt flag of timer channels.
 */
static inline uint32_t LPIT_HAL_GetInterruptFlagTimerChannels(LPIT_Type * base, uint32_t mask)
{
    return (base->MSR) & mask;
}

/*!
 * @brief Clears the interrupt flag for timer channels.
 *
 * This function clears current interrupt flag of timer channels.
 * @param base LPIT peripheral base address
 * @param mask The interrupt flag clear mask, mask decides which channels be clear interrupt flag
 *             For example: with mask = 0x01u then the interrupt flag of channel 0 only will be cleared
 *                          with mask = 0x02u then the interrupt flag of channel 1 only will be cleared
 *                          with mask = 0x03u then the interrupt flags of channel 0 and channel 1 will be cleared
 */
static inline void LPIT_HAL_ClearInterruptFlagTimerChannels(LPIT_Type * base, uint32_t mask)
{
    /* Write 1 will clear the flag. */
    base->MSR = mask;
}

/*! @} */

/*!
 * @name Timer Configuration
 * @{
 */

/*!
 * @brief Set mode operation of timer channel
 *
 * @param base LPIT peripheral base address
 * @param channel Timer channel number
 * @param mode Mode operation of timer channel, it is member of lpit_timer_modes_t
 */
static inline void LPIT_HAL_SetTimerChannelModeCmd(LPIT_Type *base, uint32_t channel, lpit_timer_modes_t mode)
{
    base->TVAL_CVAL_TCTRL[channel].TCTRL &= ~LPIT_TCTRL_MODE_MASK;
    base->TVAL_CVAL_TCTRL[channel].TCTRL |=  LPIT_TCTRL_MODE(mode);
}

/*!
 * @brief Get current mode of timer channel.
 *
 * @param base LPIT peripheral base address
 * @param channel Timer channel number
 * @return mode of timer channel.It is one of lpit_timer_modes_t
 */
static inline lpit_timer_modes_t LPIT_HAL_GetTimerChannelModeCmd(LPIT_Type *base, uint32_t channel)
{
    return (lpit_timer_modes_t)(((base->TVAL_CVAL_TCTRL[channel].TCTRL) & LPIT_TCTRL_MODE_MASK) >> LPIT_TCTRL_MODE_SHIFT);
}

/*!
 * @brief Set internal trigger source for timer channel
 *
 * @param base LPIT peripheral base address
 * @param channel Timer channel number
 * @param triggerChannelSelect: number of channel which is select to be trigger source
 */
static inline void LPIT_HAL_SetTriggerSelectCmd(LPIT_Type *base, uint32_t channel, uint32_t triggerChannelSelect)
{
    base->TVAL_CVAL_TCTRL[channel].TCTRL &= ~LPIT_TCTRL_TRG_SEL_MASK;
    base->TVAL_CVAL_TCTRL[channel].TCTRL |=  LPIT_TCTRL_TRG_SEL(triggerChannelSelect);
}

/*!
 * @brief Set trigger source for timer channel is Internal/External.
 *
 * @param base LPIT peripheral base address
 * @param channel Timer channel number
 * @param triggerSource Trigger source is external or internal
 *        - true : trigger source is internal
 *        - false: trigger source is external
*/
static inline void LPIT_HAL_SetTriggerSourceCmd(LPIT_Type *base, uint32_t channel, lpit_trigger_source_t triggerSource)
{
    BITBAND_ACCESS32(&(base->TVAL_CVAL_TCTRL[channel].TCTRL), LPIT_TCTRL_TRG_SRC_SHIFT) = (uint32_t)triggerSource;
}

/*!
 * @brief Set timer channel reload /not reload on trigger.
 *
 * @param base LPIT peripheral base address
 * @param channel Timer channel number
 * @param isReloadOnTrigger Timer channel reload on trigger or not
 *        - true : timer channel will reload on trigger
 *        - false : timer channel will not reload on trigger
 */
static inline void LPIT_HAL_SetReloadOnTriggerCmd(LPIT_Type *base, uint32_t channel, bool isReloadOnTrigger)
{
    BITBAND_ACCESS32(&(base->TVAL_CVAL_TCTRL[channel].TCTRL), LPIT_TCTRL_TROT_SHIFT) = (uint32_t)isReloadOnTrigger;
}

/*!
 * @brief Set timer channel stop /not stop after interrupt.
 *
 * @param base LPIT peripheral base address
 * @param channel Timer channel number
 * @param isStopOnInterrupt Timer channel stop on interrupt or not
 *        - true : Timer channel will stop when interrupt flag assert
 *        - false : Timer channel do not stop when interrupt flag assert
 */
static inline void LPIT_HAL_SetStopOnInterruptCmd(LPIT_Type *base, uint32_t channel, bool isStopOnInterrupt)
{
    BITBAND_ACCESS32(&(base->TVAL_CVAL_TCTRL[channel].TCTRL), LPIT_TCTRL_TSOI_SHIFT) = (uint32_t)isStopOnInterrupt;
}

/*!
 * @brief Set timer channel start /do not start on trigger.
 *
 * @param base LPIT peripheral base address
 * @param channel Timer channel number
 * @param isStartOnTrigger Timer channel start on trigger or not
 *        - true : Timer channel starts to decrement when rising edge on selected trigger is detected
 *        - false : Timer channel starts to decrement immediately based on restart condition (controlled by TSOI bit)
 */
static inline void LPIT_HAL_SetStartOnTriggerCmd(LPIT_Type *base, uint32_t channel, bool isStartOnTrigger)
{
    BITBAND_ACCESS32(&(base->TVAL_CVAL_TCTRL[channel].TCTRL), LPIT_TCTRL_TSOT_SHIFT) = (uint32_t)isStartOnTrigger;
}

/*!
 * @brief Set timer channel is chain or not chain mode.
 *
 * @param base LPIT peripheral base address
 * @param channel Timer channel number(Note: channel 0 cannot be chain)
 * @param isChannelInChain Timer channel is chain or not chain
 *        - true : Timer channel is chain. Timer channel decrements on previous channel's timeout
 *        - false : Timer channel is not chain. Timer channel runs independently
 */
static inline void LPIT_HAL_SetTimerChannelChainCmd(LPIT_Type *base, uint32_t channel, bool isChannelInChain)
{
    BITBAND_ACCESS32(&(base->TVAL_CVAL_TCTRL[channel].TCTRL), LPIT_TCTRL_CHAIN_SHIFT) = (uint32_t)isChannelInChain;
}

/*!
 * @brief Configures the timer channels to continue running or to stop in debug mode.
 *
 * In debug mode, the timer channels may or may not be frozen, based on the configuration of
 * this function. This is intended to aid software development, allowing the developer
 * to halt the processor, investigate the current state of the system (for example,
 * the timer channel values), and continue the operation.
 *
 * @param base LPIT peripheral base address
 * @param isRunInDebug Timer channels run or stop in debug mode
 *        - true: Timer channels continue to run in debug mode
 *        - false: Timer channels stop in debug mode
 */
static inline void LPIT_HAL_SetTimerRunInDebugCmd(LPIT_Type *base, bool  isRunInDebug)
{
    BITBAND_ACCESS32(&(base->MCR), LPIT_MCR_DBG_EN_SHIFT) = (uint32_t)isRunInDebug;
}

/*!
 * @brief Configures the timer channels to continue running or to stop in DOZE mode.
 *
 * In DOZE mode, the timer channels may or may not be frozen, based on the configuration of
 * this function.
 *
 * @param base LPIT peripheral base address
 * @param isRunInDoze Timer channels run or stop in DOZE mode
 *        - true: Timer channels continue to run in DOZE mode
 *        - false: Timer channels stop in DOZE mode
 */
static inline void LPIT_HAL_SetTimerRunInDozeCmd(LPIT_Type *base, bool  isRunInDoze)
{
    BITBAND_ACCESS32(&(base->MCR), LPIT_MCR_DOZE_EN_SHIFT) = (uint32_t)isRunInDoze;
}

/*! @} */

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* __FSL_LPIT_HAL_H__*/
/*******************************************************************************
 * EOF
 *******************************************************************************/
