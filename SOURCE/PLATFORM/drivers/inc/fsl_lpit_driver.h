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
#ifndef __FSL_LPIT_DRIVER_H__
#define __FSL_LPIT_DRIVER_H__

#include "fsl_lpit_hal.h"
#include "fsl_interrupt_manager.h"
#include "fsl_clock_manager.h"

/*!
 * @addtogroup lpit_driver
 * @{
 */

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief Table of base addresses for LPIT instances. */
extern LPIT_Type * const g_lpitBase[];
/* Table to save lpit IRQ enumeration numbers defined in the CMSIS header file */
extern const IRQn_Type g_lpitIrqId[];
/*! @brief Table to save LPIT indexes in PCC register map for clock configuration */
extern const clock_names_t g_lpitClkNames[LPIT_INSTANCE_COUNT];
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief Max period in count of all mode operation except for dual 16 bit periodic counter mode */
#define MAX_PERIOD_COUNT                    0xFFFFFFFFU
/*! @brief Max period in count of dual 16 bit periodic counter mode */
#define MAX_PERIOD_COUNT_IN_DUAL_16BIT_MODE 0x1FFFEU

/*! @brief Error codes for LPIT driver. */
typedef enum _lpit_status {
    LPIT_STATUS_SUCCESS        = 0x00U,        /*!< Operation successful */
    LPIT_STATUS_FAIL           = 0x01U,        /*!< Operation failed */
    LPIT_STATUS_VALID_PERIOD   = 0x02U,        /*!< Input period of timer channel is valid */
    LPIT_STATUS_INVALID_PERIOD = 0x03U         /*!< Input period of timer channel is invalid */
} lpit_status_t;

/*!
* @brief LPIT configuration structure
*
* This structure holds the configuration settings for the LPIT peripheral to
* enable or disable LPIT module in DEBUG and DOZE mode
*/
typedef struct _lpit_config {
    bool enableRunInDebug; /*!< true: Timer channels run in debug mode;
                                false: Timer channels stop in debug mode */
    bool enableRunInDoze;  /*!< true: Timer channels run in doze mode;
                                false: Timer channels stop in doze mode */
} lpit_user_config_t;


/*! @brief Structure to configure the channel timer. */
typedef struct _lpitUserChannelConfig {
    bool chainChannel;                   /*!< true: Timer channel chained to previous timer channel;
                                              false: Timer channel not chained */
    bool isInterruptEnabled;             /*!< true: Timer channel interrupt occur on timeout;
                                              false : Timer channel interrupt don't occur on timeout.*/
    lpit_timer_modes_t timerMode;        /*!< Timer channel mode of operation. */
    lpit_trigger_source_t triggerSource; /*!< Decides to choose to use external or internal trigger. */
    uint32_t triggerSelect;              /*!< Trigger selection for the timer */
    bool enableReloadOnTrigger;          /*!< true: Timer channel reloads when a trigger is detected;
                                              false: No effect */
    bool enableStopOnInterrupt;          /*!< true: Timer channel will stop when timeout;
                                              false: does not stop when timeout */
    bool enableStartOnTrigger;           /*!< true: Timer channel starts when a trigger is detected;
                                              false: decrement immediately */
    uint32_t periodUs;                   /*!< Period of timer channel in microsecond unit */

} lpit_user_channel_config_t;


/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Initialization and De-initialization
 * @{
 */

/*!
 * @brief Initializes the LPIT module.
 *
 * Call this function before calling all the other LPIT driver functions.
 * This function enables the LPIT module, resets LPIT module, configures LPIT module operation in
 * Debug and DOZE mode. The lpit_user_config_t structure passed into function affects all timer channels.
 *
 * @param instance LPIT module instance number.
 * @param config Pointer to lpit_user_config_t structure.
 *        config.enableRunInDebug = true Timers continue to run in Debug mode.
 *        config.enableRunInDebug = false Timers stop in Debug mode.
 *        config.enableRunInDoze= true Timers continue to run in Doze mode.
 *        config.enableRunInDoze= false Timers stop in Doze mode.
 * @return Fail or success status returned by API.
 */
lpit_status_t LPIT_DRV_Init(uint32_t instance, const lpit_user_config_t *config);

/*!
 * @brief Disables the LPIT module.
 *
 * This function disables LPIT module.
 * LPIT_DRV_Init must be called if you want to use LPIT module again.
 *
 * @param instance LPIT module instance number
 * @return Fail or success status returned by API
 */
lpit_status_t LPIT_DRV_Deinit(uint32_t  instance);

/*!
 * @brief Initializes the LPIT channel.
 *
 * This function initializes the LPIT timers by using a channel, this function configures timer mode,
 * trigger source, trigger select, timer channel period, interrupt enable and control bits
 * Pass in the timer channel number and its configuration structure.
 * Timer channels do not start counting by default after calling this function.
 * The function LPIT_DRV_StartTimerChannels must be called to start the timer channel counting.
 * Call the LPIT_DRV_SetTimerPeriodByUs to re-set the period.
 *
 * This is an example demonstrating how to define a LPIT channel configuration structure:
   @code
   lpit_user_channel_config_t pitTestInit = {
        .chainChannel = false,
        .isInterruptEnabled = true,
        .timerMode = LPIT_PERIODIC_COUNTER,
        .triggerSelect = 0x01U,
        .triggerSource = LPIT_TRIGGER_SOURCE_INTERNAL,
        .enableReloadOnTrigger = false,
        .enableStopOnTimeout = false,
        .enableStartOnTrigger = false,
        .periodUs = 1000000
   };
   @endcode
 *
 * @param instance LPIT module instance number
 * @param channel Timer channel number
 * @param userChannelConfig Pointer to LPIT channel configuration structure
 * @return Fail or success status returned by API
 */
lpit_status_t LPIT_DRV_InitChannel(uint32_t instance, uint32_t channel, const lpit_user_channel_config_t * userChannelConfig);

/* @} */

/*!
 * @name Timer Start and Stop
 * @{
 */

/*!
 * @brief Starts the timer channel counting.
 *
 * This function allows starting timer channels simultaneously .
 * After calling this function, timer channels are going operate depend on mode of them.
 * For example: In compare modes, the timer channels load period value and decrement.
 * Each time a timer channel reaches 0,it generates a trigger pulse , timeout pulse and sets
 * the timeout interrupt flag.
 *
 * @param instance LPIT module instance number
 * @param mask Timer channels start mask, mask decides which channels to be started
 *             For example: with mask = 0x01U then channel 0 will be started
 *                          with mask = 0x02U then channel 1 will be started
 *                          with mask = 0x03U then channel 0 and channel 1 will be started
 */
void LPIT_DRV_StartTimerChannels(uint32_t instance, uint32_t mask);

/*!
 * @brief Stops the timer channel counting.
 *
 * This function allows stop timer channels simultaneously from counting. Timer channels reload
 * their periods respectively after the next time they call the LPIT_DRV_StartTimerChannels.
 *
 * @param instance LPIT module instance number
 * @param mask Timer channels stop mask, mask decides which channels to be stop
 *                For example: with mask = 0x01U then channel 0 will stop
 *                             with mask = 0x02U then channel 1 will stop
 *                             with mask = 0x03U then channel 0 and channel 1 will stop
 */
void LPIT_DRV_StopTimerChannels(uint32_t instance, uint32_t mask);

/* @} */

/*!
 * @name Timer Period
 * @{
 */

/*!
 * @brief Sets the timer channel period in microseconds.
 *
 * The period range depends on the frequency of the LPIT source clock. If the required period
 * is out of range, use the suitable mode if applicable.
 * This function is only valid for one single channel. If channels are chained together,
 * the period here makes no sense.
 *
 * @param instance LPIT module instance number
 * @param channel Timer channel number
 * @param us Timer channel period in microseconds
 * @return Fail or success status returned by API
 */
lpit_status_t LPIT_DRV_SetTimerPeriodByUs(uint32_t instance, uint32_t channel, uint32_t us);

/*!
 * @brief Gets the timer channel period in microseconds.
 *
 * @param instance LPIT module instance number
 * @param channel Timer channel number
 * @return Timer channel period in microseconds
 */
uint64_t LPIT_DRV_GetTimerPeriodByUs(uint32_t instance, uint32_t channel);

/*!
 * @brief Reads the current timer channel counting value in microseconds.
 *
 * This function returns an absolute time stamp in microseconds.
 * One common use of this function is to measure the running time of a part of
 * code. Call this function at both the beginning and end of code. The time
 * difference between these two time stamps is the running time. Make sure the
 * running time does not exceed the timer channel period.
 *
 * @param instance LPIT module instance number
 * @param channel Timer channel number
 * @return Current timer channel counting value in microseconds
 */
uint64_t LPIT_DRV_GetCurrentTimerUs(uint32_t instance, uint32_t channel);

/*!
 * @brief Sets the timer channel period in count unit.
 *
 * Timer channels begin counting from the value set by this function.
 * The counter period of a running timer can be modified by first setting a new load value,
 * the value will be loaded after the timer channel expires, To abort the current cycle
 * and start a timer channel period with the new value, the timer channel must be disabled and enabled again.
 *
 * @param instance LPIT module instance number
 * @param channel Timer channel number
 * @param count Timer channel period in count unit
 * @return Fail or success status returned by API
 */
lpit_status_t LPIT_DRV_SetTimerPeriodByCount(uint32_t instance, uint32_t channel, uint32_t count);

/*!
 * @brief Returns the current timer channel period in count unit.
 *
 * @param instance LPIT module instance number
 * @param channel Timer channel number
 * @return Timer channel period in count unit
 */
uint32_t LPIT_DRV_GetTimerPeriodByCount(uint32_t instance, uint32_t channel);

/*!
 * @brief Reads the current timer channel counting value in count.
 *
 * This function returns the real-time timer channel counting value, The value depend on operation mode of timer
 * channel.
 *
 * @param instance LPIT module instance number
 * @param channel Timer channel number
 * @return Current timer channel counting value in count
 */
uint32_t LPIT_DRV_GetCurrentTimerCount(uint32_t instance, uint32_t channel);


/* @} */

/*!
 * @name Interrupt
 * @{
 */

/*!
 * @brief Reads the current interrupt flag of timer channels.
 *
 * In compare mode, every time when the timer channel counts to 0, the interrupt flags is set.
 * In capture mode, every time when selected trigger input is asserted, the interrupt flags is set.
 *
 * @param instance LPIT module instance number.
 * @param mask The mask decides which channels be get interrupt flag
 * For example: with mask = 0x01u then the interrupt flag of channel 0 only will be get
 *              with mask = 0x02u then the interrupt flag of channel 1 only will be get
 *              with mask = 0x03u then the interrupt flags of channel 0 and channel 1 will be get
 * @return Current the interrupt flag of timer channels
 */
uint32_t LPIT_DRV_GetInterruptFlagTimerChannels(uint32_t instance,  uint32_t mask);

/*!
 * @brief Clears the interrupt flag of timer channels.
 *
 * This function clears the interrupt flag of timer channels after
 * their interrupt event occurred.
 *
 * @param instance LPIT module instance number
 * @param mask The mask decides which channels be clear interrupt flag
 * For example: with mask = 0x01u then the interrupt flag of channel 0 only will be cleared
 *              with mask = 0x02u then the interrupt flag of channel 1 only will be cleared
 *              with mask = 0x03u then the interrupt flags of channel 0 and channel 1 will be cleared
 */
void LPIT_DRV_ClearInterruptFlagTimerChannels(uint32_t instance, uint32_t mask);

/* @} */

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __FSL_LPIT_DRIVER_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/
