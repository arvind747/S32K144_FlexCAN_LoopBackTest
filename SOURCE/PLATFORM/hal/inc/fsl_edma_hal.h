/*
 * Copyright (c) 2013 - 2016, Freescale Semiconductor, Inc.
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
#ifndef __EDMA_HAL_H__
#define __EDMA_HAL_H__

#include <stdint.h>
#include <stdbool.h>
#include "fsl_device_registers.h"

/*!
 * @addtogroup edma_hal
 * @{
 */
 
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief eDMA channel arbitration algorithm used for selection among channels. */
typedef enum _edma_channel_arbitration {
    EDMA_ARBITRATION_FIXED_PRIORITY = 0U,  /*!< Fixed Priority arbitration is used for selection
                                                among channels. */
    EDMA_ARBITRATION_ROUND_ROBIN           /*!< Round-Robin arbitration is used for selection among
                                                channels. */
} edma_channel_arbitration_t;

/*! @brief eDMA channel priority setting */
typedef enum _edma_chn_priority {
    EDMA_CHN_PRIORITY_0 = 0U,
    EDMA_CHN_PRIORITY_1 = 1U,
    EDMA_CHN_PRIORITY_2 = 2U,
    EDMA_CHN_PRIORITY_3 = 3U,
    EDMA_CHN_PRIORITY_4 = 4U,
    EDMA_CHN_PRIORITY_5 = 5U,
    EDMA_CHN_PRIORITY_6 = 6U,
    EDMA_CHN_PRIORITY_7 = 7U,
    EDMA_CHN_PRIORITY_8 = 8U,
    EDMA_CHN_PRIORITY_9 = 9U,
    EDMA_CHN_PRIORITY_10 = 10U,
    EDMA_CHN_PRIORITY_11 = 11U,
    EDMA_CHN_PRIORITY_12 = 12U,
    EDMA_CHN_PRIORITY_13 = 13U,
    EDMA_CHN_PRIORITY_14 = 14U,
    EDMA_CHN_PRIORITY_15 = 15U,
    EDMA_CHN_DEFAULT_PRIORITY = 255U
} edma_channel_priority_t;

/*! @brief eDMA modulo configuration */
typedef enum _edma_modulo {
    EDMA_MODULO_OFF = 0U,
    EDMA_MODULO_2B,
    EDMA_MODULO_4B,
    EDMA_MODULO_8B,
    EDMA_MODULO_16B,
    EDMA_MODULO_32B,
    EDMA_MODULO_64B,
    EDMA_MODULO_128B,
    EDMA_MODULO_256B,
    EDMA_MODULO_512B,
    EDMA_MODULO_1KB,
    EDMA_MODULO_2KB,
    EDMA_MODULO_4KB,
    EDMA_MODULO_8KB,
    EDMA_MODULO_16KB,
    EDMA_MODULO_32KB,
    EDMA_MODULO_64KB,
    EDMA_MODULO_128KB,
    EDMA_MODULO_256KB,
    EDMA_MODULO_512KB,
    EDMA_MODULO_1MB,
    EDMA_MODULO_2MB,
    EDMA_MODULO_4MB,
    EDMA_MODULO_8MB,
    EDMA_MODULO_16MB,
    EDMA_MODULO_32MB,
    EDMA_MODULO_64MB,
    EDMA_MODULO_128MB,
    EDMA_MODULO_256MB,
    EDMA_MODULO_512MB,
    EDMA_MODULO_1GB,
    EDMA_MODULO_2GB
} edma_modulo_t;

/*! @brief eDMA transfer configuration */
typedef enum _edma_transfer_size {
    EDMA_TRANSFER_SIZE_1B = 0x0U,
    EDMA_TRANSFER_SIZE_2B = 0x1U,
    EDMA_TRANSFER_SIZE_4B = 0x2U,
    EDMA_TRANSFER_SIZE_16B = 0x4U,
    EDMA_TRANSFER_SIZE_32B = 0x5U
} edma_transfer_size_t;

/*! @brief eDMA channel configuration. */
typedef enum _edma_channel_indicator {
    EDMA_CHANNEL_0 = 0U,     /*!< Channel 0. */
    EDMA_CHANNEL_1 = 1U,     /*!< Channel 1. */
    EDMA_CHANNEL_2 = 2U,     /*!< Channel 2. */
    EDMA_CHANNEL_3 = 3U,     /*!< Channel 3. */
    EDMA_CHANNEL_4 = 4U,     /*!< Channel 4. */
    EDMA_CHANNEL_5 = 5U,     /*!< Channel 5. */
    EDMA_CHANNEL_6 = 6U,     /*!< Channel 6. */
    EDMA_CHANNEL_7 = 7U,     /*!< Channel 7. */
    EDMA_CHANNEL_8 = 8U,     /*!< Channel 8. */
    EDMA_CHANNEL_9 = 9U,     /*!< Channel 9. */
    EDMA_CHANNEL_10 = 10U,   /*!< Channel 10. */
    EDMA_CHANNEL_11 = 11U,   /*!< Channel 11. */
    EDMA_CHANNEL_12 = 12U,   /*!< Channel 12. */
    EDMA_CHANNEL_13 = 13U,   /*!< Channel 13. */
    EDMA_CHANNEL_14 = 14U,   /*!< Channel 14. */
    EDMA_CHANNEL_15 = 15U,   /*!< Channel 15. */
    EDMA_ALL_CHANNELS = 64U  /*!< All channels. */
} edma_channel_indicator_t;

/*! @brief Bandwidth control configuration */
typedef enum _edma_bandwidth_config {
    EDMA_BANDWIDTH_STALL_NONE = 0U  ,    /*!< No eDMA engine stalls. */
    EDMA_BANDWIDTH_STALL_4_CYCLES = 2U,  /*!< eDMA engine stalls for 4 cycles after each read/write. */
    EDMA_BANDWIDTH_STALL_8_CYCLES = 3U   /*!< eDMA engine stalls for 8 cycles after each read/write. */
} edma_bandwidth_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name eDMA HAL driver module level operation
 * @{
 */

/*!
 * @brief Initializes eDMA module to known state.
 *
 * @param base Register base address for eDMA module.
 */
void EDMA_HAL_Init(DMA_Type * base);

/*!
 * @brief Cancels the remaining data transfer.
 *
 * This function stops the executing channel and forces the minor loop
 * to finish. The cancellation takes effect after the last write of the
 * current read/write sequence. The CX clears itself after the cancel has
 * been honored. This cancel retires the channel normally as if the minor
 * loop had completed.
 *
 * @param base Register base address for eDMA module.
 */
void EDMA_HAL_CancelTransfer(DMA_Type * base);

/*!
 * @brief Cancels the remaining data transfer and treats it as an error condition.
 *
 * This function stops the executing channel and forces the minor loop
 * to finish. The cancellation takes effect after the last write of the
 * current read/write sequence. The CX clears itself after the cancel has
 * been honoured. This cancel retires the channel normally as if the minor
 * loop had completed. Additional thing is to treat this operation as an error
 * condition.
 *
 * @param base Register base address for eDMA module.
 */
void EDMA_HAL_CancelTransferWithError(DMA_Type * base);

/*!
 * @brief Halts/Un-halts the DMA Operations.
 *
 * This function stalls/un-stalls the start of any new channels. Executing channels are allowed
 * to be completed.
 *
 * @param base Register base address for eDMA module.
 * @param halt Halts (true) or un-halts (false) eDMA transfer.
 */
static inline void EDMA_HAL_SetHaltCmd(DMA_Type * base, bool halt)
{
    BITBAND_ACCESS32(&(base->CR), DMA_CR_HALT_SHIFT) = (uint32_t)halt;
}

/*!
 * @brief Halts or does not halt the eDMA module when an error occurs.
 *
 * An error causes the HALT bit to be set. Subsequently, all service requests are ignored until the
 * HALT bit is cleared.
 *
 * @param base Register base address for eDMA module.
 * @param haltOnError Halts (true) or not halt (false) eDMA module when an error occurs.
 */
static inline void EDMA_HAL_SetHaltOnErrorCmd(DMA_Type * base, bool haltOnError)
{
    BITBAND_ACCESS32(&(base->CR), DMA_CR_HOE_SHIFT) = (uint32_t)haltOnError;
}

/*!
 * @brief Enables/Disables the eDMA DEBUG mode.
 *
 * This function enables/disables the eDMA Debug mode.
 * When in debug mode, the DMA stalls the start of a new
 * channel. Executing channels are allowed to complete. Channel execution resumes
 * either when the system exits debug mode or when the EDBG bit is cleared.
 *
 * @param base Register base address for eDMA module.
 * @param enable Enables (true) or Disable (false) eDMA module debug mode.
 */
static inline void EDMA_HAL_SetDebugCmd(DMA_Type * base, bool enable)
{
    BITBAND_ACCESS32(&(base->CR), DMA_CR_EDBG_SHIFT) = (uint32_t)enable;
}
/* @} */

/*!
 * @name eDMA HAL error checking
 * @{
 */
 
/*!
 * @brief Checks for valid errors.
 *
 * Returns whether a valid error exists, that has not been cleared.
 *
 * @param base Register base address for eDMA module.
 * @return true, if a valid uncleared error exists, false otherwise.
 */
static inline bool EDMA_HAL_GetValidErrorNotCleared(DMA_Type * base)
{
    return (bool)(((base->ES >> DMA_ES_VLD_SHIFT) & 1U));
}

/*!
 * @brief Checks for cancelled transfers.
 *
 * Returns whether the last entry was a cancelled transfer,
 * by the error cancel transfer input
 *
 * @param base Register base address for eDMA module.
 * @return true, if the last transfer was cancelled, false otherwise.
 */
static inline bool EDMA_HAL_GetTransferCancelledError(DMA_Type * base)
{
    return (bool)(((base->ES >> DMA_ES_ECX_SHIFT) & 1U));
}

/*!
 * @brief Checks for channel priority errors.
 *
 * Returns whether the channel priorities configuration was erroneous
 * (priorities not unique).
 *
 * @param base Register base address for eDMA module.
 * @return true, if the channels priorities were misconfigured, false otherwise.
 */
static inline bool EDMA_HAL_GetChannelPriorityError(DMA_Type * base)
{
    return (bool)(((base->ES >> DMA_ES_CPE_SHIFT) & 1U));
}

/*!
 * @brief Checks for source address errors.
 *
 * Returns whether the source address configuration was erroneous.
 *
 * @param base Register base address for eDMA module.
 * @return true, if the source address was misconfigured, false otherwise.
 */
static inline bool EDMA_HAL_GetSourceAddressError(DMA_Type * base)
{
    return (bool)(((base->ES >> DMA_ES_SAE_SHIFT) & 1U));
}

/*!
 * @brief Checks for source offset errors.
 *
 * Returns whether the source offset configuration was erroneous.
 *
 * @param base Register base address for eDMA module.
 * @return true, if the source offset was misconfigured, false otherwise.
 */
static inline bool EDMA_HAL_GetSourceOffsetError(DMA_Type * base)
{
    return (bool)(((base->ES >> DMA_ES_SOE_SHIFT) & 1U));
}

/*!
 * @brief Checks for destination address errors.
 *
 * Returns whether the destination address configuration was erroneous.
 *
 * @param base Register base address for eDMA module.
 * @return true, if the destination address was misconfigured, false otherwise.
 */
static inline bool EDMA_HAL_GetDestinationAddressError(DMA_Type * base)
{
    return (bool)(((base->ES >> DMA_ES_DAE_SHIFT) & 1U));
}

/*!
 * @brief Checks for destination offset errors.
 *
 * Returns whether the destination offset configuration was erroneous.
 *
 * @param base Register base address for eDMA module.
 * @return true, if the destination offset was misconfigured, false otherwise.
 */
static inline bool EDMA_HAL_GetDestinationOffsetError(DMA_Type * base)
{
    return (bool)(((base->ES >> DMA_ES_DOE_SHIFT) & 1U));
}

/*!
 * @brief Checks for minor/major loop configuration errors.
 *
 * Returns whether the NBYTES or CITER fields configuration was erroneous.
 *
 * @param base Register base address for eDMA module.
 * @return true, if the NBYTES/CITER was misconfigured, false otherwise.
 */
static inline bool EDMA_HAL_GetMinorMajorLoopConfigError(DMA_Type * base)
{
    return (bool)(((base->ES >> DMA_ES_NCE_SHIFT) & 1U));
}

/*!
 * @brief Checks for scatter/gather configuration errors.
 *
 * Returns whether the scatter/gather configuration was erroneous.
 *
 * @param base Register base address for eDMA module.
 * @return true, if the scatter/gather feature was misconfigured, false otherwise.
 */
static inline bool EDMA_HAL_GetScatterGatherError(DMA_Type * base)
{
    return (bool)(((base->ES >> DMA_ES_SGE_SHIFT) & 1U));
}

/*!
 * @brief Checks for source bus errors.
 *
 * Returns whether there was a bus error on a source read.
 *
 * @param base Register base address for eDMA module.
 * @return true, if there was a source bus error, false otherwise.
 */
static inline bool EDMA_HAL_GetSourceBusError(DMA_Type * base)
{
    return (bool)(((base->ES >> DMA_ES_SBE_SHIFT) & 1U));
}

/*!
 * @brief Checks for destination bus errors.
 *
 * Returns whether there was a bus error on a destination write.
 *
 * @param base Register base address for eDMA module.
 * @return true, if there was a destination bus error, false otherwise.
 */
static inline bool EDMA_HAL_GetDestinationBusError(DMA_Type * base)
{
    return (bool)(((base->ES >> DMA_ES_DBE_SHIFT) & 1U));
}

/*!
 * @brief Error channel number.
 *
 * Returns the channel number of the last recorded error.
 *
 * @param base Register base address for eDMA module.
 * @return the channel number of the last recorded error.
 */
static inline uint8_t EDMA_HAL_GetChannelWithError(DMA_Type * base)
{
    return (uint8_t)(((base->ES >> DMA_ES_ERRCHN_SHIFT) & 0xFU));
}
/* @} */

/*!
 * @name eDMA HAL driver channel priority and arbitration configuration
 * @{
 */
 
/*!
 * @brief Sets the preemption feature for the eDMA channel.
 *
 * This function sets the preemption features.
 *
 * @param base Register base address for eDMA module.
 * @param channel eDMA channel number.
 * @param preemptive eDMA channel can suspend a lower priority channel (true). eDMA channel cannot
 * suspend a lower priority channel (false).
 * @param preemptible eDMA channel can be temporarily suspended by the service request of a higher
 * priority channel (true). eDMA channel can't be suspended by a higher priority channel (false).
 */
static inline void EDMA_HAL_SetChannelPreemptMode(DMA_Type * base, uint32_t channel, bool preemptive, bool preemptible)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_EDMA_MODULE_CHANNEL);
#endif

    uint32_t index = FSL_FEATURE_EDMA_CHN_TO_DCHPRI_INDEX(channel);
    BITBAND_ACCESS32(&(base->DCHPRI[index]), DMA_DCHPRI_DPA_SHIFT) = (uint32_t)(!preemptive);
    BITBAND_ACCESS32(&(base->DCHPRI[index]), DMA_DCHPRI_ECP_SHIFT) = (uint32_t)preemptible;
}

/*!
 * @brief Sets the eDMA channel priority.
 *
 * @param base Register base address for eDMA module.
 * @param channel eDMA channel number.
 * @param priority Priority of the DMA channel. Different channels should have different priority
 * setting inside a group.
 */
static inline void EDMA_HAL_SetChannelPriority(DMA_Type * base, uint32_t channel, edma_channel_priority_t priority)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_EDMA_MODULE_CHANNEL);
#endif
    
    uint32_t regValTemp;
    uint32_t index = FSL_FEATURE_EDMA_CHN_TO_DCHPRI_INDEX(channel);
    regValTemp = base->DCHPRI[index];
    regValTemp &= ~(DMA_DCHPRI_CHPRI_MASK);
    regValTemp |= DMA_DCHPRI_CHPRI(priority);
    base->DCHPRI[index] = regValTemp;
}

/*!
 * @brief Sets the channel arbitration algorithm.
 *
 * @param base Register base address for eDMA module.
 * @param channelArbitration Round-Robin way for fixed priority way.
 */
static inline void EDMA_HAL_SetChannelArbitrationMode(DMA_Type * base, edma_channel_arbitration_t channelArbitration)
{
    BITBAND_ACCESS32(&(base->CR), DMA_CR_ERCA_SHIFT) = (uint32_t)(channelArbitration);
}

/*!
 * @brief Gets the channel arbitration algorithm.
 *
 * @param base Register base address for eDMA module.
 * @return boolean variable indicating the selected channel arbitration; false = fixed, true = round robin
 */
static inline bool EDMA_HAL_GetChannelArbitrationMode(DMA_Type * base)
{
    return ((base->CR >> DMA_CR_ERCA_SHIFT) & 1U);
}

/*!
 * @name eDMA HAL driver configuration and operation
 * @{
 */
/*!
 * @brief Enables/Disables the minor loop mapping.
 *
 * This function enables/disables the minor loop mapping feature.
 * If enabled, the NBYTES is redefined to include the individual enable fields and the NBYTES field. The
 * individual enable fields allow the minor loop offset to be applied to the source address, the
 * destination address, or both. The NBYTES field is reduced when either offset is enabled.
 *
 * @param base Register base address for eDMA module.
 * @param enable Enables (true) or Disable (false) minor loop mapping.
 */
static inline void EDMA_HAL_SetMinorLoopMappingCmd(DMA_Type * base, bool enable)
{
    BITBAND_ACCESS32(&(base->CR), DMA_CR_EMLM_SHIFT) = (uint32_t)(enable);
}

/*!
 * @brief Enables or disables the continuous transfer mode.
 *
 * This function enables or disables the continuous transfer. If set, a minor loop channel link
 * does not go through the channel arbitration before being activated again. Upon minor loop
 * completion, the channel activates again if that channel has a minor loop channel link enabled and
 * the link channel is itself.
 *
 * @param base Register base address for eDMA module.
 * @param continuous Enables (true) or Disable (false) continuous transfer mode.
 */
static inline void EDMA_HAL_SetContinuousLinkCmd(DMA_Type * base, bool continuous)
{
    BITBAND_ACCESS32(&(base->CR), DMA_CR_CLM_SHIFT) = (uint32_t)(continuous);
}

/*!
 * @brief Enables/Disables the error interrupt for channels.
 *
 * @param base Register base address for eDMA module.
 * @param channel Channel indicator. If kEDMAAllChannel is selected, all channels' error interrupt
 * will be enabled/disabled.
 * @param enable Enable(true) or Disable (false) error interrupt.
 */
void EDMA_HAL_SetErrorIntCmd(DMA_Type * base, edma_channel_indicator_t channel, bool enable);

/*!
 * @brief Gets the eDMA error interrupt status.
 *
 * @param base Register base address for eDMA module.
 * @return 32 bit variable indicating error channels. If error happens on eDMA channel n, the bit n
 * of this variable is '1'. If not, the bit n of this variable is '0'.
 */
static inline uint32_t EDMA_HAL_GetErrorIntStatusFlag(DMA_Type * base)
{
    return base->ERR;
}

/*!
 * @brief Clears the error interrupt status for the eDMA channel or channels.
 *
 * @param base Register base address for eDMA module.
 * @param channel Channel indicator. If kEDMAAllChannel is selected, all channels' error interrupt
 * status will be cleared.
 */
static inline void EDMA_HAL_ClearErrorIntStatusFlag(DMA_Type * base, edma_channel_indicator_t channel)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_EDMA_MODULE_CHANNEL);
#endif
    base->CERR = (uint8_t)channel;
}

/*!
 * @brief Enables/Disables the DMA request for the channel or all channels.
 *
 * @param base Register base address for eDMA module.
 * @param enable Enable(true) or Disable (false) DMA request.
 * @param channel Channel indicator. If kEDMAAllChannel is selected, all channels DMA request
 * are enabled/disabled.
 */
void EDMA_HAL_SetDmaRequestCmd(DMA_Type * base, edma_channel_indicator_t channel,bool enable);

/*!
 * @brief Gets the eDMA channel DMA request status.
 *
 * @param base Register base address for eDMA module.
 * @param channel eDMA channel number.
 * @return Hardware request is triggered in this eDMA channel (true) or not be triggered in this
 * channel (false).
 */
static inline bool EDMA_HAL_GetDmaRequestStatusFlag(DMA_Type * base, uint32_t channel)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_EDMA_MODULE_CHANNEL);
#endif
    return ((base->HRS >> channel) & 1U);
}

/*!
 * @brief Clears the done status for a channel or all channels.
 *
 * @param base Register base address for eDMA module.
 * @param channel Channel indicator. If kEDMAAllChannel is selected, all channels' done status will
 * be cleared.
 */
static inline void EDMA_HAL_ClearDoneStatusFlag(DMA_Type * base, edma_channel_indicator_t channel)
{  
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_EDMA_MODULE_CHANNEL);
#endif
    base->CDNE = (uint8_t)channel;
}

/*!
 * @brief Triggers the eDMA channel.
 *
 * @param base Register base address for eDMA module.
 * @param channel Channel indicator. If kEDMAAllChannel is selected, all channels are tirggere.
 */
static inline void EDMA_HAL_TriggerChannelStart(DMA_Type * base, edma_channel_indicator_t channel)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_EDMA_MODULE_CHANNEL);
#endif
    base->SSRT = (uint8_t)channel;
}

/*!
 * @brief Gets the eDMA channel interrupt request status.
 *
 * @param base Register base address for eDMA module.
 * @param channel eDMA channel number.
 * @return Interrupt request happens in this eDMA channel (true) or not happen in this
 * channel (false).
 */
static inline bool EDMA_HAL_GetIntStatusFlag(DMA_Type * base, uint32_t channel)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_EDMA_MODULE_CHANNEL);
#endif

    return ((base->INT >> channel) & 1U);
}

/*!
 * @brief Clears the interrupt status for the eDMA channel or all channels.
 *
 * @param base Register base address for eDMA module.
 * @param channel Channel indicator. If kEDMAAllChannel is selected, all channels' interrupt
 * status will be cleared.
 */
static inline void EDMA_HAL_ClearIntStatusFlag(DMA_Type * base, edma_channel_indicator_t channel)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_EDMA_MODULE_CHANNEL);
#endif
    base->CINT = (uint8_t)channel;
}

#if (FSL_FEATURE_EDMA_ASYNCHRO_REQUEST_CHANNEL_COUNT > 0x0U)
/*!
 * @brief Enables/Disables an asynchronous request in stop mode.
 *
 * @param base Register base address for eDMA module.
 * @param channel eDMA channel number.
 * @param enable Enable (true) or Disable (false) async DMA request.
 */
static inline void EDMA_HAL_SetAsyncRequestInStopModeCmd(DMA_Type * base, uint32_t channel, bool enable)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_EDMA_MODULE_CHANNEL);
#endif
    BITBAND_ACCESS32(&(base->EARS), channel) = ((uint32_t)enable);
}
#endif

/* @} */

/*!
 * @name eDMA HAL driver TCD configuration functions
 * @{
 */

/*!
 * @brief Clears all registers to 0 for the hardware TCD.
 *
 * @param base Register base address for eDMA module.
 * @param channel eDMA channel number.
 */
void EDMA_HAL_TCDClearReg(DMA_Type * base, uint32_t channel);

/*!
 * @brief Configures the source address for the hardware TCD.
 *
 * @param base Register base address for eDMA module.
 * @param channel eDMA channel number.
 * @param address The pointer to the source memory address.
 */
static inline void EDMA_HAL_TCDSetSrcAddr(DMA_Type * base, uint32_t channel, uint32_t address)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_EDMA_MODULE_CHANNEL);
#endif
    base->TCD[channel].SADDR = address;
}

/*!
 * @brief Configures the source address signed offset for the hardware TCD.
 *
 * Sign-extended offset applied to the current source address to form the next-state value as each
 * source read is complete.
 *
 * @param base Register base address for eDMA module.
 * @param channel eDMA channel number.
 * @param offset signed-offset for source address.
 */
static inline void EDMA_HAL_TCDSetSrcOffset(DMA_Type * base, uint32_t channel, int16_t offset)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_EDMA_MODULE_CHANNEL);
#endif
    base->TCD[channel].SOFF = offset;
}

/*!
 * @brief Configures the transfer attribute for the eDMA channel.
 *
 * @param base Register base address for eDMA module.
 * @param channel eDMA channel number.
 * @param srcModulo enumeration type for an allowed source modulo. The value defines a specific address range
 * specified as the value after the SADDR + SOFF calculation is performed on the original register
 * value. Setting this field provides the ability to implement a circular data. For data queues
 * requiring power-of-2 size bytes, the queue should start at a 0-modulo-size address and the SMOD
 * field should be set to the appropriate value for the queue, freezing the desired number of upper
 * address bits. The value programmed into this field specifies the number of the lower address bits
 * allowed to change. For a circular queue application, the SOFF is typically set to the transfer
 * size to implement post-increment addressing with SMOD function restricting the addresses to a
 * 0-modulo-size range.
 * @param destModulo Enum type for an allowed destination modulo.
 * @param srcTransferSize Enum type for source transfer size.
 * @param destTransferSize Enum type for destination transfer size.
 */
void EDMA_HAL_TCDSetAttribute(
                DMA_Type * base, uint32_t channel,
                edma_modulo_t srcModulo, edma_modulo_t destModulo,
                edma_transfer_size_t srcTransferSize, edma_transfer_size_t destTransferSize);

/*!
 * @brief Configures the nbytes for the eDMA channel.
 *
 * Note here that user need firstly configure the minor loop mapping feature and then call this
 * function.
 *
 * @param base Register base address for eDMA module.
 * @param channel eDMA channel number.
 * @param nbytes Number of bytes to be transferred in each service request of the channel
 */
void EDMA_HAL_TCDSetNbytes(DMA_Type * base, uint32_t channel, uint32_t nbytes);

/*!
 * @brief Gets the nbytes configuration data for the TCD.
 *
 * This function  decides whether the minor loop mapping is enabled or whether the source/dest
 * minor loop mapping is enabled. Then, the nbytes are returned accordingly.
 *
 * @param base Register base address for eDMA module.
 * @param channel eDMA channel number.
 * @return nbytes configuration according to minor loop setting.
 */
uint32_t EDMA_HAL_TCDGetNbytes(DMA_Type * base, uint32_t channel);

/*!
 * @brief Enables/disables the source minor loop offset feature for the TCD.
 *
 * Configures whether the minor loop offset is applied to the source address 
 * upon minor loop completion.
 * NOTE: EMLM bit needs to be enabled prior to calling this function, otherwise
 * it has no effect.
 *
 * @param base Register base address for eDMA module.
 * @param channel eDMA channel number.
 * @param enable Enables (true) or disables (false) source minor loop offset.
 */
static inline void EDMA_HAL_TCDSetSrcMinorLoopOffsetCmd(DMA_Type * base, uint32_t channel, bool enable)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_EDMA_MODULE_CHANNEL);
#endif

    if (BITBAND_ACCESS32(&(base->CR), DMA_CR_EMLM_SHIFT))
        BITBAND_ACCESS32(&(base->TCD[channel].NBYTES.MLOFFYES), DMA_TCD_NBYTES_MLOFFYES_SMLOE_SHIFT) = (uint32_t)enable;
}

/*!
 * @brief Enables/disables the destination minor loop offset feature for the TCD.
 *
 * Configures whether the minor loop offset is applied to the destination address 
 * upon minor loop completion.
 * NOTE: EMLM bit needs to be enabled prior to calling this function, otherwise
 * it has no effect.
 *
 * @param base Register base address for eDMA module.
 * @param channel eDMA channel number.
 * @param enable Enables (true) or disables (false) destination minor loop offset.
 */
static inline void EDMA_HAL_TCDSetDestMinorLoopOffsetCmd(DMA_Type * base, uint32_t channel, bool enable)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_EDMA_MODULE_CHANNEL);
#endif

    if (BITBAND_ACCESS32(&(base->CR), DMA_CR_EMLM_SHIFT))
        BITBAND_ACCESS32(&(base->TCD[channel].NBYTES.MLOFFYES), DMA_TCD_NBYTES_MLOFFYES_DMLOE_SHIFT) = (uint32_t)enable;
}

/*!
 * @brief Configures the minor loop offset for the TCD.
 *
 * Configures the offset value. If neither source nor destination offset is enabled, 
 * offset is not configured.
 * NOTE: EMLM bit needs to be enabled prior to calling this function, otherwise
 * it has no effect.
 *
 * @param base Register base address for eDMA module.
 * @param channel eDMA channel number.
 * @param offset Minor loop offset
 */
void EDMA_HAL_TCDSetMinorLoopOffset(DMA_Type * base, uint32_t channel, uint32_t offset);

/*!
 * @brief Configures the last source address adjustment for the TCD.
 *
 * Adjustment value added to the source address at the completion of the major iteration count. This
 * value can be applied to restore the source address to the initial value, or adjust the address to
 * reference the next data structure.
 *
 * @param base Register base address for eDMA module.
 * @param channel eDMA channel number.
 * @param size adjustment value
 */
static inline void EDMA_HAL_TCDSetSrcLastAdjust(DMA_Type * base, uint32_t channel, int32_t size)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_EDMA_MODULE_CHANNEL);
#endif
    base->TCD[channel].SLAST = size;
}

/*!
 * @brief Configures the destination address for the TCD.
 *
 * @param base Register base address for eDMA module.
 * @param channel eDMA channel number.
 * @param address The pointer to the destination address.
 */
static inline void EDMA_HAL_TCDSetDestAddr(DMA_Type * base, uint32_t channel, uint32_t address)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_EDMA_MODULE_CHANNEL);
#endif
    base->TCD[channel].DADDR = address;
}

/*!
 * @brief Configures the destination address signed offset for the TCD.
 *
 * Sign-extended offset applied to the current source address to form the next-state value as each
 * destination write is complete.
 *
 * @param base Register base address for eDMA module.
 * @param channel eDMA channel number.
 * @param offset signed-offset
 */
static inline void EDMA_HAL_TCDSetDestOffset(DMA_Type * base, uint32_t channel, int16_t offset)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_EDMA_MODULE_CHANNEL);
#endif
    base->TCD[channel].DOFF = offset;
}

/*!
 * @brief Configures the last source address adjustment.
 *
 * This function adds an adjustment value added to the source address at the completion of the major
 * iteration count. This value can be applied to restore the source address to the initial value, or
 * adjust the address to reference the next data structure.
 *
 * @param base Register base address for eDMA module.
 * @param channel eDMA channel number.
 * @param adjust adjustment value
 */
static inline void EDMA_HAL_TCDSetDestLastAdjust(DMA_Type * base, uint32_t channel, int32_t adjust)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_EDMA_MODULE_CHANNEL);
#endif
    base->TCD[channel].DLASTSGA = adjust;
}

/*!
 * @brief Configures the memory address for the next transfer TCD for the TCD.
 *
 *
 * This function enables the scatter/gather feature for the TCD and configures the next
 * TCD's address. This address points to the beginning of a 0-modulo-32 byte region containing
 * the next transfer TCD to be loaded into this channel. The channel reload is performed as the
 * major iteration count completes. The scatter/gather address must be 0-modulo-32-byte. Otherwise,
 * a configuration error is reported.
 *
 * @param base Register base address for eDMA module.
 * @param channel eDMA channel number.
 * @param nextTCDAddr The address of the next TCD to be linked to this TCD.
 */
void EDMA_HAL_TCDSetScatterGatherLink(DMA_Type * base, uint32_t channel, uint32_t nextTCDAddr);

/*!
 * @brief Configures the bandwidth for the TCD.
 *
 * Throttles the amount of bus bandwidth consumed by the eDMA. In general, as the eDMA processes the
 * minor loop, it continuously generates read/write sequences until the minor count is exhausted.
 * This field forces the eDMA to stall after the completion of each read/write access to control the
 * bus request bandwidth seen by the crossbar switch.
 *
 * @param base Register base address for eDMA module.
 * @param channel eDMA channel number.
 * @param bandwidth enum type for bandwidth control
 */
static inline void EDMA_HAL_TCDSetBandwidth(DMA_Type * base, uint32_t channel, edma_bandwidth_config_t bandwidth)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_EDMA_MODULE_CHANNEL);
#endif
    uint32_t regValTemp;
    regValTemp = base->TCD[channel].CSR;
    regValTemp &= ~(DMA_TCD_CSR_BWC_MASK);
    regValTemp |= DMA_TCD_CSR_BWC(bandwidth);
    base->TCD[channel].CSR = regValTemp;
}

/*!
 * @brief Configures the major channel link the TCD.
 *
 * If the major link is enabled, after the major loop counter is exhausted, the eDMA engine initiates a
 * channel service request at the channel defined by these six bits by setting that channel start
 * bits.
 *
 * @param base Register base address for eDMA module.
 * @param channel eDMA channel number.
 * @param majorLinkChannel channel number for major link
 * @param enable Enables (true) or Disables (false) channel major link.
 */
static inline void EDMA_HAL_TCDSetChannelMajorLink(DMA_Type * base, uint32_t channel, uint32_t majorLinkChannel, bool enable)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_EDMA_MODULE_CHANNEL);
#endif
    uint32_t regValTemp;
    regValTemp = base->TCD[channel].CSR;
    regValTemp &= ~(DMA_TCD_CSR_MAJORLINKCH_MASK);
    regValTemp |= DMA_TCD_CSR_MAJORLINKCH(majorLinkChannel);
    base->TCD[channel].CSR = regValTemp;

    BITBAND_ACCESS32(&(base->TCD[channel].CSR), DMA_TCD_CSR_MAJORELINK_SHIFT) = (uint32_t)enable;
}

/*!
 * @brief Enables/Disables the scatter/gather feature for the TCD.
 *
 * @param base Register base address for eDMA module.
 * @param channel eDMA channel number.
 * @param enable Enables (true) /Disables (false) scatter/gather feature.
 */
static inline void EDMA_HAL_TCDSetScatterGatherCmd(DMA_Type * base, uint32_t channel, bool enable)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_EDMA_MODULE_CHANNEL);
#endif
    BITBAND_ACCESS32(&(base->TCD[channel].CSR), DMA_TCD_CSR_ESG_SHIFT) = (uint32_t)enable;
}

/*!
 * @brief Disables/Enables the DMA request after the major loop completes for the TCD.
 *
 * If disabled, the eDMA hardware automatically clears the corresponding DMA request when the
 * current major iteration count reaches zero.
 *
 * @param base Register base address for eDMA module.
 * @param channel eDMA channel number.
 * @param disable Disable (true)/Enable (false) DMA request after TCD complete.
 */
static inline void EDMA_HAL_TCDSetDisableDmaRequestAfterTCDDoneCmd(DMA_Type * base, uint32_t channel, bool disable)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_EDMA_MODULE_CHANNEL);
#endif
    BITBAND_ACCESS32(&(base->TCD[channel].CSR), DMA_TCD_CSR_DREQ_SHIFT) = (uint32_t)disable;
}

/*!
 * @brief Enables/Disables the half complete interrupt for the TCD.
 *
 * If set, the channel generates an interrupt request by setting the appropriate bit in the
 * interrupt register when the current major iteration count reaches the halfway point. Specifically,
 * the comparison performed by the eDMA engine is (CITER == (BITER >> 1)). This half-way point
 * interrupt request is provided to support the double-buffered schemes or other types of data movement
 * where the processor needs an early indication of the transfer's process.
 *
 * @param base Register base address for eDMA module.
 * @param channel eDMA channel number.
 * @param enable Enable (true) /Disable (false) half complete interrupt.
 */
static inline void EDMA_HAL_TCDSetHalfCompleteIntCmd(DMA_Type * base, uint32_t channel, bool enable)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_EDMA_MODULE_CHANNEL);
#endif
    BITBAND_ACCESS32(&(base->TCD[channel].CSR), DMA_TCD_CSR_INTHALF_SHIFT) = (uint32_t)enable;
}

/*!
 * @brief Enables/Disables the interrupt after the major loop completes for the TCD.
 *
 * If enabled, the channel generates an interrupt request by setting the appropriate bit in the
 * interrupt register when the current major iteration count reaches zero.
 *
 * @param base Register base address for eDMA module.
 * @param channel eDMA channel number.
 * @param enable Enable (true) /Disable (false) interrupt after TCD done.
 */
static inline void EDMA_HAL_TCDSetIntCmd(DMA_Type * base, uint32_t channel, bool enable)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_EDMA_MODULE_CHANNEL);
#endif
    BITBAND_ACCESS32(&(base->TCD[channel].CSR), DMA_TCD_CSR_INTMAJOR_SHIFT) = (uint32_t)enable;
}

/*!
 * @brief Triggers the start bits for the TCD.
 *
 * The eDMA hardware automatically clears this flag after the channel begins execution.
 *
 * @param base Register base address for eDMA module.
 * @param channel eDMA channel number.
 */
static inline void EDMA_HAL_TCDTriggerChannelStart(DMA_Type * base, uint32_t channel)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_EDMA_MODULE_CHANNEL);
#endif
    BITBAND_ACCESS32(&(base->TCD[channel].CSR), DMA_TCD_CSR_START_SHIFT) = (uint32_t)1U;
}

/*!
 * @brief Checks whether the channel is running for the TCD.
 *
 * @param base Register base address for eDMA module.
 * @param channel eDMA channel number.
 * @return True stands for running. False stands for not.
 */
static inline bool EDMA_HAL_TCDGetChannelActiveStatus(DMA_Type * base, uint32_t channel)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_EDMA_MODULE_CHANNEL);
#endif
    return (bool)((base->TCD[channel].CSR >> DMA_TCD_CSR_ACTIVE_SHIFT) & 1U);
}

/*!
 * @brief Sets the channel minor link for the TCD.
 *
 * @param base Register base address for eDMA module.
 * @param channel eDMA channel number.
 * @param linkChannel Channel to be linked on minor loop complete.
 * @param enable Enable (true)/Disable (false) channel minor link.
 */
void EDMA_HAL_TCDSetChannelMinorLink(DMA_Type * base, uint32_t channel, uint32_t linkChannel, bool enable);

/*!
 * @brief Sets the major iteration count according to minor loop channel link setting.
 *
 * Note here that user need to first set the minor loop channel link and then call this function.
 * The execute flow inside this function is dependent on the minor loop channel link setting.
 *
 * @param base Register base address for eDMA module.
 * @param channel eDMA channel number.
 * @param count major loop count
 */
void EDMA_HAL_TCDSetMajorCount(DMA_Type * base, uint32_t channel, uint32_t count);

/*!
 * @brief Returns the begin major iteration count.
 *
 * Gets the begin major iteration count according to minor loop channel link settings.
 *
 * @param base Register base address for eDMA module.
 * @param channel eDMA channel number.
 * @return begin iteration count
 */
uint32_t EDMA_HAL_TCDGetBeginMajorCount(DMA_Type * base, uint32_t channel);

/*!
 * @brief Returns the current major iteration count.
 *
 * Gets the current major iteration count according to minor loop channel link settings.
 *
 * @param base Register base address for eDMA module.
 * @param channel eDMA channel number.
 * @return current iteration count
 */
uint32_t EDMA_HAL_TCDGetCurrentMajorCount(DMA_Type * base, uint32_t channel);

/*!
 * @brief Gets the number of bytes already transferred for the TCD.
 *
 * @param base Register base address for eDMA module.
 * @param channel eDMA channel number.
 * @return data bytes already transferred
 */
uint32_t EDMA_HAL_TCDGetFinishedBytes(DMA_Type * base, uint32_t channel);

/*!
 * @brief Gets the number of bytes haven't transferred for the TCD.
 *
 * @param base Register base address for eDMA module.
 * @param channel eDMA channel number.
 * @return data bytes already transferred
 */
uint32_t EDMA_HAL_TCDGetUnfinishedBytes(DMA_Type * base, uint32_t channel);

/*!
 * @brief Gets the channel done status.
 *
 * @param base Register base address for eDMA module.
 * @param channel eDMA channel number.
 * @return If channel done.
 */
static inline bool EDMA_HAL_TCDGetDoneStatusFlag(DMA_Type * base, uint32_t channel)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FSL_FEATURE_EDMA_MODULE_CHANNEL);
#endif
    return (bool)((base->TCD[channel].CSR >> DMA_TCD_CSR_DONE_SHIFT) & 1U);
}

/*! @} */

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* __EDMA_HAL_H__ */

/*******************************************************************************
 * EOF
 ******************************************************************************/


