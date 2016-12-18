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

#if !defined(__FSL_EDMA_REQUEST_H__)
#define __FSL_EDMA_REQUEST_H__

/*!
 * @addtogroup edma_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief Structure for the DMA hardware request
 *
 * Defines the structure for the DMA hardware request collections. The user can configure the
 * hardware request into DMAMUX to trigger the DMA transfer accordingly. The index
 * of the hardware request varies according  to the to SoC.
 */

typedef enum _dma_request_source {
#if defined(CPU_S32K144HFT0VLLT)
    EDMA_REQ_CHANNEL_DISABLED = 0U,
    EDMA_REQ_LPUART0_RX = 2U,
    EDMA_REQ_LPUART0_TX = 3U,
    EDMA_REQ_LPUART1_RX = 4U,
    EDMA_REQ_LPUART1_TX = 5U,
    EDMA_REQ_LPUART2_RX = 6U,
    EDMA_REQ_LPUART2_TX = 7U,
    EDMA_REQ_LPUART3_RX = 8U,
    EDMA_REQ_LPUART3_TX = 9U,
    EDMA_REQ_FLEXIO_SHIFTER0 = 10U,
    EDMA_REQ_FLEXIO_SHIFTER1 = 11U,
    EDMA_REQ_FLEXIO_SHIFTER2 = 12U,
    EDMA_REQ_FLEXIO_SHIFTER3 = 13U,
    EDMA_REQ_LPSPI0_RX = 14U,
    EDMA_REQ_LPSPI0_TX = 15U,
    EDMA_REQ_LPSPI1_RX = 16U,
    EDMA_REQ_LPSPI1_TX = 17U,
    EDMA_REQ_LPI2C0_RX = 18U,
    EDMA_REQ_LPI2C0_TX = 19U,
    EDMA_REQ_FTM0_CHANNEL_0 = 20U,
    EDMA_REQ_FTM0_CHANNEL_1 = 21U,
    EDMA_REQ_FTM0_CHANNEL_2 = 22U,
    EDMA_REQ_FTM0_CHANNEL_3 = 23U,
    EDMA_REQ_FTM0_CHANNEL_4 = 24U,
    EDMA_REQ_FTM0_CHANNEL_5 = 25U,
    EDMA_REQ_FTM0_CHANNEL_6 = 26U,
    EDMA_REQ_FTM0_CHANNEL_7 = 27U,
    EDMA_REQ_FTM1_CHANNEL_0 = 28U,
    EDMA_REQ_FTM1_CHANNEL_1 = 29U,
    EDMA_REQ_FTM2_CHANNEL_0 = 30U,
    EDMA_REQ_FTM2_CHANNEL_1 = 31U,
    EDMA_REQ_FTM3_CHANNEL_0 = 32U,
    EDMA_REQ_FTM3_CHANNEL_1 = 33U,
    EDMA_REQ_FTM3_CHANNEL_2 = 34U,
    EDMA_REQ_FTM3_CHANNEL_3 = 35U,
    EDMA_REQ_FTM3_CHANNEL_4 = 36U,
    EDMA_REQ_FTM3_CHANNEL_5 = 37U,
    EDMA_REQ_FTM3_CHANNEL_6 = 38U,
    EDMA_REQ_FTM3_CHANNEL_7 = 39U,
    EDMA_REQ_ADC0_COCO = 40U,
    EDMA_REQ_ADC1_COCO = 41U,
    EDMA_REQ_CMP0 = 43U,
    EDMA_REQ_LPI2C1_RX = 44U,
    EDMA_REQ_LPI2C1_TX = 45U,
    EDMA_REQ_PDB0 = 46U,
    EDMA_REQ_PDB1 = 47U,
    EDMA_REQ_PORT_A = 49U,
    EDMA_REQ_PORT_B = 50U,
    EDMA_REQ_PORT_C = 51U,
    EDMA_REQ_PORT_D = 52U,
    EDMA_REQ_PORT_E = 53U,
    EDMA_REQ_FLEXCAN0 = 54U,
    EDMA_REQ_FLEXCAN1 = 55U,
    EDMA_REQ_FLEXCAN2 = 56U,
    EDMA_REQ_FTM1_OR_CH2_CH7 = 57U,
    EDMA_REQ_FTM2_OR_CH2_CH7 = 58U,
    EDMA_REQ_LPTMR0 = 59U,
    EDMA_REQ_LPSPI2_RX = 60U,
    EDMA_REQ_LPSPI2_TX = 61U,
    EDMA_REQ_DMAMUX_ALWAYS_ENABLED0 = 62U,
    EDMA_REQ_DMAMUX_ALWAYS_ENABLED1 = 63U
#else
    #error "No valid CPU defined!"
#endif
} dma_request_source_t;

/* @} */

#endif /* __FSL_EDMA_REQUEST_H__ */

/*******************************************************************************
 * EOF
 ******************************************************************************/

