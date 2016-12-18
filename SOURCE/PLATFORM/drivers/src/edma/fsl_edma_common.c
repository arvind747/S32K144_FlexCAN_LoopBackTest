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
#include <stdint.h>
#include "fsl_device_registers.h"
#include "fsl_clock_manager.h"

/* Array of base addresses for DMA instances. */
DMA_Type * const g_edmaBase[DMA_INSTANCE_COUNT] = DMA_BASE_PTRS;

/* Array of base addresses for DMAMUX instances. */
DMAMUX_Type * const  g_dmamuxBase[DMAMUX_INSTANCE_COUNT] = DMAMUX_BASE_PTRS;

/* Array of default DMA channel interrupt handlers. */
const IRQn_Type g_edmaIrqId[FSL_FEATURE_EDMA_MODULE_CHANNELS] = DMA_CHN_IRQS;

/* Variables storing the DMA clock names, as defined in the SoC specific files. */
clock_names_t dmaClockName = PCC_DMA0_CLOCK;
clock_names_t dmamuxClockName = PCC_DMAMUX0_CLOCK;

/* Array of default DMA error interrupt handlers. */
#if defined FSL_FEATURE_EDMA_HAS_ERROR_IRQ
const IRQn_Type g_edmaErrIrqId[DMA_INSTANCE_COUNT] = DMA_ERROR_IRQS;
#endif

/*******************************************************************************
* EOF
******************************************************************************/

