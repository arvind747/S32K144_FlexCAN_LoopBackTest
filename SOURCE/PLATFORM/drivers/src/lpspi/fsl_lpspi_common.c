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
#include <stddef.h>
#include "fsl_device_registers.h"


/* Pointer to runtime state structure.*/
void * g_lpspiStatePtr[LPSPI_INSTANCE_COUNT] = { NULL };

/*! @brief Table of base pointers for SPI instances. */
LPSPI_Type * const g_lpspiBase[LPSPI_INSTANCE_COUNT] = LPSPI_BASE_PTRS;

/*! @brief Table of SPI FIFO sizes per instance. */
uint32_t g_lpspiFifoSize[LPSPI_INSTANCE_COUNT] = {2,2,2};

/*!
 * @brief Table to save LPSPI IRQ enum numbers defined in CMSIS files.
 *
 * This is used by LPSPI master and slave init functions to enable or disable LPSPI interrupts.
 * This table is indexed by the module instance number and returns LPSPI IRQ numbers.
 */
const IRQn_Type g_lpspiIrqId[] = LPSPI_IRQS;

/*******************************************************************************
* EOF
******************************************************************************/

