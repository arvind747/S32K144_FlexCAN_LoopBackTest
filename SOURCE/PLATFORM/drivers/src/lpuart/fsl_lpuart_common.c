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

#include "fsl_device_registers.h"
#include "fsl_clock_manager.h"
#include "fsl_interrupt_manager.h"
#include <stddef.h>

/*******************************************************************************
 *  Default interrupt handlers signatures
 ******************************************************************************/

/*! @brief LPUART0 interrupt handler. */
extern void LPUART0_IrqHandler(void);

/*! @brief LPUART1 interrupt handler. */
extern void LPUART1_IrqHandler(void);

/*! @brief LPUART2 interrupt handler. */
extern void LPUART2_IrqHandler(void);

/*! @brief LPUART3 interrupt handler. */
extern void LPUART3_IrqHandler(void);


/*******************************************************************************
 * Variables
 ******************************************************************************/

/* Pointer to lpuart runtime state structure.*/
void * g_lpuartStatePtr[LPUART_INSTANCE_COUNT] = { NULL };

/* Table of base addresses for lpuart instances. */
LPUART_Type * const g_lpuartBase[LPUART_INSTANCE_COUNT] = LPUART_BASE_PTRS;

/* Table to save LPUART enum numbers defined in CMSIS files. */
IRQn_Type g_lpuartRxTxIrqId[LPUART_INSTANCE_COUNT] = LPUART_RX_TX_IRQS;

/* Table to save LPUART clock names as defined in clock manager. */
clock_names_t g_lpuartClkNames[LPUART_INSTANCE_COUNT] = {PCC_LPUART0_CLOCK, PCC_LPUART1_CLOCK, PCC_LPUART2_CLOCK, PCC_LPUART3_CLOCK};

/* Table to save LPUART ISRs - to be used for interrupt service routing at runtime, parameter for INT_SYS_InstallHandler */
isr_t g_lpuartIsr[LPUART_INSTANCE_COUNT] = {LPUART0_IrqHandler, LPUART1_IrqHandler, LPUART2_IrqHandler, LPUART3_IrqHandler};

/*******************************************************************************
 * EOF
 ******************************************************************************/
