/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
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

#ifndef _BOARD_DEFINES_H_
#define _BOARD_DEFINES_H_

/* This file defines the specific ports and pins for Evaluation Board and
 * Validation Board. To use with Validation Board uncomment the following 
 * line
 */

#define EVB 

#ifdef EVB
    #define LED1            15U
    #define LED2            16U
    #define LED_GPIO        PTD
    #define LED_PORT        PORTD
    #define LED_PORT_PCC    PCC_PORTD_CLOCK
    #define BTN_GPIO        PTC
    #define BTN_PIN         13U
    #define BTN_PORT        PORTC
    #define BTN_PORT_PCC    PCC_PORTC_CLOCK
    #define BTN_PORT_IRQn   PORTC_IRQn
#else
    #define LED1            0U
    #define LED2            1U
    #define LED_GPIO        PTC
    #define LED_PORT        PORTC
    #define LED_PORT_PCC    PCC_PORTC_CLOCK
    #define BTN_GPIO        PTC
    #define BTN_PIN         13U
    #define BTN_PORT        PORTC
    #define BTN_PORT_PCC    PCC_PORTC_CLOCK
    #define BTN_PORT_IRQn   PORTC_IRQn
#endif


#include "fsl_pcc_hal.h"
#include "fsl_port_hal.h"

void boardSetup(void)
{
    /* Configure clock source */
    PCC_HAL_SetClockMode(PCC, LED_PORT_PCC, false);
    PCC_HAL_SetClockSourceSel(PCC, LED_PORT_PCC, CLK_SRC_FIRC);
    PCC_HAL_SetClockMode(PCC, LED_PORT_PCC, true);
    
    PCC_HAL_SetClockMode(PCC, BTN_PORT_PCC, false);
    PCC_HAL_SetClockSourceSel(PCC, BTN_PORT_PCC, CLK_SRC_FIRC);
    PCC_HAL_SetClockMode(PCC, BTN_PORT_PCC, true);
    
    /* Configure ports */
    PORT_HAL_SetMuxModeSel(LED_PORT, LED1,      PORT_MUX_AS_GPIO);
    PORT_HAL_SetMuxModeSel(LED_PORT, LED2,      PORT_MUX_AS_GPIO);
    PORT_HAL_SetMuxModeSel(BTN_PORT, BTN_PIN,   PORT_MUX_AS_GPIO);
#ifdef EVB
    PORT_HAL_SetPinIntSel(BTN_PORT, BTN_PIN, PORT_INT_RISING_EDGE);
#else
    PORT_HAL_SetPinIntSel(BTN_PORT, BTN_PIN, PORT_INT_FALLING_EDGE);
#endif
}
#endif
