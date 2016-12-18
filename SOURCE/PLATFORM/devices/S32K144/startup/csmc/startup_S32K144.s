; ---------------------------------------------------------------------------------------
;  @file:    startup_S32K144.s
;  @purpose: Cosmic C Cross Compiler Startup File
;            S32K144
;  @version: 1.9
;  @date:    2016-2-23
;  @build:   b160223
; ---------------------------------------------------------------------------------------
;
; Copyright (c) 1997 - 2016 , Freescale Semiconductor, Inc.
; All rights reserved.
;
; Redistribution and use in source and binary forms, with or without modification,
; are permitted provided that the following conditions are met:
;
; o Redistributions of source code must retain the above copyright notice, this list
;   of conditions and the following disclaimer.
;
; o Redistributions in binary form must reproduce the above copyright notice, this
;   list of conditions and the following disclaimer in the documentation and/or
;   other materials provided with the distribution.
;
; o Neither the name of Freescale Semiconductor, Inc. nor the names of its
;   contributors may be used to endorse or promote products derived from this
;   software without specific prior written permission.
;
; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
; ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
; WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
; DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
; ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
; (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
; ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
; (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
; SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
;*****************************************************************************
;* Version: Cosmic C Cross Compiler                                          *
;*****************************************************************************
    xref.l DefaultISR
    xref.l _main
    xref.l _init_data_bss
    xref.l _SystemInit
    ; Needed for data initialization
    xref.l __idesc__   
    xdef Reset_Handler
    xdef __StackTop

    ;.section .isr_vector, "a"
    align 2
    
__isr_vector: section .text
    dc.l   __StackTop                                      ; Top of Stack
    dc.l   Reset_Handler                                   ; Reset Handler
    dc.l   NMI_Handler                                     ; NMI Handler
    dc.l   HardFault_Handler                               ; Hard Fault Handler
    dc.l   MemManage_Handler                               ; MPU Fault Handler
    dc.l   BusFault_Handler                                ; Bus Fault Handler
    dc.l   UsageFault_Handler                              ; Usage Fault Handler
    dc.l   0                                               ; Reserved
    dc.l   0                                               ; Reserved
    dc.l   0                                               ; Reserved
    dc.l   0                                               ; Reserved
    dc.l   SVC_Handler                                     ; SVCall Handler
    dc.l   DebugMon_Handler                                ; Debug Monitor Handler
    dc.l   0                                               ; Reserved
    dc.l   PendSV_Handler                                  ; PendSV Handler
    dc.l   SysTick_Handler                                 ; SysTick Handler

                                                           ; External Interrupts
    dc.l   DMA0_IRQHandler                                 ; DMA channel 0 transfer complete
    dc.l   DMA1_IRQHandler                                 ; DMA channel 1 transfer complete
    dc.l   DMA2_IRQHandler                                 ; DMA channel 2 transfer complete
    dc.l   DMA3_IRQHandler                                 ; DMA channel 3 transfer complete
    dc.l   DMA4_IRQHandler                                 ; DMA channel 4 transfer complete
    dc.l   DMA5_IRQHandler                                 ; DMA channel 5 transfer complete
    dc.l   DMA6_IRQHandler                                 ; DMA channel 6 transfer complete
    dc.l   DMA7_IRQHandler                                 ; DMA channel 7 transfer complete
    dc.l   DMA8_IRQHandler                                 ; DMA channel 8 transfer complete
    dc.l   DMA9_IRQHandler                                 ; DMA channel 9 transfer complete
    dc.l   DMA10_IRQHandler                                ; DMA channel 10 transfer complete
    dc.l   DMA11_IRQHandler                                ; DMA channel 11 transfer complete
    dc.l   DMA12_IRQHandler                                ; DMA channel 12 transfer complete
    dc.l   DMA13_IRQHandler                                ; DMA channel 13 transfer complete
    dc.l   DMA14_IRQHandler                                ; DMA channel 14 transfer complete
    dc.l   DMA15_IRQHandler                                ; DMA channel 15 transfer complete
    dc.l   DMA_Error_IRQHandler                            ; DMA error interrupt channels 0-15
    dc.l   MCM_IRQHandler                                  ; FPU sources
    dc.l   FTFE_IRQHandler                                 ; FTFE command complete
    dc.l   Read_Collision_IRQHandler                       ; FTFE read collision
    dc.l   LVD_LVW_IRQHandler                              ; PMC controller low-voltage detect, low-voltage warning
    dc.l   FTFE_Fault_IRQHandler                           ; FTFE Double bit fault detect
    dc.l   Watchdog_IRQHandler                             ; Single interrupt vector for WDOG and EWM
    dc.l   RCM_IRQHandler                                  ; RCM Asynchronous Interrupt
    dc.l   LPI2C0_IRQHandler                               ; Inter-integrated circuit 0
    dc.l   LPI2C1_IRQHandler                               ; Inter-integrated circuit 1
    dc.l   LPSPI0_IRQHandler                               ; Serial peripheral Interface 0
    dc.l   LPSPI1_IRQHandler                               ; Serial peripheral Interface 1
    dc.l   LPSPI2_IRQHandler                               ; Serial peripheral Interface 2
    dc.l   Reserved45_IRQHandler                           ; Reserved interrupt
    dc.l   Reserved46_IRQHandler                           ; Reserved interrupt
    dc.l   LPUART0_RxTx_IRQHandler                         ; LPUART0 receive/transmit interrupt
    dc.l   Reserved48_IRQHandler                           ; Reserved interrupt
    dc.l   LPUART1_RxTx_IRQHandler                         ; LPUART1 receive/transmit interrupt
    dc.l   Reserved50_IRQHandler                           ; Reserved interrupt
    dc.l   LPUART2_RxTx_IRQHandler                         ; LPUART2 receive/transmit interrupt
    dc.l   Reserved52_IRQHandler                           ; Reserved interrupt
    dc.l   LPUART3_RxTx_IRQHandler                         ; LPUART3 receive/transmit interrupt
    dc.l   Reserved54_IRQHandler                           ; Reserved interrupt
    dc.l   ADC0_IRQHandler                                 ; ADC conversion complete interrupt
    dc.l   CMP0_IRQHandler                                 ; CMP0 interrupt
    dc.l   Reserved57_IRQHandler                           ; Reserved interrupt
    dc.l   FTM0_IRQHandler                                 ; FTM0 single interrupt vector for all sources
    dc.l   FTM1_IRQHandler                                 ; FTM1 single interrupt vector for all sources
    dc.l   FTM2_IRQHandler                                 ; FTM2 single interrupt vector for all sources
    dc.l   Reserved61_IRQHandler                           ; Reserved interrupt
    dc.l   RTC_IRQHandler                                  ; RTC alarm interrupt
    dc.l   RTC_Seconds_IRQHandler                          ; RTC seconds interrupt overflow
    dc.l   LPIT0_IRQHandler                                ; LPIT overflow
    dc.l   Reserved65_IRQHandler                           ; Reserved interrupt
    dc.l   Reserved66_IRQHandler                           ; Reserved interrupt
    dc.l   Reserved67_IRQHandler                           ; Reserved interrupt
    dc.l   PDB0_IRQHandler                                 ; Programmable delay block
    dc.l   Reserved69_IRQHandler                           ; Reserved interrupt
    dc.l   Reserved70_IRQHandler                           ; Reserved interrupt
    dc.l   Reserved71_IRQHandler                           ; Reserved interrupt
    dc.l   Reserved72_IRQHandler                           ; Reserved interrupt
    dc.l   SCG_IRQHandler                                  ; Multipurpose clock generator
    dc.l   LPTMR0_IRQHandler                               ; Single interrupt vector for  Low Power Timer 0
    dc.l   PORTA_IRQHandler                                ; Port A pin detect interrupt
    dc.l   PORTB_IRQHandler                                ; Port B pin detect interrupt
    dc.l   PORTC_IRQHandler                                ; Port C pin detect interrupt
    dc.l   PORTD_IRQHandler                                ; Port D pin detect interrupt
    dc.l   PORTE_IRQHandler                                ; Port E pin detect interrupt
    dc.l   SWI_IRQHandler                                  ; Software interrupt
    dc.l   Reserved81_IRQHandler                           ; Reserved interrupt
    dc.l   Reserved82_IRQHandler                           ; Reserved interrupt
    dc.l   Reserved83_IRQHandler                           ; Reserved interrupt
    dc.l   PDB1_IRQHandler                                 ; Programmable delay block
    dc.l   FLEXIO_IRQHandler                               ; FLEXIO
    dc.l   Reserved86_IRQHandler                           ; Reserved interrupt
    dc.l   FTM3_IRQHandler                                 ; FlexTimer module 3 fault, overflow and channels interrupt
    dc.l   Reserved88_IRQHandler                           ; Reserved interrupt
    dc.l   ADC1_IRQHandler                                 ; ADC conversion complete interrupt
    dc.l   Reserved90_IRQHandler                           ; Reserved interrupt
    dc.l   Reserved91_IRQHandler                           ; Reserved interrupt
    dc.l   Reserved92_IRQHandler                           ; Reserved interrupt
    dc.l   Reserved93_IRQHandler                           ; Reserved interrupt
    dc.l   CAN0_ORed_IRQHandler                            ; can
    dc.l   CAN0_Error_IRQHandler                           ; can
    dc.l   CAN0_Wake_Up_IRQHandler                         ; can
    dc.l   CAN0_ORed_Message_buffer_IRQHandler             ; can
    dc.l   CAN0_Reserved1_IRQHandler                       ; can
    dc.l   CAN0_Reserved2_IRQHandler                       ; can
    dc.l   CAN0_Reserved3_IRQHandler                       ; can
    dc.l   CAN1_ORed_IRQHandler                            ; can
    dc.l   CAN1_Error_IRQHandler                           ; can
    dc.l   CAN1_Wake_Up_IRQHandler                         ; can
    dc.l   CAN1_ORed_Message_buffer_IRQHandler             ; can
    dc.l   CAN1_Reserved1_IRQHandler                       ; can
    dc.l   CAN1_Reserved2_IRQHandler                       ; can
    dc.l   CAN1_Reserved3_IRQHandler                       ; can
    dc.l   CAN2_ORed_IRQHandler                            ; can
    dc.l   CAN2_Error_IRQHandler                           ; can
    dc.l   CAN2_Wake_Up_IRQHandler                         ; can
    dc.l   CAN2_ORed_Message_buffer_IRQHandler             ; can
    dc.l   CAN2_Reserved1_IRQHandler                       ; can
    dc.l   CAN2_Reserved2_IRQHandler                       ; can
    dc.l   CAN2_Reserved3_IRQHandler                       ; can
    dc.l   DefaultISR                                      ; 115
    dc.l   DefaultISR                                      ; 116
    dc.l   DefaultISR                                      ; 117
    dc.l   DefaultISR                                      ; 118
    dc.l   DefaultISR                                      ; 119
    dc.l   DefaultISR                                      ; 120
    dc.l   DefaultISR                                      ; 121
    dc.l   DefaultISR                                      ; 122
    dc.l   DefaultISR                                      ; 123
    dc.l   DefaultISR                                      ; 124
    dc.l   DefaultISR                                      ; 125
    dc.l   DefaultISR                                      ; 126
    dc.l   DefaultISR                                      ; 127
    dc.l   DefaultISR                                      ; 128
    dc.l   DefaultISR                                      ; 129
    dc.l   DefaultISR                                      ; 130
    dc.l   DefaultISR                                      ; 131
    dc.l   DefaultISR                                      ; 132
    dc.l   DefaultISR                                      ; 133
    dc.l   DefaultISR                                      ; 134
    dc.l   DefaultISR                                      ; 135
    dc.l   DefaultISR                                      ; 136
    dc.l   DefaultISR                                      ; 137
    dc.l   DefaultISR                                      ; 138
    dc.l   DefaultISR                                      ; 139
    dc.l   DefaultISR                                      ; 140
    dc.l   DefaultISR                                      ; 141
    dc.l   DefaultISR                                      ; 142
    dc.l   DefaultISR                                      ; 143
    dc.l   DefaultISR                                      ; 144
    dc.l   DefaultISR                                      ; 145
    dc.l   DefaultISR                                      ; 146
    dc.l   DefaultISR                                      ; 147
    dc.l   DefaultISR                                      ; 148
    dc.l   DefaultISR                                      ; 149
    dc.l   DefaultISR                                      ; 150
    dc.l   DefaultISR                                      ; 151
    dc.l   DefaultISR                                      ; 152
    dc.l   DefaultISR                                      ; 153
    dc.l   DefaultISR                                      ; 154
    dc.l   DefaultISR                                      ; 155
    dc.l   DefaultISR                                      ; 156
    dc.l   DefaultISR                                      ; 157
    dc.l   DefaultISR                                      ; 158
    dc.l   DefaultISR                                      ; 159
    dc.l   DefaultISR                                      ; 160
    dc.l   DefaultISR                                      ; 161
    dc.l   DefaultISR                                      ; 162
    dc.l   DefaultISR                                      ; 163
    dc.l   DefaultISR                                      ; 164
    dc.l   DefaultISR                                      ; 165
    dc.l   DefaultISR                                      ; 166
    dc.l   DefaultISR                                      ; 167
    dc.l   DefaultISR                                      ; 168
    dc.l   DefaultISR                                      ; 169
    dc.l   DefaultISR                                      ; 170
    dc.l   DefaultISR                                      ; 171
    dc.l   DefaultISR                                      ; 172
    dc.l   DefaultISR                                      ; 173
    dc.l   DefaultISR                                      ; 174
    dc.l   DefaultISR                                      ; 175
    dc.l   DefaultISR                                      ; 176
    dc.l   DefaultISR                                      ; 177
    dc.l   DefaultISR                                      ; 178
    dc.l   DefaultISR                                      ; 179
    dc.l   DefaultISR                                      ; 180
    dc.l   DefaultISR                                      ; 181
    dc.l   DefaultISR                                      ; 182
    dc.l   DefaultISR                                      ; 183
    dc.l   DefaultISR                                      ; 184
    dc.l   DefaultISR                                      ; 185
    dc.l   DefaultISR                                      ; 186
    dc.l   DefaultISR                                      ; 187
    dc.l   DefaultISR                                      ; 188
    dc.l   DefaultISR                                      ; 189
    dc.l   DefaultISR                                      ; 190
    dc.l   DefaultISR                                      ; 191
    dc.l   DefaultISR                                      ; 192
    dc.l   DefaultISR                                      ; 193
    dc.l   DefaultISR                                      ; 194
    dc.l   DefaultISR                                      ; 195
    dc.l   DefaultISR                                      ; 196
    dc.l   DefaultISR                                      ; 197
    dc.l   DefaultISR                                      ; 198
    dc.l   DefaultISR                                      ; 199
    dc.l   DefaultISR                                      ; 200
    dc.l   DefaultISR                                      ; 201
    dc.l   DefaultISR                                      ; 202
    dc.l   DefaultISR                                      ; 203
    dc.l   DefaultISR                                      ; 204
    dc.l   DefaultISR                                      ; 205
    dc.l   DefaultISR                                      ; 206
    dc.l   DefaultISR                                      ; 207
    dc.l   DefaultISR                                      ; 208
    dc.l   DefaultISR                                      ; 209
    dc.l   DefaultISR                                      ; 210
    dc.l   DefaultISR                                      ; 211
    dc.l   DefaultISR                                      ; 212
    dc.l   DefaultISR                                      ; 213
    dc.l   DefaultISR                                      ; 214
    dc.l   DefaultISR                                      ; 215
    dc.l   DefaultISR                                      ; 216
    dc.l   DefaultISR                                      ; 217
    dc.l   DefaultISR                                      ; 218
    dc.l   DefaultISR                                      ; 219
    dc.l   DefaultISR                                      ; 220
    dc.l   DefaultISR                                      ; 221
    dc.l   DefaultISR                                      ; 222
    dc.l   DefaultISR                                      ; 223
    dc.l   DefaultISR                                      ; 224
    dc.l   DefaultISR                                      ; 225
    dc.l   DefaultISR                                      ; 226
    dc.l   DefaultISR                                      ; 227
    dc.l   DefaultISR                                      ; 228
    dc.l   DefaultISR                                      ; 229
    dc.l   DefaultISR                                      ; 230
    dc.l   DefaultISR                                      ; 231
    dc.l   DefaultISR                                      ; 232
    dc.l   DefaultISR                                      ; 233
    dc.l   DefaultISR                                      ; 234
    dc.l   DefaultISR                                      ; 235
    dc.l   DefaultISR                                      ; 236
    dc.l   DefaultISR                                      ; 237
    dc.l   DefaultISR                                      ; 238
    dc.l   DefaultISR                                      ; 239
    dc.l   DefaultISR                                      ; 240
    dc.l   DefaultISR                                      ; 241
    dc.l   DefaultISR                                      ; 242
    dc.l   DefaultISR                                      ; 243
    dc.l   DefaultISR                                      ; 244
    dc.l   DefaultISR                                      ; 245
    dc.l   DefaultISR                                      ; 246
    dc.l   DefaultISR                                      ; 247
    dc.l   DefaultISR                                      ; 248
    dc.l   DefaultISR                                      ; 249
    dc.l   DefaultISR                                      ; 250
    dc.l   DefaultISR                                      ; 251
    dc.l   DefaultISR                                      ; 252
    dc.l   DefaultISR                                      ; 253
    dc.l   DefaultISR                                      ; 254
    dc.l   0xFFFFFFFF                                      ;  Reserved for user TRIM value

; Flash Configuration
    align 4
FlashConfig: section .text
    dc.l 0xFFFFFFFF     ; 8 bytes backdoor comparison key
    dc.l 0xFFFFFFFF     ;
    dc.l 0xFFFFFFFF     ; 4 bytes program flash protection bytes
    dc.l 0xFFFF7FFE     ; FDPROT:FEPROT:FOPT:FSEC(0xFE = unsecured)

; Reset Handler

ResetH_Seg: section .text
    align 2    ; allocate an byte - so that Reset_Handler will be in thumb mode
    dc.b 1     ; FIXME: remove the above hack and specify thumb mode in makefile / lkf 
Reset_Handler:
    align 2
    cpsid   i               ; Mask interrupts

    ; Init the rest of the registers
    ldr     r1,=0
    ldr     r2,=0
    ldr     r3,=0
    ldr     r4,=0
    ldr     r5,=0
    ldr     r6,=0
    ldr     r7,=0
    mov     r8,r7
    mov     r9,r7
    mov     r10,r7
    mov     r11,r7
    mov     r12,r7
    ; Initialize the stack pointer
    ldr     r0,=__StackTop
    mov     r13,r0

#ifndef __NO_SYSTEM_INIT
    ; Call the CMSIS system init routine
    bl     _SystemInit
#endif

    ; Init .data and .bss sections
    bl     _init_data_bss
    cpsie   i               ; Unmask interrupts
    bl     _main
JumpToSelf:
    b      JumpToSelf

    align 2    ; allocate an byte - so that Reset_Handler will be in thumb mode
    dc.b 1     ; FIXME: remove the above hack and specify thumb mode in makefile / lkf 
DefaultISR:
    b      DefaultISR
  
;    Macro to define default handlers. Default handler
;    will be weak symbol and just dead loops. They can be
;    overwritten by other handlers
;    +1 is used to indicate thumb mode
;    -1 is an workarround
def_irq_handler: macro \handler_name
wdef _\handler_name
xref.l _\handler_name
_\handler_name : equ DefaultISR-1
\handler_name: equ _\handler_name+1
endm

; Exception Handlers
    def_irq_handler    NMI_Handler
    def_irq_handler    HardFault_Handler
    def_irq_handler    MemManage_Handler
    def_irq_handler    BusFault_Handler
    def_irq_handler    UsageFault_Handler
    def_irq_handler    SVC_Handler
    def_irq_handler    DebugMon_Handler
    def_irq_handler    PendSV_Handler
    def_irq_handler    SysTick_Handler
    def_irq_handler    DMA0_IRQHandler
    def_irq_handler    DMA1_IRQHandler
    def_irq_handler    DMA2_IRQHandler
    def_irq_handler    DMA3_IRQHandler
    def_irq_handler    DMA4_IRQHandler
    def_irq_handler    DMA5_IRQHandler
    def_irq_handler    DMA6_IRQHandler
    def_irq_handler    DMA7_IRQHandler
    def_irq_handler    DMA8_IRQHandler
    def_irq_handler    DMA9_IRQHandler
    def_irq_handler    DMA10_IRQHandler
    def_irq_handler    DMA11_IRQHandler
    def_irq_handler    DMA12_IRQHandler
    def_irq_handler    DMA13_IRQHandler
    def_irq_handler    DMA14_IRQHandler
    def_irq_handler    DMA15_IRQHandler
    def_irq_handler    DMA_Error_IRQHandler
    def_irq_handler    MCM_IRQHandler
    def_irq_handler    FTFE_IRQHandler
    def_irq_handler    Read_Collision_IRQHandler
    def_irq_handler    LVD_LVW_IRQHandler
    def_irq_handler    FTFE_Fault_IRQHandler
    def_irq_handler    Watchdog_IRQHandler
    def_irq_handler    RCM_IRQHandler
    def_irq_handler    LPI2C0_IRQHandler
    def_irq_handler    LPI2C1_IRQHandler
    def_irq_handler    LPSPI0_IRQHandler
    def_irq_handler    LPSPI1_IRQHandler
    def_irq_handler    LPSPI2_IRQHandler
    def_irq_handler    Reserved45_IRQHandler
    def_irq_handler    Reserved46_IRQHandler
    def_irq_handler    LPUART0_RxTx_IRQHandler
    def_irq_handler    Reserved48_IRQHandler
    def_irq_handler    LPUART1_RxTx_IRQHandler
    def_irq_handler    Reserved50_IRQHandler
    def_irq_handler    LPUART2_RxTx_IRQHandler
    def_irq_handler    Reserved52_IRQHandler
    def_irq_handler    LPUART3_RxTx_IRQHandler
    def_irq_handler    Reserved54_IRQHandler
    def_irq_handler    ADC0_IRQHandler
    def_irq_handler    CMP0_IRQHandler
    def_irq_handler    Reserved57_IRQHandler
    def_irq_handler    FTM0_IRQHandler
    def_irq_handler    FTM1_IRQHandler
    def_irq_handler    FTM2_IRQHandler
    def_irq_handler    Reserved61_IRQHandler
    def_irq_handler    RTC_IRQHandler
    def_irq_handler    RTC_Seconds_IRQHandler
    def_irq_handler    LPIT0_IRQHandler
    def_irq_handler    Reserved65_IRQHandler
    def_irq_handler    Reserved66_IRQHandler
    def_irq_handler    Reserved67_IRQHandler
    def_irq_handler    PDB0_IRQHandler
    def_irq_handler    Reserved69_IRQHandler
    def_irq_handler    Reserved70_IRQHandler
    def_irq_handler    Reserved71_IRQHandler
    def_irq_handler    Reserved72_IRQHandler
    def_irq_handler    SCG_IRQHandler
    def_irq_handler    LPTMR0_IRQHandler
    def_irq_handler    PORTA_IRQHandler
    def_irq_handler    PORTB_IRQHandler
    def_irq_handler    PORTC_IRQHandler
    def_irq_handler    PORTD_IRQHandler
    def_irq_handler    PORTE_IRQHandler
    def_irq_handler    SWI_IRQHandler
    def_irq_handler    Reserved81_IRQHandler
    def_irq_handler    Reserved82_IRQHandler
    def_irq_handler    Reserved83_IRQHandler
    def_irq_handler    PDB1_IRQHandler
    def_irq_handler    FLEXIO_IRQHandler
    def_irq_handler    Reserved86_IRQHandler
    def_irq_handler    FTM3_IRQHandler
    def_irq_handler    Reserved88_IRQHandler
    def_irq_handler    ADC1_IRQHandler
    def_irq_handler    Reserved90_IRQHandler
    def_irq_handler    Reserved91_IRQHandler
    def_irq_handler    Reserved92_IRQHandler
    def_irq_handler    Reserved93_IRQHandler
    def_irq_handler    CAN0_ORed_IRQHandler
    def_irq_handler    CAN0_Error_IRQHandler
    def_irq_handler    CAN0_Wake_Up_IRQHandler
    def_irq_handler    CAN0_ORed_Message_buffer_IRQHandler
    def_irq_handler    CAN0_Reserved1_IRQHandler
    def_irq_handler    CAN0_Reserved2_IRQHandler
    def_irq_handler    CAN0_Reserved3_IRQHandler
    def_irq_handler    CAN1_ORed_IRQHandler
    def_irq_handler    CAN1_Error_IRQHandler
    def_irq_handler    CAN1_Wake_Up_IRQHandler
    def_irq_handler    CAN1_ORed_Message_buffer_IRQHandler
    def_irq_handler    CAN1_Reserved1_IRQHandler
    def_irq_handler    CAN1_Reserved2_IRQHandler
    def_irq_handler    CAN1_Reserved3_IRQHandler
    def_irq_handler    CAN2_ORed_IRQHandler
    def_irq_handler    CAN2_Error_IRQHandler
    def_irq_handler    CAN2_Wake_Up_IRQHandler
    def_irq_handler    CAN2_ORed_Message_buffer_IRQHandler
    def_irq_handler    CAN2_Reserved1_IRQHandler
    def_irq_handler    CAN2_Reserved2_IRQHandler
    def_irq_handler    CAN2_Reserved3_IRQHandler

;EOF
