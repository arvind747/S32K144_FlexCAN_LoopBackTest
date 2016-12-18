; ---------------------------------------------------------------------------------------
;  @file:    startup_S32K144.s
;  @purpose: CMSIS Cortex-M4 Core Device Startup File
;            S32K144
;  @version: 1.7
;  @date:    2015-10-21
;  @build:   b151027
; ---------------------------------------------------------------------------------------
;
; Copyright (c) 1997 - 2015 , Freescale Semiconductor, Inc.
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
;* Version: GCC for ARM Embedded Processors                                  *
;*****************************************************************************
    .file "startup_S32K144.s"
    .text

    .need DefaultISR
    .globl DefaultISR

    .thumb

    .section .isr_vector, "a"
    .align 2
    .globl __isr_vector
__isr_vector:
    .long   __StackTop                                      ; Top of Stack
    .long   Reset_Handler                                   ; Reset Handler
    .long   NMI_Handler                                     ; NMI Handler
    .long   HardFault_Handler                               ; Hard Fault Handler
    .long   MemManage_Handler                               ; MPU Fault Handler
    .long   BusFault_Handler                                ; Bus Fault Handler
    .long   UsageFault_Handler                              ; Usage Fault Handler
    .long   0                                               ; Reserved
    .long   0                                               ; Reserved
    .long   0                                               ; Reserved
    .long   0                                               ; Reserved
    .long   SVC_Handler                                     ; SVCall Handler
    .long   DebugMon_Handler                                ; Debug Monitor Handler
    .long   0                                               ; Reserved
    .long   PendSV_Handler                                  ; PendSV Handler
    .long   SysTick_Handler                                 ; SysTick Handler

                                                            ; External Interrupts
    .long   DMA0_IRQHandler                                 ; DMA channel 0 transfer complete
    .long   DMA1_IRQHandler                                 ; DMA channel 1 transfer complete
    .long   DMA2_IRQHandler                                 ; DMA channel 2 transfer complete
    .long   DMA3_IRQHandler                                 ; DMA channel 3 transfer complete
    .long   DMA4_IRQHandler                                 ; DMA channel 4 transfer complete
    .long   DMA5_IRQHandler                                 ; DMA channel 5 transfer complete
    .long   DMA6_IRQHandler                                 ; DMA channel 6 transfer complete
    .long   DMA7_IRQHandler                                 ; DMA channel 7 transfer complete
    .long   DMA8_IRQHandler                                 ; DMA channel 8 transfer complete
    .long   DMA9_IRQHandler                                 ; DMA channel 9 transfer complete
    .long   DMA10_IRQHandler                                ; DMA channel 10 transfer complete
    .long   DMA11_IRQHandler                                ; DMA channel 11 transfer complete
    .long   DMA12_IRQHandler                                ; DMA channel 12 transfer complete
    .long   DMA13_IRQHandler                                ; DMA channel 13 transfer complete
    .long   DMA14_IRQHandler                                ; DMA channel 14 transfer complete
    .long   DMA15_IRQHandler                                ; DMA channel 15 transfer complete
    .long   DMA_Error_IRQHandler                            ; DMA error interrupt channels 0-15
    .long   MCM_IRQHandler                                  ; FPU sources
    .long   FTFE_IRQHandler                                 ; FTFE command complete
    .long   Read_Collision_IRQHandler                       ; FTFE read collision
    .long   LVD_LVW_IRQHandler                              ; PMC controller low-voltage detect, low-voltage warning
    .long   FTFE_Fault_IRQHandler                           ; FTFE Double bit fault detect
    .long   Watchdog_IRQHandler                             ; Single interrupt vector for WDOG and EWM
    .long   RCM_IRQHandler                                  ; RCM Asynchronous Interrupt
    .long   LPI2C0_IRQHandler                               ; Inter-integrated circuit 0
    .long   LPI2C1_IRQHandler                               ; Inter-integrated circuit 1
    .long   LPSPI0_IRQHandler                               ; Serial peripheral Interface 0
    .long   LPSPI1_IRQHandler                               ; Serial peripheral Interface 1
    .long   LPSPI2_IRQHandler                               ; Serial peripheral Interface 2
    .long   Reserved45_IRQHandler                           ; Reserved interrupt
    .long   Reserved46_IRQHandler                           ; Reserved interrupt
    .long   LPUART0_RxTx_IRQHandler                         ; LPUART0 receive/transmit interrupt
    .long   Reserved48_IRQHandler                           ; Reserved interrupt
    .long   LPUART1_RxTx_IRQHandler                         ; LPUART1 receive/transmit interrupt
    .long   Reserved50_IRQHandler                           ; Reserved interrupt
    .long   LPUART2_RxTx_IRQHandler                         ; LPUART2 receive/transmit interrupt
    .long   Reserved52_IRQHandler                           ; Reserved interrupt
    .long   LPUART3_RxTx_IRQHandler                         ; LPUART3 receive/transmit interrupt
    .long   Reserved54_IRQHandler                           ; Reserved interrupt
    .long   ADC0_IRQHandler                                 ; ADC conversion complete interrupt
    .long   CMP0_IRQHandler                                 ; CMP0 interrupt
    .long   Reserved57_IRQHandler                           ; Reserved interrupt
    .long   FTM0_IRQHandler                                 ; FTM0 single interrupt vector for all sources
    .long   FTM1_IRQHandler                                 ; FTM1 single interrupt vector for all sources
    .long   FTM2_IRQHandler                                 ; FTM2 single interrupt vector for all sources
    .long   Reserved61_IRQHandler                           ; Reserved interrupt
    .long   RTC_IRQHandler                                  ; RTC alarm interrupt
    .long   RTC_Seconds_IRQHandler                          ; RTC seconds interrupt overflow
    .long   LPIT0_IRQHandler                                ; LPIT overflow
    .long   Reserved65_IRQHandler                           ; Reserved interrupt
    .long   Reserved66_IRQHandler                           ; Reserved interrupt
    .long   Reserved67_IRQHandler                           ; Reserved interrupt
    .long   PDB0_IRQHandler                                 ; Programmable delay block
    .long   Reserved69_IRQHandler                           ; Reserved interrupt
    .long   Reserved70_IRQHandler                           ; Reserved interrupt
    .long   Reserved71_IRQHandler                           ; Reserved interrupt
    .long   Reserved72_IRQHandler                           ; Reserved interrupt
    .long   SCG_IRQHandler                                  ; Multipurpose clock generator
    .long   LPTMR0_IRQHandler                               ; Single interrupt vector for  Low Power Timer 0
    .long   PORTA_IRQHandler                                ; Port A pin detect interrupt
    .long   PORTB_IRQHandler                                ; Port B pin detect interrupt
    .long   PORTC_IRQHandler                                ; Port C pin detect interrupt
    .long   PORTD_IRQHandler                                ; Port D pin detect interrupt
    .long   PORTE_IRQHandler                                ; Port E pin detect interrupt
    .long   SWI_IRQHandler                                  ; Software interrupt
    .long   Reserved81_IRQHandler                           ; Reserved interrupt
    .long   Reserved82_IRQHandler                           ; Reserved interrupt
    .long   Reserved83_IRQHandler                           ; Reserved interrupt
    .long   PDB1_IRQHandler                                 ; Programmable delay block
    .long   FLEXIO_IRQHandler                               ; FLEXIO
    .long   Reserved86_IRQHandler                           ; Reserved interrupt
    .long   FTM3_IRQHandler                                 ; FlexTimer module 3 fault, overflow and channels interrupt
    .long   Reserved88_IRQHandler                           ; Reserved interrupt
    .long   ADC1_IRQHandler                                 ; ADC conversion complete interrupt
    .long   Reserved90_IRQHandler                           ; Reserved interrupt
    .long   Reserved91_IRQHandler                           ; Reserved interrupt
    .long   Reserved92_IRQHandler                           ; Reserved interrupt
    .long   Reserved93_IRQHandler                           ; Reserved interrupt
    .long   CAN0_ORed_IRQHandler                            ; can
    .long   CAN0_Error_IRQHandler                           ; can
    .long   CAN0_Wake_Up_IRQHandler                         ; can
    .long   CAN0_ORed_Message_buffer_IRQHandler             ; can
    .long   CAN0_Reserved1_IRQHandler                       ; can
    .long   CAN0_Reserved2_IRQHandler                       ; can
    .long   CAN0_Reserved3_IRQHandler                       ; can
    .long   CAN1_ORed_IRQHandler                            ; can
    .long   CAN1_Error_IRQHandler                           ; can
    .long   CAN1_Wake_Up_IRQHandler                         ; can
    .long   CAN1_ORed_Message_buffer_IRQHandler             ; can
    .long   CAN1_Reserved1_IRQHandler                       ; can
    .long   CAN1_Reserved2_IRQHandler                       ; can
    .long   CAN1_Reserved3_IRQHandler                       ; can
    .long   CAN2_ORed_IRQHandler                            ; can
    .long   CAN2_Error_IRQHandler                           ; can
    .long   CAN2_Wake_Up_IRQHandler                         ; can
    .long   CAN2_ORed_Message_buffer_IRQHandler             ; can
    .long   CAN2_Reserved1_IRQHandler                       ; can
    .long   CAN2_Reserved2_IRQHandler                       ; can
    .long   CAN2_Reserved3_IRQHandler                       ; can
    .long   DefaultISR                                      ; 115
    .long   DefaultISR                                      ; 116
    .long   DefaultISR                                      ; 117
    .long   DefaultISR                                      ; 118
    .long   DefaultISR                                      ; 119
    .long   DefaultISR                                      ; 120
    .long   DefaultISR                                      ; 121
    .long   DefaultISR                                      ; 122
    .long   DefaultISR                                      ; 123
    .long   DefaultISR                                      ; 124
    .long   DefaultISR                                      ; 125
    .long   DefaultISR                                      ; 126
    .long   DefaultISR                                      ; 127
    .long   DefaultISR                                      ; 128
    .long   DefaultISR                                      ; 129
    .long   DefaultISR                                      ; 130
    .long   DefaultISR                                      ; 131
    .long   DefaultISR                                      ; 132
    .long   DefaultISR                                      ; 133
    .long   DefaultISR                                      ; 134
    .long   DefaultISR                                      ; 135
    .long   DefaultISR                                      ; 136
    .long   DefaultISR                                      ; 137
    .long   DefaultISR                                      ; 138
    .long   DefaultISR                                      ; 139
    .long   DefaultISR                                      ; 140
    .long   DefaultISR                                      ; 141
    .long   DefaultISR                                      ; 142
    .long   DefaultISR                                      ; 143
    .long   DefaultISR                                      ; 144
    .long   DefaultISR                                      ; 145
    .long   DefaultISR                                      ; 146
    .long   DefaultISR                                      ; 147
    .long   DefaultISR                                      ; 148
    .long   DefaultISR                                      ; 149
    .long   DefaultISR                                      ; 150
    .long   DefaultISR                                      ; 151
    .long   DefaultISR                                      ; 152
    .long   DefaultISR                                      ; 153
    .long   DefaultISR                                      ; 154
    .long   DefaultISR                                      ; 155
    .long   DefaultISR                                      ; 156
    .long   DefaultISR                                      ; 157
    .long   DefaultISR                                      ; 158
    .long   DefaultISR                                      ; 159
    .long   DefaultISR                                      ; 160
    .long   DefaultISR                                      ; 161
    .long   DefaultISR                                      ; 162
    .long   DefaultISR                                      ; 163
    .long   DefaultISR                                      ; 164
    .long   DefaultISR                                      ; 165
    .long   DefaultISR                                      ; 166
    .long   DefaultISR                                      ; 167
    .long   DefaultISR                                      ; 168
    .long   DefaultISR                                      ; 169
    .long   DefaultISR                                      ; 170
    .long   DefaultISR                                      ; 171
    .long   DefaultISR                                      ; 172
    .long   DefaultISR                                      ; 173
    .long   DefaultISR                                      ; 174
    .long   DefaultISR                                      ; 175
    .long   DefaultISR                                      ; 176
    .long   DefaultISR                                      ; 177
    .long   DefaultISR                                      ; 178
    .long   DefaultISR                                      ; 179
    .long   DefaultISR                                      ; 180
    .long   DefaultISR                                      ; 181
    .long   DefaultISR                                      ; 182
    .long   DefaultISR                                      ; 183
    .long   DefaultISR                                      ; 184
    .long   DefaultISR                                      ; 185
    .long   DefaultISR                                      ; 186
    .long   DefaultISR                                      ; 187
    .long   DefaultISR                                      ; 188
    .long   DefaultISR                                      ; 189
    .long   DefaultISR                                      ; 190
    .long   DefaultISR                                      ; 191
    .long   DefaultISR                                      ; 192
    .long   DefaultISR                                      ; 193
    .long   DefaultISR                                      ; 194
    .long   DefaultISR                                      ; 195
    .long   DefaultISR                                      ; 196
    .long   DefaultISR                                      ; 197
    .long   DefaultISR                                      ; 198
    .long   DefaultISR                                      ; 199
    .long   DefaultISR                                      ; 200
    .long   DefaultISR                                      ; 201
    .long   DefaultISR                                      ; 202
    .long   DefaultISR                                      ; 203
    .long   DefaultISR                                      ; 204
    .long   DefaultISR                                      ; 205
    .long   DefaultISR                                      ; 206
    .long   DefaultISR                                      ; 207
    .long   DefaultISR                                      ; 208
    .long   DefaultISR                                      ; 209
    .long   DefaultISR                                      ; 210
    .long   DefaultISR                                      ; 211
    .long   DefaultISR                                      ; 212
    .long   DefaultISR                                      ; 213
    .long   DefaultISR                                      ; 214
    .long   DefaultISR                                      ; 215
    .long   DefaultISR                                      ; 216
    .long   DefaultISR                                      ; 217
    .long   DefaultISR                                      ; 218
    .long   DefaultISR                                      ; 219
    .long   DefaultISR                                      ; 220
    .long   DefaultISR                                      ; 221
    .long   DefaultISR                                      ; 222
    .long   DefaultISR                                      ; 223
    .long   DefaultISR                                      ; 224
    .long   DefaultISR                                      ; 225
    .long   DefaultISR                                      ; 226
    .long   DefaultISR                                      ; 227
    .long   DefaultISR                                      ; 228
    .long   DefaultISR                                      ; 229
    .long   DefaultISR                                      ; 230
    .long   DefaultISR                                      ; 231
    .long   DefaultISR                                      ; 232
    .long   DefaultISR                                      ; 233
    .long   DefaultISR                                      ; 234
    .long   DefaultISR                                      ; 235
    .long   DefaultISR                                      ; 236
    .long   DefaultISR                                      ; 237
    .long   DefaultISR                                      ; 238
    .long   DefaultISR                                      ; 239
    .long   DefaultISR                                      ; 240
    .long   DefaultISR                                      ; 241
    .long   DefaultISR                                      ; 242
    .long   DefaultISR                                      ; 243
    .long   DefaultISR                                      ; 244
    .long   DefaultISR                                      ; 245
    .long   DefaultISR                                      ; 246
    .long   DefaultISR                                      ; 247
    .long   DefaultISR                                      ; 248
    .long   DefaultISR                                      ; 249
    .long   DefaultISR                                      ; 250
    .long   DefaultISR                                      ; 251
    .long   DefaultISR                                      ; 252
    .long   DefaultISR                                      ; 253
    .long   DefaultISR                                      ; 254
    .long   0xFFFFFFFF                                      ;  Reserved for user TRIM value

    .size    __isr_vector, . - __isr_vector

; Flash Configuration
    .section .FlashConfig, "a"
    .long 0xFFFFFFFF     ; 8 bytes backdoor comparison key
    .long 0xFFFFFFFF     ;
    .long 0xFFFFFFFF     ; 4 bytes program flash protection bytes
    .long 0xFFFF7FFE     ; FDPROT:FEPROT:FOPT:FSEC(0xFE = unsecured)

    .text
    .thumb

; Reset Handler
    .thumb
    .align 4
    .short 1
    .globl   Reset_Handler
    .weak    Reset_Handler
    .type    Reset_Handler, $function
Reset_Handler:
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
    ldr     r0,=SystemInit
    blx     r0
#endif

    ; Init .data and .bss sections
    ldr     r0,=init_data_bss
    blx     r0
    cpsie   i               ; Unmask interrupts
    bl      main
JumpToSelf:
    b       JumpToSelf
    .ltorg
    .size Reset_Handler, . - Reset_Handler

    .align  1
    .thumb
    .weak DefaultISR
    .type DefaultISR, $function
DefaultISR:
    b       DefaultISR
    .size DefaultISR, . - DefaultISR

;    Macro to define default handlers. Default handler
;    will be weak symbol and just dead loops. They can be
;    overwritten by other handlers
    .macro def_irq_handler	handler_name
    .weak handler_name
    .set  handler_name, DefaultISR
    .endm

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

    .end
