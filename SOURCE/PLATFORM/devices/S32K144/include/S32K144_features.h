/*
** ###################################################################
**     Version:             rev. 1.0, 2015-04-09
**     Build:               b150416
**
**     Abstract:
**         Chip specific module features.
**
**     Copyright (c) 2015 Freescale Semiconductor, Inc.
**     All rights reserved.
**
**     Redistribution and use in source and binary forms, with or without modification,
**     are permitted provided that the following conditions are met:
**
**     o Redistributions of source code must retain the above copyright notice, this list
**       of conditions and the following disclaimer.
**
**     o Redistributions in binary form must reproduce the above copyright notice, this
**       list of conditions and the following disclaimer in the documentation and/or
**       other materials provided with the distribution.
**
**     o Neither the name of Freescale Semiconductor, Inc. nor the names of its
**       contributors may be used to endorse or promote products derived from this
**       software without specific prior written permission.
**
**     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
**     ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
**     WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
**     DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
**     ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
**     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
**     ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
**     (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
**     SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
**     http:                 www.freescale.com
**     mail:                 support@freescale.com
**
**     Revisions:
**     - rev. 1.0 (2015-04-09)
**         Initial version.
**
** ###################################################################
*/

#if !defined(__FSL_S32K144_FEATURES_H__)
#define __FSL_S32K144_FEATURES_H__


/* PORT module features */

/* @brief Has control lock (register bit PCR[LK]). */
#define FSL_FEATURE_PORT_HAS_PIN_CONTROL_LOCK (1)
/* @brief Has open drain control (register bit PCR[ODE]). */
#define FSL_FEATURE_PORT_HAS_OPEN_DRAIN (0)
/* @brief Has digital filter (registers DFER, DFCR and DFWR). */
#define FSL_FEATURE_PORT_HAS_DIGITAL_FILTER (1)
/* @brief Has DMA request (register bit field PCR[IRQC] values). */
#define FSL_FEATURE_PORT_HAS_DMA_REQUEST (1)
/* @brief Has pull resistor selection available. */
#define FSL_FEATURE_PORT_HAS_PULL_SELECTION (1)
/* @brief Has slew rate control (register bit PCR[SRE]). */
#define FSL_FEATURE_PORT_HAS_SLEW_RATE (0)
/* @brief Has passive filter (register bit field PCR[PFE]). */
#define FSL_FEATURE_PORT_HAS_PASSIVE_FILTER (1)
/* @brief Has drive strength control (register bit PCR[DSE]). */
#define FSL_FEATURE_PORT_HAS_DRIVE_STRENGTH (1)
/* @brief Has separate drive strength register (HDRVE). */
#define FSL_FEATURE_PORT_HAS_DRIVE_STRENGTH_REGISTER (0)
/* @brief Has glitch filter (register IOFLT). */
#define FSL_FEATURE_PORT_HAS_GLITCH_FILTER (0)

/* SOC module features */

/* @brief PORT availability on the SoC. */
#define FSL_FEATURE_SOC_PORT_COUNT (5)

#define FSL_FEATURE_SOC_SCG_COUNT (1)
/* @brief Slow IRC low range clock frequency. */
#define FSL_FEATURE_SCG_SIRC_LOW_RANGE_FREQ  (2000000U)
/* @brief Slow IRC high range clock frequency. */
#define FSL_FEATURE_SCG_SIRC_HIGH_RANGE_FREQ (8000000U)

/* @brief Fast IRC trimmed clock frequency(48MHz). */
#define FSL_FEATURE_SCG_FIRC_FREQ0  (48000000U)
/* @brief Fast IRC trimmed clock frequency(52MHz). */
#define FSL_FEATURE_SCG_FIRC_FREQ1  (52000000U)
/* @brief Fast IRC trimmed clock frequency(56MHz). */
#define FSL_FEATURE_SCG_FIRC_FREQ2  (56000000U)
/* @brief Fast IRC trimmed clock frequency(60MHz). */
#define FSL_FEATURE_SCG_FIRC_FREQ3  (60000000U)

/* FLASH module features */

/* @brief Is of type FTFA. */
#define FSL_FEATURE_FLASH_IS_FTFA (0)
/* @brief Is of type FTFE. */
#define FSL_FEATURE_FLASH_IS_FTFE (1)
/* @brief Is of type FTFL. */
#define FSL_FEATURE_FLASH_IS_FTFL (0)
/* @brief Has flags indicating the status of the FlexRAM (register bits FCNFG[EEERDY], FCNFG[RAMRDY] and FCNFG[PFLSH]). */
#define FSL_FEATURE_FLASH_HAS_FLEX_RAM_FLAGS (1)
/* @brief Has program flash swapping status flag (register bit FCNFG[SWAP]). */
#define FSL_FEATURE_FLASH_HAS_PFLASH_SWAPPING_STATUS_FLAG (0)
/* @brief Has EEPROM region protection (register FEPROT). */
#define FSL_FEATURE_FLASH_HAS_EEROM_REGION_PROTECTION (1)
/* @brief Has data flash region protection (register FDPROT). */
#define FSL_FEATURE_FLASH_HAS_DATA_FLASH_REGION_PROTECTION (1)
/* @brief P-Flash block count. */
#define FSL_FEATURE_FLASH_PFLASH_BLOCK_COUNT (1u)
/* @brief P-Flash block size. */
#define FSL_FEATURE_FLASH_PFLASH_BLOCK_SIZE (524288u)
/* @brief P-Flash sector size. */
#define FSL_FEATURE_FLASH_PFLASH_BLOCK_SECTOR_SIZE (4096u)
/* @brief P-Flash write unit size. */
#define FSL_FEATURE_FLASH_PFLASH_BLOCK_WRITE_UNIT_SIZE (8u)
/* @brief P-Flash block swap feature. */
#define FSL_FEATURE_FLASH_HAS_PFLASH_BLOCK_SWAP (0)
/* @brief Has FlexNVM memory. */
#define FSL_FEATURE_FLASH_HAS_FLEX_NVM (1)
/* @brief FlexNVM block count. */
#define FSL_FEATURE_FLASH_FLEX_NVM_BLOCK_COUNT (1u)
/* @brief FlexNVM block size. */
#define FSL_FEATURE_FLASH_FLEX_NVM_BLOCK_SIZE (65536u)
/* @brief FlexNVM sector size. */
#define FSL_FEATURE_FLASH_FLEX_NVM_BLOCK_SECTOR_SIZE (2048u)
/* @brief FlexNVM write unit size. */
#define FSL_FEATURE_FLASH_FLEX_NVM_BLOCK_WRITE_UNIT_SIZE (8u)
/* @brief FlexNVM start address. (Valid only if FlexNVM is available.) */
#define FSL_FEATURE_FLASH_FLEX_NVM_START_ADDRESS (0x10000000u)
/* @brief Has FlexRAM memory. */
#define FSL_FEATURE_FLASH_HAS_FLEX_RAM (1)
/* @brief FlexRAM size. */
#define FSL_FEATURE_FLASH_FLEX_RAM_SIZE (4096u)
/* @brief FlexRAM start address. (Valid only if FlexRAM is available.) */
#define FSL_FEATURE_FLASH_FLEX_RAM_START_ADDRESS (0x14000000u)
/* @brief Has 0x00 Read 1s Block command. */
#define FSL_FEATURE_FLASH_HAS_READ_1S_BLOCK_CMD (1)
/* @brief Has 0x01 Read 1s Section command. */
#define FSL_FEATURE_FLASH_HAS_READ_1S_SECTION_CMD (1)
/* @brief Has 0x02 Program Check command. */
#define FSL_FEATURE_FLASH_HAS_PROGRAM_CHECK_CMD (1)
/* @brief Has 0x03 Read Resource command. */
#define FSL_FEATURE_FLASH_HAS_READ_RESOURCE_CMD (1)
/* @brief Has 0x06 Program Longword command. */
#define FSL_FEATURE_FLASH_HAS_PROGRAM_LONGWORD_CMD (0)
/* @brief Has 0x07 Program Phrase command. */
#define FSL_FEATURE_FLASH_HAS_PROGRAM_PHRASE_CMD (1)
/* @brief Has 0x08 Erase Flash Block command. */
#define FSL_FEATURE_FLASH_HAS_ERASE_FLASH_BLOCK_CMD (1)
/* @brief Has 0x09 Erase Flash Sector command. */
#define FSL_FEATURE_FLASH_HAS_ERASE_FLASH_SECTOR_CMD (1)
/* @brief Has 0x0B Program Section command. */
#define FSL_FEATURE_FLASH_HAS_PROGRAM_SECTION_CMD (1)
/* @brief Has 0x40 Read 1s All Blocks command. */
#define FSL_FEATURE_FLASH_HAS_READ_1S_ALL_BLOCKS_CMD (1)
/* @brief Has 0x41 Read Once command. */
#define FSL_FEATURE_FLASH_HAS_READ_ONCE_CMD (1)
/* @brief Has 0x43 Program Once command. */
#define FSL_FEATURE_FLASH_HAS_PROGRAM_ONCE_CMD (1)
/* @brief Has 0x44 Erase All Blocks command. */
#define FSL_FEATURE_FLASH_HAS_ERASE_ALL_BLOCKS_CMD (1)
/* @brief Has 0x45 Verify Backdoor Access Key command. */
#define FSL_FEATURE_FLASH_HAS_VERIFY_BACKDOOR_ACCESS_KEY_CMD (1)
/* @brief Has 0x46 Swap Control command. */
#define FSL_FEATURE_FLASH_HAS_SWAP_CONTROL_CMD (0)
/* @brief Has 0x49 Erase All Blocks unsecure command. */
#define FSL_FEATURE_FLASH_HAS_ERASE_ALL_BLOCKS_UNSECURE_CMD (1)
/* @brief Has 0x80 Program Partition command. */
#define FSL_FEATURE_FLASH_HAS_PROGRAM_PARTITION_CMD (1)
/* @brief Has 0x81 Set FlexRAM Function command. */
#define FSL_FEATURE_FLASH_HAS_SET_FLEXRAM_FUNCTION_CMD (1)
/* @brief P-Flash Erase/Read 1st all block command address alignment. */
#define FSL_FEATURE_FLASH_PFLASH_BLOCK_CMD_ADDRESS_ALIGMENT (16u)
/* @brief P-Flash Erase sector command address alignment. */
#define FSL_FEATURE_FLASH_PFLASH_SECTOR_CMD_ADDRESS_ALIGMENT (16u)
/* @brief P-Flash Rrogram/Verify section command address alignment. */
#define FSL_FEATURE_FLASH_PFLASH_SECTION_CMD_ADDRESS_ALIGMENT (16u)
/* @brief P-Flash Read resource command address alignment. */
#define FSL_FEATURE_FLASH_PFLASH_RESOURCE_CMD_ADDRESS_ALIGMENT (8u)
/* @brief P-Flash Program check command address alignment. */
#define FSL_FEATURE_FLASH_PFLASH_CHECK_CMD_ADDRESS_ALIGMENT (4u)
/* @brief P-Flash Program check command address alignment. */
#define FSL_FEATURE_FLASH_PFLASH_SWAP_CONTROL_CMD_ADDRESS_ALIGMENT (0u)
/* @brief FlexNVM Erase/Read 1st all block command address alignment. */
#define FSL_FEATURE_FLASH_FLEX_NVM_BLOCK_CMD_ADDRESS_ALIGMENT (8u)
/* @brief FlexNVM Erase sector command address alignment. */
#define FSL_FEATURE_FLASH_FLEX_NVM_SECTOR_CMD_ADDRESS_ALIGMENT (8u)
/* @brief FlexNVM Rrogram/Verify section command address alignment. */
#define FSL_FEATURE_FLASH_FLEX_NVM_SECTION_CMD_ADDRESS_ALIGMENT (8u)
/* @brief FlexNVM Read resource command address alignment. */
#define FSL_FEATURE_FLASH_FLEX_NVM_RESOURCE_CMD_ADDRESS_ALIGMENT (8u)
/* @brief FlexNVM Program check command address alignment. */
#define FSL_FEATURE_FLASH_FLEX_NVM_CHECK_CMD_ADDRESS_ALIGMENT (4u)
/* @brief FlexNVM partition code 0000 mapping to data flash size in bytes (0xFFFFFFFF = reserved). */
#define FSL_FEATURE_FLASH_FLEX_NVM_DFLASH_SIZE_FOR_DEPART_0000 (0x00010000u)
/* @brief FlexNVM partition code 0001 mapping to data flash size in bytes (0xFFFFFFFF = reserved). */
#define FSL_FEATURE_FLASH_FLEX_NVM_DFLASH_SIZE_FOR_DEPART_0001 (0xFFFFFFFFu)
/* @brief FlexNVM partition code 0010 mapping to data flash size in bytes (0xFFFFFFFF = reserved). */
#define FSL_FEATURE_FLASH_FLEX_NVM_DFLASH_SIZE_FOR_DEPART_0010 (0xFFFFFFFFu)
/* @brief FlexNVM partition code 0011 mapping to data flash size in bytes (0xFFFFFFFF = reserved). */
#define FSL_FEATURE_FLASH_FLEX_NVM_DFLASH_SIZE_FOR_DEPART_0011 (0x00008000u)
/* @brief FlexNVM partition code 0100 mapping to data flash size in bytes (0xFFFFFFFF = reserved). */
#define FSL_FEATURE_FLASH_FLEX_NVM_DFLASH_SIZE_FOR_DEPART_0100 (0x00000000u)
/* @brief FlexNVM partition code 0101 mapping to data flash size in bytes (0xFFFFFFFF = reserved). */
#define FSL_FEATURE_FLASH_FLEX_NVM_DFLASH_SIZE_FOR_DEPART_0101 (0xFFFFFFFFu)
/* @brief FlexNVM partition code 0110 mapping to data flash size in bytes (0xFFFFFFFF = reserved). */
#define FSL_FEATURE_FLASH_FLEX_NVM_DFLASH_SIZE_FOR_DEPART_0110 (0xFFFFFFFFu)
/* @brief FlexNVM partition code 0111 mapping to data flash size in bytes (0xFFFFFFFF = reserved). */
#define FSL_FEATURE_FLASH_FLEX_NVM_DFLASH_SIZE_FOR_DEPART_0111 (0xFFFFFFFFu)
/* @brief FlexNVM partition code 1000 mapping to data flash size in bytes (0xFFFFFFFF = reserved). */
#define FSL_FEATURE_FLASH_FLEX_NVM_DFLASH_SIZE_FOR_DEPART_1000 (0x00000000u)
/* @brief FlexNVM partition code 1001 mapping to data flash size in bytes (0xFFFFFFFF = reserved). */
#define FSL_FEATURE_FLASH_FLEX_NVM_DFLASH_SIZE_FOR_DEPART_1001 (0xFFFFFFFFu)
/* @brief FlexNVM partition code 1010 mapping to data flash size in bytes (0xFFFFFFFF = reserved). */
#define FSL_FEATURE_FLASH_FLEX_NVM_DFLASH_SIZE_FOR_DEPART_1010 (0x00004000u)
/* @brief FlexNVM partition code 1011 mapping to data flash size in bytes (0xFFFFFFFF = reserved). */
#define FSL_FEATURE_FLASH_FLEX_NVM_DFLASH_SIZE_FOR_DEPART_1011 (0x00008000u)
/* @brief FlexNVM partition code 1100 mapping to data flash size in bytes (0xFFFFFFFF = reserved). */
#define FSL_FEATURE_FLASH_FLEX_NVM_DFLASH_SIZE_FOR_DEPART_1100 (0x00010000u)
/* @brief FlexNVM partition code 1101 mapping to data flash size in bytes (0xFFFFFFFF = reserved). */
#define FSL_FEATURE_FLASH_FLEX_NVM_DFLASH_SIZE_FOR_DEPART_1101 (0xFFFFFFFFu)
/* @brief FlexNVM partition code 1110 mapping to data flash size in bytes (0xFFFFFFFF = reserved). */
#define FSL_FEATURE_FLASH_FLEX_NVM_DFLASH_SIZE_FOR_DEPART_1110 (0xFFFFFFFFu)
/* @brief FlexNVM partition code 1111 mapping to data flash size in bytes (0xFFFFFFFF = reserved). */
#define FSL_FEATURE_FLASH_FLEX_NVM_DFLASH_SIZE_FOR_DEPART_1111 (0x00010000u)
/* @brief Emulated eeprom size code 0000 mapping to emulated eeprom size in bytes (0xFFFF = reserved). */
#define FSL_FEATURE_FLASH_FLEX_NVM_EEPROM_SIZE_FOR_EEESIZE_0000 (0xFFFFu)
/* @brief Emulated eeprom size code 0001 mapping to emulated eeprom size in bytes (0xFFFF = reserved). */
#define FSL_FEATURE_FLASH_FLEX_NVM_EEPROM_SIZE_FOR_EEESIZE_0001 (0xFFFFu)
/* @brief Emulated eeprom size code 0010 mapping to emulated eeprom size in bytes (0xFFFF = reserved). */
#define FSL_FEATURE_FLASH_FLEX_NVM_EEPROM_SIZE_FOR_EEESIZE_0010 (0x1000u)
/* @brief Emulated eeprom size code 0011 mapping to emulated eeprom size in bytes (0xFFFF = reserved). */
#define FSL_FEATURE_FLASH_FLEX_NVM_EEPROM_SIZE_FOR_EEESIZE_0011 (0x0800u)
/* @brief Emulated eeprom size code 0100 mapping to emulated eeprom size in bytes (0xFFFF = reserved). */
#define FSL_FEATURE_FLASH_FLEX_NVM_EEPROM_SIZE_FOR_EEESIZE_0100 (0x0400u)
/* @brief Emulated eeprom size code 0101 mapping to emulated eeprom size in bytes (0xFFFF = reserved). */
#define FSL_FEATURE_FLASH_FLEX_NVM_EEPROM_SIZE_FOR_EEESIZE_0101 (0x0200u)
/* @brief Emulated eeprom size code 0110 mapping to emulated eeprom size in bytes (0xFFFF = reserved). */
#define FSL_FEATURE_FLASH_FLEX_NVM_EEPROM_SIZE_FOR_EEESIZE_0110 (0x0100u)
/* @brief Emulated eeprom size code 0111 mapping to emulated eeprom size in bytes (0xFFFF = reserved). */
#define FSL_FEATURE_FLASH_FLEX_NVM_EEPROM_SIZE_FOR_EEESIZE_0111 (0x0080u)
/* @brief Emulated eeprom size code 1000 mapping to emulated eeprom size in bytes (0xFFFF = reserved). */
#define FSL_FEATURE_FLASH_FLEX_NVM_EEPROM_SIZE_FOR_EEESIZE_1000 (0x0040u)
/* @brief Emulated eeprom size code 1001 mapping to emulated eeprom size in bytes (0xFFFF = reserved). */
#define FSL_FEATURE_FLASH_FLEX_NVM_EEPROM_SIZE_FOR_EEESIZE_1001 (0x0020u)
/* @brief Emulated eeprom size code 1010 mapping to emulated eeprom size in bytes (0xFFFF = reserved). */
#define FSL_FEATURE_FLASH_FLEX_NVM_EEPROM_SIZE_FOR_EEESIZE_1010 (0xFFFFu)
/* @brief Emulated eeprom size code 1011 mapping to emulated eeprom size in bytes (0xFFFF = reserved). */
#define FSL_FEATURE_FLASH_FLEX_NVM_EEPROM_SIZE_FOR_EEESIZE_1011 (0xFFFFu)
/* @brief Emulated eeprom size code 1100 mapping to emulated eeprom size in bytes (0xFFFF = reserved). */
#define FSL_FEATURE_FLASH_FLEX_NVM_EEPROM_SIZE_FOR_EEESIZE_1100 (0xFFFFu)
/* @brief Emulated eeprom size code 1101 mapping to emulated eeprom size in bytes (0xFFFF = reserved). */
#define FSL_FEATURE_FLASH_FLEX_NVM_EEPROM_SIZE_FOR_EEESIZE_1101 (0xFFFFu)
/* @brief Emulated eeprom size code 1110 mapping to emulated eeprom size in bytes (0xFFFF = reserved). */
#define FSL_FEATURE_FLASH_FLEX_NVM_EEPROM_SIZE_FOR_EEESIZE_1110 (0xFFFFu)
/* @brief Emulated eeprom size code 1111 mapping to emulated eeprom size in bytes (0xFFFF = reserved). */
#define FSL_FEATURE_FLASH_FLEX_NVM_EEPROM_SIZE_FOR_EEESIZE_1111 (0x0000u)

/* CAN module features */

/* @brief Frames available in Rx FIFO flag shift */
#define FSL_FEATURE_CAN_RXFIFO_FRAME_AVAILABLE  (5)
/* @brief Rx FIFO warning flag shift */
#define FSL_FEATURE_CAN_RXFIFO_WARNING          (6)
/* @brief Rx FIFO overflow flag shift */
#define FSL_FEATURE_CAN_RXFIFO_OVERFLOW         (7)

/* LPUART module features */

/* @brief Has receive FIFO overflow detection (bit field CFIFO[RXOFE]). */
#define FSL_FEATURE_LPUART_HAS_IRQ_EXTENDED_FUNCTIONS (0)
/* @brief Has low power features (can be enabled in wait mode via register bit C1[DOZEEN] or CTRL[DOZEEN] if the registers are 32-bit wide). */
#define FSL_FEATURE_LPUART_HAS_LOW_POWER_UART_SUPPORT (1)
/* @brief Has extended data register ED (or extra flags in the DATA register if the registers are 32-bit wide). */
#define FSL_FEATURE_LPUART_HAS_EXTENDED_DATA_REGISTER_FLAGS (1)
/* @brief Capacity (number of entries) of the transmit/receive FIFO (or zero if no FIFO is available). */
#define FSL_FEATURE_LPUART_HAS_FIFO (0)
/* @brief Has 32-bit register MODIR */
#define FSL_FEATURE_LPUART_HAS_MODIR (1)
/* @brief Hardware flow control (RTS, CTS) is supported. */
#define FSL_FEATURE_LPUART_HAS_MODEM_SUPPORT (1)
/* @brief Infrared (modulation) is supported. */
#define FSL_FEATURE_LPUART_HAS_IR_SUPPORT (1)
/* @brief 2 bits long stop bit is available. */
#define FSL_FEATURE_LPUART_HAS_STOP_BIT_CONFIG_SUPPORT (1)
/* @brief Maximal data width without parity bit. */
#define FSL_FEATURE_LPUART_HAS_10BIT_DATA_SUPPORT (1)
/* @brief Baud rate fine adjustment is available. */
#define FSL_FEATURE_LPUART_HAS_BAUD_RATE_FINE_ADJUST_SUPPORT (0)
/* @brief Baud rate oversampling is available (has bit fields C4[OSR], C5[BOTHEDGE], C5[RESYNCDIS] or BAUD[OSR], BAUD[BOTHEDGE], BAUD[RESYNCDIS] if the registers are 32-bit wide). */
#define FSL_FEATURE_LPUART_HAS_BAUD_RATE_OVER_SAMPLING_SUPPORT (1)
/* @brief Baud rate oversampling is available. */
#define FSL_FEATURE_LPUART_HAS_RX_RESYNC_SUPPORT (1)
/* @brief Baud rate oversampling is available. */
#define FSL_FEATURE_LPUART_HAS_BOTH_EDGE_SAMPLING_SUPPORT (1)
/* @brief Peripheral type. */
#define FSL_FEATURE_LPUART_IS_SCI (1)
/* @brief Capacity (number of entries) of the transmit/receive FIFO (or zero if no FIFO is available). */
#define FSL_FEATURE_LPUART_FIFO_SIZE (4)
/* @brief Maximal data width without parity bit. */
#define FSL_FEATURE_LPUART_MAX_DATA_WIDTH_WITH_NO_PARITY (10)
/* @brief Maximal data width with parity bit. */
#define FSL_FEATURE_LPUART_MAX_DATA_WIDTH_WITH_PARITY (9)
/* @brief Supports two match addresses to filter incoming frames. */
#define FSL_FEATURE_LPUART_HAS_ADDRESS_MATCHING (1)
/* @brief Has transmitter/receiver DMA enable bits C5[TDMAE]/C5[RDMAE] (or BAUD[TDMAE]/BAUD[RDMAE] if the registers are 32-bit wide). */
#define FSL_FEATURE_LPUART_HAS_DMA_ENABLE (1)
/* @brief Has transmitter/receiver DMA select bits C4[TDMAS]/C4[RDMAS], resp. C5[TDMAS]/C5[RDMAS] if IS_SCI = 0. */
#define FSL_FEATURE_LPUART_HAS_DMA_SELECT (0)
/* @brief Data character bit order selection is supported (bit field S2[MSBF] or STAT[MSBF] if the registers are 32-bit wide). */
#define FSL_FEATURE_LPUART_HAS_BIT_ORDER_SELECT (1)
/* @brief Has smart card (ISO7816 protocol) support and no improved smart card support. */
#define FSL_FEATURE_LPUART_HAS_SMART_CARD_SUPPORT (0)
/* @brief Has improved smart card (ISO7816 protocol) support. */
#define FSL_FEATURE_LPUART_HAS_IMPROVED_SMART_CARD_SUPPORT (0)
/* @brief Has local operation network (CEA709.1-B protocol) support. */
#define FSL_FEATURE_LPUART_HAS_LOCAL_OPERATION_NETWORK_SUPPORT (0)
/* @brief Has 32-bit registers (BAUD, STAT, CTRL, DATA, MATCH, MODIR) instead of 8-bit (BDH, BDL, C1, S1, D, etc.). */
#define FSL_FEATURE_LPUART_HAS_32BIT_REGISTERS (1)
/* @brief Lin break detect available (has bit BDH[LBKDIE]). */
#define FSL_FEATURE_LPUART_HAS_LIN_BREAK_DETECT (0)
/* @brief UART stops in Wait mode available (has bit C1[UARTSWAI]). */
#define FSL_FEATURE_LPUART_HAS_WAIT_MODE_OPERATION (0)
/* @brief Flag clearance mask for STAT register. */
#define FSL_FEATURE_LPUART_STAT_REG_FLAGS_MASK (0xC01FC000)
/* @brief Flag clearance mask for FIFO register. */
#define FSL_FEATURE_LPUART_FIFO_REG_FLAGS_MASK (0x00030000)
/* @brief Default oversampling ratio. */
#define FSL_FEATURE_LPUART_DEFAULT_OSR (0xF)
/* @brief Default baud rate modulo divisor. */
#define FSL_FEATURE_LPUART_DEFAULT_SBR (0x4)
/* @brief Has separate DMA RX and TX requests. */
#define FSL_FEATURE_LPUART_HAS_SEPARATE_DMA_RX_TX_REQn(x) \
    ((x) == 0 ? (1) : (-1))

/* PDB module features */

/* @brief Define the count of supporting ADC channels per each PDB. */
#define FSL_FEATURE_PDB_ADC_CHANNEL_COUNT     (2U)
/* @brief Define the count of supporting ADC pre-trigger for each channel. */
#define FSL_FEATURE_PDB_ADC_PRE_CHANNEL_COUNT (8U)
/* @brief Define the count of supporting Pulse-Out outputs per each PDB. */
#define FSL_FEATURE_PDB_PODLY_COUNT           (1U)

/* Interrupt module features */

/* @brief Lowest interrupt request number. */
#define FSL_FEATURE_INTERRUPT_IRQ_MIN (-16)
/* @brief Highest interrupt request number. */
#define FSL_FEATURE_INTERRUPT_IRQ_MAX (98)
/**< Number of priority bits implemented in the NVIC */
#define FSL_FEATURE_NVIC_PRIO_BITS    (4U)


/* SMC module features */

/* @brief Has partial stop option (register bit STOPCTRL[PSTOPO]). */
#define FSL_FEATURE_SMC_HAS_PSTOPO (1)
/* @brief Has high speed run mode (register bit PMPROT[AHSRUN]). */
#define FSL_FEATURE_SMC_HAS_HIGH_SPEED_RUN_MODE (1)


/* MPU module features */

/* @brief Has process identifier support. */
#define FSL_FEATURE_MPU_HAS_PROCESS_IDENTIFIER              (1U)
/* @brief Specifies total number of bus masters. */
#define FSL_FEATURE_MPU_MASTER_COUNT                        (3U)
/* @brief Specifies maximum number of masters which have separated
privilege rights for user and supervisor mode accesses (e.g. master0~3 in S32K144).
*/
#define FSL_FEATURE_MPU_MAX_LOW_MASTER_NUMBER               (3U)
/* @brief Specifies maximum number of masters which have only
read and write permissions (e.g. master4~7 in S32K144).
*/
#define FSL_FEATURE_MPU_MAX_HIGH_MASTER_NUMBER              (7U)

/* @brief Specifies number of set access control right bits for
   masters which have separated privilege rights for user and
   supervisor mode accesses (e.g. master0~3 in S32K144).
*/
#define FSL_FEATURE_MPU_LOW_MASTER_CONTROL_WIDTH            (6U)
/* @brief Specifies number of set access control right bits for
   masters which have only read and write permissions(e.g. master4~7 in S32K144).
*/
#define FSL_FEATURE_MPU_HIGH_MASTER_CONTROL_WIDTH           (2U)

/* @brief The MPU Logical Bus Master Number for core bus master. */
#define FSL_FEATURE_MPU_MASTER_CORE                         (0U)
/* @brief The MPU Logical Bus Master Number for Debugger master. */
#define FSL_FEATURE_MPU_MASTER_DEBUGGER                     (1U)
/* @brief The MPU Logical Bus Master Number for DMA master. */
#define FSL_FEATURE_MPU_MASTER_DMA                          (2U)

/* @brief Specifies total number of slave ports. */
#define FSL_FEATURE_MPU_SLAVE_COUNT                         (4U)
/* @brief The MPU Slave Port Assignment for Flash Controller and boot ROM. */
#define FSL_FEATURE_MPU_SLAVE_FLASH_BOOTROM                 (0U)
/* @brief The MPU Slave Port Assignment for SRAM back door. */
#define FSL_FEATURE_MPU_SLAVE_SRAM_BACKDOOR                 (1U)
/* @brief The MPU Slave Port Assignment for SRAM_L front door. */
#define FSL_FEATURE_MPU_SLAVE_SRAM_L_FRONTDOOR              (2U)
/* @brief The MPU Slave Port Assignment for SRAM_U front door. */
#define FSL_FEATURE_MPU_SLAVE_SRAM_U_FRONTDOOR              (3U)


/* WDOG module features */

/* @brief The value used for unlocking the WDOG. */
#define FSL_FEATURE_WDOG_UNLOCK_VALUE       (0xD928C520U)
/* @brief The value used for resetting the WDOG counter. */
#define FSL_FEATURE_WDOG_TRIGGER_VALUE      (0xB480A602U)
/* @brief The reset value of the timeout register. */
#define FSL_FEATURE_WDOG_TO_RESET_VALUE     (0x400U)
/* @brief The reset value of the window register. */
#define FSL_FEATURE_WDOG_WIN_RESET_VALUE    (0x0U)
/* @brief The mask of the reserved bit in the CS register. */
#define FSL_FEATURE_WDOG_CS_RESERVED_MASK   (0x2000U)


/* EDMA module features */

/* @brief Number of EDMA channels. */
#define FSL_FEATURE_EDMA_MODULE_CHANNELS (16)
/* @brief eDMA module has error interrupt. */
#define FSL_FEATURE_EDMA_HAS_ERROR_IRQ
/* @brief Conversion from channel index to DCHPRI index. */
#define FSL_FEATURE_EDMA_CHN_TO_DCHPRI_INDEX(x) (x ^ 3U)


/* DMAMUX module features */

/* @brief Number of DMA channels. */
#define FSL_FEATURE_DMAMUX_MODULE_CHANNELS (16)
/* @brief Has the periodic trigger capability */
#define FSL_FEATURE_DMAMUX_HAS_TRIG (1)


/* FTM module features */
/* @brief Number of PWM channels */
#define FSL_FEATURE_FTM_CHANNEL_COUNT           (8U)
/* @brief Number of fault channels */
#define FTM_FEATURE_FAULT_CHANNELS              (4U)
/* @brief Width of control channel */
#define FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH     (8U)
/* @brief Output channel offset */
#define FTM_FEATURE_OUTPUT_CHANNEL_OFFSET       (16U)
/* @brief Max counter value */
#define FTM_FEATURE_CNT_MAX_VALUE_U32           (0x0000FFFFU)
/* @brief Input capture for single shot */
#define FTM_FEATURE_INPUT_CAPTURE_SINGLE_SHOT   (2U)


/* CLOCK names */

/*! @brief Clock names. */
typedef enum _clock_names {

    /* Main clocks */
    CORE_CLOCK                   = 0u,       /*!< Core clock                     */
    PLATFORM_CLOCK               = 1u,       /*!< Platform clock                 */
    BUS_CLOCK                    = 2u,       /*!< Bus clock                      */
    SLOW_CLOCK                   = 3u,       /*!< Slow clock                     */

    /* Other internal clocks used by peripherals. */
    SIRC_CLOCK                   = 4u,       /*!< SIRC clock                     */
    FIRC_CLOCK                   = 5u,       /*!< FIRC clock                     */
    SOSC_CLOCK                   = 6u,       /*!< SOSC clock                     */
    SPLL_CLOCK                   = 7u,       /*!< SPLL clock                     */
    RTC_CLKIN_CLOCK              = 8u,       /*!< RTC_CLKIN clock                */
    SCG_END_OF_CLOCKS            = 9u,       /*!< End of SCG clocks              */

    /* SIM clocks */
    SIM_FTM0_CLOCKSEL            = 10u,      /*!< FTM0 External Clock Pin Select */
    SIM_FTM1_CLOCKSEL            = 11u,      /*!< FTM1 External Clock Pin Select */
    SIM_FTM2_CLOCKSEL            = 12u,      /*!< FTM2 External Clock Pin Select */
    SIM_FTM3_CLOCKSEL            = 13u,      /*!< FTM3 External Clock Pin Select */
                                                          
    SIM_CLKOUTSELL               = 14u,      /*!< CLKOUT Select                  */
    SIM_CLK32K_CLOCK             = 15u,      /*!< CLK32K clock                   */
    SIM_LPO_CLOCK                = 16u,      /*!< LPO clock                      */
    SIM_LPO_1K_CLOCK             = 17u,      /*!< LPO 1KHz clock                 */
    SIM_LPO_32K_CLOCK            = 18u,      /*!< LPO 32KHz clock                */
    SIM_END_OF_CLOCKS            = 19u,      /*!< End of SIM clocks              */

    /* PCC clocks */
    PCC_DMA0_CLOCK               = 20u,      /*!< DMA0 clock source              */
    PCC_MPU0_CLOCK               = 21u,      /*!< MPU0 clock source              */
    PCC_FLASH0_CLOCK             = 22u,      /*!< FLASH0 clock source            */
    PCC_DMAMUX0_CLOCK            = 23u,      /*!< DMAMUX0 clock source           */
    PCC_CRC0_CLOCK               = 24u,      /*!< CRC0 clock source              */
    PCC_RTC0_CLOCK               = 25u,      /*!< RTC0 clock source              */
    PCC_ATX0_CLOCK               = 26u,      /*!< ATX0 clock source              */
    PCC_SIM0_CLOCK               = 27u,      /*!< SIM0 clock source              */
    PCC_PORTA_CLOCK              = 28u,      /*!< PORTA clock source             */
    PCC_PORTB_CLOCK              = 29u,      /*!< PORTB clock source             */
    PCC_PORTC_CLOCK              = 30u,      /*!< PORTC clock source             */
    PCC_PORTD_CLOCK              = 31u,      /*!< PORTD clock source             */
    PCC_PORTE_CLOCK              = 32u,      /*!< PORTE clock source             */
    PCC_RTC_OSC0_CLOCK           = 33u,      /*!< RTC_OSC0 clock source          */
    PCC_EWM0_CLOCK               = 34u,      /*!< EWM0 clock source              */
    PCC_TRGMUX0_CLOCK            = 35u,      /*!< TRGMUX0 clock source           */
    PCC_SCG0_CLOCK               = 36u,      /*!< SCG0 clock source              */
    PCC_CMP0_CLOCK               = 37u,      /*!< CMP0 clock source              */
    PCC_END_OF_BUS_CLOCKS        = 38u,      /*!< End of BUS clocks              */
    PCC_FLEXCAN0_CLOCK           = 39u,      /*!< FLEXCAN0 clock source          */
    PCC_FLEXCAN1_CLOCK           = 40u,      /*!< FLEXCAN1 clock source          */
    PCC_FLEXCAN2_CLOCK           = 41u,      /*!< FLEXCAN2 clock source          */
    PCC_PDB0_CLOCK               = 42u,      /*!< PDB0 clock source              */
    PCC_PDB1_CLOCK               = 43u,      /*!< PDB1 clock source              */
    PCC_END_OF_SYS_CLOCKS        = 44u,      /*!< End of SYS clocks              */
    PCC_FLEXTMR3_CLOCK           = 45u,      /*!< FLEXTMR3 clock source          */
    PCC_FLEXTMR0_CLOCK           = 46u,      /*!< FLEXTMR0 clock source          */
    PCC_FLEXTMR1_CLOCK           = 47u,      /*!< FLEXTMR1 clock source          */
    PCC_FLEXTMR2_CLOCK           = 48u,      /*!< FLEXTMR2 clock source          */
    PCC_END_OF_ASYNCH_DIV1_CLOCKS= 49u,      /*!< End of ASYNCH DIV1 clocks      */
    PCC_ADC1_CLOCK               = 50u,      /*!< ADC1 clock source              */
    PCC_LPSPI0_CLOCK             = 51u,      /*!< LPSPI0 clock source            */
    PCC_LPSPI1_CLOCK             = 52u,      /*!< LPSPI1 clock source            */
    PCC_LPSPI2_CLOCK             = 53u,      /*!< LPSPI2 clock source            */
    PCC_LPIT0_CLOCK              = 54u,      /*!< LPIT0 clock source             */
    PCC_ADC0_CLOCK               = 55u,      /*!< ADC0 clock source              */
    PCC_LPTMR0_CLOCK             = 56u,      /*!< LPTMR0 clock source            */
    PCC_WDOG0_CLOCK              = 57u,      /*!< WDOG0 clock source             */
    PCC_FLEXIO0_CLOCK            = 58u,      /*!< FLEXIO0 clock source           */
    PCC_LPI2C0_CLOCK             = 59u,      /*!< LPI2C0 clock source            */
    PCC_LPI2C1_CLOCK             = 60u,      /*!< LPI2C1 clock source            */
    PCC_LPUART0_CLOCK            = 61u,      /*!< LPUART0 clock source           */
    PCC_LPUART1_CLOCK            = 62u,      /*!< LPUART1 clock source           */
    PCC_LPUART2_CLOCK            = 63u,      /*!< LPUART2 clock source           */
    PCC_LPUART3_CLOCK            = 64u,      /*!< LPUART3 clock source           */
    PCC_END_OF_ASYNCH_DIV2_CLOCKS= 65u,      /*!< End of ASYNCH DIV2 clocks      */
    PCC_END_OF_CLOCKS            = 66u,      /*!< End of PCC clocks              */
    CLOCK_NAME_COUNT             = 67u,      /*!< The total number of entries    */
                   
} clock_names_t;                  
                                  
#define PCC_INVALID_INDEX  0

  /*! @brief PCC clock name mappings
   *  Mappings between clock names and peripheral clock control indexes.
   *  If there is no peripheral clock control index for a clock name, 
   *  then the corresponding value is PCC_INVALID_INDEX.
   */
#define PCC_CLOCK_NAME_MAPPINGS \
{                                                                              \
PCC_INVALID_INDEX,                 /*!< Core clock                     0  */   \
PCC_INVALID_INDEX,                 /*!< Platform clock                 1  */   \
PCC_INVALID_INDEX,                 /*!< Bus clock                      2  */   \
PCC_INVALID_INDEX,                 /*!< Slow clock                     3  */   \
PCC_INVALID_INDEX,                 /*!< SIRC clock                     4  */   \
PCC_INVALID_INDEX,                 /*!< FIRC clock                     5  */   \
PCC_INVALID_INDEX,                 /*!< SOSC clock                     6  */   \
PCC_INVALID_INDEX,                 /*!< SPLL clock                     7  */   \
PCC_INVALID_INDEX,                 /*!< RTC_CLKIN clock                8  */   \
PCC_INVALID_INDEX,                 /*!< End of SCG clocks              9  */   \
PCC_INVALID_INDEX,                 /*!< FTM0 External Clock Pin Select 10 */   \
PCC_INVALID_INDEX,                 /*!< FTM1 External Clock Pin Select 11 */   \
PCC_INVALID_INDEX,                 /*!< FTM2 External Clock Pin Select 12 */   \
PCC_INVALID_INDEX,                 /*!< FTM3 External Clock Pin Select 13 */   \
PCC_INVALID_INDEX,                 /*!< CLKOUT Select                  14 */   \
PCC_INVALID_INDEX,                 /*!< CLK32K clock                   15 */   \
PCC_INVALID_INDEX,                 /*!< LPO clock                      16 */   \
PCC_INVALID_INDEX,                 /*!< LPO 1KHz clock                 17 */   \
PCC_INVALID_INDEX,                 /*!< LPO 32KHz clock                18 */   \
PCC_INVALID_INDEX,                 /*!< End of SIM clocks              19 */   \
PCC_DMA0_INDEX,                    /*!< DMA0 clock source              20 */   \
PCC_MPU_INDEX,                     /*!< MPU0 clock source              21 */   \
PCC_FLASH_INDEX,                   /*!< FLASH0 clock source            22 */   \
PCC_DMAMUX0_INDEX,                 /*!< DMAMUX0 clock source           23 */   \
PCC_CRC_INDEX,                     /*!< CRC0 clock source              24 */   \
PCC_RTC_INDEX,                     /*!< RTC0 clock source              25 */   \
PCC_ATX_INDEX,                     /*!< ATX0 clock source              26 */   \
PCC_SIM_INDEX,                     /*!< SIM0 clock source              27 */   \
PCC_PORTA_INDEX,                   /*!< PORTA clock source             28 */   \
PCC_PORTB_INDEX,                   /*!< PORTB clock source             29 */   \
PCC_PORTC_INDEX,                   /*!< PORTC clock source             30 */   \
PCC_PORTD_INDEX,                   /*!< PORTD clock source             31 */   \
PCC_PORTE_INDEX,                   /*!< PORTE clock source             32 */   \
PCC_RTC_OSC_INDEX,                 /*!< RTC_OSC0 clock source          33 */   \
PCC_EWM_INDEX,                     /*!< EWM0 clock source              34 */   \
PCC_TRGMUX_INDEX,                  /*!< TRGMUX0 clock source           35 */   \
PCC_SCG_INDEX,                     /*!< SCG0 clock source              36 */   \
PCC_CMP0_INDEX,                    /*!< CMP0 clock source              37 */   \
PCC_INVALID_INDEX,                 /*!< End of BUS clocks              38 */   \
PCC_FLEXCAN0_INDEX,                /*!< FLEXCAN0 clock source          39 */   \
PCC_FLEXCAN1_INDEX,                /*!< FLEXCAN1 clock source          40 */   \
PCC_FLEXCAN2_INDEX,                /*!< FLEXCAN2 clock source          41 */   \
PCC_PDB0_INDEX,                    /*!< PDB0 clock source              42 */   \
PCC_PDB1_INDEX,                    /*!< PDB1 clock source              43 */   \
PCC_INVALID_INDEX,                 /*!< End of SYS clocks              44 */   \
PCC_FLEXTMR3_INDEX,                /*!< FLEXTMR3 clock source          45 */   \
PCC_FLEXTMR0_INDEX,                /*!< FLEXTMR0 clock source          46 */   \
PCC_FLEXTMR1_INDEX,                /*!< FLEXTMR1 clock source          47 */   \
PCC_FLEXTMR2_INDEX,                /*!< FLEXTMR2 clock source          48 */   \
PCC_INVALID_INDEX,                 /*!< End of ASYNCH DIV1 clocks      49 */   \
PCC_ADC1_INDEX,                    /*!< ADC1 clock source              50 */   \
PCC_LPSPI0_INDEX,                  /*!< LPSPI0 clock source            51 */   \
PCC_LPSPI1_INDEX,                  /*!< LPSPI1 clock source            52 */   \
PCC_LPSPI2_INDEX,                  /*!< LPSPI2 clock source            53 */   \
PCC_LPIT0_INDEX,                   /*!< LPIT0 clock source             54 */   \
PCC_ADC0_INDEX,                    /*!< ADC0 clock source              55 */   \
PCC_LPTMR0_INDEX,                  /*!< LPTMR0 clock source            56 */   \
PCC_WDOG_INDEX,                    /*!< WDOG0 clock source             57 */   \
PCC_FLEXIO_INDEX,                  /*!< FLEXIO0 clock source           58 */   \
PCC_LPI2C0_INDEX,                  /*!< LPI2C0 clock source            59 */   \
PCC_LPI2C1_INDEX,                  /*!< LPI2C1 clock source            60 */   \
PCC_LPUART0_INDEX,                 /*!< LPUART0 clock source           61 */   \
PCC_LPUART1_INDEX,                 /*!< LPUART1 clock source           62 */   \
PCC_LPUART2_INDEX,                 /*!< LPUART2 clock source           63 */   \
PCC_LPUART3_INDEX,                 /*!< LPUART3 clock source           64 */   \
PCC_INVALID_INDEX,                 /*!< End of ASYNCH DIV2 clocks      65 */   \
PCC_INVALID_INDEX,                 /*!< End of PCC clocks              66 */   \
PCC_INVALID_INDEX,                 /*!< The total number of entries    67 */   \
}






#endif /* __FSL_S32K144_FEATURES_H__ */

/*******************************************************************************
 * EOF
 ******************************************************************************/
