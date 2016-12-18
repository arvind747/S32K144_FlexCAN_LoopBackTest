#ifndef __FSL_SBC_33903_4_5_DRIVER_H__
#define __FSL_SBC_33903_4_5_DRIVER_H__
/* MUX Register. */
#define SBC_MUX_ADDR                 0x0U
#define SBC_MUX_IOATT_MASK           0x8U
#define SBC_MUX_IOATT_SHIFT          3U
#define SBC_MUX_IOATT(x)             (((uint8_t)(((uint8_t)(x))<<SBC_MUX_IOATT_SHIFT))&SBC_MUX_IOATT_MASK)
#define SBC_MUX_INT2K_MASK           0x10U
#define SBC_MUX_INT2K_SHIFT          4U
#define SBC_MUX_INT2K(x)             (((uint8_t)(((uint8_t)(x))<<SBC_MUX_INT2K_SHIFT))&SBC_MUX_INT2K_MASK)
#define SBC_MUX_MUX_MASK             0xE0U
#define SBC_MUX_MUX_SHIFT            5U
#define SBC_MUX_MUX(x)               (((uint8_t)(((uint8_t)(x))<<SBC_MUX_MUX_SHIFT))&SBC_MUX_MUX_MASK)

/* Internal Memory Register RAM_A. */
#define SBC_RAMA_ADDR                0x1U

/* Internal Memory Register RAM_B. */
#define SBC_RAMB_ADDR                0x2U

/* Internal Memory Register RAM_C. */
#define SBC_RAMC_ADDR                0x3U

/* Internal Memory Register RAM_D. */
#define SBC_RAMD_ADDR                0x4U

/* Initialization Regulator Register. */
#define SBC_INITREG_ADDR             0x5U
#define SBC_INITREG_CYC_MASK         0x3U
#define SBC_INITREG_CYC_SHIFT        0U
#define SBC_INITREG_CYC(x)           (((uint8_t)(((uint8_t)(x))<<SBC_INITREG_CYC_SHIFT))&SBC_INITREG_CYC_MASK)
#define SBC_INITREG_VAUX_MASK        0x4U
#define SBC_INITREG_VAUX_SHIFT       2U
#define SBC_INITREG_VAUX(x)          (((uint8_t)(((uint8_t)(x))<<SBC_INITREG_VAUX_SHIFT))&SBC_INITREG_VAUX_MASK)
#define SBC_INITREG_VDDRST_MASK      0x18U
#define SBC_INITREG_VDDRST_SHIFT     3U
#define SBC_INITREG_VDDRST(x)        (((uint8_t)(((uint8_t)(x))<<SBC_INITREG_VDDRST_SHIFT))&SBC_INITREG_VDDRST_MASK)
#define SBC_INITREG_VDDLRST_MASK     0x60U
#define SBC_INITREG_VDDLRST_SHIFT    5U
#define SBC_INITREG_VDDLRST(x)       (((uint8_t)(((uint8_t)(x))<<SBC_INITREG_VDDLRST_SHIFT))&SBC_INITREG_VDDLRST_MASK)
#define SBC_INITREG_IOSYNC_MASK      0x80U
#define SBC_INITREG_IOSYNC_SHIFT     7U
#define SBC_INITREG_IOSYNC(x)        (((uint8_t)(((uint8_t)(x))<<SBC_INITREG_IOSYNC_SHIFT))&SBC_INITREG_IOSYNC_MASK)

/* Initialization Watchdog Register. */
#define SBC_INITWD_ADDR              0x6U
#define SBC_INITWD_WDCRANK_MASK      0x1U
#define SBC_INITWD_WDCRANK_SHIFT     0U
#define SBC_INITWD_WDCRANK(x)        (((uint8_t)(((uint8_t)(x))<<SBC_INITWD_WDCRANK_SHIFT))&SBC_INITWD_WDCRANK_MASK)
#define SBC_INITWD_WDNWIN_MASK       0x2U
#define SBC_INITWD_WDNWIN_SHIFT      1U
#define SBC_INITWD_WDNWIN(x)         (((uint8_t)(((uint8_t)(x))<<SBC_INITWD_WDNWIN_SHIFT))&SBC_INITWD_WDNWIN_MASK)
#define SBC_INITWD_WDSPI_MASK        0xCU
#define SBC_INITWD_WDSPI_SHIFT       2U
#define SBC_INITWD_WDSPI(x)          (((uint8_t)(((uint8_t)(x))<<SBC_INITWD_WDSPI_SHIFT))&SBC_INITWD_WDSPI_MASK)
#define SBC_INITWD_WDSAFE_MASK       0x10U
#define SBC_INITWD_WDSAFE_SHIFT      4U
#define SBC_INITWD_WDSAFE(x)         (((uint8_t)(((uint8_t)(x))<<SBC_INITWD_WDSAFE_SHIFT))&SBC_INITWD_WDSAFE_MASK)
#define SBC_INITWD_WDOC_MASK         0x60U
#define SBC_INITWD_WDOC_SHIFT        5U
#define SBC_INITWD_WDOC(x)           (((uint8_t)(((uint8_t)(x))<<SBC_INITWD_WDOC_SHIFT))&SBC_INITWD_WDOC_MASK)
#define SBC_INITWD_WD2INT_MASK       0x80U
#define SBC_INITWD_WD2INT_SHIFT      7U
#define SBC_INITWD_WD2INT(x)         (((uint8_t)(((uint8_t)(x))<<SBC_INITWD_WD2INT_SHIFT))&SBC_INITWD_WD2INT_MASK)

/* Initialization LIN I/O Register. */
#define SBC_INITLINIO_ADDR           0x7U
#define SBC_INITLINIO_CYCINV_MASK    0x1U
#define SBC_INITLINIO_CYCINV_SHIFT   0U
#define SBC_INITLINIO_CYCINV(x)      (((uint8_t)(((uint8_t)(x))<<SBC_INITLINIO_CYCINV_SHIFT))&SBC_INITLINIO_CYCINV_MASK)
#define SBC_INITLINIO_IO0_MASK       0x2U
#define SBC_INITLINIO_IO0_SHIFT      1U
#define SBC_INITLINIO_IO0(x)         (((uint8_t)(((uint8_t)(x))<<SBC_INITLINIO_IO0_SHIFT))&SBC_INITLINIO_IO0_MASK)
#define SBC_INITLINIO_IO1_MASK       0x4U
#define SBC_INITLINIO_IO1_SHIFT      2U
#define SBC_INITLINIO_IO1(x)         (((uint8_t)(((uint8_t)(x))<<SBC_INITLINIO_IO1_SHIFT))&SBC_INITLINIO_IO1_MASK)
#define SBC_INITLINIO_LINT1_MASK     0x18U
#define SBC_INITLINIO_LINT1_SHIFT    3U
#define SBC_INITLINIO_LINT1(x)       (((uint8_t)(((uint8_t)(x))<<SBC_INITLINIO_LINT1_SHIFT))&SBC_INITLINIO_LINT1_MASK)
#define SBC_INITLINIO_LINT2_MASK     0x60U
#define SBC_INITLINIO_LINT2_SHIFT    5U
#define SBC_INITLINIO_LINT2(x)       (((uint8_t)(((uint8_t)(x))<<SBC_INITLINIO_LINT2_SHIFT))&SBC_INITLINIO_LINT2_MASK)
#define SBC_INITLINIO_IO1OVF_MASK    0x80U
#define SBC_INITLINIO_IO1OVF_SHIFT   7U
#define SBC_INITLINIO_IO1OVF(x)      (((uint8_t)(((uint8_t)(x))<<SBC_INITLINIO_IO1OVF_SHIFT))&SBC_INITLINIO_IO1OVF_MASK)

/* Initialization Miscellaneous Register. */
#define SBC_INITMISC_ADDR            0x8U
#define SBC_INITMISC_DBGRES_MASK     0x7U
#define SBC_INITMISC_DBGRES_SHIFT    0U
#define SBC_INITMISC_DBG(x)          (((uint8_t)(((uint8_t)(x))<<SBC_INITMISC_DBGRES_SHIFT))&SBC_INITMISC_DBGRES_MASK)
#define SBC_INITMISC_INTFLASH_MASK   0x8U
#define SBC_INITMISC_INTFLASH_SHIFT  3U
#define SBC_INITMISC_INTFLASH(x)     (((uint8_t)(((uint8_t)(x))<<SBC_INITMISC_INTFLASH_SHIFT))&SBC_INITMISC_INTFLASH_MASK)
#define SBC_INITMISC_INTWIDTH_MASK   0x10U
#define SBC_INITMISC_INTWIDTH_SHIFT  4U
#define SBC_INITMISC_INTWIDTH(x)     (((uint8_t)(((uint8_t)(x))<<SBC_INITMISC_INTWIDTH_SHIFT))&SBC_INITMISC_INTWIDTH_MASK)
#define SBC_INITMISC_INTPULSE_MASK   0x20U
#define SBC_INITMISC_INTPULSE_SHIFT  5U
#define SBC_INITMISC_INTPULSE(x)     (((uint8_t)(((uint8_t)(x))<<SBC_INITMISC_INTPULSE_SHIFT))&SBC_INITMISC_INTPULSE_MASK)
#define SBC_INITMISC_SPIPAR_MASK     0x40U
#define SBC_INITMISC_SPIPAR_SHIFT    6U
#define SBC_INITMISC_SPIPAR(x)       (((uint8_t)(((uint8_t)(x))<<SBC_INITMISC_SPIPAR_SHIFT))&SBC_INITMISC_SPIPAR_MASK)
#define SBC_INITMISC_LPMRNDM_MASK    0x80U
#define SBC_INITMISC_LPMRNDM_SHIFT   7U
#define SBC_INITMISC_LPMRNDM(x)      (((uint8_t)(((uint8_t)(x))<<SBC_INITMISC_LPMRNDM_SHIFT))&SBC_INITMISC_LPMRNDM_MASK)

/* Specific Mode Register. */          
#define SBC_SPEMODE_ADDR             0x9U
#define SBC_SPEMODE_RND_MASK         0x3FU
#define SBC_SPEMODE_RND_SHIFT        0U
#define SBC_SPEMODE_RND(x)           (((uint8_t)(((uint8_t)(~x))<<SBC_SPEMODE_RND_SHIFT))&SBC_SPEMODE_RND_MASK)
#define SBC_SPEMODE_SELMODE_MASK     0xC0U
#define SBC_SPEMODE_SELMODE_SHIFT    6U
#define SBC_SPEMODE_SELMODE(x)       (((uint8_t)(((uint8_t)(x))<<SBC_SPEMODE_SELMODE_SHIFT))&SBC_SPEMODE_SELMODE_MASK)
/* Read random code command. */
#define SBC_READ_RAND_CODE_MODE      0x1300U

/* Timer Register TIM_A. */
#define SBC_TIMA_ADDR                0xAU
#define SBC_TIMA_WDNOR_MASK          0x7U /* First three bits. */
#define SBC_TIMA_WDNOR_SHIFT         0U
#define SBC_TIMA_WDNOR(x)            (((uint8_t)(((uint8_t)(x))<<SBC_TIMA_WDNOR_SHIFT))&SBC_TIMA_WDNOR_MASK)
#define SBC_TIMA_WDNOR_SUB(x)        SBC_TIMA_WDNOR(((uint8_t)(x))>>2)
#define SBC_TIMA_WDN_MASK            0x18u /* 4th and 5th bit. */
#define SBC_TIMA_WDN_SHIFT           3U
#define SBC_TIMA_WDN(x)              (((uint8_t)(((uint8_t)(x))<<SBC_TIMA_WDN_SHIFT))&SBC_TIMA_WDN_MASK)
#define SBC_TIMA_WD_SET_PERIOD(x)    (SBC_TIMA_WDN(x)|SBC_TIMA_WDNOR_SUB(x))/* Select Watchdog Period, Combination of SBC_TIMA_WDNOR and SBC_TIMA_WDN bits. 0b 000 00 ~ 2.5 ms, 0b 000 01 ~ 3 ms, 0b 000 10 ~ 3.5 ms, ... , 0b 111 11 ~ 512 ms . */
#define SBC_TIMA_MCU1_MASK           0x60
#define SBC_TIMA_MCU1_SHIFT          5U
#define SBC_TIMA_MCU1(x)             (((uint8_t)(((uint8_t)(x))<<SBC_TIMA_MCU1_SHIFT))&SBC_TIMA_MCU1_MASK)
#define SBC_TIMA_MCU1_SUB(x)         SBC_TIMA_MCU1(((uint8_t)(x))>>1)
#define SBC_TIMA_MCU2_MASK           0x80U
#define SBC_TIMA_MCU2_SHIFT          7U
#define SBC_TIMA_MCU2(x)            (((uint8_t)(((uint8_t)(x))<<SBC_TIMA_MCU2_SHIFT))&SBC_TIMA_MCU2_MASK)
#define SBC_TIMA_MCU_SET_OC(x)      (SBC_TIMA_MCU2(x)|SBC_TIMA_MCU1_SUB(x)) /* Select Overcurrent, 0b 00 0 ~ 3 ms, 0b 00 1 ~ 4 ms, 0b 01 0 ~ 6 ms, ... , 0b 11 1 ~ 32 ms. */

/* Timer Register TIM_B. */
#define SBC_TIMB_ADDR               0xBU
#define SBC_TIMB_CYCINT02_MASK      0x7U
#define SBC_TIMB_CYCINT02_SHIFT     0U
#define SBC_TIMB_CYCINT02(x)        (((uint8_t)(((uint8_t)(x))<<SBC_TIMB_CYCINT02_SHIFT))&SBC_TIMB_CYCINT02_MASK)
#define SBC_TIMB_CYCINT02_SUB(x)    SBC_TIMB_CYCINT02(((uint8_t)(x))>>1)
#define SBC_TIMB_CYCINT3_MASK       0x8U
#define SBC_TIMB_CYCINT3_SHIFT      3U
#define SBC_TIMB_CYCINT3(x)         (((uint8_t)(((uint8_t)(x))<<SBC_TIMB_CYCINT3_SHIFT))&SBC_TIMB_CYCINT3_MASK)
#define SBC_TIMB_CYCINT_SET_INT(x)  (SBC_TIMB_CYCINT02_SUB(x)|SBC_TIMB_CYCINT3(x)) /* Set Cyclic Interrupt, 0b 000 0 ~ 6 ms, 0b 000 1 ~ 8 ms, 0b 001 0 ~ 12 ms, ... , 0b 111 1 ~ 1024 ms. */
#define SBC_TIMB_CYCSEN02_MASK      0x70U
#define SBC_TIMB_CYCSEN02_SHIFT     4U
#define SBC_TIMB_CYCSEN02(x)        (((uint8_t)(((uint8_t)(x))<<SBC_TIMB_CYCSEN02_SHIFT))&SBC_TIMB_CYCSEN02_MASK)
#define SBC_TIMB_CYCSEN02_SUB(x)    SBC_TIMB_CYCSEN02(((uint8_t)(x))>>1)
#define SBC_TIMB_CYCSEN3_MASK       0x80U
#define SBC_TIMB_CYCSEN3_SHIFT      7U
#define SBC_TIMB_CYCSEN3(x)         (((uint8_t)(((uint8_t)(x))<<SBC_TIMB_CYCSEN3_SHIFT))&SBC_TIMB_CYCSEN3_MASK)
#define SBC_TIMB_CYCSEN_SET_SENCE(x)(SBC_TIMB_CYCSEN02_SUB(x)|SBC_TIMB_CYCSEN3(x)) /* Set Cyclic Sense, 0b 000 0 ~ 3 ms, 0b 000 1 ~ 4 ms, 0b 001 0 ~ 6 ms, ... , 0b 111 1 ~ 512 ms. */

/* Timer Register TIM_C. */
#define SBC_TIMC_ADDR               0xCU
#define SBC_TIMC_FWU02_MASK         0x7U
#define SBC_TIMC_FWU02_SHIFT        0U
#define SBC_TIMC_FWU02(x)           (((uint8_t)(((uint8_t)(x))<<SBC_TIMC_FWU02_SHIFT))&SBC_TIMC_FWU02_MASK)
#define SBC_TIMC_FWU02_SUB(x)       SBC_TIMC_FWU02(((uint8_t)(x))>>1)
#define SBC_TIMC_FWU3_MASK          0x8U
#define SBC_TIMC_FWU3_SHIFT         3U
#define SBC_TIMC_FWU3(x)            (((uint8_t)(((uint8_t)(x))<<SBC_TIMC_FWU3_SHIFT))&SBC_TIMC_FWU3_MASK)
#define SBC_TIMC_FWU_SET_WAKEUP(x)  (SBC_TIMC_FWU02_SUB(x)|SBC_TIMC_FWU3(x)) /* Select Forced Wake-up, 0b 000 0 ~ 48 ms, 0b 000 1 ~ 64 ms, ... , 0b 111 1 ~ 8192 ms. */
#define SBC_TIMC_WDLPF02_MASK       0x70U
#define SBC_TIMC_WDLPF02_SHIFT      4U
#define SBC_TIMC_WDLPF02(x)         (((uint8_t)(((uint8_t)(x))<<SBC_TIMC_WDLPF02_SHIFT))&SBC_TIMC_WDLPF02_MASK)
#define SBC_TIMC_WDLPF02_SUB(x)     SBC_TIMC_WDLPF02(((uint8_t)x)>>1)
#define SBC_TIMC_WDLPF3_MASK        0x80U
#define SBC_TIMC_WDLPF3_SHIFT       7U
#define SBC_TIMC_WDLPF3(x)          (((uint8_t)(((uint8_t)(x))<<SBC_TIMC_WDLPF3_SHIFT))&SBC_TIMC_WDLPF3_MASK)
#define SBC_TIMC_WDLPF_SET_WATCHDOG(x) (SBC_TIMC_WDLPF02_SUB(x)| SBC_TIMC_WDLPF3(x)) /* Values of selected Watchdog depend on mode selected (either LP VDD ON Mode or Flash Mode). */

/* Watchdog Refresh Register. */
#define SBC_WATCHDOG_ADDR           0xDU

/* Mode Register. */
#define SBC_MODE_ADDR               0xEU
#define SBC_MODE_RND_MASK           0x7U
#define SBC_MODE_RND_SHIFT          0U
#define SBC_MODE_RND(x)             (((uint8_t)(((uint8_t)(~x))<<SBC_MODE_RND_SHIFT))&SBC_MODE_RND_MASK)
#define SBC_MODE_MOD_MASK           0xF8U
#define SBC_MODE_MOD_SHIFT          3U
#define SBC_MODE_MOD(x)             (((uint8_t)(((uint8_t)(x))<<SBC_MODE_MOD_SHIFT))&SBC_MODE_MOD_MASK)

/* Clear wake up flags event. */
#define SBC_CLEAR_CAN_WAKEUP_FLAG   0xE100U
#define SBC_CLEAR_CAN_FAILURE_FLAG  0xE180U
#define SBC_CLEAR_IO_WAKEUP_FLAG    0xE380U
#define SBC_CLEAR_LIN1_WAKEUP_FLAG  0xE700U
#define SBC_CLEAR_LIN2_WAKEUP_FLAG  0xE900U
#define SBC_READ_RANDOM_CODE_LP     0x1D00U

/* Regulator Register. */
#define SBC_REG_ADDR                0xFU
/* Write values. */
#define SBC_REG_VDDOFFEN_MASK       0x1U
#define SBC_REG_VDDOFFEN_SHIFT      0U
#define SBC_REG_VDDOFFEN(x)         (((uint8_t)(((uint8_t)(x))<<SBC_REG_VDDOFFEN_SHIFT))&SBC_REG_VDDOFFEN_MASK)
#define SBC_REG_VDDBALAUTO_MASK     0x2U
#define SBC_REG_VDDBALAUTO_SHIFT    1U
#define SBC_REG_VDDBALAUTO(x)       (((uint8_t)(((uint8_t)(x))<<SBC_REG_VDDBALAUTO_SHIFT))&SBC_REG_VDDBALAUTO_MASK)
#define SBC_REG_VDDBALEN_MASK       0x4U
#define SBC_REG_VDDBALEN_SHIFT      2U
#define SBC_REG_VDDBALEN(x)         (((uint8_t)(((uint8_t)(x))<<SBC_REG_VDDBALEN_SHIFT))&SBC_REG_VDDBALEN_MASK)
#define SBC_REG_CAN_MASK            0x18U
#define SBC_REG_CAN_SHIFT           3U
#define SBC_REG_CAN(x)              (((uint8_t)(((uint8_t)(x))<<SBC_REG_CAN_SHIFT))&SBC_REG_CAN_MASK)
#define SBC_REG_VAUX_MASK           0xC0U
#define SBC_REG_VAUX_SHIFT          6U
#define SBC_REG_VAUX(x)             (((uint8_t)(((uint8_t)(x))<<SBC_REG_VAUX_SHIFT))&SBC_REG_VAUX_MASK)
/* Read Values. Bits[15,14] = [1,1], bit[7] = 1. */
#define SBC_REG_I_OC_NORMAL_MASK    0x1U
#define SBC_REG_I_OC_NORMAL_SHIFT   0U
#define SBC_REG_VSUP_UV_MASK        0x2U
#define SBC_REG_VSUP_UV_SHIFT       1U
#define SBC_REG_VSENSE_LOW_MASK     0x4U
#define SBC_REG_VSENSE_LOW_SHIFT    2U
#define SBC_REG_5VCAN_OC_MASK       0x8U
#define SBC_REG_5VCAN_OC_SHIFT      3U
#define SBC_REG_5VCAN_UV_MASK       0x10U
#define SBC_REG_5VCAN_UV_SHIFT      4U
#define SBC_REG_5VCAN_TH_SHTD_MASK  0x20U
#define SBC_REG_5VCAN_TH_SHTD_SHIFT 5U
#define SBC_REG_VAUX_OC_MASK        0x40U
#define SBC_REG_VAUX_OC_SHIFT       6U
#define SBC_REG_VAUX_LOW_MASK       0x80U
#define SBC_REG_VAUX_LOW_SHIFT      7U
/* Read Values. Bits[15,14] = [1,1], bit[7] = 0. */
#define SBC_REG_OC_LPVDDON_MASK     0x1U
#define SBC_REG_OC_LPVDDON_SHIFT    0U
#define SBC_REG_V_BATFAIL_MASK      0x2U
#define SBC_REG_V_BATFAIL_SHIFT     1U
#define SBC_REG_RST_LOW_MASK        0x4U
#define SBC_REG_RST_LOW_SHIFT       2U
#define SBC_REG_V_TH_SHTD_MASK      0x10U
#define SBC_REG_V_TH_SHTD_SHIFT     4U

/* CAN Register. */
#define SBC_CAN_ADDR                0x10U
/* Write values. */
#define SBC_CAN_CANINT_MASK         0x1U
#define SBC_CAN_CANINT_SHIFT        0U
#define SBC_CAN_CANINT(x)           (((uint8_t)(((uint8_t)(x))<<SBC_CAN_CANINT_SHIFT))&SBC_CAN_CANINT_MASK)
#define SBC_CAN_WAKEUP_MASK         0x8U
#define SBC_CAN_WAKEUP_SHIFT        3U
#define SBC_CAN_WAKEUP(x)           (((uint8_t)(((uint8_t)(x))<<SBC_CAN_WAKEUP_SHIFT))&SBC_CAN_WAKEUP_MASK)
#define SBC_CAN_SLEW_MASK           0x30U
#define SBC_CAN_SLEW_SHIFT          4U
#define SBC_CAN_SLEW(x)             (((uint8_t)(((uint8_t)(x))<<SBC_CAN_SLEW_SHIFT))&SBC_CAN_SLEW_MASK)
#define SBC_CAN_CANMOD_MASK         0xC0U
#define SBC_CAN_CANMOD_SHIFT        6U
#define SBC_CAN_CANMOD(x)           (((uint8_t)(((uint8_t)(x))<<SBC_CAN_CANMOD_SHIFT))&SBC_CAN_CANMOD_MASK)
/* Read Values. Bits[15,14] = [1,1], bit[7] = 0. */
#define SBC_CAN_OC_MASK             0x1U
#define SBC_CAN_OC_SHIFT            0U
#define SBC_CAN_DOM_CLAMP_MASK      0x2U
#define SBC_CAN_DOM_CLAMP_SHIFT     1U
#define SBC_CAN_TXD_DOM_MASK        0x4U
#define SBC_CAN_TXD_DOM_SHIFT       2U
#define SBC_CAN_RXD_HIGH_MASK       0x8U
#define SBC_CAN_RXD_HIGH_SHIFT      3U
#define SBC_CAN_RXD_LOW_MASK        0x10U
#define SBC_CAN_RXD_LOW_SHIFT       4U
#define SBC_CAN_OVERTEMP_MASK       0x20U
#define SBC_CAN_OVERTEMP_SHIFT      5U
#define SBC_CAN_WAKEUP_R_MASK       0x80U
#define SBC_CAN_WAKEUP_R_SHIFT      7U
/* Read Values. Bits[15,14] = [1,1], bit[7] = 1. */
#define SBC_CAN_CANH2GND_MASK       0x1U
#define SBC_CAN_CANH2GND_SHIFT      0U
#define SBC_CAN_CANH2VDD_MASK       0x2U
#define SBC_CAN_CANH2VDD_SHIFT      1U
#define SBC_CAN_CANH2VBAT_MASK      0x4U
#define SBC_CAN_CANH2VBAT_SHIFT     2U
#define SBC_CAN_CANL2GND_MASK       0x8U
#define SBC_CAN_CANL2GND_SHIFT      3U
#define SBC_CAN_CANL2VDD_MASK       0x10U
#define SBC_CAN_CANL2VDD_SHIFT      4U
#define SBC_CAN_CANL2VBAT_MASK      0x20U
#define SBC_CAN_CANL2VBAT_SHIFT     5U
#define SBC_CAN_F_MASK              0x40U
#define SBC_CAN_F_SHIFT             6U
#define SBC_CAN_UF_MASK             0x80U
#define SBC_CAN_UF_SHIFT            7U
/* Read Values. Bits[15,14] = [0,0], bit[7] = 1. */
#define SBC_CAN_WU_MASK             0x20U
#define SBC_CAN_WU_SHIFT            5U
#define SBC_CAN_RCV_STATE_MASK      0x40U
#define SBC_CAN_RCV_STATE_SHIFT     6U
#define SBC_CAN_DRV_STATE_MASK      0x80U
#define SBC_CAN_DRV_STATE_SHIFT     7U

/* I/O Register. */
#define SBC_IO_ADDR                 0x11U
/* Write values. */
#define SBC_IO_IO0_MASK             0x3U
#define SBC_IO_IO0_SHIFT            0U
#define SBC_IO_IO0(x)               (((uint8_t)(((uint8_t)(x))<<SBC_IO_IO0_SHIFT))&SBC_IO_IO0_MASK)
#define SBC_IO_IO1_MASK             0xCU
#define SBC_IO_IO1_SHIFT            2U
#define SBC_IO_IO1(x)               (((uint8_t)(((uint8_t)(x))<<SBC_IO_IO1_SHIFT))&SBC_IO_IO1_MASK)
#define SBC_IO_IO2_MASK             0x30U
#define SBC_IO_IO2_SHIFT            4U
#define SBC_IO_IO2(x)               (((uint8_t)(((uint8_t)(x))<<SBC_IO_IO2_SHIFT))&SBC_IO_IO2_MASK)
#define SBC_IO_IO3_MASK             0xC0U
#define SBC_IO_IO3_SHIFT            6U
#define SBC_IO_IO3(x)               (((uint8_t)(((uint8_t)(x))<<SBC_IO_IO3_SHIFT))&SBC_IO_IO3_MASK)
/* Read Values. Bits[15,14] = [1,1], bit[7] = 0. */
#define SBC_IO_WD_FLASH_MASK        0x1U
#define SBC_IO_WD_FLASH_SHIFT       0U
#define SBC_IO_THERMAL_MASK         0x2U
#define SBC_IO_THERMAL_SHIFT        1U
#define SBC_IO_VSUP_OV_MASK         0x4U
#define SBC_IO_VSUP_OV_SHIFT        2U
#define SBC_IO_VSUP_UV_MASK         0x8U
#define SBC_IO_VSUP_UV_SHIFT        3U
#define SBC_IO_CSB_LOW_MASK         0x10U
#define SBC_IO_CSB_LOW_SHIFT        4U
#define SBC_IO_SPI_PAR_ERR_MASK     0x20U
#define SBC_IO_SPI_PAR_ERR_SHIFT    5U
#define SBC_IO_HS2_SHORT2GND_MASK   0x40U
#define SBC_IO_HS2_SHORT2GND_SHIFT  6U
#define SBC_IO_HS3_SHORT2GND_MASK   0x80U
#define SBC_IO_HS3_SHORT2GND_SHIFT  7U
/* Read Values. Bits[15,14] = [1,1], bit[7] = 1. */
#define SBC_IO_HW_LEAVE_DBG_MASK    0x1U
#define SBC_IO_HW_LEAVE_DBG_SHIFT   0U
#define SBC_IO_SPI_RESET_MASK       0x2U
#define SBC_IO_SPI_RESET_SHIFT      1U
#define SBC_IO_LP_OFF_WAKEUP_MASK   0x4U
#define SBC_IO_LP_OFF_WAKEUP_SHIFT  2U
#define SBC_IO_INT_TIMEOUT_MASK     0x8U
#define SBC_IO_INT_TIMEOUT_SHIFT    3U
#define SBC_IO_FORCED_WAKEUP_MASK   0x10U
#define SBC_IO_FORCED_WAKEUP_SHIFT  4U
#define SBC_IO_SPI_WAKEUP_MASK      0x20U
#define SBC_IO_SPI_WAKEUP_SHIFT     5U
#define SBC_IO_IO02_WAKEUP_MASK     0x40U
#define SBC_IO_IO02_WAKEUP_SHIFT    6U
#define SBC_IO_IO13_WAKEUP_MASK     0x80U
#define SBC_IO_IO13_WAKEUP_SHIFT    7U
/* Read Values. Bits[15,14] = [0,0], bit[7] = 0. */
#define SBC_IO_IO0_STATE_MASK       0x1U
#define SBC_IO_IO0_STATE_SHIFT      0U
#define SBC_IO_IO1_STATE_MASK       0x4U
#define SBC_IO_IO1_STATE_SHIFT      2U
#define SBC_IO_IO2_STATE_MASK       0x10U
#define SBC_IO_IO2_STATE_SHIFT      4U
#define SBC_IO_IO3_STATE_MASK       0x40U
#define SBC_IO_IO3_STATE_SHIFT      6U

/* INT Register. */
#define SBC_INT_ADDR                0x12U
/* Write values. */
#define SBC_INT_VMON_MASK           0x1U
#define SBC_INT_VMON_SHIFT          0U
#define SBC_INT_VMON(x)             (((uint8_t)(((uint8_t)(x))<<SBC_INT_VMON_SHIFT))&SBC_INT_VMON_MASK)
#define SBC_INT_SAFE_MASK           0x4U
#define SBC_INT_SAFE_SHIFT          2U
#define SBC_INT_SAFE(x)             (((uint8_t)(((uint8_t)(x))<<SBC_INT_SAFE_SHIFT))&SBC_INT_SAFE_MASK)
#define SBC_INT_IO_MASK             0x8U
#define SBC_INT_IO_SHIFT            3U
#define SBC_INT_IO(x)               (((uint8_t)(((uint8_t)(x))<<SBC_INT_IO_SHIFT))&SBC_INT_IO_MASK)
#define SBC_INT_LIN1_MASK           0x10U
#define SBC_INT_LIN1_SHIFT          4U
#define SBC_INT_LIN1(x)             (((uint8_t)(((uint8_t)(x))<<SBC_INT_LIN1_SHIFT))&SBC_INT_LIN1_MASK)
#define SBC_INT_LIN2_MASK           0x20U
#define SBC_INT_LIN2_SHIFT          5U
#define SBC_INT_LIN2(x)             (((uint8_t)(((uint8_t)(x))<<SBC_INT_LIN2_SHIFT))&SBC_INT_LIN2_MASK)
#define SBC_INT_MCU_MASK            0x40U
#define SBC_INT_MCU_SHIFT           6U
#define SBC_INT_MCU(x)              (((uint8_t)(((uint8_t)(x))<<SBC_INT_MCU_SHIFT))&SBC_INT_MCU_MASK)
#define SBC_INT_CAN_MASK            0x80U
#define SBC_INT_CAN_SHIFT           7U
#define SBC_INT_CAN(x)              (((uint8_t)(((uint8_t)(x))<<SBC_INT_CAN_SHIFT))&SBC_INT_CAN_MASK)
/* Read Values. Bits[15,14] = [1,1], bit[7] = 0. */
#define SBC_SAFE_VAUX_OV_MASK       0x2U
#define SBC_SAFE_VAUX_OV_SHIFT      1U
#define SBC_SAFE_VDD_OVERVOL_MASK   0x4U
#define SBC_SAFE_VDD_OVERVOL_SHIFT  2U
#define SBC_SAFE_VDD_UV_MASK        0x8U
#define SBC_SAFE_VDD_UV_SHIFT       3U
#define SBC_SAFE_VDD_TEMP_MASK      0x10U
#define SBC_SAFE_VDD_TEMP_SHIFT     4U
#define SBC_SAFE_DBG_RES_MASK       0x20U
#define SBC_SAFE_DBG_RES_SHIFT      5U
#define SBC_SAFE_RST_MASK           0x40U
#define SBC_SAFE_RST_SHIFT          6U
#define SBC_SAFE_VDD_INT_REQ_MASK   0x80U
#define SBC_SAFE_VDD_INT_REQ_SHIFT  7U
/* Read Values. Bits[15,14] = [1,1], bit[7] = 1. */
#define SBC_SAFE_WATCH_REF_MASK     0x1U
#define SBC_SAFE_WATCH_REF_SHIFT    0U
#define SBC_SAFE_MULTI_REF_MASK     0x2U
#define SBC_SAFE_MULTI_REF_SHIFT    1U
#define SBC_SAFE_RST_LOW100_MASK    0x4U
#define SBC_SAFE_RST_LOW100_SHIFT   2U
#define SBC_SAFE_RST_LOW_MASK       0x8U
#define SBC_SAFE_RST_LOW_SHIFT      3U
#define SBC_SAFE_VDD_LOW_MASK       0x10U
#define SBC_SAFE_VDD_LOW_SHIFT      4U
/* Read Values. Bits[15,14] = [0,0], bit[7] = 1. */
#define SBC_SAFE_ID_MASK            0x1FU
#define SBC_SAFE_ID_SHIFT           0U
#define SBC_SAFE_DEVICE_PN0_MASK    0x20U
#define SBC_SAFE_DEVICE_PN0_SHIFT   5U
#define SBC_SAFE_DEVICE_PN1_MASK    0x40U
#define SBC_SAFE_DEVICE_PN1_SHIFT   6U
#define SBC_SAFE_DEVICE_PN01_MASK   0x60U
#define SBC_SAFE_DEVICE_PN01_SHIFT  5U
#define SBC_SAFE_VDD_MASK           0x80U
#define SBC_SAFE_VDD_SHIFT          7U

#define SBC_NUM_LINS                2U
/* LIN1 Register. */
#define SBC_LIN1_ADDR               0x13U
/* LIN2 Register. */
#define SBC_LIN2_ADDR               0x14U
/* Write values. */
#define SBC_LIN_VSUPEX_MASK         0x1U
#define SBC_LIN_VSUPEX_SHIFT        0U
#define SBC_LIN_VSUPEX(x)           (((uint8_t)(((uint8_t)(x))<<SBC_LIN_VSUPEX_SHIFT))&SBC_LIN_VSUPEX_MASK)
#define SBC_LIN_LINT1ON_MASK        0x4U
#define SBC_LIN_LINT1ON_SHIFT       2U
#define SBC_LIN_LINT1ON(x)          (((uint8_t)(((uint8_t)(x))<<SBC_LIN_LINT1ON_SHIFT))&SBC_LIN_LINT1ON_MASK)
#define SBC_LIN_SR_MASK             0x30U
#define SBC_LIN_SR_SHIFT            4U
#define SBC_LIN_SR(x)               (((uint8_t)(((uint8_t)(x))<<SBC_LIN_SR_SHIFT))&SBC_LIN_SR_MASK)
#define SBC_LIN_LINMOD_MASK         0xC0U
#define SBC_LIN_LINMOD_SHIFT        6U
#define SBC_LIN_LINMOD(x)           (((uint8_t)(((uint8_t)(x))<<SBC_LIN_LINMOD_SHIFT))&SBC_LIN_LINMOD_MASK)
/* Write values. */
#define SBC_LIN_CHANNEL_ADDR(x)     (((x) == 1) ? (SBC_LIN1_ADDR) : (SBC_LIN2_ADDR))

/* Read Values. Bits[15,14] = [1,1], bit[7] = 0. */
#define SBC_LIN_DOM_CLAMP_MASK      0x1U
#define SBC_LIN_DOM_CLAMP_SHIFT     0U
#define SBC_LIN_TXD_DOM_MASK        0x2U
#define SBC_LIN_TXD_DOM_SHIFT       1U
#define SBC_LIN_RXD_HIGH_MASK       0x4U
#define SBC_LIN_RXD_HIGH_SHIFT      2U
#define SBC_LIN_RXD_LOW_MASK        0x8U
#define SBC_LIN_RXD_LOW_SHIFT       3U
#define SBC_LIN_OVERTEMP_MASK       0x10U
#define SBC_LIN_OVERTEMP_SHIFT      4U
#define SBC_LIN_SHORT2GND_MASK      0x20U
#define SBC_LIN_SHORT2GND_SHIFT     5U
#define SBC_LIN_WAKEUP_MASK         0x40u
#define SBC_LIN_WAKEUP_SHIFT        6U
/* Read Values. Bits[15,14] = [1,1], bit[7] = 1. */
#define SBC_LIN_WAKEUP_STATE_MASK   0x40U
#define SBC_LIN_WAKEUP_STATE_SHIFT  6U
#define SBC_LIN_STATE_MASK          0x80U
#define SBC_LIN_STATE_SHIFT         7U

/* Fix and extended status bits. */
#define SBC_STATUS_VREG_0_MASK      0x1U
#define SBC_STATUS_VREG_0_SHIFT     0U
#define SBC_STATUS_VREG_1_MASK      0x2U
#define SBC_STATUS_VREG_1_SHIFT     1U
#define SBC_STATUS_IO_0_MASK        0x4U
#define SBC_STATUS_IO_0_SHIFT       2U
#define SBC_STATUS_IO_1_MASK        0x8U
#define SBC_STATUS_IO_1_SHIFT       3U
#define SBC_STATUS_LIN1_MASK        0x10U
#define SBC_STATUS_LIN1_SHIFT       4U
#define SBC_STATUS_LIN2_MASK        0x20U
#define SBC_STATUS_LIN2_SHIFT       5U
#define SBC_STATUS_CAN_LOC_MASK     0x40U
#define SBC_STATUS_CAN_LOC_SHIFT    6U
#define SBC_STATUS_CAN_BUS_MASK     0x80U
#define SBC_STATUS_CAN_BUS_SHIFT    7U
#define SBC_STATUS_VREG_G_MASK      0x100U
#define SBC_STATUS_VREG_G_SHIFT     8U
#define SBC_STATUS_SAFE_G_MASK      0x200U
#define SBC_STATUS_SAFE_G_SHIFT     9U
#define SBC_STATUS_IO_G_MASK        0x400U
#define SBC_STATUS_IO_G_SHIFT       10U
#define SBC_STATUS_LIN_G_MASK       0x800U
#define SBC_STATUS_LIN_G_SHIFT      11U
#define SBC_STATUS_CAN_G_MASK       0x1000U
#define SBC_STATUS_CAN_G_SHIFT      12U
#define SBC_STATUS_RST_MASK         0x2000U
#define SBC_STATUS_RST_SHIFT        13U
#define SBC_STATUS_WU_MASK          0x4000U
#define SBC_STATUS_WU_SHIFT         14U
#define SBC_STATUS_INT_MASK         0x8000U
#define SBC_STATUS_INT_SHIFT        15U

/* SPI Macros. */
/* Control Bits (C1, C0) + Parity Bit. */
#define SBC_C0_COMMAND_MASK             0x4000U
#define SBC_C0_COMMAND_SHIFT            14U
#define SBC_C1_COMMAND_MASK             0x8000U
#define SBC_C1_COMMAND_SHIFT            15U
#define SBC_C0_C1_COMMANDS_MASK         0xC000U

#define SBC_WRITE_TO_REGISTER           ((uint16_t)(0x1U<<14U)) /* Write to register address, to control the device operation.
If bit 8 is set to “0”: means parity not selected OR parity is selected AND parity = 0.
if bit 8 is set to “1”: means parity is selected AND parity = 1. */

#define SBC_READ_FROM_REGISTER          ((uint16_t)((0x3U<<14U)|0x100U)) /* Read of device flags form a register address. */
/* Bit 8 must be set to 1, independently of the parity function selected or not selected. */

#define SBC_READ_BACK_FROM_REGISTER     ((uint16_t)((0x0U<<14U)|0x100U)) /* Read back of register
content and block (CAN, I/O, INT, LINs) real time state. Bit 8 must be set to 1, 
independently of the parity function selected or not selected. */

#define SBC_PARITY_MASK                 0x100U
#define SBC_PARITY_SHIFT                8U
#define SBC_SET_PAR_BIT_FOR_READING     (((uint16_t)SBC_PARITY_MASK)<<SBC_PARITY_SHIFT)
#define SBC_SHIFT_ADDRESS(address)      ((uint16_t)((address << 9) & 0x3E00U))

/* Write Commands. */
#define SBC_GET_DEVICE_MODE_COMMAND1    0x1D00U /* This command reads device's mode, device leaves debug mode and keeps SAFE pin as is. */
#define SBC_GET_DEVICE_MODE_COMMAND2    0x1D80U /* This command reads device's mode and releases SAFE pin (turn off). */
#define SBC_GET_DEVICE_MODE_COMMAND3    0xDD00U /* This command reads device's mode, device leaves debug mode and keeps SAFE pin as is, MISO reports debug and safe state (bits 1, 0). */
#define SBC_GET_DEVICE_MODE_COMMAND4    0xDD80U /* This command reads device's mode, keeps device in debug mode, releases SAFE pin (turn off), MISO reports debug and safe state (bits 1, 0). */

/* Macros for Bit Extraction. */
#define SBC_GET_MODE_STATUS(x)          (((uint8_t)(((uint8_t)(x))>>3U))&0x1FU)
#define SBC_GET_SAFE_STATE_BIT(x)       (((uint8_t)(((uint8_t)(x))>>1U))&0x1U)
#define SBC_GET_DEBUG_STATE_BIT(x)      (((uint8_t)(((uint8_t)(x))>>0U))&0x1U)

/* Device Mode and SAFE Pin State Macros. */
#define SBC_DEBUG_MODE_OFF              0x0U
#define SBC_DEBUG_MODE_ON               0x1U
#define SBC_SAFE_PIN_OFF                0x0U
#define SBC_SAFE_PIN_ON                 0x1U

/* Device Status. */
#define SBC_INIT_MODE                   0x0U
#define SBC_FLASH_MODE                  0x1U
#define SBC_NORMALREQUEST_MODE          0x2U
#define SBC_NORMALMODE_MODE             0x3U

/* SPI Command to Read Random Code. */
#define SBC_READ_RANDOM_CODE_COMMAND    0x1B00U
/* SPI Commands for Watchdog Operation. */
#define SBC_SIMPLE_WATCHDOG_REFRESH_COMMAND          0x5A00U /* Refresh watchdog command in simple watchdog mode. */
#define SBC_ADVANCED_WATCHDOG_REFRESH_COMMAND_1(x)   ((uint16_t)(0x5A00U|((uint8_t)(~x)))) /* Refresh watchdog command in advanced watchdog mode, refresh by 1 SPI command. x is random code obtained from SBC. */
#define SBC_ADVANCED_WATCHDOG_REFRESH_COMMAND_2_1(x) ((uint16_t)(0x5A00U|(((uint8_t)(~x))&0xF0U))) /* Refresh watchdog command in advanced watchdog mode, refresh by 2 SPI commands. x is random code obtained from SBC. */
#define SBC_ADVANCED_WATCHDOG_REFRESH_COMMAND_2_2(x) ((uint16_t)(0x5A00U|(((uint8_t)(~x))&0xFU))) /* Refresh watchdog command in advanced watchdog mode, refresh by 2 SPI commands. x is random code obtained from SBC. */
#define SBC_ADVANCED_WATCHDOG_REFRESH_COMMAND_4_1(x) ((uint16_t)(0x5A00U|(((uint8_t)(~x))&0xC0U))) /* Refresh watchdog command in advanced watchdog mode, refresh by 4 SPI commands. x is random code obtained from SBC from last WD refresh. */
#define SBC_ADVANCED_WATCHDOG_REFRESH_COMMAND_4_2(x) ((uint16_t)(0x5A00U|(((uint8_t)(~x))&0x30U))) /* Refresh watchdog command in advanced watchdog mode, refresh by 4 SPI commands. x is random code obtained from SBC from last WD refresh. */
#define SBC_ADVANCED_WATCHDOG_REFRESH_COMMAND_4_3(x) ((uint16_t)(0x5A00U|(((uint8_t)(~x))&0xCU))) /* Refresh watchdog command in advanced watchdog mode, refresh by 4 SPI commands. x is random code obtained from SBC from last WD refresh. */
#define SBC_ADVANCED_WATCHDOG_REFRESH_COMMAND_4_4(x) ((uint16_t)(0x5A00U|(((uint8_t)(~x))&0x3U))) /* Refresh watchdog command in advanced watchdog mode, refresh by 4 SPI commands. x is random code obtained from SBC from last WD refresh. */

/* Low Power SPI Commands. */
#define SBC_WAKE_UP_LPVDDON     0x5C10U /* After this command, device wakes up from LP Vdd ON mode to "normal request mode". */
#define SBC_WAKE_UP_LPCYCINT    0x5C10U /* Wakes up device from LP with Cyclic INT. */

#endif
