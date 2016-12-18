/****************************************************************************
 (c) Copyright 2010-2015 Freescale Semiconductor, Inc.
 ALL RIGHTS RESERVED.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY DIRECT,
 INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
#ifndef FSL_FLASH_DRIVER_C90TFS_H_
#define FSL_FLASH_DRIVER_C90TFS_H_
#include <stdbool.h>
#include "fsl_device_registers.h"

/* FTFx - Register instance definitions */
/* FTFE */
#ifdef FTFE
#define FTFx_BASE                                FTFE_BASE
#define FTFx_FSTAT                               FTFE->FSTAT
#define FTFx_FCNFG                               FTFE->FCNFG
#define FTFx_FSEC                                FTFE->FSEC
#define FTFx_FOPT                                FTFE->FOPT
#define FTFx_FCCOB3                              FTFE->FCCOB[0]
#define FTFx_FCCOB2                              FTFE->FCCOB[1]
#define FTFx_FCCOB1                              FTFE->FCCOB[2]
#define FTFx_FCCOB0                              FTFE->FCCOB[3]
#define FTFx_FCCOB7                              FTFE->FCCOB[4]
#define FTFx_FCCOB6                              FTFE->FCCOB[5]
#define FTFx_FCCOB5                              FTFE->FCCOB[6]
#define FTFx_FCCOB4                              FTFE->FCCOB[7]
#define FTFx_FCCOBB                              FTFE->FCCOB[8]
#define FTFx_FCCOBA                              FTFE->FCCOB[9]
#define FTFx_FCCOB9                              FTFE->FCCOB[10]
#define FTFx_FCCOB8                              FTFE->FCCOB[11]
#define FTFx_FPROT3                              FTFE->FPROT[0]
#define FTFx_FPROT2                              FTFE->FPROT[1]
#define FTFx_FPROT1                              FTFE->FPROT[2]
#define FTFx_FPROT0                              FTFE->FPROT[3]
#define FTFx_FEPROT                              FTFE->FEPROT
#define FTFx_FDPROT                              FTFE->FDPROT
  #ifdef  FTFE_FERSTAT
  #define FTFx_FERSTAT                             FTFE_FERSTAT
  #endif
  #ifdef  FTFE_FERCNFG
  #define FTFx_FERCNFG                             FTFE_FERCNFG
  #endif

/* FSTAT Bit Fields */
#define FTFx_FSTAT_MGSTAT0_MASK                 FTFE_FSTAT_MGSTAT0_MASK
#define FTFx_FSTAT_MGSTAT0_SHIFT                FTFE_FSTAT_MGSTAT0_SHIFT
#define FTFx_FSTAT_MGSTAT0_WIDTH                FTFE_FSTAT_MGSTAT0_WIDTH
#define FTFx_FSTAT_MGSTAT0(x)                   FTFE_FSTAT_MGSTAT0(x)
#define FTFx_FSTAT_FPVIOL_MASK                  FTFE_FSTAT_FPVIOL_MASK
#define FTFx_FSTAT_FPVIOL_SHIFT                 FTFE_FSTAT_FPVIOL_SHIFT
#define FTFx_FSTAT_FPVIOL_WIDTH                 FTFE_FSTAT_FPVIOL_WIDTH
#define FTFx_FSTAT_FPVIOL(x)                    FTFE_FSTAT_FPVIOL(x)
#define FTFx_FSTAT_ACCERR_MASK                  FTFE_FSTAT_ACCERR_MASK
#define FTFx_FSTAT_ACCERR_SHIFT                 FTFE_FSTAT_ACCERR_SHIFT
#define FTFx_FSTAT_ACCERR_WIDTH                 FTFE_FSTAT_ACCERR_WIDTH
#define FTFx_FSTAT_ACCERR(x)                    FTFE_FSTAT_ACCERR(x)
#define FTFx_FSTAT_RDCOLERR_MASK                FTFE_FSTAT_RDCOLERR_MASK
#define FTFx_FSTAT_RDCOLERR_SHIFT               FTFE_FSTAT_RDCOLERR_SHIFT
#define FTFx_FSTAT_RDCOLERR_WIDTH               FTFE_FSTAT_RDCOLERR_WIDTH
#define FTFx_FSTAT_RDCOLERR(x)                  FTFE_FSTAT_RDCOLERR(x)
#define FTFx_FSTAT_CCIF_MASK                    FTFE_FSTAT_CCIF_MASK
#define FTFx_FSTAT_CCIF_SHIFT                   FTFE_FSTAT_CCIF_SHIFT
#define FTFx_FSTAT_CCIF_WIDTH                   FTFE_FSTAT_CCIF_WIDTH
#define FTFx_FSTAT_CCIF(x)                      FTFE_FSTAT_CCIF(x)
/* FCNFG Bit Fields */
#define FTFx_FCNFG_EEERDY_MASK                  FTFE_FCNFG_EEERDY_MASK
#define FTFx_FCNFG_EEERDY_SHIFT                 FTFE_FCNFG_EEERDY_SHIFT
#define FTFx_FCNFG_EEERDY_WIDTH                 FTFE_FCNFG_EEERDY_WIDTH
#define FTFx_FCNFG_EEERDY(x)                    FTFE_FCNFG_EEERDY(x)
#define FTFx_FCNFG_RAMRDY_MASK                  FTFE_FCNFG_RAMRDY_MASK
#define FTFx_FCNFG_RAMRDY_SHIFT                 FTFE_FCNFG_RAMRDY_SHIFT
#define FTFx_FCNFG_RAMRDY_WIDTH                 FTFE_FCNFG_RAMRDY_WIDTH
#define FTFx_FCNFG_RAMRDY(x)                    FTFE_FCNFG_RAMRDY(x)
#define FTFx_FCNFG_PFLSH_MASK                   FTFE_FCNFG_PFLSH_MASK
#define FTFx_FCNFG_PFLSH_SHIFT                  FTFE_FCNFG_PFLSH_SHIFT
#define FTFx_FCNFG_PFLSH_WIDTH                  FTFE_FCNFG_PFLSH_WIDTH
#define FTFx_FCNFG_PFLSH(x)                     FTFE_FCNFG_PFLSH(x)
#define FTFx_FCNFG_ERSSUSP_MASK                 FTFE_FCNFG_ERSSUSP_MASK
#define FTFx_FCNFG_ERSSUSP_SHIFT                FTFE_FCNFG_ERSSUSP_SHIFT
#define FTFx_FCNFG_ERSSUSP_WIDTH                FTFE_FCNFG_ERSSUSP_WIDTH
#define FTFx_FCNFG_ERSSUSP(x)                   FTFE_FCNFG_ERSSUSP(x)
#define FTFx_FCNFG_ERSAREQ_MASK                 FTFE_FCNFG_ERSAREQ_MASK
#define FTFx_FCNFG_ERSAREQ_SHIFT                FTFE_FCNFG_ERSAREQ_SHIFT
#define FTFx_FCNFG_ERSAREQ_WIDTH                FTFE_FCNFG_ERSAREQ_WIDTH
#define FTFx_FCNFG_ERSAREQ(x)                   FTFE_FCNFG_ERSAREQ(x)
#define FTFx_FCNFG_RDCOLLIE_MASK                FTFE_FCNFG_RDCOLLIE_MASK
#define FTFx_FCNFG_RDCOLLIE_SHIFT               FTFE_FCNFG_RDCOLLIE_SHIFT
#define FTFx_FCNFG_RDCOLLIE_WIDTH               FTFE_FCNFG_RDCOLLIE_WIDTH
#define FTFx_FCNFG_RDCOLLIE(x)                  FTFE_FCNFG_RDCOLLIE(x)
#define FTFx_FCNFG_CCIE_MASK                    FTFE_FCNFG_CCIE_MASK
#define FTFx_FCNFG_CCIE_SHIFT                   FTFE_FCNFG_CCIE_SHIFT
#define FTFx_FCNFG_CCIE_WIDTH                   FTFE_FCNFG_CCIE_WIDTH
#define FTFx_FCNFG_CCIE(x)                      FTFE_FCNFG_CCIE(x)
/* FSEC Bit Fields */
#define FTFx_FSEC_SEC_MASK                      FTFE_FSEC_SEC_MASK
#define FTFx_FSEC_SEC_SHIFT                     FTFE_FSEC_SEC_SHIFT
#define FTFx_FSEC_SEC_WIDTH                     FTFE_FSEC_SEC_WIDTH
#define FTFx_FSEC_SEC(x)                        FTFE_FSEC_SEC(x)
#define FTFx_FSEC_FSLACC_MASK                   FTFE_FSEC_FSLACC_MASK
#define FTFx_FSEC_FSLACC_SHIFT                  FTFE_FSEC_FSLACC_SHIFT
#define FTFx_FSEC_FSLACC_WIDTH                  FTFE_FSEC_FSLACC_WIDTH
#define FTFx_FSEC_FSLACC(x)                     FTFE_FSEC_FSLACC(x)
#define FTFx_FSEC_MEEN_MASK                     FTFE_FSEC_MEEN_MASK
#define FTFx_FSEC_MEEN_SHIFT                    FTFE_FSEC_MEEN_SHIFT
#define FTFx_FSEC_MEEN_WIDTH                    FTFE_FSEC_MEEN_WIDTH
#define FTFx_FSEC_MEEN(x)                       FTFE_FSEC_MEEN(x)
#define FTFx_FSEC_KEYEN_MASK                    FTFE_FSEC_KEYEN_MASK
#define FTFx_FSEC_KEYEN_SHIFT                   FTFE_FSEC_KEYEN_SHIFT
#define FTFx_FSEC_KEYEN_WIDTH                   FTFE_FSEC_KEYEN_WIDTH
#define FTFx_FSEC_KEYEN(x)                      FTFE_FSEC_KEYEN(x)
/* FOPT Bit Fields */
#define FTFx_FOPT_OPT_MASK                      FTFE_FOPT_OPT_MASK
#define FTFx_FOPT_OPT_SHIFT                     FTFE_FOPT_OPT_SHIFT
#define FTFx_FOPT_OPT_WIDTH                     FTFE_FOPT_OPT_WIDTH
#define FTFx_FOPT_OPT(x)                        FTFE_FOPT_OPT(x)
/* FCCOB Bit Fields */
#define FTFx_FCCOB_CCOBn_MASK                   FTFE_FCCOB_CCOBn_MASK
#define FTFx_FCCOB_CCOBn_SHIFT                  FTFE_FCCOB_CCOBn_SHIFT
#define FTFx_FCCOB_CCOBn_WIDTH                  FTFE_FCCOB_CCOBn_WIDTH
#define FTFx_FCCOB_CCOBn(x)                     FTFE_FCCOB_CCOBn(x)
/* FPROT Bit Fields */
#define FTFx_FPROT_PROT_MASK                    FTFE_FPROT_PROT_MASK
#define FTFx_FPROT_PROT_SHIFT                   FTFE_FPROT_PROT_SHIFT
#define FTFx_FPROT_PROT_WIDTH                   FTFE_FPROT_PROT_WIDTH
#define FTFx_FPROT_PROT(x)                      FTFE_FPROT_PROT(x)
/* FEPROT Bit Fields */
#define FTFx_FEPROT_EPROT_MASK                  FTFE_FEPROT_EPROT_MASK
#define FTFx_FEPROT_EPROT_SHIFT                 FTFE_FEPROT_EPROT_SHIFT
#define FTFx_FEPROT_EPROT_WIDTH                 FTFE_FEPROT_EPROT_WIDTH
#define FTFx_FEPROT_EPROT(x)                    FTFE_FEPROT_EPROT(x)
/* FDPROT Bit Fields */
#define FTFx_FDPROT_DPROT_MASK                  FTFE_FDPROT_DPROT_MASK
#define FTFx_FDPROT_DPROT_SHIFT                 FTFE_FDPROT_DPROT_SHIFT
#define FTFx_FDPROT_DPROT_WIDTH                 FTFE_FDPROT_DPROT_WIDTH
#define FTFx_FDPROT_DPROT(x)                    FTFE_FDPROT_DPROT(x)
/* FERSTAT Bit Fields */
  #ifdef FTFx_FERSTAT
  #define FTFx_FERSTAT_DFDIF_MASK                 FTFE_FERSTAT_DFDIF_MASK
  #define FTFx_FERSTAT_DFDIF_SHIFT                FTFE_FERSTAT_DFDIF_SHIFT
  #define FTFxFERSTAT_DFDIF_WIDTH                FTFE_FERSTAT_DFDIF_WIDTH
  #define FTFx_FERSTAT_DFDIF(x)                   FTFE_FERSTAT_DFDIF(x)
  #endif
  /* FERCNFG Bit Fields */
  #ifdef FTFx_FERCNFG
  #define FTFx_FERCNFG_DFDIE_MASK                 FTFE_FERCNFG_DFDIE_MASK
  #define FTFx_FERCNFG_DFDIE_SHIFT                FTFE_FERCNFG_DFDIE_SHIFT
  #define FTFx_FERCNFG_DFDIE_WIDTH                FTFE_FERCNFG_DFDIE_WIDTH
  #define FTFx_FERCNFG_DFDIE(x)                   FTFE_FERCNFG_DFDIE(x)
  #define FTFx_FERCNFG_FDFD_MASK                  FTFE_FERCNFG_FDFD_MASK
  #define FTFx_FERCNFG_FDFD_SHIFT                 FTFE_FERCNFG_FDFD_SHIFT
  #define FTFx_FERCNFG_FDFD_WIDTH                 FTFE_FERCNFG_FDFD_WIDTH
  #define FTFx_FERCNFG_FDFD(x)                    FTFE_FERCNFG_FDFD(x)
  #endif
#endif

#define CLEAR_FTFx_FSTAT_ERROR_BITS     FTFx_FSTAT = FTFx_FSTAT_FPVIOL_MASK | FTFx_FSTAT_ACCERR_MASK | FTFx_FSTAT_RDCOLERR_MASK

 /* Word size 2 bytes */
#define FTFx_WORD_SIZE                    0x0002U

/* Longword size 4 bytes */
#define FTFx_LONGWORD_SIZE                0x0004U

/* Phrase size 8 bytes */
#define FTFx_PHRASE_SIZE                  0x0008U

/* Double-phrase size 16 bytes */
#define FTFx_DPHRASE_SIZE                 0x0010U

/* fccob offset address to store resource code */
#if (FSL_FEATURE_FLASH_PFLASH_BLOCK_WRITE_UNIT_SIZE == FTFx_PHRASE_SIZE)
	#define FTFx_RSRC_CODE_REG        FTFx_FCCOB4
#else
	#define FTFx_RSRC_CODE_REG        FTFx_FCCOB8
#endif

/*------------- Flash hardware algorithm operation commands -------------*/
#define FTFx_VERIFY_BLOCK               0x00U
#define FTFx_VERIFY_SECTION             0x01U
#define FTFx_PROGRAM_CHECK              0x02U
#define FTFx_READ_RESOURCE              0x03U
#define FTFx_PROGRAM_LONGWORD           0x06U
#define FTFx_PROGRAM_PHRASE             0x07U
#define FTFx_ERASE_BLOCK                0x08U
#define FTFx_ERASE_SECTOR               0x09U
#define FTFx_PROGRAM_SECTION            0x0BU
#define FTFx_VERIFY_ALL_BLOCK           0x40U
#define FTFx_READ_ONCE                  0x41U
#define FTFx_PROGRAM_ONCE               0x43U
#define FTFx_ERASE_ALL_BLOCK            0x44U
#define FTFx_SECURITY_BY_PASS           0x45U
#define FTFx_PFLASH_SWAP                0x46U
#define FTFx_PROGRAM_PARTITION          0x80U
#define FTFx_SET_EERAM                  0x81U
#define FTFx_ERASE_ALL_BLOCK_UNSECURE   0x49U

/* EERAM Function Control Code */
#define EEE_ENABLE                      0x00U
#define EEE_DISABLE                     0xFFU

/*!
 * @addtogroup c90tfs_flash_driver
 * @{
 */

/*!
 * @name PFlash swap control codes
 * @{
 */
/*! @brief Initialize Swap System control code */
#define FTFx_SWAP_SET_INDICATOR_ADDR    0x01U
/*! @brief Set Swap in Update State */
#define FTFx_SWAP_SET_IN_PREPARE        0x02U
/*! @brief Set Swap in Complete State */
#define FTFx_SWAP_SET_IN_COMPLETE       0x04U
/*! @brief  Report Swap Status */
#define FTFx_SWAP_REPORT_STATUS         0x08U
/*@}*/

/*!
 * @name PFlash swap states
 * @{
 */
/*! @brief  Uninitialized swap mode */
#define FTFx_SWAP_UNINIT                0x00U
/*! @brief  Ready swap mode */
#define FTFx_SWAP_READY                 0x01U
/*! @brief  Update swap mode */
#define FTFx_SWAP_UPDATE                0x02U
/*! @brief  Update-Erased swap mode */
#define FTFx_SWAP_UPDATE_ERASED         0x03U
/*! @brief  Complete swap mode */
#define FTFx_SWAP_COMPLETE              0x04U
/*@}*/

/*! @brief Resume wait count used in FlashResume function */
#define RESUME_WAIT_CNT         0x20U

/*! @}*/ /* End of addtogroup c90tfs_flash_driver */


#if (FSL_FEATURE_FLASH_IS_FTFE == 1)
    #define DFLASH_IFR_READRESOURCE_ADDRESS                0x8003F8U
#else /* FSL_FEATURE_FLASH_IS_FTFL == 1 or FSL_FEATURE_FLASH_IS_FTFA = =1 */
    #define DFLASH_IFR_READRESOURCE_ADDRESS                0x8000FCU
#endif

#define GET_BIT_0_7(value)              ((uint8_t)(((uint32_t)(value)) & 0xFFU))
#define GET_BIT_8_15(value)             ((uint8_t)((((uint32_t)(value))>>8) & 0xFFU))
#define GET_BIT_16_23(value)            ((uint8_t)((((uint32_t)(value))>>16) & 0xFFU))
#define GET_BIT_24_31(value)            ((uint8_t)(((uint32_t)(value))>>24))

/* Flash security status */
#define FLASH_SECURITY_STATE_KEYEN         0x80U
#define FLASH_SECURITY_STATE_UNSECURED     0x02U

/*!
 * @addtogroup c90tfs_flash_driver
 * @{
 */

/*------------ Return Code Definition for FTFx SSD ---------------------*/
/*! @brief Return Code Definition for FTFx SSD */
typedef enum flash_drv_status
{
    FTFx_OK             = 0x0000U, /*!< Function executes successfully */
    FTFx_ERR_MGSTAT0    = 0x0001U, /*!< MGSTAT0 error */
    FTFx_ERR_PVIOL      = 0x0010U, /*!< Protection violation */
    FTFx_ERR_ACCERR     = 0x0020U, /*!< Flash Access error */
    FTFx_ERR_RDCOLERR   = 0x0040U, /*!< Read Collision Error */
    FTFx_ERR_CHANGEPROT = 0x0100U, /*!< Cannot change protection status */
    FTFx_ERR_NOEEE      = 0x0200U, /*!< FlexRAM is not set for EEPROM use */
    FTFx_ERR_EFLASHONLY = 0x0400U, /*!< FlexNVM is set for full EEPROM backup */
    FTFx_ERR_RAMRDY     = 0x0800U, /*!< Programming acceleration RAM is not available */
    FTFx_ERR_RANGE      = 0x1000U, /*!< Address is out of the valid range */
    FTFx_ERR_SIZE       = 0x2000U  /*!< Misaligned size */
}flash_drv_status_t;


/*!
 * @name Flash security status
 * @{
 */
/*! @brief  Flash currently not in secure state */
#define FLASH_NOT_SECURE                   0x01U
/*! @brief  Flash is secured and backdoor key access enabled */
#define FLASH_SECURE_BACKDOOR_ENABLED      0x02U
/*! @brief  Flash is secured and backdoor key access disabled */
#define FLASH_SECURE_BACKDOOR_DISABLED     0x04U
/*@}*/


/*--------------------- CallBack function period -----------------------*/
#ifndef FLASH_CALLBACK_CS
/*! @brief  Callback period coint for FlashCheckSum */
#define FLASH_CALLBACK_CS               0x0AU
#endif

/*--------------------Null Callback function definition ----------------*/
/*!
 * @name Null Callback function definition
 * @{
 */
/*! @brief  Null callback */
#define NULL_CALLBACK                   ((PCALLBACK)0xFFFFFFFFU)
/*! @brief  Null swap callback */
#define NULL_SWAP_CALLBACK              ((PFLASH_SWAP_CALLBACK)0xFFFFFFFFU)
/*@}*/

/*-------------------- Callback function prototype ---------------------*/
/*! @brief Call back function pointer data type */
typedef void (* PCALLBACK)(void);
/*! @brief Swap call back function pointer data type */
typedef bool (* PFLASH_SWAP_CALLBACK)(uint8_t function);


/*---------------- Flash SSD Configuration Structure -------------------*/
/*! @brief Flash User Config Structure
*/
typedef struct _flash_user_config
{
    uint32_t      PFlashBase;         /*!< The base address of P-Flash memory */
    uint32_t      PFlashSize;         /*!< The size in byte of P-Flash memory */
    uint32_t      DFlashBase;         /*!< For FlexNVM device, this is the base address of D-Flash memory (FlexNVM memory); For non-FlexNVM device, this field is unused */
    uint32_t      EERAMBase;          /*!< The base address of  FlexRAM  (for FlexNVM
                                              device) or acceleration RAM  memory  (for non-FlexNVM device) */
    PCALLBACK     CallBack;          /*!< Call back function to service the time critical events */
} FLASH_USER_CONFIG,*PFLASH_USER_CONFIG;
/*! @brief Flash SSD Configuration Structure
*
* The structure includes the static parameters for  C90TFS/FTFx  which are
* device-dependent. The fields including
* PFlashBlockBase, PFlashBlockSize, DFlashBlockBase, EERAMBlockBase,
* and CallBack are passed via FLASH_USER_CONFIG.
* The rest of parameters such as DFlashBlockSize, and EEEBlockSize will be
* initialized in FlashInit() automatically.
*/
typedef struct _ssd_config
{
    uint32_t      PFlashBase;         /*!< The base address of P-Flash memory */
    uint32_t      PFlashSize;         /*!< The size in byte of P-Flash memory */
    uint32_t      DFlashBase;         /*!< For FlexNVM device, this is the base address of D-Flash memory (FlexNVM memory); For non-FlexNVM device, this field is unused */
    uint32_t      DFlashSize;         /*!< For FlexNVM device, this is the size in byte of area
                                          which is used as  D-Flash  from FlexNVM
                                          memory;  For non-FlexNVM device, this field is unused */
    uint32_t      EERAMBase;          /*!< The base address of  FlexRAM  (for FlexNVM
                                          device) or acceleration RAM  memory  (for non-FlexNVM device) */
    uint32_t      EEESize;            /*!< For FlexNVM device, this is the size in byte of
                                          EEPROM area  which was partitioned from
                                          FlexRAM; For non-FlexNVM device, this field is unused */
    PCALLBACK     CallBack;          /*!< Call back function to service the time critical events */
} FLASH_SSD_CONFIG,*PFLASH_SSD_CONFIG;

/* -------------------- Function Pointer ------------------------------- */
/*! @brief FlashCommandSequence function pointer */
typedef flash_drv_status_t (*pFLASHCOMMANDSEQUENCE) (const PFLASH_SSD_CONFIG pSSDConfig);

/*---------------- Function Prototypes for Flash SSD --------------------*/

/*!
 * @name C90TFS Flash driver APIs
 * @{
 */
#if defined(__cplusplus)
extern "C" {
#endif
/*!
 * @brief Relocates a function to RAM address.
 *
 * This function provides a facility to relocate a function in RAM.
 *
 * @param dest:    Destination address where you want to place the function.
 * @param size:    Size of the function
 * @param src:     Address of the function will be relocated
 * @return Relocated address of the function .
 */
uint32_t RelocateFunction(uint32_t dest, uint32_t size, uint32_t src);

/*!
 * @brief Initializes Flash.
 *
 * This API  initializes  Flash  module by clearing status error
 * bit and reporting the memory configuration via SSD configuration structure.
 *
 * @param pUserConf:    The user configuration structure pointer.
 * @param pSSDConfig:    The SSD configuration structure pointer.
 * @param pFlashCommandSequence :     Pointer to the Flash command sequence function.
 * @return Successful completion (FTFx_OK)
 */
flash_drv_status_t FlashInit(const PFLASH_USER_CONFIG pUserConf, PFLASH_SSD_CONFIG pSSDConfig, pFLASHCOMMANDSEQUENCE pFlashCommandSequence);

/*!
 * @brief Flash command sequence.
 *
 * This API is used to perform command write sequence on  Flash.
 * It is internal function, called by driver APIs only.
 *
 * @param pSSDConfig:    The SSD configuration structure pointer.
 * @return Successful completion (FTFx_OK)
 * @return Failed in Flash command execution (FTFx_ERR_ACCERR, FTFx_ERR_PVIOL,
 * FTFx_ERR_MGSTAT0)
 */
flash_drv_status_t FlashCommandSequence(const PFLASH_SSD_CONFIG pSSDConfig);

/*!
 * @brief P-Flash get protection.
 *
 * This  API  retrieves  the current  P-Flash  protection  status.  Considering
 * the  time  consumption  for getting protection is very  low and even can
 * be  ignored. It is not necessary to utilize the Callback function to
 * support the time-critical events.
 *
 * @param protectStatus: To return the current value of the P-Flash Protection.
 *                       Each bit is corresponding
 *                       to protection of 1/32 of the total P-Flash. The least
 *                       significant bit is corresponding to the lowest
 *                       address area of P-Flash. The most significant bit
 *                       is corresponding to the highest address area of P-
 *                       Flash and so on. There are two possible cases as below:
 *                       -  0: this area is protected.
 *                       -  1: this area is unprotected.
 * @return
 */
void PFlashGetProtection(uint32_t*  protectStatus);

/*!
 * @brief P-Flash set protection.
 *
 * This API sets the P-Flash protection to the intended protection status.
 * Setting P-Flash protection status is subject to a protection transition
 * restriction. If there is a setting violation, it  returns
 * an error code and the current protection status won’t be changed.
 *
 * @param protectStatus: The expected protect status user wants to set to
 *                       P-Flash protection register. Each bit is corresponding
 *                       to protection of 1/32 of the total P-Flash. The least
 *                       significant bit is corresponding to the lowest
 *                       address area of P-Flash. The most significant bit
 *                       is corresponding to the highest address area of P-
 *                       Flash, and so on. There are two possible cases as shown below:
 *                       -  0: this area is protected.
 *                       -  1: this area is unprotected.
 * @return Successful completion (FTFx_OK )
 * @return Error value (FTFx_ERR_CHANGEPROT)
 */
flash_drv_status_t PFlashSetProtection(uint32_t  protectStatus);

/*!
 * @brief Flash get security state.
 *
 * This API retrieves the current Flash security status, including
 * the security enabling state and the back door key enabling state.
 *
 * @param securityState: To return the current security status code.
 *                       FLASH_NOT_SECURE (0x01): Flash currently not in secure state
 *                       FLASH_SECURE_BACKDOOR_ENABLED (0x02): Flash is secured and
 *                       back door key access enabled
 *                       FLASH_SECURE_BACKDOOR_DISABLED (0x04): Flash is secured and
 *                       back door key access disabled.
 * @return
 */
void FlashGetSecurityState(uint8_t* securityState);

/*!
 * @brief Flash security bypass.
 *
 * This API un-secures the device by comparing the user's provided back
 * door key with the ones in  the Flash Configuration Field.  If  they are
 * matched,  the  security is  released. Otherwise, an
 * error code is returned.
 *
 * @param pSSDConfig:    The SSD configuration structure pointer.
 * @param keyBuffer:     Point to the user buffer containing the back door key.
 * @param pFlashCommandSequence :     Pointer to the Flash command sequence function.
 * @return Successful completion (FTFx_OK)
 * @return Error value  (FTFx_ERR_ACCERR)
 */
flash_drv_status_t FlashSecurityBypass(const PFLASH_SSD_CONFIG pSSDConfig, \
                                  const uint8_t* keyBuffer, \
                                  pFLASHCOMMANDSEQUENCE pFlashCommandSequence);
/*!
 * @brief Flash erase all Blocks.
 *
 * This API  erases all Flash memory,  initializes  the FlexRAM, verifies
 * all memory contents, and then releases the MCU security.
 *
 * @param pSSDConfig:    The SSD configuration structure pointer.
 * @param pFlashCommandSequence :     Pointer to the Flash command sequence function.
 * @return Successful completion (FTFx_OK)
 * @return Error value  (FTFx_ERR_PVIOL, FTFx_ERR_MGSTAT0, FTFx_ERR_ACCERR)
 */
flash_drv_status_t FlashEraseAllBlock(const PFLASH_SSD_CONFIG pSSDConfig, \
                                      pFLASHCOMMANDSEQUENCE pFlashCommandSequence);

/*!
 * @brief Flash verify all Blocks.
 *
 * This function checks to see if the P-Flash and/or D-Flash, EEPROM
 * backup area, and D-Flash IFR have been erased to the specified read
 * margin level, if applicable, and releases security if the readout passes.
 *
 * @param pSSDConfig:    The SSD configuration structure pointer.
 * @param marginLevel:   Read Margin Choice as follows:
 *                       marginLevel = 0x0: use the Normal read level
 *                       marginLevel = 0x1: use the User read
 *                       marginLevel = 0x2: use the Factory read
 * @param pFlashCommandSequence :     Pointer to the Flash command sequence function.
 * @return Successful completion (FTFx_OK)
 * @return Error value  (FTFx_ERR_MGSTAT0, FTFx_ERR_ACCERR)
 */
flash_drv_status_t FlashVerifyAllBlock(const PFLASH_SSD_CONFIG pSSDConfig, \
                                  uint8_t marginLevel, \
                                  pFLASHCOMMANDSEQUENCE pFlashCommandSequence);

/*!
 * @brief Flash erase sector.
 *
 * This API erases one or more sectors in P-Flash or  D-Flash memory.
 * This  API  always  returns  FTFx_OK  if  size  provided  by  the user  is
 * zero  regardless  of  the  input validation.
 *
 * @param pSSDConfig:    The SSD configuration structure pointer.
 * @param dest:          Address in the first sector to be erased.
 * @param size:          Size to be erased in bytes. It is used to determine
 *                       number of sectors to be erased.
 * @param pFlashCommandSequence :     Pointer to the Flash command sequence function.
 * @return Successful completion (FTFx_OK)
 * @return Error value  (FTFx_ERR_MGSTAT0, FTFx_ERR_ACCERR, FTFx_ERR_PVIOL,FTFx_ERR_SIZE)
 */
flash_drv_status_t FlashEraseSector(const PFLASH_SSD_CONFIG pSSDConfig, \
                               uint32_t dest, \
                               uint32_t size, \
                               pFLASHCOMMANDSEQUENCE pFlashCommandSequence);

/*!
 * @brief Flash verify section.
 *
 * This API  checks  if a section of the P-Flash or the D-Flash memory
 * is erased  to  the specified read margin level.
 *
 * @param pSSDConfig:    The SSD configuration structure pointer.
 * @param dest:          Start address for the intended verify operation.
 * @param number:        Number of alignment unit to be verified. Refer to
 *                       corresponding reference manual to get correct
 *                       information of alignment constrain.
 * @param marginLevel:   Read Margin Choice as follows:
 *                       marginLevel = 0x0: use Normal read level
 *                       marginLevel = 0x1: use the User read
 *                       marginLevel = 0x2: use the Factory read
 * @param pFlashCommandSequence :     Pointer to the Flash command sequence function.
 * @return Successful completion (FTFx_OK)
 * @return Error value  (FTFx_ERR_MGSTAT0, FTFx_ERR_ACCERR)
 */
flash_drv_status_t FlashVerifySection(const PFLASH_SSD_CONFIG pSSDConfig, \
                                 uint32_t dest, \
                                 uint16_t number, \
                                 uint8_t marginLevel, \
                                 pFLASHCOMMANDSEQUENCE pFlashCommandSequence);
/*!
 * @brief Flash erase suspend.
 *
 * This API is used to suspend a current operation of Flash erase sector command.
 * This function must be located in RAM memory or different Flash blocks which are
 * targeted for writing to avoid the RWW error.
 *
 * @param
 * @return
 */
void FlashEraseSuspend(void);
/*!
 * @brief Flash erase resume.
 *
 * This API is used to resume a previous suspended operation of Flash erase sector command
 * This function must be located in RAM memory or different Flash blocks which are targeted
 * for writing to avoid RWW error.
 *
 * @param
 * @return
 */
void FlashEraseResume(void);
/*!
 * @brief Flash read once.
 *
 * This API is used to read out a reserved 64 byte field located in the P-Flash IFR via given number
 * of record. See the corresponding reference manual to get the correct value of this number.
 *
 * @param pSSDConfig:    The SSD configuration structure pointer.
 * @param recordIndex:   The record index will be read. It can be from 0x0
 *                       to 0x7 or from 0x0 to 0xF according to specific derivative.
 * @param pDataArray:    Pointer to the array to return the data read by the read once command.
 * @param pFlashCommandSequence :     Pointer to the Flash command sequence function.
 * @return Successful completion (FTFx_OK)
 * @return Error value  (FTFx_ERR_ACCERR)
 */
flash_drv_status_t FlashReadOnce(const PFLASH_SSD_CONFIG pSSDConfig, \
                            uint8_t recordIndex,\
                            uint8_t* pDataArray, \
                            pFLASHCOMMANDSEQUENCE pFlashCommandSequence);
/*!
 * @brief Flash program once.
 *
 * This API  is  used  to  program  to  a  reserved  64  byte  field  located  in  the
 * P-Flash  IFR  via  given number of record. See the corresponding reference manual
 * to get correct value of this number.
 *
 * @param pSSDConfig:    The SSD configuration structure pointer.
 * @param recordIndex:   The record index will be read. It can be from 0x0
 *                       to 0x7 or from 0x0 to 0xF according to specific derivative.
 * @param pDataArray:    Pointer to the array from which data will be
 *                       taken for program once command.
 * @param pFlashCommandSequence :     Pointer to the Flash command sequence function.
 * @return Successful completion (FTFx_OK)
 * @return Error value  (FTFx_ERR_ACCERR,FTFx_ERR_MGSTAT0)
 */
flash_drv_status_t FlashProgramOnce(const PFLASH_SSD_CONFIG pSSDConfig, \
                               uint8_t recordIndex,\
                               const uint8_t* pDataArray, \
                               pFLASHCOMMANDSEQUENCE pFlashCommandSequence);
/*!
 * @brief Flash read resource.
 *
 * This API is used to read data from special purpose memory in Flash memory module
 * including P-Flash IFR, swap IFR, D-Flash IFR space and version ID.
 *
 * @param pSSDConfig:    The SSD configuration structure pointer.
 * @param dest:          Start address for the intended read operation.
 * @param pDataArray:    Pointer to the data returned by the read resource command.
 * @param resourceSelectCode:    Read resource select code:
 *                               0 : Flash IFR
 *                               1: Version ID
 * @param pFlashCommandSequence :     Pointer to the Flash command sequence function.
 * @return Successful completion (FTFx_OK)
 * @return Error value  (FTFx_ERR_ACCERR)
 */
flash_drv_status_t FlashReadResource(const PFLASH_SSD_CONFIG pSSDConfig, \
                                uint32_t dest, \
                                uint8_t* pDataArray, \
                                uint8_t  resourceSelectCode, \
                                pFLASHCOMMANDSEQUENCE pFlashCommandSequence);
/*!
 * @brief Flash program
 *
 * This  API  is  used  to  program  4  consecutive  bytes  (for  program  long
 * word  command)  and  8 consecutive bytes (for program phrase command) on
 * P-Flash or D-Flash block. This  API  always  returns  FTFx_OK  if  size
 * provided  by  user  is  zero  regardless  of  the  input validation
 *
 * @param pSSDConfig:    The SSD configuration structure pointer.
 * @param dest:          Start address for the intended program operation.
 * @param size:          Size in byte to be programmed
 * @param pData:         Pointer of source address from which data has to
 *                       be taken for program operation.
 * @param pFlashCommandSequence :     Pointer to the Flash command sequence function.
 * @return Successful completion (FTFx_OK)
 * @return Error value  (FTFx_ERR_ACCERR, FTFx_ERR_PVIOL, FTFx_ERR_SIZE, FTFx_ERR_MGSTAT0)
 */
flash_drv_status_t FlashProgram(const PFLASH_SSD_CONFIG pSSDConfig, \
                                   uint32_t dest, \
                                   uint32_t size, \
                                   uint8_t* pData, \
                                   pFLASHCOMMANDSEQUENCE pFlashCommandSequence);
/*!
 * @brief Flash program check
 *
 * This API tests a previously programmed P-Flash or D-Flash long word
 * to see if it reads correctly at the specified margin level. This
 * API always returns FTFx_OK if size provided by user is zero
 * regardless  of  the  input validation
 *
 * @param pSSDConfig:    The SSD configuration structure pointer.
 * @param dest:          Start address for the intended program check operation.
 * @param size:          Size in byte to check accuracy of program operation
 * @param pExpectedData: The pointer to the expected data.
 * @param pFailAddr:     Returned the first aligned failing address.
 * @param marginLevel:   Read margin choice as follows:
 *                       marginLevel = 0x1: read at User margin 1/0 level.
 *                       marginLevel = 0x2: read at Factory margin 1/0 level.
 * @param pFlashCommandSequence :     Pointer to the Flash command sequence function.
 * @return Successful completion (FTFx_OK)
 * @return Error value  (FTFx_ERR_ACCERR, FTFx_ERR_MGSTAT0)
 */
flash_drv_status_t FlashProgramCheck(const PFLASH_SSD_CONFIG pSSDConfig, \
                                uint32_t  dest, \
                                uint32_t  size, \
                                uint8_t*  pExpectedData, \
                                uint32_t* pFailAddr, \
                                uint8_t   marginLevel, \
                                pFLASHCOMMANDSEQUENCE pFlashCommandSequence);
/*!
 * @brief Calculates check sum.
 *
 * This API  performs 32 bit sum of each byte data over a specified Flash
 * memory range without carry which provides rapid method for checking data integrity.
 * The  callback  time  period  of  this  API  is  determined  via  FLASH_CALLBACK_CS  macro  in the
 * SSD_FTFx_Common.h  which is used as a counter value for the CallBack() function calling in
 * this API. This value can be changed as per  the user  requirement. User can change  this value  to
 * obtain the maximum permissible callback time period.
 * This  API  always  returns  FTFx_OK  if  size  provided  by  user  is  zero  regardless  of  the  input
 * validation.
 *
 * @param pSSDConfig:    The SSD configuration structure pointer.
 * @param dest:          Start address of the Flash range to be summed
 * @param size:          Size in byte of the Flash range to be summed
 * @param pSum:          To return the sum value
 * @return Successful completion (FTFx_OK)
 * @return Error value  (FTFx_ERR_RANGE)
 */
flash_drv_status_t FlashCheckSum(const PFLASH_SSD_CONFIG pSSDConfig, \
                            uint32_t dest, \
                            uint32_t size, \
                            uint32_t* pSum);

#if FSL_FEATURE_FLASH_HAS_PROGRAM_SECTION_CMD
/*!
 * @brief Flash program section
 *
 * This API will program the data found in the Section Program Buffer
 * to previously erased locations in the Flash memory. Data is preloaded into
 * the Section Program Buffer by writing to the acceleration Ram and FlexRam
 * while it is set to function as a RAM. The Section Program Buffer is limited
 * to  the  value of FlexRam divides by a ratio. Refer to the associate reference
 * manual to get correct value of this ratio.
 * For derivatives including swap feature, the swap indicator address is encountered
 * during FlashProgramSection, it is bypassed without setting FPVIOL but the content
 * are not be programmed. In addition, the content of source data used to program to
 * swap indicator will be re-initialized to 0xFF after completion of this command.
 *
 * @param pSSDConfig:    The SSD configuration structure pointer.
 * @param dest:          Start address for the intended program operation.
 * @param number:          Number of alignment unit to be programmed. Refer to associate
 *                       reference manual to get correct value of this alignment constrain.
 * @param pFlashCommandSequence :  Pointer to the Flash command sequence function.
 * @return Successful completion (FTFx_OK)
 * @return Error value  (FTFx_ERR_ACCERR, FTFx_ERR_PVIOL, FTFx_ERR_MGSTAT0, FTFx_ERR_RAMRDY)
 */
flash_drv_status_t FlashProgramSection(const PFLASH_SSD_CONFIG pSSDConfig, \
                                  uint32_t dest, \
                                  uint16_t number, \
                                  pFLASHCOMMANDSEQUENCE pFlashCommandSequence);
#endif

#if FSL_FEATURE_FLASH_HAS_ERASE_FLASH_BLOCK_CMD
/*!
 * @brief Flash erase block
 *
 * This API  erases all addresses in an individual P-Flash or D-Flash block.
 * For  the derivatives  including multiply  logical P-Flash or D-Flash blocks,
 * this API   erases a single block in a single call.
 *
 * @param pSSDConfig:    The SSD configuration structure pointer.
 * @param dest:          Start address for the intended erase operation.
 * @param pFlashCommandSequence :  Pointer to the Flash command sequence function.
 * @return Successful completion (FTFx_OK)
 * @return Error value  (FTFx_ERR_ACCERR, FTFx_ERR_PVIOL, FTFx_ERR_MGSTAT0)
 */
flash_drv_status_t FlashEraseBlock(const PFLASH_SSD_CONFIG pSSDConfig, \
                                   uint32_t dest, \
                                   pFLASHCOMMANDSEQUENCE pFlashCommandSequence);
/*!
 * @brief Flash verify block
 *
 * This API  checks to see  if an entire P-Flash or D-Flash block has been
 * erased to the specified margin level
 * For  the derivatives  including multiply  logical P-Flash or D-Flash blocks,
 * this API   erases a single block in a single call.
 *
 * @param pSSDConfig:    The SSD configuration structure pointer.
 * @param dest:          Start address for the intended verify operation.
 * @param marginLevel:   Read Margin Choice as follows:
 *                       marginLevel = 0x0: use Normal read level
 *                       marginLevel = 0x1: use the User read
 *                       marginLevel = 0x2: use the Factory read
 * @param pFlashCommandSequence :  Pointer to the Flash command sequence function.
 * @return Successful completion (FTFx_OK)
 * @return Error value  (FTFx_ERR_ACCERR, FTFx_ERR_MGSTAT0)
 */
flash_drv_status_t FlashVerifyBlock(const PFLASH_SSD_CONFIG pSSDConfig, \
                               uint32_t dest, \
                               uint8_t marginLevel, \
                               pFLASHCOMMANDSEQUENCE pFlashCommandSequence);
#endif

#if FSL_FEATURE_FLASH_HAS_FLEX_NVM
/*!
 * @brief EERAM get protection
 *
 * This  API  retrieves  which  EEPROM  sections  of  FlexRAM  are  protected
 * against  program  and erase operations. Considering  the  time consumption
 * for getting protection is very low and even can be ignored, it is not necessary
 * to utilize the Callback function to support the time-critical events
 *
 * @param protectStatus: To return the current value of the EEPROM
 *                       Protection Register. Each bit is corresponding to
 *                       protection status of 1/8 of the total EEPROM
 *                       use. The least significant bit is corresponding to
 *                       the lowest address area of EEPROM. The most
 *                       significant bit is corresponding to the highest
 *                       address area of EEPROM and so on. There are
 *                       two possible cases as below:
 *                       -  0: this area is protected.
 *                       -  1: this area is unprotected.
 * @return Successful completion (FTFx_OK)
 * @return Error value  (FTFx_ERR_NOEEE)
 */
flash_drv_status_t EERAMGetProtection(uint8_t*  protectStatus);
/*!
 * @brief EERAM set protection
 *
 * This API sets protection to the intended protection status for EEPROM us
 * area of FlexRam. This is subject to a protection transition restriction.
 * If there is a setting violation, it returns failed information and
 * the current protection status won’t be changed.
 *
 * @param protectStatus: The intended protection status value should be
 *                       written to the EEPROM Protection Register.
 *                       Each bit is corresponding to
 *                       protection status of 1/8 of the total EEPROM
 *                       use. The least significant bit is corresponding to
 *                       the lowest address area of EEPROM. The most
 *                       significant bit is corresponding to the highest
 *                       address area of EEPROM and so on. There are
 *                       two possible cases as below:
 *                       -  0: this area is protected.
 *                       -  1: this area is unprotected.
 * @return Successful completion (FTFx_OK)
 * @return Error value  (FTFx_ERR_NOEEE,FTFx_ERR_CHANGEPROT)
 */
flash_drv_status_t EERAMSetProtection(uint8_t  protectStatus);
/*!
 * @brief Flash Set EEEEnable
 *
 * This function is used to change the function of the FlexRAM. When not
 * partitioned for EEPROM backup, the FlexRam is typically used as traditional
 * RAM. Otherwise, the FlexRam is typically used to store EEPROM data and user
 * can use this API to change its functionality according to his application requirement.
 * For example, after partitioning to have EEPROM backup, FlexRAM  is used  for EEPROM
 * use accordingly. And this API is used to set FlexRAM is available for
 * traditional RAM for FlashProgramSection() use.
 *
 * @param pSSDConfig:               The SSD configuration structure pointer.
 * @param EEEEnable:                FlexRam function control code. It can be:
 *                                  -  0xFF: make FlexRam available for RAM.
 *                                  -  0x00: make FlexRam available for EEPROM.
 * @param pFlashCommandSequence:    Pointer to the Flash command sequence function.
 * @return Successful completion (FTFx_OK)
 * @return Error value  (FTFx_ERR_ACCERR)
 */
flash_drv_status_t SetEEEEnable(const PFLASH_SSD_CONFIG pSSDConfig, \
                           uint8_t EEEEnable,pFLASHCOMMANDSEQUENCE pFlashCommandSequence);
/*!
 * @brief EEPROM Emulator Write
 *
 * This API is used to write data to FlexRAM section which is  partitioned
 * as EEPROM use for EEPROM operation. After data has been written to EEPROM
 * use section of FlexRAM, the EEPROM file system will create new data record
 * in EEPROM back-up area of FlexNVM in round-robin fashion.
 * There  is no alignment constraint for destination and size parameters
 * provided by user. However, according to user’s input provided, this
 * API will set priority to write to FlexRAM with following rules:
 * 32-bit writing is invoked if destination is 32 bit aligned and size
 * is not  less than 32 bits.
 * 16-bit writing is  invoked if destination is 16 bit aligned and size
 * is not  less than 16 bits.
 * 8-bit writing is invoked if destination is 8 bit aligned and size is not less than 8 bits.
 *
 * @param pSSDConfig:    The SSD configuration structure pointer.
 * @param dest:          Start address for the intended write operation.
 * @param size:          Size in byte to be written.
 * @param pData:         Pointer to source address from which data
 *                       has to be taken for writing operation.
 * @return Successful completion (FTFx_OK)
 * @return Error value  (FTFx_ERR_RANGE, FTFx_ERR_NOEEE, FTFx_ERR_PVIOL)
 */
flash_drv_status_t EEEWrite(const PFLASH_SSD_CONFIG pSSDConfig, uint32_t dest, uint32_t size, uint8_t* pData);
/*!
 * @brief Flash D/E-Flash Partition.
 *
 * This API prepares the FlexNVM block for use as D-Flash, EEPROM backup, or a combination
 * of both and initializes the FlexRAM.
 *
 * The single partition choice should be used through entire life time of a given
 * application to guarantee the Flash endurance and data retention of Flash module.
 *
 * @param   pSSDConfig                The SSD configuration structure pointer
 * @param   EEEDataSizeCode           EEPROM Data Size Code
 * @param   DEPartitionCode           FlexNVM Partition Code
 * @param   pFlashCommandSequence     Pointer to the Flash command sequence function.
 *
 * @return  Successful completion(FTFx_OK)
 * @return  Error value(FTFx_ERR_ACCERR, FTFx_ERR_MGSTAT0)
 */
flash_drv_status_t DEFlashPartition(const PFLASH_SSD_CONFIG pSSDConfig, \
                               uint8_t EEEDataSizeCode, \
                               uint8_t DEPartitionCode, \
                               pFLASHCOMMANDSEQUENCE pFlashCommandSequence);
/*!
 * @brief D-Flash get protection.
 *
 * This API retrieves current P-Flash protection status. Considering the time consumption
 * for getting protection is very low and even can be ignored, it is not necessary to utilize
 * the Callback function to support the time-critical events.
 *
 * @param   pSSDConfig                The SSD configuration structure pointer
 * @param   protectStatus             To return the current value of the D-Flash Protection
 *                                    Register. Each bit is corresponding to protection status
 *                                    of 1/8 of the total D-Flash. The least significant bit is
 *                                    corresponding to the lowest address area of D-Flash. The
 *                                    most significant bit is corresponding to the highest address
 *                                    area of D-Flash and so on. There are two possible cases as below:
 *                                    - 0 : this area is protected.
 *                                    - 1 : this area is unprotected.
 *
 * @return  Successful completion(FTFx_OK)
 * @return  Error value(FTFx_ERR_EFLASHONLY)
 */
flash_drv_status_t DFlashGetProtection(const PFLASH_SSD_CONFIG pSSDConfig, uint8_t*  protectStatus);
/*!
 * @brief D-Flash set protection.
 *
 * This API sets the D-Flash protection to the intended protection status. Setting D-Flash
 * protection status is subject to a protection transition restriction. If there is a setting
 * violation, it returns failed information and the current protection status won’t be changed.
 *
 * @param   pSSDConfig                The SSD configuration structure pointer
 * @param   protectStatus             The expected protect status user wants to set to D-Flash Protection
 *                                    Register. Each bit is corresponding to protection status
 *                                    of 1/8 of the total D-Flash. The least significant bit is
 *                                    corresponding to the lowest address area of D-Flash. The
 *                                    most significant bit is corresponding to the highest address
 *                                    area of D-Flash and so on. There are two possible cases as below:
 *                                    - 0 : this area is protected.
 *                                    - 1 : this area is unprotected.
 *
 * @return  Successful completion(FTFx_OK)
 * @return  Error value(FTFx_ERR_EFLASHONLY,FTFx_ERR_CHANGEPROT)
 */
flash_drv_status_t DFlashSetProtection(const PFLASH_SSD_CONFIG pSSDConfig, uint8_t  protectStatus);
#endif /* End of FSL_FEATURE_FLASH_HAS_FLEX_NVM */

#ifdef FSL_FEATURE_FLASH_HAS_PFLASH_BLOCK_SWAP
/*!
 * @brief  Swaps between the two halves of the total logical P-Flash memory blocks in the memory map.
 *
 * The swap API provides to user with an ability to interfere in a swap progress by letting the
 * user code know about the swap state in each phase of the process. This is done via pSwapCallBack()
 * parameter. To stop at each intermediate swap state,  set the return value of
 * this callback function to FALSE. To complete swap process within a single call,
 * set the return value of this function to TRUE.
 *
 * Erase the non-active swap indicator  somewhere in the
 * application code or  in the swap call back function when swap system is in UPDATE state.
 *
 * In addition, if user does not want to use the swap call back parameter,  pass the NULL_SWAP_CALLBACK
 * as a null pointer. In this situation, the PFlashSwap()  behaves in the same way as  setting the return
 * value of pSwapCallBack to TRUE and the user does not need to erase the non-active swap
 * indicator when the swap system is in UPDATE state.
 *
 * Below is an example to show how to implement a swap callback:
 * @code
 * bool PFlashSwapCallback(uint8_t currentSwapMode)
 * {
 *     switch (currentSwapMode)
 *     {
 *     case FTFx_SWAP_UNINI:
 *         Put your application-specific code here
 *         break;
 *     case FTFx_SWAP_READY:
 *         Put your application-specific code here
 *         break;
 *     case FTFx_SWAP_UPDATE:
 *         Put your application-specific code here (example: erase non-active swap indicator here)
 *         break;
 *     case FTFx_SWAP_UPDATE_ERASED:
 *         Put your application-specific code here (example: erase non-active swap indicator here)
 *         break;
 *     case FTFx_SWAP_COMPLETE:
 *         Put your application-specific code here
 *         break;
 *     default:
 *         break;
 *    }
 *        return TRUE;  Return FALSE to stop at intermediate swap state
 *}
 * @endcode
 * The swap indicator provided by the user must be within the lower half of P-Flash block but not in the
 * Flash configuration area. If P-Flash block has two logical blocks, the swap indicator must be
 * in P-Flash block 0. If the P-Flash block has four logical blocks, the swap indicator can be in block
 * 0 or block 1. It must not be in the Flash configuration field.
 * The user must use the same swap indicator for all swap control code except report swap status once
 * swap system has been initialized. To refresh swap system to un-initialization state,
 * use the FlashEraseAllBlock() to clean up the swap environment.
 *
 * @param   pSSDConfig                The SSD configuration structure pointer
 * @param   addr                      Address of swap indicator.
 * @param   pSwapCallback             Callback to do specific task while the swapping is being performed
 * @param   pFlashCommandSequence     Pointer to the Flash command sequence function.
 *
 * @return  Successful completion(FTFx_OK)
 * @return  Error value(FTFx_ERR_ACCERR,FTFx_ERR_MGSTAT0)
 */
flash_drv_status_t PFlashSwap(const PFLASH_SSD_CONFIG pSSDConfig, \
                  uint32_t addr, \
                  PFLASH_SWAP_CALLBACK pSwapCallback, \
                  pFLASHCOMMANDSEQUENCE pFlashCommandSequence);
/*!
 * @brief  Implements swap control command corresponding with the swap control code provided via the swapcmd parameter.
 *
 * @param   pSSDConfig                The SSD configuration structure pointer
 * @param   addr                      Address of swap indicator.
 * @param   swapcmd                   Swap Control Code:
 *                                      0x01 - Initialize Swap System
 *                                      0x02 - Set Swap in Update State
 *                                      0x04 - Set Swap in Complete Stat
 *                                      0x08 - Report Swap Status
 * @param   pCurrentSwapMode          Current Swap Mode:
 *                                      0x00 - Uninitialized
 *                                      0x01 - Ready
 *                                      0x02 - Update
 *                                      0x03 - Update-Erased
 *                                      0x04 - Complete
 * @param   pCurrentSwapBlockStatus   Current Swap Block Status indicates which program Flash block
 *                                    is currently located at relative Flash address 0x0_0000
 *                                      0x00 - Program Flash block 0
 *                                      0x01 - Program Flash block 1
 * @param   pNextSwapBlockStatus      Next Swap Block Status indicates which program Flash block
 *                                    is located at relative Flash address 0x0_0000 after the next reset.
 *                                      0x00 - Program Flash block 0
 *                                      0x01 - Program Flash block 1
 * @param   pFlashCommandSequence     Pointer to the Flash command sequence function.
 *
 * @return  Successful completion(FTFx_OK)
 * @return  Error value(FTFx_ERR_ACCERR,FTFx_ERR_MGSTAT0)
 */
flash_drv_status_t PFlashSwapCtl(const PFLASH_SSD_CONFIG pSSDConfig, \
                             uint32_t addr, \
                             uint8_t swapcmd, \
                             uint8_t* pCurrentSwapMode,\
                             uint8_t* pCurrentSwapBlockStatus, \
                             uint8_t* pNextSwapBlockStatus, \
                             pFLASHCOMMANDSEQUENCE pFlashCommandSequence);
#endif /* End of FSL_FEATURE_FLASH_HAS_PFLASH_BLOCK_SWAP */
#if FSL_FEATURE_FLASH_HAS_ERASE_ALL_BLOCKS_UNSECURE_CMD
/*!
 * @brief Flash erase all Blocks unsecure.
 *
 * This API  erases all Flash memory,  initializes  the FlexRAM, verifies
 * all memory contents, and then releases the MCU security.
 *
 * @param pSSDConfig:    The SSD configuration structure pointer.
 * @param pFlashCommandSequence :     Pointer to the Flash command sequence function.
 * @return Successful completion (FTFx_OK)
 * @return Error value  (FTFx_ERR_PVIOL, FTFx_ERR_MGSTAT0, FTFx_ERR_ACCERR)
 */
flash_drv_status_t FlashEraseAllBlockUnsecure (const PFLASH_SSD_CONFIG pSSDConfig, \
                       pFLASHCOMMANDSEQUENCE pFlashCommandSequence);
#endif

#if defined(__cplusplus)
}
#endif

/*@}*/ /* End of C90TFS Flash driver APIs*/
/*! @}*/ /* End of addtogroup c90tfs_flash_driver */

#endif  /* FSL_FLASH_DRIVER_C90TFS_H_ */
