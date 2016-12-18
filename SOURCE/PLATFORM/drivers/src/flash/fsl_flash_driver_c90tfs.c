/************************************************************************
 (c) Copyright 2013-2015 Freescale Semiconductor, Inc.
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
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES,
 LOSS OF USE, DATA, OR PROFITS, OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
/* include the header files */
#include "fsl_flash_driver_c90tfs.h"

/*
 * brief Relocates a function to RAM address.
 *
 * This function provides a facility to relocate a function in RAM.
 *
 * param dest:    Destination address where you want to place the function.
 * param size:    Size of the function
 * param src:     Address of the function will be relocated
 * return Relocated address of the function .
 */
uint32_t  RelocateFunction(uint32_t dest, uint32_t size, uint32_t src)
{
    uint32_t temp, i;
    uint16_t value, *pSrc, *pDest;
    /* Get the address of source - the MSB is always 1 since it is THUMB2 instruction set*/
    temp = ((uint32_t)src - 1U);
    pSrc = (uint16_t *)temp;
    pDest = (uint16_t *)dest;
    temp = size >>1U;
    for (i = 0x0U; i < temp; i++)
    {
        value = *(uint16_t *)(pSrc);
        pSrc++;
        *(uint16_t *)pDest = value;
        pDest++;
    }

    return (dest + 1U);
}

/*
 * brief Initializes Flash.
 *
 * This API  initializes  Flash  module by clearing status error
 * bit and reporting the memory configuration via SSD configuration structure.
 *
 * param pSSDConfig:    The SSD configuration structure pointer.
 * param pFlashCommandSequence :     Pointer to the Flash command sequence function.
 * return Successful completion (FTFx_OK)
 */
flash_drv_status_t FlashInit(const PFLASH_USER_CONFIG pUserConf, PFLASH_SSD_CONFIG pSSDConfig, pFLASHCOMMANDSEQUENCE pFlashCommandSequence)
{
    flash_drv_status_t ret =  FTFx_OK;
#if FSL_FEATURE_FLASH_HAS_FLEX_NVM
    uint8_t  EEEDataSetSize;    /* store EEE Data Set Size */
    uint8_t  DEPartitionCode;    /* store D/E-Flash Partition Code */
#endif
    pSSDConfig->PFlashBase = pUserConf->PFlashBase;
    pSSDConfig->PFlashSize = pUserConf->PFlashSize;
    pSSDConfig->DFlashBase = pUserConf->DFlashBase;
    pSSDConfig->EERAMBase = pUserConf->EERAMBase;
    pSSDConfig->CallBack = pUserConf->CallBack;

#if FSL_FEATURE_FLASH_HAS_FLEX_NVM


    /* clear RDCOLERR & ACCERR & FPVIOL flag in flash status register. Write 1 to clear*/
    CLEAR_FTFx_FSTAT_ERROR_BITS;


    /* Write Command Code to FCCOB0 */
    FTFx_FCCOB0 = FTFx_READ_RESOURCE;

    /* Write address to FCCOB1/2/3 */
    FTFx_FCCOB1 = GET_BIT_16_23(DFLASH_IFR_READRESOURCE_ADDRESS);
    FTFx_FCCOB2 = GET_BIT_8_15(DFLASH_IFR_READRESOURCE_ADDRESS);
    FTFx_FCCOB3 = GET_BIT_0_7(DFLASH_IFR_READRESOURCE_ADDRESS);

    /* Write Resource Select Code of 0 to FCCOB8 to select IFR. Without this, */
    /* an access error may occur if the register contains data from a previous command. */
    /* for FTFE module, resource code is FCCOB4. For others, resource code is FCCOB8 */
    FTFx_RSRC_CODE_REG = 0U;

    ret = pFlashCommandSequence(pSSDConfig);
    /* read out EEdata set size and DEpartition code from FCCOBA, FCCOBB for FTFE module, from FCCOB6 and FCCOB7 for others */
    #ifdef FTFE
        EEEDataSetSize = FTFx_FCCOBA;
        DEPartitionCode = FTFx_FCCOBB;
    #else
        EEEDataSetSize = FTFx_FCCOB6;
        DEPartitionCode = FTFx_FCCOB7;
    #endif
    DEPartitionCode = DEPartitionCode & 0x0FU;
    EEEDataSetSize = EEEDataSetSize & 0x0FU;
        /* Calculate D-Flash size and EEE size */
    if       (0x0U == DEPartitionCode) {pSSDConfig->DFlashSize = (uint32_t)FSL_FEATURE_FLASH_FLEX_NVM_DFLASH_SIZE_FOR_DEPART_0000;}
    else if (0x01U == DEPartitionCode) {pSSDConfig->DFlashSize = (uint32_t)FSL_FEATURE_FLASH_FLEX_NVM_DFLASH_SIZE_FOR_DEPART_0001;}
    else if (0x02U == DEPartitionCode) {pSSDConfig->DFlashSize = (uint32_t)FSL_FEATURE_FLASH_FLEX_NVM_DFLASH_SIZE_FOR_DEPART_0010;}
    else if (0x03U == DEPartitionCode) {pSSDConfig->DFlashSize = (uint32_t)FSL_FEATURE_FLASH_FLEX_NVM_DFLASH_SIZE_FOR_DEPART_0011;}
    else if (0x04U == DEPartitionCode) {pSSDConfig->DFlashSize = (uint32_t)FSL_FEATURE_FLASH_FLEX_NVM_DFLASH_SIZE_FOR_DEPART_0100;}
    else if (0x05U == DEPartitionCode) {pSSDConfig->DFlashSize = (uint32_t)FSL_FEATURE_FLASH_FLEX_NVM_DFLASH_SIZE_FOR_DEPART_0101;}
    else if (0x06U == DEPartitionCode) {pSSDConfig->DFlashSize = (uint32_t)FSL_FEATURE_FLASH_FLEX_NVM_DFLASH_SIZE_FOR_DEPART_0110;}
    else if (0x07U == DEPartitionCode) {pSSDConfig->DFlashSize = (uint32_t)FSL_FEATURE_FLASH_FLEX_NVM_DFLASH_SIZE_FOR_DEPART_0111;}
    else if (0x08U == DEPartitionCode) {pSSDConfig->DFlashSize = (uint32_t)FSL_FEATURE_FLASH_FLEX_NVM_DFLASH_SIZE_FOR_DEPART_1000;}
    else if (0x09U == DEPartitionCode) {pSSDConfig->DFlashSize = (uint32_t)FSL_FEATURE_FLASH_FLEX_NVM_DFLASH_SIZE_FOR_DEPART_1001;}
    else if (0x0AU == DEPartitionCode) {pSSDConfig->DFlashSize = (uint32_t)FSL_FEATURE_FLASH_FLEX_NVM_DFLASH_SIZE_FOR_DEPART_1010;}
    else if (0x0BU == DEPartitionCode) {pSSDConfig->DFlashSize = (uint32_t)FSL_FEATURE_FLASH_FLEX_NVM_DFLASH_SIZE_FOR_DEPART_1011;}
    else if (0x0CU == DEPartitionCode) {pSSDConfig->DFlashSize = (uint32_t)FSL_FEATURE_FLASH_FLEX_NVM_DFLASH_SIZE_FOR_DEPART_1100;}
    else if (0x0DU == DEPartitionCode) {pSSDConfig->DFlashSize = (uint32_t)FSL_FEATURE_FLASH_FLEX_NVM_DFLASH_SIZE_FOR_DEPART_1101;}
    else if (0x0EU == DEPartitionCode) {pSSDConfig->DFlashSize = (uint32_t)FSL_FEATURE_FLASH_FLEX_NVM_DFLASH_SIZE_FOR_DEPART_1110;}
    else if (0x0FU == DEPartitionCode) {pSSDConfig->DFlashSize = (uint32_t)FSL_FEATURE_FLASH_FLEX_NVM_DFLASH_SIZE_FOR_DEPART_1111;}
    else {/* Undefined value */}

    if       (0x0U == EEEDataSetSize) {pSSDConfig->EEESize = (uint32_t)FSL_FEATURE_FLASH_FLEX_NVM_EEPROM_SIZE_FOR_EEESIZE_0000;}
    else if (0x01U == EEEDataSetSize) {pSSDConfig->EEESize = (uint32_t)FSL_FEATURE_FLASH_FLEX_NVM_EEPROM_SIZE_FOR_EEESIZE_0001;}
    else if (0x02U == EEEDataSetSize) {pSSDConfig->EEESize = (uint32_t)FSL_FEATURE_FLASH_FLEX_NVM_EEPROM_SIZE_FOR_EEESIZE_0010;}
    else if (0x03U == EEEDataSetSize) {pSSDConfig->EEESize = (uint32_t)FSL_FEATURE_FLASH_FLEX_NVM_EEPROM_SIZE_FOR_EEESIZE_0011;}
    else if (0x04U == EEEDataSetSize) {pSSDConfig->EEESize = (uint32_t)FSL_FEATURE_FLASH_FLEX_NVM_EEPROM_SIZE_FOR_EEESIZE_0100;}
    else if (0x05U == EEEDataSetSize) {pSSDConfig->EEESize = (uint32_t)FSL_FEATURE_FLASH_FLEX_NVM_EEPROM_SIZE_FOR_EEESIZE_0101;}
    else if (0x06U == EEEDataSetSize) {pSSDConfig->EEESize = (uint32_t)FSL_FEATURE_FLASH_FLEX_NVM_EEPROM_SIZE_FOR_EEESIZE_0110;}
    else if (0x07U == EEEDataSetSize) {pSSDConfig->EEESize = (uint32_t)FSL_FEATURE_FLASH_FLEX_NVM_EEPROM_SIZE_FOR_EEESIZE_0111;}
    else if (0x08U == EEEDataSetSize) {pSSDConfig->EEESize = (uint32_t)FSL_FEATURE_FLASH_FLEX_NVM_EEPROM_SIZE_FOR_EEESIZE_1000;}
    else if (0x09U == EEEDataSetSize) {pSSDConfig->EEESize = (uint32_t)FSL_FEATURE_FLASH_FLEX_NVM_EEPROM_SIZE_FOR_EEESIZE_1001;}
    else if (0x0AU == EEEDataSetSize) {pSSDConfig->EEESize = (uint32_t)FSL_FEATURE_FLASH_FLEX_NVM_EEPROM_SIZE_FOR_EEESIZE_1010;}
    else if (0x0BU == EEEDataSetSize) {pSSDConfig->EEESize = (uint32_t)FSL_FEATURE_FLASH_FLEX_NVM_EEPROM_SIZE_FOR_EEESIZE_1011;}
    else if (0x0CU == EEEDataSetSize) {pSSDConfig->EEESize = (uint32_t)FSL_FEATURE_FLASH_FLEX_NVM_EEPROM_SIZE_FOR_EEESIZE_1100;}
    else if (0x0DU == EEEDataSetSize) {pSSDConfig->EEESize = (uint32_t)FSL_FEATURE_FLASH_FLEX_NVM_EEPROM_SIZE_FOR_EEESIZE_1101;}
    else if (0x0EU == EEEDataSetSize) {pSSDConfig->EEESize = (uint32_t)FSL_FEATURE_FLASH_FLEX_NVM_EEPROM_SIZE_FOR_EEESIZE_1110;}
    else if (0x0FU == EEEDataSetSize) {pSSDConfig->EEESize = (uint32_t)FSL_FEATURE_FLASH_FLEX_NVM_EEPROM_SIZE_FOR_EEESIZE_1111;}
    else {/* Undefined value */}

#else /* FSL_FEATURE_FLASH_HAS_FLEX_NVM == 0 */
    /* If size of D/E-Flash = 0 */
    pSSDConfig->DFlashSize = 0x00U;
    pSSDConfig->EEESize = 0x00U;
#endif /* end of FSL_FEATURE_FLASH_HAS_FLEX_NVM */

    return(ret);
}

/*
 * brief Flash command sequence.
 *
 * This API is used to perform command write sequence on  Flash.
 * It is internal function, called by driver APIs only.
 *
 * param pSSDConfig:    The SSD configuration structure pointer.
 * return Successful completion (FTFx_OK)
 * return Failed in Flash command execution (FTFx_ERR_ACCERR, FTFx_ERR_PVIOL,
 * FTFx_ERR_MGSTAT0)
 */
flash_drv_status_t FlashCommandSequence (const PFLASH_SSD_CONFIG pSSDConfig)

{
    flash_drv_status_t ret;       /* return code variable */

    /* clear CCIF to launch command */
    FTFx_FSTAT |= (FTFx_FSTAT_CCIF_MASK);

    while(0x0U == (FTFx_FSTAT & FTFx_FSTAT_CCIF_MASK))
    {
        /* wait till CCIF bit is set */
        /* serve callback function if counter reaches limitation */
        if(NULL_CALLBACK != pSSDConfig->CallBack)
        {
            (pSSDConfig->CallBack)();
        }
    }

    ret = ((flash_drv_status_t)(FTFx_FSTAT & (FTFx_FSTAT_MGSTAT0_MASK | FTFx_FSTAT_FPVIOL_MASK \
                                                | FTFx_FSTAT_ACCERR_MASK | FTFx_FSTAT_RDCOLERR_MASK)));
    return(ret);
}
/*
 * brief P-Flash get protection.
 *
 * This  API  retrieves  the current  P-Flash  protection  status.  Considering
 * the  time  consumption  for getting protection is very  low and even can
 * be  ignored. It is not necessary to utilize the Callback function to
 * support the time-critical events.
 *
 * param protectStatus: To return the current value of the P-Flash Protection.
 *                       Each bit is corresponding
 *                       to protection of 1/32 of the total P-Flash. The least
 *                       significant bit is corresponding to the lowest
 *                       address area of P-Flash. The most significant bit
 *                       is corresponding to the highest address area of P-
 *                       Flash and so on. There are two possible cases as below:
 *                       -  0: this area is protected.
 *                       -  1: this area is unprotected.
 * return
 */
void PFlashGetProtection(uint32_t* protectStatus)
{
    uint32_t reg0, reg1, reg2, reg3;

    reg0 = FTFx_FPROT0;
    reg1 = FTFx_FPROT1;
    reg2 = FTFx_FPROT2;
    reg3 = FTFx_FPROT3;

    *protectStatus = (uint32_t)((uint32_t)(reg0 << 24) | (uint32_t)(reg1 << 16) | (uint32_t)(reg2 << 8) | reg3);

}

/*
 * brief P-Flash set protection.
 *
 * This API sets the P-Flash protection to the intended protection status.
 * Setting P-Flash protection status is subject to a protection transition
 * restriction. If there is a setting violation, it  returns
 * an error code and the current protection status won’t be changed.
 *
 * param protectStatus: The expected protect status user wants to set to
 *                       P-Flash protection register. Each bit is corresponding
 *                       to protection of 1/32 of the total P-Flash. The least
 *                       significant bit is corresponding to the lowest
 *                       address area of P-Flash. The most significant bit
 *                       is corresponding to the highest address area of P-
 *                       Flash, and so on. There are two possible cases as shown below:
 *                       -  0: this area is protected.
 *                       -  1: this area is unprotected.
 * return Successful completion (FTFx_OK )
 * return Error value (FTFx_ERR_CHANGEPROT)
 */
flash_drv_status_t PFlashSetProtection(uint32_t protectStatus)
{
    flash_drv_status_t ret = FTFx_OK;
    uint8_t reg0, reg1, reg2, reg3;

    reg0 = GET_BIT_24_31(protectStatus);
    reg1 = GET_BIT_16_23(protectStatus);
    reg2 = GET_BIT_8_15(protectStatus);
    reg3 = GET_BIT_0_7(protectStatus);

     FTFx_FPROT0 = reg0;
     FTFx_FPROT1 = reg1;
     FTFx_FPROT2 = reg2;
     FTFx_FPROT3 = reg3;

    /* Read the value of FPPROT registers */
    if ((FTFx_FPROT0 != reg0) || (FTFx_FPROT1 != reg1) || (FTFx_FPROT2 != reg2) || (FTFx_FPROT3 != reg3))
    {
        ret = FTFx_ERR_CHANGEPROT;
    }
    return(ret);
}

/*
 * brief Flash get security state.
 *
 * This API retrieves the current Flash security status, including
 * the security enabling state and the back door key enabling state.
 *
 * param securityState: To return the current security status code.
 *                       FLASH_NOT_SECURE (0x01): Flash currently not in secure state
 *                       FLASH_SECURE_BACKDOOR_ENABLED (0x02): Flash is secured and
 *                       back door key access enabled
 *                       FLASH_SECURE_BACKDOOR_DISABLED (0x04): Flash is secured and
 *                       back door key access disabled.
 * return
 */
void FlashGetSecurityState(uint8_t* securityState)
{
    /* store data read from flash register */
    uint8_t  regValue;
    /*Get flash security register value */
    regValue = FTFx_FSEC;

    /* check the status of the flash security bits in the security register */
    if(FLASH_SECURITY_STATE_UNSECURED == (regValue & FTFx_FSEC_SEC_MASK))
    {
        /* Flash in unsecured state */
        *securityState = FLASH_NOT_SECURE;
    }
    else
    {
        /* Flash in secured state */
        /* check for backdoor key security enable bit */
        if(0x80U == (regValue & FTFx_FSEC_KEYEN_MASK))
        {
            /* Backdoor key security enabled */
            *securityState = FLASH_SECURE_BACKDOOR_ENABLED;
        }
        else
        {
            /* Backdoor key security disabled */
            *securityState = FLASH_SECURE_BACKDOOR_DISABLED;
        }
    }
}

/*
 * brief Flash security bypass.
 *
 * This API un-secures the device by comparing the user's provided back
 * door key with the ones in  the Flash Configuration Field.  If  they are
 * matched,  the  security is  released. Otherwise, an
 * error code is returned.
 *
 * param pSSDConfig:    The SSD configuration structure pointer.
 * param keyBuffer:     Point to the user buffer containing the back door key.
 * param pFlashCommandSequence :     Pointer to the Flash command sequence function.
 * return Successful completion (FTFx_OK)
 * return Error value  (FTFx_ERR_ACCERR)
 */
flash_drv_status_t FlashSecurityBypass(const PFLASH_SSD_CONFIG pSSDConfig, \
                           const uint8_t* keyBuffer, \
                           pFLASHCOMMANDSEQUENCE pFlashCommandSequence)
{
    flash_drv_status_t ret;      /* return code variable */
    uint32_t temp;     /* temporary variable */
    uint8_t i;

    /* clear RDCOLERR & ACCERR & FPVIOL flag in flash status register. Write 1 to clear*/
    CLEAR_FTFx_FSTAT_ERROR_BITS;

    /* passing parameter to the command */
    FTFx_FCCOB0 = FTFx_SECURITY_BY_PASS;
    for (i = 0x0U; i < 0x08U; i++)
    {
        temp = FTFx_BASE + i + 0x08U;
        *(uint8_t *) (temp) = keyBuffer[i];
    }
    ret = pFlashCommandSequence(pSSDConfig);

    return(ret);
}

/*
 * brief Flash erase all Blocks.
 *
 * This API  erases all Flash memory,  initializes  the FlexRAM, verifies
 * all memory contents, and then releases the MCU security.
 *
 * param pSSDConfig:    The SSD configuration structure pointer.
 * param pFlashCommandSequence :     Pointer to the Flash command sequence function.
 * return Successful completion (FTFx_OK)
 * return Error value  (FTFx_ERR_PVIOL, FTFx_ERR_MGSTAT0, FTFx_ERR_ACCERR)
 */
flash_drv_status_t FlashEraseAllBlock (const PFLASH_SSD_CONFIG pSSDConfig, \
                       pFLASHCOMMANDSEQUENCE pFlashCommandSequence)
{
    flash_drv_status_t ret;         /* return code variable */

    /* clear RDCOLERR & ACCERR & FPVIOL flag in flash status register. Write 1 to clear*/
    CLEAR_FTFx_FSTAT_ERROR_BITS;
    /* passing parameter to the command */
    FTFx_FCCOB0 = FTFx_ERASE_ALL_BLOCK;

    /* calling flash command sequence function to execute the command */
    ret = pFlashCommandSequence(pSSDConfig);
    return(ret);
}

/*
 * brief Flash verify all Blocks.
 *
 * This function checks to see if the P-Flash and/or D-Flash, EEPROM
 * backup area, and D-Flash IFR have been erased to the specified read
 * margin level, if applicable, and releases security if the readout passes.
 *
 * param pSSDConfig:    The SSD configuration structure pointer.
 * param marginLevel:   Read Margin Choice as follows:
 *                       marginLevel = 0x0: use the Normal read level
 *                       marginLevel = 0x1: use the User read
 *                       marginLevel = 0x2: use the Factory read
 * param pFlashCommandSequence :     Pointer to the Flash command sequence function.
 * return Successful completion (FTFx_OK)
 * return Error value  (FTFx_ERR_MGSTAT0, FTFx_ERR_ACCERR)
 */
flash_drv_status_t FlashVerifyAllBlock(const PFLASH_SSD_CONFIG pSSDConfig, \
                           uint8_t marginLevel, \
                           pFLASHCOMMANDSEQUENCE pFlashCommandSequence)
{
    flash_drv_status_t ret;         /* return code variable */
   /* clear RDCOLERR & ACCERR & FPVIOL flag in flash status register. Write 1 to clear*/
    CLEAR_FTFx_FSTAT_ERROR_BITS;
    /* passing parameter to the command */
    FTFx_FCCOB0 = FTFx_VERIFY_ALL_BLOCK;
    FTFx_FCCOB1 = marginLevel;
    /* calling flash command sequence function to execute the command */
    ret = pFlashCommandSequence(pSSDConfig);

    return(ret);
}
/*
 * brief Flash erase sector.
 *
 * This API erases one or more sectors in P-Flash or  D-Flash memory.
 * This  API  always  returns  FTFx_OK  if  size  provided  by  the user  is
 * zero  regardless  of  the  input validation.
 *
 * param pSSDConfig:    The SSD configuration structure pointer.
 * param dest:          Address in the first sector to be erased.
 * param size:          Size to be erased in bytes. It is used to determine
 *                       number of sectors to be erased.
 * param pFlashCommandSequence :     Pointer to the Flash command sequence function.
 * return Successful completion (FTFx_OK)
 * return Error value  (FTFx_ERR_MGSTAT0, FTFx_ERR_ACCERR, FTFx_ERR_PVIOL,FTFx_ERR_SIZE)
 */
flash_drv_status_t FlashEraseSector(const PFLASH_SSD_CONFIG pSSDConfig, \
                        uint32_t dest, \
                        uint32_t size, \
                        pFLASHCOMMANDSEQUENCE pFlashCommandSequence)
{

    flash_drv_status_t ret = FTFx_OK;     /* return code variable */
    uint32_t sectorSize;        /* size of one sector */
    uint32_t temp;              /* temporary variable */


#if FSL_FEATURE_FLASH_HAS_FLEX_NVM
    temp = pSSDConfig->DFlashBase;
    if((dest >= temp) && (dest < (temp + pSSDConfig->DFlashSize)))
    {
        dest = dest - temp + 0x800000U;
        sectorSize = (uint32_t)FSL_FEATURE_FLASH_FLEX_NVM_BLOCK_SECTOR_SIZE;
    }
    else
#endif
    {
        temp = pSSDConfig->PFlashBase;
        if((dest >= temp) && (dest < (temp + pSSDConfig->PFlashSize)))
        {
            dest -= temp;
            sectorSize = (uint32_t)FSL_FEATURE_FLASH_PFLASH_BLOCK_SECTOR_SIZE;
        }
        else
        {
            ret = FTFx_ERR_ACCERR;
            size = 0x0U;
        }
    }

    /* check if the size is sector alignment or not */
    if((size & (sectorSize - 0x01U)) != 0x0U)
    {
        /* return an error code FTFx_ERR_SIZE */
        ret = FTFx_ERR_SIZE;
    }

    while((size > 0x0U) && (FTFx_OK == ret))
    {
        /* clear RDCOLERR & ACCERR & FPVIOL flag in flash status register. Write 1 to clear*/
        CLEAR_FTFx_FSTAT_ERROR_BITS;
        /* passing parameter to the command */
        FTFx_FCCOB0 = FTFx_ERASE_SECTOR;
        FTFx_FCCOB1 = GET_BIT_16_23(dest);
        FTFx_FCCOB2 = GET_BIT_8_15(dest);
        FTFx_FCCOB3 = GET_BIT_0_7(dest);

        /* calling flash command sequence function to execute the command */
        ret = pFlashCommandSequence(pSSDConfig);

        /* update size and destination address */
        size -= sectorSize;
        dest += sectorSize;

    }
    return(ret);
}
/*
 * brief Flash verify sections.
 *
 * This API  checks  if a section of the P-Flash or the D-Flash memory
 * is erased  to  the specified read margin level.
 *
 * param pSSDConfig:    The SSD configuration structure pointer.
 * param dest:          Start address for the intended verify operation.
 * param number:        Number of alignment unit to be verified. Refer to
 *                       corresponding reference manual to get correct
 *                       information of alignment constrain.
 * param marginLevel:   Read Margin Choice as follows:
 *                       marginLevel = 0x0: use Normal read level
 *                       marginLevel = 0x1: use the User read
 *                       marginLevel = 0x2: use the Factory read
 * param pFlashCommandSequence :     Pointer to the Flash command sequence function.
 * return Successful completion (FTFx_OK)
 * return Error value  (FTFx_ERR_MGSTAT0, FTFx_ERR_ACCERR)
 */

flash_drv_status_t FlashVerifySection(const PFLASH_SSD_CONFIG pSSDConfig, \
                          uint32_t dest, \
                          uint16_t number, \
                          uint8_t marginLevel, \
                          pFLASHCOMMANDSEQUENCE pFlashCommandSequence)
{
    flash_drv_status_t ret = FTFx_OK;      /* return code variable */
    uint32_t temp;

    /* check if the destination is aligned or not */
#if FSL_FEATURE_FLASH_HAS_FLEX_NVM
    temp = pSSDConfig->DFlashBase;
    if((dest >= temp) && (dest < (temp + pSSDConfig->DFlashSize)))
    {
        dest = dest - temp + 0x800000U;
    }
    else
#endif
    {
        temp = pSSDConfig->PFlashBase;
        if((dest >= temp) && (dest < (temp + pSSDConfig->PFlashSize)))
        {
            dest -= temp;
        }
        else
        {
            ret = FTFx_ERR_ACCERR;
        }
    }
    if(FTFx_OK == ret)
    {
       /* clear RDCOLERR & ACCERR & FPVIOL flag in flash status register. Write 1 to clear*/
        CLEAR_FTFx_FSTAT_ERROR_BITS;
        /* passing parameter to the command */
        FTFx_FCCOB0 = FTFx_VERIFY_SECTION;
        FTFx_FCCOB1 = GET_BIT_16_23(dest);
        FTFx_FCCOB2 = GET_BIT_8_15(dest);
        FTFx_FCCOB3 = GET_BIT_0_7(dest);
        FTFx_FCCOB4 = GET_BIT_8_15(number);
        FTFx_FCCOB5 = GET_BIT_0_7(number);
        FTFx_FCCOB6 = marginLevel;

        /* calling flash command sequence function to execute the command */
        ret = pFlashCommandSequence(pSSDConfig);
    }
    return(ret);
}
/*
 * brief Flash erase suspend.
 *
 * This API is used to suspend a current operation of Flash erase sector command.
 * This function must be located in RAM memory or different Flash blocks which are
 * targeted for writing to avoid the RWW error.
 *
 * param
 * return
 */
void FlashEraseSuspend(void)
{
    if((FTFx_FSTAT & FTFx_FSTAT_CCIF_MASK) == 0x0U)
    {
        FTFx_FCNFG |= FTFx_FCNFG_ERSSUSP_MASK;


        while((FTFx_FSTAT & FTFx_FSTAT_CCIF_MASK) == 0x0U)
        {
            /* wait till CCIF bit is set */
        }
    }
}
/*
 * brief Flash erase resume.
 *
 * This API is used to resume a previous suspended operation of Flash erase sector command
 * This function must be located in RAM memory or different Flash blocks which are targeted
 * for writing to avoid RWW error.
 *
 * param
 * return
 */
void FlashEraseResume(void)

{
    uint16_t i;         /* counter variable */
    i = 0x0U;

    /* check ERSSUSP bit of the flash configuration register */
    if((FTFx_FCNFG & FTFx_FCNFG_ERSSUSP_MASK) == FTFx_FCNFG_ERSSUSP_MASK)
    {
        /* clear CCIF to launch command */
        FTFx_FSTAT |= FTFx_FSTAT_CCIF_MASK;
        /* wait for completion of this command */
        while((0x0U == (FTFx_FSTAT & FTFx_FSTAT_CCIF_MASK)) || (i < RESUME_WAIT_CNT))
        {
            i++;
        }
    }
}
/*
 * brief Flash read once.
 *
 * This API is used to read out a reserved 64 byte field located in the P-Flash IFR via given number
 * of record. See the corresponding reference manual to get the correct value of this number.
 *
 * param pSSDConfig:    The SSD configuration structure pointer.
 * param recordIndex:   The record index will be read. It can be from 0x0
 *                      to 0x7 or from 0x0 to 0xF according to specific derivative.
 * param pDataArray:    Pointer to the array to return the data read by the read once command.
 * param pFlashCommandSequence :     Pointer to the Flash command sequence function.
 * return Successful completion (FTFx_OK)
 * return Error value  (FTFx_ERR_ACCERR)
 */
flash_drv_status_t FlashReadOnce(const PFLASH_SSD_CONFIG pSSDConfig, \
                    uint8_t recordIndex,\
                    uint8_t* pDataArray, \
                    pFLASHCOMMANDSEQUENCE pFlashCommandSequence)
{
    uint8_t i;
    flash_drv_status_t ret;       /* return code variable */
    uint32_t temp;      /* temporary variable */
   /* clear RDCOLERR & ACCERR & FPVIOL flag in flash status register. Write 1 to clear*/
    CLEAR_FTFx_FSTAT_ERROR_BITS;
    /* passing parameter to the command */
    FTFx_FCCOB0 = FTFx_READ_ONCE;
    FTFx_FCCOB1 = recordIndex;

    /* calling flash command sequence function to execute the command */
    ret = pFlashCommandSequence(pSSDConfig);
    /* checking for the success of command execution */
    if(FTFx_OK == ret)
    {
        /* Read the data from the FCCOB registers into the pDataArray */
        for (i = 0x0U; i < FSL_FEATURE_FLASH_PFLASH_BLOCK_WRITE_UNIT_SIZE; i ++)
        {
            temp = FTFx_BASE + i + 0x08U;
            pDataArray[i] = *(uint8_t *)(temp);
        }
    }
    return(ret);
}
/*
 * brief Flash program once.
 *
 * This API  is  used  to  program  to  a  reserved  64  byte  field  located  in  the
 * P-Flash  IFR  via  given number of record. See the corresponding reference manual
 * to get correct value of this number.
 *
 * param pSSDConfig:    The SSD configuration structure pointer.
 * param recordIndex:   The record index will be read. It can be from 0x0
 *                       to 0x7 or from 0x0 to 0xF according to specific derivative.
 * param pDataArray:    Pointer to the array from which data will be
 *                       taken for program once command.
 * param pFlashCommandSequence :     Pointer to the Flash command sequence function.
 * return Successful completion (FTFx_OK)
 * return Error value  (FTFx_ERR_ACCERR,FTFx_ERR_MGSTAT0)
 */
flash_drv_status_t FlashProgramOnce(const PFLASH_SSD_CONFIG pSSDConfig, \
                        uint8_t recordIndex,\
                        const uint8_t* pDataArray, \
                        pFLASHCOMMANDSEQUENCE pFlashCommandSequence)
{

    uint8_t i;
    flash_drv_status_t ret;         /* return code variable */
    uint32_t temp;        /* temporary variable */

    /* clear RDCOLERR & ACCERR & FPVIOL flag in flash status register. Write 1 to clear*/
    CLEAR_FTFx_FSTAT_ERROR_BITS;
    /* passing parameter to the command */
    FTFx_FCCOB0 = FTFx_PROGRAM_ONCE;
    FTFx_FCCOB1 = recordIndex;

    for (i = 0x0U; i < FSL_FEATURE_FLASH_PFLASH_BLOCK_WRITE_UNIT_SIZE; i ++)
    {
        temp = FTFx_BASE + i + 0x08U;
        *(uint8_t *)(temp) = pDataArray[i];
    }
    /* calling flash command sequence function to execute the command */
    ret = pFlashCommandSequence(pSSDConfig);
    return(ret);
}
/*
 * brief Flash read resource.
 *
 * This API is used to read data from special purpose memory in Flash memory module
 * including P-Flash IFR, swap IFR, D-Flash IFR space and version ID.
 *
 * param pSSDConfig:    The SSD configuration structure pointer.
 * param dest:          Start address for the intended read operation.
 * param pDataArray:    Pointer to the data returned by the read resource command.
 * param resourceSelectCode:    Read resource select code:
 *                               0 : Flash IFR
 *                               1: Version ID
 * param pFlashCommandSequence :     Pointer to the Flash command sequence function.
 * return Successful completion (FTFx_OK)
 * return Error value  (FTFx_ERR_ACCERR)
 */
flash_drv_status_t FlashReadResource(const PFLASH_SSD_CONFIG pSSDConfig, \
                         uint32_t dest, \
                         uint8_t* pDataArray, \
                         uint8_t  resourceSelectCode, \
                         pFLASHCOMMANDSEQUENCE pFlashCommandSequence)
{
    uint8_t i;
    flash_drv_status_t ret = FTFx_OK;       /* return code variable */
    uint32_t temp;

    /* check if the destination is aligned or not */
#if FSL_FEATURE_FLASH_HAS_FLEX_NVM
    temp = pSSDConfig->DFlashBase;
    if((dest >= temp) && (dest < (temp + pSSDConfig->DFlashSize)))
    {
        dest = dest - temp + 0x800000U;
    }
    else
#endif
    {
        temp = pSSDConfig->PFlashBase;
        if((dest >= temp) && (dest < (temp + pSSDConfig->PFlashSize)))
        {
            dest -= temp;
        }
        else
        {
            ret = FTFx_ERR_ACCERR;
        }
    }
    if(ret == FTFx_OK)
    {
        /* clear RDCOLERR & ACCERR & FPVIOL flag in flash status register. Write 1 to clear */
        CLEAR_FTFx_FSTAT_ERROR_BITS;
        /* passing parameter to the command */
        FTFx_FCCOB0 = FTFx_READ_RESOURCE;
        FTFx_FCCOB1 = GET_BIT_16_23(dest);
        FTFx_FCCOB2 = GET_BIT_8_15(dest);
        FTFx_FCCOB3 = GET_BIT_0_7(dest);
        FTFx_RSRC_CODE_REG = resourceSelectCode;
        /* calling flash command sequence function to execute the command */
        ret = pFlashCommandSequence(pSSDConfig);

        if (FTFx_OK == ret)
        {
            /* Read the data from the FCCOB registers into the pDataArray */
            for (i = 0x0U; i < FSL_FEATURE_FLASH_PFLASH_BLOCK_WRITE_UNIT_SIZE; i ++)
            {
                temp = FTFx_BASE + i + 0x08U;
                pDataArray[i] = *(uint8_t*)temp;
            }
        }
    }
    return(ret);
}
/*
 * brief Flash program
 *
 * This  API  is  used  to  program  4  consecutive  bytes  (for  program  long
 * word  command)  and  8 consecutive bytes (for program phrase command) on
 * P-Flash or D-Flash block. This  API  always  returns  FTFx_OK  if  size
 * provided  by  user  is  zero  regardless  of  the  input validation
 *
 * param pSSDConfig:    The SSD configuration structure pointer.
 * param dest:          Start address for the intended program operation.
 * param size:          Size in byte to be programmed
 * param pData:         Pointer of source address from which data has to
 *                       be taken for program operation.
 * param pFlashCommandSequence :     Pointer to the Flash command sequence function.
 * return Successful completion (FTFx_OK)
 * return Error value  (FTFx_ERR_ACCERR, FTFx_ERR_PVIOL, FTFx_ERR_SIZE, FTFx_ERR_MGSTAT0)
 */
flash_drv_status_t FlashProgram(const PFLASH_SSD_CONFIG pSSDConfig, \
                    uint32_t dest, \
                    uint32_t size, \
                    uint8_t* pData, \
                    pFLASHCOMMANDSEQUENCE pFlashCommandSequence)
{
    flash_drv_status_t ret = FTFx_OK;      /* return code variable */
    uint8_t i;
    uint32_t temp;

    if ((size & ((uint32_t)FSL_FEATURE_FLASH_PFLASH_BLOCK_WRITE_UNIT_SIZE - 0x01U)) != 0x0U)
    {
       ret = FTFx_ERR_SIZE;
    }
    else
    {
#if FSL_FEATURE_FLASH_HAS_FLEX_NVM
        temp = pSSDConfig->DFlashBase;
        if((dest >= temp) && (dest < (temp + pSSDConfig->DFlashSize)))
        {
            dest = dest - temp + 0x800000U;
        }
        else
#endif
        {
            temp = pSSDConfig->PFlashBase;
            if((dest >= temp) && (dest < (temp + pSSDConfig->PFlashSize)))
            {
                dest -= temp;
            }
            else
            {
                ret = FTFx_ERR_ACCERR;
            }
        }
        while((size > 0x0U) && (FTFx_OK == ret))
        {
            /* clear RDCOLERR & ACCERR & FPVIOL flag in flash status register. Write 1 to clear*/
             CLEAR_FTFx_FSTAT_ERROR_BITS;
            /* passing parameter to the command */
#if (FSL_FEATURE_FLASH_FLEX_NVM_BLOCK_WRITE_UNIT_SIZE == FTFx_PHRASE_SIZE)
            FTFx_FCCOB0 = FTFx_PROGRAM_PHRASE;
#else
            FTFx_FCCOB0 = FTFx_PROGRAM_LONGWORD;
#endif
            FTFx_FCCOB1 = GET_BIT_16_23(dest);
            FTFx_FCCOB2 = GET_BIT_8_15(dest);
            FTFx_FCCOB3 = GET_BIT_0_7(dest);

            for (i = 0x0U; i <  FSL_FEATURE_FLASH_PFLASH_BLOCK_WRITE_UNIT_SIZE; i++)
            {
                temp = FTFx_BASE + i + 0x08U;
                *(uint8_t *)(temp) = pData[i];
            }

            /* calling flash command sequence function to execute the command */
            ret = pFlashCommandSequence(pSSDConfig);

            /* update destination address for next iteration */
            dest += FSL_FEATURE_FLASH_PFLASH_BLOCK_WRITE_UNIT_SIZE;
            /* update size for next iteration */
            size -= FSL_FEATURE_FLASH_PFLASH_BLOCK_WRITE_UNIT_SIZE;
            /* increment the source address by 1 */
            pData += FSL_FEATURE_FLASH_PFLASH_BLOCK_WRITE_UNIT_SIZE;
        }
    }
    return(ret);
}

/*
 * brief Flash program check
 *
 * This API tests a previously programmed P-Flash or D-Flash long word
 * to see if it reads correctly at the specified margin level. This
 * API always returns FTFx_OK if size provided by user is zero
 * regardless  of  the  input validation
 *
 * param pSSDConfig:    The SSD configuration structure pointer.
 * param dest:          Start address for the intended program check operation.
 * param size:          Size in byte to check accuracy of program operation
 * param pExpectedData: The pointer to the expected data.
 * param pFailAddr:     Returned the first aligned failing address.
 * param marginLevel:   Read margin choice as follows:
 *                       marginLevel = 0x1: read at User margin 1/0 level.
 *                       marginLevel = 0x2: read at Factory margin 1/0 level.
 * param pFlashCommandSequence :     Pointer to the Flash command sequence function.
 * return Successful completion (FTFx_OK)
 * return Error value  (FTFx_ERR_ACCERR, FTFx_ERR_MGSTAT0)
 */
flash_drv_status_t FlashProgramCheck(const PFLASH_SSD_CONFIG pSSDConfig, \
                                                uint32_t  dest, \
                                                uint32_t  size, \
                                                uint8_t*  pExpectedData, \
                                                uint32_t* pFailAddr, \
                                                uint8_t   marginLevel, \
                                                pFLASHCOMMANDSEQUENCE pFlashCommandSequence)
{

    flash_drv_status_t ret;      /* return code variable */
    uint32_t offsetAddr ; /* offset address to convert to internal memory address */
    uint32_t temp;        /* temporary variable */
    uint8_t i;
    if ((size & ((uint32_t)FSL_FEATURE_FLASH_PFLASH_CHECK_CMD_ADDRESS_ALIGMENT - 0x01U)) != 0x0U)
    {
        ret = FTFx_ERR_SIZE;

    }
    else
    {
        /* check if the destination is aligned or not */
#if FSL_FEATURE_FLASH_HAS_FLEX_NVM
        offsetAddr = pSSDConfig->DFlashBase;
        if((dest >= offsetAddr) && (dest < (offsetAddr + pSSDConfig->DFlashSize)))
        {
            dest = dest - offsetAddr + 0x800000U;
        }
        else
#endif
        {
            offsetAddr = pSSDConfig->PFlashBase;
            if((dest >= offsetAddr) && (dest < (offsetAddr + pSSDConfig->PFlashSize)))
            {
                dest -= offsetAddr;
            }
            else
            {
                ret = FTFx_ERR_ACCERR;
                size = 0x0U;
            }
        }
        while (size > (uint32_t)0x0U)
        {
            /* clear RDCOLERR & ACCERR & FPVIOL flag in flash status register. Write 1 to clear*/
            CLEAR_FTFx_FSTAT_ERROR_BITS;
            /* passing parameter to the command */
            FTFx_FCCOB0 = FTFx_PROGRAM_CHECK;
            FTFx_FCCOB1 = GET_BIT_16_23(dest);
            FTFx_FCCOB2 = GET_BIT_8_15(dest);
            FTFx_FCCOB3 = GET_BIT_0_7(dest);
            FTFx_FCCOB4 = marginLevel;

            for (i = 0x0U; i < FSL_FEATURE_FLASH_PFLASH_CHECK_CMD_ADDRESS_ALIGMENT; i++)
            {
                temp = FTFx_BASE + i + 0x0CU;
                *(uint8_t *)(temp) = pExpectedData[i];
            }
            /* calling flash command sequence function to execute the command */
            ret = pFlashCommandSequence(pSSDConfig);

            /* checking for the success of command execution */
            if(FTFx_OK != ret)
            {
#if FSL_FEATURE_FLASH_HAS_FLEX_NVM
                if(dest >= 0x800000U)
                {
                    *pFailAddr = dest + offsetAddr - 0x800000U;
                }
                else
#endif
                {
                    *pFailAddr = dest + offsetAddr;
                }
                 size = FSL_FEATURE_FLASH_PFLASH_CHECK_CMD_ADDRESS_ALIGMENT;
            }
            size -= FSL_FEATURE_FLASH_PFLASH_CHECK_CMD_ADDRESS_ALIGMENT;
            pExpectedData += FSL_FEATURE_FLASH_PFLASH_CHECK_CMD_ADDRESS_ALIGMENT;
            dest += FSL_FEATURE_FLASH_PFLASH_CHECK_CMD_ADDRESS_ALIGMENT;
        }
    }
    return(ret);
}

/*
 * brief Calculates check sum.
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
 * param pSSDConfig:    The SSD configuration structure pointer.
 * param dest:          Start address of the Flash range to be summed
 * param size:          Size in byte of the Flash range to be summed
 * param pSum:          To return the sum value
 * return Successful completion (FTFx_OK)
 * return Error value  (FTFx_ERR_RANGE)
 */
flash_drv_status_t FlashCheckSum(const PFLASH_SSD_CONFIG pSSDConfig, \
                                uint32_t dest, \
                                uint32_t size, \
                                uint32_t* pSum)
{
    uint32_t counter;          /* Counter for callback operation */
    uint32_t data;             /* Data read from Flash address */
    flash_drv_status_t ret = FTFx_OK;       /* Return code variable */
    uint32_t endAddress;       /* P Flash end address */

    counter = 0x0U;
    /* calculating Flash end address */
    endAddress = dest + size;

    /* check for valid range of the target addresses */
    if ((dest < pSSDConfig->PFlashBase) ||
        (endAddress > (pSSDConfig->PFlashBase + pSSDConfig->PFlashSize)))
    {
#if FSL_FEATURE_FLASH_HAS_FLEX_NVM
        if ((dest < pSSDConfig->DFlashBase) ||
        (endAddress > (pSSDConfig->DFlashBase + pSSDConfig->DFlashSize)))
        {
#endif
            ret = FTFx_ERR_RANGE;
            size = 0x0U;
#if FSL_FEATURE_FLASH_HAS_FLEX_NVM
        }

#endif /* End of if FSL_FEATURE_FLASH_HAS_FLEX_NVM */
    }
    *pSum = 0x0U;
    /* doing sum operation */
    while(size > 0x0U)
    {
        data = *(uint8_t *)(dest);
        *pSum += data;
        dest += 0x01U;
        size -= 0x01U;

        ++counter;
        /* Check if need to serve callback function */
        if(counter >= FLASH_CALLBACK_CS)
        {
            /* serve callback function if counter reaches limitation */
            if(NULL_CALLBACK != pSSDConfig->CallBack)
            {
                pSSDConfig->CallBack();
            }
            /* Reset counter */
            counter = 0x0U;
        }
    }
    return(ret);
}

#if FSL_FEATURE_FLASH_HAS_PROGRAM_SECTION_CMD
/*
 * brief Flash program section
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
 * param pSSDConfig:    The SSD configuration structure pointer.
 * param dest:          Start address for the intended program operation.
 * param number:          Number of alignment unit to be programmed. Refer to associate
 *                       reference manual to get correct value of this alignment constrain.
 * param pFlashCommandSequence :  Pointer to the Flash command sequence function.
 * return Successful completion (FTFx_OK)
 * return Error value  (FTFx_ERR_ACCERR, FTFx_ERR_PVIOL, FTFx_ERR_MGSTAT0, FTFx_ERR_RAMRDY)
 */
flash_drv_status_t FlashProgramSection(const PFLASH_SSD_CONFIG pSSDConfig, \
                                                uint32_t dest, \
                                                uint16_t number, \
                                                pFLASHCOMMANDSEQUENCE pFlashCommandSequence)
{

    flash_drv_status_t ret = FTFx_OK;      /* return code variable */
    uint32_t temp;

    /* check RAMRDY bit of the flash configuration register */
    if(0x0u == (FTFx_FCNFG & FTFx_FCNFG_RAMRDY_MASK))
    {
        /* return an error code FTFx_ERR_RAMRDY */
        ret = FTFx_ERR_RAMRDY;
    }
    else
    {
#if FSL_FEATURE_FLASH_HAS_FLEX_NVM
        temp = pSSDConfig->DFlashBase;
        if((dest >= temp) && (dest < (temp + pSSDConfig->DFlashSize)))
        {
            dest = dest - temp + 0x800000U;
        }
        else
#endif
        {
            temp = pSSDConfig->PFlashBase;
            if((dest >= temp) && (dest < (temp + pSSDConfig->PFlashSize)))
            {
                dest -= temp;
            }
            else
            {
                ret = FTFx_ERR_ACCERR;
            }
        }

        if(ret == FTFx_OK)
        {
            CLEAR_FTFx_FSTAT_ERROR_BITS;
            /* passing parameter to command */
            FTFx_FCCOB0 = FTFx_PROGRAM_SECTION;
            FTFx_FCCOB1 = GET_BIT_16_23(dest);
            FTFx_FCCOB2 = GET_BIT_8_15(dest);
            FTFx_FCCOB3 = GET_BIT_0_7(dest);
            FTFx_FCCOB4 = GET_BIT_8_15(number);
            FTFx_FCCOB5 = GET_BIT_0_7(number);

            /* calling flash command sequence function to execute the command */
            ret = pFlashCommandSequence(pSSDConfig);
        }
    }
    return(ret);
}

#endif

#if FSL_FEATURE_FLASH_HAS_ERASE_FLASH_BLOCK_CMD
/*
 * brief Flash erase block
 *
 * This API  erases all addresses in an individual P-Flash or D-Flash block.
 * For  the derivatives  including multiply  logical P-Flash or D-Flash blocks,
 * this API   erases a single block in a single call.
 *
 * param pSSDConfig:    The SSD configuration structure pointer.
 * param dest:          Start address for the intended erase operation.
 * param pFlashCommandSequence :  Pointer to the Flash command sequence function.
 * return Successful completion (FTFx_OK)
 * return Error value  (FTFx_ERR_ACCERR, FTFx_ERR_PVIOL, FTFx_ERR_MGSTAT0)
 */
flash_drv_status_t FlashEraseBlock (const PFLASH_SSD_CONFIG pSSDConfig, \
                        uint32_t dest, \
                        pFLASHCOMMANDSEQUENCE pFlashCommandSequence)
{

    flash_drv_status_t ret = FTFx_OK;       /* return code variable */
    uint32_t temp;      /* temporary variable */


    /* check if the destination is aligned or not */
#if FSL_FEATURE_FLASH_HAS_FLEX_NVM
    temp = pSSDConfig->DFlashBase;
    if((dest >= temp) && (dest < (temp + pSSDConfig->DFlashSize)))
    {
        dest = dest - temp + 0x800000U;
    }
    else
#endif
    {
        temp = pSSDConfig->PFlashBase;
        if((dest >= temp) && (dest < (temp + pSSDConfig->PFlashSize)))
        {
            dest -= temp;
        }
        else
        {
            ret = FTFx_ERR_ACCERR;
        }
    }

    if(FTFx_OK == ret)
    {
       /* clear RDCOLERR & ACCERR & FPVIOL flag in flash status register. Write 1 to clear*/
        CLEAR_FTFx_FSTAT_ERROR_BITS;
        /* passing parameter to the command */
        FTFx_FCCOB0 = FTFx_ERASE_BLOCK;
        FTFx_FCCOB1 = GET_BIT_16_23(dest);
        FTFx_FCCOB2 = GET_BIT_8_15(dest);
        FTFx_FCCOB3 = GET_BIT_0_7(dest);

        /* calling flash command sequence function to execute the command */
        ret = pFlashCommandSequence(pSSDConfig);
    }
    return(ret);
}
#endif
#ifdef FSL_FEATURE_FLASH_HAS_READ_1S_BLOCK_CMD
/*
 * brief Flash verify block
 *
 * This API  checks to see  if an entire P-Flash or D-Flash block has been
 * erased to the specified margin level
 * For  the derivatives  including multiply  logical P-Flash or D-Flash blocks,
 * this API   erases a single block in a single call.
 *
 * param pSSDConfig:    The SSD configuration structure pointer.
 * param dest:          Start address for the intended verify operation.
 * param marginLevel:   Read Margin Choice as follows:
 *                       marginLevel = 0x0: use Normal read level
 *                       marginLevel = 0x1: use the User read
 *                       marginLevel = 0x2: use the Factory read
 * param pFlashCommandSequence :  Pointer to the Flash command sequence function.
 * return Successful completion (FTFx_OK)
 * return Error value  (FTFx_ERR_ACCERR, FTFx_ERR_MGSTAT0)
 */
flash_drv_status_t FlashVerifyBlock(const PFLASH_SSD_CONFIG pSSDConfig, \
                                   uint32_t dest, \
                                   uint8_t marginLevel, \
                                   pFLASHCOMMANDSEQUENCE pFlashCommandSequence)
{
    flash_drv_status_t ret = FTFx_OK;      /* return code variable */
    uint32_t temp;

    /* check if the destination is aligned or not */
#if FSL_FEATURE_FLASH_HAS_FLEX_NVM
    temp = pSSDConfig->DFlashBase;
    if((dest >= temp) && (dest < (temp + pSSDConfig->DFlashSize)))
    {
        dest = dest - temp + 0x800000U;
    }
    else
#endif
    {
        temp = pSSDConfig->PFlashBase;
        if((dest >= temp) && (dest < (temp + pSSDConfig->PFlashSize)))
        {
            dest -= temp;
        }
        else
        {
        ret = FTFx_ERR_ACCERR;
        }
    }
    if(FTFx_OK == ret)
    {
        /* clear RDCOLERR & ACCERR & FPVIOL flag in flash status register. Write 1 to clear*/
        CLEAR_FTFx_FSTAT_ERROR_BITS;
        /* passing parameter to the command */
        FTFx_FCCOB0 = FTFx_VERIFY_BLOCK;
        FTFx_FCCOB1 = GET_BIT_16_23(dest);
        FTFx_FCCOB2 = GET_BIT_8_15(dest);
        FTFx_FCCOB3 = GET_BIT_0_7(dest);
        FTFx_FCCOB4 = marginLevel;
        /* calling flash command sequence function to execute the command */
        ret = pFlashCommandSequence(pSSDConfig);
    }
    return(ret);
}

#endif

#if FSL_FEATURE_FLASH_HAS_FLEX_NVM
/*
 * brief EERAM get protection
 *
 * This  API  retrieves  which  EEPROM  sections  of  FlexRAM  are  protected
 * against  program  and erase operations. Considering  the  time consumption
 * for getting protection is very low and even can be ignored, it is not necessary
 * to utilize the Callback function to support the time-critical events
 *
 * param protectStatus: To return the current value of the EEPROM
 *                       Protection Register. Each bit is corresponding to
 *                       protection status of 1/8 of the total EEPROM
 *                       use. The least significant bit is corresponding to
 *                       the lowest address area of EEPROM. The most
 *                       significant bit is corresponding to the highest
 *                       address area of EEPROM and so on. There are
 *                       two possible cases as below:
 *                       -  0: this area is protected.
 *                       -  1: this area is unprotected.
 * return Successful completion (FTFx_OK)
 * return Error value  (FTFx_ERR_NOEEE)
 */
flash_drv_status_t EERAMGetProtection(uint8_t* protectStatus)
{
    flash_drv_status_t ret = FTFx_OK;      /* return code variable */
    /* Check if EERAM is set for EEE */
    if((FTFx_FCNFG & FTFx_FCNFG_EEERDY_MASK) == FTFx_FCNFG_EEERDY_MASK)
    {
        *protectStatus = FTFx_FEPROT;
    }
    else
    {
        ret = FTFx_ERR_NOEEE;
    }
    return(ret);
}
/*
 * brief EERAM set protection
 *
 * This API sets protection to the intended protection status for EEPROM us
 * area of FlexRam. This is subject to a protection transition restriction.
 * If there is a setting violation, it returns failed information and
 * the current protection status won’t be changed.
 *
 * param protectStatus: The intended protection status value should be
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
 * return Successful completion (FTFx_OK)
 * return Error value  (FTFx_ERR_NOEEE,FTFx_ERR_CHANGEPROT)
 */
flash_drv_status_t EERAMSetProtection(uint8_t protectStatus)
{
    flash_drv_status_t ret = FTFx_OK;       /* return code variable */
    /* Check if EERAM is set for EEE */
     if(0x0U == (FTFx_FCNFG & FTFx_FCNFG_EEERDY_MASK))
    {
        /* EERAM is not set for EEE */
        ret = FTFx_ERR_NOEEE;
    }
    else
    {

        FTFx_FEPROT = protectStatus;
        if ( protectStatus != FTFx_FEPROT)
        {
            ret = FTFx_ERR_CHANGEPROT;
        }
        else
        {
            /* do nothing */
        }
    }
    return(ret);
}
/*
 * brief Flash Set EEEEnable
 *
 * This function is used to change the function of the FlexRAM. When not
 * partitioned for EEPROM backup, the FlexRam is typically used as traditional
 * RAM. Otherwise, the FlexRam is typically used to store EEPROM data and user
 * can use this API to change its functionality according to his application requirement.
 * For example, after partitioning to have EEPROM backup, FlexRAM  is used  for EEPROM
 * use accordingly. And this API is used to set FlexRAM is available for
 * traditional RAM for FlashProgramSection() use.
 *
 * param pSSDConfig:               The SSD configuration structure pointer.
 * param EEEEnable:                FlexRam function control code. It can be:
 *                                  -  0xFF: make FlexRam available for RAM.
 *                                  -  0x00: make FlexRam available for EEPROM.
 * param pFlashCommandSequence:    Pointer to the Flash command sequence function.
 * return Successful completion (FTFx_OK)
 * return Error value  (FTFx_ERR_ACCERR)
 */
flash_drv_status_t SetEEEEnable(const PFLASH_SSD_CONFIG pSSDConfig, uint8_t EEEEnable, pFLASHCOMMANDSEQUENCE pFlashCommandSequence)
{

    flash_drv_status_t ret;      /* return code variable */


    /* clear RDCOLERR & ACCERR & FPVIOL flag in flash status register. Write 1 to clear*/
    CLEAR_FTFx_FSTAT_ERROR_BITS;
    /* passing parameter to the command */
    FTFx_FCCOB0 = FTFx_SET_EERAM;
    FTFx_FCCOB1 = EEEEnable;
    /* calling flash command sequence function to execute the command */
    ret = pFlashCommandSequence(pSSDConfig);
    return(ret);
}
/*
 * brief EEPROM Emulator Write
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
 * param pSSDConfig:    The SSD configuration structure pointer.
 * param dest:          Start address for the intended write operation.
 * param size:          Size in byte to be written.
 * param pData:         Pointer to source address from which data
 *                       has to be taken for writing operation.
 * return Successful completion (FTFx_OK)
 * return Error value  (FTFx_ERR_RANGE, FTFx_ERR_NOEEE, FTFx_ERR_PVIOL)
 */
static flash_drv_status_t WaitEEWriteToFinish(uint32_t dest,\
                             const uint8_t* pData, uint8_t step)
{
    flash_drv_status_t ret;           /* return code variable */
    uint32_t temp;          /* temporary variable */

    if (0x01U == step)
    {
        *(uint8_t*)(dest) =  *pData;
    }
    if (0x02U == step)
    {
        temp = (uint32_t)(*(pData + 1)) << 8;
        temp |= (uint32_t)(*pData);

        *(volatile uint16_t *)(dest) =  (uint16_t)temp;
    }
    if (0x04U == step)
    {
        temp =  (uint32_t)(*(pData + 3)) << 24;
        temp |= (uint32_t)(*(pData + 2)) << 16;
        temp |= (uint32_t)(*(pData + 1)) << 8;
        temp |= (uint32_t)(*pData);
        *(volatile uint32_t *)(dest) = temp;
    }

    while(0x0u == (FTFx_FCNFG & FTFx_FCNFG_EEERDY_MASK))
    {
       /* wait till EEERDY bit is set */
    }
    /* Check for protection violation error */
    ret = (flash_drv_status_t)(FTFx_FSTAT & (FTFx_FSTAT_MGSTAT0_MASK|FTFx_FSTAT_FPVIOL_MASK \
                    |FTFx_FSTAT_ACCERR_MASK|FTFx_FSTAT_RDCOLERR_MASK));

    return ret;
}

flash_drv_status_t EEEWrite(const PFLASH_SSD_CONFIG pSSDConfig, \
                           uint32_t dest, \
                           uint32_t size, \
                           uint8_t* pData)
{
    flash_drv_status_t ret = FTFx_OK;           /* Return code variable */
    uint8_t i;
    /* Check if EEE is enabled */
    if((FTFx_FCNFG & FTFx_FCNFG_EEERDY_MASK) == FTFx_FCNFG_EEERDY_MASK)
    {
        /* check range */
       if((dest < pSSDConfig->EERAMBase) || \
          ((dest + size) > (pSSDConfig->EERAMBase + pSSDConfig->EEESize)))
        {
            ret = FTFx_ERR_RANGE;
        }

        while ((size > 0x0U) && (ret == FTFx_OK))
        {

            /* dest is 32bit-aligned and size is not less than 4 */
            if (((uint32_t)0x0u == (dest & (uint32_t)0x03U)) && (size >= 0x04U))
            {
                i = 0x04U;
            }
            else  if (((uint32_t)0x0u == (dest & (uint32_t)0x1U)) && (size >= 0x02U))
            {
                i = 0x02U;
            }
            else
            {
                i = 0x01U;

            }
            ret = WaitEEWriteToFinish(dest, pData, i);
            dest += i;
            size -= i;
            pData += i;

        }
    }
    else
    {
        ret = FTFx_ERR_NOEEE;
    }
    return(ret);
}
/*
 * brief Flash D/E-Flash Partition.
 *
 * This API prepares the FlexNVM block for use as D-Flash, EEPROM backup, or a combination
 * of both and initializes the FlexRAM.
 *
 * The single partition choice should be used through entire life time of a given
 * application to guarantee the Flash endurance and data retention of Flash module.
 *
 * param   pSSDConfig                The SSD configuration structure pointer
 * param   EEEDataSizeCode           EEPROM Data Size Code
 * param   DEPartitionCode           FlexNVM Partition Code
 * param   pFlashCommandSequence     Pointer to the Flash command sequence function.
 *
 * return  Successful completion(FTFx_OK)
 * return  Error value(FTFx_ERR_ACCERR, FTFx_ERR_MGSTAT0)
 */
flash_drv_status_t DEFlashPartition(const PFLASH_SSD_CONFIG pSSDConfig, \
                        uint8_t EEEDataSizeCode, \
                        uint8_t DEPartitionCode, \
                        pFLASHCOMMANDSEQUENCE pFlashCommandSequence)
{
    flash_drv_status_t ret;      /* return code variable */
    /* clear RDCOLERR & ACCERR & FPVIOL & MGSTAT0 flag in flash status register. Write 1 to clear*/
    CLEAR_FTFx_FSTAT_ERROR_BITS;
    /* passing parameter to the command */
    FTFx_FCCOB0 = FTFx_PROGRAM_PARTITION;
    FTFx_FCCOB4 = EEEDataSizeCode;
    FTFx_FCCOB5 = DEPartitionCode;
    /* calling flash command sequence function to execute the command */
    ret = pFlashCommandSequence(pSSDConfig);
    return(ret);
}
/*
 * brief D-Flash get protection.
 *
 * This API retrieves current P-Flash protection status. Considering the time consumption
 * for getting protection is very low and even can be ignored, it is not necessary to utilize
 * the Callback function to support the time-critical events.
 *
 * param   pSSDConfig                The SSD configuration structure pointer
 * param   protectStatus             To return the current value of the D-Flash Protection
 *                                    Register. Each bit is corresponding to protection status
 *                                    of 1/8 of the total D-Flash. The least significant bit is
 *                                    corresponding to the lowest address area of D-Flash. The
 *                                    most significant bit is corresponding to the highest address
 *                                    area of D-Flash and so on. There are two possible cases as below:
 *                                    - 0 : this area is protected.
 *                                    - 1 : this area is unprotected.
 *
 * return  Successful completion(FTFx_OK)
 * return  Error value(FTFx_ERR_EFLASHONLY)
 */
flash_drv_status_t DFlashGetProtection(const PFLASH_SSD_CONFIG pSSDConfig,uint8_t* protectStatus)
{
    flash_drv_status_t ret = FTFx_OK;
    /* Check if size of DFlash = 0 */
    if(pSSDConfig->DFlashSize == 0x0U)
    {
        ret = FTFx_ERR_EFLASHONLY;
    }
    else
    {
        *protectStatus = FTFx_FDPROT;
    }
    return(ret);
}

/*
 * brief D-Flash set protection.
 *
 * This API sets the D-Flash protection to the intended protection status. Setting D-Flash
 * protection status is subject to a protection transition restriction. If there is a setting
 * violation, it returns failed information and the current protection status won’t be changed.
 *
 * param   pSSDConfig                The SSD configuration structure pointer
 * param   protectStatus             The expected protect status user wants to set to D-Flash Protection
 *                                    Register. Each bit is corresponding to protection status
 *                                    of 1/8 of the total D-Flash. The least significant bit is
 *                                    corresponding to the lowest address area of D-Flash. The
 *                                    most significant bit is corresponding to the highest address
 *                                    area of D-Flash and so on. There are two possible cases as below:
 *                                    - 0 : this area is protected.
 *                                    - 1 : this area is unprotected.
 *
 * return  Successful completion(FTFx_OK)
 * return  Error value(FTFx_ERR_EFLASHONLY,FTFx_ERR_CHANGEPROT)
 */
flash_drv_status_t DFlashSetProtection(const PFLASH_SSD_CONFIG pSSDConfig, uint8_t protectStatus)
{
    flash_drv_status_t ret = FTFx_OK;       /* return code variable */

    /* Check if size of DFlash = 0 */
    if(pSSDConfig->DFlashSize == 0x0U)
    {
        ret = FTFx_ERR_EFLASHONLY;
    }
    else
    {
        FTFx_FDPROT = protectStatus;
        if ( protectStatus != FTFx_FDPROT)
        {
            ret = FTFx_ERR_CHANGEPROT;
        }
        else
        {
            /* do nothing */
        }
    }
    return(ret);
}
#endif /* End of FSL_FEATURE_FLASH_HAS_FLEX_NVM */

#ifdef FSL_FEATURE_FLASH_HAS_PFLASH_BLOCK_SWAP
/*
 * brief  Swaps between the two halves of the total logical P-Flash memory blocks in the memory map.
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
 * The swap indicator provided by the user must be within the lower half of P-Flash block but not in the
 * Flash configuration area. If P-Flash block has two logical blocks, the swap indicator must be
 * in P-Flash block 0. If the P-Flash block has four logical blocks, the swap indicator can be in block
 * 0 or block 1. It must not be in the Flash configuration field.
 * The user must use the same swap indicator for all swap control code except report swap status once
 * swap system has been initialized. To refresh swap system to un-initialization state,
 * use the FlashEraseAllBlock() to clean up the swap environment.
 *
 * param   pSSDConfig                The SSD configuration structure pointer
 * param   addr                      Address of swap indicator.
 * param   pSwapCallback             Callback to do specific task while the swapping is being performed
 * param   pFlashCommandSequence     Pointer to the Flash command sequence function.
 *
 * return  Successful completion(FTFx_OK)
 * return  Error value(FTFx_ERR_ACCERR,FTFx_ERR_MGSTAT0)
 */
flash_drv_status_t PFlashSwap(const PFLASH_SSD_CONFIG pSSDConfig, \
                  uint32_t addr, \
                  PFLASH_SWAP_CALLBACK pSwapCallback, \
                  pFLASHCOMMANDSEQUENCE pFlashCommandSequence)
{
    flash_drv_status_t ret = FTFx_OK;      /* Return code */
    uint8_t currentSwapMode , currentSwapBlockStatus , nextSwapBlockStatus;
    bool swapContinue;

    currentSwapMode = 0xFFU;
    currentSwapBlockStatus = 0xFFU;
    nextSwapBlockStatus = 0xFFU;
    swapContinue = false;

    /* Report current swap state */
    ret = PFlashSwapCtl(pSSDConfig,addr,FTFx_SWAP_REPORT_STATUS,&currentSwapMode, \
    &currentSwapBlockStatus, &nextSwapBlockStatus ,pFlashCommandSequence);

    if (FTFx_OK == ret)
    {
        if ((FTFx_SWAP_UNINIT == currentSwapMode) || (FTFx_SWAP_READY == currentSwapMode) || \
            (FTFx_SWAP_UPDATE == currentSwapMode))
        {
            /* If current swap mode is Uninitialized */
            if (FTFx_SWAP_UNINIT == currentSwapMode)
            {
                /* Initialize Swap to Initialized/READY state */
                ret = PFlashSwapCtl(pSSDConfig, addr, FTFx_SWAP_SET_INDICATOR_ADDR,&currentSwapMode, \
                &currentSwapBlockStatus, &nextSwapBlockStatus , pFlashCommandSequence);
            }
                /* If current swap mode is Initialized/Ready */
            else if (FTFx_SWAP_READY == currentSwapMode)
            {
                /* Initialize Swap to UPDATE state */
                ret = PFlashSwapCtl(pSSDConfig, addr, FTFx_SWAP_SET_IN_PREPARE,&currentSwapMode, \
                &currentSwapBlockStatus, &nextSwapBlockStatus , pFlashCommandSequence);
            }
            else
            {
                /* if (FTFx_SWAP_UPDATE == currentSwapMode) do nothing */
            }

            /* Check for the success of command execution */
            /* Report the current swap state to user via callback */
            if ((NULL_SWAP_CALLBACK != pSwapCallback) && (FTFx_OK == ret))
            {
                swapContinue = pSwapCallback(currentSwapMode);

                if (swapContinue == true)
                {
                    /* Report current swap state */
                    ret = PFlashSwapCtl(pSSDConfig,addr,FTFx_SWAP_REPORT_STATUS,&currentSwapMode, \
                                        &currentSwapBlockStatus, &nextSwapBlockStatus , pFlashCommandSequence);
                }
            }
        }else
        {
            /* do nothing */
        }
        if ((NULL_SWAP_CALLBACK == pSwapCallback)&&(FTFx_SWAP_UPDATE == currentSwapMode))
        {
            /* Erase indicator sector in non active block to proceed swap system to update-erased state */
            ret = FlashEraseSector(pSSDConfig, addr + (pSSDConfig->PFlashSize >> 1),(uint32_t)FSL_FEATURE_FLASH_PFLASH_BLOCK_SECTOR_SIZE , \
                                    pFlashCommandSequence);
            if (FTFx_OK == ret)
            {
                /* Now the swap state must be Update-Erased, so report current swap state */
                ret = PFlashSwapCtl(pSSDConfig,addr,FTFx_SWAP_REPORT_STATUS,&currentSwapMode, \
                                    &currentSwapBlockStatus, &nextSwapBlockStatus , pFlashCommandSequence);
            }
        }
        /* If current swap mode is Update or Update-Erased */
        if  (FTFx_SWAP_UPDATE_ERASED == currentSwapMode)
        {
            if (NULL_SWAP_CALLBACK == pSwapCallback)
            {
                swapContinue = true;
            }
            else
            {
                swapContinue = pSwapCallback(currentSwapMode);
            }

            if (swapContinue == true)
            {
                /* Progress Swap to COMPLETE State */
                ret = PFlashSwapCtl(pSSDConfig,addr,FTFx_SWAP_SET_IN_COMPLETE,&currentSwapMode, \
                &currentSwapBlockStatus, &nextSwapBlockStatus , pFlashCommandSequence);
            }
        }
    }
    return(ret);
}
/*
 * brief  Implements swap control command corresponding with the swap control code provided via the swapcmd parameter.
 *
 * param   pSSDConfig                The SSD configuration structure pointer
 * param   addr                      Address of swap indicator.
 * param   swapcmd                   Swap Control Code:
 *                                      0x01 - Initialize Swap System
 *                                      0x02 - Set Swap in Update State
 *                                      0x04 - Set Swap in Complete Stat
 *                                      0x08 - Report Swap Status
 * param   pCurrentSwapMode          Current Swap Mode:
 *                                      0x00 - Uninitialized
 *                                      0x01 - Ready
 *                                      0x02 - Update
 *                                      0x03 - Update-Erased
 *                                      0x04 - Complete
 * param   pCurrentSwapBlockStatus   Current Swap Block Status indicates which program Flash block
 *                                    is currently located at relative Flash address 0x0_0000
 *                                      0x00 - Program Flash block 0
 *                                      0x01 - Program Flash block 1
 * param   pNextSwapBlockStatus      Next Swap Block Status indicates which program Flash block
 *                                    is located at relative Flash address 0x0_0000 after the next reset.
 *                                      0x00 - Program Flash block 0
 *                                      0x01 - Program Flash block 1
 * param   pFlashCommandSequence     Pointer to the Flash command sequence function.
 *
 * return  Successful completion(FTFx_OK)
 * return  Error value(FTFx_ERR_ACCERR,FTFx_ERR_MGSTAT0)
 */
flash_drv_status_t PFlashSwapCtl(const PFLASH_SSD_CONFIG pSSDConfig,uint32_t addr, uint8_t swapcmd,uint8_t* pCurrentSwapMode, \
                        uint8_t* pCurrentSwapBlockStatus, \
                        uint8_t* pNextSwapBlockStatus, \
                        pFLASHCOMMANDSEQUENCE pFlashCommandSequence)
{
    flash_drv_status_t ret;      /* Return code variable */

    /* clear RDCOLERR & ACCERR & FPVIOL flag in flash status register. Write 1 to clear*/
    CLEAR_FTFx_FSTAT_ERROR_BITS;
    /* passing parameter to the command */
    FTFx_FCCOB0 = FTFx_PFLASH_SWAP;
    FTFx_FCCOB1 = GET_BIT_16_23(addr);
    FTFx_FCCOB2 = GET_BIT_8_15(addr);
    FTFx_FCCOB3 = GET_BIT_0_7(addr);

    FTFx_FCCOB4 = swapcmd;
    FTFx_FCCOB5 = 0xFFU;
    FTFx_FCCOB6 = 0xFFU;
    FTFx_FCCOB7 = 0xFFU;


    /* calling flash command sequence function to execute the command */
    ret = pFlashCommandSequence(pSSDConfig);

    if (FTFx_OK == ret)
    {
        *pCurrentSwapMode = FTFx_FCCOB5;
        *pCurrentSwapBlockStatus = FTFx_FCCOB6;
        *pNextSwapBlockStatus = FTFx_FCCOB7;
    }
    return (ret);
}
#endif /* End of FSL_FEATURE_FLASH_HAS_PFLASH_BLOCK_SWAP */
#if FSL_FEATURE_FLASH_HAS_ERASE_ALL_BLOCKS_UNSECURE_CMD
/*!
 * brief Flash erase all Blocks unsecure.
 *
 * This API  erases all Flash memory,  initializes  the FlexRAM, verifies
 * all memory contents, and then releases the MCU security.
 *
 * param pSSDConfig:    The SSD configuration structure pointer.
 * param pFlashCommandSequence :     Pointer to the Flash command sequence function.
 * return Successful completion (FTFx_OK)
 * return Error value  (FTFx_ERR_PVIOL, FTFx_ERR_MGSTAT0, FTFx_ERR_ACCERR)
 */
flash_drv_status_t FlashEraseAllBlockUnsecure (const PFLASH_SSD_CONFIG pSSDConfig, \
                       pFLASHCOMMANDSEQUENCE pFlashCommandSequence)
{
    flash_drv_status_t ret;         /* return code variable */

    /* clear RDCOLERR & ACCERR & FPVIOL flag in flash status register. Write 1 to clear*/
    CLEAR_FTFx_FSTAT_ERROR_BITS;
    /* passing parameter to the command */
    FTFx_FCCOB0 = FTFx_ERASE_ALL_BLOCK_UNSECURE;

    /* calling flash command sequence function to execute the command */
    ret = pFlashCommandSequence(pSSDConfig);
    return(ret);
}
#endif /* End of FSL_FEATURE_FLASH_HAS_ERASE_ALL_BLOCKS_UNSECURE_CMD */

