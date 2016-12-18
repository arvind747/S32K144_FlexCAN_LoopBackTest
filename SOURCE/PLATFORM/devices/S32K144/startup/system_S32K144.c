/*
** ###################################################################
**     Processor:           S32K144_100
**     Compilers:           GNU C Compiler
**                          GreenHills ARM Compiler
**                          IAR ANSI C/C++ Compiler for ARM
**
**     Reference manual:    S32K144RM Rev. 1, Draft D, 10/2015
**     Version:             rev. 1.7, 2015-10-21
**     Build:               b151027
**
**     Abstract:
**         Provides a system configuration function and a global variable that
**         contains the system frequency. It configures the device and initializes
**         the oscillator (PLL) that is part of the microcontroller device.
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
**     - rev. 1.0 (2015-04-09) - Iulian Talpiga
**         Initial version.
**     - rev. 1.1 (2015-05-19) - Bogdan Nitu
**         Updated interrupts table
**         Removed SIM_CHIPCTL_DAC2CMP
**         Compacted PORT_PCR registers
**     - rev. 1.2 (2015-09-16) - Iulian Talpiga
**         Implemented SystemCoreClockUpdate
**     - rev. 1.3 (2015-11-26) - Iulian Talpiga
**         Improved FPU enable logic and comments
**
** ###################################################################
*/

/*!
 * @file S32K144
 * @version 1.3
 * @date 2015-11-26
 * @brief Device specific configuration file for S32K144 (implementation file)
 *
 * Provides a system configuration function and a global variable that contains
 * the system frequency.
 */


#include "fsl_device_registers.h"
#include "system_S32K144.h"

#define WDOG_UNLOCK_KEY     (0xD928C520UL)
#define WDOG_CLK_FROM_BUS   (0x0UL)
#define WDOG_CLK_FROM_LPO   (0x1UL)
#define WDOG_CLK_FROM_SOSC  (0x2UL)
#define WDOG_CLK_FROM_SIRC  (0x3UL)

/* ----------------------------------------------------------------------------
   -- Core clock
   ---------------------------------------------------------------------------- */

uint32_t SystemCoreClock = DEFAULT_SYSTEM_CLOCK;

/* ----------------------------------------------------------------------------
   -- SystemInit()
   ---------------------------------------------------------------------------- */

void SystemInit (void) {
/**************************************************************************/
                      /* FPU ENABLE*/
/**************************************************************************/
#if (ENABLE_FPU == 1)
  /* Enable CP10 and CP11 coprocessors */
  FSL_SCB->CPACR |= (FSL_SCB_CPACR_CP10_MASK | FSL_SCB_CPACR_CP11_MASK); 
#endif /* ENABLE_FPU */

/**************************************************************************/
                      /* WDOG DISABLE*/
/**************************************************************************/
  
#if (DISABLE_WDOG)
  /* Write of the WDOG unlock key to CNT register, must be done in order to allow any modifications*/
  WDOG->CNT = (uint32_t ) WDOG_UNLOCK_KEY;
  
  /* Initial write of WDOG configuration register; clock select from LPO, update enable, watchdog disabled*/
  /* In the CS reg, bit 13 is reserved but if it is not written the WDOG can't be disabled*/
  WDOG->CS  = (uint32_t ) ( (1 << 13)                                |
                            (WDOG_CLK_FROM_LPO << WDOG_CS_CLK_SHIFT) | 
                            (0 << WDOG_CS_EN_SHIFT)                  |
                            (1 << WDOG_CS_UPDATE_SHIFT)              );
                            
  /* Configure timeout */
  WDOG->TOVAL = (uint32_t )0xFFFF;
#endif /* (DISABLE_WDOG) */

/**************************************************************************/
                      /* Power mode protection */
/**************************************************************************/
#ifdef SYSTEM_SMC_PMPROT_VALUE
  /* Power mode protection initialization */
  SMC->PMPROT = SYSTEM_SMC_PMPROT_VALUE;
#endif
}

/* ----------------------------------------------------------------------------
   -- SystemCoreClockUpdate()
   ---------------------------------------------------------------------------- */

void SystemCoreClockUpdate (void) {

  uint32_t SCGOUTClock;      /* Variable to store output clock frequency of the SCG module */
  uint32_t regValue;         /* Temporary variable */
  uint16_t divider, prediv, multi;
  static const uint32_t fircFreq[] = {
      FSL_FEATURE_SCG_FIRC_FREQ0,
      FSL_FEATURE_SCG_FIRC_FREQ1,
      FSL_FEATURE_SCG_FIRC_FREQ2,
      FSL_FEATURE_SCG_FIRC_FREQ3,
  };
  
  divider = ((SCG->CSR & SCG_CSR_DIVCORE_MASK) >> SCG_CSR_DIVCORE_SHIFT) + 1;

  switch ((SCG->CSR & SCG_CSR_SCS_MASK) >> SCG_CSR_SCS_SHIFT) {
    case 0x1:
      /* System OSC */
      SCGOUTClock = CPU_XTAL_CLK_HZ;
      break;
    case 0x2:
      /* Slow IRC */
      SCGOUTClock = (((SCG->SIRCCFG & SCG_SIRCCFG_RANGE_MASK) >> SCG_SIRCCFG_RANGE_SHIFT) ?
                    FSL_FEATURE_SCG_SIRC_HIGH_RANGE_FREQ : FSL_FEATURE_SCG_SIRC_LOW_RANGE_FREQ);
      break;
    case 0x3:
      /* Fast IRC */
      regValue = (SCG->FIRCCFG & SCG_FIRCCFG_RANGE_MASK) >> SCG_FIRCCFG_RANGE_SHIFT;
      SCGOUTClock= fircFreq[regValue];
      break;
    case 0x6:
      /* System PLL */
      SCGOUTClock = CPU_XTAL_CLK_HZ;
      prediv = ((SCG->SPLLCFG & SCG_SPLLCFG_PREDIV_MASK) >> SCG_SPLLCFG_PREDIV_SHIFT) + 1U;
      multi = ((SCG->SPLLCFG & SCG_SPLLCFG_MULT_MASK) >> SCG_SPLLCFG_MULT_SHIFT) + 16U;
      SCGOUTClock = SCGOUTClock * multi / (prediv * 2U);
      break;
    default:
      return;
  }

  SystemCoreClock = (SCGOUTClock / divider);
}
