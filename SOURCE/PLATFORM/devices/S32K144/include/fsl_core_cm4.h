/*
** ###################################################################
**     Version:             rev. 1.0, 2015-11-26
**     Build:               b151126
**
**     Abstract:
**         Core specific definitions.
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
**     - rev. 1.0 (2015-11-26)
**         Initial version.
**
** ###################################################################
*/

#if !defined(__FSL_CORE_CM4_H__)
#define __FSL_CORE_CM4_H__


#ifdef __cplusplus
 extern "C" {
#endif

/** \brief  Enable FPU

    ENABLE_FPU indicates whether SystemInit will enable the Floating point unit (FPU)
 */
#if defined ( __GNUC__ )
  #if defined (__VFP_FP__) && !defined(__SOFTFP__)
    #define ENABLE_FPU       1
  #endif
  
#elif defined ( __ICCARM__ )
  #if defined __ARMVFP__
    #define ENABLE_FPU       1
  #endif
  
#elif defined ( __ghs__ )
  #if defined (__VFP__)
    #define ENABLE_FPU       1
  #endif
#endif

#ifdef __cplusplus
}
#endif

#endif /* __FSL_CORE_CM4_H__ */

/*******************************************************************************
 * EOF
 ******************************************************************************/
