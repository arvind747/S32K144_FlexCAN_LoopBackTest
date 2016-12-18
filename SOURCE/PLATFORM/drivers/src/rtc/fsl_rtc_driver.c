/*
 * Copyright (c) 2014 - 2016, Freescale Semiconductor, Inc.
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

#include "fsl_rtc_driver.h"

/*!< Table of RTC base pointers */
RTC_Type *      g_rtcBase[]         = RTC_BASE_PTRS;

/*!< Table of RTC IRQ Numbers */
extern IRQn_Type       g_rtcIrqNumbers[];
/*!< Table of RTC Second IRQ Numbers */
extern IRQn_Type       g_rtcSecondsIrqNb[];

/*!
 * @brief static RTC runtime structure, it is designed only for internal
 * purposes such as storing interrupt configuration for each instance.
 */
static struct _rtc_runtime_t
{
    rtc_alarm_config_t          * alarmConfig;    /*!< Alarm configuration     */
    bool                          isAlarmTimeNew; /* Check if there is a 
                                                   * new alarm                 */
    rtc_interrupt_config_t      * intConfig;      /*!< Interrupt configuration */
    rtc_seconds_int_config_t    * secondsIntConfig;
                                                  /*!< Time seconds interrupt
                                                   * configuration
                                                   */
} g_rtcRuntimeConfig[RTC_INSTANCE_COUNT];


/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name RTC_DRV_Init
 * Description   This function initializes the RTC instance with the settings
 * provided by the user via the rtcUserCfg parameter. The user must ensure that
 * clock is enabled for the RTC instance used. If the Control register is locked
 * then this method returns RTC_STATUS_LOCKED. In order to clear the CR Lock
 * the user must perform a power-on reset.
 * Return        RTC_STAT_SUCCESS if the operation was successful, RTC_STAT_ERROR
 *               if there was a problem or RTC_STATUS_LOCKED if at least one
 *               register is locked
 *
 *END**************************************************************************/
rtc_status_t RTC_DRV_Init(uint8_t instance,
                            rtc_init_config_t * const rtcUserCfg)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < RTC_INSTANCE_COUNT)
#endif

    rtc_status_t statusCode = RTC_STAT_SUCCESS;

    /* Check if the configuration is valid */
    if(rtcUserCfg == NULL)
    {
        /* If not, return error status */
        statusCode = RTC_STAT_ERROR;
    }
    else
    {
        /* Initialize runtime structure */
        g_rtcRuntimeConfig[instance].alarmConfig        = NULL;
        g_rtcRuntimeConfig[instance].intConfig          = NULL;
        g_rtcRuntimeConfig[instance].secondsIntConfig   = NULL;
        g_rtcRuntimeConfig[instance].isAlarmTimeNew     = false;

        RTC_Type * basePtr = g_rtcBase[instance];

        /* Check if the control register is locked. If true, the method cannot
         * continue.
         */
        if(RTC_HAL_isRegisterLocked
                             (basePtr, RTC_CTRL_REG_LOCK) == true)
        {
            statusCode = RTC_STAT_LOCKED;
        }
        else
        {
            /* Disable the RTC instance IRQ to perform a software reset */
            INT_SYS_DisableIRQ(g_rtcIrqNumbers[instance]);
            /* Perform a software reset */
            RTC_HAL_SetSoftwareReset(basePtr);
            RTC_HAL_ClearSoftwareReset(basePtr);
            /* Initialize the RTC Instance */
            statusCode = RTC_HAL_Init(basePtr);

            /* Clear the pending interrupt generated by the software reset */
            INT_SYS_ClearPending(g_rtcIrqNumbers[instance]);

            /* Setup the RTC instance as configured in the structure */
            RTC_HAL_SetClockPinEnable(basePtr,  rtcUserCfg->clockPinCfg);
            RTC_HAL_SetOscLoadConfig(basePtr,   rtcUserCfg->oscLoadCfg);
            RTC_HAL_SetClockOutput(basePtr,     rtcUserCfg->clkOutEnable);
            RTC_HAL_SetClockPinSelect(basePtr,  rtcUserCfg->clockPinSelect);
            RTC_HAL_SetLPOSelect(basePtr,       rtcUserCfg->clockSelect);
            RTC_HAL_SetUpdateMode(basePtr,      rtcUserCfg->updateEnable);
            RTC_HAL_SetSupervisorAccess(basePtr,
                                         rtcUserCfg->nonSupervisorAccessEnable);
    #if FSL_RTC_DEVICE_HAS_WAKEUP_PIN
            RTC_HAL_SetWakeupPinSelect(basePtr, rtcUserCfg->wakeupPinSelect);
            RTC_HAL_SetWakeupPinEnable(basePtr, rtcUserCfg->wakeupPinEnable);
    #endif

            /* Check if compensation needs to be updated */
            if((rtcUserCfg->compensation != 0) ||
                                        (rtcUserCfg->compensationInterval != 0))
            {
                RTC_DRV_ConfigureTimeCompensation(instance,
                    rtcUserCfg->compensationInterval, rtcUserCfg->compensation);
            }
        }

    }

    return statusCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name RTC_DRV_Deinit
 * Description   This function deinitializes the RTC instance.
 * If the Control register is locked then this method returns RTC_STATUS_LOCKED.
 * In order to clear the CR Lock the user must perform a power-on reset.
 * Return        RTC_STAT_SUCCESS if the operation was successful, RTC_STAT_ERROR
 *               if there was a problem or RTC_STATUS_LOCKED if at least one
 *               register is locked
 *
 *END**************************************************************************/
rtc_status_t RTC_DRV_Deinit(uint8_t instance)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < RTC_INSTANCE_COUNT)
#endif

    rtc_status_t statusCode = RTC_STAT_SUCCESS;

    /* Check if the control register is locked. If true, the method cannot
     * continue.
     */
    if(RTC_HAL_isRegisterLocked
                        (g_rtcBase[instance], RTC_CTRL_REG_LOCK) == true)
    {
        statusCode = RTC_STAT_LOCKED;
    }
    else
    {
        /* Disable RTC instance's interrupts */
        INT_SYS_DisableIRQ(g_rtcIrqNumbers[instance]);
        INT_SYS_DisableIRQ(g_rtcSecondsIrqNb[instance]);
        /* Perform a software reset */
        RTC_HAL_SetSoftwareReset(g_rtcBase[instance]);
        RTC_HAL_ClearSoftwareReset(g_rtcBase[instance]);
        /* Clear the pending interrupt generated by the software reset */
        INT_SYS_ClearPending(g_rtcIrqNumbers[instance]);
    }

    return statusCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name RTC_DRV_CanWriteTCE
 * Description   This function checks the following conditions to find if the
 * Time Counter Enable bit is writable.
 *      - if Update Mode bitfield if 1 and:
 *          - Time is invalid or
 *          - Time Seconds Register has overflowed or
 *          - Time Counter is disabled,
 *      then the TCE bit can be set even if Status Register is locked.
 *
 *      This method is a private one, it is used only by the API internally.
 * Return  True if the TCE can be set, otherwise false
 *
 *END**************************************************************************/
static bool RTC_DRV_CanWriteTCE(uint8_t instance)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < RTC_INSTANCE_COUNT)
#endif

    bool result = false;
    /* Check if the status register is locked */
    if(RTC_HAL_isRegisterLocked(g_rtcBase[instance], RTC_STAT_REG_LOCK) == false)
    {
        result = true;
    }
    /* Get the Update Mode bit */
    else if(RTC_HAL_GetUpdateMode(g_rtcBase[instance]))
    {
        /* Check for the specified conditions */
        if(RTC_HAL_GetTimeInvalidFlag(g_rtcBase[instance])
                || RTC_HAL_GetTimeOverflowFlag(g_rtcBase[instance])
                || (RTC_HAL_GetTimeCounterEnable(g_rtcBase[instance]) == false))
        {
                result = true;
        }
    }

    return result;
}

/*FUNCTION**********************************************************************
 *
 * Function Name RTC_DRV_Enable
 * Description   This function sets the time and enables the time counter the
 * RTC instance with the time and date provided by the user via the startTime
 * parameter.
 * Return        RTC_STAT_SUCCESS if the operation was successful, RTC_STAT_ERROR
 *               if there was a problem or RTC_STATUS_LOCKED if at least one
 *               register is locked
 *
 *END**************************************************************************/
rtc_status_t RTC_DRV_Enable(uint8_t instance,
                            rtc_timedate_t * const startTime)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < RTC_INSTANCE_COUNT)
#endif

    rtc_status_t statusCode = RTC_STAT_SUCCESS;

    /* If the start time is NULL or it is not in a correct format,
     * return error
     */
    if((startTime == NULL) ||
            (RTC_DRV_IsTimeDateCorrectFormat(startTime) == false))
    {
        statusCode = RTC_STAT_ERROR;
    }
    /* Check if the TCE is writable and return corresponding status
     * if it is not
     */
    else if(RTC_DRV_CanWriteTCE(instance) == false)
    {
        statusCode = RTC_STAT_LOCKED;
    }
    else
    {
        uint32_t seconds = 0;
        /* Convert start time to seconds */
        RTC_DRV_ConvertTimeDateToSeconds(startTime, &seconds);
        /* Set start time */
        statusCode  =
                   RTC_HAL_SetTimeSecondsRegister(g_rtcBase[instance], seconds);

        /* If time setup was successful enable the counter */
        if(statusCode == RTC_STAT_SUCCESS)
        {
            statusCode  = RTC_HAL_Enable(g_rtcBase[instance]);
        }
    }

    return statusCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name RTC_DRV_Disable
 * Description   This function disables the current RTC instance
 * parameter.
 * Return        RTC_STAT_SUCCESS if the operation was successful, RTC_STAT_ERROR
 *               if there was a problem or RTC_STATUS_LOCKED if status
 *               register is locked
 *
 *END**************************************************************************/
rtc_status_t RTC_DRV_Disable(uint8_t instance)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < RTC_INSTANCE_COUNT)
#endif

    rtc_status_t statusCode = RTC_STAT_SUCCESS;

    /* Check if the TCE is writable */
    if(RTC_DRV_CanWriteTCE(instance) == false)
    {
        statusCode = RTC_STAT_LOCKED;
    }
    else
    {
        /* Disable the RTC instance */
        statusCode = RTC_HAL_Disable(g_rtcBase[instance]);
    }

    return statusCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name RTC_DRV_GetCurrentTimeDate
 * Description   This retrieves the current time and date from the RTC instance.
 * Data is saved into currentTime, which is a pointer of the rtc_timedate_t
 * type.
 * Return        RTC_STAT_SUCCESS if the operation was successful, RTC_STAT_ERROR
 *               if there was a problem.
 *
 *END**************************************************************************/
rtc_status_t RTC_DRV_GetCurrentTimeDate(uint8_t instance,
                                   rtc_timedate_t * currentTime)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < RTC_INSTANCE_COUNT)
#endif

    rtc_status_t statusCode = RTC_STAT_SUCCESS;
    uint32_t seconds, tempSeconds;

    if(currentTime == NULL)
    {
        statusCode = RTC_STAT_ERROR;
    }
    else
    {
        /* Make two consecutive reads to ensure that the read was not
         * done when the counter is incrementing.
         * This is recommended in the reference manual.
         */
        tempSeconds  = RTC_HAL_GetTimeSecondsRegister(g_rtcBase[instance]);
        seconds      = RTC_HAL_GetTimeSecondsRegister(g_rtcBase[instance]);
        /* If the read was done when the counter was incrementing,
         * try and read again.
         */
        if(tempSeconds != seconds)
        {
            tempSeconds = 0UL;
            tempSeconds = RTC_HAL_GetTimeSecondsRegister(g_rtcBase[instance]);
            if(tempSeconds != seconds)
            {
                /* If the last two reads are not equal, there is an error */
                statusCode = RTC_STAT_ERROR;
            }
            else
            {
                RTC_DRV_ConvertSecondsToTimeDate(&seconds, currentTime);
            }
        }
        else
        {
            RTC_DRV_ConvertSecondsToTimeDate(&seconds, currentTime);
        }
    }

    return statusCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name RTC_DRV_SetTimeDate
 * Description   This modifies the time and date of the RTC instance
 * type.
 * Return        RTC_STAT_SUCCESS if the operation was successful, RTC_STAT_ERROR
 *               if there was a problem, RTC_STAT_LOCKED if the TCE bit is not
 *               writable.
 *
 *END**************************************************************************/
rtc_status_t RTC_DRV_SetTimeDate(uint8_t instance, rtc_timedate_t * time)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < RTC_INSTANCE_COUNT)
#endif

    rtc_status_t statusCode = RTC_STAT_SUCCESS;

    /* Check if the time is in the correct format */
    if(RTC_DRV_IsTimeDateCorrectFormat(time) == false)
    {
        statusCode = RTC_STAT_ERROR;
    }
    /* Check if the TCE bit is writable */
    else if(RTC_DRV_CanWriteTCE(instance) == false)
    {
        statusCode = RTC_STAT_LOCKED;
    }
    else
    {
        uint32_t seconds = 0;
        /* Convert the desired time to seconds */
        RTC_DRV_ConvertTimeDateToSeconds(time, &seconds);
        /* Stop time counter */
        RTC_HAL_SetTimeCounterEnable(g_rtcBase[instance], false);
        /* Set the time */
        RTC_HAL_SetTimeSecondsRegister(g_rtcBase[instance], seconds);
        /* Re-enable the time counter */
        RTC_HAL_SetTimeCounterEnable(g_rtcBase[instance], true);

        /* Check if the time is invalid */
        if(RTC_HAL_GetTimeInvalidFlag(g_rtcBase[instance]))
        {
            statusCode = RTC_STAT_ERROR;
        }
    }

    return statusCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name RTC_DRV_ConfigureRegisterLock
 * Description   This method configures register lock for the corresponding
 * RTC instance. Remember that all the registers are unlocked only by software
 * reset or power on reset. (Excepting CR that is unlocked only by POR).
 * Return        RTC_STAT_SUCCESS if the operation was successful,
 *               RTC_STAT_LOCKED if the Lock Register is locked.
 *
 *END**************************************************************************/
rtc_status_t RTC_DRV_ConfigureRegisterLock(uint8_t instance,
            const rtc_register_lock_config_t * const lockConfig)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < RTC_INSTANCE_COUNT)
#endif

    rtc_status_t statusCode = RTC_STAT_SUCCESS;

    /* Configure register lock */
    if(lockConfig->controlRegisterLock)
    {
        statusCode = RTC_HAL_ConfigureRegisterLock
                                    (g_rtcBase[instance], RTC_CTRL_REG_LOCK);
    }
    if(lockConfig->statusRegisterLock)
    {
            statusCode = RTC_HAL_ConfigureRegisterLock
                                    (g_rtcBase[instance], RTC_STAT_REG_LOCK);
    }
    if(lockConfig->timeCompensationRegisterLock)
    {
            statusCode = RTC_HAL_ConfigureRegisterLock
                                    (g_rtcBase[instance], RTC_TCE_REG_LOCK);
    }
    if(lockConfig->lockRegisterLock)
    {
            statusCode = RTC_HAL_ConfigureRegisterLock
                                    (g_rtcBase[instance], RTC_LOCK_REG_LOCK);
    }

    return statusCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name RTC_DRV_GetRegisterLock
 * Description   This retrieves the register lock configuration from the RTC
 * instance. Data is stored in the structure referenced by the lockConfig
 * pointer.
 * Return        None
 *
 *END**************************************************************************/
void RTC_DRV_GetRegisterLock(uint8_t instance,
                rtc_register_lock_config_t * lockConfig)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < RTC_INSTANCE_COUNT)
#endif

    lockConfig->lockRegisterLock = RTC_HAL_isRegisterLocked
                                    (g_rtcBase[instance], RTC_LOCK_REG_LOCK);
    lockConfig->controlRegisterLock = RTC_HAL_isRegisterLocked
                                    (g_rtcBase[instance], RTC_CTRL_REG_LOCK);
    lockConfig->statusRegisterLock = RTC_HAL_isRegisterLocked
                                    (g_rtcBase[instance], RTC_STAT_REG_LOCK);
    lockConfig->timeCompensationRegisterLock = RTC_HAL_isRegisterLocked
                                    (g_rtcBase[instance], RTC_TCE_REG_LOCK);

}

/*FUNCTION**********************************************************************
 *
 * Function Name RTC_DRV_ConfigureTimeCompensation
 * Description   This method configures time compensation. Data is passed by
 * the compInterval and compensation parameters.
 * For more details regarding coefficient calculation see the Reference Manual.
 * Return        RTC_STAT_SUCCESS if the operation was successful,
 * RTC_STAT_LOCKED if the TC Register is locked.
 *
 *END**************************************************************************/
rtc_status_t RTC_DRV_ConfigureTimeCompensation(uint8_t instance,
                                    uint8_t compInterval, int8_t compensation)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < RTC_INSTANCE_COUNT)
#endif

    rtc_status_t statusCode = RTC_STAT_SUCCESS;

    /* Check if the TCR is locked */
    if(RTC_HAL_isRegisterLocked
            (g_rtcBase[instance], RTC_TCE_REG_LOCK) == true)
    {
        statusCode = RTC_STAT_LOCKED;
    }
    else
    {
        /* Set the corresponding values for compensation and compensation
         * interval.
         */
        RTC_HAL_SetTimeCompensationRegister
                        (g_rtcBase[instance], compensation);
        RTC_HAL_SetCompensationIntervalRegister
                        (g_rtcBase[instance], compInterval);
    }

    return statusCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name RTC_DRV_GetTimeCompensation
 * Description   This retrieves the time compensation coefficients and saves
 * them on the variables referenced by the parameters.
 *
 * Return        None
 *
 *END**************************************************************************/
void RTC_DRV_GetTimeCompensation(uint8_t instance,
                                uint8_t * compInterval, int8_t * compensation)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < RTC_INSTANCE_COUNT)
#endif

    (*compInterval) = RTC_HAL_GetCompensationIntervalRegister
                                                        (g_rtcBase[instance]);
    (*compensation) = RTC_HAL_GetTimeCompensationRegister
                                                        (g_rtcBase[instance]);
}

/*FUNCTION**********************************************************************
 *
 * Function Name RTC_DRV_ConvertSecondsToTimeDate
 * Description   This method converts seconds into time-date format.
 *
 * Return        RTC_STAT_ERROR if one or both of the parameters are NULL,
 *               RTC_STAT_SUCCESS otherwise.
 *
 *END**************************************************************************/
rtc_status_t RTC_DRV_ConvertSecondsToTimeDate(const uint32_t *seconds,
                                                      rtc_timedate_t * timeDate)
{
    rtc_status_t statusCode = RTC_STAT_SUCCESS;

    /* Check if parameters are null, if yes return error */
    if((timeDate == NULL) || (seconds == NULL))
    {
        statusCode = RTC_STAT_ERROR;
    }
    else
    {

      /* Set the year to the beginning of the range */
      timeDate->year = YEAR_RANGE_START;

      /* Declare the variables needed */
      uint8_t i;
      bool yearLeap = false;
      uint32_t numberOfDays = 0U, tempSeconds;

      uint16_t daysInYear = DAYS_IN_A_YEAR;

      /* Get the number of days */
      numberOfDays = (*seconds) / SECONDS_IN_A_DAY;
      /* Get the number of seconds remaining */
      tempSeconds = (*seconds) % SECONDS_IN_A_DAY;

      /* Get the current hour */
      timeDate->hour        = (uint16_t)(tempSeconds / SECONDS_IN_A_HOUR);
      /* Get the remaining seconds */
      tempSeconds           = tempSeconds % SECONDS_IN_A_HOUR;
      /* Get the minutes */
      timeDate->minutes     = (uint16_t)(tempSeconds / SECONDS_IN_A_MIN);
      /* Get seconds */
      timeDate->seconds = (uint8_t)(tempSeconds % SECONDS_IN_A_MIN);

      /* Get the current year */
      while (numberOfDays >= daysInYear)
      {
          /* Increment year if the number of days is greater than the ones in
           * one year
           */
          timeDate->year++;
          /* Subtract the number of the days */
          numberOfDays -= daysInYear;

          /* Check if the year is leap or unleap */
          if(!RTC_DRV_IsYearLeap(timeDate->year))
          {
              daysInYear = DAYS_IN_A_YEAR;
          }
          else
          {
              daysInYear = DAYS_IN_A_LEAP_YEAR;
          }
      }

      /* Add the current day */
      numberOfDays += 1U;

      if(RTC_DRV_IsYearLeap(timeDate->year))
      {
        yearLeap = true;
      }
      else
      {
        yearLeap = false;
      }

      /* Get the month */
      for (i = 1U; i<=12U; i++)
      {
        uint32_t daysInCurrentMonth = ((yearLeap == true) ?
                            (uint32_t)LY[i] : (uint32_t)ULY[i]);
        if(numberOfDays <= daysInCurrentMonth)
          {
              timeDate->month = (uint16_t)i;
              break;
          }
          else
          {
              numberOfDays -= daysInCurrentMonth;
          }

      }

      timeDate->day = (uint16_t)numberOfDays;
    }

    return statusCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name RTC_DRV_ConvertTimeDateToSeconds
 * Description   This method converts time-date into seconds.
 *
 * Return        RTC_STAT_SUCCESS if the conversion is completed, RTC_STAT_ERROR
 *               if any/both parameters are NULL
 *
 *END**************************************************************************/
rtc_status_t RTC_DRV_ConvertTimeDateToSeconds(rtc_timedate_t * timeDate,
                                                             uint32_t * seconds)
{
    rtc_status_t statusCode = RTC_STAT_SUCCESS;

    /* Check if parameters are null, if yes return error */
    if((timeDate == NULL) || (seconds == NULL))
    {
        statusCode = RTC_STAT_ERROR;
    }
    else
    {
      uint16_t year;

      /* Convert years to seconds */
      (*seconds) = (timeDate->year - YEAR_RANGE_START) * DAYS_IN_A_YEAR
                                                       * SECONDS_IN_A_DAY;

      /* Add the seconds from the leap years */
      for(year = YEAR_RANGE_START; year < timeDate->year; year++)
      {
          if(RTC_DRV_IsYearLeap(year))
          {
              (*seconds) += SECONDS_IN_A_DAY;
          }
      }

      /* If the current year is leap and 29th of February has passed, add
       * another day to seconds passed.
       */
      if((RTC_DRV_IsYearLeap(year)) && (timeDate->month > 2U))
      {
          (*seconds) += SECONDS_IN_A_DAY;
      }

      /* Add the rest of the seconds from the current month */
      (*seconds) += MONTH_DAYS[timeDate->month] * SECONDS_IN_A_DAY;
      /* Add the rest of the seconds from the current day */
      (*seconds) += (uint32_t)((timeDate->day - 1U) * SECONDS_IN_A_DAY);
      /* Add the rest of the seconds from the current time */
      (*seconds) += timeDate->hour * SECONDS_IN_A_HOUR + timeDate->minutes
                                         * SECONDS_IN_A_MIN + timeDate->seconds;
    }

    return statusCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name RTC_DRV_IsTimeDateCorrectFormat
 * Description   This method checks if date-time structure is in a correct
 * format
 *
 * Return        True if the following conditions are met:
 *                  - is a valid year, month and date
 *                  - is a valid time format
 *               False otherwise
 *
 *END**************************************************************************/
bool RTC_DRV_IsTimeDateCorrectFormat(const rtc_timedate_t * timeDate)
{
    bool returnCode = true;

    /* Check if the time and date are in the correct ranges */
    if((timeDate->year < YEAR_RANGE_START) || (timeDate->year > YEAR_RANGE_END)
        ||  (timeDate->month < 1U) || (timeDate->month > 12U)
        ||  (timeDate->day < 1U) || (timeDate->day > 31U)
        ||  (timeDate->hour > HOURS_IN_A_DAY)
        ||  (timeDate->minutes > 60U) || (timeDate->seconds > 60U))
    {
        returnCode = false;
    }
    /* Check if the day is a valid day from the corresponding month */
    else if(RTC_DRV_IsYearLeap(timeDate->year))
    {
        if(timeDate->day > LY[timeDate->month])
        {
            returnCode = false;
        }
        else
        {
            returnCode = true;
        }
    }
    else
    {
        if(timeDate->day > ULY[timeDate->month])
        {
            returnCode = false;
        }
        else
        {
            returnCode = true;
        }
    }

    return returnCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name RTC_DRV_IsYearLeap
 * Description   This method checks if the year passed as a parameter is a leap
 * one.
 *
 * Return        True if the year is leap, false if otherwise.
 *
 *END**************************************************************************/
bool RTC_DRV_IsYearLeap(uint16_t year)
{
    bool isYearLeap = false;

    if(year % 4U)
    {
        isYearLeap = false;
    }
    else if (year % 100U)
    {
        isYearLeap = true;
    }
    else if (year % 400U)
    {
        isYearLeap = false;
    }
    else
    {
        isYearLeap = true;
    }

    return isYearLeap;
}


/*FUNCTION**********************************************************************
 *
 * Function Name RTC_DRV_IRQHandler
 * Description   This method is the API's Interrupt handler for generic and
 * alarm IRQ. It will handle the alarm repetition and calls the user callbacks
 * if they are not NULL.
 *
 * Return        None
 *
 *END**************************************************************************/
void RTC_DRV_IRQHandler(uint8_t instance)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < RTC_INSTANCE_COUNT)
#endif

    uint32_t tempSeconds;

    rtc_alarm_config_t * alarmConfig =
                                g_rtcRuntimeConfig[instance].alarmConfig;

    rtc_interrupt_config_t * intConfig =
                                g_rtcRuntimeConfig[instance].intConfig;

    /* Check if an alarm has occurred */
    if(RTC_HAL_GetTimeAlarmFlag(g_rtcBase[instance]) == true)
    {
        /* If the alarm interrupt configuration has been defined process the
         * alarm IRQ
         */
        if((alarmConfig != NULL))
        {
            /* If recurrence is enabled modify the alarm register to the next
             * alarm.
             */
            if((alarmConfig->numberOfRepeats > 0UL) ||
                                          (alarmConfig->repeatForever == true))
            {
                tempSeconds = RTC_HAL_GetTimeSecondsRegister
                                                          (g_rtcBase[instance]);
                tempSeconds += alarmConfig->repetitionInterval - 1;

                RTC_HAL_SetTimeAlarmRegister(g_rtcBase[instance],tempSeconds);

                /* Convert the next alarm time to time date */
                RTC_DRV_ConvertSecondsToTimeDate(&tempSeconds,
                                     &g_rtcRuntimeConfig[instance].
                                                        alarmConfig->alarmTime);
                g_rtcRuntimeConfig[instance].isAlarmTimeNew = true;
                /* If the alarm repeats forever, set number of repeats to 0
                 * to avoid an accidental trigger of the core overflow flag
                 */
                alarmConfig->numberOfRepeats =
                        (alarmConfig->repeatForever == false) ?
                                (alarmConfig->numberOfRepeats - 1UL) : 0UL;
            }
            else
            {
                /* If the alarm does not repeat, write 0 to TAR to clear the
                 * alarm flag.
                 */
                RTC_HAL_SetTimeAlarmRegister(g_rtcBase[instance], 0UL);
                g_rtcRuntimeConfig[instance].isAlarmTimeNew = false;
            }
            /* If the user has defined a callback, call it */
            if(alarmConfig->alarmCallback != NULL)
            {
                alarmConfig->
                    alarmCallback(alarmConfig->callbackParams);
            }
        }
    }
    /* If the IRQ is not caused by the alarm then call the user callback if
     * defined.
     */
    else if((intConfig->rtcCallback != NULL) && (intConfig != NULL))
    {
        intConfig->rtcCallback(intConfig->callbackParams);
    }

}

/*FUNCTION**********************************************************************
 *
 * Function Name RTC_DRV_SecondsIRQHandler
 * Description   This method is the API's Interrupt handler for RTC Second
 * interrupt. This ISR will call the user callback if defined.
 *
 * Return        None
 *
 *END**************************************************************************/
void RTC_DRV_SecondsIRQHandler(uint8_t instance)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < RTC_INSTANCE_COUNT)
#endif

    rtc_seconds_int_config_t * intCfg =
            g_rtcRuntimeConfig[instance].secondsIntConfig;

    /* If the interrupt is configured by the driver API and the user callback
     * is not NULL, then call it.
     */
    if((intCfg != NULL) || (intCfg->rtcSecondsCallback != NULL))
    {
        intCfg->rtcSecondsCallback(intCfg->secondsCallbackParams);
    }

}

/*FUNCTION**********************************************************************
 *
 * Function Name RTC_DRV_ConfigureInterrupts
 * Description   This method configures general interrupts such as:
 *                  - Time Overflow Interrupt
 *                  - Time Invalid Interrupt
 *               with the user provided configuration struct intConfig.
 *
 * Return        RTC_STAT_SUCCESS if the configuration is successful or
 *               RTC_STAT_ERROR if the configuration parameter is NULL.
 *
 *END**************************************************************************/
rtc_status_t RTC_DRV_ConfigureInterrupts(uint8_t instance,
                                        rtc_interrupt_config_t * intConfig)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < RTC_INSTANCE_COUNT)
#endif

    rtc_status_t statusCode = RTC_STAT_SUCCESS;

    /* Check if the configuration structure is not NULL */
    if(intConfig != NULL)
    {
        /* Disable the IRQ to avoid accidental interrupt requests */
        INT_SYS_DisableIRQ(g_rtcIrqNumbers[instance]);
        /* Save the configuration into the instance's runtime structure */
        g_rtcRuntimeConfig[instance].intConfig = intConfig;

        /* Enable or disable selected interrupts */
        RTC_HAL_SetTimeOverflowIntEnable(g_rtcBase[instance],
                                                intConfig->overflowIntEnable);

        RTC_HAL_SetTimeInvalidIntEnable(g_rtcBase[instance],
                                             intConfig->timeInvalidIntEnable);

        /* After the configuration is done, re-enable the interrupt in NVIC */
        INT_SYS_EnableIRQ(g_rtcIrqNumbers[instance]);
    }
    else
    {
        statusCode = RTC_STAT_ERROR;
    }

    return statusCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name RTC_DRV_ConfigureSecondsInt
 * Description   This method configures the Time Seconds Interrupt with the
 * configuration from the intConfig parameter.
 *
 * Return        RTC_STAT_SUCCESS if the configuration is successful or
 *               RTC_STAT_ERROR if the configuration parameter is NULL.
 *
 *END**************************************************************************/
rtc_status_t RTC_DRV_ConfigureSecondsInt(uint8_t instance,
                                    rtc_seconds_int_config_t * intConfig)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < RTC_INSTANCE_COUNT)
#endif

    rtc_status_t statusCode = RTC_STAT_SUCCESS;
    /* Check if the configuration structure is not NULL */
    if(intConfig != NULL)
    {
        /* Disable the IRQ to avoid accidental interrupt requests */
        INT_SYS_DisableIRQ(g_rtcSecondsIrqNb[instance]);
        /* Disable the IRQ to avoid accidental interrupt requests */
        g_rtcRuntimeConfig[instance].secondsIntConfig = intConfig;

        /* Configure the interrupt frequency */
        RTC_HAL_SetTimeSecondsIntConf
                        (g_rtcBase[instance], intConfig->secondIntConfig);

        /* Enable or disable Time Seconds interrupt */
        RTC_HAL_SetTimeSecondsIntEnable
                        (g_rtcBase[instance], intConfig->secondIntEnable);

        /* After the configuration is done, re-enable the interrupt in NVIC */
        INT_SYS_EnableIRQ(g_rtcSecondsIrqNb[instance]);
    }
    else
    {
        statusCode = RTC_STAT_ERROR;
    }

    return statusCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name RTC_DRV_ConfigureAlarm
 * Description   This method configures the alarm with the
 * configuration from the alarmConfig parameter.
 *
 * Return        RTC_STAT_SUCCESS if the configuration is successful or
 *               RTC_STAT_ERROR if the configuration parameter is NULL.
 *
 *END**************************************************************************/
rtc_status_t RTC_DRV_ConfigureAlarm(uint8_t instance,
                                        rtc_alarm_config_t * alarmConfig)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < RTC_INSTANCE_COUNT)
#endif

    rtc_status_t statusCode = RTC_STAT_SUCCESS;

    /* Check if the configuration structure is not NULL */
    if(alarmConfig != NULL)
    {
        uint32_t seconds;
        /* Check if the alarm time is in a correct format */
        if(RTC_DRV_IsTimeDateCorrectFormat(&(alarmConfig->alarmTime)) == true)
        {
            /* Convert the time date to seconds */
            RTC_DRV_ConvertTimeDateToSeconds
                                        (&(alarmConfig->alarmTime), &seconds);
            /* Disable the IRQ to avoid accidental interrupt requests */
            INT_SYS_DisableIRQ(g_rtcIrqNumbers[instance]);
            /* Disable the IRQ to avoid accidental interrupt requests */
            g_rtcRuntimeConfig[instance].alarmConfig = alarmConfig;

            /* Write alarm time into Time Alarm Register */
            RTC_HAL_SetTimeAlarmRegister(g_rtcBase[instance], seconds);
            /* Enable/disable interrupt source based on the configuration */
            RTC_HAL_SetTimeAlarmIntEnable(g_rtcBase[instance],
                                                alarmConfig->alarmIntEnable);
            /* After the configuration is done, re-enable the interrupt in
             * NVIC.
             */
            INT_SYS_EnableIRQ(g_rtcIrqNumbers[instance]);
        }
        else
        {
            statusCode = RTC_STAT_ERROR;
        }
    }

    return statusCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name RTC_DRV_GetAlarmConfig
 * Description   This method retrieves the alarm configuration.
 *
 * Return        RTC_STAT_SUCCESS if the configuration exist or RTC_STAT_ERROR
 *               if the configuration is not available.
 *
 *END**************************************************************************/
rtc_status_t RTC_DRV_GetAlarmConfig(uint8_t instance,
                                        rtc_alarm_config_t * alarmConfig)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < RTC_INSTANCE_COUNT)
#endif

    rtc_status_t statusCode = RTC_STAT_SUCCESS;

    if(alarmConfig != NULL)
    {
        alarmConfig = g_rtcRuntimeConfig[instance].alarmConfig;
    }
    else
    {
        statusCode = RTC_STAT_ERROR;
    }

    return statusCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name RTC_DRV_IsAlarmPending
 * Description   This method specifies if an alarm has occurred.
 *
 * Return        True if an alarm has occurred, false if not.
 *
 *END**************************************************************************/
bool RTC_DRV_IsAlarmPending(uint8_t instance)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < RTC_INSTANCE_COUNT)
#endif

    return RTC_HAL_GetTimeAlarmFlag(g_rtcBase[instance]);
}

/*FUNCTION**********************************************************************
 *
 * Function Name RTC_DRV_GetNextAlarmTime
 * Description   This method retrieves the next alarm time;
 *
 * Return        RTC_STAT_SUCCESS if the next alarm time is valid,
 *               RTC_STAT_ERROR if there is no new alarm or alarm configuration
 *               specified.
 *
 *END**************************************************************************/
rtc_status_t RTC_DRV_GetNextAlarmTime(uint8_t instance,
                                                    rtc_timedate_t * alarmTime)
{
    rtc_status_t statusCode = RTC_STAT_SUCCESS;

    if((alarmTime != NULL) &&
                          (g_rtcRuntimeConfig[instance].isAlarmTimeNew == true))
    {
        (*alarmTime) = g_rtcRuntimeConfig[instance].alarmConfig->alarmTime;
    }
    else
    {
        statusCode = RTC_STAT_ERROR;
    }

    return statusCode;
}

#if FSL_RTC_DEVICE_HAS_WAKEUP_PIN
/*FUNCTION**********************************************************************
 *
 * Function Name RTC_DRV_AssertWakeUpPin
 * Description   This method specifies asserts the wake up pin.
 *
 * Return        None
 *
 *END**************************************************************************/
void RTC_DRV_AssertWakeUpPin(uint8_t instance)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < RTC_INSTANCE_COUNT)
#endif

    RTC_HAL_SetWakeupPinOn(g_rtcBase[instance], true);
}
#endif
