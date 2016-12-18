
#ifndef _TRACE_H_
#define _TRACE_H_

/**************************************************************************************************/
/* System Header Files Section                                                                    */
/**************************************************************************************************/

/**************************************************************************************************/
/* User Header Files Section                                                                      */
/**************************************************************************************************/

/**************************************************************************************************/
/* Export Global Definitions Section                                                              */
/**************************************************************************************************/

//#define ITM_ENABLE

/*! @brief Device instance number */
#define FSL_LPUART1 (1U)

#define SPRINTF						( tfp_sprintf )

#define TRACE_LEVEL_DEBUG			( 0x05 )
#define TRACE_LEVEL_INFO			( 0x04 )
#define TRACE_LEVEL_WARNING		( 0x03 )
#define TRACE_LEVEL_ERROR			( 0x02 )
#define TRACE_LEVEL_FATAL			( 0x01 )

#define TRACE_LEVEL 				( TRACE_LEVEL_DEBUG )


/* By default, all traces are output except the debug one */
#if !defined(TRACE_LEVEL)    
#define TRACE_LEVEL TRACE_LEVEL_INFO
#endif

/* Trace compilation depends on TRACE_LEVEL value */
#if (TRACE_LEVEL >= TRACE_LEVEL_DEBUG)
#define TRACE_DEBUG(...)      { tfp_printf("-D- " __VA_ARGS__); }
#define TRACE_DEBUG_WP(...)   { tfp_printf(__VA_ARGS__); }
#else
#define TRACE_DEBUG(...)      { }
#define TRACE_DEBUG_WP(...)   { }
#endif

#if (TRACE_LEVEL >= TRACE_LEVEL_INFO)
#define TRACE_INFO(...)       { tfp_printf("-I- " __VA_ARGS__); }
#define TRACE_INFO_WP(...)    { tfp_printf(__VA_ARGS__); }
#else
#define TRACE_INFO(...)       { }
#define TRACE_INFO_WP(...)    { }
#endif

#if (TRACE_LEVEL >= TRACE_LEVEL_WARNING)
#define TRACE_WARNING(...)    { tfp_printf("-W- " __VA_ARGS__); }
#define TRACE_WARNING_WP(...) { tfp_printf(__VA_ARGS__); }
#else
#define TRACE_WARNING(...)    { }
#define TRACE_WARNING_WP(...) { }
#endif

#if (TRACE_LEVEL >= TRACE_LEVEL_ERROR)
#define TRACE_ERROR(...)      { tfp_printf("-E- " __VA_ARGS__); }
#define TRACE_ERROR_WP(...)   { tfp_printf(__VA_ARGS__); }
#else
#define TRACE_ERROR(...)      { }
#define TRACE_ERROR_WP(...)   { }
#endif

#if (TRACE_LEVEL >= TRACE_LEVEL_FATAL)
#define TRACE_FATAL(...)      { tfp_printf("-F- " __VA_ARGS__); while(1); }
#define TRACE_FATAL_WP(...)   { tfp_printf(__VA_ARGS__); while(1); }
#else
#define TRACE_FATAL(...)      { while(1); }
#define TRACE_FATAL_WP(...)   { while(1); }
#endif

#define FUNC_ENTER  TRACE_INFO_WP("\r\nEnter: %s()\r\n", __func__)
#define FUNC_EXIT   TRACE_INFO_WP("Exit: %s()\r\n\n", __func__)


#ifdef 	ITM_ENABLE

#define TRACE_SendChar(Data) ITM_SendChar(Data)

#else

#define TRACE_SendChar(Data) //USART_SendChar(USART_3, Data)

#endif

/**************************************************************************************************/
/* Export Global Variable Section                                                                 */
/**************************************************************************************************/

/**************************************************************************************************/
/* Export Functions Section                                                                       */
/**************************************************************************************************/

/* Initialize Trace Service */
extern void TRACE_Init(void);

/* */
extern void tfp_printf(char *fmt, ...);

/* */
extern void tfp_sprintf(char* s,char *fmt, ...);


#endif



/**************************************************************************************************/
/* End of TRACE.h                                                                                 */
/**************************************************************************************************/

