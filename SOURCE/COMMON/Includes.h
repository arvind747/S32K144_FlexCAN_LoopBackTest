#ifndef _INCLUDES_H_
#define _INCLUDES_H_

/**************************************************************************************************/
/* System Header Files									                                          */
/**************************************************************************************************/

/**************************************************************************************************/
/* User defined Header Files												                      */
/**************************************************************************************************/

/**************************************************************************************************/
/* Global Definitions														                      */
/**************************************************************************************************/

#define PNULL                						(void *)0x00

typedef unsigned char       						UINT8;
typedef char                						INT8;
typedef unsigned short int  						UINT16;
typedef short int           						INT16;
typedef unsigned int       							UINT32;
typedef int                 						INT32;

typedef unsigned char       						BYTE;
typedef unsigned short int  						WORD;
typedef unsigned int        						DWORD;
typedef volatile unsigned int        				SYS_REG;
typedef volatile unsigned char                      SYS_REG_8BIT;

typedef unsigned char                               BOOLEAN;
typedef signed char                					INT8S;
typedef unsigned char       						INT8U;
typedef const char          						INT8C;
typedef short int           						INT16S;
typedef unsigned short int  						INT16U;
typedef int                 						INT32S;
typedef unsigned long int   						INT32U;
typedef float										INT32F;
typedef unsigned long long int 						INT64U;

#endif

/**************************************************************************************************/
/* End of Includes.h                                                                              */
/**************************************************************************************************/
