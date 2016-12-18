/* Including needed modules to compile this module/procedure */
#include <string.h>
#include <stdio.h>


/**************************************************************************************************/
/* User Header Files Section                                                                      */
/**************************************************************************************************/

#include "Includes.h"
#include "Cpu.h"
#include "pin_mux.h"
#include "clockMan1.h"
#include "fsl_lpuart_hal.h"
#include "fsl_lpuart_driver.h"
#include "TRACE.h"
#include "TRACE_Priv.h"

/**************************************************************************************************/
/* Global Definitions Section                                                                     */
/**************************************************************************************************/

/**************************************************************************************************/
/* Global Variables Section                                                                       */
/**************************************************************************************************/

/*! lpuart1 configuration structure */
const lpuart_user_config_t lpuart1_InitConfig0 = 
{
    .baudRate = 115200U,
    .parityMode = LPUART_PARITY_DISABLED,
    .stopBitCount = LPUART_ONE_STOP_BIT,
    .bitCountPerChar = LPUART_8_BITS_PER_CHAR,
};

/*! Driver state structure */
lpuart_state_t lpuart1_State;
void Trace_Send(void);

/******************************************************************************/
/* UART2 Receive Interrupt Handler                                            */
/******************************************************************************/

/******************************************************************************/
/* Trace Initialized                                                          */
/******************************************************************************/
typedef void (*putcf)(void *, char);
static putcf stdout_putf;
static void* stdout_putp;


/**************************************************************************************************/
/* Function Name   : TRACE_Init                                                                   */
/*                                                                                                */
/* Description     :                                                                              */
/*                                                                                                */
/* In Params       :                                                                              */
/*                                                                                                */
/*                                                                                                */
/* Out Params      : None                                                                         */
/*                                                                                                */
/* Return Value    : None                                                                         */
/**************************************************************************************************/

void TRACE_Init(void)
{
    /*clock manager component for details*/
    CLOCK_SYS_Init(g_clockManConfigsArr, FSL_CLOCK_MANAGER_CONFIG_CNT,
    g_clockManCallbacksArr, FSL_CLOCK_MANAGER_CALLBACK_CNT);
    CLOCK_SYS_UpdateConfiguration(0U, CLOCK_MANAGER_POLICY_FORCIBLE);

    /* PinSettings component for more info */
    Pins_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);

    /* LPUART component for configuration details */
    LPUART_DRV_Init(FSL_LPUART1, &lpuart1_State, &lpuart1_InitConfig0);
    
   // lpuartBuffer = 'A';
    
   // Trace_Send(lpuartBuffer);
}

/**************************************************************************************************/
/* Function Name   : ui2a                                                                         */
/*                                                                                                */
/* Description     :                                                                              */
/*                                                                                                */
/* In Params       :                                                                              */
/*                                                                                                */
/*                                                                                                */
/* Out Params      : None                                                                         */
/*                                                                                                */
/* Return Value    : None                                                                         */
/**************************************************************************************************/

static void ui2a(unsigned int num, unsigned int base, int uc,char * bf)
{
	int n=0;
	unsigned int d=1;
	while (num/d >= base)
		d*=base;
	while (d!=0) {
		int dgt = num / d;
		num%= d;
		d/=base;
		if (n || dgt>0 || d==0) {
			*bf++ = dgt+(dgt<10 ? '0' : (uc ? 'A' : 'a')-10);
			++n;
			}
		}
	*bf=0;
}

/**************************************************************************************************/
/* Function Name   : i2a                                                                          */
/*                                                                                                */
/* Description     :                                                                              */
/*                                                                                                */
/* In Params       :                                                                              */
/*                                                                                                */
/*                                                                                                */
/* Out Params      : None                                                                         */
/*                                                                                                */
/* Return Value    : None                                                                         */
/**************************************************************************************************/

static void i2a (int num, char * bf)
{
	if (num<0) {
		num=-num;
		*bf++ = '-';
		}
	ui2a(num,10,0,bf);
}

/**************************************************************************************************/
/* Function Name   : a2d                                                                          */
/*                                                                                                */
/* Description     :                                                                              */
/*                                                                                                */
/* In Params       :                                                                              */
/*                                                                                                */
/*                                                                                                */
/* Out Params      : None                                                                         */
/*                                                                                                */
/* Return Value    : None                                                                         */
/**************************************************************************************************/

static int a2d(char ch)
{
	if (ch>='0' && ch<='9')
		return ch-'0';
	else if (ch>='a' && ch<='f')
		return ch-'a'+10;
	else if (ch>='A' && ch<='F')
		return ch-'A'+10;
	else return -1;
}

/**************************************************************************************************/
/* Function Name   : a2i                                                                          */
/*                                                                                                */
/* Description     :                                                                              */
/*                                                                                                */
/* In Params       :                                                                              */
/*                                                                                                */
/*                                                                                                */
/* Out Params      : None                                                                         */
/*                                                                                                */
/* Return Value    : None                                                                         */
/**************************************************************************************************/

static char a2i(char ch, char** src,int base,int* nump)
{
	char* p= *src;
	int num=0;
	int digit;
	while ((digit=a2d(ch))>=0) {
		if (digit>base) break;
		num=num*base+digit;
		ch=*p++;
		}
	*src=p;
	*nump=num;
	return ch;
}

/**************************************************************************************************/
/* Function Name   : tputcf                                                                       */
/*                                                                                                */
/* Description     :                                                                              */
/*                                                                                                */
/* In Params       :                                                                              */
/*                                                                                                */
/*                                                                                                */
/* Out Params      : None                                                                         */
/*                                                                                                */
/* Return Value    : None                                                                         */
/**************************************************************************************************/

void tputcf(void * args, char data)
{
	TRACE_SendChar(data);
}

/**************************************************************************************************/
/* Function Name   : Trace_Send                                                                   */
/*                                                                                                */
/* Description     :                                                                              */
/*                                                                                                */
/* In Params       :                                                                              */
/*                                                                                                */
/*                                                                                                */
/* Out Params      : None                                                                         */
/*                                                                                                */
/* Return Value    : None                                                                         */
/**************************************************************************************************/
#if 0
void Trace_Send(void)
{
	LPUART_DRV_SendData(FSL_LPUART1, &lpuartBuffer,strlen(lpuartBuffer));
}

#endif

/**************************************************************************************************/
/* Function Name   : uli2a                                                                        */
/*                                                                                                */
/* Description     :                                                                              */
/*                                                                                                */
/* In Params       :                                                                              */
/*                                                                                                */
/*                                                                                                */
/* Out Params      : None                                                                         */
/*                                                                                                */
/* Return Value    : None                                                                         */
/**************************************************************************************************/

static void uli2a(unsigned long int num, unsigned int base, int uc,char * bf)
{
	int n=0;
	unsigned int d=1;
	while (num/d >= base)
		d*=base;
	while (d!=0) {
		int dgt = num / d;
		num%=d;
		d/=base;
		if (n || dgt>0|| d==0) {
			*bf++ = dgt+(dgt<10 ? '0' : (uc ? 'A' : 'a')-10);
			++n;
			}
		}
	*bf=0;
}

/**************************************************************************************************/
/* Function Name   : li2a                                                                         */
/*                                                                                                */
/* Description     :                                                                              */
/*                                                                                                */
/* In Params       :                                                                              */
/*                                                                                                */
/*                                                                                                */
/* Out Params      : None                                                                         */
/*                                                                                                */
/* Return Value    : None                                                                         */
/**************************************************************************************************/

static void li2a (long num, char * bf)
{
	if (num<0) {
		num=-num;
		*bf++ = '-';
		}
	uli2a(num,10,0,bf);
}

/**************************************************************************************************/
/* Function Name   : putchw                                                                       */
/*                                                                                                */
/* Description     :                                                                              */
/*                                                                                                */
/* In Params       :                                                                              */
/*                                                                                                */
/*                                                                                                */
/* Out Params      : None                                                                         */
/*                                                                                                */
/* Return Value    : None                                                                         */
/**************************************************************************************************/
static void putchw(void* putp, putcf putf,int n, char z, char* bf)
{
	char fc=z? '0' : ' ';
	char ch;
	char* p=bf;
	while (*p++ && n > 0)
		n--;
	while (n-- > 0)
		putf(putp,fc);
	while (0 != (ch= *bf++))
	{
		putf(putp,ch);
	}
}

/**************************************************************************************************/
/* Function Name   : tfp_format                                                                   */
/*                                                                                                */
/* Description     :                                                                              */
/*                                                                                                */
/* In Params       :                                                                              */
/*                                                                                                */
/*                                                                                                */
/* Out Params      : None                                                                         */
/*                                                                                                */
/* Return Value    : None                                                                         */
/**************************************************************************************************/

void tfp_format(void* putp,putcf putf,char *fmt, va_list va)
{
	char bf[12];

	char ch;
	char lz=0;
	char lng=0;
	int w=0;

	while (0 != (ch=*(fmt++))) {
		if (ch!='%')
			putf(putp,ch);
		else {
			ch=*(fmt++);
			if (ch=='0') {
				ch=*(fmt++);
				lz=1;
				}
			if (ch>='0' && ch<='9') {
				ch=a2i(ch,&fmt,10,&w);
				}
			if (ch=='l') {
				ch=*(fmt++);
				lng=1;
			}
			switch (ch) {
				case 0:
					goto abort;
				case 'u' : {
					if (lng)
						uli2a(va_arg(va, unsigned long int),10,0,bf);
					else
					ui2a(va_arg(va, unsigned int),10,0,bf);
					putchw(putp,putf,w,lz,bf);
					break;
					}
				case 'd' :  {
					if (lng)
						li2a(va_arg(va, unsigned long int),bf);
					else
					i2a(va_arg(va, int),bf);
					putchw(putp,putf,w,lz,bf);
					break;
					}
				case 'x': case 'X' :
					if (lng)
						uli2a(va_arg(va, unsigned long int),16,(ch=='X'),bf);
					else
					ui2a(va_arg(va, unsigned int),16,(ch=='X'),bf);
					putchw(putp,putf,w,lz,bf);
					break;
				case 'c' :
					putf(putp,(char)(va_arg(va, int)));
					break;
				case 's' :
					putchw(putp,putf,w,0,va_arg(va, char*));
					break;
				case '%' :
					putf(putp,ch);
				default:
					break;
				}
			}
		}
	abort:;
}

/**************************************************************************************************/
/* Function Name   : putcp                                                                        */
/*                                                                                                */
/* Description     :                                                                              */
/*                                                                                                */
/* In Params       :                                                                              */
/*                                                                                                */
/*                                                                                                */
/* Out Params      : None                                                                         */
/*                                                                                                */
/* Return Value    : None                                                                         */
/**************************************************************************************************/
static void putcp(void* p,char c)
{
	*(*((char**)p))++ = c;
}

/**************************************************************************************************/
/* Function Name   : init_printf                                                                  */
/*                                                                                                */
/* Description     :                                                                              */
/*                                                                                                */
/* In Params       :                                                                              */
/*                                                                                                */
/*                                                                                                */
/* Out Params      : None                                                                         */
/*                                                                                                */
/* Return Value    : None                                                                         */
/**************************************************************************************************/

void init_printf(void* putp, void (*putf) (void*,char))
{
	stdout_putf = putf;
	stdout_putp = putp;
}

/**************************************************************************************************/
/* Function Name   : tfp_printf                                                                   */
/*                                                                                                */
/* Description     :                                                                              */
/*                                                                                                */
/* In Params       :                                                                              */
/*                                                                                                */
/*                                                                                                */
/* Out Params      : None                                                                         */
/*                                                                                                */
/* Return Value    : None                                                                         */
/**************************************************************************************************/

void tfp_printf(char *fmt, ...)
{
	va_list va;
	va_start(va,fmt);
//	p_stgHALAPI->fp_mPORT_Disable_IRQ();
	tfp_format(stdout_putp,stdout_putf,fmt,va);
//	p_stgHALAPI->fp_mPORT_Enable_IRQ();
	va_end(va);
}

/**************************************************************************************************/
/* Function Name   : tfp_sprintf                                                                  */
/*                                                                                                */
/* Description     :                                                                              */
/*                                                                                                */
/* In Params       :                                                                              */
/*                                                                                                */
/*                                                                                                */
/* Out Params      : None                                                                         */
/*                                                                                                */
/* Return Value    : None                                                                         */
/**************************************************************************************************/

void tfp_sprintf(char* s,char *fmt, ...)
{
	va_list va;
	va_start(va,fmt);
	tfp_format(&s,putcp,fmt,va);
	putcp(&s,0);
	va_end(va);
}


/**************************************************************************************************/
/* End of TRACE.c                                                                                 */
/**************************************************************************************************/

