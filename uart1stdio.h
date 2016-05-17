//*****************************************************************************
//
// uartstdio.h - Prototypes for the UART console functions.
//
// Copyright (c) 2007-2014 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.0.12573 of the Tiva Utility Library.
//
//*****************************************************************************

#ifndef __UART1STDIO_H__
#define __UART1STDIO_H__

#include <stdarg.h>

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// If built for buffered operation, the following labels define the sizes of
// the transmit and receive buffers respectively.
//
//*****************************************************************************
#ifdef UART_BUFFERED
#ifndef UART_RX_BUFFER_SIZE
#define UART_RX_BUFFER_SIZE     128
#endif
#ifndef UART_TX_BUFFER_SIZE
#define UART_TX_BUFFER_SIZE     1024
#endif
#endif

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern void UART1StdioConfig(uint32_t ui32Port, uint32_t ui32Baud,
                            uint32_t ui32SrcClock);
extern int UART1gets(char *pcBuf, uint32_t ui32Len);
extern unsigned char UARTgetc(void);
extern void UART1printf(const char *pcString, ...);
extern void UART1vprintf(const char *pcString, va_list vaArgP);
extern int UART1write(const char *pcBuf, uint32_t ui32Len);
#ifdef UART_BUFFERED
extern int UART1Peek(unsigned char ucChar);
extern void UART1FlushTx(bool bDiscard);
extern void UART1FlushRx(void);
extern int UART1RxBytesAvail(void);
extern int UART1TxBytesFree(void);
extern void UART1EchoSet(bool bEnable);
#endif

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __UARTSTDIO_H__