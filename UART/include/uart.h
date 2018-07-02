/** ***************************************************************************
 * @file   uart.c
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *  UART driver for the DMU380's two user serial ports
 *	transmitting and receive of serial data and then number of bytes remaining in
 *	the circular buffer. There is no FIFO on the uart as there is in the 525
 *
 *****************************************************************************/
/*******************************************************************************
Copyright 2018 ACEINNA, INC

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*******************************************************************************/

#ifndef __UART_H
#define __UART_H
#include "port_def.h"
#include "GlobalConstants.h"
#include "boardDefinition.h"

#define USER_COMM_UART kUserA_UART
#define GPS_UART       kUserB_UART

#ifdef __cplusplus
extern "C" {
#endif    

extern void         uart_init(unsigned int uartChannel,uart_hw *port_hw);
extern void         uart_read(unsigned int channel, port_struct *port);
extern void         uart_write(unsigned int channel, port_struct *port);
extern unsigned int bytes_remaining(unsigned int channel, port_struct *port);
extern void         uart_BIT(int uartType);
extern void         uart_Pause();

//extern void         uart_IrqOnOff(unsigned int uartChannel, uint16_t uartIrqType, FunctionalState en);

#ifdef __cplusplus
}
#endif    

#endif
