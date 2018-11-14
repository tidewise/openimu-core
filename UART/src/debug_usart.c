/** ***************************************************************************
 * @file   debug_usart.c
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * usart specific routines to deal with DEBUG serial port
 *  This file provides an abstraction layer, if the processor changes,
 *  this file may have to change but the callers will not.
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

#include "debug_usart.h"
#include "osapi.h"
#include "osresources.h"
#include "platformAPI.h"
#include "uart.h"

#define PORT_DBG_RX_BUF_SIZE COM_BUF_SIZE // 512
#define PORT_DBG_TX_BUF_SIZE COM_BUF_SIZE

#define _CR    0x0D
#define _LF    0x0A
#define	_BELL  0x07
#define _TAB   0x09
#define _SPACE 0x20
#define _BS    0x08
#define _DEL   0x7F

int debugSerialChan = UART_CHANNEL_2;   // defaul channel

void  InitDebugSerialCommunication(uint32_t baudrate)
{
    debugSerialChan = platformGetSerialChannel(DEBUG_SERIAL_PORT);
    uart_init(debugSerialChan, baudrate);
}


/** ****************************************************************************
 * @name DebugSerialPutChar
 * @brief Write character out Serial Port.
 * @param [In] character
 * @retval character
 *******************************************************************************/
unsigned char DebugSerialPutChar (unsigned char c)
{ 
    uart_write(debugSerialChan, &c, 1);
    return c;
}

/** ****************************************************************************
 * @name DebugSerialReadLine
 * @brief Read a line of characters from a serial terminal Will block until a
 *        complete line is read.
 *        Offers terminal-like editing support:
 *                BS(\x08) or DEL(\x7F) delete previous character
 *                TAB(\t) replace by single space
 *                CR(\r) expanded to  CR+LF
 * @param [In] buf - buffer for storing the line
 * @param [In] *index - pointer to current location in the buffer
 * @param [In] len - total length of the buffer
 * @param [Out] *index should be passed in, same as it was before
 * @retval TRUE if _LF is the last character, FALSE otherwise
 *******************************************************************************/

int DebugSerialReadLine(uint8_t  *buf, uint32_t *index, uint32_t len)
{
    uint8_t c = 0, lf = 0;
    int num;

    while (c != _LF){
        num = uart_read(debugSerialChan, &c, 1);
        if(num <= 0){
            break;
        }
        
        if (_TAB == c) {
            c = _SPACE;
        }
        
        if (_CR == c) {
            continue;
        }

        /// Handle special character
        switch (c) {
        case _BS:
        case _DEL:
            if (*index > 0) {
                DebugSerialPutChar(_BS);
                DebugSerialPutChar(_SPACE);
                DebugSerialPutChar(_BS);
                (*index)--;
            }
            break;
        case _CR:
            c = _LF; 
            DebugSerialPutChar(c);
            buf[*index] = c;
            (*index)++;
            lf++;
            break;
        case _LF:
            DebugSerialPutChar(c);
            buf[*index] = c;
            *index = 0;
            lf++;
            break;
        default:
            /// Only keep printable characters
            if ((c >= 0x20) && (c <= 0x7E)) {
                /// check for room in the buffer, saving two characters for CR+LF
                if (*index < (len - 2)) {
                     buf[*index] = c;
                    (*index)++;
                    DebugSerialPutChar(c);
                } else { /// buffer full, send BELL
                    DebugSerialPutChar(_BELL);
                }
            }
        } //end switch
    } // end while
 return (c == _LF);
}



