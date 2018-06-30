/** ***************************************************************************
 * @file debug_usart.h usart specific routines to deal with debug port
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
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

#ifndef DEBUG_USART_H
#define DEBUG_USART_H
#include <stdint.h>

extern unsigned char  DebugSerialPutChar (unsigned char c);
extern int            IsDebugSerialIdle();
extern int            DebugSerialReadLine(uint8_t *buf, uint32_t *index, uint32_t len);
extern void           InitDebugSerialCommunication( uint32_t baudRate );

#endif /*DEBUG_USART_H */

