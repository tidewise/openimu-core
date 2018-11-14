/** ***************************************************************************
 * @file comm_buffers.h for the uart driver(extern_port), for use by the system
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

#ifndef COMM_BUFFERS_H
#define COMM_BUFFERS_H
#include "GlobalConstants.h"

#ifdef __cplusplus
extern "C" {
#endif    

// cir_buf defined in port_def.h
extern unsigned int COM_buf_bytes_available (cir_buf_t *buf_struc);
extern unsigned int COM_buf_headroom (cir_buf_t *buf_struc);
extern unsigned int COM_buf_delete (cir_buf_t *buf_struc, unsigned int pop_cnt);
extern unsigned int COM_buf_delete_byte (cir_buf_t *circBuf );
extern BOOL COM_buf_copy (cir_buf_t *buf_struc,unsigned int buf_index,unsigned int cnt,unsigned char *buf_out);
extern BOOL COM_buf_copy_byte (cir_buf_t *circBuf, unsigned int  bufIndex, unsigned char *bufOut);
extern int  COM_buf_add(cir_buf_t *buf_struc, unsigned char *buf, unsigned int cnt);
extern BOOL COM_buf_get(cir_buf_t *buf_struc, unsigned char *buf, unsigned int cnt);
extern void COM_buf_init(port_struct *port, unsigned char *tx_buf,unsigned char *rx_buf,unsigned int rx_size,unsigned int tx_size);
#ifdef __cplusplus
}
#endif    

#endif
