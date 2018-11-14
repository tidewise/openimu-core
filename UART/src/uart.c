/** ***************************************************************************
 * @file   uart.c
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 * @details
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

#include <stdint.h>

#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"
#include "port_def.h"
#include "uart.h"
#include "comm_buffers.h"
#include "osapi.h"
#include "boardDefinition.h"
#include "BITStatus.h"
#include "osresources.h"
#include "platformAPI.h"


//extern port_struct gPort[], gPort0, gPort1, gPort2;
port_struct gPort[3];

#define INT_DISABLED 0
#define INT_ENABLED  1

/// size of port-associated software circular buffers
#define PORT_RX_BUF_SIZE   COM_BUF_SIZE // 512
#define PORT_TX_BUF_SIZE   COM_BUF_SIZE

osSemaphoreId uartRxSemIds[NUM_UART_PORTS] = {0,0,0};


struct sPinConfig {
    GPIO_TypeDef *port;
    uint16_t     pin;
    uint16_t     source;
    uint8_t      altFun;
};

struct sUartConfig {
    USART_TypeDef     *uart;
    uint32_t          ahb1ClockEnable;
    uint32_t          apb1ClockEnable;
    uint32_t          ahb2ClockEnable;
    uint32_t          apb2ClockEnable;
    struct sPinConfig tx;
    struct sPinConfig rx;
    uint16_t          irqChannel;
};

// 0 - user com, 1 GPS
const struct sUartConfig gUartConfig[NUM_UART_PORTS] = {
    {
        .uart            = USER_A_UART,             // UART4
        .ahb1ClockEnable = USER_A_UART_TX_GPIO_CLK | USER_A_UART_TX_GPIO_CLK,
        .apb1ClockEnable = USER_A_UART_CLK,
        .ahb2ClockEnable = 0,
        .apb2ClockEnable = 0,
        .tx = {
            .pin    = USER_A_UART_TX_PIN,
            .port   = USER_A_UART_TX_GPIO_PORT,
            .source = USER_A_UART_TX_SOURCE,
            .altFun = USER_A_UART_TX_AF
        },
        .rx = {
            .pin    = USER_A_UART_RX_PIN,
            .port   = USER_A_UART_RX_GPIO_PORT,
            .source = USER_A_UART_RX_SOURCE,
            .altFun = USER_A_UART_RX_AF
        },
        .irqChannel = USER_A_UART_IRQn,

    }, {
        .uart            = USER_B_UART,             // UART5
        .ahb1ClockEnable = USER_B_UART_TX_GPIO_CLK | USER_B_UART_TX_GPIO_CLK,
        .apb1ClockEnable = USER_B_UART_CLK,
        .ahb2ClockEnable = 0,
        .apb2ClockEnable = 0,
        .tx = {
            .pin    = USER_B_UART_TX_PIN,            // C12
            .port   = USER_B_UART_TX_GPIO_PORT,
            .source = USER_B_UART_TX_SOURCE,
            .altFun = USER_B_UART_TX_AF
        },
        .rx = {
            .pin    = USER_B_UART_RX_PIN,            // D2
            .port   = USER_B_UART_RX_GPIO_PORT,
            .source = USER_B_UART_RX_SOURCE,
            .altFun = USER_B_UART_RX_AF
        },
        .irqChannel = USER_B_UART_IRQn,
    } , {
        .uart            = DEBUG_USART,                 // USART_1
        .ahb1ClockEnable = DEBUG_USART_TX_GPIO_CLK | DEBUG_USART_RX_GPIO_CLK,
        .apb1ClockEnable = 0,
        .ahb2ClockEnable = 0,
        .apb2ClockEnable = DEBUG_USART_CLK,
        .tx = {
            .pin    = DEBUG_USART_TX_PIN,               // PA_9
            .port   = DEBUG_USART_TX_GPIO_PORT,
            .source = DEBUG_USART_TX_SOURCE,
            .altFun = DEBUG_USART_TX_AF
        },
        .rx = {
            .pin    = DEBUG_USART_RX_PIN,               // PA_10
            .port   = DEBUG_USART_RX_GPIO_PORT,
            .source = DEBUG_USART_RX_SOURCE,
            .altFun = DEBUG_USART_RX_AF
        },
        .irqChannel = DEBUG_USART_IRQn,                 // USART1_IRQn
    }

};

void uart_registerRxSemaphore(int portType, osSemaphoreId id)
{
    int channel;
    if(id){
        channel = platformGetSerialChannel(portType);
        uartRxSemIds[channel] = id;
    }
}

/** ****************************************************************************
 * @name uart_init
 * @brief initializes all channels of the UART peripheral
 *
 * @param [in] uartChannel - 0- USER_UART, port 1 - GPS UART, 2 - DEBUG UART
 * @param [in] *port_hw - uart_hw data structure containing fifo trigger levels
 *						and the baudrate for this port
 * @retval CN/A
 ******************************************************************************/
int uart_init(int channel, int baudrate)
{
    /// port-associated software rx and tx circular buffers
	static uint8_t port_rx_buf[NUM_UART_PORTS] [PORT_RX_BUF_SIZE], port_tx_buf [NUM_UART_PORTS][PORT_TX_BUF_SIZE];

    const struct sUartConfig *uartConfig = &(gUartConfig[channel]);
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    if(channel >= NUM_UART_PORTS || channel < 0){
        return -1;
    }

    /// initialize software circular buffers comm_buffers.c User com
    COM_buf_init(&gPort[channel], 
                 port_tx_buf[channel],
                 port_rx_buf[channel],    
                 PORT_RX_BUF_SIZE,
                 PORT_TX_BUF_SIZE);

    RCC_AHB1PeriphClockCmd(uartConfig->ahb1ClockEnable, ENABLE);
    RCC_APB1PeriphClockCmd(uartConfig->apb1ClockEnable, ENABLE);
    RCC_AHB2PeriphClockCmd(uartConfig->ahb2ClockEnable, ENABLE);
    RCC_APB2PeriphClockCmd(uartConfig->apb2ClockEnable, ENABLE);

    /// configure COM TX Pins
    GPIO_InitStructure.GPIO_Pin   = uartConfig->tx.pin;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(uartConfig->tx.port, &GPIO_InitStructure);
    GPIO_PinAFConfig(uartConfig->tx.port, uartConfig->tx.source, uartConfig->tx.altFun);

    /// configure COM RX Pins
    GPIO_InitStructure.GPIO_Pin = uartConfig->rx.pin;
    GPIO_Init(uartConfig->rx.port, &GPIO_InitStructure);
    GPIO_PinAFConfig(uartConfig->rx.port, uartConfig->rx.source, uartConfig->rx.altFun);

    /** USARTx configured as follow:
    - BaudRate = from port_hw
    - Word Length = 8 Bits, one stop bit, o parity, no flow control
    - Receive and transmit enabled
    */
    USART_InitStructure.USART_BaudRate            = baudrate;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(uartConfig->uart, &USART_InitStructure);
    USART_Cmd(uartConfig->uart, ENABLE);

    /// clear anything in the rx
    if (USART_GetFlagStatus(uartConfig->uart, USART_FLAG_RXNE)) {
        USART_ReceiveData(uartConfig->uart);
    }

    /// now for interrupts, only turn rx interrupts on
    USART_ITConfig( uartConfig->uart, USART_IT_RXNE, ENABLE );
	NVIC_InitStructure.NVIC_IRQChannel                   = uartConfig->irqChannel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x8 + channel; // low
        NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init( &NVIC_InitStructure );

    return 0;

} /* end function uart_init */

//@brief NO uart_read() the uart rx is handled in the irq

/** ****************************************************************************
 * @name uart_write
 * @brief
 *  Write the bytes in the circular buffer to the serial channel until the buffer
 *  is empty or the UART FIFO is full.  If there is more data than will fit in the
 *  FIFO the transmit interrupt will be enabled and the rest of the data will be sent
 *  by the interrupt routine.  A flag will be sent so that all further call to this
 *  routine will be ignored until the interrupt routine has completly emptied the
 *  transmit circular buffer.  In the future consider letting the interrupt routine
 *  move all the data to the uart FIFO.
*

 *           		circular buffer for transmit data and the interrupt flag
 *					to handshake with this routine.
 * @retval  N/A
 ******************************************************************************/
int uart_write(int channel, uint8_t *data, int len)
{
    if(channel == UART_CHANNEL_NONE){
        return 0;
    }
    int written = COM_buf_add(&gPort[channel].xmit_buf, data, (unsigned int)len);
    USART_ITConfig( gUartConfig[channel].uart, USART_IT_TXE, ENABLE );
    return written;
} /* end function uart_write */


int uart_read(int channel, uint8_t *data, int len)
{
    if(channel == UART_CHANNEL_NONE){
        return 0;
    }
    return COM_buf_get(&gPort[channel].rec_buf, data, len);
} /* end function uart_read */

int  uart_rxBytesAvailable(int channel)
{
    return gPort[channel].rec_buf.bytes_in_buffer;
}

void uart_flushRecBuffer(int channel)
{
    if(channel == UART_CHANNEL_NONE){
        return;
    }
    int num = gPort[channel].rec_buf.bytes_in_buffer;
    COM_buf_delete(&gPort[channel].rec_buf, num);
}

int uart_txBytesRemains(int channel)
{
    if(channel == UART_CHANNEL_NONE){
        return 0;
    }
    return gPort[channel].xmit_buf.bytes_in_buffer;
}

int uart_removeRxBytes(int channel , int num)
{
    if(channel == UART_CHANNEL_NONE){
        return 0;
    }
    return COM_buf_delete(&gPort[channel].rec_buf, num);
}

BOOL uart_copyBytes(int channel , int index, int num, uint8_t *output)
{
    if(channel == UART_CHANNEL_NONE){
        return FALSE;
    }
    return COM_buf_copy (&gPort[channel].rec_buf, index, num, output);
}


/** ****************************************************************************
 * @name uart_isr - common callback for the port specific handlers below
 * @brief  this routine will read the UART status register and if there was an
 *  interrupt generated from this UART port it will check for a transmit fifo level
 *  interrupt. If no interrupt was generated this routine will exit.  If there was
 *  another interrupt source the interrupt enable register will be cleared except
 *  for the FIFO level interrupt.  If a valid transmit fifo level interrupt has
 *  occured the fifo level will be read from the UART and compared to the number
 *  of bytes in the transmit buffer.  If there is more than will fit in the UART
 *  FIFO then only what will fit is move to a temporary buffer and then put in the
 *  FIFO, the interrupt remains active.  If there is less bytes in the buffer than
 *  space in the FIFO the buffer is emptied and the interrupt disabled.
 *
 * @param [in] channel- selects UART channel to read from, must be < MAX_CHANNEL
 * @param [in] *port  - pointer to the data structure for this UART port
 *              uses xmit buffer and the tx_int_flg
 * @param [out] port.xmit_buf.buf_outptr
 *					 port.hw.tx_int_flg
 * @retval N/A
 ******************************************************************************/
void uart_isr (unsigned int channel, port_struct  *port)
{
    uint8_t ch;
    uint8_t comBuffSuccess;
    uint8_t txCharacter;
    static volatile uint8_t drReg;
    static uint32_t lastTxClr = 0;
    uint32_t        timeNow   = 0;

    USART_TypeDef *uart = gUartConfig[channel].uart;

    if (USART_GetFlagStatus(uart, USART_FLAG_RXNE)) {
        ch = uart->DR;
        comBuffSuccess = COM_buf_add(&(port->rec_buf), &ch, 1);
  		if ( comBuffSuccess == 0) { // overflow
            if (channel == UART_CHANNEL_1)
                gBitStatus.comSABIT.bit.recBufOverflow = 1;
            else if  (channel == UART_CHANNEL_2) {
                gBitStatus.comSBBIT.bit.recBufOverflow = 1;
            }
        }
        if(uartRxSemIds[channel]){
			osSemaphoreRelease(uartRxSemIds[channel]);
        }
    }
    if (USART_GetFlagStatus(uart, USART_FLAG_TXE)) {
        if (COM_buf_bytes_available (&(port->xmit_buf))) {
            COM_buf_get(&(port->xmit_buf), // get a character
                        &txCharacter,
                        1);
            USART_SendData(uart,
                           txCharacter);
        } else { // circular buffer is empty
            USART_ITConfig( uart, USART_IT_TXE, DISABLE );
        }
    }

    if (channel == kUserB_UART) {
      timeNow = osKernelSysTick();
      timeNow /= 1000;
      drReg = uart->DR;
      if ((timeNow - lastTxClr) > 300) {
        uart->DR = drReg;
        lastTxClr = timeNow;
      }
    }

} /* end function uart_isr */

/** ****************************************************************************
 * @name USER_A_UART_IRQ uart A ISR - USER Com
 * @brief handle
 * @param [in] N/A
 * @retval N/A
 ******************************************************************************/
void USER_A_UART_IRQ()
{
    unsigned int channel;

    OSEnterISR();

    channel           = UART_CHANNEL_0;
    port_struct *port = &gPort[0];

    uart_isr (channel, port);
    
    OSExitISR();
}

/** ****************************************************************************
 * @name USER_B_UART_IRQ uart B ISR - GPS
 * @brief handle
 * @param [in] N/A
 * @retval N/A
 ******************************************************************************/
void USER_B_UART_IRQ()
{
    unsigned int channel;

    OSEnterISR();
    
    channel           = UART_CHANNEL_1;
    port_struct *port = &gPort[1];

    uart_isr (channel, port);

    OSExitISR();
}



/** ****************************************************************************
 * @name IsDebugSerialIdle
 * @brief Determines if the serial port is idle, only should be used when
 *        ouputting a large quantity of data as fast as possible
 * @param N/A
 * @retval True if serial port is idle
 *******************************************************************************/
int IsDebugSerialIdle()
{   // reverses the return so empty is true
    port_struct *port = &gPort[2];

    return (COM_buf_bytes_available(&port->xmit_buf) == 0) ? 1 : 0;
}

/** ****************************************************************************
 * @name DEBUG_USART_IRQ
 * @brief USART handles rx and tx interrupts
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void DEBUG_USART_IRQ()
{
    unsigned int channel;

    OSEnterISR();
    
    channel           = UART_CHANNEL_2;
    port_struct *port = &gPort[2];

    uart_isr (channel, port);

    OSExitISR();
/*


    uint8_t ch;
    OSEnterISR();

    port_struct *port = &gPort[2];

    //volatile int ulDummy = OSEnDisableIntFromISR();

//FIXME if protection is needed it should be a mutex
//    OSDisableHook(); /// to disable interrupts

    /// Check the state of the "receive data register not empty" flag. Set binary
    ///   semaphore when detected
    if (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_RXNE)) {
        ch = DEBUG_USART->DR;
        if(cliSem){
			COM_buf_add(&port->rec_buf, &ch, 1);
			osSemaphoreRelease(cliSem);
        }
    }

    /// Check the state of the "transmit data register empty" flag
    if (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE)) {
          if ( IsDebugSerialIdle() ) {
            USART_ITConfig( DEBUG_USART, USART_IT_TXE, DISABLE );
        } else {
            COM_buf_get(&port->xmit_buf, &ch, 1);
            USART_SendData(DEBUG_USART, ch);
        }
    }

    OSExitISR();
*/
}




/** ****************************************************************************
 * @name bytes_remaining
 * @brief
 * -returns the total amount of bytes remaing in the transmit buffer and FIFO
 *
 * Trace:  [SDD_EXT_PORT_DRAIN <-- SRC_UART_BYTES_REMAINING]
 * Trace:  [SDD_EXT_PORT_DRAIN_WD <-- SRC_UART_BYTES_REMAINING]
 *
 * @param [in] channel - UART channel #, range 0-3
 * @param [in] port_struct *port - pointer to port structure
 * @retval  byte_total	-integer number of total bytes remaining to be sent
 *******************************************************************************/
unsigned int bytes_remaining(unsigned int channel,
                             port_struct  *port)
{
	unsigned int byte_total;

    byte_total = COM_buf_bytes_available (&(port->xmit_buf));
    if (byte_total == 0) {
        if (!(USART_GetFlagStatus(gUartConfig[channel].uart, USART_FLAG_TXE))) {
            byte_total++;
        }
    }

   	return byte_total;
}

/** ****************************************************************************
 * @name uart_BIT - read the serial port register status flags and pass to BIT
 * @param [in] channel- selects UART channel to read from, must be < MAX_CHANNEL
 * @retval N/A
 ******************************************************************************/
void uart_BIT (int uartType)
{
    int uartChan = platformGetSerialChannel(uartType);
    USART_TypeDef *uart;

    gBitStatus.comSABIT.bit.breakDetect  = 0;
    gBitStatus.comSABIT.bit.framingError = 0;
    gBitStatus.comSABIT.bit.parityError  = 0;

    gBitStatus.comSBBIT.bit.breakDetect  = 0;
    gBitStatus.comSBBIT.bit.framingError = 0;
    gBitStatus.comSBBIT.bit.parityError  = 0;

    if (uartType == USER_SERIAL_PORT) {// user
        uart = gUartConfig[uartChan].uart;
        gBitStatus.comSABIT.bit.breakDetect  = USART_GetFlagStatus(uart, USART_FLAG_LBD);
        gBitStatus.comSABIT.bit.framingError = USART_GetFlagStatus(uart, USART_FLAG_FE);
        gBitStatus.comSABIT.bit.parityError  = USART_GetFlagStatus(uart, USART_FLAG_PE);
    }
    else if (uartType == GPS_SERIAL_PORT) { // GPS
        uart = gUartConfig[uartChan].uart;
        gBitStatus.comSBBIT.bit.breakDetect  = USART_GetFlagStatus(uart, USART_FLAG_LBD);
        gBitStatus.comSBBIT.bit.framingError = USART_GetFlagStatus(uart, USART_FLAG_FE);
        gBitStatus.comSBBIT.bit.parityError  = USART_GetFlagStatus(uart, USART_FLAG_PE);
   }

} /* end function uart_BIT */


void uart_IrqOnOff(unsigned int uartChannel, uint16_t uartIrqType, FunctionalState en)
{
  USART_TypeDef *uart = gUartConfig[uartChannel].uart;

  switch (uartIrqType)
  {
      case USART_IT_TXE:
          USART_ITConfig( uart, USART_IT_TXE, en );
          break;
      case USART_IT_RXNE:
          USART_ITConfig( uart, USART_IT_RXNE, en );
          break;
      default:
          break;
  }
}

void uart_Pause()
{
    uart_IrqOnOff(1, USART_IT_RXNE, DISABLE);
    OS_Delay(600);
    uart_IrqOnOff(1, USART_IT_RXNE, ENABLE);
}