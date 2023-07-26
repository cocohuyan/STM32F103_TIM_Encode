#ifndef _UART_H_
#define _UART_H_

#include "stm32f10x.h"
#include "stm32f10x_usart.h"

void USART_Configuration(void);
void USART1_Receive(uint8_t chbyte);
void USART1_Putc(const uint8_t c);
void USART1_SendBuf(const uint8_t* u8buf, const uint16_t u16len);

#endif /* _UART_H_ */
