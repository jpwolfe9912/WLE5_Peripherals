#ifndef __USART_H__
#define __USART_H__

#include "main.h"

extern uint8_t rxBuf;

void usart1Init(void);
void serialWrite(uint8_t ch);
void serialBeginRead8(void);

#endif /* __USART_H__ */

