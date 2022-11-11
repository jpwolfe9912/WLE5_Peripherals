#include "usart.h"
/** @brief Initializes the low level uart registers in order to use printf.
*
*  @return Void.
*/
void
usart1Init(void)
{

	RCC->AHB2ENR 	|= RCC_AHB2ENR_GPIOBEN; 						// enable the clock for port B
	RCC->APB2ENR 	|= RCC_APB2ENR_USART1EN; 						// enable the clock for USART1

	GPIOB->AFR[0] |= (0x7 << (4 * 6U));							// set pin A2 as alternate function
	GPIOB->AFR[0] |= (0x7 << (4 * 7U));							// set pin A3 as alternate function

	GPIOB->MODER &= ~(GPIO_MODER_MODE6);
	GPIOB->MODER &= ~(GPIO_MODER_MODE7);
	GPIOB->MODER |= GPIO_MODER_MODE6_1;						// set PC12 as alternate function
	GPIOB->MODER |= GPIO_MODER_MODE7_1;						// set PD2 as alternate function

	NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
	NVIC_EnableIRQ(USART1_IRQn);

	USART1->BRR = 0x1A1; 										// set baud rate to 115200
	USART1->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_UE; 	// enable the receiver, transmitter, and USART								// enable USART1 interrupts on the NVIC (nested vector interrupt controller)


}

/** @brief Uses polling to write data to the transmit buffer.
 *
 *  @param ch The character to send.
 *  @return Void.
 */
void
serialWrite(uint8_t ch)
{
	while (!(USART1->ISR & USART_ISR_TC)){}	// waits for TX buffer to become empty
	USART1->TDR = ch;						// transfers the value of the data register into ch
}

/** @brief Uses interrupts to read uint8 data to the receive buffer.
*
*  @param uint8_t *num Pointer to the location you want to store the received number
*  @return Void.
*/
void
serialRead8(uint8_t *num)
{
	temp = '\0';
	USART1->CR1 |= USART_CR1_RXNEIE;
	while(!temp);

	*num = (uint8_t)temp - 48;
	USART1->CR1 &= ~USART_CR1_RXNEIE;
}

/** @brief Uses interrupts to read a string of PID values.
*
*  @param * Pointer to the location you want to store the received number
*  @return Void.
*/
void
serialReadPID(float *P, float *I, float *D)
{
	serialIndex = 0;
	memset(serialBuf, '\0', sizeof(serialBuf));

	endOfString = false;
	USART1->CR1 |= USART_CR1_RXNEIE;
	while(!endOfString);

	sscanf((char*)serialBuf, "%f, %f, %f", P, I, D);

	USART1->CR1 &= ~USART_CR1_RXNEIE;
	memset(serialBuf, '\0', sizeof(serialBuf));
}

/** @brief Waits for a character.
*
*  @param wait Character to wait for.
*  @return bool True or False based on whether or not the character.
*  received is the input to the function.
*/
bool
serialWaitFor(char wait)
{
	temp = '\0';
	serialIndex = 0;

	USART1->CR1 |= USART_CR1_RXNEIE;
	while(!temp);
	if(temp == wait){
		temp = '\0';
		USART1->CR1 &= ~USART_CR1_RXNEIE;
		return true;
	}
	else{
		temp = '\0';
		USART1->CR1 &= ~USART_CR1_RXNEIE;
		return false;
	}
}

/* Interrupt Handlers */

/**
* @brief This function handles USART1 global interrupt.
*/
void
USART1_IRQHandler(void)
{
	if((USART1->ISR & USART_ISR_RXNE) &&(USART1->CR1 & USART_CR1_RXNEIE)){
		temp = USART1->RDR;
		USART1->TDR = temp;
		if(temp == '\r'){
			endOfString = true;
			serialIndex = 0;
		}
		else{
			serialBuf[serialIndex] = temp;
			serialIndex++;
		}

	}
	if(USART1->ISR & USART_ISR_ORE)
		USART1->ICR |= USART_ICR_ORECF;
}