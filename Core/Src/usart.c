/* Includes */
#include "usart.h"

/* Global Variables */
uint8_t rxBuf;
bool rxEvent = false;

/* Function Prototypes */
static void usart1ReceiveCb(void);

/* Functions */
/** @brief Initializes the low level uart registers in order to use printf.
*
*  @return Void.
*/
void usart1Init(void)
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
	USART1->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_UE; 	// enable the receiver, transmitter, and USART								
    // enable USART1 interrupts on the NVIC (nested vector interrupt controller)
}

/** @brief Uses polling to write data to the transmit buffer.
 *
 *  @param ch The character to send.
 *  @return Void.
 */
void serialWrite(uint8_t ch)
{
	while (!(USART1->ISR & USART_ISR_TC)){}	// waits for TX buffer to become empty
	USART1->TDR = ch;						// transfers the value of the data register into ch
}

void serialBeginRead8(void)
{
    USART1->ICR |= USART_ICR_ORECF;
    USART1->CR1 |= USART_CR1_RXNEIE_RXFNEIE;
}

static void usart1ReceiveCb(void)
{
    rxBuf = USART1->RDR;
	rxEvent = true;
}

/* Interrupt Handlers */

/**
* @brief This function handles USART1 global interrupt.
*/
void USART1_IRQHandler(void)
{
	if((USART1->ISR & USART_ISR_RXNE_RXFNE) &&(USART1->CR1 & USART_CR1_RXNEIE_RXFNEIE))    // check for interrupt and that it's enabled
		usart1ReceiveCb();
	if(USART1->ISR & USART_ISR_ORE)
		USART1->ICR |= USART_ICR_ORECF;
}

/*	This is required to use printf											*/
/*	This basically tells the compiler what to do when it encounters printf	*/
/*	I honestly can't fully explain what is going on but it works			*/

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
//	#define GETCHAR_PROTOTYPE int __io_getchar (void)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
//	#define GETCHAR_PROTOTYPE int fgetc(FILE * f)
#endif

PUTCHAR_PROTOTYPE{
	serialWrite(ch);
	return ch;
}