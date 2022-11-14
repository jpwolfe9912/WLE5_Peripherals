/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    i2c.c
 * @brief   This file provides code for the configuration
 *          of the I2C instances.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

/* USER CODE BEGIN 0 */
#define LCD_ADDRESS 0x27
#define I2C2_TX_DMA_REQUEST 0xE
#define I2C2_RX_DMA_REQUEST 0xD

volatile uint8_t tx_finished;
volatile uint8_t rx_finished;
/* USER CODE END 0 */

/* USER CODE BEGIN 1 */
void i2c2Init(void)
{
    /* DMA Clock Enable */
    RCC->AHB1ENR |= RCC_AHB1ENR_DMAMUX1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    /* DMA1_Channel2_IRQn interrupt configuration */
    NVIC_SetPriority(DMA1_Channel2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    /* DMA1_Channel3_IRQn interrupt configuration */
    NVIC_SetPriority(DMA1_Channel3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(DMA1_Channel3_IRQn);

    RCC->CCIPR &= ~RCC_CCIPR_I2C2SEL;       // PCLK source for I2C2
    
    /* Enable GPIO Clock */
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

    GPIOA->MODER &= ~GPIO_MODER_MODE15;
    GPIOA->MODER |= GPIO_MODER_MODE15_1;     // AF mode
    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED15; // very high speed
    GPIOA->OTYPER |= GPIO_OTYPER_OT15;       // open drain
    GPIOA->PUPDR |= GPIO_PUPDR_PUPD15_0;     // pull up
    GPIOA->AFR[1] &= GPIO_AFRH_AFSEL15;
    GPIOA->AFR[1] |= (4U << GPIO_AFRH_AFSEL15_Pos); // AF4

    GPIOB->MODER &= ~GPIO_MODER_MODE15;
    GPIOB->MODER |= GPIO_MODER_MODE15_1;     // AF mode
    GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED15; // very high speed
    GPIOB->OTYPER |= GPIO_OTYPER_OT15;       // open drain
    GPIOB->PUPDR |= GPIO_PUPDR_PUPD15_0;     // pull up
    GPIOB->AFR[1] &= GPIO_AFRH_AFSEL15;
    GPIOB->AFR[1] |= (4U << GPIO_AFRH_AFSEL15_Pos); // AF4

    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;

    /* TX DMA Configuration */
    // Set I2C2_RX as Channel 2 Request
    DMAMUX1_Channel1->CCR &= ~DMAMUX_CxCR_DMAREQ_ID;
    DMAMUX1_Channel1->CCR |= (I2C2_TX_DMA_REQUEST << DMAMUX_CxCR_DMAREQ_ID_Pos);
    DMA1_Channel2->CCR |= DMA_CCR_DIR;    // mem to per
    DMA1_Channel2->CCR &= ~DMA_CCR_PL;    // low priority
    DMA1_Channel2->CCR &= ~DMA_CCR_CIRC;  // normal mode
    DMA1_Channel2->CCR &= ~DMA_CCR_PINC;  // no per inc
    DMA1_Channel2->CCR |= DMA_CCR_MINC;   // mem inc
    DMA1_Channel2->CCR &= ~DMA_CCR_PSIZE; // per size byte
    DMA1_Channel2->CCR &= ~DMA_CCR_MSIZE; // mem size byte
    DMA1_Channel2->CPAR = (uint32_t)(&(I2C2->TXDR));
    DMA1_Channel2->CCR |= DMA_CCR_TCIE;
    /* RX DMA Configuration */
    // Set I2C2_TX as Channel 3 Request
    DMAMUX1_Channel2->CCR &= ~DMAMUX_CxCR_DMAREQ_ID;
    DMAMUX1_Channel2->CCR |= (I2C2_RX_DMA_REQUEST << DMAMUX_CxCR_DMAREQ_ID_Pos);
    DMA1_Channel3->CCR &= ~DMA_CCR_DIR;   // per to mem
    DMA1_Channel3->CCR &= ~DMA_CCR_PL;    // low priority
    DMA1_Channel3->CCR &= ~DMA_CCR_CIRC;  // normal mode
    DMA1_Channel3->CCR &= ~DMA_CCR_PINC;  // no per inc
    DMA1_Channel3->CCR |= DMA_CCR_MINC;   // mem inc
    DMA1_Channel3->CCR &= ~DMA_CCR_PSIZE; // per size byte
    DMA1_Channel3->CCR &= ~DMA_CCR_MSIZE; // mem size byte
    DMA1_Channel3->CPAR = (uint32_t)(&(I2C2->RXDR));
    DMA1_Channel3->CCR |= DMA_CCR_TCIE;

    /* I2C2 Initialization */
    I2C2->CR2 |= I2C_CR2_AUTOEND;
    I2C2->OAR2 &= ~I2C_OAR2_OA2EN;
    I2C2->CR1 &= ~I2C_CR1_GCEN;
    I2C2->CR1 &= ~I2C_CR1_NOSTRETCH;
    I2C2->CR1 &= ~I2C_CR1_PE;
    I2C2->CR1 &= ~I2C_CR1_ANFOFF;
    I2C2->CR1 &= ~I2C_CR1_DNF;
    I2C2->TIMINGR = 0x20303E5D;
    I2C2->CR1 |= I2C_CR1_PE;
    I2C2->OAR1 &= ~I2C_OAR1_OA1EN;
    I2C2->CR2 &= ~I2C_CR2_NACK;
    I2C2->CR1 |= I2C_CR1_TXDMAEN;
}

/**
 * @brief Sends data over I2C2 using DMA
 * @param data
 */
void i2c2CommandDMA(uint8_t data)
{

    DMA1_Channel2->CCR &= ~DMA_CCR_EN; // disable channel
    DMA1_Channel2->CNDTR = 1;
    DMA1_Channel2->CMAR = (uint32_t)&data;
    DMA1_Channel2->CCR |= DMA_CCR_EN;

    I2C2->CR2 |= (LCD_ADDRESS << 1U);
    I2C2->CR2 &= ~I2C_CR2_RD_WRN; // write operation
    I2C2->CR2 &= ~I2C_CR2_NBYTES;
    I2C2->CR2 |= (1 << 16U);
    I2C2->CR2 |= I2C_CR2_AUTOEND;
    I2C2->CR2 |= I2C_CR2_START;

    while (!tx_finished)
        ;
    tx_finished = 0;
}

void i2c2WriteMemDMA(uint8_t *pData, uint8_t memadd, uint8_t size)
{
    uint8_t combinedData[size + 1];

    combinedData[0] = memadd;
    memcpy((combinedData + 0x1), pData, size);

    DMA1_Channel2->CCR &= ~DMA_CCR_EN; // disable channel
    DMA1_Channel2->CNDTR = 1;
    DMA1_Channel2->CMAR = (uint32_t)&combinedData;
    DMA1_Channel2->CCR |= DMA_CCR_EN;

    I2C2->CR1 |= I2C_CR1_TXDMAEN;
    I2C2->CR2 |= (LCD_ADDRESS << 1U);
    I2C2->CR2 &= ~I2C_CR2_RD_WRN; // write operation
    I2C2->CR2 &= ~I2C_CR2_NBYTES;
    I2C2->CR2 |= (1 << 16U);
    I2C2->CR2 |= I2C_CR2_AUTOEND;
    I2C2->CR2 |= I2C_CR2_START;

    while (!tx_finished)
        ;
    tx_finished = 0;
}

void i2c2ReadMemoryDMA(uint8_t *pData, uint8_t memadd, uint8_t size)
{
    I2C2->CR2 |= (LCD_ADDRESS << 1U);
    I2C2->CR2 &= ~I2C_CR2_RD_WRN; // write operation
    I2C2->CR2 &= ~I2C_CR2_NBYTES;
    I2C2->CR2 |= (1 << I2C_CR2_NBYTES_Pos);
    I2C2->CR2 |= I2C_CR2_AUTOEND;
    I2C2->CR2 |= I2C_CR2_START;

    while (!(I2C2->ISR & I2C_ISR_TXIS))
        ;
    I2C2->TXDR = (uint32_t)memadd;
    while (!(I2C2->ISR & I2C_ISR_TXE))
        ;

    /* Set up DMA Receiver*/
    DMA1_Channel3->CCR &= ~DMA_CCR_EN;
    DMA1_Channel3->CNDTR = size;
    DMA1_Channel3->CMAR = (uint32_t)pData;
    DMA1_Channel3->CCR |= DMA_CCR_EN;

    I2C2->CR1 |= I2C_CR1_RXDMAEN;
    I2C2->CR2 |= (LCD_ADDRESS << 1U);
    I2C2->CR2 |= I2C_CR2_RD_WRN; // write operation
    I2C2->CR2 &= ~I2C_CR2_NBYTES;
    I2C2->CR2 |= (size << I2C_CR2_NBYTES_Pos);
    I2C2->CR2 |= I2C_CR2_AUTOEND;
    I2C2->CR2 |= I2C_CR2_START;

    while (!rx_finished)
        ;
    rx_finished = 0;
}

void DMA1_Channel2_IRQHandler(void)
{
    if (DMA1->ISR & DMA_ISR_TCIF2)
    {
        DMA1->IFCR |= DMA_IFCR_CTCIF2;
        tx_finished = 1;
    }
}

void DMA1_Channel3_IRQHandler(void)
{
    if (DMA1->ISR & DMA_ISR_TCIF3)
    {
        DMA1->IFCR |= DMA_IFCR_CTCIF3;
        rx_finished = 1;
    }
}
/* USER CODE END 1 */
