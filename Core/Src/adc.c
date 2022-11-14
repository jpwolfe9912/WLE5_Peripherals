/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    adc.c
 * @brief   This file provides code for the configuration
 *          of the ADC instances.
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
#include "adc.h"

/* USER CODE BEGIN 0 */
uint16_t rawAdc;
bool convCplt = false;

static void adcEnableIntReg(void);
static uint8_t adcCalibrate(void);
/* USER CODE END 0 */

/* ADC init function */
void MX_ADC_Init(void)
{

    /* USER CODE BEGIN ADC_Init 0 */
    RCC->AHB1ENR |= RCC_AHB1ENR_DMAMUX1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    /* DMA1_Channel1_IRQn interrupt configuration */
    NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

    GPIOB->MODER &= ~GPIO_MODER_MODE4;
    GPIOB->MODER |= GPIO_MODER_MODE4;
    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD4;

    DMAMUX1_Channel0->CCR &= ~DMAMUX_CxCR_DMAREQ_ID;
    DMAMUX1_Channel0->CCR |= (ADC_DMA_REQUEST << DMAMUX_CxCR_DMAREQ_ID_Pos);
    DMA1_Channel1->CCR &= ~DMA_CCR_DIR;   // per to mem
    DMA1_Channel1->CCR &= ~DMA_CCR_PL;    // low priority
    DMA1_Channel1->CCR &= ~DMA_CCR_CIRC;  // normal mode
    DMA1_Channel1->CCR &= ~DMA_CCR_PINC;  // no per inc
    DMA1_Channel1->CCR |= DMA_CCR_MINC;   // mem inc
    DMA1_Channel1->CCR &= ~DMA_CCR_PSIZE; // per size byte
    DMA1_Channel1->CCR &= ~DMA_CCR_MSIZE; // mem size byte
    DMA1_Channel1->CPAR = (uint32_t)(&(ADC->DR));
    DMA1_Channel1->CMAR = (uint32_t)(&rawAdc);
    DMA1_Channel1->CNDTR = 1U;
    DMA1_Channel1->CCR |= DMA_CCR_TCIE;
    DMA1_Channel1->CCR |= DMA_CCR_EN;

    /* Configure ADC */
    /* Configure NVIC to enable ADC interruptions */
    NVIC_SetPriority(ADC_IRQn, 0); /* ADC IRQ greater priority than DMA IRQ */
    NVIC_EnableIRQ(ADC_IRQn);

    ADC->CR &= ~ADC_CR_ADEN;           // turn off ADC
    ADC->CFGR2 |= ADC_CFGR2_CKMODE_0;  // PCLK/2
    ADC->CFGR1 &= ~ADC_CFGR1_RES;      // 12 bit res
    ADC->CFGR1 &= ~ADC_CFGR1_ALIGN;    // right alignment
    ADC->CFGR1 &= ~ADC_CFGR1_AUTOFF;   // no lp mode
    ADC->CFGR1 |= ADC_CFGR1_CHSELRMOD; // channels configurable

    while (!(ADC->ISR & ADC_ISR_CCRDY))
        ;                      // wait for flag
    ADC->ISR |= ADC_ISR_CCRDY; // write to clear

    ADC->CFGR1 &= ~ADC_CFGR1_EXTEN;  // software trigger
    ADC->CFGR1 &= ~ADC_CFGR1_DISCEN; // disc mode off
    ADC->CFGR1 &= ~ADC_CFGR1_CONT;   // cont mode off
    ADC->CFGR1 &= ~ADC_CFGR1_DMACFG; // one shot mode
    ADC->CFGR1 &= ~ADC_CFGR1_DMAEN;  // disable dma mode
    ADC->CFGR1 &= ~ADC_CFGR1_OVRMOD; // preserve overrun data

    adcEnableIntReg();

    ADC->CFGR2 &= ~ADC_CFGR2_OVSE;               // disable oversampling
    ADC->SMPR &= ~ADC_SMPR_SMP1;                 // 1.5 clock cycles
    ADC->SMPR &= ~ADC_SMPR_SMP2;                 // 1.5 clock cycles
    ADC->IER &= ~ADC_IER_EOCIE;                  // disable EOC IT
    ADC->IER &= ~ADC_IER_EOSIE;                  // disable EOS IT
    ADC->CFGR2 &= ~ADC_CFGR2_LFTRIG;             // high freq trigger
    ADC->CHSELR |= (0x3 << ADC_CHSELR_SQ1_Pos);  // channel 3 as 1st conversion
    ADC->CHSELR |= (0xFF << ADC_CHSELR_SQ2_Pos); // 0xFF as next selection to indicate stop
    while (!(ADC->ISR & ADC_ISR_CCRDY))
        ; // wait for ADC to be ready
    ADC->ISR |= ADC_ISR_CCRDY;
    ADC->SMPR &= ~ADC_SMPR_SMPSEL3; // choose selection 1
    ADC->IER |= ADC_IER_OVRIE;      // enable OVR IT

    while (!(ADC->ISR & ADC_ISR_ADRDY))
        ;                          // wait for ADC to be ready
    adcEnableIntReg();
    ADC->CFGR1 &= ~ADC_CFGR1_DMAEN; // disable DMA

    adcCalibrate();
    ADC->CFGR1 |= ADC_CFGR1_DMAEN; // enable DMA
    delayMicroseconds(64);         // wait between calibration and enable
    ADC->ISR |= ADC_ISR_ADRDY;
    ADC->CR |= ADC_CR_ADEN; // enable ADC
    while (!(ADC->ISR & ADC_ISR_ADRDY))
        ; // wait for ADC to be ready
    /* USER CODE BEGIN ADC_Init 2 */

    /* USER CODE END ADC_Init 2 */
}

/* USER CODE BEGIN 1 */
void adcStartConv(void)
{
    if ((ADC->CR & ADC_CR_ADEN) &&
        !(ADC->CR & ADC_CR_ADDIS) &&
        !(ADC->CR & ADC_CR_ADSTART))
    {
        ADC->CR |= ADC_CR_ADSTART;
    }
}

static void adcEnableIntReg(void)
{
    ADC->CR &= ~ADC_CR_ADCAL;
    ADC->CR &= ~ADC_CR_ADDIS;
    ADC->CR &= ~ADC_CR_ADEN;
    ADC->CR &= ~ADC_CR_ADSTART;
    ADC->CR &= ~ADC_CR_ADSTP;
    
    ADC->CR |= ADC_CR_ADVREGEN;
    delayMicroseconds(20);      // wait for regulator to stabilize
}

static uint8_t adcCalibrate(void)
{
    ADC->CR &= ~ADC_CR_ADCAL;
    ADC->CR &= ~ADC_CR_ADDIS;
    ADC->CR &= ~ADC_CR_ADEN;
    ADC->CR &= ~ADC_CR_ADSTART;
    ADC->CR &= ~ADC_CR_ADSTP;

    ADC->CR |= ADC_CR_ADCAL;
    while (ADC->CR & ADC_CR_ADCAL)
        ; // wait for cal to stop
    return (uint8_t)(ADC->CALFACT);
}

void DMA1_Channel1_IRQHandler(void)
{
    if (DMA1->ISR & DMA_ISR_TCIF1)
    {
        DMA1->IFCR |= DMA_IFCR_CGIF1;
        convCplt = true;
    }
}

void ADC_IRQHandler(void)
{
    if (ADC->ISR & ADC_ISR_OVR)
        ADC->ISR |= ADC_ISR_OVR;
}
/* USER CODE END 1 */
