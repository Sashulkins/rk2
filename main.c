


#include "stm32f0xx.h"




void Tim3_Init()
{
   RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
   TIM3->PSC = 720 - 1;
   TIM3->ARR  = 100;
   TIM3->CR2 |= TIM_CR2_MMS_1;
   TIM3->CR1 |= TIM_CR1_CEN;
}





uint8_t data[256];
void init_usart_dma() {
	for (int i = 0; i < 256; i++) {
		data[i] = i;
	}

	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	uint8_t data[256];
	USART1->CR1 |= USART_CR1_RE;
	USART1->CR1 |= USART_CR1_TE;
	USART1->CR2 &= ~USART_CR2_STOP;
	USART1->BRR = SystemCoreClock;
	USART1->CR3 |= USART_CR3_DMAT;
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
		DMA1_Channel2->CPAR = (uint32_t)(&(USART1->TDR));
		DMA1_Channel2->CMAR = (uint32_t)(&data[0]);
		DMA1_Channel2->CNDTR = 256;
		DMA1_Channel2->CCR |= DMA_CCR_TCIE;
		DMA1_Channel2->CCR |= DMA_CCR_DIR;
		DMA1_Channel2->CCR |= DMA_CCR_MINC;


		NVIC_SetPriority(DMA1_Channel2_3_IRQn, 9);
		NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);


		DMA1_Channel2->CCR |= DMA_CCR_EN;

		USART1->CR1 |= USART_CR1_UE;
}


void DMA1_Channel1_IRQHandler() {
	if ((DMA1->ISR & DMA_ISR_HTIF1) == DMA_ISR_HTIF1) {
		GPIOC->BSRR = GPIO_BSRR_BS_8;
		DMA1->IFCR |= DMA_IFCR_CHTIF1;
	}
}

void init_usart_dma() {

	USART1->CR3 |= USART_CR3_DMAT;
}

void USART1_IRQHandler() {
	if ((USART1->ISR & USART_ISR_TXE) == USART_ISR_TXE) {
		static uint8_t byte = 0;
		USART1->TDR = byte++;
	}

	if ((USART1->ISR & USART_ISR_RXNE) == USART_ISR_RXNE) {
		byteRx = USART1->RDR;
		bufferPutToEnd();
	}
}

void adc_start() {
	ADC1->CR |= ADC_CR_ADSTART;
}

void adc_stop() {
	ADC1->CR |= ADC_CR_ADSTP;
}


void adc_init_cont() {
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	RCC->CR2 |= RCC_CR2_HSI14ON;

	ADC1->CHSELR = ADC_CHSELR_CHSEL0;
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER |= GPIO_MODER_MODER0
					GPIO_MODER_MODER1;

	ADC1->IER |= ADC_IER_EOCIE;
	ADC1->IER |= ADC_IER_EOSEQIE;
	NVIC_SetPriority(ADC1_COMP_IRQn, 5);
	NVIC_EnableIRQ(ADC1_COMP_IRQn);

	ADC1->CFGR1 |= ADC_CFGR1_CONT;

	ADC1->CR |= ADC_CR_ADEN;
}

void DMA1_Channel2_3_IRQHandler(void) {

	if ( (DMA1->ISR & DMA_ISR_TCIF2) == DMA_ISR_TCIF2)  {
		DMA1->IFCR |= DMA_IFCR_CTCIF2;

	}
}




int main(void)
{
	init_usart_dma() ;
	Tim3_Init();
	adc_init_cont();
	init_usart_dma();
   ADC1->CR |= ADC_CR_ADSTART;
   ADC1->CR2 &= ~(ADC_CR2_JEXTSEL_0 | ADC_CR2_JEXTSEL_2);
   ADC1->CR2 |= ADC_CR2_JEXTSEL_1;
   ADC1->CR2 |= ADC_CR2_JEXTTRIG;


   ADC1->CR1 |= ADC_CR1_EOCIE;

   	ADC1->CFGR1 |= ADC_CFGR1_DMAEN;
   	ADC1->CFGR1 |= ADC_CFGR1_OVRMOD;
   	RCC->AHBENR |= RCC_AHBENR_DMAEN;
   	DMA1_Channel1->CCR |= DMA_CCR_HTIE | DMA_CCR_TCIE;
   	DMA1_Channel1->CCR |= DMA_CCR_MSIZE_0;
   	DMA1_Channel1->CCR |= DMA_CCR_PSIZE_0;
   	DMA1_Channel1->CCR |= DMA_CCR_MINC;
   	DMA1_Channel1->CCR &= ~DMA_CCR_DIR;
   	DMA1_Channel1->CCR |= DMA_CCR_CIRC;

   	DMA1_Channel1->CMAR = (uint32_t)(&adc_data[0]);
   	DMA1_Channel1->CPAR = (uint32_t)(&(ADC1->DR));
   	DMA1_Channel1->CNDTR = SIZE_ADC;

  // 	NVIC_SetPriority(DMA1_Ch1_IRQn, 10);
 //  	NVIC_EnableIRQ(DMA1_Ch1_IRQn);

   	DMA1_Channel1->CCR |= DMA_CCR_EN;
  while (1)
  {

  }


}
