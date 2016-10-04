/*
  ******************************************************************************
  070253 - Circuitos Microprocessados
  Prof. Lucio Rene Prade
  Sexto Exemplo  - Interrupções
  ******************************************************************************
 */

#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Struct de configuração da comunicação serial definido em  stm32f0xx_hal_uart.h*/
UART_HandleTypeDef UartHandle;

/* Struct definida em  stm32f0xx_hal_gpio.h para configrar o port*/
static GPIO_InitTypeDef  GPIO_InitStruct;

/* Buffer de Transmissão  */
uint8_t Buffer[BUFFERSIZE] = " **** Inicio do Programa ****\n\r ";
uint8_t BufferC[BUFFERSIZE] = {};


static void SystemClock_Config(void);
static void Error_Handler(void);

volatile uint16_t click1 = 0;
volatile uint16_t click2 = 0;
volatile uint16_t click3 = 0;
volatile int cont = 0;
volatile int tcont = 0;
volatile int ttemp = 0;

int main(void) {


  /* Inicializa as bibliotecas HAL */
  HAL_Init();

  /* Configura o clock do sistema */
  SystemClock_Config();

  /* Habilita o clock no port onde está o pino que possui função de TX e RX */
   __GPIOA_CLK_ENABLE();
   __GPIOC_CLK_ENABLE();

   /* Configura  Botao 1 */
     GPIO_InitStruct.Mode  = GPIO_MODE_IT_RISING;
     GPIO_InitStruct.Pull  = GPIO_PULLUP;
     GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
     GPIO_InitStruct.Pin = GPIO_PIN_8;

     HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

     /* Configura  Botao 2 */
     GPIO_InitStruct.Mode  = GPIO_MODE_IT_RISING;
     GPIO_InitStruct.Pull  = GPIO_PULLUP;
     GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
     GPIO_InitStruct.Pin = GPIO_PIN_7;

     HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//     /* Configura  o pino do botão como um pino de interrupção externa*/
//      GPIO_InitStruct.Mode  = GPIO_MODE_IT_RISING;
//      GPIO_InitStruct.Pull  = GPIO_NOPULL;
//      GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
//      GPIO_InitStruct.Pin   = GPIO_PIN_1;
//
//      HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

 //    PA2     ------> USART2_TX
 //    PA3     ------> USART2_RX

  /*Configura os pinos GPIOs  para a função alternativa de RX e TX */
  GPIO_InitStruct.Pin       = GPIO_PIN_2|GPIO_PIN_3;  // Pino de TX verificar no  esquemático
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_USART2;

  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


  /* habilita o clock da  USART2 */
  __USART2_CLK_ENABLE();


  /*Configuração do Periférico USART
   	  - modo assincrono (UART Mode)
      - Word  = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = None
      - BaudRate = 9600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  UartHandle.Instance        = USART2;
  UartHandle.Init.BaudRate   = 9600;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX_RX;

  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    Error_Handler();
  }

   /* Configura  o pino do led como output push-pull */
   GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
   GPIO_InitStruct.Pin 	= GPIO_PIN_5;
   GPIO_InitStruct.Alternate = 0;

   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


   /* Habilita o Clock no port do botão,  função definida em stm32f0xx_hal_rcc.h*/
     __GPIOC_CLK_ENABLE();

   /* Configura  o pino do botão como um pino de interrupção externa*/
    GPIO_InitStruct.Mode  = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Pin   = GPIO_PIN_13;

    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Habilita e conigura a priodidade da interrupção  EXTI line 4_15  */
    HAL_NVIC_SetPriority(EXTI4_15_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);


    /* Envia a mensagem de inicio */
    HAL_UART_Transmit(&UartHandle, (uint8_t*)Buffer, BUFFERSIZE, 5000);

   /* Infinite loop */
    while (1){
    	if (ttemp == 0){
//			HAL_UART_Transmit(&UartHandle, BufferC, BUFFERSIZE , 30);
//		    HAL_UART_Transmit(&UartHandle, " ", 1 , 30);
    		ttemp = 1;
    	}
    }
}

// Declaração Função de tratamento da Interrupção
//Esta função vai ser chamada quando ocorrer o evento de interrupção
//
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
  if (GPIO_Pin == GPIO_PIN_8){
	  if(click1 == 0){
		 debounce_pino8(0);
		 if (cont <= 254 ){
			 cont = cont + 1;
			 itoa (cont, BufferC, 10);
			 HAL_UART_Transmit(&UartHandle, BufferC, BUFFERSIZE , 30);
			 HAL_UART_Transmit(&UartHandle, " ", 1 , 30);
			 ttemp = 0;
		 }
	  }

	  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == 1 && click1 == 1){
	   	 debounce_pino8(1);
	  }
  }

  if (GPIO_Pin == GPIO_PIN_7){
	  if(click2 == 0){
		 debounce_pino7(0);
		 if (cont > 0 ){
			 cont = cont - 1;
			 itoa (cont, BufferC, 10);
			 HAL_UART_Transmit(&UartHandle, BufferC, BUFFERSIZE , 30);
			 HAL_UART_Transmit(&UartHandle, " ", 1 , 30);
			 ttemp = 0;
		 }
	  }

	  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == 1 && click2 == 1){
	   	 debounce_pino7(1);
	  }
  }

	int i=0;
	int j=0;
	for (i=0 ; i < 50000000 ; i++){
		j++;
	}

	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

// PINO 13 funcionando para decrementar
//	  if (GPIO_Pin == GPIO_PIN_13){
//		  if(click1 == 0){
//			 debounce_pino13(0);
//			 if (cont > 0 ){
//				 cont = cont - 1;
//				 itoa (cont, BufferC, 10);
//				 HAL_UART_Transmit(&UartHandle, BufferC, BUFFERSIZE , 30);
//				 HAL_UART_Transmit(&UartHandle, " ", 1 , 30);
//				 ttemp = 0;
//			 }
//		  }
//
//		  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 1 && click1 == 1){
//		   	 debounce_pino13(1);
//		  }

//  if (GPIO_Pin == GPIO_PIN_1){
//	  if(click2 == 0){
//		 debounce_pino1(0);
//		 if (cont > 0){
//			 cont = cont - 1;
//			 itoa (cont, BufferC, 10);
//			 HAL_UART_Transmit(&UartHandle, BufferC, BUFFERSIZE , 30);
//			 HAL_UART_Transmit(&UartHandle, " ", 1 , 30);
//			 ttemp = 0;
//		 }
//	  }
//
//	  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == 1 && click2 == 1){
//	   	 debounce_pino1(1);
//	  }
//
//  }


}


/* Função de configuração do clock - Até o momento iremos utiliza-la apenas, no futuro estudaremos as configuracoes */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* No HSE Oscillator on Nucleo, Activate PLL with HSI/2 as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_NONE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
  {
    Error_Handler();
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1)!= HAL_OK)
  {
    Error_Handler();
  }
}


/* Função chamada no caso de erro na configuração */
static void Error_Handler(void)
{
   while(1);
}

void debounce_pino13 (uint16_t a){
	int i=0;
	int j=0;
	for (i=0 ; i < 5000 ; i++){
	        j++;
	    }
	if (a == 0 && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 0) {
		click1 = 1;
	}
	if (a == 1 && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 1) {
		click1 = 0;
	}
    return;
}

void debounce_pino8 (uint16_t a){
	int i=0;
	int j=0;
	for (i=0 ; i < 5000 ; i++){
	        j++;
	    }
	if (a == 0 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == 0) {
		click1 = 1;
	}
	if (a == 1 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == 1) {
		click1 = 0;
	}
    return;
}

void debounce_pino7 (uint16_t a){
	int i=0;
	int j=0;
	for (i=0 ; i < 5000 ; i++){
	        j++;
	    }
	if (a == 0 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == 0) {
		click2 = 1;
	}
	if (a == 1 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == 1) {
		click2 = 0;
	}
    return;
}

