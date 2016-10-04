/*
  ******************************************************************************
  070253 - Circuitos Microprocessados
  Prof. Lucio Rene Prade
  Quinto Exemplo  - UART
  ******************************************************************************
 */

#include "main.h"

/* Struct de configuração da comunicação serial definido em  stm32f0xx_hal_uart.h*/
UART_HandleTypeDef UartHandle;

/* Struct definida em  stm32f0xx_hal_gpio.h para configrar o port*/
static GPIO_InitTypeDef  GPIO_InitStruct;

/* Buffer de Transmissão  */
uint8_t Buffer[BUFFERSIZE] = " **** Exemplo de Serial ****\n\r ";


static void SystemClock_Config(void);
static void Error_Handler(void);



int main(void) {

  char protocolo = 0;
  /* Inicializa as bibliotecas HAL */
  HAL_Init();

  /* Configura o clock do sistema */
  SystemClock_Config();

  /* Habilita o clock no port onde está o pino que possui função de TX e RX */
   __GPIOA_CLK_ENABLE();

   GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
   //GPIO_InitStruct.Pull  = GPIO_PULLUP;
   GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
   GPIO_InitStruct.Pin 	= GPIO_PIN_7;

   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

 
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
  

  /* Envia a mensagem de inicio */
  HAL_UART_Transmit(&UartHandle, (uint8_t*)Buffer, BUFFERSIZE, 5000);



  while (1){


    /*Le dado recebido na  UART  -- processo bloqueia o fluxo do programa */
   if(HAL_UART_Receive(&UartHandle, (uint8_t *)Buffer, 1, 0x1000) != HAL_OK)
   {
	  if('L' == protocolo){
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
	  }
	  Error_Handler();
   }
 
  
    /*Escreve o dado a ser transmitido */
    HAL_UART_Transmit(&UartHandle, (uint8_t*)Buffer, 1, 100);


    HAL_Delay(100);
  }

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

