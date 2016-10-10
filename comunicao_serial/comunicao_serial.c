/*
  ******************************************************************************
  070253 - Circuitos Microprocessados
  Prof. Lucio Rene Prade
  Quinto Exemplo  - UART
  ******************************************************************************
 */

#include "main.h"

int debouce_read(GPIO_TypeDef *GPIOx, int GPIO_PIN);

/* Struct de configuração da comunicação serial definido em  stm32f0xx_hal_uart.h*/
UART_HandleTypeDef UartHandle;

/* Struct definida em  stm32f0xx_hal_gpio.h para configrar o port*/
static GPIO_InitTypeDef  GPIO_InitStruct;

/* Buffer de Transmissão  */
uint8_t Buffer_liga[BUFFERSIZE] = {'L'};
uint8_t Buffer_pisca[BUFFERSIZE] = {'P'};
uint8_t Buffer_desliga[BUFFERSIZE] = {'D'};
uint8_t Buffer_1[5];


static void SystemClock_Config(void);
static void Error_Handler(void);


int btn_1;
int main(void) {
	int estado = 0;
  /* Inicializa as bibliotecas HAL */
  HAL_Init();

  /* Configura o clock do sistema */
  SystemClock_Config();

  /* Habilita o clock no port onde está o pino que possui função de TX e RX */
   __GPIOA_CLK_ENABLE();
   __GPIOC_CLK_ENABLE();

 
 //    PA2     ------> USART2_TX
 //    PA3     ------> USART2_RX

  /*Configura os pinos GPIOs  para a função alternativa de RX e TX */
  GPIO_InitStruct.Pin       = GPIO_PIN_2|GPIO_PIN_3;  // Pino de TX verificar no  esquemático
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_USART2;

  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Configura  o pino do led como output push-pull */
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  //GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Pin 	= (GPIO_PIN_7 | GPIO_PIN_4);

  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Configura  o pino do botão como input push-pull, utilizo a mesma variável declarada para a configuração do pino anterior*/
   GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
   GPIO_InitStruct.Pull  = GPIO_PULLUP;
   GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
   GPIO_InitStruct.Pin 	 = GPIO_PIN_7;

   HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
 // HAL_UART_Transmit(&UartHandle, (uint8_t*)Buffer, BUFFERSIZE, 5000);


  estado = 0;
  while (1)
  {

	  btn_1 = debouce_read(GPIOC, GPIO_PIN_7);
	  if((1 == btn_1) && (estado == 0))
	  {
		    /*Escreve o dado a ser transmitido */
		    if(HAL_UART_Transmit(&UartHandle, (uint8_t*)Buffer_liga, 1, 100)!= HAL_OK)
		    {
		   //   Error_Handler();
		    }

		    HAL_Delay(100);

		    /*Le dado recebido na  UART  -- processo bloqueia o fluxo do programa */
		   if(HAL_UART_Receive(&UartHandle, (uint8_t *)Buffer_1, 1, 0x200) != HAL_OK)
		   {
			//  Error_Handler();
		   }
		  if (Buffer_1[0] == 'L')
		  {
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
			  estado = 1;
		  }
	  }

	  else if((1 == btn_1) && (estado == 1))
	  {
		    /*Escreve o dado a ser transmitido */
		    if(HAL_UART_Transmit(&UartHandle, (uint8_t*)Buffer_pisca, 1, 100)!= HAL_OK)
		    {
		   //   Error_Handler();
		    }

		    HAL_Delay(100);

		    /*Le dado recebido na  UART  -- processo bloqueia o fluxo do programa */
		   if(HAL_UART_Receive(&UartHandle, (uint8_t *)Buffer_1, 1, 0x200) != HAL_OK)
		   {
			//  Error_Handler();
		   }
		  if (Buffer_1[0] == 'P')
		  {
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
			  estado = 2;
		  }
	  }

	  else if((0 == btn_1) && (estado == 2))
	  {
		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
		  HAL_Delay(500);
	  }
#if 1
	  else if((1 == btn_1) && (estado == 2))
	  	  {
	  		    /*Escreve o dado a ser transmitido */
	  		    if(HAL_UART_Transmit(&UartHandle, (uint8_t*)Buffer_desliga, 1, 100)!= HAL_OK)
	  		    {
	  		   //   Error_Handler();
	  		    }

	  		    HAL_Delay(100);

	  		    /*Le dado recebido na  UART  -- processo bloqueia o fluxo do programa */
	  		   if(HAL_UART_Receive(&UartHandle, (uint8_t *)Buffer_1, 1, 0x200) != HAL_OK)
	  		   {
	  			//  Error_Handler();
	  		   }
	  		  if (Buffer_1[0] == 'D')
	  		  {
	  			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
	  			  estado = 0;
	  		  }
	  	  }
#endif


  }

}

int debouce_read(GPIO_TypeDef *GPIOx, int GPIO_PIN){
	if(HAL_GPIO_ReadPin(GPIOx, GPIO_PIN) == 0){
		   char i;
		   char cont_temp = 0;
		   char leitura_atual = HAL_GPIO_ReadPin(GPIOx, GPIO_PIN);
		   char leitura_anterior = 1;
		   for (i = 0; i <= 99; i++){

			   HAL_Delay(10);
			   leitura_anterior = leitura_atual;
			   leitura_atual = HAL_GPIO_ReadPin(GPIOx, GPIO_PIN);

			   if ((leitura_atual == leitura_anterior) && (leitura_atual == 0)){
				   cont_temp++;
				   if(cont_temp >= 4){

					   HAL_Delay(100);
					   return 1;
				   }
			   }
			   else{
				   cont_temp = 0;
			   }
		   }
		   return 0;
	}
	return 0;
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

