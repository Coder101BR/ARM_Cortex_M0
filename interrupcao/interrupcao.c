/*
  ******************************************************************************
  070253 - Circuitos Microprocessados
  Prof. Lucio Rene Prade
  Sexto Exemplo  - Interrupções
  ******************************************************************************
 */


#include "main.h"

int debouce_read(GPIO_TypeDef *GPIOx, int GPIO_PIN)


/* Struct definida em  stm32f0xx_hal_gpio.h para configrar o port*/
static GPIO_InitTypeDef  GPIO_InitStruct;

static void SystemClock_Config(void);
static void Error_Handler(void);

uint8_t Buffer[8];
int num_global = 0;
int num_anterior = 0;
int main(void) {

  /* Inicializa as bibliotecas HAL */
  HAL_Init();

  /* Configura o clock do sistema */
  SystemClock_Config();

  /* Habilita o Clock no port do led,  função definida em stm32f0xx_hal_rcc.h*/
   __GPIOA_CLK_ENABLE();

   /* Configura  o pino do led como output push-pull */
   GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
   GPIO_InitStruct.Pin 	= GPIO_PIN_5;

   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

     /*Configura os pinos GPIOs  para a função alternativa de RX e TX */
    GPIO_InitStruct.Pin       = GPIO_PIN_2|GPIO_PIN_3;  // Pino de TX verificar no  esquemático
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_USART2;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


    /* Habilita o Clock no port do botão,  função definida em stm32f0xx_hal_rcc.h*/
    __GPIOC_CLK_ENABLE();

    /* Configura  o pino do botão como um pino de interrupção externa*/
    GPIO_InitStruct.Mode  = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Pin   = (GPIO_PIN_13 | GPIO_PIN_14);

    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Habilita e conigura a priodidade da interrupção  EXTI line 4_15  */
    HAL_NVIC_SetPriority(EXTI4_15_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

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

   /* Infinite loop */

    while (1)
   {
        if(num_anterior != num_global){
            Buffer[0] = num_global;
            num_anterior = num_global;
            HAL_UART_Transmit(&UartHandle, (uint8_t*)Buffer, 1, 100)!= HAL_OK)
        }
   }

    return 0;
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

void  funcaoTratamentoInterrupcao(uint16_t GPIO_Pin){
	int btn_1;
	int btn_2;

	/* Se ocorreu a interrupção externa */
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_Pin) != RESET) {

		//Limpa a flag de interrupção
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);

	    if (GPIO_Pin == GPIO_PIN_13) {
            btn_1 = debouce_read(GPIOC, GPIO_PIN_13);

            if((1 == btn_1) && (255 != num_global)){
    		num_global++;
            }
	    }

	    else if (GPIO_Pin == GPIO_PIN_14){
            btn_2 = debouce_read(GPIOC, GPIO_PIN_14);

            if((1 == btn_2) && (0 != num_global)){
            num_global--;
            }
	    }

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


