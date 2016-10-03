/*
  ******************************************************************************
  070253 - Circuitos Microprocessados
  Prof. Lucio Rene Prade
  Quarto Exemplo  - GPIO
  ******************************************************************************
 */
// Adaptado para a placa discovery


#include "main.h"

/* Struct definida em  stm32f0xx_hal_gpio.h para configrar o port*/

void debouce_read(GPIO_TypeDef *GPIOx, int GPIO_PIN);
void leds(int num);

static GPIO_InitTypeDef  GPIO_InitStruct;

static void SystemClock_Config(void);
static void Error_Handler(void);

int num_global = 0;
int main(void) {

  /* Inicializa as bibliotecas HAL */
  HAL_Init();

  /* Configura o clock do sistema */
  SystemClock_Config();

  /* Habilita o Clock no port do led,  função definida em stm32f0xx_hal_rcc.h*/
  __GPIOA_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();

   /* Configura  o pino do led como output push-pull */
   GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
   //GPIO_InitStruct.Pull  = GPIO_PULLUP;
   GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
   GPIO_InitStruct.Pin 	= 0xFF;

   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


   /* Habilita o Clock no port do botão,  função definida em stm32f0xx_hal_rcc.h*/
   //  __GPIOC_CLK_ENABLE();

   /* Configura  o pino do botão como input push-pull, utilizo a mesma variável declarada para a configuração do pino anterior*/
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Pin 	 = GPIO_PIN_7;

    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


    /* Liga o Led e mantem até o botão ser pressionado novamente */


    while (1) {

   	/* função definida em stm32f0xx_hal_gpio.h */

    	debouce_read(GPIOC, GPIO_PIN_7);
    	// HAL_GPIO_WritePin(GPIOA, (GPIO_PIN_7 | GPIO_PIN_6 | GPIO_PIN_5 | GPIO_PIN_4), 1);
    	// HAL_GPIO_WritePin(GPIOA, 0xFF, 1);

       /* função definida em stm32f0xx_hal.h */
    	leds(num_global);

    }

}

void debouce_read(GPIO_TypeDef *GPIOx, int GPIO_PIN)
{
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
   				   //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
   				   num_global++;
   				   HAL_Delay(100);
   				   break;
   			   }
   		   }
   		   else{
   			   cont_temp = 0;
   		   }
   	   }

      }
}

void leds(int num)
{
	int resto = 0;
	int i = 0;
	int leds[7];

    for(i = 0; i <= 7; i++)
    {
      resto = num%2;
      num = num/2;

      if(1 == resto)
      {
          leds[i] = 1;
      }
      else{
        leds[i] = 0;
      }
    }

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, leds[7]);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, leds[6]);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, leds[5]);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, leds[4]);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, leds[3]);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, leds[2]);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, leds[1]);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, leds[0]);
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

