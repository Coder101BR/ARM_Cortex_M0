/*
  ******************************************************************************
  070253 - Circuitos Microprocessados
  Prof. Lucio Rene Prade
  Projeto GB - Carregador
  ******************************************************************************
 */

#include <stdlib.h>
#include "main.h"
#include <HD44780LIB.h>

/* Struct definida em  stm32f0xx_hal_tim.h para configrar o timer*/
TIM_HandleTypeDef    TimHandle;

/* Struct de configuração da comunicação serial definido em  stm32f0xx_hal_uart.h*/
UART_HandleTypeDef UartHandle;

/* Struct definida em  stm32f0xx_hal_gpio.h para configrar o port*/
GPIO_InitTypeDef  GPIO_InitStruct;

/* ADC configuration structure declaration */
ADC_HandleTypeDef    AdcHandle;

/* ADC channel configuration structure declaration */
ADC_ChannelConfTypeDef   sConfig;

/* Declaração da variável Prescaler */
uint32_t uwPrescalerValue = 0;

/* Buffer de Transmissão  */
uint8_t Buffer[50];
uint8_t Buffer2[50];
uint8_t Buffer3[50];
uint32_t g_ADCValue;

/* Protótipos */
static void SystemClock_Config(void);
static void Error_Handler(void);
void le_entradas_analogicas();
void configura_entradas_analogicas();

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

  /* Configura a variável prescaler com valor de contagem  para 10000 Hz */
  uwPrescalerValue = (uint32_t)(SystemCoreClock / 10000) - 1;

	/* Configura TIM1 */
	TimHandle.Instance = TIM3;
	TimHandle.Init.Period            = 10000 - 1;
	TimHandle.Init.Prescaler         = uwPrescalerValue;
	TimHandle.Init.ClockDivision     = 0;
	TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
	TimHandle.Init.RepetitionCounter = 0;
	HAL_TIM_Base_Init(&TimHandle);

	/* Configura a geração de interrupção para o timer 1 */
	HAL_TIM_Base_Start_IT(&TimHandle);

   /* ADC1 Channel8 GPIO pin configuration */
   GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4;
   GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
   GPIO_InitStruct.Pull = GPIO_NOPULL;

   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


   /*Configura os pinos GPIOs  para a função alternativa de RX e TX da USART2*/
   GPIO_InitStruct.Pin       = GPIO_PIN_2|GPIO_PIN_3;
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

   HAL_UART_Init(&UartHandle);



    /* Envia a mensagem de inicio */
    HAL_UART_Transmit(&UartHandle, (uint8_t*)Buffer, 50, 100);


    /* Infinite Loop */
    while (1)
    {

		HAL_UART_Transmit(&UartHandle, "\r Tensao: ", 11, 100);
		HAL_UART_Transmit(&UartHandle, (uint8_t*)Buffer, 4, 100);
		HAL_UART_Transmit(&UartHandle, "\n\t", 4, 100);

		HAL_UART_Transmit(&UartHandle, "\r Corrente: ", 13, 100);
		HAL_UART_Transmit(&UartHandle, (uint8_t*)Buffer2, 4, 100);
		HAL_UART_Transmit(&UartHandle, "\n\t", 4, 100);


		HAL_UART_Transmit(&UartHandle, "\r Temperatura: ", 16, 100);
		HAL_UART_Transmit(&UartHandle, (uint8_t*)Buffer3, 4, 100);
		HAL_UART_Transmit(&UartHandle, "\n\t", 4, 100);
		HAL_UART_Transmit(&UartHandle, "\n\t", 4, 100);

		HAL_Delay(1000);
     }

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	le_entradas_analogicas();

}

void configura_entradas_analogicas()
{

	   /* Habilita o clock do ADC 1*/
	   __ADC1_CLK_ENABLE();

	   /* Configuração  do periférico ADC */
	    /*
	     *  Instance                  = ADC1.
	     *  ClockPrescaler            = PCLK divided by 4.
	     *  LowPowerAutoWait          = Disabled
	     *  LowPowerAutoPowerOff      = Disabled
	     *  Resolution                = 12 bit (increased to 16 bit with oversampler)
	     *  ScanConvMode              = ADC_SCAN_ENABLE
	     *  DataAlign                 = Right
	     *  ContinuousConvMode        = Enabled
	     *  DiscontinuousConvMode     = Enabled
	     *  ExternalTrigConv          = ADC_SOFTWARE_START
	     *  ExternalTrigConvEdge      = None (Software start)
	     *  EOCSelection              = End Of Conversion event
	     *  DMAContinuousRequests     = Disabled
	     */

	    AdcHandle.Instance = ADC1;

	    AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_ASYNC;
	    AdcHandle.Init.LowPowerAutoWait      = DISABLE;
	    AdcHandle.Init.LowPowerAutoPowerOff  = DISABLE;
	    AdcHandle.Init.Resolution            = ADC_RESOLUTION12b;
	    AdcHandle.Init.ScanConvMode          = ADC_SCAN_ENABLE;
	    AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
	    AdcHandle.Init.ContinuousConvMode    = DISABLE;
	    AdcHandle.Init.DiscontinuousConvMode = ENABLE;
	    AdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
	    AdcHandle.Init.EOCSelection          = EOC_SINGLE_CONV;
	    AdcHandle.Init.DMAContinuousRequests = DISABLE;
	    AdcHandle.Init.Overrun               = OVR_DATA_OVERWRITTEN;
	    AdcHandle.NbrOfConversionRank = 3;

	    /* Inicilaiza  o ADC  com as configurações*/
	    if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
	    {
	      Error_Handler();
	    }


	    /*Calibra o ADC com as configurações */
	    if (HAL_ADCEx_Calibration_Start(&AdcHandle) != HAL_OK)
	   {
	     Error_Handler();
	   }


	    /* Seleciona o canal analogico (Channel 0) */
	    sConfig.Channel      = ADC_CHANNEL_0;
	    sConfig.Rank         = ADC_RANK_CHANNEL_NUMBER;
	    sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;  // AUMENTA O TEMPO DE AMOSTRAGEM, RESOLVE O PROBLEMA DE INSTABILIDADE
	    if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
	    {
	      Error_Handler();
	    }

	    /* Seleciona o canal analogico (Channel 0) */
	    sConfig.Channel      =  ADC_CHANNEL_1;
	    sConfig.Rank         = ADC_RANK_CHANNEL_NUMBER;
	    sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
	    if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
	    {
	     Error_Handler();
	    }

	    /* Seleciona o canal analogico (Channel 0) */
	    sConfig.Channel      =  ADC_CHANNEL_4;
	    sConfig.Rank         = ADC_RANK_CHANNEL_NUMBER;
	    sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
	    if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
	    {
	     Error_Handler();
	    }

}

void le_entradas_analogicas()
{
	configura_entradas_analogicas();
	/* variaveis de uso geral */
	float res = (3.3/4095); // resolução do AD
	float analog; // tensao analogica lida pelo AD
	int num_1;
	float num_2;
	int num_3;
	unsigned char snum_1[10];
	unsigned char snum_2[10];
	/* variaveis para leitura da tensão da bateria */
	float r1 = 50000; // resistencia r1
	float r2 = 10000; // resistencia variavel r2
	float vbat;
	/* variaveis para leitura da corrente da bateria */
	int ganho = 22;
	float shunt = 0.1;
	float ibat;

	/* Leitura da tensão da bateria */
	HAL_ADC_Start(&AdcHandle); // Inicia a amostragem do primeiro canal
	if (HAL_ADC_PollForConversion(&AdcHandle, 1000000) == HAL_OK)
	{

	 // Le o valor do primeiro canal
	 g_ADCValue = HAL_ADC_GetValue(&AdcHandle);

	 analog = g_ADCValue*res;
	 vbat = (analog*(r1 +r2))/r2;  // tensao da bateria


	 num_1 = vbat;
	 num_2 = (vbat - num_1)*1000;
	 num_3 = num_2;
	 itoa(num_1, snum_1, 10);
	 itoa(num_3, snum_2, 10);

	 strcat(snum_1, ",");
	 strcat(snum_1, snum_2);
	 strcpy(Buffer, snum_1);
	}

	/* Leitura da corrente da bateria */
	HAL_ADC_Start(&AdcHandle); // Inicia a amostragem do segundo canal
	if (HAL_ADC_PollForConversion(&AdcHandle, 1000000) == HAL_OK)
	{

		 // Le o valor do segundo canal
		 g_ADCValue = HAL_ADC_GetValue(&AdcHandle);


		 analog = g_ADCValue*res;

		 ibat = (float)(analog/shunt)/ganho;
		 num_1 = ibat;
		 num_2 = (ibat - num_1)*1000;

		 //num_1 = analog;
		 //num_2 = (analog - num_1)*1000;
		 num_3 = num_2;
		 itoa(num_1, snum_1, 10);
		 itoa(num_3, snum_2, 10);


		 strcat(snum_1, ",");
		 strcat(snum_1, snum_2);
		 strcpy(Buffer2, snum_1);
	}

	/* Leitura da temperatura */
	HAL_ADC_Start(&AdcHandle); // Inicia a amostragem do teceiro canal
	if (HAL_ADC_PollForConversion(&AdcHandle, 1000000) == HAL_OK)
	{

		 // Le o valor do terceiro canal
		 g_ADCValue = HAL_ADC_GetValue(&AdcHandle);

		 analog = (g_ADCValue*res)*100;
		 num_1 = analog;
		 num_2 = (analog - num_1)*1000;
		 num_3 = num_2;
		 itoa(num_1, snum_1, 10);
		 itoa(num_3, snum_2, 10);

		 strcat(snum_1, ",");
		 strcat(snum_1, snum_2);
		 strcpy(Buffer3, snum_1);

	 }
	 HAL_ADC_Stop(&AdcHandle);
}


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
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
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

