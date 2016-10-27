/*
  ******************************************************************************
  070253 - Circuitos Microprocessados
  Prof. Lucio Rene Prade
  Exemplo  - Timer
  ******************************************************************************
 */


#include "main.h"

/* Struct definida em  stm32f0xx_hal_tim.h para configrar o timer*/
TIM_HandleTypeDef    TimHandle;

/* Struct de configuração da comunicação serial definido em  stm32f0xx_hal_uart.h*/
UART_HandleTypeDef UartHandle;

/* Struct definida em  stm32f0xx_hal_gpio.h para configrar o port*/
static GPIO_InitTypeDef  GPIO_InitStruct;

/* ADC configuration structure declaration */
ADC_HandleTypeDef    AdcHandle;

/* ADC channel configuration structure declaration */
ADC_ChannelConfTypeDef   sConfig;

/* Buffer de Transmissão  */
uint8_t Buffer[50] = " **** Exemplo de ADC ****\n\r ";

uint32_t g_ADCValue;

/* Declaração da variável Prescaler */
uint32_t uwPrescalerValue = 0;

static void SystemClock_Config(void);
static void Error_Handler(void);

volatile int flag = 0;

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
   GPIO_InitStruct.Pin = GPIO_PIN_0;
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

    AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    AdcHandle.Init.LowPowerAutoWait      = DISABLE;
    AdcHandle.Init.LowPowerAutoPowerOff  = DISABLE;
    AdcHandle.Init.Resolution            = ADC_RESOLUTION12b;
    AdcHandle.Init.ScanConvMode          = ADC_SCAN_ENABLE;
    AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    AdcHandle.Init.ContinuousConvMode    = DISABLE;
    AdcHandle.Init.DiscontinuousConvMode = DISABLE;
    AdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    AdcHandle.Init.EOCSelection          = EOC_SINGLE_CONV;
    AdcHandle.Init.DMAContinuousRequests = DISABLE;
    AdcHandle.Init.Overrun               = OVR_DATA_PRESERVED;

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
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
    {
      Error_Handler();
    }



    /* Envia a mensagem de inicio */
    HAL_UART_Transmit(&UartHandle, (uint8_t*)Buffer, 50, 100);

   while (1) {

	 if(0 == flag)
	 {

	 }
	 else
	 {
		 flag = 0;
	  	 HAL_ADC_Start(&AdcHandle);
	  	 if (HAL_ADC_PollForConversion(&AdcHandle, 1000000) == HAL_OK)
	  	 {
	  		 g_ADCValue = HAL_ADC_GetValue(&AdcHandle);

	  		 itoa(g_ADCValue, (uint8_t*)Buffer, 10);

	  		 HAL_UART_Transmit(&UartHandle, (uint8_t*)Buffer, 4, 100);
	  		 HAL_UART_Transmit(&UartHandle, "\n\r", 2, 100);

	  	  }
	  	  HAL_ADC_Stop(&AdcHandle);

	  	 // HAL_Delay(500);
	 }


   }


}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	flag = 1;
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}



/* Função de configuração do clock
 *            System Clock source            = PLL (HSI/2)
 *            SYSCLK(Hz)                     = 48000000
 *            HCLK(Hz)                       = 48000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 1
 *            HSI Frequency(Hz)              = 8000000
 *            PREDIV                         = 1
 *            PLLMUL                         = 12
 *            Flash Latency(WS)              = 1
 * @param  None
 * @retval None
 */

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

