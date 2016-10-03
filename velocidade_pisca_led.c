/*
  ******************************************************************************
  070253 - Circuitos Microprocessados
  Prof. Lucio Rene Prade
  Exercício 3 - Utilizando  dois botão que deverão ser montados na protoboard
  e conectados  no Port C pinos 6 e 8, implemente  um controle de velocidade para o piscar do led,
  um botão deverá incrementar e o outro  decrementar a velocidade.
  ******************************************************************************
 */

#include "stm32f0xx.h"

#define BSRR_VAL        0x0060   /* 0000 0000 0110 0000 */

void delay (int a);
void pisca_led();

int main(void) {

    unsigned int contador = 50000;


  /* GPIOA Periph clock enable */
    RCC->AHBENR |= (RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOCEN);
    // Habilita o clock para o GPIOA e GPIOC

    GPIOA->MODER |= (GPIO_MODER_MODER5_0 | GPIO_MODER_MODER6_0) ;
    /* Configure PA5 and PA6 in output  mode  */
    GPIOC->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER8);
    // Configura como entrada  PC6 e PC8

    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_5 | GPIO_OTYPER_OT_6) ;
    // Ensure push pull mode selected--default

    GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR5|GPIO_OSPEEDER_OSPEEDR6);
    //Ensure maximum speed setting (even though it is unnecessary)
    GPIOC->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR8);
    // configura a velocidade da IO para velocidade maxima

    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR5|GPIO_PUPDR_PUPDR6);
    //Ensure all pull up pull down resistors are disabled
    GPIOC->PUPDR |= (GPIO_PUPDR_PUPDR6_0 | GPIO_PUPDR_PUPDR8_0);
    // Habilita o pull-up no PC6

#if 0
    while (1)
    {
        /* Set PA5 and PA6 */
        GPIOA->BSRR = BSRR_VAL;
        delay(500000);
        /* Reset PA5 and PA6 */
        GPIOA->BRR = BSRR_VAL;
        delay(500000);
    }
#endif

     while (1)
    {
        /* Write output data PA5 and PA6 */
        GPIOA->ODR |= BSRR_VAL;
        delay(contador);
        /* Write output data PA5 and PA6 */
        GPIOA->ODR &= ~BSRR_VAL;
        delay(contador);

        if ((GPIOC->IDR & GPIO_IDR_6) == 0){
            if(contador >= 100){
            contador -= 100;
            }
        }

        if ((GPIOC->IDR & GPIO_IDR_8) == 0){
        contador += 100;
        }
    }

    return 0;
}


void delay (int a)
{
    volatile int i,j;

    for (i=0 ; i < a ; i++)
    {
        j++;
    }

    return;
}


