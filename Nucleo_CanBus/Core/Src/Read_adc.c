/*
 * Read_adc.c
 *
 *  Created on: Feb 2, 2021
 *      Author: oktay
*/

#include "Read_adc.h"
#include "string.h"
#include "stm32f1xx_hal.h"


#define LD2_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC

char *data = "Adc=";
uint16_t AD_RES = 0;
char *alt = "\n";
char str[10]="";

extern UART_HandleTypeDef huart2;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2;

void adc_okuma(void){
	// while(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin)==0){}
 	 HAL_ADC_Start(&hadc1); // Start ADC Conversion
  	 HAL_ADC_PollForConversion(&hadc1, 1); // Poll ADC1 Perihperal & TimeOut = 1mSec
  	 AD_RES = HAL_ADC_GetValue(&hadc1);  // Read The ADC Conversion Result & Map It To PWM DutyCycle
  	 TIM2->CCR1 = (AD_RES<<4);
 	 sprintf(str, "%d", AD_RES); //int2char


 	 HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

 	 HAL_UART_Transmit(&huart2, (uint8_t*)data, strlen(data),  0xFFFF);
	 HAL_UART_Transmit(&huart2, (uint16_t*)str, strlen(str),  0xFFFF);
	 HAL_UART_Transmit(&huart2, (uint8_t*)alt, strlen(alt),  0xFFFF);
	 HAL_ADC_Stop(&hadc1);
	 return 1;
}

