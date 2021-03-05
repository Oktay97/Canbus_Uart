/*
 * Uart_Rx.c
 *
 *  Created on: Feb 4, 2021
 *      Author: oktay
 */

#include "Uart_Rx.h"
#include "string.h"
#include "stm32f1xx_hal.h"


#define LD2_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5

extern UART_HandleTypeDef huart2;

uint8_t RX2_Char = 0x00;



void uart_okuma(void){
    if(RX2_Char == '1')
    {
	  HAL_UART_Receive_IT(&huart2, &RX2_Char, 1);
	  RX2_Char = 0x00;
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    }
    if(RX2_Char == '2')
    {
	  HAL_UART_Receive_IT(&huart2, &RX2_Char, 1);
	  RX2_Char = 0x00;
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    }
}
