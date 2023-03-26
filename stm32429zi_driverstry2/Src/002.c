/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
#include "stm32f429zi.h"
#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif
void	delay(void){
	for(uint32_t i=0;i<500000/2;i++);
}
#define 	BTN_PRESSED	HIGH
#define		HIGH		1
int main(void)
{
	GPIO_Handle_t	GpioLed,GpioBtn;
	GpioLed.pGPIOx=GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber	=GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode		=GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed	=GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType	=GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_NO_PUPD;
	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioLed);

	GpioBtn.pGPIOx=GPIOA;
		GpioBtn.GPIO_PinConfig.GPIO_PinNumber	=GPIO_PIN_NO_0;
		GpioBtn.GPIO_PinConfig.GPIO_PinMode		=GPIO_MODE_IN;
		GpioBtn.GPIO_PinConfig.GPIO_PinSpeed	=GPIO_SPEED_FAST;
		GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_NO_PUPD;
		GPIO_PeriClockControl(GPIOA, ENABLE);
		GPIO_Init(&GpioBtn);
	while(1){
		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)==BTN_PRESSED){
		GPIO_TogglePin(GPIOD, GPIO_PIN_NO_12);
		delay();
		}
	}
	for(;;);
}