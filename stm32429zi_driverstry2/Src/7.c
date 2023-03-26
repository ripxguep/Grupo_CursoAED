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
#include <string.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif
void	delay(void){
	for(uint32_t i=0;i<500000/2;i++);
}
#define 	BTN_PRESSED	HIGH
#define		HIGH		1
#define		LOW			0

/*
 * PB14->SPI2_MISO
 * PB15->SPI2_MOSI
 * PB13->SPI2_SCLK
 * PB12->SPI2_NSS
 * ALT FUNCTION MODE: 5
 */
void GPIO_ButtonInit(void){
	GPIO_Handle_t	GpioBtn;
	GpioBtn.pGPIOx=GPIOA;
			GpioBtn.GPIO_PinConfig.GPIO_PinNumber	=GPIO_PIN_NO_0;
			GpioBtn.GPIO_PinConfig.GPIO_PinMode		=GPIO_MODE_IN;
			GpioBtn.GPIO_PinConfig.GPIO_PinSpeed	=GPIO_SPEED_FAST;
			GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_NO_PUPD;

			GPIO_Init(&GpioBtn);
}
void SPI_GPIOInits(void){
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx=GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode=5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);
	//MOSI
		SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_15;
		GPIO_Init(&SPIPins);
		//NSS
			SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
			GPIO_Init(&SPIPins);
			//MISO
				SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_14;
				GPIO_Init(&SPIPins);

}
void SPI2_Inits(){
	SPI_Handle_t	SPI2handle;
	SPI2handle.pSPIx=SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig=SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode=SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed=SPI_SCLK_SPEED_DIV8;
	SPI2handle.SPIConfig.SPI_DFF=SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL=SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA=SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM=SPI_SSM_DI;//HW SLAVE MANAGEMENT ENABLE
	SPI_Init(&SPI2handle);
}
int main(void)
{
	char user_data[]="Hello world";
	SPI_GPIOInits();
	SPI2_Inits();
	SPI_SSOEConfig(SPI2, ENABLE);
	while(1){
		//wait till button is pressed
	while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
	delay();
	//enable spi2 peripheral
	SPI_PeripheralControl(SPI2,ENABLE);
	//First send length info
	uint8_t dataLen= strlen(user_data);
	SPI_SendData(SPI2,&dataLen,1);
	//send data
	SPI_SendData(SPI2,(uint8_t*) user_data, strlen(user_data));
	//if tx ended

	while(SPI_GetFlagStatus(SPI2, SPI_SR_BSY));
	//disable peripheral
	SPI_PeripheralControl(SPI2,DISABLE);
	}
	return 0;
}
