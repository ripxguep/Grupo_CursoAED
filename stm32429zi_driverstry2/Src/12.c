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
#include <stdio.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif
void	delay(void){
	for(uint32_t i=0;i<500000/2;i++);
}
I2C_Handle_t	I2C1Handle;


#define 	BTN_PRESSED	HIGH
#define		LED_ON		1
#define		LED_OFF			0
#define 	SLAVE_ADDRRESS 0x1
#define LED_PIN	9
//some data
uint8_t rcv_buff[];
uint8_t rxcomplt;
/*
 * PB6->SCL
 * PB9->SDA
 * PB13->SPI2_SCLK
 * PB12->SPI2_NSS
 * ALT FUNCTION MODE: 5
 */
void GPIO_ButtonInit(void){
	GPIO_Handle_t	GpioBtn,GpioLed;
	GpioBtn.pGPIOx=GPIOA;
			GpioBtn.GPIO_PinConfig.GPIO_PinNumber	=GPIO_PIN_NO_0;
			GpioBtn.GPIO_PinConfig.GPIO_PinMode		=GPIO_MODE_IN;
			GpioBtn.GPIO_PinConfig.GPIO_PinSpeed	=GPIO_SPEED_FAST;
			GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_NO_PUPD;
			GPIO_Init(&GpioBtn);

			GpioLed.pGPIOx=GPIOD;
						GpioLed.GPIO_PinConfig.GPIO_PinNumber	=GPIO_PIN_NO_12;
						GpioLed.GPIO_PinConfig.GPIO_PinMode		=GPIO_MODE_OUT;
						GpioLed.GPIO_PinConfig.GPIO_PinSpeed	=GPIO_SPEED_FAST;
						GpioLed.GPIO_PinConfig.GPIO_PinOPType	=GPIO_OP_TYPE_OD;
						GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_NO_PUPD;
						GPIO_Init(&GpioLed);
}

void I2C1_GPIOInits(void){
GPIO_Handle_t	I2CPins;
I2CPins.pGPIOx=GPIOB;
I2CPins.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALTFN;
I2CPins.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_OD;
I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_PU;
I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode=4;
I2CPins.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
//SCL
I2CPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_6;
GPIO_Init(&I2CPins);
//SDA
I2CPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_7;
GPIO_Init(&I2CPins);

}
void I2C1_Inits(){

	I2C1Handle.pI2Cx=I2C1;
	I2C1Handle.I2C_Config->I2C_ACKControl=I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config->I2C_DeviceAddress=0x61;
	I2C1Handle.I2C_Config->I2C_SCLSpeed=I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config->I2C_SCLSpeed=I2C_SCL_SPEED_SM;
	I2C_Init(&I2C1Handle);
}
void initialise_monitor_handles(void);
int main(void)
{
	printf("Application is running\n");
	uint8_t commandcode;
	uint8_t len;
	//i2c configs
	//button
	GPIO_ButtonInit();
	//i2c pin
	I2C1_GPIOInits();
	//i2c peri confg
	I2C1_Inits();
	//i2c irq config
	I2C_IRQConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQConfig(IRQ_NO_I2C1_ER, ENABLE);
	//I2C PERI ENABLE
	I2C_PeripheralControl(I2C1, ENABLE);
	//ACK BIT MADE 1 AFTER PE=1
	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);
	while(1){
			//wait till button is pressed
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		delay();
	commandcode=0x51;
	while(I2C_MasterSendDataIT(&I2C1Handle, &commandcode, 1, SLAVE_ADDRRESS,I2C_SR)!=I2C_READY);
	while(I2C_MasterReceiveDataIT(&I2C1Handle, &len, 1, SLAVE_ADDRRESS,I2C_SR));
	commandcode=0x52;
	while(I2C_MasterSendDataIT(&I2C1Handle, &commandcode, 1, SLAVE_ADDRRESS,I2C_SR));
	while(I2C_MasterReceiveDataIT(&I2C1Handle, 	rcv_buff, len, SLAVE_ADDRRESS,I2C_NO_SR));
	rxcomplt=RESET;
	//wait to rx completion
	while(rxcomplt!=SET);
	rcv_buff[len+1]='\0'; //we terminate last bit with null
	printf("Data : %s",rcv_buff);
	rxcomplt=RESET;
	}
}

void I2C1_EV_IRQHandler (){
I2C_EV_IRQHandling(&I2C1Handle);
}
void I2C1_ER_IRQHandler (){
	I2C_ER_IRQHandling(&I2C1Handle);
}
void 	I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv){

if(AppEv==I2C_EV_TX_COMPLETE)  {
	printf("Tx is completed\n");
}else if(AppEv==I2C_EV_RX_COMPLETE){
	printf("Rx is completed\n");
	rxcomplt=SET;
}else if(AppEv==I2C_ERROR_AF){
	printf("Error : Ack failure\n");
	//in master ack failure happens when salve fails to send
	//ack for the bit sent from the master
	I2C_CloseSendData(pI2CHandle);
	//generate stop condition
I2C_GenerateStopCondition(I2C1);
//hang in infinite loop
while(1);
}
}
