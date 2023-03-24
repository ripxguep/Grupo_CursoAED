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
#define 	MY_ADDR		SLAVE_ADDRRESS
#define LED_PIN	9
static uint8_t commandCode =0;
static uint8_t Cnt=0;
//some data
uint8_t rcv_buff[];
//some data
uint8_t TX_buff[32]="no se que poner primo";
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
	I2C_SlaveEnableDisableCallbackEvents(I2C1,ENABLE);
	//I2C PERI ENABLE
	I2C_PeripheralControl(I2C1, ENABLE);
	//ACK BIT MADE 1 AFTER PE=1
	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);
	while(1);
}

void I2C1_EV_IRQHandler (){
I2C_EV_IRQHandling(&I2C1Handle);
}
void I2C1_ER_IRQHandler (){
	I2C_ER_IRQHandling(&I2C1Handle);
}
void 	I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv){
	if(AppEv==I2C_EV_DATA_REQ)  {
		//Master wants some data that slave has to send
		if(commandCode==0x51){
			//send the length info to the master
			I2C_SlaveSendData(pI2CHandle->pI2Cx, strlen((char*)TX_buff));
		}else if(commandCode==0x52){
				//Send the contents of Tx_buf
				I2C_SlaveSendData(pI2CHandle->pI2Cx, TX_buff[Cnt++]);
			}

	}else if(AppEv==I2C_EV_DATA_RCV){

		//data is waiting for the slave to read.slave has to read it
	commandCode =I2C_SlaveReceiveData(pI2CHandle->pI2Cx);

	}	else if(AppEv==I2C_EV_TX_COMPLETE)  {
	printf("Tx is completed\n");
}else if(AppEv==I2C_EV_RX_COMPLETE){
	printf("Rx is completed\n");
	rxcomplt=SET;
}else if(AppEv==I2C_ERROR_AF){
	printf("Error : Ack failure\n");
	//in master ack failure happens when salve fails to send
	//ack for the bit sent from the master
	commandCode=0xff;
	Cnt=0;
	I2C_CloseSendData(pI2CHandle);
	//generate stop condition
I2C_GenerateStopCondition(I2C1);
//hang in infinite loop
while(1);
}else if(AppEv==I2C_EV_STOP){
	//only during slave reception
	//master has ended i2c comm with slave
}
}
