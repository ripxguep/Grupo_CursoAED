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
#include "main.h"
#include <stdint.h>
#include <stdio.h>
extern void initialise_monitor_handles(void);

// 0.0.1 version.nothing works
int main(void)
{
 	initialise_monitor_handles();
// docs used:
 	//all register documentation for board family https://www.st.com/resource/en/reference_manual/rm0090-stm32f405415-stm32f407417-stm32f427437-and-stm32f429439-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
 	 	//User Manual (Memory map) https://www.st.com/resource/en/user_manual/um1974-stm32-nucleo144-boards-mb1137-stmicroelectronics.pdf

 	uint32_t const volatile data 	= 	123;
 	RCC_AHB1ENR_t volatile *const RCC_AHB1ENR   = ADDR_REG_AHB1ENR; //0x40023830
 	RCC_APB1ENR_t volatile *const RCC_APB1ENR   = ADDR_REG_APB1ENR; //0x40023840
 	GPIOx_PINS_t volatile *const GPIO_MODER	=	ADDR_REG_GPIO_HEAD1; //0x40020400 DEFINES PORT MODE (INPUT/OUTPUT/ALT/ANALOG)0/1/2/3
 	GPIOx_xDR_t volatile *const GPIO_OTR	=	ADDR_REG_GPIO_HEAD2; //0x40020404 DEFINES OUTPUT TYPE OF THE IO PORT PUSH-PULL/OPEN-DRAIN
	GPIOx_PINS_t volatile *const GPIO_OSPEED	=	ADDR_REG_GPIO_HEAD1+0X08; //0x40020408 DEFINES OUTPUT SPEED OF THE PORT LOW-MEDIUM-HIGH-VERY HIGH (DEFINED AT PAGE 139 OF THE DATASHEET https://www.st.com/en/microcontrollers-microprocessors/stm32f429zi.html)
	GPIOx_PINS_t volatile *const GPIO_PULL	=	ADDR_REG_GPIO_HEAD1+0X0C; //0x4002040C DEFINES NO PULL UP/DOWN ,PULL UP OR PULL DOWN FOR THE IO OF THE PORT
	GPIOx_AFRx_t volatile *const GPIO_AFRH	=	ADDR_REG_GPIO_HEAD3; //0x40020424 DEFINES FUNCTIONS OF THE ALTERNATE FUNCTION PINS defined at usermanual page 76
	I2Cx_CR1_t	 volatile *const I2C1_CR1	=	ADDR_REG_I2C_HEAD1; //0x40005400 DEFINES A LOT OF CONFIG OF I2C_CR
	I2Cx_CR2_t	volatile *const I2C1_CR2	=	ADDR_REG_I2C_HEAD2; //0x40005404 DEFINES A LOT OF CONFIG OF I2C_CR part 2
	I2Cx_CCR_t	volatile *const I2C_CCR	=	ADDR_REG_I2C_HEAD3; //0x4000541C DEFINES A LOT OF CONFIG OF I2C CLOCK RELATED
	I2C_TRISE_t	volatile *const I2C_TRISE	=	ADDR_REG_I2C_HEAD4; //0x40005420 DEFINES Maximum rise time in Fm/Sm mode

	I2C_SR1_t	volatile *const I2C_SR1	=	ADDR_REG_I2C_HEAD5; //0x40005414 DEFINES Data register for I2C com
	I2C_DR_t	volatile *const I2C_DR	=	ADDR_REG_I2C_HEAD6; //0x40005410 DEFINES Status register1 for I2C

//TESTING WHAT WE LEARNED with a found exercise for other board so we can find out if we're doing alright
 		RCC_APB1ENR->i2c1_en=1;
 		RCC_AHB1ENR->gpiob_en =1;
 		GPIO_MODER->PIN8=2;		//PINS AS ALT
 		GPIO_MODER->PIN9=2;
 		GPIO_OTR->PIN8=1;		//OPEN DRAIN
 		GPIO_OTR->PIN9=1;
 		GPIO_OSPEED->PIN8=3;//HIGH SPEEEEEEEED
 		GPIO_OSPEED->PIN9=3;
 		GPIO_PULL->PIN8=1;		//PULL UP
 		GPIO_PULL->PIN9=1;
 		GPIO_AFRH->PIN1=4; //AFRH AND AFRL ARE THE SAME TYPE OF STRUCT ,PIN1 OF AFRH->PIN8 ,PIN2 OF AFRH->PIN9...
 		GPIO_AFRH->PIN2=4;
 		I2C1_CR1->SWRST=1;//RESET

 		I2C1_CR1->SWRST=0;//STOP RESET
 		I2C1_CR2->FREQ=45;//SETUP 45MHZ max 50
 		I2C_CCR->CCR=225; //I2C CLOCK FREQUENCY, CCR=(TR(SCL)+TW(SCLH) )/ TPCLK1
 		// PROBLEMA POR DISTINTA PLACA, HAY QUE VER CON EL USERMANUAL PG 142 QUE VALOR PONER PARA NUESTRO CCR O COMO VA



 		I2C_TRISE->CCR = 46;// TRSCL/TPCLK1 +1


 		//this line ends the setup now we test for real
 		I2C1_CR1->PE=1; //peripheral enable
 		I2C1_CR1->ACK=1;
 		I2C1_CR1->START=1; //starting com

 		while (!(I2C_SR1->TXE)); //IF I'M BUSY I CANT TRANSFER, I2C RULING
 		I2C_DR->DR = data;	// IF I'M FREE I TRANSFER
 		while (!(I2C_SR1->BTF));	// I WAIT UNTIL I GET CONFIRMATION OF BIT TRANSFER FINISH



	for(;;);
}
