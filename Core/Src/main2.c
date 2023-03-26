/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"
#include "stdio.h"
#include "stm32f4xx_hal.h"
void determinamensajei2c(uint8_t pinn);
 void I2C1_Init(void);
 void I2C2_Init(void);
void GPIO_Init(void);
void Error_handler(void);

void SystemClock_Config_HSE(uint8_t clock_freq);
void CAN1_Init(void);
void CAN1_Tx();
void I2C1_EV_IRQHandler(void);
void CAN1_Rx(void);
void CAN_Filter_Config(void);
void TIMER6_Init(void);
void CAN_Send_response(uint32_t StdId);
void LED_Manage_Output(uint8_t led_no);
void starti2c_tx(uint8_t rcvd_msg[]);
uint8_t determinamensajeCAN(uint8_t pinn,uint8_t trigger);
#define I2C_BUFFER_SIZE 1
uint8_t i2c_buffer;//tx buffer
uint8_t aTxBuffer[] = "00011010";
uint8_t memory[];
uint8_t rxbuffer[RXBUFFERSIZE];
uint8_t reg_addr_rcvd = 0;
uint8_t led_conf[5];
#define I2C_REG_ADD_SIZE        1
#define I2C_PAYLOAD_SIZE        4
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
UART_HandleTypeDef huart2;
CAN_HandleTypeDef hcan1;
TIM_HandleTypeDef htimer6;
uint8_t req_counter = 0;
CAN_RxHeaderTypeDef RxHeader;
 uint16_t pinn;
#define ADDR_SLAVE		0x1 <<1
uint8_t ledst[INPUTPINCOUNT];
/* comment this line to use the board as slave */
#define MASTER_BOARD
#define I2C_ADDRESS        ADDR_SLAVE
/* uncomment this line to use the board as Button board */
//#define BTN_BOARD
#ifndef MASTER_BOARD
//does the addr match?
__IO	uint32_t	uwtfrq =0;
#endif
int main(void)
{

	HAL_Init();

	SystemClock_Config_HSE(SYS_CLOCK_FREQ_50_MHZ); //50 mhz

	GPIO_Init(); //sets buttons and leds



	TIMER6_Init(); //can stuff
	I2C1_Init();
	I2C2_Init();
	CAN1_Init();

	//we start can coms
		CAN_Filter_Config();


		if(HAL_CAN_ActivateNotification(&hcan1,CAN_IT_TX_MAILBOX_EMPTY|CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_BUSOFF)!= HAL_OK)
			{
					Error_handler();
			}


			if( HAL_CAN_Start(&hcan1) != HAL_OK)
			{
				Error_handler();
			}

//we wait for button event
for(int n=0;n<INPUTPINCOUNT;n++){
	ledst[n]=0;
}
#ifdef BTN_BOARD
while(1){
	int pinn=0;
	while(pinn==0){ //we wait for set
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_3 )==GPIO_PIN_SET){
				pinn=3;
			}else if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_4 )==GPIO_PIN_SET){
				pinn=4;
				}else if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_5 )==GPIO_PIN_SET){
					pinn=5;
					}else if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_6 )==GPIO_PIN_SET){
						pinn=6;
						}else if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_7 )==GPIO_PIN_SET){
							pinn=7;
							}
	}
	//we wait for reset
	pinn=0;
		while(pinn==0){
				if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_3 )==GPIO_PIN_SET){
					pinn=3;
				}else if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_4 )==GPIO_PIN_SET){
					pinn=4;
					}else if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_5 )==GPIO_PIN_SET){
						pinn=5;
						}else if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_6 )==GPIO_PIN_SET){
							pinn=6;
							}else if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_7 )==GPIO_PIN_SET){
								pinn=7;
								}
		}
}
#endif
#ifdef MASTER_BOARD


#else
		  if(HAL_I2C_EnableListen_IT(&hi2c2) != HAL_OK)
		  {
		     /* Transfer error in reception process */
		     Error_handler();
		   }
		  while(uwtfrq != 1)
		    {
		    }
		//  Put I2C2 peripheral in reception proces
		  if(HAL_I2C_Slave_Seq_Receive_IT(&hi2c2, (uint8_t *)rxbuffer, RXBUFFERSIZE, I2C_FIRST_FRAME) != HAL_OK)
		    {
		      /* Transfer error in reception process */
		      Error_handler();
		    }
		  //wait for end of transfer
		  while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_LISTEN)
		    {
		    }
		  //While the I2C in reception process, user can transmit data through
		  if(HAL_I2C_Slave_Seq_Transmit_IT(&hi2c2, (uint8_t*)aTxBuffer, TXBUFFERSIZE, I2C_LAST_FRAME)!= HAL_OK)
		   {
		     /* Transfer error in transmission process */
		     Error_handler();
		   }
	#endif  /*master or slave*/

		  //wait for End of transfer in this test case of last
		  //msg receiver(master)

		  while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
		    {
		    }
#ifndef	MASTER_BOARD
		  LED_Manage_Output(rxbuffer);
#endif







	return 0;
}



uint8_t trigger=0;
void determinamensajei2c(uint8_t pinn){
	uint8_t temp=0;
		 		switch(pinn)
		 		 	{
		 		 	case 2 :

		 		 		temp=ledst[pinn-3];
		 		 		temp&=~(0x11011111);
		 		 		if(temp & 0x20){
		 		 			 i2c_buffer&=~(1<<5);
		 		 			}else {
			 		 			i2c_buffer|=(1<<5);
			 		 		}
		 		 		ledst[pinn-3]=i2c_buffer;
		 		 		i2c_buffer&=~(11<<0);
		 		 		i2c_buffer|=(1<<2);
		 		 		i2c_buffer&=~(11<<3);


		 		 		break;
		 		 	case 3 :
		 		 temp=ledst[pinn-3];
		 		 			temp&=~(0x11011111);
		 		 		if(temp & 0x20){
		 		 			 i2c_buffer&=~(1<<5);
		 		 	}else {
		 				i2c_buffer|=(1<<5);
		 		  		}
		 		 		ledst[pinn-3]=i2c_buffer;
		 		 		i2c_buffer&=~(11<<0);
		 		 	i2c_buffer|=(1<<2);
		 		 	i2c_buffer&=~(11<<3);
		 		 		break;
		 		 	case 4 :
		 		 		  temp=ledst[pinn-3];
		 		 			 		 			temp&=~(0x11011111);
		 		 			 		 		if(temp & 0x20){
		 		 			 		 			 i2c_buffer&=~(1<<5);
		 		 			 		 	}else {
		 		 			 				i2c_buffer|=(1<<5);
		 		 			 		  		}
		 		 			 		 	ledst[pinn-3]=i2c_buffer;
		 		 			 		 		i2c_buffer&=~(11<<0);
		 		 			 		 	i2c_buffer|=(1<<2);
		 		 			 		 	i2c_buffer&=~(11<<3);
		 		 		break;
		 		 	case 5 :
		 		 		  temp=ledst[pinn-3];
		 		 			 		 			temp&=~(0x11011111);
		 		 			 		 		if(temp & 0x20){
		 		 			 		 			 i2c_buffer&=~(1<<5);
		 		 			 		 	}else {
		 		 			 				i2c_buffer|=(1<<5);
		 		 			 		  		}
		 		 			 		 	ledst[pinn-3]=i2c_buffer;
		 		 			 		 		i2c_buffer&=~(11<<0);
		 		 			 		 	i2c_buffer|=(1<<2);
		 		 			 		 	i2c_buffer&=~(11<<3);
		 		 		break;
		 		 	case 6 :
		 		 		 temp=ledst[pinn-3];
		 		 			 		 			temp&=~(0x11011111);
		 		 			 		 		if(temp & 0x20){
		 		 			 		 			 i2c_buffer&=~(1<<5);
		 		 			 		 	}else {
		 		 			 				i2c_buffer|=(1<<5);
		 		 			 		  		}
		 		 			 		 	ledst[pinn-3]=i2c_buffer;
		 		 			 		 		i2c_buffer&=~(11<<0);
		 		 			 		 	i2c_buffer|=(1<<2);
		 		 			 		 	i2c_buffer&=~(11<<3);
		 		 		 break;
		 		 	case 7 :
		 		 		 temp=ledst[pinn-3];
		 		 			 		 			temp&=~(0x11011111);
		 		 			 		 		if(temp & 0x20){
		 		 			 		 			 i2c_buffer&=~(1<<5);

		 		 			 		 	}else {
		 		 			 				i2c_buffer|=(1<<5);
		 		 			 		  		}
		 		 			 		 	ledst[pinn-3]=i2c_buffer;
		 		 			 		 		i2c_buffer&=~(11<<0);
		 		 			 		 	i2c_buffer|=(1<<2);
		 		 			 		 	i2c_buffer&=~(11<<3);
		 		 		 break;
		 		 	}

}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config_HSE(uint8_t clock_freq)
{
	RCC_OscInitTypeDef Osc_Init;
	RCC_ClkInitTypeDef Clock_Init;
    uint8_t flash_latency=0;

	Osc_Init.OscillatorType = RCC_OSCILLATORTYPE_HSE ;
	Osc_Init.HSEState = RCC_HSE_ON;
	Osc_Init.PLL.PLLState = RCC_PLL_ON;
	Osc_Init.PLL.PLLSource = RCC_PLLSOURCE_HSE;

	switch(clock_freq)
	 {
	  case SYS_CLOCK_FREQ_50_MHZ:
		  Osc_Init.PLL.PLLM = 4;
		  Osc_Init.PLL.PLLN = 50;
		  Osc_Init.PLL.PLLP = RCC_PLLP_DIV2;
		  Osc_Init.PLL.PLLQ = 2;

		  Clock_Init.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
		  Clock_Init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		  Clock_Init.AHBCLKDivider = RCC_SYSCLK_DIV1;
		  Clock_Init.APB1CLKDivider = RCC_HCLK_DIV2;
		  Clock_Init.APB2CLKDivider = RCC_HCLK_DIV1;
          flash_latency = 1;
	     break;

	  case SYS_CLOCK_FREQ_84_MHZ:
		  Osc_Init.PLL.PLLM = 4;
		  Osc_Init.PLL.PLLN = 84;
		  Osc_Init.PLL.PLLP = RCC_PLLP_DIV2;
		  Osc_Init.PLL.PLLQ = 2;

		  Clock_Init.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
		  Clock_Init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		  Clock_Init.AHBCLKDivider = RCC_SYSCLK_DIV1;
		  Clock_Init.APB1CLKDivider = RCC_HCLK_DIV2;
		  Clock_Init.APB2CLKDivider = RCC_HCLK_DIV1;
          flash_latency = 2;
	     break;

	  case SYS_CLOCK_FREQ_120_MHZ:
		  Osc_Init.PLL.PLLM = 4;
		  Osc_Init.PLL.PLLN = 120;
		  Osc_Init.PLL.PLLP = RCC_PLLP_DIV2;
		  Osc_Init.PLL.PLLQ = 2;

		  Clock_Init.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
		  Clock_Init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		  Clock_Init.AHBCLKDivider = RCC_SYSCLK_DIV1;
		  Clock_Init.APB1CLKDivider = RCC_HCLK_DIV4;
		  Clock_Init.APB2CLKDivider = RCC_HCLK_DIV2;
          flash_latency = 3;
	     break;

	  default:
	   return ;
	 }

		if (HAL_RCC_OscConfig(&Osc_Init) != HAL_OK)
	{
			Error_handler();
	}



	if (HAL_RCC_ClockConfig(&Clock_Init, flash_latency) != HAL_OK)
	{
		Error_handler();
	}


	/*Configure the systick timer interrupt frequency (for every 1 ms) */
	uint32_t hclk_freq = HAL_RCC_GetHCLKFreq();
	HAL_SYSTICK_Config(hclk_freq/1000);

	/**Configure the Systick
	*/
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);



 }


uint8_t determinamensajeCAN(uint8_t pinn,uint8_t trigger){

	switch(trigger){
	case 0:
		if(pinn<=5){
			return 0;
		}else if(pinn==6){
			return 1;
		}else {
			return 2;
		}
		break;
	case 1:
		return pinn;
	break;
	case 2:
		if (memory[pinn]==0){
			return 1;
			memory[pinn]=1;
		}

		else {return 0;
		memory[pinn]=0;}
		break;

}}


void CAN1_Tx()
{
	CAN_TxHeaderTypeDef TxHeader;

	uint32_t TxMailbox;

	uint8_t message;

	TxHeader.DLC = 1;
	TxHeader.StdId = 0x5B0;
	TxHeader.IDE   = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;

	while(trigger!=3){
	message =determinamensajeCAN(pinn,trigger);
	trigger++;


	HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_14);

	if( HAL_CAN_AddTxMessage(&hcan1,&TxHeader,&message,&TxMailbox) != HAL_OK)
	{
		Error_handler();
	}
	}

}

void CAN1_Rx(void)
{
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t rcvd_msg[3];

	char msg[50];

	//we are waiting for at least one message in to the RX FIFO0
	while(! HAL_CAN_GetRxFifoFillLevel(&hcan1,CAN_RX_FIFO0));

	if(HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&RxHeader,rcvd_msg) != HAL_OK)
	{
		Error_handler();
	}

	sprintf(msg,"Message Received : %s\r\n",rcvd_msg);

	starti2c_tx(rcvd_msg);

}
uint8_t decipher_pin(uint8_t rcvd_msg[],uint8_t pinn){
	uint8_t temp1=0;

	switch(rcvd_msg[0]){
	case 0:
		//single led
		temp1=rcvd_msg[1];
		break;
	case 1:
		//effect
			temp1=6;
			break;
	case 2:
		//alarm
			temp1=7;
			break;
	}
	return temp1;
	}

void starti2c_tx(uint8_t rcvd_msg[]){
	uint8_t pinn=0;
		pinn=decipher_pin(rcvd_msg,pinn);
		determinamensajei2c(pinn);	//this creates the sequence to send to trigger the leds
		//start tx
		 while(HAL_I2C_Master_Seq_Transmit_IT(&hi2c1, (uint16_t)I2C_ADDRESS, &i2c_buffer, sizeof(i2c_buffer), I2C_FIRST_FRAME)!= HAL_OK)
		  {
			 if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
			    {
			      Error_handler();
			    }
			  }
		 	//wait for end of tx
		 while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
		   {
		   }
		 //Put I2C peripheral in reception process
		 while(HAL_I2C_Master_Seq_Receive_IT(&hi2c1, (uint16_t)I2C_ADDRESS, (uint8_t *)rxbuffer, RXBUFFERSIZE, I2C_LAST_FRAME) != HAL_OK)
		  {
			  if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
			    {
			      Error_handler();
			    }
			  }
}

void CAN_Filter_Config(void)
{
	CAN_FilterTypeDef can1_filter_init;

	can1_filter_init.FilterActivation = ENABLE;
	can1_filter_init.FilterBank  = 0;
	can1_filter_init.FilterFIFOAssignment = CAN_RX_FIFO0;
	can1_filter_init.FilterIdHigh = 0x0000;
	can1_filter_init.FilterIdLow = 0x0000;
	can1_filter_init.FilterMaskIdHigh = 0X01C0;
	can1_filter_init.FilterMaskIdLow = 0x0000;
	can1_filter_init.FilterMode = CAN_FILTERMODE_IDMASK;
	can1_filter_init.FilterScale = CAN_FILTERSCALE_32BIT;

	if( HAL_CAN_ConfigFilter(&hcan1,&can1_filter_init) != HAL_OK)
	{
		Error_handler();
	}

}


void GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();


	GPIO_InitTypeDef ledgpio,buttongpio;


	ledgpio.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
	ledgpio.Mode = GPIO_MODE_OUTPUT_PP;
	ledgpio.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOE,&ledgpio);



	buttongpio.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
	buttongpio.Mode = GPIO_MODE_INPUT;
	buttongpio.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC,&buttongpio);




}


void TIMER6_Init(void)
{
	htimer6.Instance = TIM6;
	htimer6.Init.Prescaler = 4999;
	htimer6.Init.Period = 10000-1;
	if( HAL_TIM_Base_Init(&htimer6) != HAL_OK )
	{
		Error_handler();
	}

}



void CAN1_Init(void)
{
	hcan1.Instance = CAN1;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.AutoBusOff = ENABLE;
	hcan1.Init.AutoRetransmission = ENABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;

	//Settings related to CAN bit timings
	hcan1.Init.Prescaler = 3;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;

	if ( HAL_CAN_Init (&hcan1) != HAL_OK)
	{
		Error_handler();
	}

}


void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
	char msg[50];
	sprintf(msg,"Message Transmitted:M0\r\n");
	HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);

}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
	char msg[50];
	sprintf(msg,"Message Transmitted:M1\r\n");
	HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);

}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
	char msg[50];
	sprintf(msg,"Message Transmitted:M2\r\n");
	HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);

}

 void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t rcvd_msg[8];

	char msg[50];

	if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RxHeader,rcvd_msg) != HAL_OK)
	{
		Error_handler();
	}

	if(RxHeader.StdId == 0x5B0 && RxHeader.RTR == 0 )
	{
		//This is data frame sent by n1 to n2
	//	LED_Manage_Output(rcvd_msg[0]);
		sprintf(msg,"Message Received : #%x\r\n",rcvd_msg[0]);
	}
	else if ( RxHeader.StdId == 0x5B0 && RxHeader.RTR == 1)
	{
		//This is a remote frame sent by n1 to n2
		CAN_Send_response(RxHeader.StdId);
		return;
	}
	else if ( RxHeader.StdId == 0x5A0 && RxHeader.RTR == 0)
	{
		//its a reply ( data frame) by n2 to n1
		sprintf(msg,"Reply Received : %#X\r\n",rcvd_msg[0] << 8 | rcvd_msg[1]);
	}

	 HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);

}



 void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
 {
	 CAN_TxHeaderTypeDef TxHeader;

	 uint32_t TxMailbox;

	 uint8_t message; //no meaning for data frame

	if ( req_counter  == 4)
	{
		//N1 sending Remote frame to N2
		TxHeader.DLC = 2; //N1 demanding 2 bytes of reply
		TxHeader.StdId = 0x5B0;
		TxHeader.IDE   = CAN_ID_STD;
		TxHeader.RTR = CAN_RTR_REMOTE;

		if( HAL_CAN_AddTxMessage(&hcan1,&TxHeader,&message,&TxMailbox) != HAL_OK)
		{
			Error_handler();
		}
		req_counter = 0;

	}else
	{
		CAN1_Tx();
		req_counter++;
	}

 }
#ifndef MASTER_BOARD
 void LED_Manage_Output(uint8_t I2C_MSG)
 {

	 uint8_t mode=I2C_MSG;
	 mode&=~(11111001<<mode);
	 mode|=(mode>>1);
	 uint8_t id=I2C_MSG;
	 id&=~(11100111<<id);
	 id|=(id>>3);
	 uint8_t onoff=I2C_MSG;
	 onoff&=~(11011111<<onoff);

 	switch(mode)
 	{
 	case 1 :
 		HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_2+id*4);
 		//we toggle single pin
 		break;
 	case 2 :
 		gestor_efectos(id,mode);
 		break;
 	case 3 :
 		gestor_alarma();
 		break;

 	}
 }
 void gestor_alarma(){
	 HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_2);
	 HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_3);
	 HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_4);
	 HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_5);
	 HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_6);
 }
void gestor_efectos(uint8_t id,uint8_t mode){
	 switch(id){
	 case 1:
		while(mode!=3){
			for(int u=0;u<=4;u++){
				HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_2+4*u);
									 HAL_Delay(1);
			}

		}


	 		 break;
	 case 2:
		 while(mode!=3){
		 			for(int u=0;u<=4;u++){
		 				HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_6-4*u);
		 									 HAL_Delay(1);
		 			}

		 		}
	 		 break;
	 case 3:
		 while(mode!=3){
		 		 	for(int u=0;u<=2;u++){
		 		 		HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_6);
		 		 		HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_6-4*u);
		 		 						 HAL_Delay(1);
		 		 		HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_2+4*u);
		 		 						 HAL_Delay(1);
		 		 			}

		 		 		}
	 		 break;
	 case 4:
		 for(int u=0;u<=2;u++){
		 		 		 		HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_6);
		 		 		 	 HAL_Delay(1);
		 		 		 		HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_5-4*u);
		 		 		 						 HAL_Delay(1);
		 		 		 		HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_3+4*u);
		 		 		 						 HAL_Delay(1);
		 		 		 			}

		 		 		 		}
	 		 break;
	 }
 }
#endif


 void CAN_Send_response(uint32_t StdId)
 {

 	CAN_TxHeaderTypeDef TxHeader;

 	uint32_t TxMailbox;

 	uint8_t response[2] = { 0xAB,0XCD};

 	TxHeader.DLC = 2;
 	TxHeader.StdId = StdId;
 	TxHeader.IDE   = 0x5A0;
 	TxHeader.RTR = CAN_RTR_DATA;

 	if( HAL_CAN_AddTxMessage(&hcan1,&TxHeader,response,&TxMailbox) != HAL_OK)
 	{
 		Error_handler();
 	}

 }
 void I2C1_Init(void)
	 {

	   /* USER CODE BEGIN I2C1_Init 0 */

	   /* USER CODE END I2C1_Init 0 */

	   /* USER CODE BEGIN I2C1_Init 1 */

	   /* USER CODE END I2C1_Init 1 */
	   hi2c1.Instance = I2C1;
	   hi2c1.Init.ClockSpeed = 400000;
	   hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	   hi2c1.Init.OwnAddress1 = 0;
	   hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	   hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	   hi2c1.Init.OwnAddress2 = 0;
	   hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	   hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	   if(HAL_I2C_Init(&hi2c1) != HAL_OK)
	   	     {
	   	       /* Initialization Error */
	   	       Error_handler();
	   	     }

	 }

	 /**
	   * @brief I2C2 Initialization Function
	   * @param None
	   * @retval None
	   */
	  void I2C2_Init(void)
	 {

	   /* USER CODE BEGIN I2C2_Init 0 */

	   /* USER CODE END I2C2_Init 0 */

	   /* USER CODE BEGIN I2C2_Init 1 */

	   /* USER CODE END I2C2_Init 1 */
	   hi2c2.Instance = I2C2;
	   hi2c2.Init.ClockSpeed = 400000;
	   hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	   hi2c2.Init.OwnAddress1 = ADDR_SLAVE;
	   hi2c2.Init.OwnAddress2 = 0xFF;
	   hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	   hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	   hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	   hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	   if(HAL_I2C_Init(&hi2c2) != HAL_OK)
	     {
	       /* Initialization Error */
	       Error_handler();
	     }




	 }

	  void I2C1_EV_IRQHandler(void) {

	  	HAL_I2C_EV_IRQHandler(&hi2c1);
	  }
	  void I2C1_ER_IRQHandler(void) {
	  	HAL_I2C_ER_IRQHandler(&hi2c1);
	  }
	  void I2C2_EV_IRQHandler(void) {
	  	HAL_I2C_EV_IRQHandler(&hi2c2);
	  }
	  void I2C2_ER_IRQHandler(void) {
	  	HAL_I2C_ER_IRQHandler(&hi2c2);
	  }

	  /**
	    * @brief  Tx Transfer completed callback.
	    * @param  I2cHandle: I2C handle
	    * @note   This example shows a simple way to report end of IT Tx transfer, and
	    *         you can add your own implementation.
	    * @retval None
	    */
	  #ifdef MASTER_BOARD
	  void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
	  {
	    /* Turn LED3 on: Transfer in transmission process is correct */
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,  1);
	  }
	  #else
	  void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
	  {
	    /* Turn LED3 on: Transfer in transmission process is correct */
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,  1);
	  }
	  #endif /* MASTER_BOARD */


	  /**
	    * @brief  Rx Transfer completed callback.
	    * @param  I2cHandle: I2C handle
	    * @note   This example shows a simple way to report end of IT Rx transfer, and
	    *         you can add your own implementation.
	    * @retval None
	    */
	  #ifdef MASTER_BOARD
	  void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
	  {
	    /* Turn LED1 on: Transfer in reception process is correct */
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,  1);
	  }
	  #else
	  void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
	  {
	    /* Turn LED1 on: Transfer in reception process is correct */
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,  1);
	  }
	  #endif /* MASTER_BOARD */

	  #ifndef MASTER_BOARD
	  /**
	    * @brief  Slave Address Match callback.
	    * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
	    *                the configuration information for the specified I2C.
	    * @param  TransferDirection: Master request Transfer Direction (Write/Read), value of @ref I2C_XferOptions_definition
	    * @param  AddrMatchCode: Address Match Code
	    * @retval None
	    */
	  void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
	  {
		  uwtfrq = 1;

	    /* A new communication with a Master is initiated */
	    /* Turn LED2 On: A Communication is initiated */
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,  1);
	  }

	  /**
	    * @brief  Listen Complete callback.
	    * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
	    *                the configuration information for the specified I2C.
	    * @retval None
	    */
	  void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
	  {
	    /* Turn LED2 off: Communication is completed */
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,  1);
	  }
	  #endif
	  void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle)
	  {
	    /* Turn Off LED1 */
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,  0);

	    /* Turn Off LED2 */
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,  0);


	    /* Turn Off LED3 */
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,  1);
	  }

	  void Error_handler(void)
	  {
	  	while(1);
	  }
