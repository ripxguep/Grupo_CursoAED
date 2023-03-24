/*
 * stm32429zi_gpio.c
 *
 *  Created on: Mar 14, 2023
 *      Author: hcg-c
 */

#include	"stm32429zi_gpio.h"

//Peripheral Clock Setup

//@fn				-GPIO_PeriClockControl
//brief				-This function enable or disable peripheral clock for a given GPIO port
//@param[in]		-base address of gpio peripheral
//@param[in]		-ENABLE or Disable	macros
//@param[in]		-
//@return			-none
//@Note				-none

void	GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi){
	if(EnorDi==ENABLE){
		if(pGPIOx==GPIOA){
			GPIOA_PCLCK_EN();

		}
	else if(pGPIOx==GPIOB)
			{
		GPIOB_PCLCK_EN();
			}
	else if(pGPIOx==GPIOC)
				{
			GPIOC_PCLCK_EN();
				}
	else if(pGPIOx==GPIOD)
				{
			GPIOD_PCLCK_EN();
				}
	else if(pGPIOx==GPIOE)
				{
			GPIOE_PCLCK_EN();
				}
	else if(pGPIOx==GPIOF)
				{
			GPIOF_PCLCK_EN();
				}
	else if(pGPIOx==GPIOG)
				{
			GPIOG_PCLCK_EN();
				}
	else if(pGPIOx==GPIOH)
				{
			GPIOH_PCLCK_EN();
				}
	else if(pGPIOx==GPIOI)
				{
			GPIOI_PCLCK_EN();
				}
	}
	else{
		if(pGPIOx==GPIOA){
					GPIOA_PCLCK_DI();
				}
			else if(pGPIOx==GPIOB)
					{
				GPIOB_PCLCK_DI();
					}
			else if(pGPIOx==GPIOC)
						{
					GPIOC_PCLCK_DI();
						}
			else if(pGPIOx==GPIOD)
						{
					GPIOD_PCLCK_DI();
						}
			else if(pGPIOx==GPIOE)
						{
					GPIOE_PCLCK_DI();
						}
			else if(pGPIOx==GPIOF)
						{
					GPIOF_PCLCK_DI();
						}
			else if(pGPIOx==GPIOG)
						{
					GPIOG_PCLCK_DI();
						}
			else if(pGPIOx==GPIOH)
						{
					GPIOH_PCLCK_DI();
						}
			else if(pGPIOx==GPIOI)
						{
					GPIOI_PCLCK_DI();
						}
	}
}
//Init and deinit
//@fn				-GPIO_Init
//brief				-This function enable or disable peripheral clock for a given GPIO port
//@param[in]		-base address of gpio peripheral
//@param[in]		-ENABLE or Disable	macros
//@param[in]		-
//@return			-none
//@Note				-none
void	GPIO_Init(GPIO_Handle_t	*pGPIOHandle){
	// 1.configure mode of gpio pin
	uint32_t temp=0;	//temp register

	//enable peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<=GPIO_MODE_ANALOG){
		//no interrupt mode
		temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		//clear
		pGPIOHandle->pGPIOx->MODER&=~(0x3<<2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	//setting
		pGPIOHandle->pGPIOx->MODER|=temp;
	}else	//interrupt mode
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_FT){
			//1. configure the FTSR

			EXTI->FTSR|=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding RTSR bit
			EXTI->RTSR&=~(0x1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_RT){
			//1.configure RTSR
			EXTI->RTSR|=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
						//Clear the corresponding RTSR bit
						EXTI->FTSR&=~(0x1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_RFT){
			//1.configure both FTSR and RTSR
			EXTI->FTSR|=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
						//Clear the corresponding RTSR bit
						EXTI->RTSR|=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		//2. configure GPIO port selection in SYSCFG_EXTICR
		uint8_t	temp1	=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;
		uint8_t	temp2	=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4;
		uint8_t	portcode	=GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
						SYSCFG_PCLK_EN();
							SYSCFG->EXTICR[temp1]=portcode<<(temp2*4);
		//3.enable the exti interrupt delivery using IMR
		EXTI->IMR|=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	temp=0;


	// 2.configure speed
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR|=temp;
	temp=0;
	// 3.configure pupd settings
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	//clear
			pGPIOHandle->pGPIOx->PUPDR&=~(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		//setting
			pGPIOHandle->pGPIOx->PUPDR|=temp;
		temp=0;
	// 4. configure the optype
		temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType<<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		//clear
				pGPIOHandle->pGPIOx->OTYPER&=~(0x1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//setting
		pGPIOHandle->pGPIOx->OTYPER|=temp;
				temp=0;
	// 5. configure the alt functionality
				if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_ALTFN)
				{
					//CONFIGURE THE ALT FUNCTION REGISTERS.
					uint32_t	temp1,temp2;
					temp1=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8;
					temp2=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8;

					//CLEAR
					pGPIOHandle->pGPIOx->AFR[temp1]&=~(0xF<<(4*temp2));
					//SET
					pGPIOHandle->pGPIOx->AFR[temp1]|=(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode<<(4*temp2));

				}


}
//@fn				-GPIO_DeInit
//brief				-This function resets peripheral clock for a given GPIO port
//@param[in]		-base address of gpio peripheral
//@param[in]		-
//@param[in]		-
//@return			-none
//@Note				-none
void	GPIO_DeInit(GPIO_RegDef_t *pGPIOx){

			if(pGPIOx==GPIOA){
				GPIOA_REG_RESET();
			}
		else if(pGPIOx==GPIOB)
				{
			GPIOB_REG_RESET();
				}
		else if(pGPIOx==GPIOC)
					{
				GPIOC_REG_RESET();
					}
		else if(pGPIOx==GPIOD)
					{
				GPIOD_REG_RESET();
					}
		else if(pGPIOx==GPIOE)
					{
				GPIOE_REG_RESET();
					}
		else if(pGPIOx==GPIOF)
					{
				GPIOF_REG_RESET();
					}
		else if(pGPIOx==GPIOG)
					{
				GPIOG_REG_RESET();
					}
		else if(pGPIOx==GPIOH)
					{
				GPIOH_REG_RESET();
					}
		else if(pGPIOx==GPIOI)
					{
				GPIOI_REG_RESET();
					}


}
//Data read and write

//@fn				-GPIO_ReadFromInputPin
//brief				-This function reads a bit from a given pin of a given port
//@param[in]		-base address of gpio peripheral
//@param[in]		-number of pin (0..15)
//@param[in]		-
//@return			-read value from pin (0 or 1)
//@Note				-none
uint8_t	GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t	PinNumber){
	uint8_t value;
	value=(uint8_t)(pGPIOx->IDR>>PinNumber&0X00000001);
	return value;
}
//@fn				-GPIO_ReadFromInputPort
//brief				-This function reads a value from a port and write its in the console
//@param[in]		-base address of gpio peripheral
//@param[in]		-
//@param[in]		-
//@return			-value from port 16 bits long
//@Note				-none
uint16_t	GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint8_t value;
		value=(uint16_t)(pGPIOx->IDR);
		return value;
}
//@fn				-GPIO_WriteToOutputPin
//brief				-Writes a value to a determined pin
//@param[in]		-base address of gpio peripheral
//@param[in]		-Number of pin
//@param[in]		-value to write
//@return			-none
//@Note				-none
void	GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t	PinNumber,uint8_t value){
	if(value==GPIO_PIN_SET){
		//write 1 to the output data register at the bit field corresponding to the pin
		pGPIOx->ODR |=(1<<PinNumber);
	}
	else{
		pGPIOx->ODR &=~(1<<PinNumber);
	}
}
//@fn				-GPIO_WriteToOutputPort
//brief				-Writes a value to a determined port
//@param[in]		-base address of gpio peripheral
//@param[in]		-
//@param[in]		-value to write
//@return			-none
//@Note				-none
void	GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t value){
	pGPIOx->ODR=value;
}
//@fn				-GPIO_TogglePin
//brief				-Toggles a led
//@param[in]		-base address of gpio peripheral
//@param[in]		-Number of pin
//@param[in]		-
//@return			-none
//@Note				-uses XOR Operator to XOR desired pin
void	GPIO_TogglePin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber){
	pGPIOx->ODR^=(1<<PinNumber);
}

//IRQ config and handling
//@fn				-GPIO_IRQConfig
//brief				-Sets the initialization a specific interruption
//@param[in]		-number of interruption
//@param[in]		-Interruption priority
//@param[in]		-Disable or enable the interruption
//@return			-none
//@Note				-none
void	GPIO_IRQConfig(uint8_t	IRQNumber,uint8_t	EnorDi){
	{

		if(EnorDi == ENABLE)
		{
			if(IRQNumber <= 31)
			{
				//program ISER0 register
				*NVIC_ISER0 |= ( 1 << IRQNumber );

			}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
			{
				//program ISER1 register
				*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
			}
			else if(IRQNumber >= 64 && IRQNumber < 96 )
			{
				//program ISER2 register //64 to 95
				*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
			}
		}else
		{
			if(IRQNumber <= 31)
			{
				//program ICER0 register
				*NVIC_ICER0 |= ( 1 << IRQNumber );
			}else if(IRQNumber > 31 && IRQNumber < 64 )
			{
				//program ICER1 register
				*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
			}
			else if(IRQNumber >= 64 && IRQNumber < 96 )
			{
				//program ICER2 register
				*NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
			}
		}

	}
}
//@fn				-GPIO_IRQPriorityConfig
//brief				-Manages interruption priority for a given IRQNumber
//@param[in]		-number of IRQ
//@param[in]		-priority stablished
//@param[in]		-
//@return			-none
//@Note				-none
void	GPIO_IRQPriorityConfig(uint8_t	IRQNumber,uint32_t	IRQPriority){
	uint8_t iprx	=IRQNumber/4;
	uint8_t iprx_section	=IRQNumber%4;
	uint8_t	shift_amount	=(8*iprx_section)+(8-NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR+iprx)|=(IRQPriority<<(shift_amount));
}
//@fn				-GPIO_IRQHandling
//brief				-Manages interruption behavior
//@param[in]		-number of pin where it happened
//@param[in]		-
//@param[in]		-
//@return			-none
//@Note				-none
void	GPIO_IRQHandling(uint8_t PinNumber){

	if(EXTI->PR &(1<<PinNumber)){
		//clear

		EXTI->PR|=(1<<PinNumber);
	}
}
uint8_t	GPIO_regc32handling(uint32_t prevregister){
	uint8_t pos=0;
	prevregister|=EXTI->PR;
	if(prevregister==0){
		return 33;
	}
	while(prevregister){
		prevregister=prevregister>>1;
		pos++;
	}

	return pos;
}
