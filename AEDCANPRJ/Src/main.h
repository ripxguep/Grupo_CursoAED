/*
 * main.h
 *
 *  Created on: 25 feb. 2023
 *      Author: hcg-c
 */
#include <stdint.h>
#ifndef MAIN_H_
#define MAIN_H_
//register definition

//me creo direcciones para acceder de distintos tipos
//el nombre no es muy relevante r

#define  ADDR_REG_AHB1ENR  ((RCC_AHB1ENR_t*) 0x40023830)
#define	 ADDR_REG_APB1ENR  ((RCC_APB1ENR_t*) 0x40023840)
#define	 ADDR_REG_GPIO_HEAD1  ((GPIOx_PINS_t*) 0x40020400)
#define	 ADDR_REG_GPIO_HEAD2  ((GPIOx_xDR_t*) 0x40020404)
#define	 ADDR_REG_GPIO_HEAD3  ((GPIOx_AFRx_t*) 0x40020424)

#define	 ADDR_REG_GPIOB_OD  ((GPIOx_xDR_t*) 0x40020414)
#define	 ADDR_REG_GPIOB_ID  ((GPIOx_xDR_t*) 0x40020410)

#define	 ADDR_REG_I2C_HEAD1  ((I2Cx_CR1_t*) 0x40005400)
#define  ADDR_REG_I2C_HEAD2		((I2Cx_CR2_t*) 0x40005404)
#define  ADDR_REG_I2C_HEAD3		((I2Cx_CCR_t*) 0x4000541C)
#define  ADDR_REG_I2C_HEAD4		((I2C_TRISE_t*) 0x40005420)
#define  ADDR_REG_I2C_HEAD5		((I2C_SR1_t*) 0x40005410)
#define  ADDR_REG_I2C_HEAD6		((I2C_DR_t*) 0x40005414)


//status of registers
#define CLOCK_ENABLE	1
#define	MODE_CONF_OP	1
#define	PIN_HIGH		1
#define	PIN_LOW			1
//define register structures
typedef	struct{
	uint32_t	tim2_en		:1;
	uint32_t	tim3_en		:1;
	uint32_t	tim4_en		:1;
	uint32_t	tim5_en		:1;
	uint32_t	tim6_en		:1;
	uint32_t	tim7_en		:1;
	uint32_t	tim12_en		:1;
	uint32_t	tim13_en		:1;
	uint32_t	tim14_en		:1;
	uint32_t	RESERVED_1		:2;
	uint32_t	wwdg_en			:1;
	uint32_t	RESERVED_2		:2;
	uint32_t	spi2_en			:1;
	uint32_t	spi3_en			:1;
	uint32_t	RESERVED_3		:1;
	uint32_t	usart2_en		:1;
	uint32_t	usart3_en		:1;
	uint32_t	uart4_en		:1;
	uint32_t	uart5_en		:1;
	uint32_t	i2c1_en			:1;
	uint32_t	i2c2_en			:1;
	uint32_t	RESERVED_4		:1;
	uint32_t	can1_en			:1;
	uint32_t	can2_en			:1;
	uint32_t	RESERVED_5		:1;
	uint32_t	pwr_en			:1;
	uint32_t	dac_en			:1;
	uint32_t	RESERVED_6		:12;
}RCC_APB1ENR_t;
typedef	struct{
	uint32_t	gpioa_en		:1;
	uint32_t	gpiob_en		:1;
	uint32_t	gpioc_en		:1;
	uint32_t	gpiod_en		:1;
	uint32_t	gpioe_en		:1;
	uint32_t	gpiof_en		:1;
	uint32_t	gpiog_en		:1;
	uint32_t	gpioh_en		:1;
	uint32_t	gpioi_en		:1;
	uint32_t	gpioj_en		:1;
	uint32_t	gpioak_en		:1;
	uint32_t	RESERVED_1		:1;
	uint32_t	CRCEN			:1;
	uint32_t	RESERVED_2		:5;
	uint32_t	BKPSRAMEN		:1;
	uint32_t	RESERVED_3		:1;
	uint32_t	CCMDATARAMEN	:1;
	uint32_t	DMA1EN			:1;
	uint32_t	DMA2EN			:1;
	uint32_t	DMA2DEN			:1;
	uint32_t	RESERVED_4		:1;
	uint32_t	ETHMACEN		:1;
	uint32_t	ETHMACTXEN		:1;
	uint32_t	ETHMACRXEN		:1;
	uint32_t	ETHMACPTPEN		:1;
	uint32_t	OTGHSEN			:1;
	uint32_t	OTGHSULPIEN		:1;
	uint32_t	RESERVED_5		:1;
}RCC_AHB1ENR_t;
typedef	struct{
	uint32_t	PIN0 		:2;
	uint32_t	PIN1		:2;
	uint32_t	PIN2		:2;
	uint32_t	PIN3		:2;
	uint32_t	PIN4		:2;
	uint32_t	PIN5		:2;
	uint32_t	PIN6		:2;
	uint32_t	PIN7		:2;
	uint32_t	PIN8		:2;
	uint32_t	PIN9		:2;
	uint32_t	PIN10		:2;
	uint32_t	PIN11		:2;
	uint32_t	PIN12		:2;
	uint32_t	PIN13		:2;
	uint32_t	PIN14		:2;
	uint32_t	PIN15		:2;


}GPIOx_PINS_t;
typedef	struct{
	uint32_t	PIN0 		:4;
	uint32_t	PIN1		:4;
	uint32_t	PIN2		:4;
	uint32_t	PIN3		:4;
	uint32_t	PIN4		:4;
	uint32_t	PIN5		:4;
	uint32_t	PIN6		:4;
	uint32_t	PIN7		:4;



}GPIOx_AFRx_t;
typedef	struct{
	uint32_t	PIN0		:1;
	uint32_t	PIN1		:1;
	uint32_t	PIN2		:1;
	uint32_t	PIN3		:1;
	uint32_t	PIN4		:1;
	uint32_t	PIN5		:1;
	uint32_t	PIN6		:1;
	uint32_t	PIN7		:1;
	uint32_t	PIN8		:1;
	uint32_t	PIN9		:1;
	uint32_t	PIN10		:1;
	uint32_t	PIN11		:1;
	uint32_t	PIN12		:1;
	uint32_t	PIN13		:1;
	uint32_t	PIN14		:1;
	uint32_t	PIN15		:1;
	uint32_t	RESERVED 		:16;

}GPIOx_xDR_t;
typedef	struct{
	uint16_t	PE 				:1;
	uint16_t	SM_BUS			:1;
	uint16_t	RESERVED_1		:1;
	uint16_t	SMB_TYPE		:1;
	uint16_t	ENARP			:1;
	uint16_t	ENPEC			:1;
	uint16_t	ENGC			:1;
	uint16_t	NO_STRETCH		:1;
	uint16_t	START			:1;
	uint16_t	STOP			:1;
	uint16_t	ACK				:1;
	uint16_t	POS				:1;
	uint16_t	PEC				:1;
	uint16_t	ALERT			:1;
	uint16_t	RESERVED_2		:1;
	uint16_t	SWRST			:1;


}I2Cx_CR1_t;
typedef	struct{
	uint16_t	FREQ			:6;
	uint16_t	RESERVED_1		:2;
	uint16_t	ITERREN			:1;
	uint16_t	ITEVTEN			:1;
	uint16_t	ITBUFEN			:1;
	uint16_t	DMAEN			:1;
	uint16_t	LAST			:1;
	uint16_t	RESERVED_2		:3;


}I2Cx_CR2_t;
typedef	struct{
	uint16_t	CCR				:12;
	uint16_t	RESERVED_1		:2;
	uint16_t	DUTY			:1;
	uint16_t	F_S				:1;

}I2Cx_CCR_t;
typedef	struct{
	uint16_t	CCR				:6;
	uint16_t	RESERVED_1		:10;

}I2C_TRISE_t;
typedef	struct{
	uint16_t	DR				:8;
	uint16_t	RESERVED_1		:8;

}I2C_DR_t;
typedef	struct{
	uint16_t	SMB_ALERT				:1;
	uint16_t	TIME_OUT				:1;
	uint16_t	RESERVED_2				:1;
	uint16_t	PEC_ERR					:1;
	uint16_t	OVR						:1;
	uint16_t	AF						:1;
	uint16_t	ARLO					:1;
	uint16_t	BERR					:1;
	uint16_t	TXE						:1;
	uint16_t	RXNE					:1;
	uint16_t	RESERVED_1				:1;
	uint16_t	STOPF					:1;
	uint16_t	ADD10					:1;
	uint16_t	BTF						:1;
	uint16_t	ADDR					:1;
	uint16_t	SB						:1;


}I2C_SR1_t;
#endif /* MAIN_H_ */
