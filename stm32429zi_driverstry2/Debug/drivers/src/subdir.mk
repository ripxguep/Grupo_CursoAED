################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/src/stm32429zi_I2C.c \
../drivers/src/stm32429zi_gpio.c 

OBJS += \
./drivers/src/stm32429zi_I2C.o \
./drivers/src/stm32429zi_gpio.o 

C_DEPS += \
./drivers/src/stm32429zi_I2C.d \
./drivers/src/stm32429zi_gpio.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/src/%.o drivers/src/%.su: ../drivers/src/%.c drivers/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F429ZITx -DSTM32F4 -DNUCLEO_F429ZI -c -I../Inc -I"C:/Users/hcg-c/Downloads/EMB/My_workspace/target/stm32429zi_driverstry2/drivers/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-drivers-2f-src

clean-drivers-2f-src:
	-$(RM) ./drivers/src/stm32429zi_I2C.d ./drivers/src/stm32429zi_I2C.o ./drivers/src/stm32429zi_I2C.su ./drivers/src/stm32429zi_gpio.d ./drivers/src/stm32429zi_gpio.o ./drivers/src/stm32429zi_gpio.su

.PHONY: clean-drivers-2f-src

