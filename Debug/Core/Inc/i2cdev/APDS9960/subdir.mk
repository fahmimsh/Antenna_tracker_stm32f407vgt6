################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/i2cdev/APDS9960/APDS9960.c 

OBJS += \
./Core/Inc/i2cdev/APDS9960/APDS9960.o 

C_DEPS += \
./Core/Inc/i2cdev/APDS9960/APDS9960.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/i2cdev/APDS9960/APDS9960.o: ../Core/Inc/i2cdev/APDS9960/APDS9960.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Inc/i2cdev/APDS9960/APDS9960.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

