################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/BSP/bsp.c \
../Core/BSP/bsp_encoder.c \
../Core/BSP/bsp_motor.c 

OBJS += \
./Core/BSP/bsp.o \
./Core/BSP/bsp_encoder.o \
./Core/BSP/bsp_motor.o 

C_DEPS += \
./Core/BSP/bsp.d \
./Core/BSP/bsp_encoder.d \
./Core/BSP/bsp_motor.d 


# Each subdirectory must supply rules for building sources it contributes
Core/BSP/%.o: ../Core/BSP/%.c Core/BSP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Core/BSP -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-BSP

clean-Core-2f-BSP:
	-$(RM) ./Core/BSP/bsp.d ./Core/BSP/bsp.o ./Core/BSP/bsp_encoder.d ./Core/BSP/bsp_encoder.o ./Core/BSP/bsp_motor.d ./Core/BSP/bsp_motor.o

.PHONY: clean-Core-2f-BSP

