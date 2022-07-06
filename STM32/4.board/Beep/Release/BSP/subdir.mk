################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BSP/bsp.c \
../BSP/bsp_beep.c \
../BSP/bsp_key.c 

OBJS += \
./BSP/bsp.o \
./BSP/bsp_beep.o \
./BSP/bsp_key.o 

C_DEPS += \
./BSP/bsp.d \
./BSP/bsp_beep.d \
./BSP/bsp_key.d 


# Each subdirectory must supply rules for building sources it contributes
BSP/%.o BSP/%.su: ../BSP/%.c BSP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-BSP

clean-BSP:
	-$(RM) ./BSP/bsp.d ./BSP/bsp.o ./BSP/bsp.su ./BSP/bsp_beep.d ./BSP/bsp_beep.o ./BSP/bsp_beep.su ./BSP/bsp_key.d ./BSP/bsp_key.o ./BSP/bsp_key.su

.PHONY: clean-BSP

