################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BSP/bsp_beep.c \
../BSP/bsp_key.c \
../BSP/bsp_task.c 

OBJS += \
./BSP/bsp_beep.o \
./BSP/bsp_key.o \
./BSP/bsp_task.o 

C_DEPS += \
./BSP/bsp_beep.d \
./BSP/bsp_key.d \
./BSP/bsp_task.d 


# Each subdirectory must supply rules for building sources it contributes
BSP/%.o: ../BSP/%.c BSP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../BSP -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-BSP

clean-BSP:
	-$(RM) ./BSP/bsp_beep.d ./BSP/bsp_beep.o ./BSP/bsp_key.d ./BSP/bsp_key.o ./BSP/bsp_task.d ./BSP/bsp_task.o

.PHONY: clean-BSP

