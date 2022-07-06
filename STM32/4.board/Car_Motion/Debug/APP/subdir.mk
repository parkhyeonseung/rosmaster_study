################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../APP/app.c \
../APP/app_motion.c \
../APP/app_pid.c 

OBJS += \
./APP/app.o \
./APP/app_motion.o \
./APP/app_pid.o 

C_DEPS += \
./APP/app.d \
./APP/app_motion.d \
./APP/app_pid.d 


# Each subdirectory must supply rules for building sources it contributes
APP/%.o APP/%.su: ../APP/%.c APP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../BSP -I../APP -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-APP

clean-APP:
	-$(RM) ./APP/app.d ./APP/app.o ./APP/app.su ./APP/app_motion.d ./APP/app_motion.o ./APP/app_motion.su ./APP/app_pid.d ./APP/app_pid.o ./APP/app_pid.su

.PHONY: clean-APP

