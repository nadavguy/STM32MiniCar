################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/GPIO/Src/PWMControl.c 

OBJS += \
./Core/GPIO/Src/PWMControl.o 

C_DEPS += \
./Core/GPIO/Src/PWMControl.d 


# Each subdirectory must supply rules for building sources it contributes
Core/GPIO/Src/PWMControl.o: ../Core/GPIO/Src/PWMControl.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32F777xx -DBNO055_API -DUSE_HAL_DRIVER -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Core/GPIO/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Core/Peripherals/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/GPIO/Src/PWMControl.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

