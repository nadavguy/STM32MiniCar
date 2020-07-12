################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/GPIO/Src/PWMControl.c \
../Core/GPIO/Src/PushButton.c 

OBJS += \
./Core/GPIO/Src/PWMControl.o \
./Core/GPIO/Src/PushButton.o 

C_DEPS += \
./Core/GPIO/Src/PWMControl.d \
./Core/GPIO/Src/PushButton.d 


# Each subdirectory must supply rules for building sources it contributes
Core/GPIO/Src/PWMControl.o: ../Core/GPIO/Src/PWMControl.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32F777xx -DBNO055_API -DUSE_HAL_DRIVER -DDEBUG -c -I../FATFS/App -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Core/GPIO/Inc -I../FATFS/Target -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Core/Peripherals/Inc -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/GPIO/Src/PWMControl.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/GPIO/Src/PushButton.o: ../Core/GPIO/Src/PushButton.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32F777xx -DBNO055_API -DUSE_HAL_DRIVER -DDEBUG -c -I../FATFS/App -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Core/GPIO/Inc -I../FATFS/Target -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Core/Peripherals/Inc -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/GPIO/Src/PushButton.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

