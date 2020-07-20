################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/fs_display.c \
../Core/Inc/stm32f7xx_hal_sd.c 

OBJS += \
./Core/Inc/fs_display.o \
./Core/Inc/stm32f7xx_hal_sd.o 

C_DEPS += \
./Core/Inc/fs_display.d \
./Core/Inc/stm32f7xx_hal_sd.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/fs_display.o: ../Core/Inc/fs_display.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32F777xx -DBNO055_API -DUSE_HAL_DRIVER -DDEBUG -c -I../FATFS/App -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Core/GPIO/Inc -I../FATFS/Target -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Core/Peripherals/Inc -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Inc/fs_display.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Inc/stm32f7xx_hal_sd.o: ../Core/Inc/stm32f7xx_hal_sd.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32F777xx -DBNO055_API -DUSE_HAL_DRIVER -DDEBUG -c -I../FATFS/App -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Core/GPIO/Inc -I../FATFS/Target -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Core/Peripherals/Inc -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Inc/stm32f7xx_hal_sd.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

