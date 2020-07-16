################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Peripherals/Src/FlashQSPIAgent.c \
../Core/Peripherals/Src/LED.c \
../Core/Peripherals/Src/bno055.c \
../Core/Peripherals/Src/bno055_support.c \
../Core/Peripherals/Src/ms5611.c \
../Core/Peripherals/Src/terminal.c 

OBJS += \
./Core/Peripherals/Src/FlashQSPIAgent.o \
./Core/Peripherals/Src/LED.o \
./Core/Peripherals/Src/bno055.o \
./Core/Peripherals/Src/bno055_support.o \
./Core/Peripherals/Src/ms5611.o \
./Core/Peripherals/Src/terminal.o 

C_DEPS += \
./Core/Peripherals/Src/FlashQSPIAgent.d \
./Core/Peripherals/Src/LED.d \
./Core/Peripherals/Src/bno055.d \
./Core/Peripherals/Src/bno055_support.d \
./Core/Peripherals/Src/ms5611.d \
./Core/Peripherals/Src/terminal.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Peripherals/Src/FlashQSPIAgent.o: ../Core/Peripherals/Src/FlashQSPIAgent.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32F777xx -DBNO055_API -DUSE_HAL_DRIVER -DDEBUG -c -I../FATFS/App -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Core/GPIO/Inc -I../FATFS/Target -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Core/Peripherals/Inc -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Peripherals/Src/FlashQSPIAgent.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Peripherals/Src/LED.o: ../Core/Peripherals/Src/LED.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32F777xx -DBNO055_API -DUSE_HAL_DRIVER -DDEBUG -c -I../FATFS/App -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Core/GPIO/Inc -I../FATFS/Target -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Core/Peripherals/Inc -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Peripherals/Src/LED.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Peripherals/Src/bno055.o: ../Core/Peripherals/Src/bno055.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32F777xx -DBNO055_API -DUSE_HAL_DRIVER -DDEBUG -c -I../FATFS/App -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Core/GPIO/Inc -I../FATFS/Target -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Core/Peripherals/Inc -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Peripherals/Src/bno055.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Peripherals/Src/bno055_support.o: ../Core/Peripherals/Src/bno055_support.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32F777xx -DBNO055_API -DUSE_HAL_DRIVER -DDEBUG -c -I../FATFS/App -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Core/GPIO/Inc -I../FATFS/Target -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Core/Peripherals/Inc -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Peripherals/Src/bno055_support.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Peripherals/Src/ms5611.o: ../Core/Peripherals/Src/ms5611.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32F777xx -DBNO055_API -DUSE_HAL_DRIVER -DDEBUG -c -I../FATFS/App -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Core/GPIO/Inc -I../FATFS/Target -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Core/Peripherals/Inc -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Peripherals/Src/ms5611.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Peripherals/Src/terminal.o: ../Core/Peripherals/Src/terminal.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32F777xx -DBNO055_API -DUSE_HAL_DRIVER -DDEBUG -c -I../FATFS/App -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Core/GPIO/Inc -I../FATFS/Target -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Core/Peripherals/Inc -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Peripherals/Src/terminal.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

