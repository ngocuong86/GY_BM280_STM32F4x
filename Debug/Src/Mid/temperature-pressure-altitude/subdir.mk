################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/Mid/temperature-pressure-altitude/BME280-Sensor.c 

OBJS += \
./Src/Mid/temperature-pressure-altitude/BME280-Sensor.o 

C_DEPS += \
./Src/Mid/temperature-pressure-altitude/BME280-Sensor.d 


# Each subdirectory must supply rules for building sources it contributes
Src/Mid/temperature-pressure-altitude/BME280-Sensor.o: ../Src/Mid/temperature-pressure-altitude/BME280-Sensor.c Src/Mid/temperature-pressure-altitude/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F401RETx -DSTM32F4 -c -I../Inc -I"D:/IOT/SDK_1.0.3_NUCLEO-F401RE/shared/Drivers/CMSIS/Include" -I"D:/IOT/SDK_1.0.3_NUCLEO-F401RE/shared/Drivers/STM32F401RE_StdPeriph_Driver/inc" -I"D:/IOT/SDK_1.0.3_NUCLEO-F401RE/shared/Middle/ucglib" -I"D:/IOT/SDK_1.0.3_NUCLEO-F401RE/lib_stm" -I"D:/IOT/SDK_1.0.3_NUCLEO-F401RE/shared/Middle/rtos" -I"E:/TTS/GY-BMP280/Startup" -I"E:/TTS/GY-BMP280/Src/utils" -I"D:/IOT/SDK_1.0.3_NUCLEO-F401RE/shared/Middle/ucglib" -I"D:/IOT/SDK_1.0.3_NUCLEO-F401RE/shared/Middle/button" -I"D:/IOT/SDK_1.0.3_NUCLEO-F401RE/shared/Drivers/CMSIS/Include" -I"D:/IOT/SDK_1.0.3_NUCLEO-F401RE/shared/Drivers/STM32F401RE_StdPeriph_Driver/inc" -I"D:/IOT/SDK_1.0.3_NUCLEO-F401RE/shared/Utilities" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/Mid/temperature-pressure-altitude/BME280-Sensor.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

