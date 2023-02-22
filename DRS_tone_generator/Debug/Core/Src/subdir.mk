################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/i2c1_master.c \
../Core/Src/led.c \
../Core/Src/m24c64.c \
../Core/Src/main.c \
../Core/Src/max2871.c \
../Core/Src/module.c \
../Core/Src/rs485.c \
../Core/Src/stm32g0xx_hal_msp.c \
../Core/Src/stm32g0xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g0xx.c \
../Core/Src/uart1.c 

OBJS += \
./Core/Src/i2c1_master.o \
./Core/Src/led.o \
./Core/Src/m24c64.o \
./Core/Src/main.o \
./Core/Src/max2871.o \
./Core/Src/module.o \
./Core/Src/rs485.o \
./Core/Src/stm32g0xx_hal_msp.o \
./Core/Src/stm32g0xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g0xx.o \
./Core/Src/uart1.o 

C_DEPS += \
./Core/Src/i2c1_master.d \
./Core/Src/led.d \
./Core/Src/m24c64.d \
./Core/Src/main.d \
./Core/Src/max2871.d \
./Core/Src/module.d \
./Core/Src/rs485.d \
./Core/Src/stm32g0xx_hal_msp.d \
./Core/Src/stm32g0xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g0xx.d \
./Core/Src/uart1.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G030xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/i2c1_master.d ./Core/Src/i2c1_master.o ./Core/Src/i2c1_master.su ./Core/Src/led.d ./Core/Src/led.o ./Core/Src/led.su ./Core/Src/m24c64.d ./Core/Src/m24c64.o ./Core/Src/m24c64.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/max2871.d ./Core/Src/max2871.o ./Core/Src/max2871.su ./Core/Src/module.d ./Core/Src/module.o ./Core/Src/module.su ./Core/Src/rs485.d ./Core/Src/rs485.o ./Core/Src/rs485.su ./Core/Src/stm32g0xx_hal_msp.d ./Core/Src/stm32g0xx_hal_msp.o ./Core/Src/stm32g0xx_hal_msp.su ./Core/Src/stm32g0xx_it.d ./Core/Src/stm32g0xx_it.o ./Core/Src/stm32g0xx_it.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g0xx.d ./Core/Src/system_stm32g0xx.o ./Core/Src/system_stm32g0xx.su ./Core/Src/uart1.d ./Core/Src/uart1.o ./Core/Src/uart1.su

.PHONY: clean-Core-2f-Src

