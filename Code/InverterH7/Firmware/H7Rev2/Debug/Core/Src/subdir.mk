################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/adc.c \
../Core/Src/appMath.c \
../Core/Src/can.c \
../Core/Src/encoder.c \
../Core/Src/fault.c \
../Core/Src/hall.c \
../Core/Src/kalman.c \
../Core/Src/lut.c \
../Core/Src/main.c \
../Core/Src/mtpva.c \
../Core/Src/off_detect.c \
../Core/Src/off_param.c \
../Core/Src/speed.c \
../Core/Src/stm32h7xx_hal_msp.c \
../Core/Src/stm32h7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h7xx.c \
../Core/Src/warn.c 

OBJS += \
./Core/Src/adc.o \
./Core/Src/appMath.o \
./Core/Src/can.o \
./Core/Src/encoder.o \
./Core/Src/fault.o \
./Core/Src/hall.o \
./Core/Src/kalman.o \
./Core/Src/lut.o \
./Core/Src/main.o \
./Core/Src/mtpva.o \
./Core/Src/off_detect.o \
./Core/Src/off_param.o \
./Core/Src/speed.o \
./Core/Src/stm32h7xx_hal_msp.o \
./Core/Src/stm32h7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h7xx.o \
./Core/Src/warn.o 

C_DEPS += \
./Core/Src/adc.d \
./Core/Src/appMath.d \
./Core/Src/can.d \
./Core/Src/encoder.d \
./Core/Src/fault.d \
./Core/Src/hall.d \
./Core/Src/kalman.d \
./Core/Src/lut.d \
./Core/Src/main.d \
./Core/Src/mtpva.d \
./Core/Src/off_detect.d \
./Core/Src/off_param.d \
./Core/Src/speed.d \
./Core/Src/stm32h7xx_hal_msp.d \
./Core/Src/stm32h7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h7xx.d \
./Core/Src/warn.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H723xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/adc.cyclo ./Core/Src/adc.d ./Core/Src/adc.o ./Core/Src/adc.su ./Core/Src/appMath.cyclo ./Core/Src/appMath.d ./Core/Src/appMath.o ./Core/Src/appMath.su ./Core/Src/can.cyclo ./Core/Src/can.d ./Core/Src/can.o ./Core/Src/can.su ./Core/Src/encoder.cyclo ./Core/Src/encoder.d ./Core/Src/encoder.o ./Core/Src/encoder.su ./Core/Src/fault.cyclo ./Core/Src/fault.d ./Core/Src/fault.o ./Core/Src/fault.su ./Core/Src/hall.cyclo ./Core/Src/hall.d ./Core/Src/hall.o ./Core/Src/hall.su ./Core/Src/kalman.cyclo ./Core/Src/kalman.d ./Core/Src/kalman.o ./Core/Src/kalman.su ./Core/Src/lut.cyclo ./Core/Src/lut.d ./Core/Src/lut.o ./Core/Src/lut.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/mtpva.cyclo ./Core/Src/mtpva.d ./Core/Src/mtpva.o ./Core/Src/mtpva.su ./Core/Src/off_detect.cyclo ./Core/Src/off_detect.d ./Core/Src/off_detect.o ./Core/Src/off_detect.su ./Core/Src/off_param.cyclo ./Core/Src/off_param.d ./Core/Src/off_param.o ./Core/Src/off_param.su ./Core/Src/speed.cyclo ./Core/Src/speed.d ./Core/Src/speed.o ./Core/Src/speed.su ./Core/Src/stm32h7xx_hal_msp.cyclo ./Core/Src/stm32h7xx_hal_msp.d ./Core/Src/stm32h7xx_hal_msp.o ./Core/Src/stm32h7xx_hal_msp.su ./Core/Src/stm32h7xx_it.cyclo ./Core/Src/stm32h7xx_it.d ./Core/Src/stm32h7xx_it.o ./Core/Src/stm32h7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32h7xx.cyclo ./Core/Src/system_stm32h7xx.d ./Core/Src/system_stm32h7xx.o ./Core/Src/system_stm32h7xx.su ./Core/Src/warn.cyclo ./Core/Src/warn.d ./Core/Src/warn.o ./Core/Src/warn.su

.PHONY: clean-Core-2f-Src

