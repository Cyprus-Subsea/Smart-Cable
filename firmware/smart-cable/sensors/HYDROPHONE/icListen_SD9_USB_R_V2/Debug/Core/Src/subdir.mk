################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/crc.c \
../Core/Src/em_sd_storage.c \
../Core/Src/freertos.c \
../Core/Src/icListen.c \
../Core/Src/main.c \
../Core/Src/mcu_flash.c \
../Core/Src/sd.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_hal_timebase_tim.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/wav.c 

OBJS += \
./Core/Src/crc.o \
./Core/Src/em_sd_storage.o \
./Core/Src/freertos.o \
./Core/Src/icListen.o \
./Core/Src/main.o \
./Core/Src/mcu_flash.o \
./Core/Src/sd.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_hal_timebase_tim.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/wav.o 

C_DEPS += \
./Core/Src/crc.d \
./Core/Src/em_sd_storage.d \
./Core/Src/freertos.d \
./Core/Src/icListen.d \
./Core/Src/main.d \
./Core/Src/mcu_flash.d \
./Core/Src/sd.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_hal_timebase_tim.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/wav.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../USB_HOST/App -I../USB_HOST/Target -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"D:/GD/upwork/projects/Ehsan_Abdi/USIB/HW/Smart-Cable/firmware/smart-cable/sensors/HYDROPHONE/icListen_SD9_USB_R_V2/Middlewares/Third_Party/FatFs/src" -I"D:/GD/upwork/projects/Ehsan_Abdi/USIB/HW/Smart-Cable/firmware/smart-cable/sensors/HYDROPHONE/icListen_SD9_USB_R_V2/FATFS/App" -I"D:/GD/upwork/projects/Ehsan_Abdi/USIB/HW/Smart-Cable/firmware/smart-cable/sensors/HYDROPHONE/icListen_SD9_USB_R_V2/FATFS/Target" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/crc.d ./Core/Src/crc.o ./Core/Src/crc.su ./Core/Src/em_sd_storage.d ./Core/Src/em_sd_storage.o ./Core/Src/em_sd_storage.su ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/icListen.d ./Core/Src/icListen.o ./Core/Src/icListen.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/mcu_flash.d ./Core/Src/mcu_flash.o ./Core/Src/mcu_flash.su ./Core/Src/sd.d ./Core/Src/sd.o ./Core/Src/sd.su ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_hal_timebase_tim.d ./Core/Src/stm32f4xx_hal_timebase_tim.o ./Core/Src/stm32f4xx_hal_timebase_tim.su ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/wav.d ./Core/Src/wav.o ./Core/Src/wav.su

.PHONY: clean-Core-2f-Src

