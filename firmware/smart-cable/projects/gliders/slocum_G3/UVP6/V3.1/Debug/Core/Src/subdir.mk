################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/UVP6.c \
../Core/Src/base64.c \
../Core/Src/disp_proc.c \
../Core/Src/extra_calc.c \
../Core/Src/freertos.c \
../Core/Src/fsm.c \
../Core/Src/main.c \
../Core/Src/mbcrc.c \
../Core/Src/mcu_flash.c \
../Core/Src/slocum_g3.c \
../Core/Src/stm32f1xx_hal_msp.c \
../Core/Src/stm32f1xx_hal_timebase_tim.c \
../Core/Src/stm32f1xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f1xx.c 

OBJS += \
./Core/Src/UVP6.o \
./Core/Src/base64.o \
./Core/Src/disp_proc.o \
./Core/Src/extra_calc.o \
./Core/Src/freertos.o \
./Core/Src/fsm.o \
./Core/Src/main.o \
./Core/Src/mbcrc.o \
./Core/Src/mcu_flash.o \
./Core/Src/slocum_g3.o \
./Core/Src/stm32f1xx_hal_msp.o \
./Core/Src/stm32f1xx_hal_timebase_tim.o \
./Core/Src/stm32f1xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f1xx.o 

C_DEPS += \
./Core/Src/UVP6.d \
./Core/Src/base64.d \
./Core/Src/disp_proc.d \
./Core/Src/extra_calc.d \
./Core/Src/freertos.d \
./Core/Src/fsm.d \
./Core/Src/main.d \
./Core/Src/mbcrc.d \
./Core/Src/mcu_flash.d \
./Core/Src/slocum_g3.d \
./Core/Src/stm32f1xx_hal_msp.d \
./Core/Src/stm32f1xx_hal_timebase_tim.d \
./Core/Src/stm32f1xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f1xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F107xC -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/UVP6.cyclo ./Core/Src/UVP6.d ./Core/Src/UVP6.o ./Core/Src/UVP6.su ./Core/Src/base64.cyclo ./Core/Src/base64.d ./Core/Src/base64.o ./Core/Src/base64.su ./Core/Src/disp_proc.cyclo ./Core/Src/disp_proc.d ./Core/Src/disp_proc.o ./Core/Src/disp_proc.su ./Core/Src/extra_calc.cyclo ./Core/Src/extra_calc.d ./Core/Src/extra_calc.o ./Core/Src/extra_calc.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/fsm.cyclo ./Core/Src/fsm.d ./Core/Src/fsm.o ./Core/Src/fsm.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/mbcrc.cyclo ./Core/Src/mbcrc.d ./Core/Src/mbcrc.o ./Core/Src/mbcrc.su ./Core/Src/mcu_flash.cyclo ./Core/Src/mcu_flash.d ./Core/Src/mcu_flash.o ./Core/Src/mcu_flash.su ./Core/Src/slocum_g3.cyclo ./Core/Src/slocum_g3.d ./Core/Src/slocum_g3.o ./Core/Src/slocum_g3.su ./Core/Src/stm32f1xx_hal_msp.cyclo ./Core/Src/stm32f1xx_hal_msp.d ./Core/Src/stm32f1xx_hal_msp.o ./Core/Src/stm32f1xx_hal_msp.su ./Core/Src/stm32f1xx_hal_timebase_tim.cyclo ./Core/Src/stm32f1xx_hal_timebase_tim.d ./Core/Src/stm32f1xx_hal_timebase_tim.o ./Core/Src/stm32f1xx_hal_timebase_tim.su ./Core/Src/stm32f1xx_it.cyclo ./Core/Src/stm32f1xx_it.d ./Core/Src/stm32f1xx_it.o ./Core/Src/stm32f1xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f1xx.cyclo ./Core/Src/system_stm32f1xx.d ./Core/Src/system_stm32f1xx.o ./Core/Src/system_stm32f1xx.su

.PHONY: clean-Core-2f-Src

