################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32f103retx.s 

OBJS += \
./Core/Startup/startup_stm32f103retx.o 

S_DEPS += \
./Core/Startup/startup_stm32f103retx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m3 -g3 -DDEBUG -c -I"D:/GD/upwork/projects/Ehsan_Abdi/USIB/HW/Smart-Cable/firmware/smart-cable/sensors/HydroC_CH4/F103RE_25MHz/HydroC_CH4_V2/FATFS/App" -I"D:/GD/upwork/projects/Ehsan_Abdi/USIB/HW/Smart-Cable/firmware/smart-cable/sensors/HydroC_CH4/F103RE_25MHz/HydroC_CH4_V2/Middlewares/Third_Party/FatFs/src" -I"D:/GD/upwork/projects/Ehsan_Abdi/USIB/HW/Smart-Cable/firmware/smart-cable/sensors/HydroC_CH4/F103RE_25MHz/HydroC_CH4_V2/FATFS/Target" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32f103retx.d ./Core/Startup/startup_stm32f103retx.o

.PHONY: clean-Core-2f-Startup

