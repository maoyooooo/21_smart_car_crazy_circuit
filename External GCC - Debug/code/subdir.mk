################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../code/Motor_encoder.c \
../code/flash.c \
../code/image_process.c \
../code/kalman.c \
../code/motor.c \
../code/navigation.c 

C_DEPS += \
./code/Motor_encoder.d \
./code/flash.d \
./code/image_process.d \
./code/kalman.d \
./code/motor.d \
./code/navigation.d 

OBJS += \
./code/Motor_encoder.o \
./code/flash.o \
./code/image_process.o \
./code/kalman.o \
./code/motor.o \
./code/navigation.o 


# Each subdirectory must supply rules for building sources it contributes
code/%.o: ../code/%.c code/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: AURIX GCC Compiler'
	tricore-gcc -std=c99 "@C:/Users/McGRADY/AURIX-v1.10.2-workspace/21_smart_car_crazy_circuit/External GCC - Debug/AURIX_GCC_Compiler-Include_paths__-I_.opt" -O0 -g3 -Wall -c -fmessage-length=0 -fno-common -fstrict-volatile-bitfields -fdata-sections -ffunction-sections -mtc161 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-code

clean-code:
	-$(RM) ./code/Motor_encoder.d ./code/Motor_encoder.o ./code/flash.d ./code/flash.o ./code/image_process.d ./code/image_process.o ./code/kalman.d ./code/kalman.o ./code/motor.d ./code/motor.o ./code/navigation.d ./code/navigation.o

.PHONY: clean-code

