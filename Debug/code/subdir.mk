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

COMPILED_SRCS += \
code/Motor_encoder.src \
code/flash.src \
code/image_process.src \
code/kalman.src \
code/motor.src \
code/navigation.src 

C_DEPS += \
code/Motor_encoder.d \
code/flash.d \
code/image_process.d \
code/kalman.d \
code/motor.d \
code/navigation.d 

OBJS += \
code/Motor_encoder.o \
code/flash.o \
code/image_process.o \
code/kalman.o \
code/motor.o \
code/navigation.o 


# Each subdirectory must supply rules for building sources it contributes
code/Motor_encoder.src: ../code/Motor_encoder.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fC:/Users/McGRADY/AURIX-v1.10.2-workspace/21_smart_car_crazy_circuit/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/Motor_encoder.o: code/Motor_encoder.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/flash.src: ../code/flash.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fC:/Users/McGRADY/AURIX-v1.10.2-workspace/21_smart_car_crazy_circuit/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/flash.o: code/flash.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/image_process.src: ../code/image_process.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fC:/Users/McGRADY/AURIX-v1.10.2-workspace/21_smart_car_crazy_circuit/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/image_process.o: code/image_process.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/kalman.src: ../code/kalman.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fC:/Users/McGRADY/AURIX-v1.10.2-workspace/21_smart_car_crazy_circuit/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/kalman.o: code/kalman.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/motor.src: ../code/motor.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fC:/Users/McGRADY/AURIX-v1.10.2-workspace/21_smart_car_crazy_circuit/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/motor.o: code/motor.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/navigation.src: ../code/navigation.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fC:/Users/McGRADY/AURIX-v1.10.2-workspace/21_smart_car_crazy_circuit/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/navigation.o: code/navigation.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-code

clean-code:
	-$(RM) code/Motor_encoder.d code/Motor_encoder.o code/Motor_encoder.src code/flash.d code/flash.o code/flash.src code/image_process.d code/image_process.o code/image_process.src code/kalman.d code/kalman.o code/kalman.src code/motor.d code/motor.o code/motor.src code/navigation.d code/navigation.o code/navigation.src

.PHONY: clean-code

