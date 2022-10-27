################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
ASM_SRCS += \
../source/DSP2834x_CodeStartBranch.asm \
../source/DSP2834x_usDelay.asm 

C_SRCS += \
../source/DSP2834x_CpuTimers.c \
../source/DSP2834x_EPwm.c \
../source/DSP2834x_EQep.c \
../source/DSP2834x_GlobalVariableDefs.c \
../source/DSP2834x_Gpio.c \
../source/DSP2834x_PieCtrl.c \
../source/DSP2834x_SWPrioritizedDefaultIsr.c \
../source/DSP2834x_SWPrioritizedPieVect.c \
../source/DSP2834x_SysCtrl.c \
../source/DSP2834x_Xintf.c \
../source/FsampInterrupt.c \
../source/FscInterrupt.c \
../source/adc.c \
../source/da.c \
../source/easy28x_gen2_bitfield_v9.4.c \
../source/fault.c \
../source/filter.c \
../source/main.c \
../source/variable.c 

C_DEPS += \
./source/DSP2834x_CpuTimers.d \
./source/DSP2834x_EPwm.d \
./source/DSP2834x_EQep.d \
./source/DSP2834x_GlobalVariableDefs.d \
./source/DSP2834x_Gpio.d \
./source/DSP2834x_PieCtrl.d \
./source/DSP2834x_SWPrioritizedDefaultIsr.d \
./source/DSP2834x_SWPrioritizedPieVect.d \
./source/DSP2834x_SysCtrl.d \
./source/DSP2834x_Xintf.d \
./source/FsampInterrupt.d \
./source/FscInterrupt.d \
./source/adc.d \
./source/da.d \
./source/easy28x_gen2_bitfield_v9.4.d \
./source/fault.d \
./source/filter.d \
./source/main.d \
./source/variable.d 

OBJS += \
./source/DSP2834x_CodeStartBranch.obj \
./source/DSP2834x_CpuTimers.obj \
./source/DSP2834x_EPwm.obj \
./source/DSP2834x_EQep.obj \
./source/DSP2834x_GlobalVariableDefs.obj \
./source/DSP2834x_Gpio.obj \
./source/DSP2834x_PieCtrl.obj \
./source/DSP2834x_SWPrioritizedDefaultIsr.obj \
./source/DSP2834x_SWPrioritizedPieVect.obj \
./source/DSP2834x_SysCtrl.obj \
./source/DSP2834x_Xintf.obj \
./source/DSP2834x_usDelay.obj \
./source/FsampInterrupt.obj \
./source/FscInterrupt.obj \
./source/adc.obj \
./source/da.obj \
./source/easy28x_gen2_bitfield_v9.4.obj \
./source/fault.obj \
./source/filter.obj \
./source/main.obj \
./source/variable.obj 

ASM_DEPS += \
./source/DSP2834x_CodeStartBranch.d \
./source/DSP2834x_usDelay.d 

OBJS__QUOTED += \
"source\DSP2834x_CodeStartBranch.obj" \
"source\DSP2834x_CpuTimers.obj" \
"source\DSP2834x_EPwm.obj" \
"source\DSP2834x_EQep.obj" \
"source\DSP2834x_GlobalVariableDefs.obj" \
"source\DSP2834x_Gpio.obj" \
"source\DSP2834x_PieCtrl.obj" \
"source\DSP2834x_SWPrioritizedDefaultIsr.obj" \
"source\DSP2834x_SWPrioritizedPieVect.obj" \
"source\DSP2834x_SysCtrl.obj" \
"source\DSP2834x_Xintf.obj" \
"source\DSP2834x_usDelay.obj" \
"source\FsampInterrupt.obj" \
"source\FscInterrupt.obj" \
"source\adc.obj" \
"source\da.obj" \
"source\easy28x_gen2_bitfield_v9.4.obj" \
"source\fault.obj" \
"source\filter.obj" \
"source\main.obj" \
"source\variable.obj" 

C_DEPS__QUOTED += \
"source\DSP2834x_CpuTimers.d" \
"source\DSP2834x_EPwm.d" \
"source\DSP2834x_EQep.d" \
"source\DSP2834x_GlobalVariableDefs.d" \
"source\DSP2834x_Gpio.d" \
"source\DSP2834x_PieCtrl.d" \
"source\DSP2834x_SWPrioritizedDefaultIsr.d" \
"source\DSP2834x_SWPrioritizedPieVect.d" \
"source\DSP2834x_SysCtrl.d" \
"source\DSP2834x_Xintf.d" \
"source\FsampInterrupt.d" \
"source\FscInterrupt.d" \
"source\adc.d" \
"source\da.d" \
"source\easy28x_gen2_bitfield_v9.4.d" \
"source\fault.d" \
"source\filter.d" \
"source\main.d" \
"source\variable.d" 

ASM_DEPS__QUOTED += \
"source\DSP2834x_CodeStartBranch.d" \
"source\DSP2834x_usDelay.d" 

ASM_SRCS__QUOTED += \
"../source/DSP2834x_CodeStartBranch.asm" \
"../source/DSP2834x_usDelay.asm" 

C_SRCS__QUOTED += \
"../source/DSP2834x_CpuTimers.c" \
"../source/DSP2834x_EPwm.c" \
"../source/DSP2834x_EQep.c" \
"../source/DSP2834x_GlobalVariableDefs.c" \
"../source/DSP2834x_Gpio.c" \
"../source/DSP2834x_PieCtrl.c" \
"../source/DSP2834x_SWPrioritizedDefaultIsr.c" \
"../source/DSP2834x_SWPrioritizedPieVect.c" \
"../source/DSP2834x_SysCtrl.c" \
"../source/DSP2834x_Xintf.c" \
"../source/FsampInterrupt.c" \
"../source/FscInterrupt.c" \
"../source/adc.c" \
"../source/da.c" \
"../source/easy28x_gen2_bitfield_v9.4.c" \
"../source/fault.c" \
"../source/filter.c" \
"../source/main.c" \
"../source/variable.c" 


