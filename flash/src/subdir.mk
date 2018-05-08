################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/I2C_MasterSlave.c \
../src/MCUXpresso_Retarget.c \
../src/MCUXpresso_cr_startup.c \
../src/MCUXpresso_crp.c \
../src/MCUXpresso_mtb.c \
../src/Multi_Timer_Blinky.c \
../src/Multi_Timer_Blinky_ISR.c \
../src/Serial.c \
../src/UART0_Terminal.c \
../src/system.c 

OBJS += \
./src/I2C_MasterSlave.o \
./src/MCUXpresso_Retarget.o \
./src/MCUXpresso_cr_startup.o \
./src/MCUXpresso_crp.o \
./src/MCUXpresso_mtb.o \
./src/Multi_Timer_Blinky.o \
./src/Multi_Timer_Blinky_ISR.o \
./src/Serial.o \
./src/UART0_Terminal.o \
./src/system.o 

C_DEPS += \
./src/I2C_MasterSlave.d \
./src/MCUXpresso_Retarget.d \
./src/MCUXpresso_cr_startup.d \
./src/MCUXpresso_crp.d \
./src/MCUXpresso_mtb.d \
./src/Multi_Timer_Blinky.d \
./src/Multi_Timer_Blinky_ISR.d \
./src/Serial.d \
./src/UART0_Terminal.d \
./src/system.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DDEBUG -D__USE_CMSIS -D__CODE_RED -DCORE_M0PLUS -D__MTB_DISABLE -D__MTB_BUFFER_SIZE=256 -D__LPC84X__ -D__REDLIB__ -I"D:\Software\System\IDE\LPC845_MCUXpresso_Qvoice\LPC845_MCUXpresso_Qvoice\Multi_LED_Driver_Askey\inc" -I"D:\Software\System\IDE\LPC845_MCUXpresso_Qvoice\LPC845_MCUXpresso_Qvoice\peripherals_lib\inc" -I"D:\Software\System\IDE\LPC845_MCUXpresso_Qvoice\LPC845_MCUXpresso_Qvoice\utilities_lib\inc" -I"D:\Software\System\IDE\LPC845_MCUXpresso_Qvoice\LPC845_MCUXpresso_Qvoice\common\inc" -O0 -fno-common -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m0 -mthumb -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


