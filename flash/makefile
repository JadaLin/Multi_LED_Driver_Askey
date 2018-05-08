################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: Multi_LED_Driver_Askey.axf

# Tool invocations
Multi_LED_Driver_Askey.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -L"D:\Software\System\IDE\LPC845_MCUXpresso_Qvoice\LPC845_MCUXpresso_Qvoice\peripherals_lib\flash" -L"D:\Software\System\IDE\LPC845_MCUXpresso_Qvoice\LPC845_MCUXpresso_Qvoice\utilities_lib\flash" -Xlinker -Map="Multi_LED_Driver_Askey.map" -Xlinker --gc-sections -Xlinker -print-memory-usage -mcpu=cortex-m0 -mthumb -T "Multi_Timer_Blinky_flash.ld" -o "Multi_LED_Driver_Askey.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) Multi_LED_Driver_Askey.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "Multi_LED_Driver_Askey.axf"; # arm-none-eabi-objcopy -v -O binary "Multi_LED_Driver_Askey.axf" "Multi_LED_Driver_Askey.bin" ; # checksum -p LPC845 -d "Multi_LED_Driver_Askey.bin";
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
