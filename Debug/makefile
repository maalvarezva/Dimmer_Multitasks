################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/subdir.mk
-include Drivers/STM32L4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Dimmer_Multitasks.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Dimmer_Multitasks.list \

OBJCOPY_BIN += \
Dimmer_Multitasks.bin \


# All Target
all: Dimmer_Multitasks.elf secondary-outputs

# Tool invocations
Dimmer_Multitasks.elf: $(OBJS) $(USER_OBJS) C:\Users\Asus-Pc\STM32CubeIDE\workspace_1.3.0\Dimmer_Multisks\STM32L476RGTX_FLASH.ld
	arm-none-eabi-gcc -o "Dimmer_Multitasks.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\Asus-Pc\STM32CubeIDE\workspace_1.3.0\Dimmer_Multisks\STM32L476RGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Dimmer_Multitasks.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Dimmer_Multitasks.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Dimmer_Multitasks.list"
	@echo 'Finished building: $@'
	@echo ' '

Dimmer_Multitasks.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Dimmer_Multitasks.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
