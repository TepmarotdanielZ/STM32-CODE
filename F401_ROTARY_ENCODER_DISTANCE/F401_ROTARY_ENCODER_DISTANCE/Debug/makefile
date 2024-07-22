################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := F401_ROTARY_ENCODER_DISTANCE
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
F401_ROTARY_ENCODER_DISTANCE.elf \

MAP_FILES += \
F401_ROTARY_ENCODER_DISTANCE.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
F401_ROTARY_ENCODER_DISTANCE.list \


# All Target
all: main-build

# Main-build Target
main-build: F401_ROTARY_ENCODER_DISTANCE.elf secondary-outputs

# Tool invocations
F401_ROTARY_ENCODER_DISTANCE.elf F401_ROTARY_ENCODER_DISTANCE.map: $(OBJS) $(USER_OBJS) /home/tepnarotdabiel/STM32CubeIDE/STM32_WS_V3/F401_ROTARY_ENCODER_DISTANCE/STM32F401CCUX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "F401_ROTARY_ENCODER_DISTANCE.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/home/tepnarotdabiel/STM32CubeIDE/STM32_WS_V3/F401_ROTARY_ENCODER_DISTANCE/STM32F401CCUX_FLASH.ld" --specs=nosys.specs -Wl,-Map="F401_ROTARY_ENCODER_DISTANCE.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

F401_ROTARY_ENCODER_DISTANCE.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "F401_ROTARY_ENCODER_DISTANCE.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) F401_ROTARY_ENCODER_DISTANCE.elf F401_ROTARY_ENCODER_DISTANCE.list F401_ROTARY_ENCODER_DISTANCE.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
