################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
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


BUILD_ARTIFACT_NAME := Logic und Timer und Outputs
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Logic\ und\ Timer\ und\ Outputs.elf \

MAP_FILES += \
Logic\ und\ Timer\ und\ Outputs.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Logic\ und\ Timer\ und\ Outputs.list \


# All Target
all: main-build

# Main-build Target
main-build: Logic\ und\ Timer\ und\ Outputs.elf secondary-outputs

dependents:
	-cd /Users/philipprohlke/Documents/STMCubeIDE/GitHub/Ohne\ Titel/EDS_2024_fork/CMSIS/Debug && $(MAKE) all
	-cd /Users/philipprohlke/Documents/STMCubeIDE/GitHub/Ohne\ Titel/EDS_2024_fork/MCAL/Debug && $(MAKE) all

# Tool invocations
Logic\ und\ Timer\ und\ Outputs.elf Logic\ und\ Timer\ und\ Outputs.map: $(OBJS) $(USER_OBJS) /Users/philipprohlke/Documents/STMCubeIDE/GitHub/Ohne\ Titel/EDS_2024_fork/CMSIS/Debug/libCMSIS.a /Users/philipprohlke/Documents/STMCubeIDE/GitHub/Ohne\ Titel/EDS_2024_fork/MCAL/Debug/libMCAL.a /Users/philipprohlke/Documents/STMCubeIDE/GitHub/Ohne\ Titel/EDS_2024_fork/Logic\ und\ Timer\ und\ Outputs/STM32F401RETX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "Logic und Timer und Outputs.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/Users/philipprohlke/Documents/STMCubeIDE/GitHub/Ohne Titel/EDS_2024_fork/Logic und Timer und Outputs/STM32F401RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Logic und Timer und Outputs.map" -Wl,--gc-sections -static -L"/Users/philipprohlke/Documents/STMCubeIDE/GitHub/Ohne Titel/EDS_2024_fork/MCAL/Debug" -L"/Users/philipprohlke/Documents/STMCubeIDE/GitHub/Ohne Titel/EDS_2024_fork/CMSIS/Debug" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Logic\ und\ Timer\ und\ Outputs.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Logic und Timer und Outputs.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) Logic\ und\ Timer\ und\ Outputs.elf Logic\ und\ Timer\ und\ Outputs.list Logic\ und\ Timer\ und\ Outputs.map Logic\\ und\\ Timer\\ und\\ Outputs.elf default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified
/Users/philipprohlke/Documents/STMCubeIDE/GitHub/Ohne\ Titel/EDS_2024_fork/CMSIS/Debug/libCMSIS.a:
/Users/philipprohlke/Documents/STMCubeIDE/GitHub/Ohne\ Titel/EDS_2024_fork/MCAL/Debug/libMCAL.a:

-include ../makefile.targets
