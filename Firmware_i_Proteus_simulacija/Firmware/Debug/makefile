################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
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
Brojac_saobracaja.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Brojac_saobracaja.list \

OBJCOPY_HEX += \
Brojac_saobracaja.hex \

OBJCOPY_BIN += \
Brojac_saobracaja.bin \


# All Target
all: Brojac_saobracaja.elf secondary-outputs

# Tool invocations
Brojac_saobracaja.elf: $(OBJS) $(USER_OBJS) C:\Users\Lazar\Desktop\MPS_Zavrsni\ projekat\Firmware_i_Proteus_simulacija\Firmware\STM32F103C6TX_FLASH.ld
	arm-none-eabi-gcc -o "Brojac_saobracaja.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\Lazar\Desktop\MPS_Zavrsni projekat\Firmware_i_Proteus_simulacija\Firmware\STM32F103C6TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Brojac_saobracaja.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Brojac_saobracaja.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Brojac_saobracaja.list"
	@echo 'Finished building: $@'
	@echo ' '

Brojac_saobracaja.hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "Brojac_saobracaja.hex"
	@echo 'Finished building: $@'
	@echo ' '

Brojac_saobracaja.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Brojac_saobracaja.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
