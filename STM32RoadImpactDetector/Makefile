# Command-line build of the STM32RoadImpactDetector firmware (STM32F756ZG, Cortex-M7).
#
# Uses the same settings as the STM32CubeIDE "Debug" configuration (.cproject).
# STM32CubeIDE does not use this file: it keeps building with its own makefiles
# in Debug/, so the project opens and builds in the IDE exactly as before.
#
# Requirements: GNU Arm Embedded Toolchain (arm-none-eabi-gcc) and make.
# Usage:
#   make          - build build/STM32RoadImpactDetector.elf, .hex and .bin
#   make clean    - remove the build directory

TARGET    := STM32RoadImpactDetector
BUILD_DIR := build

PREFIX  := arm-none-eabi-
CC      := $(PREFIX)gcc
OBJCOPY := $(PREFIX)objcopy
SIZE    := $(PREFIX)size

# ---- MCU (from .cproject) ----
MCU := -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard

DEFS := -DDEBUG -DUSE_HAL_DRIVER -DSTM32F756xx

INC_DIRS := \
	Core/Inc \
	Core/Application/Inc \
	Drivers/STM32F7xx_HAL_Driver/Inc \
	Drivers/STM32F7xx_HAL_Driver/Inc/Legacy \
	Middlewares/Third_Party/FreeRTOS/Source/include \
	Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS \
	Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 \
	Drivers/CMSIS/Device/ST/STM32F7xx/Include \
	Drivers/CMSIS/Include
INCS := $(addprefix -I,$(INC_DIRS))

# ---- Sources ----
# Like STM32CubeIDE: every .c / .s file inside the project's source folders.
SRC_DIRS := Core Drivers Middlewares
C_SRCS   := $(shell find $(SRC_DIRS) -name '*.c')
ASM_SRCS := $(shell find $(SRC_DIRS) -name '*.s')

LDSCRIPT := STM32F756ZGTX_FLASH.ld

# ---- Flags (STM32CubeIDE defaults for Debug) ----
CFLAGS := $(MCU) $(DEFS) $(INCS) \
	-std=gnu11 -g3 -O0 -Wall \
	-ffunction-sections -fdata-sections \
	--specs=nano.specs

ASFLAGS := $(MCU) $(DEFS) -g3 -x assembler-with-cpp --specs=nano.specs

# -u _printf_float: printf("%f") support, enabled in the CubeIDE project settings.
LDFLAGS := $(MCU) -T$(LDSCRIPT) \
	--specs=nosys.specs --specs=nano.specs -u _printf_float \
	-Wl,-Map=$(BUILD_DIR)/$(TARGET).map -Wl,--gc-sections -static \
	-Wl,--start-group -lc -lm -Wl,--end-group

# ---- Objects ----
OBJS := \
	$(addprefix $(BUILD_DIR)/,$(C_SRCS:.c=.o)) \
	$(addprefix $(BUILD_DIR)/,$(ASM_SRCS:.s=.o))

# ---- Rules ----
.PHONY: all clean

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin
	$(SIZE) $(BUILD_DIR)/$(TARGET).elf

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: %.s
	@mkdir -p $(dir $@)
	$(CC) $(ASFLAGS) -c $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJS) $(LDSCRIPT)
	$(CC) $(OBJS) $(LDFLAGS) -o $@

$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

clean:
	rm -rf $(BUILD_DIR)
