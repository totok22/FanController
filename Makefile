##########################################################
# FanController Makefile
# Target: STM32F103C8Tx, HAL, arm-none-eabi-gcc
# Build output: Debug/
##########################################################

TARGET    := FanController
BUILD_DIR := Debug

# 本机常用工具链路径（存在则优先使用）
ARM_GNU_TOOLCHAIN_BIN := /Applications/ArmGNUToolchain/15.2.rel1/arm-none-eabi/bin

ifneq ("$(wildcard $(ARM_GNU_TOOLCHAIN_BIN)/arm-none-eabi-gcc)","")
PREFIX  := $(ARM_GNU_TOOLCHAIN_BIN)/arm-none-eabi-
else
PREFIX  := arm-none-eabi-
endif

CC      := $(PREFIX)gcc
AS      := $(PREFIX)gcc -x assembler-with-cpp
OBJCOPY := $(PREFIX)objcopy
SIZE    := $(PREFIX)size

CPU := -mcpu=cortex-m3
MCU := $(CPU) -mthumb

DEFS := \
  -DDEBUG \
  -DUSE_HAL_DRIVER \
  -DSTM32F103xB

INCLUDES := \
  -ICore/Inc \
  -IDrivers/STM32F1xx_HAL_Driver/Inc \
  -IDrivers/STM32F1xx_HAL_Driver/Inc/Legacy \
  -IDrivers/CMSIS/Device/ST/STM32F1xx/Include \
  -IDrivers/CMSIS/Include

ASM_SOURCES := Startup/startup_stm32f103c8tx.s
LDSCRIPT := STM32F103C8TX_FLASH.ld

# 递归收集 C 源文件
C_SOURCES := $(shell find Core/Src Drivers/STM32F1xx_HAL_Driver/Src -name "*.c" 2>/dev/null)

# Keep source paths in object names to avoid collisions.
OBJECTS := $(patsubst %.c,$(BUILD_DIR)/%.o,$(C_SOURCES))
OBJECTS += $(patsubst %.s,$(BUILD_DIR)/%.o,$(ASM_SOURCES))

CFLAGS  := $(MCU) $(DEFS) $(INCLUDES)
CFLAGS  += -std=gnu11 -Wall -Wextra
CFLAGS  += -ffunction-sections -fdata-sections
CFLAGS  += -g3 -O0
CFLAGS  += -MMD -MP

ASFLAGS := $(MCU) $(DEFS) $(INCLUDES) -g3

LDFLAGS := $(MCU)
LDFLAGS += -T$(LDSCRIPT)
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref
LDFLAGS += -specs=nano.specs -specs=nosys.specs
LDFLAGS += -lc -lm -lnosys

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SIZE) $@

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) -c $(CFLAGS) -o $@ $<

$(BUILD_DIR)/%.o: %.s
	@mkdir -p $(dir $@)
	$(AS) -c $(ASFLAGS) -o $@ $<

$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O binary -S $< $@

clean:
	rm -rf $(BUILD_DIR)

.PHONY: all clean

-include $(OBJECTS:.o=.d)
