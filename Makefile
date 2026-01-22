# ============================================================
# Project: blinky (bare-metal STM32F401, CMSIS only, no HAL)
# ============================================================

PROJECT := blinky
TARGET  := $(PROJECT).elf
BIN     := $(PROJECT).bin
HEX     := $(PROJECT).hex
MAP     := $(PROJECT).map

# ------------------------------------------------------------
# MCU selection
# ------------------------------------------------------------
MCU        := cortex-m4
MCU_DEF    := STM32F401xC          # <-- THIS WAS MISSING
FPU        := fpv4-sp-d16
FLOAT_ABI  := hard

# ------------------------------------------------------------
# Paths
# ------------------------------------------------------------
ST32_ROOT  := $(HOME)/ST32
CMSIS_ARM  := $(ST32_ROOT)/CMSIS_5/CMSIS/Core/Include
CMSIS_ST   := $(ST32_ROOT)/cmsis-device-f4/Include
STM32_DEV  := $(ST32_ROOT)/cmsis-device-f4/Include
STM32_TPL  := $(ST32_ROOT)/cmsis-device-f4/Source/Templates

# Startup and system files
STARTUP    := $(STM32_TPL)/gcc/startup_stm32f401xc.s
SYSTEM     := $(STM32_TPL)/system_stm32f4xx.c

# Linker script
LDSCRIPT   := stm32f401.ld

# ------------------------------------------------------------
# Toolchain
# ------------------------------------------------------------
CC       := arm-none-eabi-gcc
OBJCOPY  := arm-none-eabi-objcopy
SIZE     := arm-none-eabi-size
OBJDUMP  := arm-none-eabi-objdump

# ------------------------------------------------------------
# Flags
# ------------------------------------------------------------
COMMON_FLAGS = \
  -mcpu=$(MCU) -mthumb \
  -mfpu=$(FPU) -mfloat-abi=$(FLOAT_ABI) \
  -ffreestanding -fdata-sections -ffunction-sections \
  -Wall -Wextra -O2

INCLUDES = \
  -I$(CMSIS_ARM) \
  -I$(CMSIS_ST) 

# *** MCU DEFINE ADDED HERE ***
CFLAGS = $(COMMON_FLAGS) $(INCLUDES) -D$(MCU_DEF)

LDFLAGS = \
  -T$(LDSCRIPT) \
  -Wl,--gc-sections \
  -Wl,-Map=$(MAP)

# ------------------------------------------------------------
# Debug toggle
#   make DEBUG=1
# ------------------------------------------------------------
ifeq ($(DEBUG),1)
  CFLAGS  += -g -O0
  LDFLAGS += -g
else
  CFLAGS  += -DNDEBUG
endif

# ------------------------------------------------------------
# Sources
# ------------------------------------------------------------
SRCS = blinky.c $(SYSTEM)
OBJS = $(SRCS:.c=.o)

# ------------------------------------------------------------
# Build rules
# ------------------------------------------------------------
all: $(TARGET) $(BIN) $(HEX)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) $(STARTUP) $(LDFLAGS) -o $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

$(BIN): $(TARGET)
	$(OBJCOPY) -O binary $< $@

$(HEX): $(TARGET)
	$(OBJCOPY) -O ihex $< $@

# ------------------------------------------------------------
# Size / memory usage
# ------------------------------------------------------------
size: $(TARGET)
	$(SIZE) --format=berkeley $(TARGET)

sections: $(TARGET)
	$(OBJDUMP) -h $(TARGET)

# ------------------------------------------------------------
# Flash via ST-LINK (OpenOCD)
# ------------------------------------------------------------
flash: $(BIN)
	openocd \
	-s /usr/local/share/openocd/scripts \
	-f interface/stlink.cfg \
	-c "transport select swd" \
	-f target/stm32f4x.cfg \
	-c "adapter speed 1000" \
	-c "program blinky.elf verify reset exit"

# ------------------------------------------------------------
# Cleanup
# ------------------------------------------------------------
clean:
	rm -f *.o $(TARGET) $(BIN) $(HEX) $(MAP)

# ------------------------------------------------------------
# Help
# ------------------------------------------------------------
help:
	@echo "make            -> optimized build"
	@echo "make DEBUG=1    -> build with debug symbols"
	@echo "make size       -> show FLASH/RAM usage"
	@echo "make sections   -> show section layout"
	@echo "make flash      -> program via ST-LINK"
	@echo "make clean      -> remove build files"


