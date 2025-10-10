SRC_DIR = ./src/
BUILD_DIR = ./build/
BIN_DIR = ./bin/

# UTILITY VARIABLES
AS = arm-none-eabi-as
CC = arm-none-eabi-gcc
LD = arm-none-eabi-ld
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SZ = arm-none-eabi-size
RM      = rm -rf
MKDIR   = @mkdir -p $(@D) #creates folders if not present

# debug build?
# FLAGS
MCU = -mcpu=cortex-m33 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16
DEBUG = 1
OPT = -Og
LIBS = -lc -lm -lnosys
LDSCRIPT = $(SRC_DIR)stm32h563.ld
INC = -I$(SRC_DIR)/inc -I$(SRC_DIR)/hw

ASFLAGS = $(MCU) $(OPT)
CFLAGS = $(MCU) $(OPT) $(INC) -Wall -fdata-sections -ffunction-sections
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBS) -Wl,-Map=$(BIN_DIR)main.map,--cref -Wl,--gc-sections -Wl,--print-memory-usage

CFLAGS += -ffreestanding -nostdlib -fno-builtin

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif

# Sources
C_SRCS = $(wildcard $(SRC_DIR)*.c $(SRC_DIR)hw/*.c)
S_SRCS = $(wildcard *.s)
OBJS = $(addprefix $(BUILD_DIR),$(notdir $(C_SRCS:.c=.o)))
vpath %.c $(sort $(dir $(C_SRCS)))
OBJS += $(addprefix $(BUILD_DIR),$(notdir $(S_SRCS:.s=.o)))
vpath %.s $(sort $(dir $(S_SRCS)))


TARGET = $(BIN_DIR)main.elf
BIN    = $(BIN_DIR)main.bin
HEX    = $(BIN_DIR)main.hex

all: $(TARGET) $(BIN)

# Compile C files
$(BUILD_DIR)%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

# Link
$(TARGET): $(OBJS) $(LDSCRIPT) Makefile | $(BIN_DIR)
#$(LD) $(OBJS) $(LDFLAGS) -o $@
	$(CC) $(OBJS) $(LDFLAGS) -o $@
	$(SZ) $@

# Generate binary
$(BIN): $(TARGET) Makefile | $(BIN_DIR)
	$(OBJCOPY) -O binary -S $< $@

# Generate hex
$(HEX): $(TARGET) Makefile | $(BIN_DIR)
	$(OBJCOPY) -O ihex $< $@

$(BUILD_DIR):
	mkdir $@
	
$(BIN_DIR):
	mkdir $@


flash: $(BIN)
	openocd -f interface/stlink-dap.cfg -f target/stm32h5x.cfg \
	-c "program $(BIN) verify reset exit 0x08000000"

clean:
	rm -rf build
	rm -rf bin
