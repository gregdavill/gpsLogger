CC := msp430-elf-gcc
SIZE := msp430-elf-size
OBJCOPY := msp430-elf-objcopy
OBJDUMP := msp430-elf-objdump
TARGET_EXEC ?= gpslogger

BUILD_DIR ?= ./build
LIB_DIR ?= ../lib
SRC_DIRS ?= ./src $(LIB_DIR)

SRCS := $(shell find $(SRC_DIRS) -name *.cpp -or -name *.c -or -name *.s)
OBJS := $(SRCS:%=$(BUILD_DIR)/%.o)
DEPS := $(OBJS:.o=.d)

INC_DIRS := $(shell find $(SRC_DIRS) -type d) $(LIB_DIR)/headers $(LIB_DIR)/IQMATHLIB_01_10_00_05/include 
INC_FLAGS := $(addprefix -I,$(INC_DIRS))

VCS_COMMIT := $(shell git describe --abbrev=4 --dirty --always --tags)

CPPFLAGS ?= $(INC_FLAGS) -MMD -MP -Os -ffunction-sections -fdata-sections -Wl,-gc-sections -D__MSP430F5514__ -D__MSP430__ -std=gnu18 -D_VCS_COMMIT_=$(VCS_COMMIT) 
LDFLAGS ?= $(CPPFLAGS) -Wl,--Map=build/out.map -L ./$(LIB_DIR)/headers -T msp430f5514.ld -T src/USB_API/msp430USB.ld $(LIB_DIR)/IQMATHLIB_01_10_00_05/libraries/CCS/MPY32/5xx_6xx/IQmathLib_CCS_MPY32_5xx_6xx_CPUX_small_code_small_data.lib

all: $(BUILD_DIR)/$(TARGET_EXEC).elf $(BUILD_DIR)/$(TARGET_EXEC).bin $(BUILD_DIR)/$(TARGET_EXEC).lst $(BUILD_DIR)/$(TARGET_EXEC).dmp

$(BUILD_DIR)/$(TARGET_EXEC).elf: $(OBJS)
	$(CC) $(OBJS) -o $@ $(LDFLAGS)
	$(SIZE) $@

%.dmp: %.bin
	xxd -o 0x4400 $< $@

%.lst : %.elf
	$(OBJDUMP) -DS $< > $@ 

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@
	./append_crc.py $@

# assembly
$(BUILD_DIR)/%.s.o: %.s
	$(MKDIR_P) $(dir $@)
	$(AS) $(ASFLAGS) -c $< -o $@

# c source
$(BUILD_DIR)/%.c.o: %.c
	$(MKDIR_P) $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

# c++ source
$(BUILD_DIR)/%.cpp.o: %.cpp
	$(MKDIR_P) $(dir $@)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@


.PHONY: clean

clean:
	$(RM) -r $(BUILD_DIR)

-include $(DEPS)

MKDIR_P ?= mkdir -p

 