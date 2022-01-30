CC := msp430-elf-gcc
SIZE := msp430-elf-size
TARGET_EXEC ?= gpslogger.elf

BUILD_DIR ?= ./build
SRC_DIRS ?= ./src

SRCS := $(shell find $(SRC_DIRS) -name *.cpp -or -name *.c -or -name *.s)
OBJS := $(SRCS:%=$(BUILD_DIR)/%.o)
DEPS := $(OBJS:.o=.d)

INC_DIRS := $(shell find $(SRC_DIRS) -type d) lib/headers lib/IQMATHLIB_01_10_00_05/include /opt/ti/msp430-gcc/lib/gcc/msp430-elf/9.3.1/include /opt/ti/msp430-gcc/msp430-elf/include /opt/ti/msp430-gcc/include ./src/driverlib/MSP430F5xx_6xx
INC_FLAGS := $(addprefix -I,$(INC_DIRS))

VCS_COMMIT := $(shell git describe --abbrev=4 --dirty --always --tags)

CPPFLAGS ?= $(INC_FLAGS) -MMD -MP -Os -ffunction-sections -fdata-sections -Wl,-gc-sections -D__MSP430F5514__ -D__MSP430__ -std=c11 -D_VCS_COMMIT_=$(VCS_COMMIT) -mlarge -mcode-region=either -mdata-region=either
LDFLAGS ?= $(CPPFLAGS) -Wl,--Map=build/out.map -L ./lib/headers -T msp430f5514.ld -T src/USB_API/msp430USB.ld -Llib/IQMATHLIB_01_10_00_05/libraries/CCS/MPY32/5xx_6xx -l:IQmathLib_CCS_MPY32_5xx_6xx_CPUX_large_code_large_data.lib


$(BUILD_DIR)/$(TARGET_EXEC): $(OBJS)
	$(CC) $(OBJS) -o $@ $(LDFLAGS)
	$(SIZE) $@

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

