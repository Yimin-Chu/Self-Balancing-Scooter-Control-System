######################################
# 平衡小车 STM32F103C8T6 命令行构建 (arm-none-eabi-gcc + make)
# 模仿追觅命令行 Makefile 思路，不依赖 Keil
######################################
TARGET     = balance_car
BUILD_DIR  = build
DEBUG      = 1
OPT        = -Og

######################################
# 源文件
######################################
# C 源码：用户代码 + HAL 库(自动收集，排除 *_template.c)
C_SOURCES =  \
Core/Src/main.c \
Core/Src/pid.c \
Core/Src/sr04.c \
Core/Src/motor.c \
Core/Src/encoder.c \
Core/Src/oled.c \
Core/Src/gpio.c \
Core/Src/i2c.c \
Core/Src/tim.c \
Core/Src/usart.c \
Core/Src/stm32f1xx_it.c \
Core/Src/stm32f1xx_hal_msp.c \
Core/Src/system_stm32f1xx.c \
MyCode/mpu6050.c \
MyCode/IIC.c \
MyCode/inv_mpu.c \
MyCode/inv_mpu_dmp_motion_driver.c \
$(filter-out %_template.c,$(wildcard Drivers/STM32F1xx_HAL_Driver/Src/*.c))

# 汇编启动文件 (GCC 版)
ASM_SOURCES = Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/startup_stm32f103xb.s

######################################
# 工具链
######################################
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size

# STM32F103 = Cortex-M3, 无 FPU
MCU = -mcpu=cortex-m3 -mthumb

######################################
# 宏定义 / 头文件路径
######################################
C_DEFS = -DUSE_HAL_DRIVER -DSTM32F103xB

C_INCLUDES = \
-ICore/Inc \
-IMyCode \
-IDrivers/STM32F1xx_HAL_Driver/Inc \
-IDrivers/STM32F1xx_HAL_Driver/Inc/Legacy \
-IDrivers/CMSIS/Device/ST/STM32F1xx/Include \
-IDrivers/CMSIS/Include

######################################
# 编译选项
######################################
ASFLAGS = $(MCU) $(OPT) -Wall -fdata-sections -ffunction-sections
CFLAGS  = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections
ifeq ($(DEBUG),1)
CFLAGS += -g -gdwarf-2
endif
# 自动生成头文件依赖 (.d)
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

######################################
# 链接选项
######################################
LDSCRIPT = STM32F103C8Tx_FLASH.ld
LIBS = -lc -lm -lnosys
# -u _printf_float: 让 nano 库的 sprintf 支持 %f (OLED 显示 roll 用了 %.1f)
LDFLAGS = $(MCU) -specs=nano.specs -u _printf_float -T$(LDSCRIPT) $(LIBS) \
          -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

######################################
# 构建产物
######################################
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

OBJECTS  = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(CP) -O ihex $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(CP) -O binary -S $< $@

$(BUILD_DIR):
	mkdir "$(BUILD_DIR)"

######################################
# 烧录 (ST-Link, SWD)
######################################
# 默认用 ST 官方 STM32_Programmer_CLI
flash: $(BUILD_DIR)/$(TARGET).hex
	STM32_Programmer_CLI -c port=SWD -w $< -rst

# 备选: 开源 stlink-tools
flash-stlink: $(BUILD_DIR)/$(TARGET).bin
	st-flash write $< 0x08000000

# 备选: OpenOCD
flash-openocd: $(BUILD_DIR)/$(TARGET).elf
	openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c "program $< verify reset exit"

######################################
# 清理
######################################
# Windows cmd/PowerShell:
clean:
	-rmdir /S /Q "$(BUILD_DIR)"
# Git Bash / MSYS2 请改用: rm -rf $(BUILD_DIR)

# 头文件依赖
-include $(wildcard $(BUILD_DIR)/*.d)

.PHONY: all clean flash flash-stlink flash-openocd
