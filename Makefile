
# Compile-time options
OPTIONS		?=

# Debugger optons, must be empty or GDB
DEBUG ?=

# Working directories
ROOT		 = $(dir $(lastword $(MAKEFILE_LIST)))
OBJECT_DIR	 = $(ROOT)/obj
BIN_DIR		 = $(ROOT)/obj

LIBSDIR    = ./Libraries
CORELIBDIR = $(LIBSDIR)/CMSIS/Include
DEVDIR  = $(LIBSDIR)/CMSIS/Device/ST/STM32F0xx
STMSPDDIR    = $(LIBSDIR)/STM32F0xx_StdPeriph_Driver
STMSPSRCDDIR = $(STMSPDDIR)/src
STMSPINCDDIR = $(STMSPDDIR)/inc



# Source files common to all targets
SRC =  ./startup/startup_stm32f0xx.s
SRC += ./src/main.c
SRC += ./src/RX.c
SRC += ./src/MPU6050.c
SRC += ./src/adc.c
SRC += ./src/serial.c
SRC += ./src/timer.c
SRC += ./src/adc.c
SRC += ./src/stm32f0xx_it.c
SRC += ./src/system_stm32f0xx.c
SRC += ./src/nrf24l01.c
SRC += ./src/nrf24RX.c
## used parts of the STM-Library
SRC += $(STMSPSRCDDIR)/stm32f0xx_adc.c
SRC += $(STMSPSRCDDIR)/stm32f0xx_cec.c
SRC += $(STMSPSRCDDIR)/stm32f0xx_crc.c
SRC += $(STMSPSRCDDIR)/stm32f0xx_comp.c
SRC += $(STMSPSRCDDIR)/stm32f0xx_dac.c
SRC += $(STMSPSRCDDIR)/stm32f0xx_dbgmcu.c
SRC += $(STMSPSRCDDIR)/stm32f0xx_dma.c
SRC += $(STMSPSRCDDIR)/stm32f0xx_exti.c
SRC += $(STMSPSRCDDIR)/stm32f0xx_flash.c
SRC += $(STMSPSRCDDIR)/stm32f0xx_gpio.c
SRC += $(STMSPSRCDDIR)/stm32f0xx_syscfg.c
SRC += $(STMSPSRCDDIR)/stm32f0xx_i2c.c
SRC += $(STMSPSRCDDIR)/stm32f0xx_iwdg.c
SRC += $(STMSPSRCDDIR)/stm32f0xx_pwr.c
SRC += $(STMSPSRCDDIR)/stm32f0xx_rcc.c
SRC += $(STMSPSRCDDIR)/stm32f0xx_rtc.c
SRC += $(STMSPSRCDDIR)/stm32f0xx_spi.c
SRC += $(STMSPSRCDDIR)/stm32f0xx_tim.c
SRC += $(STMSPSRCDDIR)/stm32f0xx_usart.c
SRC += $(STMSPSRCDDIR)/stm32f0xx_wwdg.c
SRC += $(STMSPSRCDDIR)/stm32f0xx_misc.c



###############################################################################
#
# Things that might need changing to use different tools
#

# Tool names
CC		 = arm-none-eabi-gcc
OBJCOPY		 = arm-none-eabi-objcopy

#
# Tool options.
#
INCLUDE_DIRS = $(DEVDIR)/Include \
          $(CORELIBDIR) \
          $(STMSPINCDDIR) \
          ./src

ARCH_FLAGS	 = -mthumb -mcpu=cortex-m0
BASE_CFLAGS		 = $(ARCH_FLAGS) \
		   $(addprefix -D,$(OPTIONS)) \
		   $(addprefix -I,$(INCLUDE_DIRS)) \
		   -Wall \
		   -ffunction-sections \
		   -fdata-sections \
		   -DSTM32F05X_MD \
		   -DUSE_STDPERIPH_DRIVER \

ASFLAGS		 = $(ARCH_FLAGS) \
		   -x assembler-with-cpp \
		   $(addprefix -I,$(INCLUDE_DIRS))

# XXX Map/crossref output?
LD_SCRIPT	 = $(ROOT)/linker/stm32f0_linker.ld
LDFLAGS		 = -lm \
		   $(ARCH_FLAGS) \
		   -static \
		   -nostartfiles \
		   -Wl,-gc-sections \
		   -T$(LD_SCRIPT)

###############################################################################
# No user-serviceable parts below
###############################################################################

#
# Things we will build
#

ifeq ($(DEBUG),GDB)
CFLAGS = $(BASE_CFLAGS) \
	-ggdb \
	-O0
else
CFLAGS = $(BASE_CFLAGS) \
	-Os
endif

TRGT = arm-none-eabi-

TARGET_HEX	 = $(BIN_DIR)/cheersonAF.hex
TARGET_ELF	 = $(BIN_DIR)/cheersonAF.elf
TARGET_OBJS	 = $(addsuffix .o,$(addprefix $(OBJECT_DIR)/,$(basename $(SRC))))

# List of buildable ELF files and their object dependencies.
# It would be nice to compute these lists, but that seems to be just beyond make.

$(TARGET_HEX): $(TARGET_ELF)
	$(OBJCOPY) -O ihex $< $@
	$(TRGT)size $(TARGET_ELF)
	$(TRGT)size $(TARGET_HEX)

$(TARGET_ELF):  $(TARGET_OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)
	

# Compile
$(OBJECT_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(CFLAGS) $<

# Assemble
$(OBJECT_DIR)/%.o: %.s
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(ASFLAGS) $< 
$(OBJECT_DIR)/%.o): %.S
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(ASFLAGS) $< 

clean:
	rm -f $(TARGET_HEX) $(TARGET_ELF) $(TARGET_OBJS)

help:
	@echo ""
	@echo "Makefile for STM32"
	@echo ""
	@echo "Usage:"
	@echo "        make [OPTIONS=\"<options>\"]"
	@echo ""
