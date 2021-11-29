########################################
## bare metal makefile for ARM Cortex ##
########################################



## This is the name of the elf, bin, obj file produced by this makefile
## Elf file is the one to be used with gdb 
## Bin file is the one to be flashed into the platform 

NAME = LED2_switch
SOURCES = src/
INCLUDES = inc/
BUILD = build/
BIN_NAME  = $(addprefix $(BUILD), $(NAME))
OPENOCD_PATH = /usr/local/share/openocd/scripts/board/
RENODE_SCRIPT = ./renode_sim.resc

## Cross-compilation commands 
GCCPREFIX = /Users/luca/Documents/gcc-arm-none-eabi-10.3-2021.10/bin/arm-none-eabi-
CC      = $(GCCPREFIX)gcc
LD      = $(GCCPREFIX)gcc
AS      = $(GCCPREFIX)as
OBJCOPY = $(GCCPREFIX)objcopy
SIZE    = $(GCCPREFIX)size


## Source files (ASM + C)

SRCS		= $(wildcard $(SOURCES)*.c)
SRCS		+= $(wildcard $(SOURCES)*.s)
SRCS		+= $(wildcard ./*.c)

## Compiled, Linked files (*.o, *.bin, ...)

OBJS = $(wildcard $(BUILD)src/*.o)
OBJS += $(wildcard $(BUILD)src/*.d)
OBJS += $(wildcard $(BUILD)*.o)
OBJS += $(wildcard $(BUILD)*.d)
OBJS += $(wildcard $(BUILD)*.elf)
OBJS += $(wildcard $(BUILD)*.bin)
OBJS += $(wildcard $(BUILD)*.map)

## Compiled Objects files (the ones to be linked together to make the .elf, .bin)

OBJECTS = $(addprefix $(BUILD), $(addsuffix .o, $(basename $(SRCS))))

## Platform and optimization options

# Target architecture (optimise the code for a target processor)
CFLAGS = -mcpu=cortex-m4 

# Use gcc to define a macro (ex: in my code I often use #IFDEF DEBUG ... #ENDIF)
CFLAGS += -DSTM32 
CFLAGS += -DSTM32F4 
CFLAGS += -DDEBUG 
CFLAGS += -DSTM32F411xE

# Optimization level (0, I do not want any optimization, otherwise busy_loop [look in main.c] will probably be deleted by compiler)
CFLAGS +=  -O0 

# Debug level (Comment this if you do not want any debug symbol, remember also to comment the -DDEBUG)
CFLAGS +=-g3 

# Warnings
CFLAGS += -Wall 

# Compilation (code is compiled with respect to ISO std-c99)
CFLAGS += -c
CFLAGS += -std=c99

# Dead code elimination (Options to be used for garbage code elimination during linking phase)

CFLAGS += -ffunction-sections
CFLAGS += -fdata-sections

# Linker
LFLAGS =-mcpu=cortex-m4 --entry Reset_Handler -T"LinkerScript.ld" -Wl,-Map=$(BUILD)output.map -Wl,--gc-sections 

## Rules
all: $(BIN_NAME).elf $(BIN_NAME).bin size 

# print the size of the ELF file
size: $(BIN_NAME).elf
	@echo;
	@echo [SIZE] $<:
	$(SIZE) $< 

# .c sources compiled and saved into ./build directory
$(BUILD)%.o: %.c
	@echo;
	@echo [CC C] $<:
	$(CC) $(CFLAGS) -o $@ $<

# .s sources compiled and saved into ./build directory
$(BUILD)%.o: %.s
	@echo;
	@echo [CC ASM] $<:
	$(CC) $(CFLAGS) -o $@ $<

# linking phase
$(BIN_NAME).elf: $(OBJECTS)
	@echo;
	@echo [LINK] $<:
	$(LD) $(LFLAGS) -o $@ $^

# binary
$(BIN_NAME).bin: $(BIN_NAME).elf
	@echo;
	@echo [BIN] $<:
	$(OBJCOPY) -O binary $< $@


# clean everything that's been produced by this makefile
.PHONY: clean
clean:
	@echo;
	@echo [CLEAN] $<:
	rm -f $(OBJS)

# flash the code into the board, tool used: st-flash (st-link tools)
.PHONY: install
install: $(BIN_NAME).bin
	@echo;
	@echo [INSTALL] $<:
	st-flash write $< 0x08000000

# open a gdb server on port 3333, tool used: openocd 
.PHONY: connect
connect:
	@echo;
	@echo [CONNECT] $<:
	openocd -f $(OPENOCD_PATH)st_nucleo_f4.cfg

# start a renode simulation, tool used: renode
.PHONY: sim
sim:
	@echo;
	@echo [SIM] $<:
	/Applications/Renode.app/Contents/MacOS/macos_run.command $(RENODE_SCRIPT)









