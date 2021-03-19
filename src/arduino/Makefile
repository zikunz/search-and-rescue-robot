#!/usr/bin/make -f

include ../common/variables.mk

SRC = $(shell find . ../common/ -name '*.c' -o -name '*.cpp' -o -name '*.ino')
# SRC = $(wildcard **/*.cpp) $(wildcard **/*.ino) $(wildcard **/*.c) $(wildcard ../common/**/*.cpp) $(wildcard ../common/**/*.ino) $(wildcard ../common/**/*.c) # not elegant but may work on Windows

######## THESE CAN BE SAFELY DELETED AFTER WE BARE-METAL EVERYTHING ########
ARDUINO_DIR = /usr/share/arduino/hardware/arduino
ARDUINO_SRC = $(filter-out %WInterrupts.c %main.cpp, $(wildcard $(ARDUINO_DIR)/cores/arduino/*.c) $(wildcard $(ARDUINO_DIR)/cores/arduino/*.cpp))
INC += -I $(ARDUINO_DIR)/cores/arduino -I $(ARDUINO_DIR)/variants/standard
############################################################################

INC += -I ../common/ -I .
###### AVR TOOLCHAIN ######
CC = avr-gcc
INC += $(patsubst %, -I%,$(shell $(CC) -E -Wp,-v -xc++ /dev/null 2>&1 | awk '/\#include <...> search starts here:/,/End of search list./' | sed '1d;$$d'))
OBJCOPY = avr-objcopy
BIN = alex
CLK = 16000000L
CXXFLAGS += -x c++ -std=gnu++17 -Wall -Wextra -Wpedantic -Os -DF_CPU=$(CLK) -mmcu=$(MCU) -fno-exceptions -fno-threadsafe-statics -ffunction-sections -fdata-sections # We may want to add -Werror later
BAUD = 115200

flash: $(BIN).hex
	$(AVRDUDE) -P $(PORT) -b $(BAUD) -U flash:w:$(BIN).hex:i

$(BIN).hex: $(BIN).elf
	${OBJCOPY} -O ihex -R .eeprom $(BIN).elf $(BIN).hex

$(BIN).elf:
	$(CC) $(CXXFLAGS) $(INC) -w $(ARDUINO_SRC) -c
	$(CC) *.o $(CXXFLAGS) $(INC) $(SRC) -o $@

clean:
	rm -f *.hex *.elf *.o *.a

.PHONY: clean flash

include ../common/common_tgts.mk
