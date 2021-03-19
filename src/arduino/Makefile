#!/usr/bin/make -f

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
OBJCOPY = avr-objcopy
BIN = alex
CLK = 16000000L
CXXFLAGS += -x c++ -std=gnu++17 -Wall -Wextra -Wpedantic -Os -DF_CPU=$(CLK) -mmcu=$(MCU) -fno-exceptions -fno-threadsafe-statics -ffunction-sections -fdata-sections # We may want to add -Werror later
BAUD = 115200
PORT = $(firstword $(wildcard /dev/ttyACM*))
export PORT

flash: $(BIN).hex
	if [ -z "$(PORT)" ]; then false; fi
	$(AVRDUDE) -P $(PORT) -b $(BAUD) -U flash:w:$(BIN).hex:i

$(BIN).hex: $(BIN).elf
	${OBJCOPY} -O ihex -R .eeprom $(BIN).elf $(BIN).hex

$(BIN).elf:
	$(CC) $(CXXFLAGS) $(INC) -w $(ARDUINO_SRC) -c
	$(CC) *.o $(CXXFLAGS) $(INC) $(SRC) -o $@

clean:
	rm -f *.hex *.elf *.o *.a

.PHONY: clean flash
