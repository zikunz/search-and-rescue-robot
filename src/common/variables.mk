TIDY = clang-tidy
FORMAT = clang-format
PRG = arduino
MCU = atmega328p
AVRDUDE = avrdude -c $(PRG) -p $(MCU) # flags needed for all invocation
TIDY_OPT += -extra-arg-before=-xc++
FORMAT_OPT += -i
PORT ?= $(strip $(firstword $(wildcard /dev/ttyACM*)))
PORT := $(if $(PORT),$(PORT),"/dev/null")# if empty or null
$(shell $(AVRDUDE) -P $(PORT))
