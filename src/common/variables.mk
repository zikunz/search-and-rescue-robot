TIDY = clang-tidy
FORMAT = clang-format
PRG = arduino
MCU = atmega328p
AVRDUDE = avrdude -c $(PRG) -p $(MCU) # flags needed for all invocation
TIDY_OPT += -extra-arg-before=-xc++
FORMAT_OPT += -i
PORT = $(firstword $(wildcard /dev/ttyACM*))
$(if $(shell $(AVRDUDE) -P $(PORT)), $(error PORT not correct.), $(info Select PORT $(PORT)))
