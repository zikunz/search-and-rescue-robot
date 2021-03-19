#!/usr/bin/make

PRG = arduino
MCU = atmega328p
AVRDUDE = avrdude -c $(PRG) -p $(MCU) # flags needed for all invocation
PORT = $(firstword $(wildcard /dev/ttyACM*))
$(if $(shell $(AVRDUDE) -P $(PORT)), $(error PORT not correct.), $(info Select PORT $(PORT)))
export

all:
	$(MAKE) -w -C arduino
	$(MAKE) -w -C pi
	./pi/client

clean:
	$(MAKE) -w -C arduino clean
	$(MAKE) -w -C pi clean

.PHONY: all clean
