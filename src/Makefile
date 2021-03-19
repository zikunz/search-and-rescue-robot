#!/usr/bin/make

include common/variables.mk

all:
	$(MAKE) -w -C arduino
	$(MAKE) -w -C pi
	./pi/client

clean:
	$(MAKE) -w -C arduino clean
	$(MAKE) -w -C pi clean

lint:
	$(MAKE) -w -C arduino lint
	$(MAKE) -w -C pi lint

format:
	$(MAKE) -w -C arduino format
	$(MAKE) -w -C pi format

.PHONY: all clean lint format
