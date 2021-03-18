#!/usr/bin/make

all:
	$(MAKE) -w -C arduino
	$(MAKE) -w -C pi
	./pi/client

clean:
	$(MAKE) -w -C arduino clean
	$(MAKE) -w -C pi clean

.PHONY: all clean
