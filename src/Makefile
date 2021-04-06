#!/usr/bin/make

include common/variables.mk

all:
	$(MAKE) -w -C arduino
	$(MAKE) -w -C pi
	./pi/client

clean:
	$(MAKE) -w -C arduino clean
	$(MAKE) -w -C pi clean
	rm -rf alex/

lint:
	$(MAKE) -w -C arduino lint
	$(MAKE) -w -C pi lint

format:
	$(MAKE) -w -C arduino format
	$(MAKE) -w -C pi format

ide:
	mkdir alex
	cp -r arduino/* common/* alex/
	mv alex/alex.cpp alex/alex.ino
	rm alex/serialize*
	$(MAKE) -w -C pi

.PHONY: all clean lint format ide
