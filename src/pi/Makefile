#!/usr/bin/make -f

include ../common/variables.mk

SRC += $(shell find . ../common/ -name '*.c' -o -name '*.cpp')
INC += -I ../common/ -I .
CXXFLAGS += -pthread -std=gnu++17 -Wall -Wextra -Wpedantic -DPORT_NAME=\"$(PORT)\" # We may want to add -Werror later

client: $(SRC)
	$(CXX) $(CXXFLAGS) $(INC) $(SRC) -o $@

clean:
	rm -f client

.PHONY: clean

include ../common/common_tgts.mk
