CC      ?= gcc
CFLAGS  += -std=c11 -Wall -Wextra -Wpedantic \
           $(shell pkg-config --cflags glib-2.0 gobject-2.0 libssc 2>/dev/null) \
           -O2 -g
LDFLAGS += $(shell pkg-config --libs glib-2.0 gobject-2.0 libssc 2>/dev/null) \
           -lm

PREFIX  ?= /usr/local
BINDIR  ?= $(PREFIX)/bin

all: ssc-bridge

ssc-bridge: ssc-bridge.c
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

install: ssc-bridge
	install -Dm755 ssc-bridge $(DESTDIR)$(BINDIR)/ssc-bridge

clean:
	rm -f ssc-bridge

.PHONY: all install clean
