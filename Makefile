# Builds the library and the C example. Cross-compile with TARGET=aarch64-unknown-linux-gnu
# (needs `rustup target add $(TARGET)` and a matching C cross compiler in CC).
CC      ?= cc
PREFIX  ?= /usr/local
TARGET  ?=
CARGO_TARGET = $(if $(TARGET),--target $(TARGET),)
LIBDIR  = target/$(if $(TARGET),$(TARGET)/,)release

.PHONY: all lib example test install clean

all: lib example

lib:
	cargo build --release $(CARGO_TARGET)

example: lib
	$(CC) -Wall -Wextra -O2 -Iinclude examples/heat_pump.c $(LIBDIR)/libeebus_c.a \
		-lpthread -ldl -lm -o $(LIBDIR)/heat_pump

test:
	cargo test

install: lib
	install -d $(DESTDIR)$(PREFIX)/include $(DESTDIR)$(PREFIX)/lib
	install -m 644 include/eebus.h $(DESTDIR)$(PREFIX)/include/
	install -m 644 $(LIBDIR)/libeebus_c.a $(DESTDIR)$(PREFIX)/lib/
	install -m 755 $(LIBDIR)/libeebus_c.so $(DESTDIR)$(PREFIX)/lib/

clean:
	cargo clean
