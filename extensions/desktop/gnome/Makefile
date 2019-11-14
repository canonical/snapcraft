#!/usr/bin/make -f

SRC_DIR ?= .

PLATFORM_DIR := $(DESTDIR)/gnome-platform
DATA_DIR     := $(DESTDIR)/data-dir
BIN_DIR      := $(DESTDIR)/snap/command-chain
LIB_DIR	     := $(DESTDIR)/lib
DEST_LAUNCHER	:= desktop-launch
BINDTEXTDOMAIN	:= bindtextdomain.so

build: $(DEST_LAUNCHER)

clean:
	rm -f $(DEST_LAUNCHER)
	rm -f $(FLAVOR_FILE)
	rm -f $(BINDTEXTDOMAIN)

$(DEST_LAUNCHER):
	@cat $(SRC_DIR)/init >> $(DEST_LAUNCHER)
# tail -n +2 to remove the shebang
	@tail -n +2 $(SRC_DIR)/desktop-exports >> $(DEST_LAUNCHER)
	@tail -n +2 $(SRC_DIR)/launcher-specific >> $(DEST_LAUNCHER)
	@tail -n +2 $(SRC_DIR)/mark-and-exec >> $(DEST_LAUNCHER)
	gcc -Wall -O2 -o $(BINDTEXTDOMAIN) -fPIC -shared $(SRC_DIR)/../src/bindtextdomain.c -ldl

install: $(DEST_LAUNCHER)
	install -d $(PLATFORM_DIR)
	install -d $(DATA_DIR)
	install -d $(DATA_DIR)/icons
	install -d $(DATA_DIR)/sounds
	install -d $(DATA_DIR)/themes
	install -D -m755 $(DEST_LAUNCHER) "$(BIN_DIR)"/$(DEST_LAUNCHER)
	install -D -m644 $(BINDTEXTDOMAIN) "$(LIB_DIR)"/$(BINDTEXTDOMAIN)
