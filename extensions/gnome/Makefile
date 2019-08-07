# -*- Mode: Makefile; indent-tabs-mode:t; tab-width: 8 -*-
.PHONY: all install

PLATFORM_DIR	:= $(DESTDIR)/gnome-platform
BIN_DIR		:= $(DESTDIR)/snap/command-chain

all:
	@echo -n ""

install:
	install -d $(PLATFORM_DIR)
	install -d $(BIN_DIR)
	install desktop-gnome-specific $(BIN_DIR)/
