# -*- Mode: Makefile; indent-tabs-mode:t; tab-width: 8 -*-
.PHONY: all install

DATA_DIR	:= $(DESTDIR)/data-dir
BIN_DIR		:= $(DESTDIR)/snap/command-chain

all:
	@echo -n ""

install:
	install -d $(DATA_DIR)
	install -d $(DATA_DIR)/icons
	install -d $(DATA_DIR)/sounds
	install -d $(DATA_DIR)/themes
	install -d $(BIN_DIR)
	install desktop-common $(BIN_DIR)/
	install desktop-init $(BIN_DIR)/
	install desktop-utils.sh $(BIN_DIR)/
