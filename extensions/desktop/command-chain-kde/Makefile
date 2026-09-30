#!/usr/bin/make -f

BIN_DIR := $(DESTDIR)/snap/command-chain

scripts = hooks-configure-desktop hooks-configure-fonts desktop-launch run

*:
	install -D -m755 "$@" "$(BIN_DIR)"/"$@"

install: $(scripts)

.PHONY: $(scripts)
