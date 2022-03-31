#!/usr/bin/make -f

BIN_DIR              := $(DESTDIR)/snap/command-chain

*:
	install -D -m755 "$@" "$(BIN_DIR)"/"$@"

install: hooks-configure-fonts desktop-launch run

.PHONY: hooks-configure-fonts desktop-launch run