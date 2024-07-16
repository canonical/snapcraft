#!/usr/bin/make -f

BIN_DIR := $(DESTDIR)/snap/command-chain
GPU_WRAPPER :=

scripts = hooks-configure-fonts desktop-launch run $(GPU_WRAPPER)

*:
	install -D -m755 "$@" "$(BIN_DIR)"/"$@"

install: $(scripts)

.PHONY: $(scripts)
