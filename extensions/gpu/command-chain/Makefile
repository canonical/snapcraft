#!/usr/bin/make -f

BIN_DIR := $(DESTDIR)/snap/command-chain
GPU_WRAPPER := gpu-2404-wrapper

scripts = $(GPU_WRAPPER)

*:
	install -D -m755 "$@" "$(BIN_DIR)"/"$@"

install: $(scripts)

.PHONY: $(scripts)
