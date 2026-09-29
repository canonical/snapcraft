#!/usr/bin/make -f

# The make plugin runs `make` (default goal) before `make install DESTDIR=...`.
# Keep the default goal a no-op so the initial bare `make` doesn't run `install` with an empty DESTDIR.
all:

BIN_DIR := $(DESTDIR)/snap/command-chain
GPU_INTERFACE := gpu-2404

scripts = $(GPU_INTERFACE)-wrapper

$(GPU_INTERFACE)-wrapper: gpu-wrapper
	install -D -m755 gpu-wrapper "$(BIN_DIR)"/"$@"
	sed --in-place 's/%GPU_INTERFACE%/$(GPU_INTERFACE)/g' "$(BIN_DIR)"/"$@"

install: $(scripts)

.PHONY: all $(scripts)
