#!/usr/bin/make -f

BIN_DIR := $(DESTDIR)/snap/command-chain
GPU_WRAPPER :=

scripts = hooks-configure-fonts desktop-launch run $(GPU_WRAPPER)

*:
	sed -e "s/%PLATFORM_PLUG%/$${PLATFORM_PLUG:?}/" -i desktop-launch hooks-configure-fonts
	install -D -m755 "$@" "$(BIN_DIR)"/"$@"

install: $(scripts)

.PHONY: $(scripts)
