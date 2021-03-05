#!/usr/bin/make -f

SRC_DIR ?= .

PLATFORM_DIR         := $(DESTDIR)/gnome-platform
DATA_DIR             := $(DESTDIR)/data-dir
BIN_DIR              := $(DESTDIR)/snap/command-chain
LIB_DIR              := $(DESTDIR)/lib
DEST_LAUNCHER        := desktop-launch
BINDTEXTDOMAIN       := bindtextdomain.so
DEST_CONFIGURE_HOOK  := hooks-configure-desktop
HW_PLATFORM          := $(shell uname --hardware-platform)
ARCH                 := $(HW_PLATFORM)-linux-gnu
WITH_PYTHON          := "3.6"

.PHONY: clean install-$(DESTLAUNCHER) install-$(BINDTEXTDOMAIN) install-$(DEST_CONFIGURE_HOOK)

build: $(DEST_LAUNCHER) $(DEST_CONFIGURE_HOOK)

clean:
	rm -f $(DEST_LAUNCHER)
	rm -f $(DEST_CONFIGURE_HOOK)
	rm -f $(BINDTEXTDOMAIN)

$(DEST_LAUNCHER):
	@cat $(SRC_DIR)/init >> $(DEST_LAUNCHER)
# tail -n +2 to remove the shebang
	@tail -n +2 $(SRC_DIR)/desktop-exports | sed -e "s/%PLATFORM_PLUG%/$${PLATFORM_PLUG:?}/" >> $(DEST_LAUNCHER)
	@tail -n +2 $(SRC_DIR)/launcher-specific | sed -e "s/%WITH_PYTHON%/$(WITH_PYTHON)/" >> $(DEST_LAUNCHER)
	@tail -n +2 $(SRC_DIR)/mark-and-exec >> $(DEST_LAUNCHER)

$(DEST_CONFIGURE_HOOK):
	@cat $(SRC_DIR)/fonts > $(DEST_CONFIGURE_HOOK)

$(BINDTEXTDOMAIN):
	gcc -Wall -O2 -o $(BINDTEXTDOMAIN) -fPIC -shared $(SRC_DIR)/../src/bindtextdomain.c -ldl

install-$(DEST_LAUNCHER): $(DEST_LAUNCHER)
	install -d $(PLATFORM_DIR)
	install -d $(DATA_DIR)
	install -d $(DATA_DIR)/icons
	install -d $(DATA_DIR)/sounds
	install -d $(DATA_DIR)/themes
	install -D -m755 $(DEST_LAUNCHER) "$(BIN_DIR)"/$(DEST_LAUNCHER)

install-$(DEST_CONFIGURE_HOOK): $(DEST_CONFIGURE_HOOK)
	install -D -m755 $(DEST_CONFIGURE_HOOK) "$(BIN_DIR)"/$(DEST_CONFIGURE_HOOK)

install-$(BINDTEXTDOMAIN): $(BINDTEXTDOMAIN)
	install -D -m644 $(BINDTEXTDOMAIN) "$(LIB_DIR)"/$(ARCH)/$(BINDTEXTDOMAIN)

ifdef WITH_BINDTEXTDOMAIN
install: install-$(DEST_LAUNCHER) install-$(DEST_CONFIGURE_HOOK) install-$(BINDTEXTDOMAIN)
else
install: install-$(DEST_LAUNCHER) install-$(DEST_CONFIGURE_HOOK)
endif
