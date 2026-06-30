#!/usr/bin/make -f

# The make plugin runs `make` (default goal) before `make install DESTDIR=...`.
# Keep the default goal a no-op so install only runs with a populated DESTDIR.
all:

install:
	install -D -m755 launch "$(DESTDIR)"/snap/command-chain/ros2-launch
	install -D -m755 ros2 "$(DESTDIR)"/ros2

.PHONY: all install
