#!/usr/bin/make -f

# The make plugin runs `make` (default goal) before `make install DESTDIR=...`.
# Keep the default goal a no-op so the initial bare `make` doesn't run `install` with an empty DESTDIR
all:

install:
	install -D -m755 launch "$(DESTDIR)"/snap/command-chain/ros1-launch
	install -D -m755 roslaunch "$(DESTDIR)"/roslaunch
	install -D -m755 rosrun "$(DESTDIR)"/rosrun

.PHONY: all install
