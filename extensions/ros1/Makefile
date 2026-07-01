#!/usr/bin/make -f

# The make plugin runs `make` (default goal) before `make install DESTDIR=...`.
# Keep the default goal a no-op so install only runs with a populated DESTDIR.
all:

install:
	install -D -m755 launch "$(DESTDIR)"/snap/command-chain/ros1-launch
	install -D -m755 roslaunch "$(DESTDIR)"/roslaunch
	install -D -m755 rosrun "$(DESTDIR)"/rosrun

.PHONY: all install
