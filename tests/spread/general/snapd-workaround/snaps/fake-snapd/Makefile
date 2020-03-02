# -*- Mode: Makefile; indent-tabs-mode:t; tab-width: 4 -*-

all:
	gcc -o fake-snapd fake-snapd.c

install:
	install -D -m755 fake-snapd "$(DESTDIR)/bin/fake-snapd"
