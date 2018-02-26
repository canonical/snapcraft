# -*- Mode: Makefile; indent-tabs-mode:t; tab-width: 4 -*-

all:
	gcc -o test test.c

install:
	execstack --set-execstack test
	install -d $(DESTDIR)/bin
	install test $(DESTDIR)/bin/
	