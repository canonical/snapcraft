# -*- Mode: Makefile; indent-tabs-mode:t; tab-width: 4 -*-

all:
	gcc -o p4 $(CFLAGS) $(LDFLAGS) p4.c -lp1 -lp2

install:
	mkdir -p $(DESTDIR)/bin
	cp -a p4 $(DESTDIR)/bin
