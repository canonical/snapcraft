# -*- Mode: Makefile; indent-tabs-mode:t; tab-width: 4 -*-

all:
	gcc -shared -fPIC -o libp2.so $(CFLAGS) $(LDFLAGS) p2.c -lp1

install:
	mkdir -p $(DESTDIR)/lib $(DESTDIR)/include
	cp -a libp2.so $(DESTDIR)/lib
	cp -a p2.h $(DESTDIR)/include
