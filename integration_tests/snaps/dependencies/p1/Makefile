# -*- Mode: Makefile; indent-tabs-mode:t; tab-width: 4 -*-

all:
	gcc -shared -fPIC -o libp1.so $(CFLAGS) $(LDFLAGS) p1.c

install:
	mkdir -p $(DESTDIR)/lib $(DESTDIR)/include
	cp -a libp1.so $(DESTDIR)/lib
	cp -a p1.h $(DESTDIR)/include
