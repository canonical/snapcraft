# -*- Mode: Makefile; indent-tabs-mode:t; tab-width: 4 -*-

all:
	echo "share1" > share1
	echo "share2" > share2
	echo "bin1" > bin1
	echo "bin2" > bin2

install:
	mkdir -p $(DESTDIR)/to-share
	mkdir -p $(DESTDIR)/to-bin1
	mkdir -p $(DESTDIR)/to-bin2
	cp -a share1 $(DESTDIR)/to-share
	cp -a share2 $(DESTDIR)/
	cp -a bin1 $(DESTDIR)/to-bin1
	cp -a bin2 $(DESTDIR)/to-bin2
