# -*- Mode: Makefile; indent-tabs-mode:t; tab-width: 4 -*-

all:
	echo "file1" > file1
	echo "file2" > file2

install:
	mkdir -p $(DESTDIR)/dir1
	mkdir -p $(DESTDIR)/dir2
	cp -a file1 $(DESTDIR)/
	cp -a file2 $(DESTDIR)/
	cp -a file1 $(DESTDIR)/dir2
