# -*- Mode: Makefile; indent-tabs-mode:t; tab-width: 4 -*-
.PHONY: all

all: hello-classic

install: hello-classic
	install -d $(DESTDIR)/bin/
	install -D $^ $(DESTDIR)/bin/

hello-classic: hello-classic.c
	$(CC) hello-classic.c -o hello-classic -lcurl
