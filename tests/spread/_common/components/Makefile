# -*- Mode: Makefile; indent-tabs-mode:t; tab-width: 4 -*-
.PHONY: all

all: hello

install: hello
	install -d $(DESTDIR)/bin/
	install -D $^ $(DESTDIR)/bin/

hello: hello.c
	$(CC) hello.c -o hello -lcurl
