# -*- Mode: Makefile; indent-tabs-mode:t; tab-width: 4 -*-
.PHONY: all

all: hello-strict

install: hello-strict
	install -d $(DESTDIR)/bin/
	install -D $^ $(DESTDIR)/bin/

hello-strict: hello-strict.c
	$(CC) hello-strict.c -o hello-strict -lcurl
