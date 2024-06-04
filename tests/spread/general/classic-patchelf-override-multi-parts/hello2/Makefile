# -*- Mode: Makefile; indent-tabs-mode:t; tab-width: 4 -*-
.PHONY: all

all: hello2

install: hello2
	install -d $(DESTDIR)/bin/
	install -D $^ $(DESTDIR)/bin/

hello2: hello2.c
	$(CC) hello2.c -o hello2 -lcurl
