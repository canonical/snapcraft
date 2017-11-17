# -*- Mode: Makefile; indent-tabs-mode:t; tab-width: 4 -*-
.PHONY: all

all: hello-classic

install: hello-classic
	install -d $(DESTDIR)/bin
	install $^ $(DESTDIR)/bin/

hello-classic: hello-classic.c
