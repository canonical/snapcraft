# -*- Mode: Makefile; indent-tabs-mode:t; tab-width: 4 -*-

HELLO ?= 0

all:
	gcc -DHELLO=$(HELLO) -o hello hello.c

install:
	install -D -m755 hello "$(DESTDIR)/bin/hello"
