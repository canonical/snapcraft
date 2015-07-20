# -*- Mode: Makefile; indent-tabs-mode:t; tab-width: 4 -*-

all:

install:
	cp -a bin $(DESTDIR)/bin
	if [ -e copy-meta ]; then \
		cp -arT snappy $(DESTDIR)/meta; \
	fi
	cp -a service-start service-stop binary1 binary2 $(DESTDIR)/
