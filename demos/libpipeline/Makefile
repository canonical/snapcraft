# -*- Mode:Makefile; indent-tabs-mode:t; tab-width:4 -*-

all:
	gcc -o test ./test.c $(CFLAGS) $(LDFLAGS) -lpipeline

install:
	install -d -m755  $(DESTDIR)/bin/
	install -m755 ./test $(DESTDIR)/bin/test

clean:
	rm -f test
