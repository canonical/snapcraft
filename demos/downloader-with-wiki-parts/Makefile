# -*- Mode:Makefile; indent-tabs-mode:t; tab-width:4 -*-

all:
	gcc -Wall -Wextra -Werror -pedantic -o test ./test.c $(CFLAGS) $(LDFLAGS) -lcurl -lssl -lcrypto -lssl -lcrypto -lz

install:
	install -d -m755  $(DESTDIR)/bin/
	install -m755 ./test $(DESTDIR)/bin/test

clean:
	rm -f test
