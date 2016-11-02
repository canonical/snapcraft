# -*- Mode: Makefile; indent-tabs-mode:t; tab-width: 4 -*-

LIBNAME      = apitest
BINNAME      = testapp
CFLAGS       = -fPIC -c #-pedantic -Wall -Wextra -ggdb3
LDFLAGS      = -shared -Wl
CC           = gcc

all: libapitest #main 

libapitest: 
	$(CC) $(CFLAGS)  libapitest.c -o libapitest.o
	$(CC) $(LDFLAGS),-soname,libapitest.so.1 -o libapitest.so.1.0.1 libapitest.o
	ln -s lib$(LIBNAME).so.1.0.1 lib$(LIBNAME).so.1
	ln -s lib$(LIBNAME).so.1.0.1 lib$(LIBNAME).so

main:
	$(CC) main.c -L. -l$(LIBNAME) -o $(BINNAME)

install:
	cp -a * $(DESTDIR)

.PHONY: clean

clean:
	rm -f *.o *$(LIBNAME).so* core *~ $(BINNAME)

