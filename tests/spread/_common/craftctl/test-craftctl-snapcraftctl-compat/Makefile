CC = gcc
CFLAGS = -O2 -Wall
LD = gcc
LDFLAGS =
OBJS = hello.o
BIN = hello

.c.o:
	$(CC) -c $(CFLAGS) -o$*.o $<

$(BIN): $(OBJS)
	$(LD) -o $@ $(OBJS)

install:
	mkdir -p $(DESTDIR)/usr/bin
	install -m755 $(BIN) $(DESTDIR)/usr/bin

clean:
	rm -f $(OBJS)
