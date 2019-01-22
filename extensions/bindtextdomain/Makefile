#!/usr/bin/make -f

BINDTEXTDOMAIN = bindtextdomain.so

clean:
	rm -f $(BINDTEXTDOMAIN)

$(BINDTEXTDOMAIN):
	gcc -Wall -O2 -o $(BINDTEXTDOMAIN) -fPIC -shared $(SRC_DIR)/bindtextdomain.c -ldl

install: $(BINDTEXTDOMAIN)
	install -D -m644 $(BINDTEXTDOMAIN) "$(DESTDIR)"/lib/$(BINDTEXTDOMAIN)
