all: main

libfoo.so: foo.h foo.cpp
	@g++ -fPIC -c foo.cpp
	@g++ -shared foo.o -o libfoo.so
	@mkdir -p lib
	@mv libfoo.so lib/

main: libfoo.so
	@g++ main.cpp -o binary -Wl,-rpath=\$$ORIGIN/lib -Llib -lfoo

install: main
	@mkdir -p $(DESTDIR)/lib
	@cp binary $(DESTDIR)/
	@cp lib/* $(DESTDIR)/lib/

clean:
	@rm *.o binary
	@rm -rf lib
