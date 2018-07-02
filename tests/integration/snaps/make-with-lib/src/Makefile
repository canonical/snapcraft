all: usefoo

foo.a: foo.h foo.cpp
	@echo "Building foo"
	@g++ -c foo.cpp
	@ar crs foo.a foo.o

usefoo: foo.a foo.h main.cpp
	@echo "Building usefoo"
	@g++ main.cpp foo.a -o usefoo

install:
	@install -d ${SNAPCRAFT_PART_INSTALL}/bin
	@install usefoo ${SNAPCRAFT_PART_INSTALL}/bin

clean:
	@rm *.a *.o usefoo
