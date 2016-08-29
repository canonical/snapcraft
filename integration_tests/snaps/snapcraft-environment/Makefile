# -*- Mode: Makefile; indent-tabs-mode:t; tab-width: 4 -*-

all:
	echo -n "$(SNAPCRAFT_PROJECT_NAME)" > test_name
	echo -n "$(SNAPCRAFT_PROJECT_VERSION)" > test_version
	echo -n "$(SNAPCRAFT_STAGE)" > test_stage
	echo -n "$(SNAPCRAFT_PART_INSTALL)" > test_part_install

install:
	mkdir -p $(DESTDIR)
	install -m 644 test_name $(DESTDIR)
	install -m 644 test_version $(DESTDIR)
	install -m 644 test_stage $(DESTDIR)
	install -m 644 test_part_install $(DESTDIR)
