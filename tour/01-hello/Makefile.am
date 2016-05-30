# Process this file with automake to produce Makefile.in (in this,
# and all subdirectories).
# Makefile for the top-level directory of GNU hello.
#
# Copyright 1997, 1998, 2005, 2006, 2007, 2008, 2009, 2010, 2011, 2013,
# 2014 Free Software Foundation, Inc.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

# Find gnulib headers.
ACLOCAL_AMFLAGS = -I m4

AM_CPPFLAGS = -Ilib -I$(top_srcdir)/lib -Isrc -I$(top_srcdir)/src

# Additional files to distribute.
EXTRA_DIST = \
	README-dev \
	ChangeLog.O \
	m4/gnulib-cache.m4 \
	man/hello.x \
	contrib/de_franconian_po.txt \
	contrib/evolution.txt

# Subdirectories to descend into.
SUBDIRS = po

bin_PROGRAMS = hello
hello_SOURCES = src/hello.c src/system.h

hello_LDADD = $(LIBINTL) $(top_builddir)/lib/lib$(PACKAGE).a

localedir = $(datadir)/locale
DEFS = -DLOCALEDIR=\"$(localedir)\" @DEFS@

CLEANFILES =

if !GIT_CROSS_COMPILING
man_MANS = hello.1
EXTRA_DIST += $(man_MANS)
endif

if BUILD_FROM_GIT
hello.1: hello
	$(HELP2MAN) --include=$(top_srcdir)/man/hello.x $(top_builddir)/hello -o $@-t
	chmod a=r $@-t
	mv -f $@-t $@

CLEANFILES += $(man_MANS)
endif

TESTS = \
	tests/greeting-1 \
	tests/greeting-2 \
	tests/hello-1 \
	tests/last-1 \
	tests/traditional-1

EXTRA_DIST += $(TESTS)

noinst_LIBRARIES =
MOSTLYCLEANFILES =
MOSTLYCLEANDIRS =
BUILT_SOURCES =

LOG_COMPILER = $(SHELL)

TESTS_ENVIRONMENT = \
  top_srcdir=$(top_srcdir) \
  PATH=.$(PATH_SEPARATOR)$(PATH_SEPARATOR)$$PATH \
  HELLO=`echo hello | sed '$(transform)'`; \
  export HELLO PATH top_srcdir;

# Verify that all source files using _() are listed in po/POTFILES.in.
# The idea is to run this before making pretests, as well as official
# releases, so that translators will be sure to have all the messages.
# (From coreutils.)
po-check:
	if test -f po/POTFILES.in; then					\
	  grep -E -v '^(#|$$)' po/POTFILES.in				\
	    | grep -v '^src/false\.c$$' | sort > $@-1;			\
	  files=;							\
	  for file in $$($(CVS_LIST_EXCEPT)) `find * -name '*.[ch]'`; do \
	    case $$file in						\
	    gnulib/* | man/*) continue;;				\
	    esac;							\
	    case $$file in						\
	    *.[ch])							\
	      base=`expr " $$file" : ' \(.*\)\..'`;			\
	      { test -f $$base.l || test -f $$base.y; } && continue;;	\
	    esac;							\
	    files="$$files $$file";					\
	  done;								\
	  grep -E -l '\b(N?_|gettext *)\([^)"]*("|$$)' $$files		\
	    | sort -u > $@-2;						\
	  diff -u $@-1 $@-2 || exit 1;					\
	  rm -f $@-1 $@-2;						\
	fi


# Example of updating the online web pages for the documentation
# with the gendocs.sh script; see
# http://www.gnu.org/prep/maintain/html_node/Invoking-gendocs_002esh.html
#
gendocs = build-aux/gendocs.sh
gendocs_envvars = GENDOCS_TEMPLATE_DIR=doc
#
manual = hello
manual_title=Hello, GNU World
email = $(PACKAGE_BUGREPORT)
gendocs_args = --email $(email) $(manual) $(manual_title)
#
www_target = $(HOME)/gnu/www/hello/manual
#
doctemp = doc/wwwtemp
wwwdoc:
	rm -rf $(doctemp) && mkdir $(doctemp)
	cd $(doctemp) \
	&& ln -s ../*.texi . \
	&& env $(gendocs_envvars) $(gendocs) $(gendocs_args)
	cp -arf $(doctemp)/manual/. $(www_target)
	ls -ltu $(www_target)/html_node | tail  # $VCS rm -f obsolete files
# followed by $VCS add of new files and cvs commit.

include $(top_srcdir)/lib/local.mk
include $(top_srcdir)/doc/local.mk

dist-hook: gen-ChangeLog

gen_start_date = 2011-12-17
.PHONY: gen-ChangeLog
gen-ChangeLog:
	$(AM_V_GEN)if test -d .git; then				\
	  log_fix="$(srcdir)/build-aux/git-log-fix";			\
	  test -e "$$log_fix"						\
	    && amend_git_log="--amend=$$log_fix"			\
	    || amend_git_log=;						\
	  $(top_srcdir)/build-aux/gitlog-to-changelog			\
	    $$amend_git_log --since=$(gen_start_date) > $(distdir)/cl-t && \
	    { rm -f $(distdir)/ChangeLog &&				\
	      mv $(distdir)/cl-t $(distdir)/ChangeLog; }		\
	fi
