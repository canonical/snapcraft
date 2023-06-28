#!/bin/sh -e

test -n "$srcdir" || srcdir=$(dirname "$0")
test -n "$srcdir" || srcdir=.

sed -i "${srcdir}/hello.c" -e "s/ autogen//"
autoreconf --install --verbose "${srcdir}"

# Just make sure NOCONFIGURE is set in the plugin when calling autogen.sh.
test -n "$NOCONFIGURE" || exit 1
