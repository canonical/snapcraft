#!/bin/bash

set -e

EXAMPLES=`find examples/ -maxdepth 1 -type d | tail -n +2`

for example in $EXAMPLES; do
	echo "Building snap for: $example"
	(cd $example && \
		../../bin/snapcraft $* && \
		../../bin/snapcraft clean )
done
