#!/bin/bash

EXAMPLES=`find examples/ -maxdepth 1 -type d | tail -n +2`

for example in $EXAMPLES; do
	echo $example
	(cd $example ; ../../bin/snapcraft $* )
done
