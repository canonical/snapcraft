#!/bin/sh

output=$(snapctl get fail)
if [ "$output" = "true" ]; then
	echo "Failing as requested."
	exit 1
fi

echo "I'm the configure hook!"
