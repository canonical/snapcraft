#! /bin/bash

registries_file="$1"

# flip-flop between 'access' being read and write
if grep -q "^ *access:.*read" "$registries_file"; then
  access="write"
else
  access="read"
fi

sed -i "s/^\([[:space:]]*\)access:.*/\1access: $access/g" "$registries_file"
