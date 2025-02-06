#! /bin/bash

confdbs_file="$1"

# flip-flop between 'access' being read and write
if grep -q "^ *access:.*read" "$confdbs_file"; then
  access="write"
else
  access="read"
fi

sed -i "s/^\([[:space:]]*\)access:.*/\1access: $access/g" "$confdbs_file"
