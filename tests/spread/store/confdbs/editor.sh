#! /bin/bash

confdb_schema_file="$1"

# flip-flop between 'access' being read and write
if grep -q "^ *access:.*read" "$confdb_schema_file"; then
  access="write"
else
  access="read"
fi

sed -i "s/^\([[:space:]]*\)access:.*/\1access: $access/g" "$confdb_schema_file"
