#! /bin/bash

validation_set_file="$1"

# flip-flop between two valid revisions: 1 and 2
if grep -q "^  revision:.*1" "$validation_set_file"; then
  (( revision=2 ))
else
  (( revision=1 ))
fi

cat "$validation_set_file" > debug-before.txt

sed -i "s/  revision:.*/  revision: $revision/g" $validation_set_file

cat "$validation_set_file" > debug-after.txt
