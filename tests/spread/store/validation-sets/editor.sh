#! /bin/bash

validation_set_file="$1"

# flip-flop between two valid revisions of `test-snapcraft-assertions` in the staging store: 1 and 2
if grep -q "^  revision:.*1" "$validation_set_file"; then
  (( revision=2 ))
else
  (( revision=1 ))
fi

sed -i "s/  revision:.*/  revision: $revision/g" "$validation_set_file"
