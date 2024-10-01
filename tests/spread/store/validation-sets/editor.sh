#! /bin/bash

validation_set_file="$1"

# flip-flop between 'hello-world' being optional or required
if grep -q "^  presence:.*optional" "$validation_set_file"; then
  presence="required"
else
  presence="optional"
fi

sed -i "s/  presence:.*/  presence: $presence/g" "$validation_set_file"
