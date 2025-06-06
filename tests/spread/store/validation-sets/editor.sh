#! /bin/bash

validation_set_file="$1"

# flip-flop between 'hello-world' being optional or required
if grep -q "^  presence:.*optional" "$validation_set_file"; then
  presence="required"
else
  presence="optional"
fi

sed -i "s/  presence:.*/  presence: $presence/g" "$validation_set_file"

# increment the sequence
# shellcheck disable=SC2002 # yq snap can't access /tmp
cat "$validation_set_file" | yq '.sequence += 1' | tee "$validation_set_file"
