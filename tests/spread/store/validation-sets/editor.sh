#! /bin/bash

validation_set_file="$1"

# flip-flop between 'hello-world' being optional or required
if grep -q "^  presence:.*optional" "$validation_set_file"; then
  presence="required"
else
  presence="optional"
fi

sed -i "s/  presence:.*/  presence: $presence/g" "$validation_set_file"

# Increments the sequence.
#
# shellcheck disable=SC2002 # yq snap can't access /tmp
new_content="$(cat "$validation_set_file" | yq '.sequence += 1')"
echo "$new_content" >"$validation_set_file"
cat "$validation_set_file"
