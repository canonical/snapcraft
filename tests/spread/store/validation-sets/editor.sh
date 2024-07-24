#! /bin/bash

validation_set_file="$1"

# flip-flop between two valid revisions: 100 and 101
if grep -q "^  revision:.*100" "$validation_set_file"; then
  (( revision=101 ))
else
  (( revision=100 ))
fi

# capture core22 id from the staging store
cat "$validation_set_file" > debug.txt

cat << EOF > "$validation_set_file"
account-id: pv8nW1ZaULF7xXAkE3tiU3TdlOnYlGUr
name: testset
sequence: 1
snaps:
- name: core22
  presence: required
  revision: $revision
EOF
