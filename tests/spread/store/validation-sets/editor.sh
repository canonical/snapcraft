#! /bin/bash

validation_set_file="$1"

cat "$validation_set_file"

cat << EOF > $validation_set_file
account-id: pv8nW1ZaULF7xXAkE3tiU3TdlOnYlGUr
name: testset
sequence: 1
# The revision for this validation set
# revision: 61
snaps:
- id: JtwEnisYi8Mmk51vNLZPSOwSOFLwGdhs
  name: hello-world
  presence: required
EOF
