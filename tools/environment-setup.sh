#!/bin/bash -e

SNAPCRAFT_DIR=${SNAPCRAFT_DIR:=$( cd "$(dirname "${BASH_SOURCE[0]}")/.." >/dev/null 2>&1 && pwd )}

# Check if SNAPCRAFT_DIR is pointing to snapcraft sources.
if ! grep -q '^name: snapcraft$' "${SNAPCRAFT_DIR}/snap/snapcraft.yaml"; then
    echo "This is not the snapcraft.yaml for the snapcraft project"
    exit 1
fi

# Create the container.
if ! lxc info snapcraft-dev >/dev/null 2>&1; then
    lxc init ubuntu:18.04 snapcraft-dev
fi
if ! lxc config get snapcraft-dev raw.idmap | grep -q "both $UID 1000"; then
    lxc config set snapcraft-dev raw.idmap "both $UID 1000"
fi

if ! lxc info snapcraft-dev | grep -q "Status: Running"; then
    lxc start snapcraft-dev
fi

# Wait for cloud-init before moving on.
lxc exec snapcraft-dev -- cloud-init status --wait

# First login for ubuntu user.
lxc exec snapcraft-dev -- sudo -iu ubuntu bash -c true

# Now that /home/ubuntu has been used, add the project.
if ! lxc config device show snapcraft-dev | grep -q snapcraft-project; then
    lxc config device add snapcraft-dev snapcraft-project disk \
        source="$SNAPCRAFT_DIR" path=/home/ubuntu/snapcraft
fi

# Install snapcraft and dependencies.
lxc exec snapcraft-dev -- sudo -iu ubuntu /home/ubuntu/snapcraft/tools/environment-setup-local.sh

# Set virtual environment on login.
lxc exec snapcraft-dev -- sudo -iu ubuntu bash -c \
    "echo 'source /home/ubuntu/.venv/snapcraft/bin/activate' >> .profile"
lxc exec snapcraft-dev -- sudo -iu ubuntu bash -c \
    "echo 'source /home/ubuntu/.venv/snapcraft/bin/activate' >> .bashrc"

echo "Container ready, enter it by running: "
echo "lxc exec snapcraft-dev -- sudo -iu ubuntu bash"
