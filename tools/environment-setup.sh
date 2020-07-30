#!/bin/bash

set -e

# Check if we are in snapcraft sources
if [ ! -f snap/snapcraft.yaml ]; then
    echo "This tool is meant to be run from the root of the snapcraft source tree."
    exit 1
fi

if ! grep -q '^name: snapcraft$' snap/snapcraft.yaml; then
    echo "This is not the snapcraft.yaml for the snapcraft project"
    exit 1
fi

# Create the container
if ! lxc info snapcraft-dev >/dev/null 2>&1; then
    lxc init ubuntu:18.04 snapcraft-dev
fi
if ! lxc config get snapcraft-dev raw.idmap | grep -q "both $UID 1000"; then
    lxc config set snapcraft-dev raw.idmap "both $UID 1000"
fi

if ! lxc info snapcraft-dev | grep -q "Status: Running"; then
    lxc start snapcraft-dev
fi

# Wait for cloud-init before moving on
lxc exec snapcraft-dev -- cloud-init status --wait

# Install apt dependencies
lxc exec snapcraft-dev -- apt update
lxc exec snapcraft-dev -- apt install --yes \
    execstack \
    g++ \
    gcc \
    libapt-pkg-dev \
    libffi-dev \
    libsodium-dev \
    libssl-dev \
    libxml2-dev \
    libxslt1-dev \
    libyaml-dev \
    make \
    patchelf \
    python3-dev \
    python3-pip \
    python3-venv \
    rpm2cpio \
    squashfs-tools

# Create a virtual environment and set it as default 
lxc exec snapcraft-dev -- sudo -iu ubuntu python3 -m venv .venv/snapcraft
lxc exec snapcraft-dev -- sudo -iu ubuntu bash -c \
    "echo 'source /home/ubuntu/.venv/snapcraft/bin/activate' >> .profile"
lxc exec snapcraft-dev -- sudo -iu ubuntu bash -c \
    "echo 'source /home/ubuntu/.venv/snapcraft/bin/activate' >> .bashrc"
lxc exec snapcraft-dev -- sudo -iu ubuntu pip install --upgrade wheel

# Now that /home/ubuntu has been used, add the project
if ! lxc config device show snapcraft-dev | grep -q snapcraft-project; then
    lxc config device add snapcraft-dev snapcraft-project disk \
        source="$PWD" path=/home/ubuntu/snapcraft
fi

# Install python dependencies
lxc exec snapcraft-dev -- sudo -iu ubuntu pip install \
    -r snapcraft/requirements.txt \
    -r snapcraft/requirements-devel.txt

# Install the project for quick tests
lxc exec snapcraft-dev -- sudo -iu ubuntu pip install --editable snapcraft

# Install black to run static tests.
lxc exec snapcraft-dev -- snap install black --beta

# Install shellcheck for static tests.
lxc exec snapcraft-dev -- snap install shellcheck

echo "Environment ready, enter it by running: "
echo "lxc exec snapcraft-dev -- sudo -iu ubuntu bash"
