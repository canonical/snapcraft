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
lxc init ubuntu:16.04 snapcraft-dev
lxc config set snapcraft-dev raw.idmap "both $UID 1000"
lxc start snapcraft-dev

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
    libxml2-dev \
    libxslt-dev \
    libyaml-dev \
    make \
    p7zip-full \
    patchelf \
    python3-dev \
    python3-pip \
    python3-venv \
    rpm2cpio \
    squashfs-tools

# Create a virtual environment and set it as default 
lxc exec snapcraft-dev -- sudo -iu ubuntu pyvenv .venv/snapcraft
lxc exec snapcraft-dev -- sudo -iu ubuntu bash -c \
    "echo 'source /home/ubuntu/.venv/snapcraft/bin/activate' >> .profile"
lxc exec snapcraft-dev -- sudo -iu ubuntu bash -c \
    "echo 'source /home/ubuntu/.venv/snapcraft/bin/activate' >> .bashrc"
lxc exec snapcraft-dev -- sudo -iu ubuntu pip install --upgrade pip

# Now that /home/ubuntu has been used, add the project
lxc config device add snapcraft-dev snapcraft-project disk \
    source="$PWD" path=/home/ubuntu/snapcraft

# Install python dependencies
lxc exec snapcraft-dev -- sudo -iu ubuntu pip install \
    -r snapcraft/requirements.txt \
    -r snapcraft/requirements-devel.txt

# Install the project for quick tests
lxc exec snapcraft-dev -- sudo -iu ubuntu pip install --editable snapcraft

# Install black to run static tests
lxc exec snapcraft-dev -- snap install black --beta --devmode

echo "Environment ready, enter it by running: "
echo "lxc exec snapcraft-dev -- sudo -iu ubuntu bash"
