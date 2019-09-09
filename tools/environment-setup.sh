#!/bin/bash

set -e

CONTAINER_NAME="${1:-snapcraft-dev}"
CONTAINER_IMAGE="${2:-16.04}"

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
if ! lxc info "${CONTAINER_NAME}" >/dev/null 2>&1; then
    lxc init "ubuntu:${CONTAINER_IMAGE}" "${CONTAINER_NAME}"
fi
if ! lxc config get "${CONTAINER_NAME}" raw.idmap | grep -q "both $UID 1000"; then
    lxc config set "${CONTAINER_NAME}" raw.idmap "both $UID 1000"
fi

if ! lxc info "${CONTAINER_NAME}" | grep -q "Status: Running"; then
    lxc start "${CONTAINER_NAME}"
fi

# Wait for cloud-init before moving on
lxc exec "${CONTAINER_NAME}" -- cloud-init status --wait

# Install apt dependencies
lxc exec "${CONTAINER_NAME}" -- apt update
lxc exec "${CONTAINER_NAME}" -- apt install --yes \
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
lxc exec "${CONTAINER_NAME}" -- sudo -iu ubuntu pyvenv .venv/snapcraft
lxc exec "${CONTAINER_NAME}" -- sudo -iu ubuntu bash -c \
    "echo 'source /home/ubuntu/.venv/snapcraft/bin/activate' >> .profile"
lxc exec "${CONTAINER_NAME}" -- sudo -iu ubuntu bash -c \
    "echo 'source /home/ubuntu/.venv/snapcraft/bin/activate' >> .bashrc"
lxc exec "${CONTAINER_NAME}" -- sudo -iu ubuntu pip install --upgrade pip

# Now that /home/ubuntu has been used, add the project
if ! lxc config device show "${CONTAINER_NAME}" | grep -q snapcraft-project; then
    lxc config device add "${CONTAINER_NAME}" snapcraft-project disk \
        source="$PWD" path=/home/ubuntu/snapcraft
fi

# Install python dependencies
lxc exec "${CONTAINER_NAME}" -- sudo -iu ubuntu pip install \
    -r snapcraft/requirements.txt \
    -r snapcraft/requirements-devel.txt

# Install the project for quick tests
lxc exec "${CONTAINER_NAME}" -- sudo -iu ubuntu pip install --editable snapcraft

# Install black to run static tests
lxc exec "${CONTAINER_NAME}" -- snap install black --beta --devmode

echo "Environment ready, enter it by running: "
echo "lxc exec ${CONTAINER_NAME} -- sudo -iu ubuntu bash"
