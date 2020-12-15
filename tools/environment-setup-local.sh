#!/bin/bash -ex

SNAPCRAFT_DIR=${SNAPCRAFT_DIR:=$( cd "$(dirname "${BASH_SOURCE[0]}")/.." >/dev/null 2>&1 && pwd )}
SNAPCRAFT_VIRTUAL_ENV_DIR=${SNAPCRAFT_VIRTUAL_ENV_DIR:=${HOME}/.venv/snapcraft}

sudo apt update
sudo apt install --yes \
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

# Create a virtual environment
python3 -m venv "${SNAPCRAFT_VIRTUAL_ENV_DIR}"

# Activate virtual environment
# shellcheck source=/dev/null
source "${SNAPCRAFT_VIRTUAL_ENV_DIR}/bin/activate"

# Install python dependencies
pip install --upgrade wheel
pip install -r "${SNAPCRAFT_DIR}/requirements-devel.txt"
pip install -r "${SNAPCRAFT_DIR}/requirements.txt"

# Install the project for quick tests
pip install --editable "${SNAPCRAFT_DIR}"

# Install black to run static tests.
sudo snap install black --beta

# Install shellcheck for static tests.
sudo snap install shellcheck

echo "Virtual environment may be activated by running:"
echo "source ${SNAPCRAFT_VIRTUAL_ENV_DIR}/bin/activate"
