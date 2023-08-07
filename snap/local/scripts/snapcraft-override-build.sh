#!/bin/sh

set -e
set -x

# Restore patched files
PYTHON_PACKAGE_PATH="${SNAPCRAFT_PART_INSTALL}/usr/lib/python3.8/"
[ -f "patched/ctypes/__init__.py.orig" ] && mv "patched/ctypes/__init__.py.orig" "${PYTHON_PACKAGE_PATH}/ctypes/__init__.py"

# Apply patches
echo "Patching ctypes..."
patch -s -b "${PYTHON_PACKAGE_PATH}/ctypes/__init__.py" "${SNAPCRAFT_PROJECT_DIR}/snap/local/patches/python/ctypes_init.diff"

# Save patches to allow rebuilding
mkdir -p patched/ctypes
[ -f "${PYTHON_PACKAGE_PATH}/ctypes/__init__.py.orig" ] && mv "${PYTHON_PACKAGE_PATH}/ctypes/__init__.py.orig" patched/ctypes

sed -i "${SNAPCRAFT_PART_INSTALL}/usr/lib/python3.8/site.py" -e 's/^ENABLE_USER_SITE = None$/ENABLE_USER_SITE = False/'
