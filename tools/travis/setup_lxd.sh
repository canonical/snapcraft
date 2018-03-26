#!/bin/bash
#
# Copyright (C) 2017 Canonical Ltd
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 3 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

# Set up LXD in travis.

set -ev

while fuser /var/lib/apt/lists/lock >/dev/null 2>&1; do
    sleep 1
done
apt-get update
while fuser /var/lib/dpkg/lock >/dev/null 2>&1; do
    sleep 1
done
apt-get install --yes snapd
apt-get remove --yes lxd lxd-client

if [ ! -f $TRAVIS_BUILD_DIR/snaps-cache/downloaded ]; then
    mkdir -p "$TRAVIS_BUILD_DIR/snaps-cache"
    pushd "$TRAVIS_BUILD_DIR/snaps-cache"
    snap download core
    snap download lxd
    touch downloaded
    popd
fi  
# snap install core exits with this error message:
# - Setup snap "core" (3604) security profiles (cannot reload udev rules: exit status 2)
# but the installation succeeds, so we just ingore it.
snap ack $TRAVIS_BUILD_DIR/snaps-cache/core_*.assert
snap install $TRAVIS_BUILD_DIR/snaps-cache/core_*.snap || echo 'ignored error'
snap ack $TRAVIS_BUILD_DIR/snaps-cache/lxd_*.assert
snap install $TRAVIS_BUILD_DIR/snaps-cache/lxd_*.snap
# Wait while LXD first generates its keys. In a low entropy environment this
# can take a while.

# From LXD's CI.
# shellcheck disable=SC2034
for i in $(seq 12); do
    /snap/bin/lxd waitready --timeout=10 >/dev/null 2>&1 && break
done

/snap/bin/lxd init --auto
/snap/bin/lxc network create testbr0
/snap/bin/lxc network attach-profile testbr0 default eth0
