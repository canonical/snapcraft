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
if which systemctl > /dev/null; then
    # Workaround for https://bugs.launchpad.net/snapd/+bug/1709536
    mkdir -p /systemd/system/snapd.service.d
    printf '[Service]\nNice=0\n' > /systemd/system/snapd.service.d/override.conf
    systemctl daemon-reload
    systemctl start snapd
    echo hola
    cat /systemd/system/snapd.service.d/override.conf
fi

# Use edge because the feature to copy links to the container has not yet been
# released to stable:
# https://github.com/lxc/lxd/commit/004e7c361e1d914795d3ba7582654622e32ff193
snap install lxd --edge
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
