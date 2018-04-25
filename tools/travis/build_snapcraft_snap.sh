#!/bin/bash
#
# Copyright (C) 2015-2017 Canonical Ltd
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

# Build the snapcraft snap.

set -ev

if [ "$TRAVIS_OS_NAME" = "osx" ]; then
    echo "We do not build the snapcraft snap on osx"
    exit 0
fi

script_path="$(dirname "$0")"
project_path="$(readlink -f "$script_path/../..")"

lxc="/snap/bin/lxc"

"$script_path/setup_lxd.sh"
"$script_path/run_lxd_container.sh" snap-builder
# Workaround for
# - Setup snap "core" (2462) security profiles (cannot reload udev rules: exit status 2
$lxc exec snap-builder -- sh -c "snap install core" || echo "ignored error"
$lxc file push --recursive $project_path snap-builder/root/
# TODO use the stable snap once it's published.
$lxc exec snap-builder -- sh -c "snap install snapcraft --candidate --classic"
$lxc exec snap-builder -- sh -c "cd snapcraft && /snap/bin/snapcraft snap --output snapcraft-pr$TRAVIS_PULL_REQUEST.snap"
# Pull the snap from the container to save it into the cache.
mkdir -p "$TRAVIS_BUILD_DIR/snaps-cache"
$lxc file pull "snap-builder/root/snapcraft/snapcraft-pr$TRAVIS_PULL_REQUEST.snap" "$TRAVIS_BUILD_DIR/snaps-cache/"
# Send the snap to transfer.sh
$lxc exec snap-builder -- sh -c "snap install transfer"
# XXX The transfer is getting stuck in master: https://bugs.launchpad.net/snapcraft/+bug/1738461
# For now, set a timeout and don't fail if the snap couldn't be uploaded.
# --elopio - 20171215
$lxc exec snap-builder -- sh -c "timeout 180 transfer snapcraft/snapcraft-pr$TRAVIS_PULL_REQUEST.snap || true"
$lxc stop snap-builder
