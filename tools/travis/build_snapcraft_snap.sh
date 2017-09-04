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

PULL_REQUEST="$1"

script_path="$(dirname "$0")"
project_path="$(readlink -f "$script_path/../..")"

lxc="/snap/bin/lxc"

"$script_path/setup_lxd.sh"
"$script_path/run_lxd_container.sh" snap-builder
$lxc file push --recursive $project_path snap-builder/root/
# TODO use the stable snap once it's published.
$lxc exec snap-builder -- sh -c "apt install squashfuse && snap install snapcraft --candidate --classic"
$lxc exec snap-builder -- sh -c "cd snapcraft && /snap/bin/snapcraft snap --output snapcraft-branch.snap"
[ "$PULL_REQUEST" != "false" ] && $lxc exec snap-builder -- sh -c "cd snapcraft && /snap/bin/snapcraft push snapcraft-branch.snap --release edge/pr-$PULL_REQUEST"
$lxc stop snap-builder
