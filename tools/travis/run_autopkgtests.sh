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

# Run snapcraft autopkgtests.
# Arguments:
#   suite: The test suite to run.

set -ev

if [ "$#" -ne 2 ] ; then
    echo "Usage: "$0" <distro> <test>"
    exit 1
fi

distro="$1"
test="$2"

lxc="/snap/bin/lxc"

script_path="$(dirname "$0")"
project_path="$(readlink -f "$script_path/../..")"

"$script_path/setup_lxd.sh"
"$script_path/run_lxd_container.sh" test-runner
$lxc file push --recursive $project_path test-runner/root/
$lxc exec test-runner -- sh -c "apt remove --yes lxd lxd-client"
# Ignore the core install error as a workaround for
# - Setup snap "core" (2462) security profiles (cannot reload udev rules: exit status 2
$lxc exec test-runner -- sh -c "snap install core" || echo "ignored error"
$lxc exec test-runner -- sh -c "cd snapcraft && ./tools/travis/setup_lxd.sh"
$lxc exec test-runner -- sh -c "apt install --yes autopkgtest"
# Ignore the core install error as a workaround for
# - Setup snap "core" (2462) security profiles (cannot reload udev rules: exit status 2
$lxc exec test-runner -- sh -c "cd snapcraft && adt-run --testname $test snapcraft --setup-commands $script_path/setup_autopkgtests_ppa.sh -U --setup-commands \"apt install squashfuse && (snap install core || echo 'ignored error')\" --- lxd $distro"

$lxc stop test-runner
