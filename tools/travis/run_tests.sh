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

# Run snapcraft tests.
# Arguments:
#   suite: The test suite to run.

set -ev

if [ "$#" -ne 1 ] ; then
    echo "Usage: "$0" <test>"
    exit 1
fi

test="$1"

pattern="$2"

if [ "$test" = "static" ]; then
    dependencies="apt install -y python3-pip && python3 -m pip install -r requirements-devel.txt"
elif [ "$test" = "snapcraft/tests/unit" ]; then
    dependencies="apt install -y git bzr subversion mercurial libnacl-dev libsodium-dev libffi-dev libapt-pkg-dev libarchive-dev python3-pip squashfs-tools xdelta3 && python3 -m pip install -r requirements-devel.txt -r requirements.txt codecov && apt install -y python3-coverage"
elif [[ "$test" = "snapcraft/tests/integration"* ]]; then
    # snap install core exits with this error message:
    # - Setup snap "core" (2462) security profiles (cannot reload udev rules: exit status 2
    # but the installation succeeds, so we just ingore it.
    dependencies="apt install -y bzr curl git libnacl-dev libsodium-dev libffi-dev libapt-pkg-dev libarchive-dev mercurial python3-pip subversion squashfs-tools sudo snapd xdelta3 && python3 -m pip install -r requirements-devel.txt -r requirements.txt && (snap install core || echo 'ignored error') && sudo snap install snaps-cache/snapcraft-pr$TRAVIS_PULL_REQUEST.snap --dangerous --classic"
else
    echo "Unknown test suite: $test"
    exit 1
fi

script_path="$(dirname "$0")"
project_path="$(readlink -f "$script_path/../..")"

lxc="/snap/bin/lxc"

"$script_path/setup_lxd.sh"
"$script_path/run_lxd_container.sh" test-runner

$lxc file push --recursive $project_path test-runner/root/
$lxc exec test-runner -- sh -c "cd snapcraft && $dependencies"
$lxc exec test-runner -- sh -c "cd snapcraft && ./runtests.sh $test"

if [ "$test" = "snapcraft/tests/unit" ]; then
    # Report code coverage.
    $lxc exec test-runner -- sh -c "cd snapcraft && python3 -m coverage xml"
    $lxc exec test-runner -- sh -c "cd snapcraft && codecov --token=$CODECOV_TOKEN"
fi

$lxc stop test-runner
