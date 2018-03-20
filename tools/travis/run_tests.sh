#!/bin/bash
#
# Copyright (C) 2015-2018 Canonical Ltd
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

if [ "$#" -lt 1 ] ; then
    echo "Usage: "$0" <test> [native|<image>]"
    exit 1
fi

test="$1"
image="${2:-ubuntu:xenial}"

if [ "$test" = "static" ]; then
    dependencies="apt install -y python3-pip && python3 -m pip install -r requirements-devel.txt"
elif [ "$test" = "tests/unit" ]; then
    dependencies="apt install -y git bzr subversion mercurial rpm2cpio p7zip-full libnacl-dev libsodium-dev libffi-dev libapt-pkg-dev python3-pip squashfs-tools xdelta3 && python3 -m pip install -r requirements-devel.txt -r requirements.txt codecov && apt install -y python3-coverage"
elif [[ "$test" = "tests/integration"* || "$test" = "tests.integration"* ]]; then
    # TODO remove the need to install the snapcraft dependencies due to nesting
    #      the tests in the snapcraft package
    # snap install core exits with this error message:
    # - Setup snap "core" (2462) security profiles (cannot reload udev rules: exit status 2
    # but the installation succeeds, so we just ingore it.
    dependencies="apt install -y bzr git libnacl-dev libsodium-dev libffi-dev libapt-pkg-dev mercurial python3-pip subversion sudo snapd && python3 -m pip install -r requirements-devel.txt -r requirements.txt && (snap install core || echo 'ignored error') && ${SNAPCRAFT_INSTALL_COMMAND:-sudo snap install snaps-cache/snapcraft-pr$TRAVIS_PULL_REQUEST.snap --dangerous --classic}"
else
    echo "Unknown test suite: $test"
    exit 1
fi

script_path="$(dirname "$0")"
project_path="$(readlink -f "$script_path/../..")"

if [ "$image" = "native" ]; then
    if grep 14.04 /etc/os-release; then
        sed -i 's/apt_1.1.0~beta1build1/apt_0.9.3.5ubuntu2/' requirements.txt
        sudo add-apt-repository -y ppa:chris-lea/libsodium
        sudo apt update
    fi
    run(){ sh -c "SNAPCRAFT_FROM_SNAP=1 $1"; }
elif [ "$image" = "ubuntu:xenial" ]; then
    lxc="/snap/bin/lxc"
    "$script_path/setup_lxd.sh"
    "$script_path/run_lxd_container.sh" test-runner
    $lxc file push --recursive $project_path test-runner/root/
    run(){ $lxc exec test-runner -- sh -c "cd snapcraft && $1"; }
    run "./tools/travis/setup_lxd.sh"
else
    echo "Invalid image: $image"
    exit 1
fi

run "$dependencies"
run "./runtests.sh $test"
if [ "$test" = "snapcraft/tests/unit" ]; then
    # Report code coverage.
    run "python3 -m coverage xml"
    run "codecov --token=$CODECOV_TOKEN"
fi

if [ "$image" != "native" ]; then
    $lxc stop test-runner
fi
