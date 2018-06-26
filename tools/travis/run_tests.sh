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

argc="$#"
if [ "$argc" -gt 2 ]; then
    echo "Usage: $0 <test> [<use-run>]"
    exit 1
fi

test_suite="$1"
use_run="$2"


if [ "$TRAVIS_OS_NAME" = "osx" ] && [ -z "$SNAPCRAFT_FROM_BREW" ]; then
    echo "osx tests require snapcraft from brew to be setup"
    exit 0
fi

if [ "$TRAVIS_OS_NAME" = "osx" ]; then
    brew upgrade python
    # readlink -f doesn't work on osx.
    brew upgrade coreutils || brew install coreutils
    python3 ./tools/brew_install_from_source.py
    python3 -m pip install -r requirements.txt
    python3 -m pip install -r requirements-devel.txt
elif [ "$test_suite" = "static" ]; then
    dependencies="apt install -y python3-pip shellcheck && python3 -m pip install -r requirements-devel.txt"
elif [ "$test_suite" = "tests/unit" ]; then
    dependencies="apt install -y git bzr subversion mercurial rpm2cpio p7zip-full libnacl-dev libssl-dev libsodium-dev libffi-dev libapt-pkg-dev python3-pip squashfs-tools xdelta3 && python3 -m pip install -r requirements-devel.txt -r requirements.txt codecov && apt install -y python3-coverage"
elif [[ "$test_suite" = "tests/integration"* || "$test_suite" = "tests.integration"* ]]; then
    # TODO remove the need to install the snapcraft dependencies due to nesting
    #      the tests in the snapcraft package
    dependencies="apt install -y bzr git libnacl-dev libssl-dev libsodium-dev libffi-dev libapt-pkg-dev mercurial python3-pip subversion sudo snapd && python3 -m pip install -r requirements-devel.txt -r requirements.txt && (snap install core || echo 'ignored error') && ${SNAPCRAFT_INSTALL_COMMAND:-sudo snap install snaps-cache/snapcraft-pr$TRAVIS_PULL_REQUEST.snap --dangerous --classic}"
elif [ "$test_suite" = "spread" ]; then
    cp "$TRAVIS_BUILD_DIR/snaps-cache/snapcraft-pr$TRAVIS_PULL_REQUEST.snap" .
else
    echo "Unknown test suite: $test_suite"
    exit 1
fi

script_path="$(dirname "$0")"

echo "Going to run $test_suite on $TRAVIS_OS_NAME"
if [ "$TRAVIS_OS_NAME" = "osx" ]; then
    project_path="$(greadlink -f "$script_path/../..")"
    ./runtests.sh "$test_suite" "$use_run"
elif [ "$test_suite" = "spread" ]; then
    ./runtests.sh "$test_suite"
else
    project_path="$(readlink -f "$script_path/../..")"
    lxc="/snap/bin/lxc"

    "$script_path/setup_lxd.sh"
    "$script_path/run_lxd_container.sh" test-runner

    $lxc file push --recursive "$project_path" test-runner/root/
    $lxc exec test-runner -- sh -c "cd snapcraft && ./tools/travis/setup_lxd.sh"
    $lxc exec test-runner -- sh -c "cd snapcraft && $dependencies"
    $lxc exec test-runner -- sh -c "cd snapcraft && ./runtests.sh $test_suite $use_run"

    # By checking for SNAPCRAFT_TEST_MOCK_MACHINE we ensure coverage results are uploaded
    # only once.
    if [ "$test_suite" = "tests/unit" ] && [ -z "$SNAPCRAFT_TEST_MOCK_MACHINE" ]; then
        # Report code coverage.
        $lxc exec test-runner -- sh -c "cd snapcraft && python3 -m coverage xml"
        $lxc exec test-runner -- sh -c "cd snapcraft && codecov --token=$CODECOV_TOKEN"
    fi

    $lxc stop test-runner
fi
