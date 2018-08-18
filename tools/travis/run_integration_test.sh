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

export lxc="/snap/bin/lxc"
export test_suite=$1
project_path="$(readlink -f "$(dirname "$0")/../..")"
export project_path


setup_lxd() {
    name="$1"
    image="$2"

    apt-get update
    apt-get install --yes snapd
    apt-get remove --yes lxd lxd-client

    snap install lxd --channel=3.0/stable

    # From LXD's CI.
    # shellcheck disable=SC2034
    for i in $(seq 12); do
        /snap/bin/lxd waitready --timeout=10 >/dev/null 2>&1 && break
    done

    /snap/bin/lxd init --auto

    echo "Starting the LXD container."
    $lxc launch --ephemeral "$image" "$name"
    # This is likely needed to wait for systemd in the container to start and get
    # an IP, configure DNS. First boot is always a bit slow because cloud-init
    # needs to run too.
    $lxc exec "$name" -- sh -c "for i in {1..50}; do ping -c1 www.ubuntu.com &> /dev/null && break; done"
    sleep 5

    $lxc config set "$name" environment.TRAVIS_COMMIT_RANGE "$TRAVIS_COMMIT_RANGE"
    $lxc config set "$name" environment.TEST_USER_EMAIL "$TEST_USER_EMAIL"
    $lxc config set "$name" environment.TEST_USER_PASSWORD "$TEST_USER_PASSWORD"
    $lxc config set "$name" environment.TEST_STORE "$TEST_STORE"
    $lxc config set "$name" environment.TEST_SNAP_WITH_TRACKS "$TEST_SNAP_WITH_TRACKS"
    $lxc config set "$name" environment.GITHUB_TEST_USER_NAME "$GITHUB_TEST_USER_NAME"
    $lxc config set "$name" environment.GITHUB_TEST_PASSWORD "$GITHUB_TEST_PASSWORD"
    $lxc config set "$name" environment.GH_TOKEN "$GH_TOKEN"
    $lxc config set "$name" environment.LC_ALL "C.UTF-8"
    $lxc config set "$name" environment.SNAPCRAFT_FROM_SNAP "1"

    $lxc exec "$name" -- apt update
}


echo "Going to run $test_suite on $TRAVIS_OS_NAME"

setup_lxd test-runner "${LXD_IMAGE:-ubuntu:16.04}"

$lxc file push "snaps-cache/snapcraft-pr$TRAVIS_PULL_REQUEST.snap" test-runner/root/ || true
$lxc file push --recursive "$project_path" test-runner/root/
$lxc exec test-runner -- sh -c "apt install -y bzr git libnacl-dev libssl-dev libsodium-dev libffi-dev libapt-pkg-dev mercurial python3-pip subversion sudo snapd"
$lxc exec test-runner -- sh -c "python3 -m pip install -r snapcraft/requirements-devel.txt -r snapcraft/requirements.txt"
$lxc exec test-runner -- sh -c "${SNAPCRAFT_INSTALL_COMMAND:-sudo snap install snapcraft-pr$TRAVIS_PULL_REQUEST.snap --dangerous --classic}"
$lxc exec test-runner -- sh -c "cd snapcraft && ./runtests.sh $test_suite"

$lxc stop test-runner
