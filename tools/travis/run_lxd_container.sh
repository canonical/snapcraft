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

# Start a LXD container to run jobs in travis.
# Arguments:
#   name: The name of the container.

set -ev

if [ "$#" -lt 1 ]; then
    echo "Usage: "$0" <name>"
    exit 1
fi

script_path="$(dirname "$0")"
project_path="$(readlink -f "$script_path/../..")"
name="$1"
image="$LXD_IMAGE"
[ -z "$image" ] && image="ubuntu:xenial"

lxc="/snap/bin/lxc"

echo "Starting the LXD container."
$lxc launch --ephemeral --config security.nesting=true "$image" "$name"
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
$lxc config set "$name" environment.SNAPCRAFT_AUTOPKGTEST_SECRET "$SNAPCRAFT_AUTOPKGTEST_SECRET"
$lxc config set "$name" environment.GH_TOKEN "$GH_TOKEN"
$lxc config set "$name" environment.CODECOV_TOKEN "$CODECOV_TOKEN"
$lxc config set "$name" environment.SNAPCRAFT_AUTOPKGTEST_COOKIE "$SNAPCRAFT_AUTOPKGTEST_COOKIE"
$lxc config set "$name" environment.GH_TOKEN_PPA_AUTOPKGTEST_RESULTS "$GH_TOKEN_PPA_AUTOPKGTEST_RESULTS"
$lxc config set "$name" environment.LC_ALL "C.UTF-8"
$lxc config set "$name" environment.SNAPCRAFT_FROM_SNAP "1"
if [ -n "$SNAPCRAFT_TEST_MOCK_MACHINE" ]; then
    $lxc config set "$name" environment.SNAPCRAFT_TEST_MOCK_MACHINE "$SNAPCRAFT_TEST_MOCK_MACHINE"
fi

$lxc exec "$name" -- apt update

# We install squashfuse here as it is only relevant when running the tests in a container
$lxc exec "$name" -- apt install squashfuse
