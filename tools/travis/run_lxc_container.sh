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

# Start a lxc container to run jobs in travis.
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

#lxc=/snap/bin/lxc
lxc=lxc

echo "Start the LXC container."
sudo $lxc launch --ephemeral ubuntu:xenial "$name"
# This is likely needed to wait for systemd in the container to start and get
# an IP, configure DNS. First boot is always a bit slow because cloud-init
# needs to run too.
sleep 10

sudo $lxc config set "$name" environment.TRAVIS_COMMIT_RANGE "$TRAVIS_COMMIT_RANGE"
sudo $lxc config set "$name" environment.TEST_USER_EMAIL "$TEST_USER_EMAIL"
sudo $lxc config set "$name" environment.TEST_USER_PASSWORD "$TEST_USER_PASSWORD"
sudo $lxc config set "$name" environment.TEST_STORE "$TEST_STORE"
sudo $lxc config set "$name" environment.TEST_SNAP_WITH_TRACKS "$TEST_SNAP_WITH_TRACKS"
sudo $lxc config set "$name" environment.GITHUB_TEST_USER_NAME "$GITHUB_TEST_USER_NAME"
sudo $lxc config set "$name" environment.GITHUB_TEST_PASSWORD "$GITHUB_TEST_PASSWORD"
sudo $lxc config set "$name" environment.SNAPCRAFT_AUTOPKGTEST_SECRET "$SNAPCRAFT_AUTOPKGTEST_SECRET"
sudo $lxc config set "$name" environment.SNAPCRAFT_TEST_MOCK_MACHINE "$SNAPCRAFT_TEST_MOCK_MACHINE"
sudo $lxc config set "$name" environment.GH_TOKEN "$GH_TOKEN"
sudo $lxc config set "$name" environment.CODECOV_TOKEN "$CODECOV_TOKEN"
sudo $lxc config set "$name" environment.LC_ALL "C.UTF-8"

sudo $lxc config device add "$name" project_dir disk path="$project_path" source="$project_path"

sudo $lxc exec "$name" -- apt update
