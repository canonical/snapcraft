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

# Start a docker container to run jobs in travis.
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

echo "Start the docker container."
docker run \
  --name "$name" \
  --env TRAVIS_COMMIT_RANGE=$TRAVIS_COMMIT_RANGE \
  --env TEST_USER_EMAIL=$TEST_USER_EMAIL \
  --env TEST_USER_PASSWORD=$TEST_USER_PASSWORD \
  --env TEST_STORE=$TEST_STORE \
  --env TEST_SNAP_WITH_TRACKS=$TEST_SNAP_WITH_TRACKS \
  --env GITHUB_TEST_USER_NAME=$GITHUB_TEST_USER_NAME \
  --env GITHUB_TEST_PASSWORD=$GITHUB_TEST_PASSWORD \
  --env SNAPCRAFT_AUTOPKGTEST_SECRET=$SNAPCRAFT_AUTOPKGTEST_SECRET \
  --env SNAPCRAFT_TEST_MOCK_MACHINE=$SNAPCRAFT_TEST_MOCK_MACHINE \
  --env GH_TOKEN=$GH_TOKEN \
  --env CODECOV_TOKEN=$CODECOV_TOKEN \
  --env LC_ALL=C.UTF-8 \
  --volume "$project_path":"$project_path" \
  --workdir "$project_path" \
  --tty \
  --detach \
  ubuntu:xenial

echo "Set up the main archive."
docker exec -i "$name" sed -i s/archive.ubuntu.com/us.archive.ubuntu.com/g /etc/apt/sources.list
docker exec -i "$name" apt update
