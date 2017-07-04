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

if [ "$#" -lt 1 ] || [ "$#" -gt 2 ] ; then
    echo "Usage: "$0" <test> [PATTERN]"
    exit 1
fi

test="$1"

if [ "$#" -gt 1 ]; then
    pattern="$2"
fi

if [ "$test" = "static" ]; then
    dependencies="apt install -y python3-pip && python3 -m pip install -r requirements-devel.txt"
elif [ "$test" = "unit" ]; then
    dependencies="apt install -y git bzr subversion mercurial libnacl-dev libsodium-dev libffi-dev libapt-pkg-dev libarchive-dev python3-pip squashfs-tools xdelta3 && python3 -m pip install -r requirements-devel.txt -r requirements.txt codecov && apt install -y python3-coverage"
elif [ "$test" = "integration" ] || [ "$test" = "plugins" ] || [ "$test" = "store" ]; then
    dependencies="apt install -y bzr curl git libnacl-dev libsodium-dev libffi-dev libapt-pkg-dev libarchive-dev mercurial python3-pip subversion squashfs-tools sudo snapd xdelta3 && python3 -m pip install -r requirements-devel.txt -r requirements.txt"
else
    echo "Unknown test suite: $test"
    exit 1
fi

script_path="$(dirname "$0")"
"$script_path/run_docker_container.sh" test-runner
docker exec -i test-runner sh -c "$dependencies"
docker exec -i test-runner ./runtests.sh $test $pattern

if [ "$test" = "unit" ]; then
    # Report code coverage.
    docker exec -i test-runner sh -c "python3 -m coverage xml"
    docker exec -i test-runner sh -c "codecov --token=$CODECOV_TOKEN"
fi

docker rm -f test-runner
