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

# Run the CLA check.

set -ev

script_path="$(dirname "$0")"
"$script_path/run_docker_container.sh" autopkgtest-requests
docker exec -i autopkgtest-requests apt install -y python-launchpadlib
docker exec -i autopkgtest-requests ./tools/run_ppa_autopkgtests.py
docker rm -f autopkgtest-requests
