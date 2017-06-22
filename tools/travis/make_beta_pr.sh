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

# Make the pull request that triggers beta tests.

set -ev

script_path="$(dirname "$0")"
"$script_path/run_docker_container.sh" beta
docker exec -i beta apt install -y python3-git python3-github wget
docker exec -i beta ./tools/make_beta_pr.py
docker rm -f beta
