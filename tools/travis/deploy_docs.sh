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

# Deploy the API documentation to GitHub pages.

set -ev

script_path="$(dirname "$0")"
"$script_path/run_docker_container.sh" docs
docker exec -i docs apt install libnacl-dev libsodium-dev libffi-dev libapt-pkg-dev libarchive-dev git make python3-pip python3-sphinx -y && python3 -m pip install -r requirements-devel.txt -r requirements.txt
docker exec -i docs ./tools/gen_api_docs.sh && ./tools/push_api_gh_pages.sh
docker rm -f docs
