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

# Trigger the PPA autopkgtests, in a docker container.

set -ev

lxc="/snap/bin/lxc"

script_path="$(dirname "$0")"
project_path="$(readlink -f "$script_path/../..")"

"$script_path/setup_lxd.sh"
"$script_path/run_lxd_container.sh" test-trigger
$lxc file push --recursive $project_path/tools test-trigger/root/
$lxc exec test-trigger -- sh -c "apt install --yes python python-launchpadlib"
$lxc exec test-trigger -- sh -c "/root/tools/run_ppa_autopkgtests.py"

$lxc stop test-trigger
