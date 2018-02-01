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

# Collect the autopkgtest PPA results, in a docker container.
# Arguments:
#   day: The day of the results, with format yyyymmdd

set -ev

if [ "$#" -ne 1 ] ; then
    echo "Usage: "$0" <day>"
    exit 1
fi

day="$1"

lxc="/snap/bin/lxc"

script_path="$(dirname "$0")"
project_path="$(readlink -f "$script_path/../..")"

"$script_path/setup_lxd.sh"
"$script_path/run_lxd_container.sh" autopkgtest-results
$lxc file push --recursive $project_path/tools autopkgtest-results/root/
$lxc exec autopkgtest-results -- sh -c "apt update && apt install --yes squashfuse"
# Ignore the core install error as a workaround for
# - Setup snap "core" (2462) security profiles (cannot reload udev rules: exit status 2
$lxc exec autopkgtest-results -- sh -c "snap install core" || echo "ignored error"
$lxc exec autopkgtest-results -- sh -c "/root/tools/collect_ppa_autopkgtests_results.py $1"
$lxc stop autopkgtest-results
