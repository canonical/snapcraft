#!/bin/bash
# -*- Mode:sh; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015 Canonical Ltd
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

set -e

export PATH=$(pwd)/bin:$PATH
export PYTHONPATH=$(pwd):$PYTHONPATH

SRC_PATHS="bin snapcraft snapcraft/tests"

# These three checks could easily be done with flake8 in one shot if
# we had python3-flake8 provide flake8
# Ignore 501 (line-too-long)
pep8 $SRC_PATHS --ignore=E501

pyflakes3 $SRC_PATHS

# mccabe in 'warning' mode as we have high complexity
mccabe_list=
for unit in $(find snapcraft -type f -name '*.py')
do
  output=$(python3 -m mccabe --min 10 "$unit")
  [ -n "$output" ] && mccabe_list="- $unit:\n  $output\n$mccabe_list"
done

if [ -n "$mccabe_list" ]; then
  echo -e "\e[1;31mThe project has gotten complex\e[0m."
  echo "Here's the list of units exceeding 10:"
  echo -e "$mccabe_list"
fi

if which python3-coverage >/dev/null 2>&1; then
    python3-coverage erase
    python3-coverage run --branch --source snapcraft -m unittest
    mv .coverage .coverage.unit
else
    python3 -m unittest
fi

if [ -z "$SNAPCRAFT_TESTS_SKIP_PLAINBOX" ]; then
(
    # well, well, what can we do
    if ! which plainbox >/dev/null; then
        cat <<EOF

WARNING: no plainbox binary can be found
Please see the README for details how to install the plainbox package
for running the integration tests.

EOF
        exit 1
    fi

    if which python3-coverage >/dev/null 2>&1; then
        python3-coverage erase
        export PROJECT_PATH=$(pwd)
        export SNAPCRAFT=snapcraft-coverage
    fi

    # Go to the plainbox provider of snapcraft tests
    cd integration-tests
    ./runtests.sh
)
fi

if which python3-coverage >/dev/null 2>&1; then
    python3-coverage combine
    python3-coverage report

    echo
    echo "Run 'python3-coverage html' to get a nice report"
    echo "View it by running 'x-www-browser htmlcov'"
    echo
fi

echo -e "\e[1;32mEverything passed\e[0m"
