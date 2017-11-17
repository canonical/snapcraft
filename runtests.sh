#!/bin/bash
# -*- Mode:sh; indent-tabs-mode:nil; tab-width:4 -*-
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

set -e

export PATH=$(pwd)/bin:$PATH
export PYTHONPATH=$(pwd)${PYTHONPATH:+:$PYTHONPATH}

parseargs(){
    if [[ "$#" -eq 0 ]]; then
        echo "Usage: ./runtests.sh unit|snaps|<snapcraft-suite>"
        echo "<snapcraft-suite> can be snapcraft/tests, or any of its subdirectories."
        exit 1
    else
        if [ "$1" == "static" ] ; then
            run_static_tests
        elif [ "$1" == "snaps" ] ; then
            # shift to remove the test suite name and be able to pass the rest
            # to the snaps suite.
            shift
            run_snaps "$@"
        elif [ "$1" == "spread" ] ; then
            run_spread
        else
            run_snapcraft_tests "$@"
        fi
    fi
}

python3 -m coverage 1>/dev/null 2>&1 && coverage="true"

run_static_tests(){
    SRC_PATHS="bin snapcraft snaps_tests external_snaps_tests setup.py"
    python3 -m flake8 --max-complexity=10 $SRC_PATHS
}

run_snapcraft_tests(){
    if [[ ! -z "$coverage" ]] && [[ "$1" == "snapcraft/tests/unit"* ]]; then
        python3 -m coverage erase
        python3 -m coverage run --branch --source snapcraft -m unittest discover -b -v -s "$1" -t .
    else
        python3 -m unittest discover -b -v -s "$1" -t .
    fi
}

run_snaps(){
    python3 -m snaps_tests "$@"
}

run_spread(){
    TMP_SPREAD="$(mktemp -d)"

    export PATH=$TMP_SPREAD:$PATH
    ( cd "$TMP_SPREAD" && curl -s -O https://niemeyer.s3.amazonaws.com/spread-amd64.tar.gz && tar xzvf spread-amd64.tar.gz )

    spread -v linode:
}

parseargs "$@"

if [[ ! -z "$coverage" ]] && [[ "$1" == "snapcraft/tests/unit"* ]]; then
    python3 -m coverage report

    echo
    echo "Run 'python3-coverage html' to get a nice report"
    echo "View it by running 'x-www-browser htmlcov'"
    echo
fi

echo -e "\e[1;32mEverything passed\e[0m"
