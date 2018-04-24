#!/bin/bash
# -*- Mode:sh; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2018 Canonical Ltd
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

printhelp(){
    echo "Usage: "
    echo "    ./runtests.sh static"
    echo "    ./runtests.sh tests/unit [<use-run>]"
    echo "    ./runtests.sh tests/integration[/<test-suite>]"
    echo "    ./runtests.sh snaps"
    echo ""
    echo "<test-suite> can be: $(ls tests/integration| grep '^[a-z].*' | tr '\n' ' ')"
    echo "<use-run> makes use of run instead of discover to run the tests"
}

test_suite="$1"
use_run="$2"

parseargs(){
    if [[ "$#" -eq 0 ]]; then
        printhelp
	exit 1
    else
        if [ "$test_suite" == "static" ] ; then
            run_static_tests
        elif [ "$test_suite" == "snaps" ] ; then
            # shift to remove the test suite name and be able to pass the rest
            # to the snaps suite.
            shift
            run_snaps "$@"
        elif [ "$test_suite" == "spread" ] ; then
            run_spread
        else
            run_snapcraft_tests "$@"
        fi
    fi
}

python3 -m coverage 1>/dev/null 2>&1 && coverage="true"

run_static_tests(){
    SRC_PATHS="bin external_snaps_tests setup.py snapcraft snaps_tests tests"
    python3 -m flake8 --max-complexity=10 $SRC_PATHS
    codespell -S "*.tar,*.xz,*.zip,*.bz2,*.7z,*.gz,*.deb,*.rpm,*.snap,*.gpg,*.pyc,*.png,*.ico,*.jar,./.git,changelog" -q4
    mypy --ignore-missing-imports --follow-imports=silent -p snapcraft
}

run_snapcraft_tests(){
    if [[ ! -z "$use_run" ]]; then
        python3 -m unittest -b -v run "$test_suite"
    elif [[ ! -z "$coverage" ]] && [[ "$test_suite" == "tests/unit"* ]]; then
        python3 -m coverage erase
        python3 -m coverage run --branch --source snapcraft -m unittest discover -b -v -s "$test_suite" -t .
    else
        python3 -m unittest discover -b -v -s "$test_suite" -t .
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

if [ "$test_suite" == "-h" ] || [ "$test_suite" == "--help" ]; then
    printhelp
    exit 0
fi

parseargs "$@"

if [[ ! -z "$coverage" ]] && [[ "$test_suite" == "tests/unit"* ]]; then
    python3 -m coverage report

    echo
    echo "Run 'python3-coverage html' to get a nice report"
    echo "View it by running 'x-www-browser htmlcov'"
    echo
fi

echo -e "\e[1;32mEverything passed\e[0m"
