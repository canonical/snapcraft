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

parseargs(){
    if [[ "$#" -eq 0 ]] || [[ "$1" == "all" ]]; then
        export RUN_UNIT="true"
        export RUN_PLAINBOX="true"
        export PLAINBOX_TEST_PLANS="normal"
    else
        if [ "$1" == "unit" ] ; then
            export RUN_UNIT="true"
        elif [ "$1" == "plainbox" ] ; then
            export RUN_PLAINBOX="true"
            if [ "$#" -gt 1 ]; then
                export PLAINBOX_TEST_PLANS="$2"
            else
                export PLAINBOX_TEST_PLANS="normal"
            fi
        else
            echo "Not recognized option, should be one of all, unit or plainbox"
            exit 1
        fi
    fi
}

run_unit_tests(){
    SRC_PATHS="bin snapcraft snapcraft/tests"

    # These three checks could easily be done with flake8 in one shot if
    # we had python3-flake8 provide flake8
    python3 -m pep8 $SRC_PATHS

    python3 -m pyflakes $SRC_PATHS

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
}

run_plainbox(){
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
    pushd integration-tests
    ./runtests.sh $PLAINBOX_TEST_PLANS
    popd
}

parseargs "$@"

if [ ! -z "$RUN_UNIT" ]; then
    run_unit_tests
fi

if [ -z "$SNAPCRAFT_TESTS_SKIP_PLAINBOX" ] && [ ! -z "$RUN_PLAINBOX" ] ; then
    run_plainbox
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
