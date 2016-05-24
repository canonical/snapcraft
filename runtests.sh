#!/bin/bash
# -*- Mode:sh; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2016 Canonical Ltd
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
        export RUN_STATIC="true"
        export RUN_UNIT="true"
        export RUN_INTEGRATION="true"
        export RUN_DEMOS="true"
    else
        if [ "$1" == "static" ] ; then
            export RUN_STATIC="true"
        elif [ "$1" == "unit" ] ; then
            export RUN_UNIT="true"
        elif [ "$1" == "integration" ] ; then
            export RUN_INTEGRATION="true"
        elif [ "$1" == "examples" ] ; then
            export RUN_EXAMPLES="true"
        elif [ "$1" == "demos" ] ; then
            export RUN_DEMOS="true"
        else
            echo "Not recognized option, should be one of all, static, unit, integration, examples or demos"
            exit 1
        fi
    fi
}

run_static_tests(){
    SRC_PATHS="bin snapcraft snapcraft/tests demos_tests"
    python3 -m flake8 $SRC_PATHS

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
        exit 1
    fi
}

run_unit_tests(){
    if which python3-coverage >/dev/null 2>&1; then
        python3 -m coverage erase
        python3 -m coverage run --branch --source snapcraft -m unittest discover -b -v -s snapcraft -t .
    else
        python3 -m unittest discover -b -v -s snapcraft -t .
    fi
}

run_integration(){
    python3 -m unittest discover -b -v -s integration_tests
}

run_examples(){
    python3 -m examples_tests "$@"
}

run_demos(){
    python3 -m demos_tests "$@"
}

parseargs "$@"

if [ ! -z "$RUN_STATIC" ] ; then
    run_static_tests
fi

if [ ! -z "$RUN_UNIT" ]; then
    run_unit_tests
fi

if [ ! -z "$RUN_INTEGRATION" ]; then
    run_integration
fi
if [ ! -z "$RUN_EXAMPLES" ]; then
    if [ "$1" == "examples" ] ; then
        # shift to remove the test suite name and be able to pass the rest
        # to the examples suite.
        shift
    fi
    run_examples "$@"
    # Temporary: until CI add "runtests demo" target
    run_demos "$@"
fi

if [ ! -z "$RUN_DEMOS" ]; then
    if [ "$1" == "demos" ] ; then
        # shift to remove the test suite name and be able to pass the rest
        # to the demos suite.
        shift
    fi
    run_demos "$@"
fi

if [ ! -z "$RUN_UNIT" ]; then
    if which python3-coverage >/dev/null 2>&1; then
        python3 -m coverage report

        echo
        echo "Run 'python3-coverage html' to get a nice report"
        echo "View it by running 'x-www-browser htmlcov'"
        echo
    fi
fi

echo -e "\e[1;32mEverything passed\e[0m"
