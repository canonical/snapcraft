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
export LC_ALL=en_US.UTF-8

parseargs(){
    if [[ "$#" -eq 0 ]] || [[ "$1" == "all" ]]; then
        export RUN_STATIC="true"
        export RUN_UNIT="true"
        export RUN_INTEGRATION="true"
        export RUN_EXAMPLES="true"
    else
        if [ "$1" == "static" ] ; then
            export RUN_STATIC="true"
        elif [ "$1" == "unit" ] ; then
            export RUN_UNIT="true"
        elif [ "$1" == "integration" ] ; then
            export RUN_INTEGRATION="true"
        elif [ "$1" == "examples" ] ; then
            export RUN_EXAMPLES="true"
        else
            echo "Not recognized option, should be one of all, static, unit, integration or examples"
            exit 1
        fi
    fi
}

run_static_tests(){
    SRC_PATHS="bin snapcraft snapcraft/tests examples_tests"

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
}

run_unit_tests(){
    if which python3-coverage >/dev/null 2>&1; then
        python3-coverage erase
        python3-coverage run --branch --source snapcraft -m unittest discover -s snapcraft -t .
        mv .coverage .coverage.unit
    else
        python3 -m unittest discover -s snapcraft -t .
    fi
}

run_integration(){
    if which python3-coverage >/dev/null 2>&1; then
        python3-coverage erase
        export SNAPCRAFT=snapcraft-coverage
    fi

    python3 -m unittest discover -s integration_tests
}

run_examples(){
    if which python3-coverage >/dev/null 2>&1; then
        python3-coverage erase
        export SNAPCRAFT=snapcraft-coverage
    fi

    python3 -m examples_tests "$@"
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
