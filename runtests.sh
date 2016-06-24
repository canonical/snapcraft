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
        export RUN_SNAPS="true"
    else
        if [ "$1" == "static" ] ; then
            export RUN_STATIC="true"
        elif [ "$1" == "unit" ] ; then
            export RUN_UNIT="true"
        elif [ "$1" == "integration" ] ; then
            export RUN_INTEGRATION="true"
        elif [ "$1" == "snaps" ] ; then
            export RUN_SNAPS="true"
        # Temporary: backward compatibility until CI run the "snaps" target
        elif [ "$1" == "examples" ] ; then
            export RUN_SNAPS="true"
        else
            echo "Not recognized option, should be one of all, static, unit, integration or snaps"
            exit 1
        fi
    fi
}

run_static_tests(){
    SRC_PATHS="bin snapcraft snapcraft/tests snaps_tests"
    python3 -m flake8 --max-complexity=10 $SRC_PATHS
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
    if [[ "$#" -lt 2 ]]; then
        pattern="test_*.py"
    else
        pattern=$2
    fi
    python3 -m unittest discover -b -v -s integration_tests -p $pattern
}

run_snaps(){
    python3 -m snaps_tests "$@"
}

parseargs "$@"

if [ ! -z "$RUN_STATIC" ] ; then
    run_static_tests
fi

if [ ! -z "$RUN_UNIT" ]; then
    run_unit_tests
fi

if [ ! -z "$RUN_INTEGRATION" ]; then
    run_integration "$@"
fi

if [ ! -z "$RUN_SNAPS" ]; then
    if [ "$1" == "snaps" ] ; then
        # shift to remove the test suite name and be able to pass the rest
        # to the snaps suite.
        shift
    fi
    ## Temporary: backward compatibility until CI run the "snaps" target
    if [ "$1" == "examples" ] ; then
        # shift to remove the test suite name and be able to pass the rest
        # to the snaps suite.
        shift
    fi
    ##
    run_snaps "$@"
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
