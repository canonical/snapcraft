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
    if [[ "$#" -eq 0 ]] || [[ "$1" == "all" ]]; then
        export RUN_STATIC="true"
        export RUN_UNIT="true"
        export RUN_INTEGRATION="true"
        export RUN_STORE="true"
        export RUN_PLUGINS="true"
        export RUN_SNAPD="true"
        export RUN_SNAPS="true"
        export RUN_SPREAD="true"
    else
        if [ "$1" == "static" ] ; then
            export RUN_STATIC="true"
        elif [ "$1" == "unit" ] ; then
            export RUN_UNIT="true"
        elif [ "$1" == "integration" ] ; then
            export RUN_INTEGRATION="true"
        elif [ "$1" == "store" ] ; then
            export RUN_STORE="true"
        elif [ "$1" == "plugins" ] ; then
            export RUN_PLUGINS="true"
        elif [ "$1" == "snapd" ] ; then
            export RUN_SNAPD="true"
        elif [ "$1" == "snaps" ] ; then
            export RUN_SNAPS="true"
        # Temporary: backward compatibility until CI run the "snaps" target
        elif [ "$1" == "examples" ] ; then
            export RUN_SNAPS="true"
        elif [ "$1" == "spread" ] ; then
            export RUN_SPREAD="true"
        else
            echo "Not recognized option, should be one of all, static, unit, integration, store, plugins, snapd, snaps or spread"
            exit 1
        fi
    fi
}

python3 -m coverage 1>/dev/null 2>&1 && coverage="true"

run_static_tests(){
    SRC_PATHS="bin snapcraft integration_tests snaps_tests external_snaps_tests setup.py"
    python3 -m flake8 --max-complexity=10 $SRC_PATHS
}

run_unit_tests(){
    if [[ "$#" -lt 2 ]]; then
        pattern="test_*.py"
    else
        pattern=$2
    fi

    if [ ! -z "$coverage" ]; then
        python3 -m coverage erase
        python3 -m coverage run --branch --source snapcraft -m unittest discover -b -v -s snapcraft/tests -t . -p $pattern
    else
        python3 -m unittest discover -b -v -s snapcraft/tests -t . -p $pattern
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

run_store(){
    if [[ "$#" -lt 2 ]]; then
        pattern="test_*.py"
    else
        pattern=$2
    fi
    python3 -m unittest discover -b -v -s integration_tests/store -p $pattern
}

run_plugins(){
    if [[ "$#" -lt 2 ]]; then
        pattern="test_*.py"
    else
        pattern=$2
    fi
    python3 -m unittest discover -b -v -s integration_tests/plugins -p $pattern
}

run_snapd(){
    if [[ "$#" -lt 2 ]]; then
        pattern="test_*.py"
    else
        pattern=$2
    fi
    python3 -m unittest discover -b -v -s integration_tests/snapd -p $pattern
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

if [ ! -z "$RUN_STATIC" ] ; then
    run_static_tests
fi

if [ ! -z "$RUN_UNIT" ]; then
    run_unit_tests "$@"
fi

if [ ! -z "$RUN_INTEGRATION" ]; then
    run_integration "$@"
fi

if [ ! -z "$RUN_STORE" ]; then
    run_store "$@"
fi

if [ ! -z "$RUN_PLUGINS" ]; then
    run_plugins "$@"
fi

if [ ! -z "$RUN_SNAPD" ]; then
    run_snapd "$@"
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

if [ ! -z "$RUN_SPREAD" ]; then
    run_spread
fi

if [ ! -z "$RUN_UNIT" ]; then
    if [ ! -z "$coverage" ]; then
        python3 -m coverage report

        echo
        echo "Run 'python3-coverage html' to get a nice report"
        echo "View it by running 'x-www-browser htmlcov'"
        echo
    fi
fi

echo -e "\e[1;32mEverything passed\e[0m"
