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
# GNU General Public License for more details.f-1
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

set -e

usage() {
    echo "Usage: "
    echo "    ./runtests.sh static"
    echo "    ./runtests.sh tests/unit [<use-run>]"
    echo "    ./runtests.sh tests/integration[/<test-suite>]"
    echo "    ./runtests.sh spread"
    echo ""
    echo "<test-suite> can be one of: $(find tests/integration/ -mindepth 1 -maxdepth 1 -type d ! -name __pycache__ | tr '\n' ' ')"
    echo "<use-run> makes use of run instead of discover to run the tests"
}

run_static_tests() {
    echo "Running black"
    black --check --diff .

    echo "Running flake8"
    python3 -m flake8 .

    echo "Running mypy"
    mypy .

    echo "Running codespell"
    codespell -S "*.tar,*.xz,*.zip,*.bz2,*.7z,*.gz,*.deb,*.rpm,*.snap,*.gpg,*.pyc,*.png,*.ico,*.jar,changelog,.git,.hg,.mypy_cache,.tox,.venv,_build,buck-out,__pycache__,build,dist,.vscode,parts,stage,prime,test_appstream.py,./snapcraft.spec" -q4 -L keyserver

    echo "Running shellcheck"
    # Need to skip 'demos/gradle/gradlew' as it wasn't written by us and has
    # tons of issues.
    find . \( -name .git -o -name gradlew \) -prune -o -print0 | xargs -0 file -N | awk -F": " '$2~/shell.script/{print $1}' | xargs shellcheck

    echo "Running shellcheck inside spread yaml"
    ./tools/spread-shellcheck.py spread.yaml tests/spread/
}

run_snapcraft_tests(){
    test_suite="$1"
    use_run="$2"

    if [[ -n "$use_run" ]]; then
        python3 -m unittest -b -v run "$test_suite"
    elif [[ -n "$(command -v coverage)" ]] && [[ "$test_suite" == "tests/unit"* ]]; then
        # Run with coverage results, if available.
        python3 -m coverage erase
        python3 -m coverage run --branch --source snapcraft -m unittest discover -b -v -s "$test_suite" -t .

        coverage report
        echo
        echo "Run 'python3-coverage html' to get a nice report"
        echo "View it by running 'x-www-browser htmlcov'"
        echo
    else
        python3 -m unittest discover -b -v -s "$test_suite" -t .
    fi
}

run_spread(){
    TMP_SPREAD="$(mktemp -d)"
    curl -s https://niemeyer.s3.amazonaws.com/spread-amd64.tar.gz | tar xzv -C "$TMP_SPREAD"

    if [[ "$#" -eq 0 ]]; then
        "$TMP_SPREAD/spread" -v lxd:
    else
        "$TMP_SPREAD/spread" -v "$@"
    fi
}

if [[ "$#" -eq 0 ]]; then
    usage
    exit 1
fi

test_suite=$1
shift

if [[ "$test_suite" == "static" ]]; then
    run_static_tests
elif [[ "$test_suite" == "spread" ]]; then
    run_spread "$@"
elif [[ "$test_suite" == "-h" ]] || [[ "$test_suite" == "help" ]]; then
    usage
    exit 0
else
    run_snapcraft_tests "$test_suite" "$@"
fi

echo -e '\e[1;32mEverything passed\e[0m'
