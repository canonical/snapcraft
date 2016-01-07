#!/bin/bash
# -*- Mode:sh; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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

set -ex

apt-get update -qq
apt-get install -qq build-essential dpkg-dev pyflakes python3-apt python3-docopt python3-coverage python3-fixtures python3-jsonschema python3-mccabe python3-pip python3-pep8 python3-requests python3-testscenarios python3-testtools python3-yaml python3-lxml squashfs-tools

./runtests.sh $TEST_SUITE

python3 -m pip install coveralls

COVERALLS_REPO_TOKEN=$COVERALLS_TOKEN coveralls
