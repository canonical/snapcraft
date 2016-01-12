# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
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

import argparse
import logging
import sys

from testtools import run

from examples_tests import tests


def main():
    logging.basicConfig(level=logging.INFO)

    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--skip-install',
        help=('skip the tests that install the example snaps into a snappy '
              'virtual machine.'),
        action='store_true')
    parser.add_argument(
        '--ip',
        help=('IP of the testbed. If no IP is passed, a virtual machine will '
              'be created for the test.'))
    parser.add_argument(
        '--port',
        help=('SSH port of the testbed. Defaults to use port 22.'))
    parser.add_argument(
        '--filter',
        help=('a regular expression to filter the examples to test.'))
    args = parser.parse_args()
    if args.skip_install:
        tests.config['skip-install'] = True
    if args.ip:
        tests.config['ip'] = args.ip
    if args.port:
        tests.config['port'] = args.port
    if args.filter:
        tests.config['filter'] = args.filter

    # Strip all the command line arguments, so unittest does not handle them
    # again.
    argv = [sys.argv[0]]
    run.TestProgram('examples_tests.tests', verbosity=2, argv=argv)


if __name__ == '__main__':
    main()
