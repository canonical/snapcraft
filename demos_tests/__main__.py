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

"""Snapcraft demos tests.

Usage:
  demos_tests [--skip-install] [--ip IP_OR_HOSTNAME]
              [--port PORT_NUMBER] [--filter REGEXP]
              [--subunit]

Options:
  --skip-install       skip the tests that install the demo snaps into a
                       snapp test bed.
  --ip IP_OR_HOSTNAME  IP of the testbed. If no IP is passed, a virtual
                       machine will be created for the test.
  --port PORT_NUMBER   SSH port of the testbed. Defaults to use port 22.
  --filter REGEXP      a regular expression to filter the demos to test.
  --subunit            generate subunit results.

"""

import logging
import os
import sys

import docopt

import demos_tests


def main():
    logging.basicConfig(level=logging.INFO)

    arguments = docopt.docopt(__doc__)

    demos_tests.config['skip-install'] = arguments['--skip-install']
    demos_tests.config['ip'] = arguments['--ip']
    demos_tests.config['port'] = arguments['--port']
    demos_tests.config['filter'] = arguments['--filter']

    if arguments['--subunit']:
        from subunit import run
        runner = run.SubunitTestProgram
        stdout = open('results.subunit', 'wb')
        test_runner = run.SubunitTestRunner
        exit = False
    else:
        from testtools import run
        runner = run.TestProgram
        stdout = None
        test_runner = None
        exit = True

    # Strip all the command line arguments, so the test runner does not handle
    # them again.
    argv = [sys.argv[0]]
    argv.append('discover')
    argv.append(os.path.dirname(__file__).split(os.path.sep)[-1])
    runner(module=None, verbosity=2, stdout=stdout,
           testRunner=test_runner, exit=exit, argv=argv)


if __name__ == '__main__':
    main()
