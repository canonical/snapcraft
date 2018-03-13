# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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

import functools
import json
import logging
import os
import sys

import click

from snapcraft.internal.errors import SnapcraftEnvironmentError
from snapcraft.cli._errors import exception_handler
from snapcraft.internal import log


@click.group()
@click.option('--debug', '-d', is_flag=True)
def run(debug):
    """snapcraftctl is how snapcraft.yaml can communicate with snapcraft"""

    if debug:
        log_level = logging.DEBUG
    else:
        log_level = logging.INFO

    # Setup global exception handler (to be called for unhandled exceptions)
    sys.excepthook = functools.partial(exception_handler, debug=debug)

    # In an ideal world, this logger setup would be replaced
    log.configure(log_level=log_level)


@run.command()
def build():
    """Run the 'build' step of the calling part's plugin"""
    _call_function('build')


def _call_function(function_name, args=None):
    if not args:
        args = {}

    data = {
        'function': function_name,
        'args': args,
    }

    # We could load the FIFOs in `run` and shove them in the context, but
    # that's too early to error out if these variables aren't defined. Doing it
    # here allows one to run e.g. `snapcraftctl build --help` without needing
    # these variables defined, which is a win for usability.
    try:
        call_fifo = os.environ['SNAPCRAFTCTL_CALL_FIFO']
        feedback_fifo = os.environ['SNAPCRAFTCTL_FEEDBACK_FIFO']
    except KeyError as e:
        raise SnapcraftEnvironmentError(
            "{!s} environment variable must be defined. Note that this "
            "utility is only designed for use within a snapcraft.yaml".format(
                e)) from e

    with open(call_fifo, 'w') as f:
        f.write(json.dumps(data))
        f.flush()

    with open(feedback_fifo, 'r') as f:
        f.readline()
