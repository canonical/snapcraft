#!/usr/bin/python3
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

"""
snapcraft

Usage:
  snapcraft-examples [options] [<directory>]

Options:
  -h --help                             show this help message band exit
  -v --version                          show program version and exit
  -d --debug                            print debug information while executing
                                        (including backtraces)

Calling snapcraft without a directory will ask you for which directory to
write to.

For getting started on examples with this command, visit our documentation at:
https://snapcraft.io
"""

from glob import glob
import logging
import os
import pkg_resources
import readline
import shutil
import sys
import textwrap

from docopt import docopt

from snapcraft.internal import log
from snapcraft.internal.common import get_examplesdir


logger = logging.getLogger(__name__)


def _get_version():
    try:
        return pkg_resources.require('snapcraft')[0].version
    except pkg_resources.DistributionNotFound:
        return 'devel'


def main(argv=None):
    args = docopt(__doc__, version=_get_version(), argv=argv)

    # Default log level is INFO unless --debug is specified
    log_level = logging.INFO
    if args['--debug']:
        log_level = logging.DEBUG

    log.configure(log_level=log_level)

    dest_dir = args['<directory>']
    while not dest_dir:
        readline.set_startup_hook(lambda:
                                  readline.insert_text("snapcraft-examples"))
        dest_dir = input("Create examples in: ")

    dest_dir = os.path.abspath(dest_dir)

    try:
        return copy(dest_dir)
    except Exception as e:
        if args['--debug']:
            raise
        sys.exit(textwrap.fill(str(e)))


def copy(dest_dir):

    logger.debug("Copying examples to {}".format(dest_dir))
    # If dest_dir doesn't exist, we dump all examples in it.
    # If it does exist, we dump them into a subdirectory
    try:
        shutil.copytree(get_examplesdir(), dest_dir)
    except FileExistsError:
        # don't event try to copy if the dest isn't a directory
        if not os.path.isdir(dest_dir):
            raise NotADirectoryError("{} is a file, can't be used as a "
                                     "destination".format(dest_dir))
        dest_dir = os.path.join(dest_dir, "snapcraft-examples")
        shutil.copytree(get_examplesdir(), dest_dir)

    first_example = glob(os.path.join(dest_dir, "01-*"))[0]
    print("You can start now getting familiar with snapcraft by "
          "looking at {}. \n Instructions are available at "
          "https://snapcraft.io/create-snaps".format(first_example))

    return 0


if __name__ == '__main__':  # pragma: no cover
    main()                  # pragma: no cover
