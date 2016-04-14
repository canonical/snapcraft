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
  snapcraft [options] [--enable-geoip --no-parallel-build]
  snapcraft [options] init
  snapcraft [options] pull [<part> ...]  [--enable-geoip]
  snapcraft [options] build [<part> ...] [--no-parallel-build]
  snapcraft [options] stage [<part> ...]
  snapcraft [options] strip [<part> ...]
  snapcraft [options] clean [<part> ...] [--step <step>]
  snapcraft [options] snap [<directory> --output <snap-file>]
  snapcraft [options] cleanbuild
  snapcraft [options] login
  snapcraft [options] logout
  snapcraft [options] register-name <snap-name>
  snapcraft [options] upload <snap-file>
  snapcraft [options] list-plugins
  snapcraft [options] help (topics | <plugin> | <topic>) [--devel]
  snapcraft (-h | --help)
  snapcraft --version

Options:
  -h --help                             show this help message and exit
  -v --version                          show program version and exit
  -d --debug                            print debug information while executing
                                        (including backtraces)
  --target-arch ARCH                    EXPERIMENTAL: sets the target
                                        architecture. Very few plugins support
                                        this.

Options specific to pulling:
  --enable-geoip         enables geoip for the pull step if stage-packages
                         are used.

Options specific to building:
  --no-parallel-build                   use only a single build job per part
                                        (the default number of jobs per part is
                                        equal to the number of CPUs)

Options specific to cleaning:
  -s <step>, --step <step>              only clean the specified step and those
                                        that depend upon it. <step> can be one
                                        of: pull, build, stage or strip.

Options specific to snapping:
  -o <snap-file>, --output <snap-file>  used in case you want to rename the
                                        snap.

The available commands are:
  help           Obtain help for a certain plugin or topic
  init           Initialize a snapcraft project.
  list-plugins   List the available plugins that handle different types of
                 part.
  login          Authenticate session against Ubuntu One SSO.
  logout         Clear session credentials.
  register-name  Register the package name in the store.
  upload         Upload a snap to the Ubuntu Store.

The available lifecycle commands are:
  clean        Remove content - cleans downloads, builds or install artifacts.
  cleanbuild   Create a snap using a clean environment managed by lxd.
  pull         Download or retrieve artifacts defined for a part.
  build        Build artifacts defined for a part. Build systems capable of
               running parallel build jobs will do so unless
               "--no-parallel-build" is specified.
  stage        Stage the part's built artifacts into the common staging area.
  strip        Final copy and preparation for the snap.
  snap         Create a snap.

Calling snapcraft without a COMMAND will default to 'snap'

The cleanbuild command requires a properly setup lxd environment that
can connect to external networks. Refer to the "Ubuntu Desktop and
Ubuntu Server" section on
https://linuxcontainers.org/lxd/getting-started-cli
to get started.

For more help, visit the documentation:
http://developer.ubuntu.com/snappy/snapcraft
"""

import logging
import pkg_resources
import pkgutil
import sys
import textwrap

from docopt import docopt

import snapcraft
from snapcraft.internal import lifecycle, log


logger = logging.getLogger(__name__)


def _get_version():
    try:
        return pkg_resources.require('snapcraft')[0].version
    except pkg_resources.DistributionNotFound:
        return 'devel'


def _list_plugins():
    for importer, modname, is_package in pkgutil.iter_modules(
            snapcraft.plugins.__path__):
        print(modname.replace('_', '-'))


def _get_project_options(args):
    options = {}
    options['use_geoip'] = args['--enable-geoip']
    options['parallel_builds'] = not args['--no-parallel-build']
    options['target_deb_arch'] = args['--target-arch']

    return snapcraft.ProjectOptions(**options)


def main(argv=None):
    args = docopt(__doc__, version=_get_version(), argv=argv)

    # Default log level is INFO unless --debug is specified
    log_level = logging.INFO
    if args['--debug']:
        log_level = logging.DEBUG

    log.configure(log_level=log_level)
    project_options = _get_project_options(args)

    try:
        return run(args, project_options)
    except Exception as e:
        if args['--debug']:
            raise

        sys.exit(textwrap.fill(str(e)))


def _get_lifecycle_command(args):
    lifecycle_commands = ['pull', 'build', 'stage', 'strip']
    lifecycle_command = [k for k in lifecycle_commands if args[k]]
    if len(lifecycle_command) == 0:
        return None
    return lifecycle_command[0]


def _get_command_from_arg(args):
    functions = {
        'init': lifecycle.init,
        'login': snapcraft.login,
        'logout': snapcraft.logout,
        'list-plugins': _list_plugins,
    }
    function = [k for k in functions if args[k]]
    if len(function) == 0:
        return None
    return functions[function[0]]


def run(args, project_options):
    lifecycle_command = _get_lifecycle_command(args)
    argless_command = _get_command_from_arg(args)
    if lifecycle_command:
        lifecycle.execute(
            lifecycle_command, project_options, args['<part>'])
    elif argless_command:
        argless_command()
    elif args['clean']:
        lifecycle.clean(project_options, args['<part>'], args['--step'])
    elif args['register-name']:
        snapcraft.register_name(args['<snap-name>'])
    elif args['upload']:
        snapcraft.upload(args['<snap-file>'])
    elif args['cleanbuild']:
        lifecycle.cleanbuild(project_options),
    elif args['help']:
        snapcraft.topic_help(args['<topic>'] or args['<plugin>'],
                             args['--devel'], args['topics'])
    else:  # snap by default:
        lifecycle.snap(project_options, args['<directory>'], args['--output'])

    return project_options


if __name__ == '__main__':  # pragma: no cover
    main()                  # pragma: no cover
