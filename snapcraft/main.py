#!/usr/bin/python3
# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015 Canonical Ltd
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
import pkg_resources
import sys

import snapcraft.cmds

from snapcraft import help
from snapcraft import log


class _VersionAction(argparse.Action):

    def __init__(self, option_strings, version=None, dest=argparse.SUPPRESS,
                 default=argparse.SUPPRESS, nargs=0, help=None):
        super(_VersionAction, self).__init__(
            option_strings=option_strings,
            dest=dest,
            default=default,
            nargs=0,
            help=help)

    def __call__(self, parser, namespace, values, option_string=None):
        version_string = '{prog} ({ver}).\nRun "{prog} help" to get started.'

        try:
            version = pkg_resources.require('snapcraft')[0].version
        except pkg_resources.DistributionNotFound:
            version = 'devel'

        print(version_string.format(prog=parser.prog.split()[0], ver=version))
        parser.exit()


def main():
    log.configure()
    root_parser = argparse.ArgumentParser()
    subparsers = root_parser.add_subparsers(dest='cmd')

    # Parent parsers

    force_parser = argparse.ArgumentParser(add_help=False)
    force_parser.add_argument('--force', action='store_true',
                              help='redo all steps for all parts')

    cmd_parser = argparse.ArgumentParser(add_help=False,
                                         parents=[force_parser])
    cmd_parser.add_argument('part', nargs='*')

    root_parser.add_argument('-v', '--version', action=_VersionAction,
                             help="show the program's version number and exit")

    # Command parsers

    parser = subparsers.add_parser('init', help='start a project')
    parser.add_argument('part', nargs='*', help='part to add to new project')
    parser.set_defaults(func=snapcraft.cmds.init)

    parser = subparsers.add_parser('shell', help='enter staging environment')
    parser.add_argument(
        'userCommand', nargs=argparse.REMAINDER,
        metavar='cmd',
        help='optional command to run inside staging environment')
    parser.set_defaults(func=snapcraft.cmds.shell)

    parser = subparsers.add_parser('run', help='run snap in kvm',
                                   add_help=False)
    parser.set_defaults(func=snapcraft.cmds.run)

    parser = subparsers.add_parser(
        'list-plugins',
        help='list the available plugins that handle different types '
        'of a part')
    parser.set_defaults(func=snapcraft.cmds.list_plugins)

    parser = subparsers.add_parser(
        'clean',
        help='clean up the environment (to start from scratch)')
    parser.set_defaults(func=snapcraft.cmds.clean)
    parser.add_argument('parts', nargs='*', metavar='PART',
                        help='specific part to clean')

    parser = subparsers.add_parser('pull', help='get sources',
                                   parents=[cmd_parser])
    parser.set_defaults(func=snapcraft.cmds.cmd)

    parser = subparsers.add_parser('build', help='build parts',
                                   parents=[cmd_parser])
    parser.set_defaults(func=snapcraft.cmds.cmd)

    parser = subparsers.add_parser(
        'stage',
        help='put parts into staging area', parents=[cmd_parser])
    parser.set_defaults(func=snapcraft.cmds.cmd)

    parser = subparsers.add_parser(
        'snap',
        help='put parts into snap area',
        parents=[cmd_parser])
    parser.set_defaults(func=snapcraft.cmds.snap)

    parser = subparsers.add_parser(
        'assemble',
        help='make snap package', parents=[force_parser],
        aliases=['all'])
    parser.set_defaults(func=snapcraft.cmds.assemble)

    parser = subparsers.add_parser(
        'version', help="show the program's version number and exit")
    parser.add_argument('foo', action=_VersionAction, nargs='?',
                        help=argparse.SUPPRESS)

    parser = subparsers.add_parser(
        'help',
        usage=help.topic.__doc__,
        help='obtain help for plugins and specific topics')
    parser.set_defaults(func=help.topic)
    parser.add_argument('topic', help='plugin name or topic to get help from')
    parser.add_argument('--devel', action='store_true',
                        help='show the development help')

    # Now run parser

    if len(sys.argv) < 2:
        args = root_parser.parse_args(['all'])
    else:
        args = root_parser.parse_args()

    if not hasattr(args, 'func'):
        root_parser.print_help()
        sys.exit(1)

    args.func(args)
    sys.exit(0)
