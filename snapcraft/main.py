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
import logging
import sys

import snapcraft.cmds


def main():
    logging.basicConfig(format='%(message)s', level=logging.INFO)

    root_parser = argparse.ArgumentParser()
    subparsers = root_parser.add_subparsers(dest='cmd')

    # Parent parsers

    force_parser = argparse.ArgumentParser(add_help=False)
    force_parser.add_argument('--force', action='store_true', help='redo all steps for all parts')

    cmd_parser = argparse.ArgumentParser(add_help=False, parents=[force_parser])
    cmd_parser.add_argument('part', nargs='*')

    # Command parsers

    parser = subparsers.add_parser('init', help='start a project')
    parser.add_argument('part', nargs='*', help='part to add to new project')
    parser.set_defaults(func=snapcraft.cmds.init)

    parser = subparsers.add_parser('shell', help='enter staging environment')
    parser.add_argument('userCommand', nargs=argparse.REMAINDER, metavar='cmd', help='optional command to run inside staging environment')
    parser.set_defaults(func=snapcraft.cmds.shell)

    parser = subparsers.add_parser('run', help='run snap in kvm', add_help=False)
    parser.set_defaults(func=snapcraft.cmds.run)

    parser = subparsers.add_parser('pull', help='get sources', parents=[cmd_parser])
    parser.set_defaults(func=snapcraft.cmds.cmd)

    parser = subparsers.add_parser('build', help='build parts', parents=[cmd_parser])
    parser.set_defaults(func=snapcraft.cmds.cmd)

    parser = subparsers.add_parser('stage', help='put parts into staging area', parents=[cmd_parser])
    parser.set_defaults(func=snapcraft.cmds.cmd)

    parser = subparsers.add_parser('snap', help='put parts into snap area', parents=[cmd_parser])
    parser.set_defaults(func=snapcraft.cmds.snap)

    parser = subparsers.add_parser('assemble', help='make snap package', parents=[force_parser], aliases=['all'])
    parser.set_defaults(func=snapcraft.cmds.assemble)

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
