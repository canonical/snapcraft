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
import os
import pkgutil
import shutil
import sys

import click

import snapcraft
from snapcraft.internal.common import format_output_in_columns
from snapcraft.internal.common import get_terminal_width
from snapcraft.internal.common import get_tourdir
from . import echo


@click.group()
def discoverycli():
    pass

_SNAPCRAFT_TOUR_DIR = './snapcraft-tour/'


@discoverycli.command()
@click.argument('directory', metavar='<directory>',
                required=False,
                default=_SNAPCRAFT_TOUR_DIR,
                type=click.Path(exists=False))
def tour(directory):
    """Setup the snapcraft examples used in the tour."""
    echo.info('Copying examples tour to {}'.format(directory))
    dest_dir = os.path.abspath(directory)

    # If dest_dir doesn't exist, we dump all examples in it.
    # If it does exist, we dump them into a subdirectory if it's not
    # the default dir.
    try:
        shutil.copytree(get_tourdir(), dest_dir)
    except FileExistsError:
        # default crafted directory shouldn't add itself inside
        if directory == _SNAPCRAFT_TOUR_DIR:
            echo.error('{!r} already exists, please specify a '
                       'destination directory.'.format(directory))
            sys.exit(1)
        # don't event try to copy if the dest exists already
        if not os.path.isdir(dest_dir):
            echo.error('{!r} is a file, cannot be used as a '
                       'destination'.format(dest_dir))
            sys.exit(1)
        dest_dir = os.path.normpath(os.path.join(dest_dir,
                                                 _SNAPCRAFT_TOUR_DIR))
        shutil.copytree(get_tourdir(), dest_dir)

    click.echo('Snapcraft tour initialized in {!r}\n'
               'Instructions are in the README, or '
               'http://snapcraft.io/create/#tour'.format(directory))


@discoverycli.command('list-plugins')
def list_plugins():
    """List the available plugins that handle different types of part.

    This command has an alias of `plugins`.
    """
    plugins = []
    for importer, modname, is_package in pkgutil.iter_modules(
            snapcraft.plugins.__path__):
        plugins.append(modname.replace('_', '-'))

    # we wrap the output depending on terminal size
    width = get_terminal_width()
    for line in format_output_in_columns(plugins, max_width=width):
        click.echo(line)
