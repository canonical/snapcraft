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
import pkgutil

import click

import snapcraft
from snapcraft.internal.common import format_output_in_columns
from snapcraft.internal.common import get_terminal_width


@click.group()
def discoverycli():
    pass


@discoverycli.command("list-plugins")
def list_plugins():
    """List the available plugins that handle different types of part.

    This command has an alias of `plugins`.
    """
    plugins = []
    for _, modname, _ in pkgutil.iter_modules(snapcraft.plugins.__path__):
        # Only add non-private modules/packages to the plugin list
        if not modname.startswith("_"):
            plugins.append(modname.replace("_", "-"))

    # we wrap the output depending on terminal size
    width = get_terminal_width()
    for line in format_output_in_columns(plugins, max_width=width):
        click.echo(line)
