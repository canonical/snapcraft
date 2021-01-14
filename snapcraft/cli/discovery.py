# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017,2020 Canonical Ltd
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
from typing import Iterable

import click

import snapcraft
from snapcraft.internal import errors
from snapcraft.internal.common import format_output_in_columns, get_terminal_width
from snapcraft.project import errors as project_errors

from ._options import get_project


@click.group()
def discoverycli():
    pass


def _try_get_base_from_project() -> str:
    """Return the base used in the Snapcraft project or the current default"""
    try:
        base = get_project()._snap_meta.get_build_base()
    except (errors.ProjectNotFoundError, project_errors.MissingSnapcraftYamlError):
        base = "core20"

    return base


def _get_modules_iter(base: str) -> Iterable:
    if base in ("core", "core16", "core18"):
        modules_path = snapcraft.plugins.v1.__path__  # type: ignore  # mypy issue #1422
    else:
        modules_path = snapcraft.plugins.v2.__path__  # type: ignore  # mypy issue #1422

    # TODO make this part of plugin_finder.
    return pkgutil.iter_modules(modules_path)


@discoverycli.command("list-plugins")
@click.option(
    "--base",
    help="Show plugins for specific base",
    type=click.Choice(["core", "core16", "core18", "core20"]),
)
def list_plugins(base):
    """List the available plugins that handle different types of part.

    This command has an alias of `plugins`.
    """
    if base is None:
        base = _try_get_base_from_project()

    plugins = []
    for _, modname, _ in _get_modules_iter(base):
        # Only add non-private modules/packages to the plugin list
        if not modname.startswith("_"):
            plugins.append(modname.replace("_", "-"))

    # we wrap the output depending on terminal size
    width = get_terminal_width()
    click.echo(f"Displaying plugins available for {base!r}")
    for line in format_output_in_columns(plugins, max_width=width):
        click.echo(line)
