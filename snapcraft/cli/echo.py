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
"""Facilities to output to the terminal.

These methods, which are named after common logging levels, wrap around
click.echo adding the corresponding color codes for each level.
"""
import click

from snapcraft.internal import common


def wrapped(msg: str) -> None:
    """Output msg wrapped to the terminal width to stdout.

    The maximum wrapping is determined by
    snapcraft.internal.common.MAX_CHARACTERS_WRAP
    """
    click.echo(
        click.formatting.wrap_text(
            msg, width=common.MAX_CHARACTERS_WRAP, preserve_paragraphs=True
        )
    )


def info(msg: str) -> None:
    """Output msg as informative to stdout.
    If the terminal supports colors the output will be green.
    """
    click.echo("\033[0;32m{}\033[0m".format(msg))


def warning(msg: str) -> None:
    """Output msg as a warning to stdout.
    If the terminal supports color the output will be yellow.
    """
    click.echo("\033[1;33m{}\033[0m".format(msg))


def error(msg: str) -> None:
    """Output msg as an error to stdout.
    If the terminal supports color the output will be red.
    """
    click.echo("\033[0;31m{}\033[0m".format(msg))
