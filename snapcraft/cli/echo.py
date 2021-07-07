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
import distutils.util
import os
import sys
from typing import Any, Optional

import click

from snapcraft.internal import common


def is_tty_connected() -> bool:
    """ Check to see if running under TTY. """
    if distutils.util.strtobool(os.getenv("SNAPCRAFT_HAS_TTY", "n")) == 1:
        return True

    return sys.stdin.isatty()


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
    click.echo("\033[1;33m{}\033[0m".format(msg), err=True)


def error(msg: str) -> None:
    """Output msg as an error to stdout.
    If the terminal supports color the output will be red.
    """
    click.echo("\033[0;31m{}\033[0m".format(msg), err=True)


def exit_error(
    *,
    brief: str,
    resolution: Optional[str] = None,
    details: Optional[str] = None,
    docs_url: Optional[str] = None,
    exit_code: int = 2
):
    """Display an error and gracefully exit."""
    message_parts = [brief]

    if resolution is not None:
        message_parts.extend(["", "Recommended resolution:", resolution])

    if details is not None:
        message_parts.extend(["", "Detailed information:", details])

    if docs_url is not None:
        message_parts.extend(["", "For more information, check out:", docs_url])

    error("\n".join(message_parts))
    sys.exit(exit_code)


def confirm(
    msg: str,
    default: bool = False,
    abort: bool = False,
    prompt_suffix: str = ": ",
    show_default: bool = True,
    err: bool = False,
) -> bool:
    """Output message as a confirmation prompt.
    If not running on a tty, assume the default value.
    """
    return (
        click.confirm(
            msg,
            default=default,
            abort=abort,
            prompt_suffix=prompt_suffix,
            show_default=show_default,
            err=err,
        )
        if is_tty_connected()
        else default
    )


def prompt(
    msg: str,
    default: Any = None,
    hide_input: bool = False,
    confirmation_prompt: bool = False,
    type=None,
    value_proc=None,
    prompt_suffix: str = ": ",
    show_default: bool = True,
    err: bool = False,
) -> Any:
    """Output message as a generic prompt.
    If not running on a tty, assume the default value.
    """
    return (
        click.prompt(
            msg,
            default=default,
            hide_input=hide_input,
            confirmation_prompt=confirmation_prompt,
            type=type,
            value_proc=value_proc,
            prompt_suffix=prompt_suffix,
            show_default=show_default,
            err=err,
        )
        if is_tty_connected()
        else default
    )
