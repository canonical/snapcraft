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
import logging
import os
import sys

import click

import snapcraft
import snapcraft.internal.dirs
from snapcraft.internal import deprecations
from snapcraft.internal import log
from . import echo
from .assertions import assertionscli
from .discovery import discoverycli
from .lifecycle import lifecyclecli
from .store import storecli
from .parts import partscli
from .help import helpcli
from .ci import cicli
from ._options import add_build_options
from ._errors import exception_handler


command_groups = [
    storecli,
    cicli,
    assertionscli,
    discoverycli,
    helpcli,
    lifecyclecli,
    partscli,
]

_CMD_DEPRECATED_REPLACEMENTS = {
    'strip': 'prime',
    'upload': 'push',
    'history': 'list-revisions',
}

_CMD_ALIASES = {
    'registered': 'list-registered',
    'keys': 'list-keys',
    'revisions': 'list-revisions',
    'plugins': 'list-plugins',
}

_CMD_DEPRECATION_NOTICES = {
    'history': 'dn4',
}


snapcraft.internal.dirs.setup_dirs()


class SnapcraftGroup(click.Group):

    def get_command(self, ctx, cmd_name):
        new_cmd_name = _CMD_DEPRECATED_REPLACEMENTS.get(cmd_name)
        if new_cmd_name:
            if _CMD_DEPRECATION_NOTICES.get(cmd_name):
                deprecations.handle_deprecation_notice(
                    _CMD_DEPRECATION_NOTICES.get(cmd_name))
            else:
                echo.warning('DEPRECATED: Use {!r} instead of {!r}'.format(
                    new_cmd_name, cmd_name))
            cmd = click.Group.get_command(self, ctx, new_cmd_name)
        else:
            cmd_name = _CMD_ALIASES.get(cmd_name, cmd_name)
            cmd = click.Group.get_command(self, ctx, cmd_name)
        return cmd


@click.group(cls=SnapcraftGroup, invoke_without_command=True)
@click.version_option(version=snapcraft.__version__)
@click.pass_context
@add_build_options(hidden=True)
@click.option('--debug', '-d', is_flag=True)
def run(ctx, debug, catch_exceptions=False, **kwargs):
    """Snapcraft is a delightful packaging tool."""

    if debug:
        log_level = logging.DEBUG
        click.echo('Starting snapcraft {} from {}.'.format(
            snapcraft.__version__, os.path.dirname(__file__)))
    else:
        log_level = logging.INFO

    # Setup global exception handler (to be called for unhandled exceptions)
    sys.excepthook = functools.partial(exception_handler, debug=debug)

    # In an ideal world, this logger setup would be replaced
    log.configure(log_level=log_level)
    # The default command
    if not ctx.invoked_subcommand:
        ctx.forward(lifecyclecli.commands['snap'])


# This would be much easier if they were subcommands
for command_group in command_groups:
    for command in command_group.commands:
        run.add_command(command_group.commands[command])
