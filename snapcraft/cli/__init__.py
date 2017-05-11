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
import logging

import click

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


command_groups = [
    storecli,
    cicli,
    assertionscli,
    discoverycli,
    helpcli,
    lifecyclecli,
    partscli,
]

_cmd_deprecated_replacements = {
    'strip': 'prime',
    'upload': 'push',
    'history': 'list-revisions',
}

_cmd_aliases = {
    'registered': 'list-registered',
    'keys': 'list-keys',
    'revisions': 'list-revisions',
    'plugins': 'list-plugins',
}

_cmd_deprecation_notices = {
    'history': 'dn4',
}


snapcraft.internal.dirs.setup_dirs()


class SnapcraftGroup(click.Group):

    def get_command(self, ctx, cmd_name):
        new_cmd_name = _cmd_deprecated_replacements.get(cmd_name)
        if new_cmd_name:
            if _cmd_deprecation_notices.get(cmd_name):
                deprecations.handle_deprecation_notice(
                    _cmd_deprecation_notices.get(cmd_name))
            else:
                echo.warning('DEPRECATED: Use {!r} instead of {!r}'.format(
                    new_cmd_name, cmd_name))
            cmd = click.Group.get_command(self, ctx, new_cmd_name)
        else:
            cmd_name = _cmd_aliases.get(cmd_name, cmd_name)
            cmd = click.Group.get_command(self, ctx, cmd_name)
        return cmd

    def parse_args(self, ctx, args):
        if not args:
            args.insert(0, 'snap')
        return super().parse_args(ctx, args)


@click.group(cls=SnapcraftGroup)
@click.pass_context
@click.option('--debug', '-d', is_flag=True)
def run(ctx, debug, catch_exceptions=False):
    """Snapcraft is a delightful packaging tool."""
    if debug:
        log_level = logging.DEBUG
    else:
        log_level = logging.INFO
    # In an ideal world, this logger setup would be replaced
    log.configure(log_level=log_level)


# This would be much easier if they were subcommands
for command_group in command_groups:
    for command in command_group.commands:
        run.add_command(command_group.commands[command])
