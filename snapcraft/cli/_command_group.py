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
import click

from snapcraft.internal import deprecations
from . import echo
from . import env


_CMD_DEPRECATED_REPLACEMENTS = {
    "strip": "prime",
    "upload": "push",
    "history": "list-revisions",
}

_CMD_ALIASES = {
    "registered": "list-registered",
    "keys": "list-keys",
    "revisions": "list-revisions",
    "plugins": "list-plugins",
    "collaborators": "edit-collaborators",
    "extensions": "list-extensions",
}

_CMD_DEPRECATION_NOTICES = {"history": "dn4"}


class SnapcraftGroup(click.Group):
    def get_command(self, ctx, cmd_name):
        new_cmd_name = _CMD_DEPRECATED_REPLACEMENTS.get(cmd_name)
        if new_cmd_name:
            if _CMD_DEPRECATION_NOTICES.get(cmd_name):
                deprecations.handle_deprecation_notice(
                    _CMD_DEPRECATION_NOTICES.get(cmd_name)
                )
            else:
                echo.warning(
                    "DEPRECATED: Use {!r} instead of {!r}".format(
                        new_cmd_name, cmd_name
                    )
                )
            cmd = click.Group.get_command(self, ctx, new_cmd_name)
        else:
            cmd_name = _CMD_ALIASES.get(cmd_name, cmd_name)
            cmd = click.Group.get_command(self, ctx, cmd_name)
        return cmd

    def list_commands(self, ctx):
        commands = super().list_commands(ctx)
        # Let's keep edit-collaborators hidden until we get the green light
        # from the store.
        commands.pop(commands.index("edit-collaborators"))

        # Inspect is for internal usage: hide it
        commands.pop(commands.index("inspect"))
        build_environment = env.BuilderEnvironmentConfig()
        if build_environment.is_host:
            commands.pop(commands.index("refresh"))
        return commands
