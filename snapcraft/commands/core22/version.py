# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
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

"""Snapcraft version command."""

from craft_cli import BaseCommand, emit

from snapcraft import __version__


class VersionCommand(BaseCommand):
    """Show the snapcraft version."""

    name = "version"
    help_msg = "Show the application version and exit"
    overview = "Show the application version and exit"
    common = True

    def run(self, parsed_args):
        """Run the command."""
        emit.message(f"snapcraft {__version__}")
