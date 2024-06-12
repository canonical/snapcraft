# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2024 Canonical Ltd.
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

"""Snapcraft test command."""

import argparse
import os

import craft_application.commands
from typing_extensions import override


class TestCommand(craft_application.commands.TestCommand):

    @override
    def fill_parser(self, parser: argparse.ArgumentParser) -> None:
        """Specify the parameters for this command."""
        super().fill_parser(parser)

        parser.add_argument(
            "--snap",
            type=str,
            default="",
            help="filename of the snap to test",
        )

    @override
    def _get_spread_environment(
        self, parsed_args: argparse.Namespace
    ) -> dict[str, str]:
        """Set the environment for the spread command."""
        env = os.environ.copy()
        env["SNAPCRAFT_TEST_SNAP_FILE"] = parsed_args.snap
        return env
