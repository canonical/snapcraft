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
"""Snapcraft init command."""

import argparse
import pathlib

import craft_application.commands
from typing_extensions import override


class InitCommand(craft_application.commands.InitCommand):

    @override
    def fill_parser(self, parser: argparse.ArgumentParser) -> None:
        """Specify the parameters for this command."""
        super().fill_parser(parser)

        parser.add_argument(
            "--profile",
            choices=["default", "tests"],
            default="default",
            help="Use the specified project profile (defaults to 'default')",
        )

    @override
    def _get_template_dir(self, parsed_args: argparse.Namespace) -> pathlib.Path:
        """Return the path to the template directory."""
        return pathlib.Path("templates") / parsed_args.profile
