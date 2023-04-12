# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2023 Canonical Ltd.
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

"""Snapcraft lint commands."""

import argparse
import os
import textwrap
from pathlib import Path
from typing import Optional

from craft_cli import BaseCommand, emit
from craft_cli.errors import ArgumentParsingError
from overrides import overrides


class LintCommand(BaseCommand):
    """Lint-related commands."""

    name = "lint"
    help_msg = "Lint a snap file"
    overview = textwrap.dedent(
        """
        Lint an existing snap file.

        The snap is installed and linted inside a build environment. If an assertion
        file exists in the same directory as the snap file with the name
        `<snap-name>.assert`, it will be used to install the snap in the instance.
        """
    )

    @overrides
    def fill_parser(self, parser: "argparse.ArgumentParser") -> None:
        parser.add_argument(
            "snap_file",
            metavar="snap-file",
            type=Path,
            help="Snap file to lint",
        )
        parser.add_argument(
            "--use-lxd",
            action="store_true",
            help="Use LXD to lint",
        )
        parser.add_argument(
            "--http-proxy",
            type=str,
            default=os.getenv("http_proxy"),
            help="Set http proxy",
        )
        parser.add_argument(
            "--https-proxy",
            type=str,
            default=os.getenv("https_proxy"),
            help="Set https proxy",
        )

    @overrides
    def run(self, parsed_args):
        """Run the linter command.

        :raises ArgumentParsingError: If the snap file does not exist or is not valid.
        """
        emit.progress("Running linter.", permanent=True)
        snap_file = Path(parsed_args.snap_file)

        if not snap_file.exists():
            raise ArgumentParsingError(f"Snap file {str(snap_file)!r} does not exist.")

        if not snap_file.is_file():
            raise ArgumentParsingError(
                f"Snap file {str(snap_file)!r} is not a valid file."
            )

        self._get_assert_file(snap_file)

        emit.progress("'snapcraft lint' not implemented.", permanent=True)

    def _get_assert_file(self, snap_file: Path) -> Optional[Path]:
        """Get an assertion file for a snap file.

        The assertion file name should be formatted as `<snap-name>.assert`.

        :param snap_file: Path to snap file.

        :returns: Path to the assertion file or None if the file does not exist or is
        not valid.
        """
        assert_file = snap_file.with_suffix(".assert")

        if not assert_file.exists():
            emit.debug(f"Assertion file {str(assert_file)!r} does not exist.")
            return None

        if not assert_file.is_file():
            emit.debug(f"Assertion file {str(assert_file)!r} is not a valid file.")
            return None

        emit.debug(f"Found assertion file {str(assert_file)!r}.")
        return assert_file
