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
from shlex import join
from subprocess import CalledProcessError
from typing import Optional

from craft_cli import BaseCommand, emit
from craft_cli.errors import ArgumentParsingError
from overrides import overrides

from snapcraft import providers
from snapcraft.errors import SnapcraftError
from snapcraft.utils import get_managed_environment_home_path, is_managed_mode


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
    def run(self, parsed_args: argparse.Namespace):
        """Run the linter command.

        :param parsed_args: snapcraft's argument namespace

        :raises ArgumentParsingError: If the snap file does not exist or is not valid.
        """
        emit.progress("Running linter.", permanent=True)

        snap_file = parsed_args.snap_file

        if not snap_file.exists():
            raise ArgumentParsingError(f"snap file {str(snap_file)!r} does not exist")

        if not snap_file.is_file():
            raise ArgumentParsingError(
                f"snap file {str(snap_file)!r} is not a valid file"
            )

        assert_file = self._get_assert_file(snap_file)

        if is_managed_mode():
            self._run_linter(snap_file, assert_file)
        else:
            self._prepare_instance(
                snap_file, assert_file, parsed_args.http_proxy, parsed_args.https_proxy
            )

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

    def _prepare_instance(
        self,
        snap_file: Path,
        assert_file: Optional[Path],
        http_proxy: Optional[str],
        https_proxy: Optional[str],
    ) -> None:
        """Prepare an instance to lint a snap file.

        Launches an instance, pushes the snap file and optional assert file, then
        re-executes `snapcraft lint` inside the instance.

        :param snap_file: Path to snap file to push into the instance.
        :param assert_file: Optional path to assertion file to push into the instance.
        :param http_proxy: http proxy to add to environment
        :param https_proxy: https proxy to add to environment
        """
        emit.progress("Checking build provider availability.")

        provider = providers.get_provider()
        providers.ensure_provider_is_available(provider)

        # create base configuration
        instance_name = "snapcraft-linter"
        build_base = providers.SNAPCRAFT_BASE_TO_PROVIDER_BASE["core22"]
        base_configuration = providers.get_base_configuration(
            alias=build_base,
            instance_name=instance_name,
            http_proxy=http_proxy,
            https_proxy=https_proxy,
        )

        emit.progress("Launching instance.")

        with provider.launched_environment(
            project_name="snapcraft-linter",
            project_path=Path().absolute(),
            base_configuration=base_configuration,
            build_base=build_base.value,
            instance_name=instance_name,
            allow_unstable=False,
        ) as instance:
            # push snap file
            snap_file_instance = get_managed_environment_home_path() / snap_file.name
            instance.push_file(source=snap_file, destination=snap_file_instance)

            # push assert file
            if assert_file:
                assert_file_instance = (
                    get_managed_environment_home_path() / assert_file.name
                )
                instance.push_file(source=assert_file, destination=assert_file_instance)

            # run linter inside the instance
            command = ["snapcraft", "lint", str(snap_file_instance)]
            try:
                with emit.pause():
                    instance.execute_run(command, check=True)
            except CalledProcessError as err:
                raise SnapcraftError(
                    f"failed to execute {join(command)!r} in instance",
                ) from err

    # pylint: disable-next=unused-argument
    def _run_linter(self, snap_file: Path, assert_file: Optional[Path]) -> None:
        """Run snapcraft linters on a snap file.

        :param snap_file: Path to snap file to lint.
        :param assert_file: Optional path to assertion file for the snap file.
        """
        emit.progress("'snapcraft lint' not implemented.", permanent=True)
