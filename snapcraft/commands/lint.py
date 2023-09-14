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
import shlex
import subprocess
import tempfile
import textwrap
from contextlib import contextmanager
from pathlib import Path, PurePosixPath
from typing import Iterator, Optional

from craft_cli import BaseCommand, emit
from craft_cli.errors import ArgumentParsingError
from craft_providers.multipass import MultipassProvider
from craft_providers.util import snap_cmd
from overrides import overrides

from snapcraft import errors, linters, projects, providers
from snapcraft.meta import snap_yaml
from snapcraft.parts.yaml_utils import apply_yaml, extract_parse_info, process_yaml
from snapcraft.utils import (
    get_host_architecture,
    get_managed_environment_home_path,
    is_managed_mode,
)


class LintCommand(BaseCommand):
    """Lint-related commands."""

    name = "lint"
    help_msg = "Lint a snap file"
    overview = textwrap.dedent(
        """
        Lint an existing snap file.

        The snap is installed and linted inside a build environment. If an assertion
        file exists in the same directory as the snap file with the name
        ``<snap-name>.assert``, it will be used to install the snap in the instance.
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

        :raises errors.SnapcraftError: If `snapcraft lint` fails inside the instance.
        """
        emit.progress("Checking build provider availability.")

        provider = providers.get_provider()
        if isinstance(provider, MultipassProvider):
            # blocked by https://github.com/canonical/craft-providers/issues/169
            raise errors.SnapcraftError(
                "'snapcraft lint' is not supported with Multipass as the build provider"
            )
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
            home_path = get_managed_environment_home_path()

            # push snap file
            snap_file_instance = home_path / snap_file.name
            instance.push_file(source=snap_file, destination=snap_file_instance)

            # push assert file
            if assert_file:
                assert_file_instance = home_path / assert_file.name
                instance.push_file(source=assert_file, destination=assert_file_instance)

            # run linter inside the instance
            command = ["snapcraft", "lint", str(snap_file_instance)]
            try:
                emit.debug(f"running {shlex.join(command)!r} in instance")
                with emit.pause():
                    instance.execute_run(command, check=True)
            except subprocess.CalledProcessError as error:
                raise errors.SnapcraftError(
                    f"failed to execute {shlex.join(command)!r} in instance",
                ) from error
            finally:
                providers.capture_logs_from_instance(instance)

    def _run_linter(self, snap_file: Path, assert_file: Optional[Path]) -> None:
        """Run snapcraft linters on a snap file.

        :param snap_file: Path to snap file to lint.
        :param assert_file: Optional path to assertion file for the snap file.
        """
        # unsquash, load snap.yaml, and optionally load snapcraft.yaml
        with self._unsquash_snap(snap_file) as unsquashed_snap:
            snap_metadata = snap_yaml.read(unsquashed_snap)
            project = self._load_project(unsquashed_snap / "snap" / "snapcraft.yaml")

        snap_install_path = self._install_snap(snap_file, assert_file, snap_metadata)

        lint_filters = self._load_lint_filters(project)

        # run the linters
        issues = linters.run_linters(location=snap_install_path, lint=lint_filters)
        linters.report(issues, intermediate=True)

    @contextmanager
    def _unsquash_snap(self, snap_file: Path) -> Iterator[Path]:
        """Unsquash a snap file to a temporary directory.

        :param snap_file: Snap package to extract.

        :yields: Path to the snap's unsquashed directory.

        :raises errors.SnapcraftError: If the snap fails to unsquash.
        """
        snap_file = snap_file.resolve()

        with tempfile.TemporaryDirectory(prefix=str(snap_file.parent)) as temp_dir:
            emit.progress(f"Unsquashing snap file {snap_file.name!r}.")

            # unsquashfs [options] filesystem [directories or files to extract] options:
            # -force: if file already exists then overwrite
            # -dest <pathname>: unsquash to <pathname>
            extract_command = [
                "unsquashfs",
                "-force",
                "-dest",
                temp_dir,
                str(snap_file),
            ]

            try:
                subprocess.run(extract_command, capture_output=True, check=True)
            except subprocess.CalledProcessError as error:
                raise errors.SnapcraftError(
                    f"could not unsquash snap file {snap_file.name!r}"
                ) from error

            yield Path(temp_dir)

    def _load_project(self, snapcraft_yaml_file: Path) -> Optional[projects.Project]:
        """Load a snapcraft Project from a snapcraft.yaml, if present.

        The snapcraft.yaml exist for snaps built with the `--enable-manifest` parameter.

        :param snapcraft_yaml_file: path to snapcraft.yaml file to load

        :returns: A Project containing the snapcraft.yaml's data or None if the yaml
        file does not exist.
        """
        if not snapcraft_yaml_file.exists():
            emit.debug(f"Could not find {snapcraft_yaml_file.name!r}.")
            return None

        try:
            # process_yaml will not parse core, core18, and core20 snaps
            yaml_data = process_yaml(snapcraft_yaml_file)
        except (errors.LegacyFallback, errors.MaintenanceBase) as error:
            raise errors.SnapcraftError(
                "can not lint snap using a base older than core22"
            ) from error

        # process yaml before unmarshalling the data
        arch = get_host_architecture()
        yaml_data_for_arch = apply_yaml(yaml_data, arch, arch)
        # discard parse-info - it is not needed
        extract_parse_info(yaml_data_for_arch)
        project = projects.Project.unmarshal(yaml_data_for_arch)
        return project

    def _install_snap(
        self,
        snap_file: Path,
        assert_file: Optional[Path],
        snap_metadata: snap_yaml.SnapMetadata,
    ) -> Path:
        """Install a snap file and optional assertion file.

        If the architecture of the snap file does not match the host architecture, then
        `snap install` will exit with a descriptive error.

        :param snap_file: Snap file to install.
        :param assert_file: Optional assertion file to install.
        :param snap_metadata: SnapMetadata from the snap file.

        :returns: Path to where snap was installed.

        :raises errors.SnapcraftError: If the snap cannot be installed.
        """
        is_dangerous = not bool(assert_file)

        if assert_file:
            ack_command = snap_cmd.formulate_ack_command(PurePosixPath(assert_file))
            emit.progress(
                f"Installing assertion file with {shlex.join(ack_command)!r}."
            )

            try:
                subprocess.run(ack_command, capture_output=True, check=True)
            except subprocess.CalledProcessError:
                # if assertion fails, then install the snap dangerously
                is_dangerous = True
                emit.progress(
                    f"Could not add assertions from file {assert_file.name!r}",
                    permanent=True,
                )

        install_command = snap_cmd.formulate_local_install_command(
            classic=bool(snap_metadata.confinement == "classic"),
            dangerous=is_dangerous,
            snap_path=PurePosixPath(snap_file),
        )
        if snap_metadata.grade == "devel":
            install_command.append("--devmode")

        emit.progress(f"Installing snap with {shlex.join(install_command)!r}.")

        try:
            subprocess.run(install_command, capture_output=True, check=True)
        except subprocess.CalledProcessError as error:
            raise errors.SnapcraftError(
                f"could not install snap file {snap_file.name!r}"
            ) from error

        return Path("/snap") / snap_metadata.name / "current"

    def _load_lint_filters(self, project: Optional[projects.Project]) -> projects.Lint:
        """Load lint filters from a Project and disable the classic linter.

        :param project: Project from the snap file, if present.

        :returns: Lint config with classic linter disabled.
        """
        lint_config = projects.Lint(ignore=["classic"])

        if project:
            if project.lint:
                emit.verbose("Collected lint config from 'snapcraft.yaml'.")
                lint_config = project.lint

                # remove any file-specific classic filters
                for item in lint_config.ignore:
                    if isinstance(item, dict) and "classic" in item.keys():
                        lint_config.ignore.remove(item)

                # disable entire classic linter with the "classic" string
                if "classic" not in lint_config.ignore:
                    lint_config.ignore.append("classic")

            else:
                emit.verbose("No lint filters defined in 'snapcraft.yaml'.")
        else:
            emit.verbose(
                "Not loading lint filters from 'snapcraft.yaml' because the file "
                "does not exist inside the snap file."
            )
            emit.verbose(
                "To include 'snapcraft.yaml' in a snap file, use the parameter "
                "'--enable-manifest' when building the snap."
            )

        return lint_config
