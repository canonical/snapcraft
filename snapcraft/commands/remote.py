# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022-2023 Canonical Ltd.
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

"""Snapcraft remote build command."""

import argparse
import os
import textwrap
from enum import Enum
from pathlib import Path
from typing import Optional

from craft_cli import BaseCommand, emit
from craft_cli.helptexts import HIDDEN
from overrides import overrides

from snapcraft.errors import MaintenanceBase, SnapcraftError
from snapcraft.legacy_cli import run_legacy
from snapcraft.parts import yaml_utils
from snapcraft.remote import get_build_id, is_repo
from snapcraft.utils import confirm_with_user, humanize_list
from snapcraft_legacy.internal.remote_build.errors import AcceptPublicUploadError

_CONFIRMATION_PROMPT = (
    "All data sent to remote builders will be publicly available. "
    "Are you sure you want to continue?"
)


_STRATEGY_ENVVAR = "SNAPCRAFT_REMOTE_BUILD_STRATEGY"


class _Strategies(Enum):
    """Possible values of the build strategy."""

    DISABLE_FALLBACK = "disable-fallback"
    FORCE_FALLBACK = "force-fallback"


class RemoteBuildCommand(BaseCommand):
    """Command passthrough for the remote-build command."""

    name = "remote-build"
    help_msg = "Dispatch a snap for remote build"
    overview = textwrap.dedent(
        """
        Command remote-build sends the current project to be built
        remotely.  After the build is complete, packages for each
        architecture are retrieved and will be available in the
        local filesystem.

        If not specified in the snapcraft.yaml file, the list of
        architectures to build can be set using the --build-on option.
        If both are specified, an error will occur.

        Interrupted remote builds can be resumed using the --recover
        option, followed by the build number informed when the remote
        build was originally dispatched. The current state of the
        remote build for each architecture can be checked using the
        --status option."""
    )

    @overrides
    def fill_parser(self, parser: argparse.ArgumentParser) -> None:
        parser.add_argument(
            "--recover", action="store_true", help="recover an interrupted build"
        )
        parser.add_argument(
            "--status", action="store_true", help="display remote build status"
        )
        parser_target = parser.add_mutually_exclusive_group()
        parser_target.add_argument(
            "--build-on",
            metavar="arch",
            nargs="+",
            help=HIDDEN,
        )
        parser_target.add_argument(
            "--build-for",
            metavar="arch",
            nargs="+",
            help="architecture to build for",
        )
        parser.add_argument(
            "--build-id", metavar="build-id", help="specific build id to retrieve"
        )
        parser.add_argument(
            "--launchpad-accept-public-upload",
            action="store_true",
            help="acknowledge that uploaded code will be publicly available.",
        )

    @overrides
    def run(self, parsed_args: argparse.Namespace) -> None:
        """Run the remote-build command.

        :param parsed_args: Snapcraft's argument namespace.

        :raises AcceptPublicUploadError: If the user does not agree to upload data.
        :raises SnapcraftError: If the project cannot be loaded and parsed.
        """
        if os.getenv("SUDO_USER") and os.geteuid() == 0:
            emit.message(
                "Running with 'sudo' may cause permission errors and is discouraged."
            )

        emit.message(
            "snapcraft remote-build is experimental and is subject to change "
            "- use with caution."
        )

        if parsed_args.build_on:
            emit.message("Use --build-for instead of --build-on")
            parsed_args.build_for = parsed_args.build_on

        if not parsed_args.launchpad_accept_public_upload and not confirm_with_user(
            _CONFIRMATION_PROMPT
        ):
            raise AcceptPublicUploadError()

        # pylint: disable=attribute-defined-outside-init
        self._snapcraft_yaml = yaml_utils.get_snap_project().project_file
        self._parsed_args = parsed_args
        # pylint: enable=attribute-defined-outside-init
        try:
            base = self._get_effective_base()
        except MaintenanceBase as base_err:
            base = base_err.base
            emit.progress(_get_esm_warning_for_base(base), permanent=True)

        self._run_new_or_fallback_remote_build(base)

    def _run_new_or_fallback_remote_build(self, base: str) -> None:
        """Run the new or fallback remote-build.

        Three checks determine whether to run the new or fallback remote-build:
        1. Base: snaps newer than core22 must use the new remote-build. Core22 and older
           snaps may use the new or fallback remote-build.
        2. Envvar: If the envvar "SNAPCRAFT_REMOTE_BUILD_STRATEGY" is "force-fallback",
           the fallback remote-build is used. If it is "disable-fallback", the new
           remote-build code is used.
        3. Repo: If the project is in a git repository, the new remote-build is used.
           Otherwise, the fallback remote-build is used.

        :param base: The effective base of the project.
        """
        # bases newer than core22 must use the new remote-build
        if base in yaml_utils.CURRENT_BASES - {"core22"}:
            emit.debug("Running new remote-build because base is newer than core22.")
            self._run_new_remote_build()
            return

        strategy = self._get_build_strategy()

        if strategy == _Strategies.DISABLE_FALLBACK:
            emit.debug(
                "Running new remote-build because environment variable "
                f"{_STRATEGY_ENVVAR!r} is {_Strategies.DISABLE_FALLBACK.value!r}."
            )
            self._run_new_remote_build()
            return

        if strategy == _Strategies.FORCE_FALLBACK:
            emit.debug(
                "Running fallback remote-build because environment variable "
                f"{_STRATEGY_ENVVAR!r} is {_Strategies.FORCE_FALLBACK.value!r}."
            )
            run_legacy()
            return

        if is_repo(Path().absolute()):
            emit.debug(
                "Running new remote-build because project is in a git repository."
            )
            self._run_new_remote_build()
            return

        emit.debug("Running fallback remote-build.")
        run_legacy()

    def _get_project_name(self) -> str:
        """Get the project name from the project's snapcraft.yaml.

        :returns: The project name.

        :raises SnapcraftError: If the snapcraft.yaml does not contain a 'name' keyword.
        """
        with open(self._snapcraft_yaml, encoding="utf-8") as file:
            data = yaml_utils.safe_load(file)

        project_name = data.get("name")

        if project_name:
            emit.debug(
                f"Using project name {project_name!r} from "
                f"{str(self._snapcraft_yaml)!r}."
            )
            return project_name

        raise SnapcraftError(
            f"Could not get project name from {str(self._snapcraft_yaml)!r}."
        )

    def _run_new_remote_build(self) -> None:
        """Run new remote-build code."""
        # the build-id will be passed to the new remote-build code as part of #4323
        if self._parsed_args.build_id:
            build_id = self._parsed_args.build_id
            emit.debug(f"Using build ID {build_id!r} passed as a parameter.")
        else:
            build_id = get_build_id(
                app_name="snapcraft",
                project_name=self._get_project_name(),
                project_path=Path(),
            )
            emit.debug(f"Using computed build ID {build_id!r}.")

        # TODO: use new remote-build code (#4323)
        emit.debug(
            "Running fallback remote-build because new remote-build is not available."
        )
        run_legacy()

    def _get_build_strategy(self) -> Optional[_Strategies]:
        """Get the build strategy from the envvar `SNAPCRAFT_REMOTE_BUILD_STRATEGY`.

        :returns: The strategy or None.

        :raises SnapcraftError: If the variable is set to an invalid value.
        """
        strategy = os.getenv(_STRATEGY_ENVVAR)

        if not strategy:
            return None

        try:
            return _Strategies(strategy)
        except ValueError as err:
            valid_strategies = humanize_list(
                (strategy.value for strategy in _Strategies), "and"
            )
            raise SnapcraftError(
                f"Unknown value {strategy!r} in environment variable "
                f"{_STRATEGY_ENVVAR!r}. Valid values are {valid_strategies}."
            ) from err

    def _get_effective_base(self) -> str:
        """Get a valid effective base from the project's snapcraft.yaml.

        :returns: The project's effective base.

        :raises SnapcraftError: If the base is unknown or missing.
        :raises MaintenanceBase: If the base is not supported
        """
        with open(self._snapcraft_yaml, encoding="utf-8") as file:
            base = yaml_utils.get_base(file)

        if base is None:
            raise SnapcraftError(
                f"Could not determine base from {str(self._snapcraft_yaml)!r}."
            )

        emit.debug(f"Got base {base!r} from {str(self._snapcraft_yaml)!r}.")

        if base in yaml_utils.ESM_BASES:
            raise MaintenanceBase(base)

        if base not in yaml_utils.BASES:
            raise SnapcraftError(
                f"Unknown base {base!r} in {str(self._snapcraft_yaml)!r}."
            )

        return base


def _get_esm_warning_for_base(base: str) -> str:
    """Return a warning appropriate for the base under ESM."""
    channel: Optional[str] = None
    match base:
        case "core":
            channel = "4.x"
            version = "4"
        case "core18":
            channel = "7.x"
            version = "7"
        case _:
            raise RuntimeError(f"Unmatched base {base!r}")

    return (
        f"WARNING: base {base!r} was last supported on Snapcraft {version} available "
        f"on the {channel!r} channel."
    )
