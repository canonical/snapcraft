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

"""Snapcraft lifecycle commands."""

import argparse
import os
import pathlib
import shutil
import textwrap
from typing import Any, cast

import craft_application.commands
from craft_cli import emit
from typing_extensions import override

import snapcraft.errors
import snapcraft.pack
from snapcraft import errors, utils
from snapcraft.models.project import Project


def _add_ua_args(parser: argparse.ArgumentParser) -> None:
    """Add hidden UA args."""
    parser.add_argument(
        "--ua-token",
        type=str,
        metavar="ua-token",
        default=None,
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--enable-experimental-ua-services",
        action="store_true",
        default=False,
        help=argparse.SUPPRESS,
    )


def _validate_ua_args(parsed_args: argparse.Namespace) -> None:
    """Validate if UA args or environment variables are used.

    Using a UA command line argument is an error, but the UA environment variable
    will only emit a warning.
    """
    if os.environ.get("SNAPCRAFT_UA_TOKEN"):
        emit.warning(
            "Ignoring the 'SNAPCRAFT_UA_TOKEN' environment variable. "
            "The Pro token attached to the host will be used instead."
        )

    if parsed_args.ua_token is not None:
        raise errors.SnapcraftError(
            "'--ua-token' is not supported for this base.",
            details="The Pro token attached to the host is used instead.",
            resolution="Remove the '--ua-token' argument.",
        )

    if parsed_args.enable_experimental_ua_services:
        raise errors.SnapcraftError(
            "Pro support is stable for this base.",
            resolution="Remove the '--enable-experimental-ua-services' argument.",
        )


class PullCommand(craft_application.commands.lifecycle.PullCommand):
    """Snapcraft pull command."""

    @override
    def _fill_parser(self, parser: argparse.ArgumentParser) -> None:
        super()._fill_parser(parser)
        _add_ua_args(parser)

    @override
    def _run(
        self,
        parsed_args: argparse.Namespace,
        step_name: str | None = None,
        **kwargs: Any,
    ) -> None:
        _validate_ua_args(parsed_args)
        super()._run(parsed_args, step_name=step_name, **kwargs)


class BuildCommand(craft_application.commands.lifecycle.BuildCommand):
    """Snapcraft build command."""

    @override
    def _fill_parser(self, parser: argparse.ArgumentParser) -> None:
        super()._fill_parser(parser)
        _add_ua_args(parser)

    @override
    def _run(
        self,
        parsed_args: argparse.Namespace,
        step_name: str | None = None,
        **kwargs: Any,
    ) -> None:
        _validate_ua_args(parsed_args)
        super()._run(parsed_args, step_name=step_name, **kwargs)


class StageCommand(craft_application.commands.lifecycle.StageCommand):
    """Snapcraft stage command."""

    @override
    def _fill_parser(self, parser: argparse.ArgumentParser) -> None:
        super()._fill_parser(parser)
        _add_ua_args(parser)

    @override
    def _run(
        self,
        parsed_args: argparse.Namespace,
        step_name: str | None = None,
        **kwargs: Any,
    ) -> None:
        _validate_ua_args(parsed_args)
        super()._run(parsed_args, step_name=step_name, **kwargs)


class PrimeCommand(craft_application.commands.lifecycle.PrimeCommand):
    """Snapcraft prime command."""

    @override
    def _fill_parser(self, parser: argparse.ArgumentParser) -> None:
        super()._fill_parser(parser)
        _add_ua_args(parser)

    @override
    def _run(
        self,
        parsed_args: argparse.Namespace,
        step_name: str | None = None,
        **kwargs: Any,
    ) -> None:
        _validate_ua_args(parsed_args)
        super()._run(parsed_args, step_name=step_name, **kwargs)


class CleanCommand(craft_application.commands.lifecycle.CleanCommand):
    """Snapcraft clean command."""

    @override
    def _fill_parser(self, parser: argparse.ArgumentParser) -> None:
        super()._fill_parser(parser)
        _add_ua_args(parser)

    @override
    def _run(self, parsed_args: argparse.Namespace, **kwargs: Any) -> None:
        _validate_ua_args(parsed_args)
        super()._run(parsed_args, **kwargs)


class PackCommand(craft_application.commands.lifecycle.PackCommand):
    """Snapcraft pack command."""

    name = "pack"
    help_msg = "Create the final artifact"
    overview = textwrap.dedent(
        """
        Process parts and create a snap file containing the project payload
        with the provided metadata. If a directory is specified, pack its
        contents instead.
        """
    )

    @override
    def _fill_parser(self, parser: argparse.ArgumentParser) -> None:
        """Add arguments specific to the pack command."""
        super()._fill_parser(parser)
        _add_ua_args(parser)

        parser.add_argument(
            "directory",
            metavar="directory",
            type=str,
            nargs="?",
            default=None,
            help="Directory to pack",
        )

    @override
    def _run(
        self,
        parsed_args: argparse.Namespace,
        step_name: str | None = None,
        **kwargs: Any,
    ) -> None:
        """Pack a directory or run the lifecycle and pack all artifacts."""
        if parsed_args.directory:
            emit.progress("Packing...")
            snap_filename = snapcraft.pack.pack_snap(
                parsed_args.directory, output=str(parsed_args.output)
            )
            emit.message(f"Packed {snap_filename}")
        else:
            _validate_ua_args(parsed_args)
            # The managed instance runs this same command line, but only the
            # project directory is shared with it, so an absolute output path
            # points somewhere the host can't see. Pack into the project
            # directory instead and let the host move the artifacts.
            use_provider = self._use_provider(parsed_args)
            output = pathlib.Path(getattr(parsed_args, "output", None) or ".")
            if utils.is_managed_mode() and output.is_absolute():
                parsed_args.output = pathlib.Path()
            super()._run(parsed_args)
            if use_provider and output.is_absolute():
                project_dir = getattr(parsed_args, "project_dir", None)
                self._move_artifacts(
                    project_dir=pathlib.Path(project_dir or pathlib.Path.cwd()),
                    output=output,
                )

    def _move_artifacts(
        self, *, project_dir: pathlib.Path, output: pathlib.Path
    ) -> None:
        """Move artifacts packed in the managed instance to the requested output.

        The output follows the same rules as packing on the host: an existing
        directory receives the artifacts, anything else names the snap file and
        components go next to it.
        """
        output_dir = output if output.is_dir() else output.parent
        package_service = self._services.get("package")
        found_any = False
        for build_info in self._services.get("build_plan").plan():
            try:
                artifacts = package_service.read_state(build_info.platform).artifacts
            except KeyError:
                continue
            found_any = True
            for artifact in artifacts:
                old_path = project_dir / artifact.path
                if artifact.name is None and not output.is_dir():
                    new_path = output
                else:
                    new_path = output_dir / artifact.path.name
                if old_path.resolve() == new_path.resolve():
                    continue
                new_path.parent.mkdir(parents=True, exist_ok=True)
                shutil.move(old_path, new_path)
                emit.progress(
                    f"Moved {artifact.path.name} to {new_path}", permanent=True
                )
        if not found_any:
            emit.debug("Could not find artifacts in the state service. Not moving.")

    @override
    def needs_project(self, parsed_args: argparse.Namespace) -> bool:
        """Project is not required to pack a directory."""
        if parsed_args.directory:
            emit.debug("Not loading project because a directory was provided.")
            return False

        emit.debug("Loading project because a directory was not provided.")
        return True

    @override
    def _use_provider(self, parsed_args: argparse.Namespace) -> bool:
        """Return whether the command should run in managed mode or not.

        Packing a directory always runs locally.
        """
        if parsed_args.directory:
            emit.debug("Not running managed mode because a directory was provided.")
            return False

        return super()._use_provider(parsed_args)


class TryCommand(PackCommand):
    """Prepare the parts for ``snap try``."""

    name = "try"
    help_msg = 'Prepare a snap for "snap try".'
    overview = textwrap.dedent(
        """
        Process parts and expose the ``prime`` directory containing the
        final payload, in preparation for ``snap try prime``.
        """
    )

    @override
    def _use_provider(self, parsed_args: argparse.Namespace) -> bool:
        """Overridden to return false, such that the command fails early."""
        return False

    @override
    def _run(
        self,
        parsed_args: argparse.Namespace,
        step_name: str | None = None,
        **kwargs: Any,
    ) -> None:
        project = cast(Project, self.services.get("project").get())
        effective_base = project.get_effective_base()

        raise snapcraft.errors.FeatureNotImplemented(
            f"'snapcraft try' is not implemented for {effective_base!r}"
        )


class SnapCommand(PackCommand):
    """Removed command to pack the final snap payload."""

    name = "snap"
    hidden = True

    @override
    def _run(
        self,
        parsed_args: argparse.Namespace,
        step_name: str | None = None,
        **kwargs: Any,
    ) -> None:
        raise errors.RemovedCommand(removed_command=self.name, new_command=super().name)
