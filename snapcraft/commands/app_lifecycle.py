# Copyright 2023 Canonical Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU Lesser General Public License version 3, as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranties of MERCHANTABILITY,
# SATISFACTORY QUALITY, or FITNESS FOR A PARTICULAR PURPOSE.
# See the GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.
"""Basic lifecycle commands for a Craft Application."""
from __future__ import annotations

import os
import pathlib
import subprocess
import textwrap
from typing import TYPE_CHECKING

from craft_cli import CommandGroup, emit
from craft_parts.features import Features
from typing_extensions import override

from craft_application.commands import base

if TYPE_CHECKING:  # pragma: no cover
    import argparse


def get_lifecycle_command_group() -> CommandGroup:
    """Return the lifecycle related command group."""
    commands: list[type[_LifecycleCommand]] = [
        CleanCommand,
        PullCommand,
        OverlayCommand,
        BuildCommand,
        StageCommand,
        PrimeCommand,
        PackCommand,
    ]
    if not Features().enable_overlay:
        commands.remove(OverlayCommand)

    return CommandGroup(
        "Lifecycle",
        commands,  # type: ignore[arg-type] # https://github.com/canonical/craft-cli/pull/157
    )


class _LifecycleCommand(base.AppCommand):
    """Lifecycle-related commands."""

    @override
    def run(self, parsed_args: argparse.Namespace) -> None:
        emit.trace(f"lifecycle command: {self.name!r}, arguments: {parsed_args!r}")


class _LifecyclePartsCommand(_LifecycleCommand):
    # All lifecycle-related commands need a project to work
    always_load_project = True

    @override
    def fill_parser(self, parser: argparse.ArgumentParser) -> None:
        super().fill_parser(parser)  # type: ignore[arg-type]
        parser.add_argument(
            "parts",
            metavar="part-name",
            type=str,
            nargs="*",
            help="Optional list of parts to process",
        )
        parser.add_argument(
            "--destructive-mode",
            action="store_true",
            help="Build in the current host",
        )

    @override
    def get_managed_cmd(self, parsed_args: argparse.Namespace) -> list[str]:
        cmd = super().get_managed_cmd(parsed_args)

        cmd.extend(parsed_args.parts)

        return cmd


class _LifecycleStepCommand(_LifecyclePartsCommand):
    @override
    def run_managed(self, parsed_args: argparse.Namespace) -> bool:
        return not parsed_args.destructive_mode

    @override
    def fill_parser(self, parser: argparse.ArgumentParser) -> None:
        super().fill_parser(parser)

        if self._should_add_shell_args():
            group = parser.add_mutually_exclusive_group()
            group.add_argument(
                "--shell",
                action="store_true",
                help="Shell into the environment in lieu of the step to run.",
            )
            group.add_argument(
                "--shell-after",
                action="store_true",
                help="Shell into the environment after the step has run.",
            )

        parser.add_argument(
            "--debug",
            action="store_true",
            help="Shell into the environment if the build fails.",
        )

        group = parser.add_mutually_exclusive_group()
        group.add_argument(
            "--platform",
            type=str,
            metavar="name",
            default=os.getenv("CRAFT_PLATFORM"),
            help="Set platform to build for",
        )
        group.add_argument(
            "--build-for",
            type=str,
            metavar="arch",
            default=os.getenv("CRAFT_BUILD_FOR"),
            help="Set architecture to build for",
        )

    @override
    def get_managed_cmd(self, parsed_args: argparse.Namespace) -> list[str]:
        """Get the command to run in managed mode.

        :param parsed_args: The parsed arguments used.
        :returns: A list of strings ready to be passed into a craft-providers executor.
        :raises: RuntimeError if this command is not supposed to run managed.
        """
        cmd = super().get_managed_cmd(parsed_args)

        if getattr(parsed_args, "shell", False):
            cmd.append("--shell")
        if getattr(parsed_args, "shell_after", False):
            cmd.append("--shell-after")

        return cmd

    @override
    def run(
        self, parsed_args: argparse.Namespace, step_name: str | None = None
    ) -> None:
        """Run a lifecycle step command."""
        super().run(parsed_args)

        shell = getattr(parsed_args, "shell", False)
        shell_after = getattr(parsed_args, "shell_after", False)
        debug = getattr(parsed_args, "debug", False)

        step_name = step_name or self.name

        if shell:
            previous_step = self._services.lifecycle.previous_step_name(step_name)
            step_name = previous_step
            shell_after = True

        try:
            self._services.lifecycle.run(
                step_name=step_name,
                part_names=parsed_args.parts,
            )
        except Exception as err:
            if debug:
                emit.progress(str(err), permanent=True)
                _launch_shell()
            raise

        if shell_after:
            _launch_shell()

    @staticmethod
    def _should_add_shell_args() -> bool:
        return True


class PullCommand(_LifecycleStepCommand):
    """Command to pull parts."""

    name = "pull"
    help_msg = "Download or retrieve artifacts defined for a part"
    overview = textwrap.dedent(
        """
        Download or retrieve artifacts defined for a part. If part names
        are specified only those parts will be pulled, otherwise all parts
        will be pulled.
        """
    )


class OverlayCommand(_LifecycleStepCommand):
    """Command to overlay parts."""

    name = "overlay"
    help_msg = "Create part layers over the base filesystem."
    overview = textwrap.dedent(
        """
        Execute operations defined for each part on a layer over the base
        filesystem, potentially modifying its contents.
        """
    )


class BuildCommand(_LifecycleStepCommand):
    """Command to build parts."""

    name = "build"
    help_msg = "Build artifacts defined for a part"
    overview = textwrap.dedent(
        """
        Build artifacts defined for a part. If part names are specified only
        those parts will be built, otherwise all parts will be built.
        """
    )


class StageCommand(_LifecycleStepCommand):
    """Command to stage parts."""

    name = "stage"
    help_msg = "Stage built artifacts into a common staging area"
    overview = textwrap.dedent(
        """
        Stage built artifacts into a common staging area. If part names are
        specified only those parts will be staged. The default is to stage
        all parts.
        """
    )


class PrimeCommand(_LifecycleStepCommand):
    """Command to prime parts."""

    name = "prime"
    help_msg = "Prime artifacts defined for a part"
    overview = textwrap.dedent(
        """
        Prepare the final payload to be packed, performing additional
        processing and adding metadata files. If part names are specified only
        those parts will be primed. The default is to prime all parts.
        """
    )

    @override
    def run(
        self, parsed_args: argparse.Namespace, step_name: str | None = None
    ) -> None:
        """Run the prime command."""
        super().run(parsed_args, step_name=step_name)

        self._services.package.write_metadata(self._services.lifecycle.prime_dir)


class PackCommand(PrimeCommand):
    """Command to pack the final artifact."""

    name = "pack"
    help_msg = "Create the final artifact"
    overview = textwrap.dedent(
        """
        Process parts and create the final artifact.
        """
    )

    @override
    def fill_parser(self, parser: argparse.ArgumentParser) -> None:
        super().fill_parser(parser)
        parser.add_argument(
            "directory",
            metavar="directory",
            type=str,
            nargs="?",
            default=None,
            help="Directory to pack",
        ) # TODO: actually use this
        parser.add_argument(
            "--output",
            "-o",
            type=pathlib.Path,
            default=pathlib.Path(),
            help="Output directory for created packages.",
        )

    @override
    def run(
        self, parsed_args: argparse.Namespace, step_name: str | None = None
    ) -> None:
        """Run the pack command."""
        if step_name not in ("pack", None):
            raise RuntimeError(f"Step name {step_name} passed to pack command.")
        super().run(parsed_args, step_name="prime")

        emit.progress("Packing...")
        packages = self._services.package.pack(
            self._services.lifecycle.prime_dir, parsed_args.output
        )

        if not packages:
            emit.progress("No packages created.", permanent=True)
        elif len(packages) == 1:
            emit.progress(f"Packed {packages[0].name}", permanent=True)
        else:
            package_names = ", ".join(pkg.name for pkg in packages)
            emit.progress(f"Packed: {package_names}", permanent=True)

    @staticmethod
    @override
    def _should_add_shell_args() -> bool:
        return False


class CleanCommand(_LifecyclePartsCommand):
    """Command to remove part assets."""

    name = "clean"
    help_msg = "Remove a part's assets"
    overview = textwrap.dedent(
        """
        Clean up artifacts belonging to parts. If no parts are specified,
        remove the packing environment.
        """
    )

    @override
    def run(self, parsed_args: argparse.Namespace) -> None:
        """Run the clean command."""
        super().run(parsed_args)

        if parsed_args.destructive_mode or not self._should_clean_instances(
            parsed_args
        ):
            self._services.lifecycle.clean(parsed_args.parts)
        else:
            self._services.provider.clean_instances()

    @override
    def run_managed(self, parsed_args: argparse.Namespace) -> bool:
        if parsed_args.destructive_mode:
            # In destructive mode, always run on the host.
            return False

        # "clean" should run managed if cleaning specific parts.
        # otherwise, should run on the host to clean the build provider.
        return not self._should_clean_instances(parsed_args)

    @staticmethod
    def _should_clean_instances(parsed_args: argparse.Namespace) -> bool:
        return not bool(parsed_args.parts)


def _launch_shell() -> None:
    """Launch a user shell for debugging environment."""
    emit.progress("Launching shell on build environment...", permanent=True)
    with emit.pause():
        subprocess.run(["bash"], check=False)
