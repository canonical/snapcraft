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

"""Unimplemented commands that should be sent to snapcraft's cli handler instead."""

import argparse
from typing import final

from snapcraft import commands, errors

# pylint: disable=missing-class-docstring,unused-argument


class UnimplementedMixin:
    """A mixin that allows you to declare a command unimplemented.

    Lets us scaffold the snapcraft help but then fall back
    """

    @final
    def run(self, parsed_args: argparse.Namespace) -> None:
        """Execute a command's functionality."""
        if self.config["core24"]:  # type: ignore[attr-defined]
            # We know that this is a core24 project, but the actual command has
            # not yet been ported.
            command_name = self.name  # type: ignore[attr-defined]
            raise RuntimeError(
                f'"{command_name}" command is not implemented for core24!'
            )

        # Fallback to the codepaths for non-core24-code.
        raise errors.ClassicFallback()

    def run_managed(
        self,
        parsed_args: argparse.Namespace,  # noqa: ARG002 (the unused argument is for subclasses)
    ) -> bool:
        """Overridden to always return False, for now."""
        return False

    def provider_name(
        self,
        parsed_args: argparse.Namespace,  # noqa: ARG002 (the unused argument is for subclasses)
    ) -> str | None:
        """Name of the provider where the command should be run inside of.

        By default returns None. Subclasses can override this method to change this,
        including by inspecting the arguments in `parsed_args`.
        """
        return None

    always_load_project: bool = False


class ListPlugins(
    UnimplementedMixin, commands.core22.ListPluginsCommand
):  # noqa: D101 (missing docstring)
    pass


class Plugins(
    UnimplementedMixin, commands.core22.PluginsCommand
):  # noqa: D101 (missing docstring)
    pass


class ExpandExtensions(
    UnimplementedMixin, commands.core22.ExpandExtensionsCommand
):  # noqa: D101 (missing docstring)
    pass


class ListExtensions(
    UnimplementedMixin, commands.core22.ListExtensionsCommand
):  # noqa: D101 (missing docstring)
    pass


class Extensions(
    UnimplementedMixin, commands.core22.ExtensionsCommand
):  # noqa: D101 (missing docstring)
    pass


class Init(
    UnimplementedMixin, commands.core22.InitCommand
):  # noqa: D101 (missing docstring)
    pass


class CreateKey(
    UnimplementedMixin, commands.legacy.StoreLegacyCreateKeyCommand
):  # noqa: D101 (missing docstring)
    pass


class Gated(
    UnimplementedMixin, commands.legacy.StoreLegacyGatedCommand
):  # noqa: D101 (missing docstring)
    pass


class ListKeys(
    UnimplementedMixin, commands.legacy.StoreLegacyListKeysCommand
):  # noqa: D101 (missing docstring)
    pass


class ListValidationSets(
    UnimplementedMixin, commands.legacy.StoreLegacyListValidationSetsCommand
):  # noqa: D101 (missing docstring)
    pass


class Metrics(
    UnimplementedMixin, commands.legacy.StoreLegacyMetricsCommand
):  # noqa: D101 (missing docstring)
    pass


class Promote(
    UnimplementedMixin, commands.legacy.StoreLegacyPromoteCommand
):  # noqa: D101 (missing docstring)
    pass


class RegisterKey(
    UnimplementedMixin, commands.legacy.StoreLegacyRegisterKeyCommand
):  # noqa: D101 (missing docstring)
    pass


class SetDefaultTrack(
    UnimplementedMixin, commands.legacy.StoreLegacySetDefaultTrackCommand
):  # noqa: D101 (missing docstring)
    pass


class SignBuild(
    UnimplementedMixin, commands.legacy.StoreLegacySignBuildCommand
):  # noqa: D101 (missing docstring)
    pass


class UploadMetadata(
    UnimplementedMixin, commands.legacy.StoreLegacyUploadMetadataCommand
):  # noqa: D101 (missing docstring)
    pass


class Validate(
    UnimplementedMixin, commands.legacy.StoreLegacyValidateCommand
):  # noqa: D101 (missing docstring)
    pass


class Try(
    UnimplementedMixin, commands.core22.TryCommand
):  # noqa: D101 (missing docstring)
    pass


class Lint(
    UnimplementedMixin, commands.core22.LintCommand
):  # noqa: D101 (missing docstring)
    pass


class List(
    UnimplementedMixin, commands.legacy.StoreLegacyListCommand
):  # noqa: D101 (missing docstring)
    pass


class ListRegistered(
    UnimplementedMixin, commands.legacy.StoreLegacyListRegisteredCommand
):  # noqa: D101 (missing docstring)
    pass


class Push(
    UnimplementedMixin, commands.legacy.StoreLegacyPushCommand
):  # noqa: D101 (missing docstring)
    pass
