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

"""Unimplemented commands that should be sent to  instead."""

import argparse
from typing import final

from snapcraft import commands, errors

# pylint: disable=missing-class-docstring


class UnimplementedMixin:
    """A mixin that allows you to declare a command unimplemented.

    Lets us scaffold the snapcraft help but then fall back
    """

    @final
    def run(self, parsed_args: argparse.Namespace) -> None:
        """Execute a command's functionality."""
        raise errors.ClassicFallback()


class ExportLogin(
    UnimplementedMixin, commands.StoreExportLoginCommand
):  # noqa: D101 (missing docstring)
    pass


class Login(
    UnimplementedMixin, commands.StoreLoginCommand
):  # noqa: D101 (missing docstring)
    pass


class Logout(
    UnimplementedMixin, commands.StoreLogoutCommand
):  # noqa: D101 (missing docstring)
    pass


class Whoami(
    UnimplementedMixin, commands.StoreWhoAmICommand
):  # noqa: D101 (missing docstring)
    pass


class ListPlugins(
    UnimplementedMixin, commands.ListPluginsCommand
):  # noqa: D101 (missing docstring)
    pass


class Plugins(
    UnimplementedMixin, commands.PluginsCommand
):  # noqa: D101 (missing docstring)
    pass


class ExpandExtensions(
    UnimplementedMixin, commands.ExpandExtensionsCommand
):  # noqa: D101 (missing docstring)
    pass


class ListExtensions(
    UnimplementedMixin, commands.ListExtensionsCommand
):  # noqa: D101 (missing docstring)
    pass


class Extensions(
    UnimplementedMixin, commands.ExtensionsCommand
):  # noqa: D101 (missing docstring)
    pass


class Init(UnimplementedMixin, commands.InitCommand):  # noqa: D101 (missing docstring)
    pass


class CreateKey(
    UnimplementedMixin, commands.StoreLegacyCreateKeyCommand
):  # noqa: D101 (missing docstring)
    pass


class Gated(
    UnimplementedMixin, commands.StoreLegacyGatedCommand
):  # noqa: D101 (missing docstring)
    pass


class ListKeys(
    UnimplementedMixin, commands.StoreLegacyListKeysCommand
):  # noqa: D101 (missing docstring)
    pass


class ListValidationSets(
    UnimplementedMixin, commands.StoreLegacyListValidationSetsCommand
):  # noqa: D101 (missing docstring)
    pass


class Metrics(
    UnimplementedMixin, commands.StoreLegacyMetricsCommand
):  # noqa: D101 (missing docstring)
    pass


class Promote(
    UnimplementedMixin, commands.StoreLegacyPromoteCommand
):  # noqa: D101 (missing docstring)
    pass


class RegisterKey(
    UnimplementedMixin, commands.StoreLegacyRegisterKeyCommand
):  # noqa: D101 (missing docstring)
    pass


class SetDefaultTrack(
    UnimplementedMixin, commands.StoreLegacySetDefaultTrackCommand
):  # noqa: D101 (missing docstring)
    pass


class SignBuild(
    UnimplementedMixin, commands.StoreLegacySignBuildCommand
):  # noqa: D101 (missing docstring)
    pass


class UploadMetadata(
    UnimplementedMixin, commands.StoreLegacyUploadMetadataCommand
):  # noqa: D101 (missing docstring)
    pass


class Validate(
    UnimplementedMixin, commands.StoreLegacyValidateCommand
):  # noqa: D101 (missing docstring)
    pass


class Build(
    UnimplementedMixin, commands.BuildCommand
):  # noqa: D101 (missing docstring)
    pass


class Clean(
    UnimplementedMixin, commands.CleanCommand
):  # noqa: D101 (missing docstring)
    pass


class Pack(UnimplementedMixin, commands.PackCommand):  # noqa: D101 (missing docstring)
    pass


class Prime(
    UnimplementedMixin, commands.PrimeCommand
):  # noqa: D101 (missing docstring)
    pass


class Pull(UnimplementedMixin, commands.PullCommand):  # noqa: D101 (missing docstring)
    pass


class Snap(UnimplementedMixin, commands.SnapCommand):  # noqa: D101 (missing docstring)
    pass


class Stage(
    UnimplementedMixin, commands.StageCommand
):  # noqa: D101 (missing docstring)
    pass


class Try(UnimplementedMixin, commands.TryCommand):  # noqa: D101 (missing docstring)
    pass


class Lint(UnimplementedMixin, commands.LintCommand):  # noqa: D101 (missing docstring)
    pass


class Close(
    UnimplementedMixin, commands.StoreCloseCommand
):  # noqa: D101 (missing docstring)
    pass


class Release(
    UnimplementedMixin, commands.StoreReleaseCommand
):  # noqa: D101 (missing docstring)
    pass


class List(
    UnimplementedMixin, commands.StoreLegacyListCommand
):  # noqa: D101 (missing docstring)
    pass


class ListRegistered(
    UnimplementedMixin, commands.StoreLegacyListRegisteredCommand
):  # noqa: D101 (missing docstring)
    pass


class Names(
    UnimplementedMixin, commands.StoreNamesCommand
):  # noqa: D101 (missing docstring)
    pass


class Register(
    UnimplementedMixin, commands.StoreRegisterCommand
):  # noqa: D101 (missing docstring)
    pass


class ListRevisions(
    UnimplementedMixin, commands.StoreListRevisionsCommand
):  # noqa: D101 (missing docstring)
    pass


class ListTracks(
    UnimplementedMixin, commands.StoreListTracksCommand
):  # noqa: D101 (missing docstring)
    pass


class Revisions(
    UnimplementedMixin, commands.StoreRevisionsCommand
):  # noqa: D101 (missing docstring)
    pass


class Status(
    UnimplementedMixin, commands.StoreStatusCommand
):  # noqa: D101 (missing docstring)
    pass


class Tracks(
    UnimplementedMixin, commands.StoreTracksCommand
):  # noqa: D101 (missing docstring)
    pass


class Push(
    UnimplementedMixin, commands.StoreLegacyPushCommand
):  # noqa: D101 (missing docstring)
    pass


class Upload(
    UnimplementedMixin, commands.StoreUploadCommand
):  # noqa: D101 (missing docstring)
    pass


class EditValidationSets(
    UnimplementedMixin, commands.StoreEditValidationSetsCommand
):  # noqa: D101 (missing docstring)
    pass


class Version(
    UnimplementedMixin, commands.VersionCommand
):  # noqa: D101 (missing docstring)
    pass


class RemoteBuild(
    UnimplementedMixin, commands.RemoteBuildCommand
):  # noqa: D101 (missing docstring)
    pass
