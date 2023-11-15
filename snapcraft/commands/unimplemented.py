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


class UnimplementedMixin:
    """A mixin that allows you to declare a command unimplemented.

    Lets us scaffold the snapcraft help but then fall back
    """

    @final
    def run(self, parsed_args: argparse.Namespace) -> None:
        raise errors.ClassicFallback()


class ExportLogin(UnimplementedMixin, commands.StoreExportLoginCommand):
    pass


class Login(UnimplementedMixin, commands.StoreLoginCommand):
    pass


class Logout(UnimplementedMixin, commands.StoreLogoutCommand):
    pass


class Whoami(UnimplementedMixin, commands.StoreWhoAmICommand):
    pass


class ListPlugins(UnimplementedMixin, commands.ListPluginsCommand):
    pass


class Plugins(UnimplementedMixin, commands.PluginsCommand):
    pass


class ExpandExtensions(UnimplementedMixin, commands.ExpandExtensionsCommand):
    pass


class ListExtensions(UnimplementedMixin, commands.ListExtensionsCommand):
    pass


class Extensions(UnimplementedMixin, commands.ExtensionsCommand):
    pass


class Init(UnimplementedMixin, commands.InitCommand):
    pass


class CreateKey(UnimplementedMixin, commands.StoreLegacyCreateKeyCommand):
    pass


class Gated(UnimplementedMixin, commands.StoreLegacyGatedCommand):
    pass


class ListKeys(UnimplementedMixin, commands.StoreLegacyListKeysCommand):
    pass


class ListValidationSets(
    UnimplementedMixin, commands.StoreLegacyListValidationSetsCommand
):
    pass


class Metrics(UnimplementedMixin, commands.StoreLegacyMetricsCommand):
    pass


class Promote(UnimplementedMixin, commands.StoreLegacyPromoteCommand):
    pass


class RegisterKey(UnimplementedMixin, commands.StoreLegacyRegisterKeyCommand):
    pass


class SetDefaultTrack(UnimplementedMixin, commands.StoreLegacySetDefaultTrackCommand):
    pass


class SignBuild(UnimplementedMixin, commands.StoreLegacySignBuildCommand):
    pass


class UploadMetadata(UnimplementedMixin, commands.StoreLegacyUploadMetadataCommand):
    pass


class Validate(UnimplementedMixin, commands.StoreLegacyValidateCommand):
    pass


class Build(UnimplementedMixin, commands.BuildCommand):
    pass


class Clean(UnimplementedMixin, commands.CleanCommand):
    pass


class Pack(UnimplementedMixin, commands.PackCommand):
    pass


class Prime(UnimplementedMixin, commands.PrimeCommand):
    pass


class Pull(UnimplementedMixin, commands.PullCommand):
    pass


class Snap(UnimplementedMixin, commands.SnapCommand):
    pass


class Stage(UnimplementedMixin, commands.StageCommand):
    pass


class Try(UnimplementedMixin, commands.TryCommand):
    pass


class Lint(UnimplementedMixin, commands.LintCommand):
    pass


class Close(UnimplementedMixin, commands.StoreCloseCommand):
    pass


class Release(UnimplementedMixin, commands.StoreReleaseCommand):
    pass


class List(UnimplementedMixin, commands.StoreLegacyListCommand):
    pass


class ListRegistered(UnimplementedMixin, commands.StoreLegacyListRegisteredCommand):
    pass


class Names(UnimplementedMixin, commands.StoreNamesCommand):
    pass


class Register(UnimplementedMixin, commands.StoreRegisterCommand):
    pass


class ListRevisions(UnimplementedMixin, commands.StoreListRevisionsCommand):
    pass


class ListTracks(UnimplementedMixin, commands.StoreListTracksCommand):
    pass


class Revisions(UnimplementedMixin, commands.StoreRevisionsCommand):
    pass


class Status(UnimplementedMixin, commands.StoreStatusCommand):
    pass


class Tracks(UnimplementedMixin, commands.StoreTracksCommand):
    pass


class Push(UnimplementedMixin, commands.StoreLegacyPushCommand):
    pass


class Upload(UnimplementedMixin, commands.StoreUploadCommand):
    pass


class EditValidationSets(UnimplementedMixin, commands.StoreEditValidationSetsCommand):
    pass


class RemoteBuild(UnimplementedMixin, commands.RemoteBuildCommand):
    pass
