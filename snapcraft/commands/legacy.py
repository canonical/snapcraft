# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
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

"""Snapcraft Commands that call to the legacy implementation."""

import textwrap
from typing import TYPE_CHECKING

from craft_cli import BaseCommand
from overrides import overrides

from snapcraft.commands.store._legacy_account import set_legacy_env
from snapcraft.legacy_cli import run_legacy

if TYPE_CHECKING:
    import argparse


class LegacyBaseCommand(BaseCommand):
    """Legacy command runner."""

    @overrides
    def run(self, parsed_args):
        # Setup env var for legacy credentials.
        set_legacy_env()

        run_legacy()


#########
# Store #
#########


class StoreLegacyUploadMetadataCommand(LegacyBaseCommand):
    """Command passthrough for the upload-metadata command."""

    name = "upload-metadata"
    help_msg = "Upload metadata from <snap-file> to the store"
    overview = textwrap.dedent(
        """
        The following information will be retrieved from <snap-file> and used to
        update the store:

        - summary
        - description
        - icon

        If --force is given, it will force the local metadata into the Store,
        ignoring any possible conflict.

        Examples:
            snapcraft upload-metadata my-snap_0.1_amd64.snap
            snapcraft upload-metadata my-snap_0.1_amd64.snap --force
        """
    )

    @overrides
    def fill_parser(self, parser: "argparse.ArgumentParser") -> None:
        parser.add_argument(
            "--force",
            action="store_true",
            default=False,
            help="Force metadata update to override any possible conflict",
        )


class StoreLegacyPromoteCommand(LegacyBaseCommand):
    """Command passthrough for the promote command."""

    name = "promote"
    help_msg = "Promote a build set from a channel"
    overview = textwrap.dedent(
        """
        A build set is a set of commonly tagged revisions, the most simple
        form of a build set is a set of revisions released to a channel.

        Currently, only channels are supported to release from (<from-channel>)

        Prior to releasing, visual confirmation shall be required.

        The format for channels is `[<track>/]<risk>[/<branch>]` where

        - <track> is used to have long term release channels. It is implicitly
          set to the default.
        - <risk> is mandatory and can be either `stable`, `candidate`, `beta`
          or `edge`.
        - <branch> is optional and dynamically creates a channel with a
          specific expiration date.
        """
    )

    @overrides
    def fill_parser(self, parser: "argparse.ArgumentParser") -> None:
        parser.add_argument(
            "snap_name",
            metavar="snap-name",
        )
        parser.add_argument(
            "--from-channel",
            metavar="from-channel",
            help="the channel to promote from",
            required=True,
        )
        parser.add_argument(
            "--to-channel",
            metavar="to-channel",
            help="the channel to promote to",
            required=True,
        )
        parser.add_argument(
            "--yes", action="store_true", help="do not prompt for confirmation"
        )


class StoreLegacyListRevisionsCommand(LegacyBaseCommand):
    """Command passthrough for the list-revisions command."""

    name = "list-revisions"
    help_msg = "List published revisions for <snap-name>"
    overview = textwrap.dedent(
        """
        Examples:
            snapcraft list-revisions my-snap
            snapcraft list-revisions my-snap --arch armhf
            snapcraft revisions my-snap
        """
    )

    @overrides
    def fill_parser(self, parser: "argparse.ArgumentParser") -> None:
        parser.add_argument(
            "snap_name",
            metavar="snap-name",
        )
        parser.add_argument(
            "--arch",
            metavar="arch",
            help="architecture filter",
        )


class StoreLegacySetDefaultTrackCommand(LegacyBaseCommand):
    """Command passthrough for the set-default-track command."""

    name = "set-default-track"
    help_msg = "Set the default track for a snap"
    overview = textwrap.dedent(
        """
        Set the default track for <snap-name> to <track>. <track> must already exist."""
    )

    @overrides
    def fill_parser(self, parser: "argparse.ArgumentParser") -> None:
        parser.add_argument(
            "snap_name",
            metavar="snap-name",
        )
        parser.add_argument(
            "track",
        )


class StoreLegacyMetricsCommand(LegacyBaseCommand):
    """Command passthrough for the metrics command."""

    name = "metrics"
    help_msg = "Get metrics for a snap"
    overview = textwrap.dedent(
        """
        Get different metrics from the Snap Store for a given snap."""
    )

    @overrides
    def fill_parser(self, parser: "argparse.ArgumentParser") -> None:
        parser.add_argument("snap_name", metavar="snap-name")
        parser.add_argument("--name", metavar="name", required=True, help="metric name")
        parser.add_argument(
            "--start",
            metavar="start-date",
            help="date in format YYYY-MM-DD",
        )
        parser.add_argument(
            "--end",
            metavar="end-date",
            help="date in format YYYY-MM-DD",
        )
        parser.add_argument(
            "--format",
            metavar="format",
            help="format for output",
            choices=["table", "json"],
            required=True,
        )


#########
# Build #
#########


class StoreLegacyRemoteBuildCommand(LegacyBaseCommand):
    """Command passthrough for the remote-build command."""

    name = "remote-build"
    help_msg = "Dispatch a snap for remote build"
    overview = textwrap.dedent(
        """
        Command remote-build sends the current project to be built remotely. After the build
        is complete, packages for each architecture are retrieved and will be available in
        the local filesystem.

        If not specified in the snapcraft.yaml file, the list of architectures to build
        can be set using the --build-on option. If both are specified, an error will occur.

        Interrupted remote builds can be resumed using the --recover option, followed by
        the build number informed when the remote build was originally dispatched. The
        current state of the remote build for each architecture can be checked using the
        --status option."""
    )

    @overrides
    def fill_parser(self, parser: "argparse.ArgumentParser") -> None:
        parser.add_argument(
            "--recover", action="store_true", help="recover an interrupted build"
        )
        parser.add_argument(
            "--status", action="store_true", help="display remote build status"
        )
        parser.add_argument(
            "--build-on",
            metavar="arch",
            nargs="+",
            help="architecture to build on",
        )
        parser.add_argument(
            "--build-id", metavar="build-id", help="specific build id to retrieve"
        )
        parser.add_argument(
            "--launchpad-accept-public-upload",
            action="store_true",
            help="acknowledge that uploaded code will be publicly available.",
        )


##############
# Assertions #
##############


class StoreLegacyListKeysCommand(LegacyBaseCommand):
    """Command passthrough for the list-keys command."""

    name = "list-keys"
    help_msg = "List the keys available to sign assertions"
    overview = textwrap.dedent(
        """
        List the available keys to sign assertions together with their
        local availability."""
    )


class StoreLegacyCreateKeyCommand(LegacyBaseCommand):
    """Command passthrough for the create-key command."""

    name = "create-key"
    help_msg = "Create a key to sign assertions."
    overview = textwrap.dedent(
        """
        Create a key and store it locally. Use the register-key command to register
        it on the store."""
    )

    @overrides
    def fill_parser(self, parser: "argparse.ArgumentParser") -> None:
        parser.add_argument(
            "key_name", metavar="key-name", help="Key used to sign the assertion"
        )


class StoreLegacyRegisterKeyCommand(LegacyBaseCommand):
    """Command passthrough for the register-key command."""

    name = "register-key"
    help_msg = "Register a key to sign assertions with the Snap Store."
    overview = textwrap.dedent(
        """
        Register a a key with the Snap Store. Prior to registration, use register-key
        to create one."""
    )

    @overrides
    def fill_parser(self, parser: "argparse.ArgumentParser") -> None:
        parser.add_argument(
            "key_name", metavar="key-name", help="Key used to sign the assertion"
        )


class StoreLegacySignBuildCommand(LegacyBaseCommand):
    """Command passthrough for the sign-build command."""

    name = "sign-build"
    help_msg = "Sign a built snap file and assert it using the developer's key"
    overview = textwrap.dedent(
        """
        Sign a specific build of a snap with a given key and upload the assertion
        to the Snap Store (unless --local)."""
    )

    @overrides
    def fill_parser(self, parser: "argparse.ArgumentParser") -> None:
        parser.add_argument(
            "--key-name", metavar="key-name", help="key used to sign the assertion"
        )
        parser.add_argument(
            "--local",
            "--local",
            action="store_true",
            help="do not aupload to the Snap Store",
        )


class StoreLegacyValidateCommand(LegacyBaseCommand):
    """Command passthrough for the validate command."""

    name = "validate"
    help_msg = "Validate a gated snap"
    overview = textwrap.dedent(
        """
        Each validation can be presented with either syntax:

        -  <snap-name>=<revision>
        -  <snap-id>=<revision>"""
    )

    @overrides
    def fill_parser(self, parser: "argparse.ArgumentParser") -> None:
        parser.add_argument(
            "--key-name", metavar="key-name", help="key used to sign the assertion"
        )
        parser.add_argument("--revoke", action="store_true", help="revoke validations")
        parser.add_argument("snap_name", metavar="snap-name")
        parser.add_argument("validations", nargs="+")


class StoreLegacyGatedCommand(LegacyBaseCommand):
    """Command passthrough for the gated command."""

    name = "gated"
    help_msg = "List all gated snaps for <snap-name>"
    overview = textwrap.dedent(
        """
        Get the list of snaps and revisions gating a snaps"""
    )

    @overrides
    def fill_parser(self, parser: "argparse.ArgumentParser") -> None:
        parser.add_argument("snap_name", metavar="snap-name")


class StoreLegacyListValidationSetsCommand(LegacyBaseCommand):
    """Command passthrough for the edit-validation-sets command."""

    name = "list-validation-sets"
    help_msg = "Get the list of validation sets"
    overview = textwrap.dedent(
        """
        List all list-validation-sets snaps.
        """
    )

    @overrides
    def fill_parser(self, parser: "argparse.ArgumentParser") -> None:
        parser.add_argument("snap_name", metavar="snap-name")


class StoreLegacyEditValidationSetsCommand(LegacyBaseCommand):
    """Command passthrough for the edit-validation-sets command."""

    name = "edit-validation-sets"
    help_msg = "Edit the list of validations for <name>"
    overview = textwrap.dedent(
        """
        Refer to https://snapcraft.io/docs/validation-sets for further information
        on Validation Sets.
        """
    )

    @overrides
    def fill_parser(self, parser: "argparse.ArgumentParser") -> None:
        parser.add_argument(
            "--key-name", metavar="key-name", help="Key used to sign the assertion"
        )
        parser.add_argument("account_id", metavar="account-id")
        parser.add_argument("set_name", metavar="set-name")
        parser.add_argument("sequence", metavar="sequence")
