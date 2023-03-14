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

"""Snapcraft Store uploading related commands."""

import pathlib
import textwrap
from typing import TYPE_CHECKING, List, Optional

from craft_cli import BaseCommand, emit
from craft_cli.errors import ArgumentParsingError
from overrides import overrides
from requests_toolbelt import MultipartEncoder, MultipartEncoderMonitor

from snapcraft import store, utils
from snapcraft_legacy._store import get_data_from_snap_file

if TYPE_CHECKING:
    import argparse


class StoreUploadCommand(BaseCommand):
    """Upload a snap to the Snap Store."""

    name = "upload"
    help_msg = "Upload a snap to the Snap Store"
    overview = textwrap.dedent(
        """
        By passing --release with a comma-separated list of channels the snap would
        be released to the selected channels if the store review passes for this
        <snap-file>.

        This operation will block until the store finishes processing this <snap-
        file>.

        If --release is used, the channel map will be displayed after the operation
        takes place.
        """
    )

    @overrides
    def fill_parser(self, parser: "argparse.ArgumentParser") -> None:
        parser.add_argument(
            "snap_file",
            metavar="snap-file",
            type=str,
            help="Snap to upload",
        )
        parser.add_argument(
            "--release",
            metavar="channels",
            dest="channels",
            type=str,
            default=None,
            help="Optional comma-separated list of channels to release to",
        )

    @overrides
    def run(self, parsed_args):
        snap_file = pathlib.Path(parsed_args.snap_file)
        if not snap_file.exists() or not snap_file.is_file():
            raise ArgumentParsingError(f"{str(snap_file)!r} is not a valid file")

        channels: Optional[List[str]] = None
        if parsed_args.channels:
            channels = parsed_args.channels.split(",")

        client = store.StoreClientCLI()

        snap_yaml = get_data_from_snap_file(snap_file)
        snap_name = snap_yaml["name"]
        built_at = snap_yaml.get("snapcraft-started-at")

        client.verify_upload(snap_name=snap_name)

        upload_id = client.store_client.upload_file(
            filepath=snap_file, monitor_callback=create_callback
        )

        revision = client.notify_upload(
            snap_name=snap_name,
            upload_id=upload_id,
            built_at=built_at,
            channels=channels,
            snap_file_size=snap_file.stat().st_size,
        )

        message = f"Revision {revision!r} created for {snap_name!r}"
        if channels:
            message += f" and released to {utils.humanize_list(channels, 'and')}"
        emit.message(message)


def create_callback(encoder: MultipartEncoder):
    """Create a callback suitable for upload_file."""
    with emit.progress_bar("Uploading...", encoder.len, delta=False) as progress:

        def progress_callback(monitor: MultipartEncoderMonitor):
            progress.advance(monitor.bytes_read)

        return progress_callback


class StoreLegacyPushCommand(StoreUploadCommand):
    """Legacy command to upload a snap to the Snap Store."""

    name = "push"
    hidden = True
