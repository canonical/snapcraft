# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022,2024 Canonical Ltd.
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

from __future__ import annotations

import pathlib
import textwrap
from typing import TYPE_CHECKING

from craft_application.commands import AppCommand
from craft_cli import emit
from craft_cli.errors import ArgumentParsingError
from overrides import overrides

from snapcraft import errors, store, utils
from snapcraft.meta import SnapMetadata
from snapcraft_legacy._store import get_data_from_snap_file

if TYPE_CHECKING:
    import argparse
    from collections import abc

    from requests_toolbelt import MultipartEncoder, MultipartEncoderMonitor


class ComponentOption:
    """Argparse helper to validate and convert a 'component' option.

    Example of use:

        parser.add_argument('--component',  type=ComponentOption())
    """

    name: str | None = None
    path: pathlib.Path | None = None

    def __init__(self, value: str) -> None:
        """Run by argparse to validate and convert the given argument."""
        parts = [p.strip() for p in value.split("=") if p.strip()]
        if len(parts) == 2:
            self.name = parts[0]
            self.path = pathlib.Path(parts[1])
        else:
            raise ValueError("the `--component` format must be <name>=<path>")


class StoreUploadCommand(AppCommand):
    """Upload a snap to the Snap Store."""

    name = "upload"
    help_msg = "Upload a snap to the Snap Store"
    overview = textwrap.dedent(
        """
        By passing --release with a comma-separated list of channels the snap would
        be released to the selected channels if the store review passes for this
        <snap-file>.

        This operation blocks until the store finishes processing the <snap-file>.

        If --release is used, the channel map will be displayed after the operation
        takes place.
        """
    )

    @overrides
    def fill_parser(self, parser: argparse.ArgumentParser) -> None:
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
        parser.add_argument(
            "--component",
            action="append",
            type=ComponentOption,
            default=[],
            help=(
                "The component(s) to upload with the snap, "
                "in the format 'name=filename'."
            ),
        )

    @overrides
    def run(self, parsed_args: argparse.Namespace) -> None:
        snap_file = pathlib.Path(parsed_args.snap_file)
        if not snap_file.exists() or not snap_file.is_file():
            raise ArgumentParsingError(f"{str(snap_file)!r} is not a valid file")

        channels: list[str] | None = None
        if parsed_args.channels:
            channels = parsed_args.channels.split(",")

        client = store.StoreClientCLI()

        snap_yaml, manifest_yaml = get_data_from_snap_file(snap_file)
        snap_metadata = SnapMetadata.unmarshal(snap_yaml)
        snap_name = snap_metadata.name
        built_at = None
        if manifest_yaml:
            built_at = manifest_yaml.get("snapcraft-started-at")

        components = parsed_args.component
        _validate_components(components, snap_metadata)

        client.verify_upload(snap_name=snap_name)

        snap_upload_id = client.store_client.upload_file(
            filepath=snap_file, monitor_callback=create_callback
        )

        component_upload_ids: dict[str, str] = {}
        for component in components:
            emit.debug(f"Uploading component {component.name!r}")
            upload_id = client.store_client.upload_file(
                filepath=pathlib.Path(component.path),
                monitor_callback=create_callback,
            )
            component_upload_ids[component.name] = upload_id

        revision = client.notify_upload(
            snap_name=snap_name,
            upload_id=snap_upload_id,
            built_at=built_at,
            channels=channels,
            snap_file_size=snap_file.stat().st_size,
            components=component_upload_ids or None,
        )

        message = f"Revision {revision!r} created for {snap_name!r}"
        if channels:
            message += f" and released to {utils.humanize_list(channels, 'and')}"
        emit.message(message)


def _validate_components(
    provided_components: abc.Collection[ComponentOption], snap_metadata: SnapMetadata
) -> None:
    """Validate component data.

    Validates:
     - The components provided by the user match the components listed in the snap's metadata.
     - All component files exist.

    :param provided_components: A list of ComponentOptions provided by the user.
    :param snap_metadata: The snap's metadata.
    """
    # get the names of the components provided by the user
    provided_names = {
        component.name for component in provided_components if component.name
    }
    emit.debug(f"Components provided by the user: {provided_names}")

    if snap_metadata.components:
        expected_names = set(snap_metadata.components.keys())
        emit.debug(f"Components found in snap metadata: {expected_names}")
    else:
        expected_names = set()
        emit.debug("No components found in snap metadata.")

    if unknown_components := provided_names - expected_names:
        raise ArgumentParsingError(
            f"Unknown component(s) provided {utils.humanize_list(unknown_components, 'and')}."
        )

    if missing_components := expected_names - provided_names:
        raise ArgumentParsingError(
            f"Missing component(s): {utils.humanize_list(missing_components, 'and')}. "
            "Use `--component <name>=<filename>`."
        )

    for component in provided_components:
        if not component.path:
            # should not occur after argparse validation
            raise RuntimeError(f"Component {component.name} has no filename.")

        component_filepath = pathlib.Path(component.path)
        if not component_filepath.is_file():
            raise errors.SnapcraftError(
                f"File '{component_filepath}' does not exist for component {component.name!r}."
            )


def create_callback(encoder: MultipartEncoder):
    """Create a callback suitable for upload_file."""
    with emit.progress_bar("Uploading...", encoder.len, delta=False) as progress:

        def progress_callback(monitor: MultipartEncoderMonitor):
            progress.advance(monitor.bytes_read)

        return progress_callback


class StoreLegacyPushCommand(StoreUploadCommand):
    """Removed command alias to upload a snap to the Snap Store."""

    name = "push"
    hidden = True

    @overrides
    def run(self, parsed_args: argparse.Namespace):
        raise errors.RemovedCommand(removed_command=self.name, new_command=super().name)
