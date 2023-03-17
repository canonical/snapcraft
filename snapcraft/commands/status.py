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

"""Snapcraft Store Account management commands."""
import itertools
import operator
import textwrap
from collections import OrderedDict
from typing import TYPE_CHECKING, Dict, List, Optional, Sequence, Set, Tuple, cast

from craft_cli import BaseCommand, emit
from overrides import overrides
from tabulate import tabulate
from typing_extensions import Final

from snapcraft import store
from snapcraft.store.channel_map import ChannelMap, MappedChannel, Revision, SnapChannel

if TYPE_CHECKING:
    import argparse


class StoreStatusCommand(BaseCommand):
    """Check the status of a snap in the Snap Store."""

    name = "status"
    help_msg = "Show the status of a snap in the Snap Store"
    overview = textwrap.dedent(
        """
        Show the status of a snap in the Snap Store.
        The name must be accessible from the requesting account by being
        the owner or a collaborator of the snap."""
    )

    @overrides
    def fill_parser(self, parser: "argparse.ArgumentParser") -> None:
        parser.add_argument(
            "name",
            type=str,
            help="Get the status on a snap from the Snap Store",
        )
        parser.add_argument(
            "--arch",
            metavar="arch",
            type=str,
            action="append",
            nargs="?",
            help="Limit the status report to the requested architectures",
        )
        parser.add_argument(
            "--track",
            metavar="track",
            type=str,
            action="append",
            nargs="?",
            help="Limit the status report to the requested tracks",
        )

    @overrides
    def run(self, parsed_args):
        snap_channel_map = store.StoreClientCLI().get_channel_map(
            snap_name=parsed_args.name
        )

        existing_architectures = snap_channel_map.get_existing_architectures()
        if not snap_channel_map.channel_map:
            emit.message("This snap has no released revisions")
            return

        architectures = existing_architectures
        if parsed_args.arch:
            architectures = set(parsed_args.arch)
            for architecture in architectures.copy():
                if architecture not in existing_architectures:
                    emit.progress(f"No revisions for architecture {architecture!r}")
                    architectures.remove(architecture)

            # If we have no revisions for any of the architectures requested, there's
            # nothing to do here.
            if not architectures:
                return

        tracks: List[str] = []
        if parsed_args.track:
            tracks = cast(list, parsed_args.track)
            existing_tracks = {
                s.track for s in snap_channel_map.snap.channels if s.track in tracks
            }
            for track in set(tracks) - existing_tracks:
                emit.progress(f"No revisions for track {track!r}")
            tracks = list(existing_tracks)

            # If we have no revisions in any of the tracks requested, there's
            # nothing to do here.
            if not tracks:
                return

        emit.message(
            get_tabulated_channel_map(
                snap_channel_map,
                architectures=list(architectures),
                tracks=tracks,
            )
        )


class _HINTS:
    CLOSED: Final[str] = "-"
    FOLLOWING: Final[str] = "↑"
    NO_PROGRESS: Final[str] = "-"
    PROGRESSING_TO: Final[str] = "→"
    UNKNOWN: Final[str] = "?"


def _get_channel_order(snap_channels, tracks: Sequence[str]) -> OrderedDict:
    channel_order: OrderedDict = OrderedDict()

    if tracks:
        snap_channels = [s for s in snap_channels if s.track in tracks]

    for snap_channel in snap_channels:
        if snap_channel.track not in channel_order:
            channel_order[snap_channel.track] = []
        if snap_channel.fallback is None:
            channel_order[snap_channel.track].append(snap_channel.name)
        else:
            try:
                channel_order[snap_channel.track].insert(
                    channel_order[snap_channel.track].index(snap_channel.fallback) + 1,
                    snap_channel.name,
                )
            except ValueError:
                channel_order[snap_channel.track].append(snap_channel.name)

    return channel_order


def _get_channel_line(
    *,
    mapped_channel: Optional[MappedChannel],
    revision: Optional[Revision],
    channel_info: SnapChannel,
    hint: str,
    progress_string: str,
) -> List[str]:
    version_string = hint
    revision_string = hint
    expiration_date_string = ""
    channel_string = channel_info.risk

    if revision is not None:
        version_string = revision.version
        revision_string = f"{revision.revision}"

    if mapped_channel is not None:
        if channel_info.branch is None and mapped_channel.progressive.percentage:
            channel_string = ""
        elif channel_info.branch is not None:
            channel_string = f"{channel_info.risk}/{channel_info.branch}"
            if mapped_channel.expiration_date is not None:
                expiration_date_string = mapped_channel.expiration_date

    return [
        channel_string,
        version_string,
        revision_string,
        progress_string,
        expiration_date_string,
    ]


def _get_channel_lines_for_channel(  # noqa: C901  # pylint: disable=too-many-locals
    snap_channel_map: ChannelMap,
    channel_name: str,
    architecture: str,
    current_tick: str,
) -> Tuple[str, List[List[str]]]:
    channel_lines: List[List[str]] = []

    channel_info = snap_channel_map.get_channel_info(channel_name)

    try:
        progressive_mapped_channel: Optional[
            MappedChannel
        ] = snap_channel_map.get_mapped_channel(
            channel_name=channel_name, architecture=architecture, progressive=True
        )
    except ValueError:
        progressive_mapped_channel = None

    if progressive_mapped_channel is not None:
        progressive_revision = snap_channel_map.get_revision(
            progressive_mapped_channel.revision
        )

        if progressive_mapped_channel.progressive.percentage is None:
            raise RuntimeError("Unexpected null progressive percentage")
        percentage = progressive_mapped_channel.progressive.percentage

        if progressive_mapped_channel.progressive.current_percentage is None:
            current_percentage_fmt = _HINTS.UNKNOWN
            remaining_percentage_fmt = _HINTS.UNKNOWN
        else:
            current_percentage = (
                progressive_mapped_channel.progressive.current_percentage
            )
            current_percentage_fmt = f"{current_percentage:.0f}"
            remaining_percentage_fmt = f"{100 - current_percentage:.0f}"

        progressive_mapped_channel_line = _get_channel_line(
            mapped_channel=progressive_mapped_channel,
            revision=progressive_revision,
            channel_info=channel_info,
            hint=current_tick,
            progress_string=f"{current_percentage_fmt}{_HINTS.PROGRESSING_TO}{percentage:.0f}%",
        )
        # Setup progress for the actually released revision, this needs to be
        # calculated. But only show it if the channel is open.
        progress_string = (
            f"{remaining_percentage_fmt}{_HINTS.PROGRESSING_TO}{100 - percentage:.0f}%"
        )
    else:
        progressive_mapped_channel_line = []
        progress_string = _HINTS.NO_PROGRESS

    try:
        mapped_channel: Optional[MappedChannel] = snap_channel_map.get_mapped_channel(
            channel_name=channel_name, architecture=architecture, progressive=False
        )
    except ValueError:
        mapped_channel = None

    next_tick = current_tick
    if mapped_channel is not None:
        revision = snap_channel_map.get_revision(mapped_channel.revision)
        channel_lines.append(
            _get_channel_line(
                mapped_channel=mapped_channel,
                revision=revision,
                channel_info=channel_info,
                hint=current_tick,
                progress_string=progress_string,
            )
        )
        if channel_info.branch is None:
            next_tick = _HINTS.FOLLOWING
    # Show an empty entry if there is no specific channel information, but
    # only for <track>/<risks> (ignoring /<branch>).
    elif channel_info.branch is None:
        channel_lines.append(
            _get_channel_line(
                mapped_channel=None,
                revision=None,
                channel_info=channel_info,
                hint=current_tick,
                progress_string=_HINTS.NO_PROGRESS
                if current_tick == _HINTS.CLOSED
                else progress_string,
            )
        )

    if progressive_mapped_channel is not None:
        channel_lines.append(progressive_mapped_channel_line)
        if channel_info.branch is None:
            next_tick = _HINTS.FOLLOWING

    return next_tick, channel_lines


def _has_channels_for_architecture(
    snap_channel_map, architecture: str, channels: List[str]
) -> bool:
    progressive = (False, True)
    # channel_query = (channel_name, progressive)
    for channel_query in itertools.product(channels, progressive):
        try:
            snap_channel_map.get_mapped_channel(
                channel_name=channel_query[0],
                architecture=architecture,
                progressive=channel_query[1],
            )
            found_architecture = True
            break
        except ValueError:
            continue
    else:
        found_architecture = False

    return found_architecture


def get_tabulated_channel_map(  # pylint: disable=too-many-branches, too-many-locals  # noqa: C901
    snap_channel_map,
    *,
    architectures: Sequence[str],
    tracks: Sequence[str],
):
    """Return a tabulated channel map."""
    channel_order = _get_channel_order(snap_channel_map.snap.channels, tracks)

    channel_lines = []
    for track_name in channel_order:
        track_mentioned = False
        for architecture in sorted(architectures):
            if not _has_channels_for_architecture(
                snap_channel_map, architecture, channel_order[track_name]
            ):
                continue
            architecture_mentioned = False
            next_tick = _HINTS.CLOSED
            for channel_name in channel_order[track_name]:
                if not track_mentioned:
                    track_mentioned = True
                    track_string = track_name
                else:
                    track_string = ""

                if not architecture_mentioned:
                    architecture_mentioned = True
                    architecture_string = architecture
                else:
                    architecture_string = ""

                next_tick, parsed_channels = _get_channel_lines_for_channel(
                    snap_channel_map, channel_name, architecture, next_tick
                )
                for channel_line in parsed_channels:
                    channel_lines.append(
                        [track_string, architecture_string] + channel_line
                    )
                    track_string = ""
                    architecture_string = ""

    headers = ["Track", "Arch", "Channel", "Version", "Revision", "Progress"]
    expires_column = 6

    if any(line[expires_column] != "" for line in channel_lines):
        headers.append("Expires at")
        for index, _ in enumerate(channel_lines):
            if not channel_lines[index][expires_column]:  # pylint: disable=R1736
                channel_lines[index][expires_column] = "-"
    else:
        headers.append("")

    return tabulate(channel_lines, numalign="left", headers=headers, tablefmt="plain")


class StoreListTracksCommand(BaseCommand):
    """List the tracks of a snap in the Snap Store."""

    name = "list-tracks"
    help_msg = "Show the available tracks for a snap in the Snap Store"
    overview = textwrap.dedent(
        """
        Track status, creation dates and version patterns are returned alongside the
        track names in a space formatted table.

        Possible Status values are:

        - active, visible tracks available for installation
        - default, the default track to install from when not explicit
        - hidden, tracks available for installation but unlisted
        - closed, tracks that are no longer available to install from

        A version pattern is a regular expression that restricts a snap revision
        from being released to a track if the version string set does not match."""
    )

    @overrides
    def fill_parser(self, parser: "argparse.ArgumentParser") -> None:
        parser.add_argument(
            "name",
            type=str,
            help="The snap name to request the information from on the Snap Store",
        )

    @overrides
    def run(self, parsed_args):
        snap_channel_map = store.StoreClientCLI().get_channel_map(
            snap_name=parsed_args.name
        )

        # Iterate over the entries, replace None with - for consistent presentation
        track_table: List[List[str]] = [
            [
                track.name,
                track.status,
                track.creation_date if track.creation_date else "-",
                track.version_pattern if track.version_pattern else "-",
            ]
            for track in snap_channel_map.snap.tracks
        ]

        emit.message(
            tabulate(
                # Sort by "creation-date".
                sorted(track_table, key=operator.itemgetter(2)),
                headers=["Name", "Status", "Creation-Date", "Version-Pattern"],
                tablefmt="plain",
            )
        )


class StoreTracksCommand(StoreListTracksCommand):
    """Command alias to list the tracks of a snap in the Snap Store."""

    name = "tracks"
    hidden = True


class StoreListRevisionsCommand(BaseCommand):
    """List revisions of a published snap."""

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

    @overrides
    def run(self, parsed_args):
        releases = store.StoreClientCLI().list_revisions(
            snap_name=parsed_args.snap_name
        )

        parsed_revisions = []
        for rev in releases.revisions:
            if parsed_args.arch and parsed_args.arch not in rev.architectures:
                continue
            if releases.releases:
                channels_for_revision = self._get_channels_for_revision(
                    releases, rev.revision
                )
                if channels_for_revision:
                    channels = ",".join(channels_for_revision)
                else:
                    channels = "-"
                parsed_revisions.append(
                    (
                        str(rev.revision),
                        rev.created_at,
                        ",".join(rev.architectures),
                        rev.version,
                        channels,
                    )
                )
            else:
                parsed_revisions.append(
                    (
                        str(rev.revision),
                        rev.created_at,
                        ",".join(rev.architectures),
                        rev.version,
                    )
                )

        if releases.releases:
            headers = ["Rev.", "Uploaded", "Arches", "Version", "Channels"]
        else:
            headers = ["Rev.", "Uploaded", "Arches", "Version"]

        tabulated_revisions = tabulate(
            parsed_revisions,
            numalign="left",
            headers=headers,
            tablefmt="plain",
        )

        emit.message(tabulated_revisions)

    def _get_channels_for_revision(self, releases, revision: int) -> List[str]:
        # channels: the set of channels revision was released to, active or not.
        channels: Set[str] = set()
        # seen_channel: applies to channels regardless of revision.
        # The first channel that shows up for each architecture is to
        # be marked as the active channel, all others are historic.
        seen_channel: Dict[str, Set[str]] = {}

        for release in releases.releases:
            if release.architecture not in seen_channel:
                seen_channel[release.architecture] = set()

            # If the revision is in this release entry and was not seen
            # before it means that this channel is active and needs to
            # be represented with a *.
            if (
                release.revision == revision
                and release.channel not in seen_channel[release.architecture]
            ):
                channels.add(f"{release.channel}*")
            # All other releases found for a revision are inactive.
            elif (
                release.revision == revision
                and release.channel not in channels
                and f"{release.channel}*" not in channels
            ):
                channels.add(release.channel)

            seen_channel[release.architecture].add(release.channel)

        return sorted(list(channels))


class StoreRevisionsCommand(StoreListRevisionsCommand):
    """Command alias to list-revisions."""

    name = "revisions"
    hidden = True
