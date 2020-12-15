# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2020 Canonical Ltd
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

import itertools
import os
from collections import OrderedDict
from typing import List, Optional, Tuple

from tabulate import tabulate
from typing_extensions import Final

from snapcraft.storeapi.v2.channel_map import (
    ChannelMap,
    MappedChannel,
    Revision,
    SnapChannel,
)


class _HINTS:
    CLOSED: Final[str] = "-"
    FOLLOWING: Final[str] = "↑"
    NO_PROGRESS: Final[str] = "-"
    PROGRESSING_TO: Final[str] = "→"
    UNKNOWN: Final[str] = "?"


def _get_channel_order(
    snap_channels, tracks: Optional[List[str]] = None
) -> OrderedDict:
    channel_order: OrderedDict = OrderedDict()

    if tracks is not None:
        snap_channels = [s for s in snap_channels if s.track in tracks]

    for snap_channel in snap_channels:
        if snap_channel.track not in channel_order:
            channel_order[snap_channel.track] = list()
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

    if os.getenv("SNAPCRAFT_EXPERIMENTAL_PROGRESSIVE_RELEASES"):
        return [
            channel_string,
            version_string,
            revision_string,
            progress_string,
            expiration_date_string,
        ]
    else:
        return [channel_string, version_string, revision_string, expiration_date_string]


def _get_channel_lines_for_channel(  # noqa: C901
    snap_channel_map: ChannelMap,
    channel_name: str,
    architecture: str,
    current_tick: str,
) -> Tuple[str, List[List[str]]]:
    channel_lines: List[List[str]] = list()

    channel_info = snap_channel_map.get_channel_info(channel_name)

    try:
        progressive_mapped_channel: Optional[MappedChannel] = (
            snap_channel_map.get_mapped_channel(
                channel_name=channel_name, architecture=architecture, progressive=True
            )
        )
    except ValueError:
        progressive_mapped_channel = None

    if progressive_mapped_channel is not None:
        progressive_revision = snap_channel_map.get_revision(
            progressive_mapped_channel.revision
        )

        if progressive_mapped_channel.progressive.percentage is None:
            raise RuntimeError("Unexpected null progressive percentage")
        else:
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

    if (
        os.getenv("SNAPCRAFT_EXPERIMENTAL_PROGRESSIVE_RELEASES")
        and progressive_mapped_channel is not None
    ):
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


def get_tabulated_channel_map(
    snap_channel_map, *, architectures: List[str], tracks: Optional[List[str]] = None
):
    channel_order = _get_channel_order(snap_channel_map.snap.channels, tracks)

    channel_lines = list()
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

    headers = ["Track", "Arch", "Channel", "Version", "Revision"]
    if os.getenv("SNAPCRAFT_EXPERIMENTAL_PROGRESSIVE_RELEASES"):
        headers.append("Progress")
        # Item 6 is expiration_date when progressive releases are enabled.
        expires_column = 6
    else:
        expires_column = 5

    if any(line[expires_column] != "" for line in channel_lines):
        headers.append("Expires at")
    else:
        headers.append("")

    return tabulate(channel_lines, numalign="left", headers=headers, tablefmt="plain")
