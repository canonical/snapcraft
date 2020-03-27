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

import os
from collections import OrderedDict
from typing import List, Optional, Union, TYPE_CHECKING

from tabulate import tabulate

if TYPE_CHECKING:
    from snapcraft.storeapi.v2 import snap_channel_map


def _get_channel_hint(*, channel_map, fallback: str, architecture: str) -> str:
    for c in channel_map:
        if c.channel == fallback and c.architecture == architecture:
            tick = "^"
            break
    else:
        if fallback is None:
            tick = "-"
        else:
            tick = "^"
    return tick


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
    mapped_channel: Optional["snap_channel_map.channel.MappedChannel"],
    revision: Optional["snap_channel_map.revision.Revision"],
    channel_info: "snap_channel_map.channel.SnapChannel",
    hint: str,
    progress_string: str,
) -> List[Union[int, str]]:
    version_string = hint
    revision_string = hint
    expiration_date_string = ""
    channel_string = channel_info.risk

    if revision is not None:
        version_string = revision.version
        revision_string = f"{revision.revision}"

    if mapped_channel is not None:
        if mapped_channel.progressive.percentage:
            channel_string = ""
        elif channel_info.branch is not None:
            channel_string = f"{channel_info.risk}/{channel_info.branch}"
            # This value can be None.
            if mapped_channel.expiration_date:
                expiration_date_string = mapped_channel.expiration_date

    if os.getenv("SNAPCRAFT_EXPERIMENTAL_PROGRESSIVE_RELEASE"):
        return [
            channel_string,
            version_string,
            revision_string,
            progress_string,
            expiration_date_string,
        ]
    else:
        return [channel_string, version_string, revision_string, expiration_date_string]


def _has_channels_for_architecture(
    snap_channel_map, architecture: str, channels: List[str]
) -> bool:
    for channel_name in channels:
        try:
            snap_channel_map.get_mapped_channel(
                channel_name=channel_name, architecture=architecture, progressive=False
            )
            return True
        except ValueError:
            continue
    return False


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

                channel_info = snap_channel_map.get_channel_info(channel_name)
                hint = _get_channel_hint(
                    channel_map=snap_channel_map.channel_map,
                    fallback=channel_info.fallback,
                    architecture=architecture,
                )

                try:
                    progressive_mapped_channel = snap_channel_map.get_mapped_channel(
                        channel_name=channel_name,
                        architecture=architecture,
                        progressive=True,
                    )
                    progressive_revision = snap_channel_map.get_revision(
                        progressive_mapped_channel.revision
                    )
                    progressive_mapped_channel_line = _get_channel_line(
                        mapped_channel=progressive_mapped_channel,
                        revision=progressive_revision,
                        channel_info=channel_info,
                        hint=hint,
                        progress_string=f"→ {progressive_mapped_channel.progressive.percentage:.0f}%",
                    )
                except ValueError:
                    progressive_mapped_channel = None

                if progressive_mapped_channel is not None:
                    progress_string = "→ {:.0f}%".format(
                        100 - progressive_mapped_channel.progressive.percentage
                    )
                else:
                    progress_string = "-"

                try:
                    mapped_channel = snap_channel_map.get_mapped_channel(
                        channel_name=channel_name,
                        architecture=architecture,
                        progressive=False,
                    )
                    revision = snap_channel_map.get_revision(mapped_channel.revision)
                    mapped_channel_line = _get_channel_line(
                        mapped_channel=mapped_channel,
                        revision=revision,
                        channel_info=channel_info,
                        hint=hint,
                        progress_string=progress_string,
                    )
                except ValueError:
                    # Don't show empty branches
                    if channel_info.branch is None:
                        mapped_channel_line = _get_channel_line(
                            mapped_channel=None,
                            revision=None,
                            channel_info=channel_info,
                            hint=hint,
                            progress_string=progress_string,
                        )
                    else:
                        mapped_channel_line = []

                if mapped_channel_line:
                    channel_lines.append(
                        [track_string, architecture_string] + mapped_channel_line
                    )
                    track_string = ""
                    architecture_string = ""

                if (
                    os.getenv("SNAPCRAFT_EXPERIMENTAL_PROGRESSIVE_RELEASE")
                    and progressive_mapped_channel is not None
                ):
                    channel_lines.append(
                        [track_string, architecture_string]
                        + progressive_mapped_channel_line
                    )

    headers = ["Track", "Arch", "Channel", "Version", "Revision"]
    if os.getenv("SNAPCRAFT_EXPERIMENTAL_PROGRESSIVE_RELEASE"):
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
