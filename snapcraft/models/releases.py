# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2026 Canonical Ltd
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

"""Releases and revision models.

https://dashboard.snapcraft.io/docs/reference/v2/en/snaps.html#snap-releases
"""

from typing import Any

from craft_application import models


class Progressive(models.CraftBaseModel):
    """Progressive release information."""

    paused: bool | None
    """Whether the progressive release is paused."""

    percentage: float | None
    """Target percentage of devices to receive this release."""

    current_percentage: float | None
    """Current percentage of devices that have received this release."""


class Epoch(models.CraftBaseModel):
    """Snap epoch information."""

    read: list[int]
    """List of epoch values this snap can read from."""

    write: list[int]
    """List of epoch values this snap writes."""


class Release(models.CraftBaseModel):
    """A Snap release."""

    architecture: str
    """The architecture this release targets."""

    branch: str | None
    """The branch name for this release or null if not a branch release."""

    channel: str
    """The channel name for this release."""

    expiration_date: str | None = None
    """Timestamp of when this release expires, in ISO 8601. If null, the release does not expire."""

    progressive: Progressive | None = None
    """Progressive release details, if this is a progressive release."""

    revision: int | None
    """The snap revision number, or null if the channel has been closed."""

    risk: str
    """The risk level for this release."""

    track: str
    """The track name for this release."""

    when: str
    """Timestamp of when this release was made, in ISO 8601."""


class Revision(models.CraftBaseModel):
    """A Snap revision."""

    architectures: list[str]
    """List of architectures supported by this revision."""

    attributes: dict[str, Any] = {}
    """Additional attributes for this revision."""

    base: str | None = None
    """The base snap this revision was built against or null for base snaps."""

    build_url: str | None = None
    """URL to the build log for this revision or null if unavailable."""

    confinement: str
    """Confinement of the snap."""

    created_at: str
    """Timestamp of when this revision was uploaded, in ISO 8601 format."""

    epoch: Epoch | None = None
    """Epoch information for this revision."""

    grade: str
    """Grade of the snap."""

    revision: int
    """Snap revision number."""

    sha3_384: str
    """SHA3-384 hash of the snap file."""

    size: int
    """Size of the snap file in bytes."""

    status: str
    """Review and publication status of this revision."""

    version: str
    """Version string declared in the snap's metadata."""


class Releases(models.CraftBaseModel, extra="ignore"):
    """Represent the data returned from the releases Snap Store endpoint."""

    releases: list[Release] = []
    """Channel map entries showing which revision is released to each channel."""

    revisions: list[Revision]
    """All revisions involved in the release context, including unreleased ones."""
