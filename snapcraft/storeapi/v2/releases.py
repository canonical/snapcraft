# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2020 Canonical Ltd
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

from typing import Any, Dict, List, Optional

import jsonschema

from ._api_schema import RELEASES_JSONSCHEMA

"""
This module holds representations for results for the v2 releases
API endpoint provided by the Snap Store. It currently only covers
Revisions and Releases.

The full API is documented on
https://dashboard.snapcraft.io/docs/v2/en/snaps.html#snap-releases
"""


class Release:
    """Represent a snap release."""

    @classmethod
    def unmarshal(cls, payload: Dict[str, Any]) -> "Release":
        jsonschema.validate(
            payload,
            RELEASES_JSONSCHEMA["properties"]["releases"]["items"]["properties"],
        )
        return cls(
            architecture=payload["architecture"],
            branch=payload["branch"],
            channel=payload["channel"],
            expiration_date=payload["expiration-date"],
            revision=payload["revision"],
            risk=payload["risk"],
            track=payload["track"],
            when=payload["when"],
        )

    def marshal(self) -> Dict[str, Any]:
        return {
            "architecture": self.architecture,
            "branch": self.branch,
            "channel": self.channel,
            "expiration-date": self.expiration_date,
            "revision": self.revision,
            "risk": self.risk,
            "track": self.track,
            "when": self.when,
        }

    def __repr__(self) -> str:
        return f"<{self.__class__.__name__}: {self.channel!r}>"

    def __init__(
        self,
        architecture: str,
        branch: Optional[str],
        channel: str,
        expiration_date: Optional[str],
        revision: Optional[int],
        risk: str,
        track: str,
        when: str,
    ) -> None:
        self.architecture = architecture
        self.branch = branch
        self.channel = channel
        self.expiration_date = expiration_date
        self.revision = revision
        self.risk = risk
        self.track = track
        self.when = when


class Revision:
    """Represent a snap revision."""

    @classmethod
    def unmarshal(cls, payload: Dict[str, Any]) -> "Revision":
        jsonschema.validate(
            payload,
            RELEASES_JSONSCHEMA["properties"]["revisions"]["items"]["properties"],
        )
        return cls(
            architectures=payload["architectures"],
            base=payload.get("base"),
            build_url=payload["build_url"],
            confinement=payload["confinement"],
            created_at=payload["created_at"],
            grade=payload["grade"],
            revision=payload["revision"],
            sha3_384=payload["sha3-384"],
            size=payload["size"],
            status=payload["status"],
            version=payload["version"],
        )

    def marshal(self) -> Dict[str, Any]:
        payload = {
            "architectures": self.architectures,
            "build_url": self.build_url,
            "confinement": self.confinement,
            "created_at": self.created_at,
            "grade": self.grade,
            "revision": self.revision,
            "sha3-384": self.sha3_384,
            "size": self.size,
            "status": self.status,
            "version": self.version,
        }

        if self.base is not None:
            payload["base"] = self.base

        return payload

    def __repr__(self) -> str:
        return f"<{self.__class__.__name__}: {self.revision!r}>"

    def __init__(
        self,
        *,
        architectures: List[str],
        base: Optional[str],
        build_url: Optional[str],
        confinement: str,
        created_at: str,
        grade: str,
        revision: int,
        sha3_384: str,
        size: int,
        status: str,
        version: str,
    ):
        self.architectures = architectures
        self.base = base
        self.build_url = build_url
        self.confinement = confinement
        self.created_at = created_at
        self.grade = grade
        self.revision = revision
        self.sha3_384 = sha3_384
        self.size = size
        self.status = status
        self.version = version


class Releases:
    """Represent the data returned from the releases Snap Store endpoint."""

    @classmethod
    def unmarshal(cls, payload: Dict[str, Any]) -> "Releases":
        jsonschema.validate(payload, RELEASES_JSONSCHEMA)
        return cls(
            releases=[Release.unmarshal(r) for r in payload["releases"]],
            revisions=[Revision.unmarshal(r) for r in payload["revisions"]],
        )

    def marshal(self) -> Dict[str, Any]:
        return {
            "releases": [r.marshal() for r in self.releases],
            "revisions": [r.marshal() for r in self.revisions],
        }

    def __repr__(self) -> str:
        return f"<{self.__class__.__name__}>"

    def __init__(self, releases: List[Release], revisions: List[Revision]) -> None:
        self.releases = releases
        self.revisions = revisions
