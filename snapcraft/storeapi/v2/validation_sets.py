# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2021 Canonical Ltd
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

from ._api_schema import BUILD_ASSERTION_JSONSCHEMA


class Snap:
    """Represent a Snap in a Validation Set."""

    @classmethod
    def unmarshal(cls, payload: Dict[str, Any]) -> "Snap":
        jsonschema.validate(
            payload, BUILD_ASSERTION_JSONSCHEMA["properties"]["snaps"]["items"]
        )

        return cls(
            name=payload["name"],
            snap_id=payload.get("id"),
            presence=payload.get("presence"),
            revision=payload.get("revision"),
        )

    def marshal(self) -> Dict[str, Any]:
        payload = {"name": self.name}

        if self.snap_id is not None:
            payload["id"] = self.snap_id

        if self.presence is not None:
            payload["presence"] = self.presence

        if self.revision is not None:
            payload["revision"] = self.revision

        return payload

    def __eq__(self, other) -> bool:
        if not isinstance(other, Snap):
            return False

        return (
            self.name == other.name
            and self.snap_id == other.snap_id
            and self.presence == other.presence
            and self.revision == other.revision
        )

    def __repr__(self) -> str:
        return f"<{self.__class__.__name__}: {self.name!r}>"

    def __init__(
        self,
        name: str,
        snap_id: Optional[str] = None,
        presence: Optional[str] = None,
        revision: Optional[str] = None,
    ):
        self.name = name
        self.snap_id = snap_id
        self.presence = presence
        self.revision = revision


class BuildAssertion:
    """Represent Validation Set assertion headers ready local signing."""

    @classmethod
    def unmarshal(cls, payload: Dict[str, Any]) -> "BuildAssertion":
        jsonschema.validate(payload, BUILD_ASSERTION_JSONSCHEMA)

        return cls(
            account_id=payload["account-id"],
            authority_id=payload["authority-id"],
            name=payload["name"],
            revision=payload.get("revision", "0"),
            sequence=payload["sequence"],
            series=payload["series"],
            snaps=[Snap.unmarshal(s) for s in payload["snaps"]],
            timestamp=payload["timestamp"],
            assertion_type=payload["type"],
        )

    def marshal(self) -> Dict[str, Any]:
        return {
            "account-id": self.account_id,
            "authority-id": self.authority_id,
            "name": self.name,
            "revision": self.revision,
            "sequence": self.sequence,
            "snaps": [s.marshal() for s in self.snaps],
            "timestamp": self.timestamp,
            "type": self.assertion_type,
            "series": self.series,
        }

    def __eq__(self, other) -> bool:
        if not isinstance(other, BuildAssertion):
            return False

        return (
            self.name == other.name
            and self.account_id == other.account_id
            and self.authority_id == other.authority_id
            and self.revision == other.revision
            and self.sequence == other.sequence
            and self.snaps == other.snaps
            and self.timestamp == other.timestamp
            and self.assertion_type == other.assertion_type
            and self.series == other.series
        )

    def __repr__(self) -> str:
        return f"<{self.__class__.__name__}: {self.name!r}>"

    def __init__(
        self,
        *,
        account_id: str,
        authority_id: str,
        name: str,
        revision: str,
        sequence: str,
        snaps: List[Snap],
        timestamp: str,
        assertion_type: str,
        series: str = "16",
    ):
        self.account_id = account_id
        self.authority_id = authority_id
        self.name = name
        self.revision = revision
        self.sequence = sequence
        self.snaps = snaps
        self.series = series
        self.timestamp = timestamp
        self.assertion_type = assertion_type


class ValidationSets:
    @classmethod
    def unmarshal(cls, payload: Dict[str, Any]) -> "ValidationSets":
        return cls(
            assertions=[
                BuildAssertion.unmarshal(a["headers"])
                for a in payload.get("assertions", list())
            ]
        )

    def marshal(self) -> Dict[str, Any]:
        return {"assertions": [{"headers": a.marshal()} for a in self.assertions]}

    def __repr__(self) -> str:
        assertion_count = len(self.assertions)
        return f"<{self.__class__.__name__}: assertions {assertion_count!r}>"

    def __init__(self, assertions: List[BuildAssertion]) -> None:
        self.assertions = assertions
