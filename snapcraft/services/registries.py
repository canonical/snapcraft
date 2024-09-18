# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2024 Canonical Ltd.
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

"""Service class for registries."""

from __future__ import annotations

from typing import Any

from typing_extensions import override

from snapcraft import models
from snapcraft.services import Assertion


class Registries(Assertion):
    """Service for interacting with registries."""

    @property
    @override
    def _assertion_name(self) -> str:
        return "registries set"

    @override
    def _get_assertions(self, name: str | None = None) -> list[models.Assertion]:
        return self._store_client.list_registries(name=name)

    @override
    def _normalize_assertions(
        self, assertions: list[models.Assertion]
    ) -> tuple[list[str], list[list[Any]]]:
        headers = ["Account ID", "Name", "Revision", "When"]
        registries = [
            [
                assertion.account_id,
                assertion.name,
                assertion.revision,
                assertion.timestamp[:10],
            ]
            for assertion in assertions
        ]

        return headers, registries
