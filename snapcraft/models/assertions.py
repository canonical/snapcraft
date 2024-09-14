# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
#  Copyright 2024 Canonical Ltd.
#
#  This program is free software: you can redistribute it and/or modify it
#  under the terms of the GNU Lesser General Public License version 3, as
#  published by the Free Software Foundation.
#
#  This program is distributed in the hope that it will be useful, but WITHOUT
#  ANY WARRANTY; without even the implied warranties of MERCHANTABILITY,
#  SATISFACTORY QUALITY, or FITNESS FOR A PARTICULAR PURPOSE.
#  See the GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with this program.  If not, see <http://www.gnu.org/licenses/>.

"""Models for assertion sets."""

from typing import Any, Literal

from craft_application import models


class RegistryAssertion(models.CraftBaseModel):
    """Data model for a registry assertion."""

    account_id: str
    authority_id: str
    body: dict[str, Any] | str | None = None
    body_length: str | None = None
    name: str
    revision: int = 0
    sign_key_sha3_384: str | None = None
    summary: str | None = None
    timestamp: str
    type: Literal["registry"]
    views: dict[str, Any]


# this will be a union for validation sets and registries once
# validation sets are migrated from the legacy codebase
Assertion = RegistryAssertion
