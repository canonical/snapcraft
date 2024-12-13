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

"""Tests for the Snapcraft provider service."""

from craft_application.services.provider import DEFAULT_FORWARD_ENVIRONMENT_VARIABLES

from snapcraft.const import SNAPCRAFT_ENVIRONMENT_VARIABLES


def test_provider(provider_service, monkeypatch):
    variables = SNAPCRAFT_ENVIRONMENT_VARIABLES | set(
        DEFAULT_FORWARD_ENVIRONMENT_VARIABLES
    )
    for variable in variables:
        monkeypatch.setenv(variable, f"{variable}-test-value")

    provider_service.setup()

    for variable in variables:
        assert provider_service.environment[variable] == f"{variable}-test-value"
