# Copyright 2025 Canonical Ltd.
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

from typing import Any

import pytest


@pytest.fixture
def project_yaml_data():
    def _project_yaml_data(
        *, name: str = "name", version: str = "0.1", summary: str = "summary", **kwargs
    ) -> dict[str, Any]:
        return {
            "name": name,
            "version": version,
            "base": "core22",
            "summary": summary,
            "description": "description",
            "grade": "stable",
            "confinement": "strict",
            "parts": {},
            **kwargs,
        }

    yield _project_yaml_data
