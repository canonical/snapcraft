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
"""Tests for a snapd type project."""

from typing import Any

import pydantic
import pytest

from snapcraft import models

BASE_SNAPD_DATA = {
    "name": "snapd",
    "type": "snapd",
    "summary": "This snap contains snapd.",
    "description": "Yo dawg, I heard you like snaps...",
    "parts": {},
    "confinement": "strict",
    "adopt-info": "snapd",
}


@pytest.fixture(
    params=[
        pytest.param({"build-base": "core22"}, id="core22"),
        pytest.param({"build-base": "core24"}, id="core24"),
        pytest.param({"build-base": "devel"}, id="devel"),
    ]
)
def get_project_yaml(request: pytest.FixtureRequest):
    def _get_project_yaml(**kwargs) -> dict[str, Any]:
        return BASE_SNAPD_DATA | request.param | kwargs

    return _get_project_yaml


def test_load_valid_project(get_project_yaml):
    project_yaml = get_project_yaml()

    project = models.Project.unmarshal(project_yaml)

    assert isinstance(
        project,
        (models.project._BaselessProject, models.project._BaselessCore22Project),
    )


@pytest.mark.parametrize("missing_key", ["name", "build-base"])
def test_load_partial_project(get_project_yaml, missing_key):
    project_yaml = get_project_yaml()
    del project_yaml[missing_key]

    with pytest.raises(pydantic.ValidationError, match=missing_key):
        models.Project.unmarshal(project_yaml)


def test_load_snapd_info(get_project_yaml):
    snapd_info = {
        "ubuntu-core-tracks": {
            "18": {"latest": "18", "fips-updates": "18-fips"},
            "20": {"latest": "20"},
        }
    }

    project = models.Project.unmarshal(
        get_project_yaml(**{"snapd-info": snapd_info})
    )

    assert project.snapd_info == snapd_info


@pytest.mark.parametrize(
    "snapd_info",
    [
        {},
        {"ubuntu-core-tracks": {}},
        {"ubuntu-core-tracks": {"18": {"latest": "18"}}},
        {"ubuntu-core-tracks": {}, "extra": True},
        {
            "ubuntu-core-tracks": {
                18: {"latest": "18"},
                20: {"latest": "20"},
            }
        },
    ],
)
def test_load_valid_snapd_info(get_project_yaml, snapd_info):
    project = models.Project.unmarshal(
        get_project_yaml(**{"snapd-info": snapd_info})
    )

    assert project.snapd_info == snapd_info


@pytest.mark.parametrize("snapd_info", [[], "foo", None])
def test_load_invalid_snapd_info(get_project_yaml, snapd_info):
    with pytest.raises(pydantic.ValidationError, match="snapd-info"):
        models.Project.unmarshal(
            get_project_yaml(**{"snapd-info": snapd_info})
        )
