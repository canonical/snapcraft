# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
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

"""Tests for the Snapcraft project service."""

import itertools
import pathlib
from typing import Any

import pytest
import pytest_mock
from craft_application.errors import CraftValidationError

from snapcraft import const
from snapcraft.application import APP_METADATA
from snapcraft.services.project import Project


@pytest.mark.parametrize(
    ("raw_project", "expected"),
    [
        pytest.param(
            {
                "base": "core22",
                "architectures": [
                    {"build-on": ["amd64", "arm64"], "build-for": ["all"]}
                ],
            },
            {"all": {"build-on": ["amd64", "arm64"], "build-for": ["all"]}},
        ),
        *(
            pytest.param(
                {"base": "core22", "architectures": [arch.value]},
                {arch.value: {"build-on": [arch.value], "build-for": [arch.value]}},
                id=arch.value,
            )
            for arch in const.SnapArch
        ),
        *(
            pytest.param(
                {"base": "core22", "architectures": [arch1.value, arch2.value]},
                {
                    arch1.value: {
                        "build-on": [arch1.value],
                        "build-for": [arch1.value],
                    },
                    arch2.value: {
                        "build-on": [arch2.value],
                        "build-for": [arch2.value],
                    },
                },
                id=f"{arch1.value}-{arch2.value}",
            )
            for arch1, arch2 in itertools.combinations(const.SnapArch, 2)
        ),
    ],
)
def test_render_legacy_platforms_success(
    mocker: pytest_mock.MockerFixture,
    in_project_path: pathlib.Path,
    raw_project: dict[str, Any],
    expected: dict[str, dict[str, list[str]]],
):
    service = Project(
        app=APP_METADATA,
        services=None,  # pyright: ignore[reportArgumentType] # ty: ignore[invalid-argument-type] other services not needed
        project_dir=in_project_path,
    )
    mocker.patch.object(service, "get_raw", return_value=raw_project)

    assert service._app_render_legacy_platforms() == expected


@pytest.mark.parametrize(
    "raw_project",
    [
        {"base": "core22", "platforms": None},
        {"base": "core22", "platforms": {"s390x": None}},
        {"base": "core22", "platforms": None, "architectures": ["s390x"]},
    ],
)
def test_render_legacy_platforms_core22_platforms_error(
    mocker: pytest_mock.MockerFixture,
    in_project_path: pathlib.Path,
    raw_project: dict[str, Any],
):
    service = Project(
        app=APP_METADATA,
        services=None,  # pyright: ignore[reportArgumentType] # ty: ignore[invalid-argument-type] other services not needed
        project_dir=in_project_path,
    )
    mocker.patch.object(service, "get_raw", return_value=raw_project)

    with pytest.raises(CraftValidationError, match="not supported for base 'core22'"):
        service._app_render_legacy_platforms()
