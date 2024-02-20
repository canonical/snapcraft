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
"""Unit tests for application classes."""

import pytest
from craft_providers import bases

from snapcraft import application
from snapcraft.models.project import Architecture


@pytest.fixture(
    params=[
        ["amd64"],
        ["riscv64"],
        ["amd64", "riscv64", "s390x"],
        [Architecture(build_on="amd64", build_for="riscv64")],
    ]
)
def architectures(request):
    return request.param


@pytest.mark.parametrize(
    ("base", "expected_base"),
    [
        ("core20", bases.BaseName("ubuntu", "20.04")),
        ("core22", bases.BaseName("ubuntu", "22.04")),
        ("core24", bases.BaseName("ubuntu", "24.04")),
    ],
)
def test_build_planner_success_default_architecture(base, expected_base):
    data = {
        "base": base,
    }

    planner = application.SnapcraftBuildPlanner.parse_obj(data)

    actual = planner.get_build_plan()

    for build_info in actual:
        assert build_info.base == expected_base
        assert [build_info.build_for] in [a.build_for for a in planner.architectures]
        assert [build_info.build_on] in [a.build_on for a in planner.architectures]
        assert build_info.platform == f"{expected_base.name}@{expected_base.version}"


@pytest.mark.parametrize(
    ("base", "expected_base"),
    [
        ("core20", bases.BaseName("ubuntu", "20.04")),
        ("core22", bases.BaseName("ubuntu", "22.04")),
        ("core24", bases.BaseName("ubuntu", "24.04")),
    ],
)
def test_build_planner_success_base_only(architectures, base, expected_base):
    data = {
        "architectures": architectures,
        "base": base,
    }

    planner = application.SnapcraftBuildPlanner.parse_obj(data)

    actual = planner.get_build_plan()

    for build_info in actual:
        assert build_info.base == expected_base
        assert [build_info.build_for] in [a.build_for for a in planner.architectures]
        assert [build_info.build_on] in [a.build_on for a in planner.architectures]
        assert build_info.platform == f"{expected_base.name}@{expected_base.version}"


@pytest.mark.parametrize("base", ["core20", "core22", "core24"])
@pytest.mark.parametrize(
    ("build_base", "expected_base"),
    [
        ("core20", bases.BaseName("ubuntu", "20.04")),
        ("core22", bases.BaseName("ubuntu", "22.04")),
        ("core24", bases.BaseName("ubuntu", "24.04")),
    ],
)
def test_build_planner_success_build_base(
    architectures, base, build_base, expected_base
):
    data = {
        "architectures": architectures,
        "base": base,
        "build-base": build_base,
    }

    planner = application.SnapcraftBuildPlanner.parse_obj(data)

    actual = planner.get_build_plan()

    for build_info in actual:
        assert build_info.base == expected_base
        assert [build_info.build_for] in [a.build_for for a in planner.architectures]
        assert [build_info.build_on] in [a.build_on for a in planner.architectures]
        assert build_info.platform == f"{expected_base.name}@{expected_base.version}"


@pytest.mark.parametrize("base", ["core20", "core22", "core24"])
@pytest.mark.parametrize(
    ("build_base", "expected_base"),
    [
        ("core20", bases.BaseName("ubuntu", "20.04")),
        ("core22", bases.BaseName("ubuntu", "22.04")),
        ("core24", bases.BaseName("ubuntu", "24.04")),
    ],
)
def test_build_planner_success_architecture_all(
    base, build_base, expected_base
):
    data = {
        "architectures": [{"build-on": ["amd64"], "build-for": "all"}],
        "base": base,
        "build-base": build_base,
    }

    planner = application.SnapcraftBuildPlanner.parse_obj(data)

    actual = planner.get_build_plan()

    for build_info in actual:
        assert build_info.base == expected_base
        assert [build_info.build_on] in [a.build_on for a in planner.architectures]
        assert build_info.platform == f"{expected_base.name}@{expected_base.version}"

    assert "all" not in [a.build_on for a in planner.architectures]
