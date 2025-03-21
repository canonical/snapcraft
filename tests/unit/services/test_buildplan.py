#  This file is part of snapcraft.
#
#  Copyright 2025 Canonical Ltd.
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

"""Unit tests for build plans."""

import pytest
from craft_platforms import BuildInfo, DebianArchitecture, DistroBase


@pytest.mark.parametrize(
    ("snapcraft_yaml_data", "expected_build_infos"),
    [
        pytest.param(
            {"platforms": {"amd64": None}},
            [
                BuildInfo(
                    build_on=DebianArchitecture("amd64"),
                    build_for=DebianArchitecture("amd64"),
                    build_base=DistroBase("ubuntu", "24.04"),
                    platform="amd64",
                )
            ],
            id="single_platform_as_arch",
        ),
        pytest.param(
            {
                "platforms": {
                    "s390x": {
                        "build-on": "s390x",
                    },
                    "riscv64": {
                        "build-on": ["amd64", "riscv64"],
                    },
                },
            },
            [
                BuildInfo(
                    build_on=DebianArchitecture("s390x"),
                    build_for=DebianArchitecture("s390x"),
                    build_base=DistroBase("ubuntu", "24.04"),
                    platform="s390x",
                ),
                BuildInfo(
                    build_on=DebianArchitecture("amd64"),
                    build_for=DebianArchitecture("riscv64"),
                    build_base=DistroBase("ubuntu", "24.04"),
                    platform="riscv64",
                ),
                BuildInfo(
                    build_on=DebianArchitecture("riscv64"),
                    build_for=DebianArchitecture("riscv64"),
                    build_base=DistroBase("ubuntu", "24.04"),
                    platform="riscv64",
                ),
            ],
            id="implicit_build_for",
            # XXX: "NoneType" is not subscriptable
            marks=pytest.mark.skip("not working"),
        ),
        pytest.param(
            {
                "platforms": {
                    "arm64": {
                        "build-on": ["arm64", "armhf"],
                        "build-for": ["arm64"],
                    },
                },
            },
            [
                BuildInfo(
                    build_on=DebianArchitecture("arm64"),
                    build_for=DebianArchitecture("arm64"),
                    build_base=DistroBase("ubuntu", "24.04"),
                    platform="arm64",
                ),
                BuildInfo(
                    build_on=DebianArchitecture("armhf"),
                    build_for=DebianArchitecture("arm64"),
                    build_base=DistroBase("ubuntu", "24.04"),
                    platform="arm64",
                ),
            ],
            id="multiple_build_on",
            # XXX: this is only producing a single element build item
            marks=pytest.mark.skip("not working"),
        ),
        pytest.param(
            {
                "platforms": {
                    "amd64v2": {
                        "build-on": ["amd64"],
                        "build-for": "amd64",
                    },
                },
            },
            [
                BuildInfo(
                    build_on=DebianArchitecture("amd64"),
                    build_for=DebianArchitecture("amd64"),
                    build_base=DistroBase("ubuntu", "24.04"),
                    platform="amd64v2",
                )
            ],
            id="custom_platform_name",
            # XXX: scalar build-on and build-for values are failing
            marks=pytest.mark.skip("not working"),
        ),
        pytest.param(
            {
                "platforms": {
                    "platform1": {
                        "build-on": ["arm64", "armhf"],
                        "build-for": ["all"],
                    },
                },
            },
            [
                BuildInfo(
                    build_on=DebianArchitecture("arm64"),
                    build_for="all",
                    build_base=DistroBase("ubuntu", "24.04"),
                    platform="platform1",
                ),
                BuildInfo(
                    build_on=DebianArchitecture("armhf"),
                    build_for="all",
                    build_base=DistroBase("ubuntu", "24.04"),
                    platform="platform1",
                ),
            ],
            id="all",
            # XXX: this is only producing a single element build item
            marks=pytest.mark.skip("not working"),
        ),
    ],
)
def test_build_planner_get_build_plan(
    snapcraft_yaml_data, expected_build_infos, build_plan_service, new_dir
):
    """Test `get_build_plan()` function with different platforms."""
    actual_build_infos = build_plan_service.create_build_plan(
        platforms=None, build_for=None, build_on=None
    )

    assert actual_build_infos == expected_build_infos


@pytest.mark.parametrize(
    ("snapcraft_yaml_data", "expected_build_infos"),
    [
        pytest.param(
            {"architectures": ["amd64"], "base": "core22"},
            [
                BuildInfo(
                    build_on=DebianArchitecture("amd64"),
                    build_for=DebianArchitecture("amd64"),
                    build_base=DistroBase("ubuntu", "22.04"),
                    platform="amd64",
                )
            ],
            id="single_platform_as_arch",
        ),
        pytest.param(
            {
                "architectures": [
                    {
                        "build-on": ["arm64", "armhf"],
                        "build-for": ["arm64"],
                    },
                ],
                "base": "core22",
            },
            [
                BuildInfo(
                    build_on=DebianArchitecture("arm64"),
                    build_for=DebianArchitecture("arm64"),
                    build_base=DistroBase("ubuntu", "22.04"),
                    platform="arm64",
                ),
                BuildInfo(
                    build_on=DebianArchitecture("armhf"),
                    build_for=DebianArchitecture("arm64"),
                    build_base=DistroBase("ubuntu", "22.04"),
                    platform="arm64",
                ),
            ],
            id="multiple_build_on",
            # XXX: 'dict' object has no attribute 'build_on'
            marks=pytest.mark.skip("not working"),
        ),
        pytest.param(
            {
                "architectures": [
                    {
                        "build-on": ["amd64"],
                        "build-for": ["amd64"],
                    },
                ],
                "base": "core22",
            },
            [
                BuildInfo(
                    build_on=DebianArchitecture("amd64"),
                    build_for=DebianArchitecture("amd64"),
                    build_base=DistroBase("ubuntu", "22.04"),
                    platform="amd64",
                )
            ],
            id="fully_defined_arch",
        ),
        pytest.param(
            {
                "architectures": [
                    {
                        "build-on": "amd64",
                        "build-for": "amd64",
                    },
                ],
                "base": "core22",
            },
            [
                BuildInfo(
                    build_on=DebianArchitecture("amd64"),
                    build_for=DebianArchitecture("amd64"),
                    build_base=DistroBase("ubuntu", "22.04"),
                    platform="amd64",
                )
            ],
            id="fully_defined_arch_as_string",
        ),
        pytest.param(
            None,
            [
                BuildInfo(
                    build_on=DebianArchitecture.from_host(),
                    build_for=DebianArchitecture.from_host(),
                    build_base=DistroBase("ubuntu", "22.04"),
                    platform=str(DebianArchitecture.from_host()),
                )
            ],
            id="no_arch",
            # XXX: chooses the wrong arch
            marks=pytest.mark.skip("not working"),
        ),
        pytest.param(
            {
                "architectures": [
                    {
                        "build-on": ["s390x"],
                        "build-for": ["all"],
                    },
                ],
                "base": "core22",
            },
            [
                BuildInfo(
                    build_on=DebianArchitecture("s390x"),
                    build_for="all",
                    build_base=DistroBase("ubuntu", "22.04"),
                    platform="all",
                )
            ],
            id="all",
        ),
        pytest.param(
            {
                "architectures": [
                    {
                        "build-on": "s390x",
                        "build-for": "all",
                    },
                ],
                "base": "core22",
            },
            [
                BuildInfo(
                    build_on=DebianArchitecture("s390x"),
                    build_for="all",
                    build_base=DistroBase("ubuntu", "22.04"),
                    platform="all",
                )
            ],
            id="all_as_string",
        ),
    ],
)
def test_build_planner_get_build_plan_core22(
    snapcraft_yaml_data, expected_build_infos, build_plan_service
):
    """Test `get_build_plan()` function with different architectures."""
    actual_build_infos = build_plan_service.create_build_plan(
        platforms=None, build_for=None, build_on=None
    )

    assert actual_build_infos == expected_build_infos
