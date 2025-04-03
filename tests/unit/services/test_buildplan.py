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

import snapcraft.models


@pytest.mark.parametrize(
    ("platforms", "expected_build_infos"),
    [
        pytest.param(
            {"amd64": None},
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
                "s390x": {
                    "build-on": "s390x",
                },
                "riscv64": {
                    "build-on": ["amd64", "riscv64"],
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
        ),
        pytest.param(
            {
                "arm64": {
                    "build-on": ["arm64", "armhf"],
                    "build-for": ["arm64"],
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
        ),
        pytest.param(
            {
                "amd64v2": {
                    "build-on": ["amd64"],
                    "build-for": "amd64",
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
        ),
        pytest.param(
            {
                "platform1": {
                    "build-on": ["arm64", "armhf"],
                    "build-for": ["all"],
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
        ),
    ],
)
def test_build_planner_create_unfiltered_build_plan(
    platforms, expected_build_infos, default_project, fake_services, setup_project
):
    """Test `create_unfiltered_build_plan()` function with different platforms."""
    setup_project(fake_services, {**default_project.marshal(), "platforms": platforms})

    actual_build_infos = fake_services.get("build_plan").create_unfiltered_build_plan()

    assert actual_build_infos == expected_build_infos


@pytest.mark.parametrize(
    ("architectures", "expected_build_infos"),
    [
        pytest.param(
            ["amd64"],
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
            [
                {
                    "build-on": ["arm64", "armhf"],
                    "build-for": ["arm64"],
                },
            ],
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
        ),
        pytest.param(
            [
                {
                    "build-on": ["amd64"],
                    "build-for": ["amd64"],
                },
            ],
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
            [
                {
                    "build-on": "amd64",
                    "build-for": "amd64",
                },
            ],
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
            [],
            [
                BuildInfo(
                    build_on=DebianArchitecture.from_host(),
                    build_for=DebianArchitecture.from_host(),
                    build_base=DistroBase("ubuntu", "22.04"),
                    platform=str(DebianArchitecture.from_host()),
                )
            ],
            id="no_arch",
        ),
        pytest.param(
            [
                {
                    "build-on": ["s390x"],
                    "build-for": ["all"],
                },
            ],
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
            [
                {
                    "build-on": "s390x",
                    "build-for": "all",
                },
            ],
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
def test_build_planner_create_unfiltered_build_plan_core22(
    architectures,
    expected_build_infos,
    default_project,
    fake_services,
    setup_project,
):
    """Test `create_unfiltered_build_plan()` function with different architectures."""
    # creating a project model will convert architectures to platforms
    project = snapcraft.models.Project.model_validate(
        {**default_project.marshal(), "base": "core22", "architectures": architectures}
    )
    setup_project(fake_services, project.marshal())

    actual_build_infos = fake_services.get("build_plan").create_unfiltered_build_plan()

    assert actual_build_infos == expected_build_infos


def test_get_build_plan_devel(default_project, fake_services, setup_project):
    """Test that "devel" build-bases are correctly reflected on the build plan."""
    setup_project(
        fake_services,
        {
            **default_project.marshal(),
            "build-base": "devel",
            "platforms": {"amd64": None},
        },
    )

    actual_build_infos = fake_services.get("build_plan").create_unfiltered_build_plan()

    assert actual_build_infos == [
        BuildInfo(
            build_on=DebianArchitecture("amd64"),
            build_for=DebianArchitecture("amd64"),
            build_base=DistroBase("ubuntu", "devel"),
            platform="amd64",
        )
    ]


def test_platform_default(
    fake_host_architecture, default_project, fake_services, setup_project
):
    """Default value for platforms is the host architecture."""
    setup_project(fake_services, default_project.marshal())

    actual_build_infos = fake_services.get("build_plan").create_unfiltered_build_plan()

    assert actual_build_infos == [
        BuildInfo(
            build_on=fake_host_architecture,
            build_for=fake_host_architecture,
            build_base=DistroBase("ubuntu", "24.04"),
            platform=str(fake_host_architecture),
        )
    ]
