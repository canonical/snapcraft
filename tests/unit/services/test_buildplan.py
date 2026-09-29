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

import collections
from collections.abc import Collection
from typing import Literal
from unittest import mock

import craft_platforms
import pytest
import pytest_check
from craft_platforms import BuildInfo, DebianArchitecture, DistroBase

import snapcraft.models


@pytest.fixture(
    params=[
        "64-bit-pc",
        "some-phone",
        "risky",
    ]
)
def fake_platform(request: pytest.FixtureRequest) -> str:
    return request.param


@pytest.fixture
def platform_independent_project_data(default_project):
    return {
        **default_project.marshal(),
        "platforms": {
            "platform-independent": {
                "build-on": [
                    str(arch)
                    for arch in DebianArchitecture
                    if arch is not DebianArchitecture.I386
                ],
                "build-for": ["all"],
            }
        },
    }


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
def test_build_planner_create_launchpad_build_plan(
    platforms, expected_build_infos, default_project, fake_services, setup_project
):
    """Test `create_unfiltered_build_plan()` function with different platforms."""
    setup_project(fake_services, {**default_project.marshal(), "platforms": platforms})

    actual_build_infos = fake_services.get("build_plan").create_launchpad_build_plan(
        platforms=None, build_on=None, build_for=None
    )

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
def test_build_planner_create_launchpad_build_plan_core22(
    architectures,
    expected_build_infos,
    default_project,
    fake_services,
    setup_project,
):
    """Test `create_unfiltered_build_plan()` function with different architectures."""
    # creating a project model will convert architectures to platforms
    project = snapcraft.models.Project.unmarshal(
        {**default_project.marshal(), "base": "core22", "architectures": architectures}
    )
    setup_project(fake_services, project.marshal())

    actual_build_infos = fake_services.get("build_plan").create_launchpad_build_plan(
        platforms=None, build_on=None, build_for=None
    )

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

    actual_build_infos = fake_services.get("build_plan").create_launchpad_build_plan(
        platforms=None, build_on=None, build_for=None
    )

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

    actual_build_infos = fake_services.get("build_plan").create_launchpad_build_plan(
        platforms=None, build_on=None, build_for=None
    )

    assert actual_build_infos == [
        BuildInfo(
            build_on=fake_host_architecture,
            build_for=fake_host_architecture,
            build_base=DistroBase("ubuntu", "24.04"),
            platform=str(fake_host_architecture),
        )
    ]


def test_build_planner_all_with_other_builds(
    default_project, fake_services, setup_project
):
    """'build-for: all' cannot be combined with other builds."""
    setup_project(
        fake_services,
        {
            **default_project.marshal(),
            "base": "core24",
            "platforms": {
                "platform1": {
                    "build-on": ["arm64", "armhf"],
                    "build-for": ["arm64"],
                },
                "platform2": {
                    "build-on": ["s390x"],
                    "build-for": ["all"],
                },
            },
        },
    )

    with pytest.raises(craft_platforms.AllOnlyBuildError):
        fake_services.get("build_plan").create_build_plan(
            platforms=None, build_on=None, build_for=None
        )


_base = DistroBase("", "")
_pc_on_amd64_for_amd64 = BuildInfo(
    platform="pc",
    build_on=DebianArchitecture.AMD64,
    build_for=DebianArchitecture.AMD64,
    build_base=_base,
)
_pc_on_amd64_for_i386 = BuildInfo(
    platform="legacy-pc",
    build_on=DebianArchitecture.AMD64,
    build_for=DebianArchitecture.I386,
    build_base=_base,
)
_amd64_on_amd64_for_amd64 = BuildInfo(
    platform="amd64",
    build_on=DebianArchitecture.AMD64,
    build_for=DebianArchitecture.AMD64,
    build_base=_base,
)
_i386_on_amd64_for_i386 = BuildInfo(
    platform="i386",
    build_on=DebianArchitecture.AMD64,
    build_for=DebianArchitecture.I386,
    build_base=_base,
)
_i386_on_i386_for_i386 = BuildInfo(
    platform="i386",
    build_on=DebianArchitecture.I386,
    build_for=DebianArchitecture.I386,
    build_base=_base,
)


@pytest.mark.parametrize(
    ("plan", "platform", "build_for", "build_on", "result"),
    [
        pytest.param(
            [_pc_on_amd64_for_amd64],
            None,
            None,
            DebianArchitecture.AMD64,
            [_pc_on_amd64_for_amd64],
            id="0",
        ),
        pytest.param(
            [_pc_on_amd64_for_amd64],
            "pc",
            None,
            DebianArchitecture.AMD64,
            [_pc_on_amd64_for_amd64],
            id="1",
        ),
        pytest.param(
            [_pc_on_amd64_for_amd64],
            "legacy-pc",
            None,
            DebianArchitecture.AMD64,
            [],
            id="2",
        ),
        pytest.param(
            [_pc_on_amd64_for_amd64],
            None,
            DebianArchitecture.AMD64,
            DebianArchitecture.AMD64,
            [_pc_on_amd64_for_amd64],
            id="3",
        ),
        pytest.param(
            [_pc_on_amd64_for_amd64],
            "pc",
            DebianArchitecture.AMD64,
            DebianArchitecture.AMD64,
            [_pc_on_amd64_for_amd64],
            id="4",
        ),
        pytest.param(
            [_pc_on_amd64_for_amd64],
            "legacy-pc",
            DebianArchitecture.AMD64,
            DebianArchitecture.AMD64,
            [],
            id="5",
        ),
        pytest.param(
            [_pc_on_amd64_for_amd64],
            None,
            DebianArchitecture.I386,
            DebianArchitecture.AMD64,
            [],
            id="6",
        ),
        pytest.param(
            [_pc_on_amd64_for_amd64],
            None,
            DebianArchitecture.AMD64,
            DebianArchitecture.I386,
            [],
            id="7",
        ),
        pytest.param(
            [_pc_on_amd64_for_amd64, _pc_on_amd64_for_i386],
            None,
            DebianArchitecture.I386,
            DebianArchitecture.AMD64,
            [_pc_on_amd64_for_i386],
            id="8",
        ),
        pytest.param(
            [_pc_on_amd64_for_amd64, _pc_on_amd64_for_i386],
            "pc",
            DebianArchitecture.AMD64,
            DebianArchitecture.I386,
            [],
            id="9",
        ),
        pytest.param(
            [_pc_on_amd64_for_amd64, _pc_on_amd64_for_i386],
            "legacy-pc",
            DebianArchitecture.I386,
            DebianArchitecture.AMD64,
            [_pc_on_amd64_for_i386],
            id="10",
        ),
        pytest.param(
            [_pc_on_amd64_for_amd64, _pc_on_amd64_for_i386],
            None,
            DebianArchitecture.I386,
            DebianArchitecture.I386,
            [],
            id="11",
        ),
        pytest.param(
            [_pc_on_amd64_for_amd64, _pc_on_amd64_for_i386],
            None,
            None,
            DebianArchitecture.I386,
            [],
            id="12",
        ),
        pytest.param(
            [_pc_on_amd64_for_amd64, _pc_on_amd64_for_i386],
            "legacy-pc",
            None,
            DebianArchitecture.I386,
            [],
            id="13",
        ),
        pytest.param(
            [_pc_on_amd64_for_amd64, _pc_on_amd64_for_i386],
            None,
            None,
            DebianArchitecture.AMD64,
            [_pc_on_amd64_for_amd64, _pc_on_amd64_for_i386],
            id="14",
        ),
        pytest.param(
            [_pc_on_amd64_for_amd64, _pc_on_amd64_for_i386],
            "legacy-pc",
            None,
            DebianArchitecture.AMD64,
            [_pc_on_amd64_for_i386],
            id="15",
        ),
        pytest.param(
            [_pc_on_amd64_for_amd64, _amd64_on_amd64_for_amd64],
            None,
            DebianArchitecture.AMD64,
            DebianArchitecture.AMD64,
            [_pc_on_amd64_for_amd64, _amd64_on_amd64_for_amd64],
            id="16",
        ),
        pytest.param(
            [_pc_on_amd64_for_amd64, _amd64_on_amd64_for_amd64],
            DebianArchitecture.AMD64,
            None,
            DebianArchitecture.AMD64,
            [_amd64_on_amd64_for_amd64],
            id="17",
        ),
        pytest.param(
            [_pc_on_amd64_for_amd64, _amd64_on_amd64_for_amd64],
            DebianArchitecture.AMD64,
            DebianArchitecture.AMD64,
            DebianArchitecture.AMD64,
            [_amd64_on_amd64_for_amd64],
            id="18",
        ),
        pytest.param(
            [_pc_on_amd64_for_amd64, _i386_on_amd64_for_i386],
            None,
            DebianArchitecture.I386,
            DebianArchitecture.AMD64,
            [_i386_on_amd64_for_i386],
            id="19",
        ),
        pytest.param(
            [_pc_on_amd64_for_amd64, _i386_on_amd64_for_i386],
            DebianArchitecture.AMD64,
            None,
            DebianArchitecture.AMD64,
            [],
            id="20",
        ),
        pytest.param(
            [
                _pc_on_amd64_for_amd64,
                _amd64_on_amd64_for_amd64,
                _i386_on_amd64_for_i386,
            ],
            None,
            DebianArchitecture.AMD64,
            DebianArchitecture.AMD64,
            [_pc_on_amd64_for_amd64, _amd64_on_amd64_for_amd64],
            id="21",
        ),
        pytest.param(
            [_pc_on_amd64_for_amd64, _i386_on_amd64_for_i386],
            DebianArchitecture.I386,
            DebianArchitecture.I386,
            DebianArchitecture.AMD64,
            [_i386_on_amd64_for_i386],
            id="22",
        ),
        pytest.param(
            [
                _pc_on_amd64_for_amd64,
                _amd64_on_amd64_for_amd64,
                _i386_on_amd64_for_i386,
            ],
            None,
            DebianArchitecture.I386,
            DebianArchitecture.AMD64,
            [_i386_on_amd64_for_i386],
            id="23",
        ),
        pytest.param(
            [_pc_on_amd64_for_amd64, _amd64_on_amd64_for_amd64],
            None,
            DebianArchitecture.I386,
            DebianArchitecture.AMD64,
            [],
            id="24",
        ),
        pytest.param(
            [_pc_on_amd64_for_amd64, _i386_on_amd64_for_i386, _i386_on_i386_for_i386],
            DebianArchitecture.AMD64,
            None,
            DebianArchitecture.AMD64,
            [],
            id="25",
        ),
        pytest.param(
            [_pc_on_amd64_for_amd64, _i386_on_amd64_for_i386, _i386_on_i386_for_i386],
            DebianArchitecture.AMD64,
            None,
            DebianArchitecture.I386,
            [],
            id="26",
        ),
        pytest.param(
            [_pc_on_amd64_for_amd64, _i386_on_amd64_for_i386, _i386_on_i386_for_i386],
            DebianArchitecture.I386,
            None,
            DebianArchitecture.AMD64,
            [_i386_on_amd64_for_i386],
            id="27",
        ),
        pytest.param(
            [_pc_on_amd64_for_amd64, _i386_on_amd64_for_i386, _i386_on_i386_for_i386],
            DebianArchitecture.I386,
            None,
            DebianArchitecture.I386,
            [_i386_on_i386_for_i386],
            id="28",
        ),
        pytest.param(
            [_pc_on_amd64_for_amd64, _i386_on_amd64_for_i386, _i386_on_i386_for_i386],
            None,
            DebianArchitecture.I386,
            DebianArchitecture.I386,
            [_i386_on_i386_for_i386],
            id="29",
        ),
        # Filters without a build-on. Which specific items are selected here is an
        # implementation detail and may change.
        pytest.param(
            [
                _pc_on_amd64_for_amd64,
                _pc_on_amd64_for_i386,
                _amd64_on_amd64_for_amd64,
                _i386_on_amd64_for_i386,
                _i386_on_i386_for_i386,
            ],
            None,
            None,
            None,
            [
                _pc_on_amd64_for_amd64,
                _pc_on_amd64_for_i386,
                _amd64_on_amd64_for_amd64,
                _i386_on_amd64_for_i386,
                _i386_on_i386_for_i386,
            ],
            id="empty-filter",
        ),
        pytest.param(
            [
                _pc_on_amd64_for_amd64,
                _pc_on_amd64_for_i386,
                _amd64_on_amd64_for_amd64,
                _i386_on_amd64_for_i386,
            ],
            None,
            "i386",
            None,
            [
                _pc_on_amd64_for_i386,
                _i386_on_amd64_for_i386,
            ],
            id="build-on-anything-for-i386",
        ),
    ],
)
def test_filter_launchpad_plan(
    default_project,
    fake_services,
    setup_project,
    plan: list[BuildInfo],
    platform: str | None,
    build_for: DebianArchitecture | Literal["all"] | None,
    build_on: DebianArchitecture | None,
    result,
):
    setup_project(fake_services, default_project.marshal())
    build_plan_service = fake_services.get("build_plan")

    assert (
        list(
            build_plan_service._filter_launchpad_plan(
                plan,
                platforms=[platform] if platform else None,
                build_for=[build_for] if build_for else None,
                build_on=[build_on] if build_on else None,
            )
        )
        == result
    )


def check_plan(
    plan: Collection[BuildInfo],
    *,
    min_length: int = 1,
    max_length: int = 0,
    build_on: str | None = None,
    build_for: str | None = None,
    platform: str | None = None,
):
    assert len(plan) >= min_length
    if max_length:
        assert len(plan) <= max_length
    platform_items = collections.defaultdict(list)
    for item in plan:
        platform_items[item.platform].append(item)
        if build_on:
            pytest_check.equal(item.build_on, build_on)
        if build_for:
            pytest_check.equal(item.build_for, build_for)
        if platform:
            pytest_check.equal(item.platform, platform)


@pytest.mark.usefixtures("fake_host_architecture")
def test_create_build_plan_no_filter(default_project, fake_services, setup_project):
    setup_project(fake_services, default_project.marshal())
    build_plan_service = fake_services.get("build_plan")

    plan = build_plan_service.create_launchpad_build_plan(
        platforms=None,
        build_for=None,
        build_on=None,
    )

    check_plan(plan)


@pytest.mark.usefixtures("fake_host_architecture")
def test_create_build_plan_no_filter_platform_independent(
    default_project, fake_services, setup_project, platform_independent_project_data
):
    setup_project(fake_services, platform_independent_project_data)
    build_plan_service = fake_services.get("build_plan")

    plan = build_plan_service.create_launchpad_build_plan(
        platforms=None,
        build_for=None,
        build_on=None,
    )

    check_plan(plan)


def test_create_build_plan_platform_filter(
    default_project,
    fake_services,
    setup_project,
    fake_host_architecture,
    fake_platform,
):
    if fake_host_architecture == DebianArchitecture.I386:
        pytest.skip("i386 isn't supported")
    setup_project(
        fake_services,
        {
            **default_project.marshal(),
            "platforms": {
                fake_platform: {
                    "build-on": [str(fake_host_architecture)],
                    "build-for": [str(fake_host_architecture)],
                }
            },
        },
    )
    build_plan_service = fake_services.get("build_plan")

    plan = build_plan_service.create_launchpad_build_plan(
        platforms=[fake_platform],
        build_for=None,
        build_on=None,
    )

    check_plan(
        plan,
        min_length=0,  # Not all platforms in our project can be built on all archs
        platform=fake_platform,
    )


@pytest.mark.usefixtures("fake_host_architecture")
def test_create_build_plan_platform_filter_all(
    default_project, fake_services, setup_project, platform_independent_project_data
):
    setup_project(fake_services, platform_independent_project_data)
    build_plan_service = fake_services.get("build_plan")

    plan = build_plan_service.create_launchpad_build_plan(
        platforms=["platform-independent"],
        build_for=None,
        build_on=None,
    )

    expected_length = len(DebianArchitecture) - 1  # All but i386
    check_plan(
        plan,
        min_length=expected_length,
        max_length=expected_length,
        platform="platform-independent",
    )


@pytest.mark.usefixtures("fake_host_architecture")
@pytest.mark.parametrize("build_for", DebianArchitecture)
def test_create_build_plan_build_for_filter(
    default_project, fake_services, setup_project, build_for
):
    setup_project(fake_services, default_project.marshal())
    build_plan_service = fake_services.get("build_plan")

    plan = build_plan_service.create_launchpad_build_plan(
        platforms=None,
        build_for=[str(build_for)],
        build_on=None,
    )

    check_plan(
        plan,
        min_length=0,  # Not all build-on/build-for combos exist.
        build_for=build_for,
    )


@pytest.mark.usefixtures("fake_host_architecture")
def test_create_build_plan_build_for_filter_platform_independent(
    default_project, fake_services, setup_project, platform_independent_project_data
):
    setup_project(fake_services, platform_independent_project_data)
    build_plan_service = fake_services.get("build_plan")

    plan = build_plan_service.create_launchpad_build_plan(
        platforms=None,
        build_for=["all"],
        build_on=None,
    )

    expected_length = len(DebianArchitecture) - 1  # All but i386
    check_plan(
        plan,
        min_length=expected_length,
        max_length=expected_length,
        build_for="all",
    )


@pytest.mark.parametrize("build_on", DebianArchitecture)
def test_create_build_plan_build_on_filter(
    default_project,
    fake_services,
    setup_project,
    emitter,
    fake_host_architecture,
    build_on,
):
    build_plan_service = fake_services.get("build_plan")
    plan = build_plan_service.create_launchpad_build_plan(
        platforms=None,
        build_for=None,
        build_on=[build_on],
    )

    assert (
        mock.call(
            "trace",
            f"No build-on filter set, using the default of {[fake_host_architecture.value]}",
        )
        not in emitter.interactions
    )

    check_plan(
        plan,
        min_length=0,  # Not all build-on/build-for combos exist.
        build_on=build_on,
    )


@pytest.mark.parametrize(
    "build_on", [arch for arch in DebianArchitecture if arch != DebianArchitecture.I386]
)
def test_create_build_plan_build_on_filter_platform_independent(
    default_project,
    fake_services,
    setup_project,
    emitter,
    fake_host_architecture,
    platform_independent_project_data,
    build_on,
):
    setup_project(fake_services, platform_independent_project_data)
    build_plan_service = fake_services.get("build_plan")

    plan = build_plan_service.create_launchpad_build_plan(
        platforms=None,
        build_for=None,
        build_on=[build_on],
    )

    assert (
        mock.call(
            "trace",
            f"No build-on filter set, using the default of {[fake_host_architecture.value]}",
        )
        not in emitter.interactions
    )

    check_plan(
        plan,
        max_length=1,
        build_on=build_on,
    )


@pytest.mark.parametrize("build_for", DebianArchitecture)
def test_create_build_plan_multiple_filters(
    default_project,
    fake_services,
    setup_project,
    fake_platform,
    fake_host_architecture,
    build_for,
):
    build_plan_service = fake_services.get("build_plan")
    plan = build_plan_service.create_launchpad_build_plan(
        platforms=[fake_platform],
        build_for=[build_for],
        build_on=[fake_host_architecture],
    )

    check_plan(
        plan,
        min_length=0,
        max_length=1,
        platform=fake_platform,
        build_for=build_for,
        build_on=fake_host_architecture,
    )


def test_create_build_plan_filters_to_empty(
    default_project, fake_services, setup_project
):
    build_plan_service = fake_services.get("build_plan")
    assert (
        build_plan_service.create_launchpad_build_plan(
            platforms=["not-a-platform"],
            build_for=None,
            build_on=None,
        )
        == []
    )
