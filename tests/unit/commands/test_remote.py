# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022-2024 Canonical Ltd.
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

"""Remote-build command tests."""

import os
import sys
from unittest.mock import ANY

import pytest
from craft_platforms import DebianArchitecture

from snapcraft import const

# remote-build control logic may check if the working dir is a git repo,
# so execute all tests inside a test directory
pytestmark = pytest.mark.usefixtures("new_dir")


@pytest.fixture()
def mock_argv(mocker, request):
    """Mock `snapcraft remote-build` cli."""
    args = ["snapcraft", "remote-build", "--launchpad-accept-public-upload"]
    # Append build-fors if set through parametrization
    if getattr(request, "param", None) is not None:
        args.extend(["--build-for", str(request.param)])
    return mocker.patch.object(sys, "argv", args)


@pytest.fixture()
def mock_remote_build_run(mocker):
    _mock_remote_build_run = mocker.patch(
        "snapcraft.commands.remote.RemoteBuildCommand._run"
    )
    return _mock_remote_build_run


@pytest.fixture()
def mock_remote_start_builds(mocker):
    _mock_start_builds = mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.start_builds"
    )
    return _mock_start_builds


#######################
# Control logic tests #
#######################


@pytest.mark.parametrize("base", const.CURRENT_BASES)
@pytest.mark.usefixtures("emitter", "mock_argv")
def test_run(snapcraft_yaml, base, mock_remote_build_run, fake_app):
    """Run a remote build."""
    snapcraft_yaml_dict = {"base": base, "build-base": "devel", "grade": "devel"}
    snapcraft_yaml(**snapcraft_yaml_dict)

    fake_app.run()

    mock_remote_build_run.assert_called_once()


######################
# Architecture tests #
######################


@pytest.mark.parametrize("base", const.CURRENT_BASES - {"devel"})
@pytest.mark.usefixtures("emitter", "mock_argv")
def test_default_architecture(
    default_project,
    fake_services,
    setup_project,
    base,
    mock_remote_start_builds,
    fake_app,
):
    """Default to the host architecture if not defined elsewhere."""
    setup_project(
        fake_services, {**default_project.marshal(), "base": base}, write_project=True
    )

    fake_app.run()

    mock_remote_start_builds.assert_called_once_with(
        ANY, architectures=[str(DebianArchitecture.from_host())]
    )


@pytest.mark.parametrize("base", const.CURRENT_BASES - {"core22", "devel"})
@pytest.mark.parametrize(
    "mock_argv",
    [pytest.param(None, id="implicit"), pytest.param("all", id="explicit")],
    indirect=True,
)
@pytest.mark.usefixtures("emitter", "mock_argv")
def test_platform_build_for_all(
    default_project,
    fake_services,
    setup_project,
    base,
    mock_remote_start_builds,
    fake_app,
):
    """Use the 'build-on' archs when building for all architectures."""
    setup_project(
        fake_services,
        {
            **default_project.marshal(),
            "base": base,
            "platforms": {
                "test-platform": {"build-on": ["arm64", "riscv64"], "build-for": "all"},
            },
        },
        write_project=True,
    )

    fake_app.run()

    mock_remote_start_builds.assert_called_once()
    # launchpad decides which build-on to use
    assert sorted(mock_remote_start_builds.call_args[1]["architectures"]) == sorted(
        ["arm64", "riscv64"]
    )


@pytest.mark.parametrize(
    "mock_argv",
    [pytest.param(None, id="implicit"), pytest.param("all", id="explicit")],
    indirect=True,
)
@pytest.mark.usefixtures("emitter", "mock_argv")
def test_platform_build_for_all_core22(
    default_project, fake_services, setup_project, mock_remote_start_builds, fake_app
):
    """Use the 'build-on' archs when building for all architectures."""
    setup_project(
        fake_services,
        {
            **default_project.marshal(),
            "base": "core22",
            "architectures": [
                {"build-on": ["amd64", "riscv64"], "build-for": ["all"]},
            ],
        },
        write_project=True,
    )

    fake_app.run()

    mock_remote_start_builds.assert_called_once()
    # launchpad decides which build-on to use
    assert sorted(mock_remote_start_builds.call_args[1]["architectures"]) == sorted(
        ["amd64", "riscv64"]
    )


@pytest.mark.parametrize("base", const.CURRENT_BASES - {"core22", "devel"})
@pytest.mark.usefixtures("emitter", "mock_argv")
def test_platform_in_project_metadata(
    default_project,
    fake_services,
    setup_project,
    base,
    mock_remote_start_builds,
    fake_app,
):
    """Use the platform's build-for architectures from the project metadata."""
    setup_project(
        fake_services,
        {
            **default_project.marshal(),
            "base": base,
            "platforms": {
                "rpi4": {"build-on": "arm64", "build-for": "arm64"},
                "x86-64": {"build-on": "amd64", "build-for": "amd64"},
                "same-build-for-x86-64": {"build-on": "riscv64", "build-for": "amd64"},
            },
        },
        write_project=True,
    )

    fake_app.run()

    mock_remote_start_builds.assert_called_once()
    assert sorted(mock_remote_start_builds.call_args[1]["architectures"]) == sorted(
        ["arm64", "amd64", "riscv64"]
    )


@pytest.mark.usefixtures("emitter", "mock_argv")
def test_architecture_in_project_metadata(
    default_project, fake_services, setup_project, mock_remote_start_builds, fake_app
):
    """Use the build-for architectures from the project metadata."""
    setup_project(
        fake_services,
        {
            **default_project.marshal(),
            "base": "core22",
            "architectures": [
                {"build-on": ["arm64", "s390x"], "build-for": ["arm64"]},
                {"build-on": ["riscv64"], "build-for": ["riscv64"]},
            ],
        },
        write_project=True,
    )

    fake_app.run()

    mock_remote_start_builds.assert_called_once()
    assert sorted(mock_remote_start_builds.call_args[1]["architectures"]) == sorted(
        ["arm64", "s390x", "riscv64"]
    )


@pytest.mark.parametrize("base", const.CURRENT_BASES - {"devel"})
@pytest.mark.parametrize(
    ("mock_argv", "expected_build_fors"),
    [
        *(
            pytest.param(*arch_and_list, id=f"zipped-{arch_and_list[0]}")
            for arch_and_list in zip(
                const.SnapArch, [[arch] for arch in const.SnapArch]
            )
        ),
        pytest.param("amd64,riscv64", ["amd64", "riscv64"], id="two-arch"),
        pytest.param(
            "amd64,riscv64,s390x", ["amd64", "riscv64", "s390x"], id="three-arch"
        ),
        pytest.param(" amd64 , riscv64 ", ["amd64", "riscv64"], id="with-whitespace"),
        pytest.param(
            "amd64,amd64,riscv64",
            ["amd64", "amd64", "riscv64"],
            id="duplicates-passthrough-to-launchpad",
        ),
    ],
    indirect=["mock_argv"],
)
@pytest.mark.usefixtures("emitter", "mock_argv")
def test_build_for_argument(
    default_project,
    fake_services,
    setup_project,
    base,
    expected_build_fors,
    mock_remote_start_builds,
    fake_app,
):
    """Use architectures provided by the `--build-for` argument."""
    setup_project(
        fake_services, {**default_project.marshal(), "base": base}, write_project=True
    )

    fake_app.run()

    mock_remote_start_builds.assert_called_once_with(
        ANY, architectures=expected_build_fors
    )


@pytest.mark.parametrize(
    ("mock_argv", "expected_archs"),
    [
        pytest.param(
            "amd64",
            ["amd64", "s390x"],
            id="amd64",
        ),
        pytest.param("riscv64", ["riscv64"], id="riscv64"),
        pytest.param(
            "amd64,riscv64",
            ["amd64", "s390x", "riscv64"],
            id="both",
        ),
    ],
    indirect=["mock_argv"],
)
@pytest.mark.usefixtures("emitter", "mock_argv")
def test_architectures_filter(
    default_project,
    fake_services,
    setup_project,
    expected_archs,
    mock_remote_start_builds,
    fake_app,
):
    """Filter an 'architectures' key with '--build-for'."""
    setup_project(
        fake_services,
        {
            **default_project.marshal(),
            "base": "core22",
            "architectures": [
                {"build-on": ["amd64", "s390x"], "build-for": ["amd64"]},
                {"build-on": ["riscv64"], "build-for": ["riscv64"]},
            ],
        },
        write_project=True,
    )

    fake_app.run()

    mock_remote_start_builds.assert_called_once_with(ANY, architectures=expected_archs)


@pytest.mark.parametrize("mock_argv", ["amd64"], indirect=True)
@pytest.mark.usefixtures("emitter", "mock_argv")
def test_architectures_filter_error(
    default_project, fake_services, setup_project, capsys, fake_app
):
    """Error if '--build-for' entirely filters the build plan."""
    setup_project(
        fake_services,
        {
            **default_project.marshal(),
            "base": "core22",
            "architectures": [{"build-on": ["riscv64"], "build-for": ["riscv64"]}],
        },
        write_project=True,
    )

    fake_app.run()

    _, err = capsys.readouterr()

    assert "No build matches the current execution environment." in err
    assert (
        "Check the project's 'platforms' declaration, and the "
        "'--platform' and '--build-for' parameters."
    ) in err


@pytest.mark.parametrize(
    ("mock_argv", "expected_archs"),
    [
        pytest.param(
            "amd64",
            ["amd64", "s390x"],
            id="amd64",
        ),
        pytest.param("riscv64", ["riscv64"], id="riscv64"),
        pytest.param(
            "amd64,riscv64",
            ["amd64", "s390x", "riscv64"],
            id="both",
        ),
    ],
    indirect=["mock_argv"],
)
@pytest.mark.usefixtures("emitter", "mock_argv")
def test_platforms_filter(
    default_project,
    fake_services,
    setup_project,
    expected_archs,
    mock_remote_start_builds,
    fake_app,
):
    """Filter a 'platforms' key with '--build-for'."""
    setup_project(
        fake_services,
        {
            **default_project.marshal(),
            "base": "core24",
            "platforms": {
                "amd64": {"build-on": ["amd64", "s390x"], "build-for": "amd64"},
                "riscv64": {"build-on": "riscv64", "build-for": "riscv64"},
            },
        },
        write_project=True,
    )

    fake_app.run()

    mock_remote_start_builds.assert_called_once_with(ANY, architectures=expected_archs)


@pytest.mark.parametrize("mock_argv", ["arm64"], indirect=True)
@pytest.mark.usefixtures("emitter", "mock_argv")
def test_platforms_filter_error(
    default_project, fake_services, setup_project, capsys, fake_app, new_dir
):
    """Error if '--build-for' entirely filters the build plan."""
    setup_project(
        fake_services,
        {
            **default_project.marshal(),
            "base": "core24",
            "platforms": {
                "riscv64": {"build-on": "riscv64", "build-for": "riscv64"},
            },
        },
        write_project=True,
    )

    fake_app.run()

    _, err = capsys.readouterr()

    assert "No build matches the current execution environment." in err
    assert (
        "Check the project's 'platforms' declaration, and the "
        "'--platform' and '--build-for' parameters."
    ) in err


@pytest.mark.parametrize(
    "mock_argv",
    [
        "nonexistent",
        "nonexistent,riscv64",
        "riscv64,nonexistent",
        "riscv64,nonexistent,amd64",
    ],
    indirect=True,
)
@pytest.mark.parametrize("base", const.CURRENT_BASES - {"core22", "devel"})
@pytest.mark.usefixtures("emitter", "mock_argv", "mock_remote_start_builds")
def test_unknown_build_for_error(
    default_project, fake_services, setup_project, capsys, base, fake_app
):
    """Error if `--build-for` is not a valid debian architecture."""
    setup_project(fake_services, {**default_project.marshal(), "base": base})

    assert fake_app.run() == os.EX_CONFIG

    _, err = capsys.readouterr()

    assert "Unsupported build-for architecture 'nonexistent'" in err
    assert (
        "Recommended resolution: Use a supported Debian architecture. "
        "Supported architectures are:"
    ) in err


@pytest.mark.parametrize(
    ("base", "build_info", "error_messages"),
    [
        pytest.param(
            "core22",
            {
                "architectures": [
                    {"build-on": "amd64", "build-for": "amd64"},
                    {"build-on": ["amd64", "riscv64"], "build-for": "riscv64"},
                ],
            },
            ["Building on amd64 will create snaps for amd64 and riscv64."],
            id="core22-simple",
        ),
        pytest.param(
            "core22",
            {
                "architectures": [
                    {"build-on": "amd64", "build-for": "amd64"},
                    {"build-on": ["amd64", "riscv64"], "build-for": "riscv64"},
                    {"build-on": "s390x", "build-for": "s390x"},
                    {"build-on": "s390x", "build-for": "ppc64el"},
                ],
            },
            [
                "Building on amd64 will create snaps for amd64 and riscv64.",
                "Building on s390x will create snaps for ppc64el and s390x.",
            ],
            id="core22-complex",
        ),
        pytest.param(
            "core24",
            {
                "platforms": {
                    "platform1": {"build-on": "amd64", "build-for": "amd64"},
                    "platform2": {
                        "build-on": ["amd64", "riscv64"],
                        "build-for": "riscv64",
                    },
                },
            },
            ["Building on amd64 will create snaps for amd64 and riscv64."],
            id="core24-simple",
        ),
        pytest.param(
            "core24",
            {
                "platforms": {
                    "platform1": {"build-on": "amd64", "build-for": "riscv64"},
                    "identical-platform": {"build-on": "amd64", "build-for": "riscv64"},
                },
            },
            # this will show riscv64 twice because the platform name is different
            ["Building on amd64 will create snaps for riscv64 and riscv64."],
            id="core24-same-architectures-different-platform-name",
        ),
        pytest.param(
            "core24",
            {
                "platforms": {
                    "amd64": None,
                    "riscv64": {"build-on": "amd64", "build-for": "riscv64"},
                },
            },
            ["Building on amd64 will create snaps for amd64 and riscv64."],
            id="core24-shorthand",
        ),
        pytest.param(
            "core24",
            {
                "platforms": {
                    "platform1": {"build-on": "amd64", "build-for": "amd64"},
                    "platform2": {
                        "build-on": ["amd64", "s390x"],
                        "build-for": "riscv64",
                    },
                    "s390x": None,
                    "platform4": {"build-on": "s390x", "build-for": "riscv64"},
                },
            },
            [
                "Building on amd64 will create snaps for amd64 and riscv64.",
                "Building on s390x will create snaps for riscv64, riscv64, and s390x.",
            ],
            id="core24-complex",
        ),
    ],
)
@pytest.mark.usefixtures("emitter", "mock_argv", "mock_remote_start_builds")
def test_multiple_artifacts_per_build_on(
    default_project,
    fake_services,
    setup_project,
    check,
    base,
    build_info,
    error_messages,
    capsys,
    fake_app,
):
    """Error when multiple artifacts will be produced on one build-on architecture."""
    setup_project(
        fake_services,
        {
            **default_project.marshal(),
            "base": base,
            **build_info,
        },
    )

    assert fake_app.run() == os.EX_CONFIG

    _, err = capsys.readouterr()

    check.is_in(
        "Remote build does not support building multiple snaps on the same architecture",
        err,
    )
    for message in error_messages:
        check.is_in(message, err)
