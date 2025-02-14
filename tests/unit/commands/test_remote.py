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
import shutil
import subprocess
import sys
from pathlib import Path
from unittest.mock import ANY

import pytest
from craft_application.git import GitRepo
from craft_platforms import DebianArchitecture

from snapcraft import application, const

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
        "snapcraft.services.remotebuild.RemoteBuild.start_builds"
    )
    return _mock_start_builds


@pytest.fixture()
def mock_run_legacy(mocker):
    return mocker.patch("snapcraft_legacy.cli.legacy.legacy_run")


#######################
# Control logic tests #
#######################


@pytest.mark.parametrize("base", const.CURRENT_BASES)
@pytest.mark.usefixtures("emitter", "mock_argv", "fake_services")
def test_run_core22_and_later(snapcraft_yaml, base, mock_remote_build_run):
    """Bases that are core22 and later will use craft-application remote-build."""
    snapcraft_yaml_dict = {"base": base, "build-base": "devel", "grade": "devel"}
    snapcraft_yaml(**snapcraft_yaml_dict)
    application.main()

    mock_remote_build_run.assert_called_once()


@pytest.mark.parametrize("base", const.LEGACY_BASES)
@pytest.mark.usefixtures("mock_argv")
def test_run_core20(
    snapcraft_yaml,
    base,
    mock_run_legacy,
    mock_remote_build_run,
):
    """core20 base use fallback remote-build."""
    snapcraft_yaml_dict = {"base": base}
    snapcraft_yaml(**snapcraft_yaml_dict)
    application.main()

    mock_run_legacy.assert_called_once()
    mock_remote_build_run.assert_not_called()


@pytest.mark.parametrize("base", const.CURRENT_BASES)
@pytest.mark.usefixtures("emitter", "mock_argv", "fake_services")
def test_run_in_repo_newer_than_core22(
    snapcraft_yaml, base, new_dir, mock_remote_start_builds
):
    """Bases newer than core22 run craft-application remote-build regardless of being in a repo."""
    # initialize a git repo
    GitRepo(new_dir)
    snapcraft_yaml_dict = {"base": base, "build-base": "devel", "grade": "devel"}
    snapcraft_yaml(**snapcraft_yaml_dict)

    application.main()

    mock_remote_start_builds.assert_called_once()


@pytest.mark.xfail(reason="not implemented in craft-application")
@pytest.mark.parametrize("base", const.CURRENT_BASES)
@pytest.mark.usefixtures(
    "mock_argv",
    "mock_remote_builder_start_builds",
    "fake_services",
)
def test_run_in_shallow_repo_unsupported(capsys, new_dir, snapcraft_yaml, base):
    """devel / core24 and newer bases run new remote-build in a shallow git repo."""
    root_path = Path(new_dir)
    snapcraft_yaml_dict = {"base": base, "build-base": "devel", "grade": "devel"}
    snapcraft_yaml(**snapcraft_yaml_dict)

    git_normal_path = root_path / "normal"
    git_normal_path.mkdir()
    git_shallow_path = root_path / "shallow"

    shutil.move(root_path / "snap", git_normal_path)

    repo_normal = GitRepo(git_normal_path)
    (repo_normal.path / "1").write_text("1")
    repo_normal.add_all()
    repo_normal.commit("1")

    (repo_normal.path / "2").write_text("2")
    repo_normal.add_all()
    repo_normal.commit("2")

    (repo_normal.path / "3").write_text("3")
    repo_normal.add_all()
    repo_normal.commit("3")

    # pygit2 does not support shallow cloning, so we use git directly
    subprocess.run(
        [
            "git",
            "clone",
            "--depth",
            "1",
            git_normal_path.absolute().as_uri(),
            git_shallow_path.absolute().as_posix(),
        ],
        check=True,
    )

    os.chdir(git_shallow_path)

    # no exception because run() catches it
    application.main()
    _, err = capsys.readouterr()

    assert (
        "Remote builds are not supported for projects in shallowly cloned "
        "git repositories."
    ) in err


######################
# Architecture tests #
######################


@pytest.mark.parametrize("base", const.CURRENT_BASES - {"devel"})
@pytest.mark.usefixtures("emitter", "mock_argv", "fake_services")
def test_default_architecture(snapcraft_yaml, base, mock_remote_start_builds):
    """Default to the host architecture if not defined elsewhere."""
    snapcraft_yaml(base=base)
    app = application.create_app()
    app.run()

    mock_remote_start_builds.assert_called_once_with(
        ANY, architectures=[str(DebianArchitecture.from_host())]
    )


@pytest.mark.parametrize("base", const.CURRENT_BASES - {"core22", "devel"})
@pytest.mark.parametrize(
    "mock_argv",
    [pytest.param(None, id="implicit"), pytest.param("all", id="explicit")],
    indirect=True,
)
@pytest.mark.usefixtures("emitter", "mock_argv", "fake_services")
def test_platform_build_for_all(
    snapcraft_yaml,
    base,
    mock_remote_start_builds,
):
    """Use 'build-for: all' from the project metadata with the platforms keyword."""
    snapcraft_yaml_dict = {
        "base": base,
        "platforms": {
            "test-platform": {"build-on": "arm64", "build-for": "all"},
        },
    }
    snapcraft_yaml(**snapcraft_yaml_dict)
    app = application.create_app()
    app.run()

    mock_remote_start_builds.assert_called_once()
    assert mock_remote_start_builds.call_args[1]["architectures"] == ["all"]


@pytest.mark.parametrize(
    "mock_argv",
    [pytest.param(None, id="implicit"), pytest.param("all", id="explicit")],
    indirect=True,
)
@pytest.mark.usefixtures("emitter", "mock_argv", "fake_services")
def test_platform_build_for_all_core22(snapcraft_yaml, mock_remote_start_builds):
    """Use 'build-for: all' from the project metadata with the architectures keyword."""
    snapcraft_yaml_dict = {
        "base": "core22",
        "architectures": [
            {"build-on": ["arm64"], "build-for": ["all"]},
        ],
    }
    snapcraft_yaml(**snapcraft_yaml_dict)
    app = application.create_app()
    app.run()

    mock_remote_start_builds.assert_called_once()
    assert mock_remote_start_builds.call_args[1]["architectures"] == ["all"]


@pytest.mark.parametrize("base", const.CURRENT_BASES - {"core22", "devel"})
@pytest.mark.usefixtures("emitter", "mock_argv", "fake_services")
def test_platform_in_project_metadata(snapcraft_yaml, base, mock_remote_start_builds):
    """Use the platform's build-for architectures from the project metadata."""
    snapcraft_yaml_dict = {
        "base": base,
        "platforms": {
            "rpi4": {"build-on": "arm64", "build-for": "arm64"},
            "x86-64": {"build-on": "amd64", "build-for": "amd64"},
            "same-build-for-x86-64": {"build-on": "riscv64", "build-for": "amd64"},
        },
    }
    snapcraft_yaml(**snapcraft_yaml_dict)
    app = application.create_app()
    app.run()

    mock_remote_start_builds.assert_called_once()
    assert (
        const.SnapArch.arm64 in mock_remote_start_builds.call_args[1]["architectures"]
    )
    assert (
        const.SnapArch.amd64 in mock_remote_start_builds.call_args[1]["architectures"]
    )


@pytest.mark.usefixtures("emitter", "mock_argv", "fake_services")
def test_architecture_in_project_metadata(snapcraft_yaml, mock_remote_start_builds):
    """Use the build-for architectures from the project metadata."""
    snapcraft_yaml_dict = {
        "base": "core22",
        "architectures": [
            {"build-on": ["arm64"], "build-for": ["arm64"]},
            {"build-on": ["riscv64"], "build-for": ["riscv64"]},
        ],
    }
    snapcraft_yaml(**snapcraft_yaml_dict)
    app = application.create_app()
    app.run()

    mock_remote_start_builds.assert_called_once()
    assert sorted(mock_remote_start_builds.call_args[1]["architectures"]) == sorted(
        ["arm64", "riscv64"]
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
@pytest.mark.usefixtures("emitter", "mock_argv", "fake_services")
def test_build_for_argument(
    snapcraft_yaml, base, expected_build_fors, mock_remote_start_builds
):
    """Use architectures provided by the `--build-for` argument."""
    snapcraft_yaml(base=base)
    app = application.create_app()
    app.run()

    mock_remote_start_builds.assert_called_once_with(
        ANY, architectures=expected_build_fors
    )


@pytest.mark.parametrize(
    ("mock_argv", "expected_archs"),
    [
        pytest.param("amd64", ["amd64"], id="amd64"),
        pytest.param("riscv64", ["riscv64"], id="riscv64"),
        pytest.param("amd64,riscv64", ["amd64", "riscv64"], id="both"),
    ],
    indirect=["mock_argv"],
)
@pytest.mark.usefixtures("emitter", "mock_argv", "fake_services")
def test_architectures_filter(snapcraft_yaml, expected_archs, mock_remote_start_builds):
    """Filter an 'architectures' key with '--build-for'."""
    snapcraft_yaml_dict = {
        "base": "core22",
        "architectures": [
            {"build-on": ["amd64"], "build-for": ["amd64"]},
            {"build-on": ["riscv64"], "build-for": ["riscv64"]},
        ],
    }
    snapcraft_yaml(**snapcraft_yaml_dict)
    app = application.create_app()
    app.run()

    mock_remote_start_builds.assert_called_once_with(ANY, architectures=expected_archs)


@pytest.mark.parametrize("mock_argv", ["amd64"], indirect=True)
@pytest.mark.usefixtures("emitter", "mock_argv", "fake_services")
def test_architectures_filter_error(
    capsys,
    snapcraft_yaml,
):
    """Error if '--build-for' entirely filters the build plan."""
    snapcraft_yaml_dict = {
        "base": "core22",
        "architectures": [{"build-on": ["riscv64"], "build-for": ["riscv64"]}],
    }
    snapcraft_yaml(**snapcraft_yaml_dict)
    app = application.create_app()
    app.run()

    _, err = capsys.readouterr()

    assert "No build matches the current execution environment." in err
    assert (
        "Check the project's 'platforms' declaration, and the "
        "'--platform' and '--build-for' parameters."
    ) in err


@pytest.mark.parametrize(
    ("mock_argv", "expected_archs"),
    [
        pytest.param("amd64", ["amd64"], id="amd64"),
        pytest.param("riscv64", ["riscv64"], id="riscv64"),
        pytest.param("amd64,riscv64", ["amd64", "riscv64"], id="both"),
    ],
    indirect=["mock_argv"],
)
@pytest.mark.usefixtures("emitter", "mock_argv", "fake_services")
def test_platforms_filter(snapcraft_yaml, expected_archs, mock_remote_start_builds):
    """Filter a 'platforms' key with '--build-for'."""
    snapcraft_yaml_dict = {
        "base": "core24",
        "platforms": {
            "amd64": {"build-on": "amd64", "build-for": "amd64"},
            "riscv64": {"build-on": "riscv64", "build-for": "riscv64"},
        },
    }
    snapcraft_yaml(**snapcraft_yaml_dict)
    app = application.create_app()
    app.run()

    mock_remote_start_builds.assert_called_once_with(ANY, architectures=expected_archs)


@pytest.mark.parametrize("mock_argv", ["arm64"], indirect=True)
@pytest.mark.usefixtures("emitter", "mock_argv", "fake_services")
def test_platforms_filter_error(
    capsys,
    snapcraft_yaml,
):
    """Error if '--build-for' entirely filters the build plan."""
    snapcraft_yaml_dict = {
        "base": "core24",
        "platforms": {
            "riscv64": {"build-on": "riscv64", "build-for": "riscv64"},
        },
    }
    snapcraft_yaml(**snapcraft_yaml_dict)
    app = application.create_app()
    app.run()

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
@pytest.mark.usefixtures(
    "emitter", "mock_argv", "fake_services", "mock_remote_start_builds"
)
def test_unknown_build_for_error(
    capsys,
    snapcraft_yaml,
    base,
):
    """Error if `--build-for` is not a valid debian architecture."""
    snapcraft_yaml(base=base)
    app = application.create_app()
    assert app.run() == os.EX_CONFIG

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
            ["Building on 'amd64' will create snaps for 'amd64' and 'riscv64'."],
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
                "Building on 'amd64' will create snaps for 'amd64' and 'riscv64'.",
                "Building on 's390x' will create snaps for 'ppc64el' and 's390x'.",
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
            ["Building on 'amd64' will create snaps for 'amd64' and 'riscv64'."],
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
            # this will show 'riscv64' twice because the platform name is different
            ["Building on 'amd64' will create snaps for 'riscv64' and 'riscv64'."],
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
            ["Building on 'amd64' will create snaps for 'amd64' and 'riscv64'."],
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
                "Building on 'amd64' will create snaps for 'amd64' and 'riscv64'.",
                "Building on 's390x' will create snaps for 'riscv64', 'riscv64', and 's390x'.",
            ],
            id="core24-complex",
        ),
    ],
)
@pytest.mark.usefixtures(
    "emitter", "mock_argv", "fake_services", "mock_remote_start_builds"
)
def test_multiple_artifacts_per_build_on(
    check,
    base,
    build_info,
    error_messages,
    capsys,
    snapcraft_yaml,
):
    """Error when multiple artifacts will be produced on one build-on architecture."""
    snapcraft_yaml(**{"base": base, **build_info})
    app = application.create_app()
    assert app.run() == os.EX_CONFIG

    _, err = capsys.readouterr()

    check.is_in(
        "Remote build does not support building multiple snaps on the same architecture",
        err,
    )
    for message in error_messages:
        check.is_in(message, err)
