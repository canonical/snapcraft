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

import argparse
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path
from unittest.mock import ANY, Mock

import craft_cli
import pytest
from craft_application import launchpad
from craft_application.errors import RemoteBuildError
from craft_application.launchpad.models import BuildState
from craft_application.remote.git import GitRepo
from craft_application.remote.utils import get_build_id

from snapcraft import application, commands, const, models
from snapcraft.errors import ClassicFallback
from snapcraft.utils import get_host_architecture

# remote-build control logic may check if the working dir is a git repo,
# so execute all tests inside a test directory
pytestmark = pytest.mark.usefixtures("new_dir")


@pytest.fixture()
def create_snapcraft_yaml(request, snapcraft_yaml):
    """Create a snapcraft.yaml file with a particular base."""
    snapcraft_yaml(base=request.param)


@pytest.fixture()
def fake_sudo(monkeypatch):
    monkeypatch.setenv("SUDO_USER", "fake")
    monkeypatch.setattr("os.geteuid", lambda: 0)


@pytest.fixture()
def mock_argv(mocker):
    """Mock `snapcraft remote-build` cli."""
    return mocker.patch.object(sys, "argv", ["snapcraft", "remote-build"])


@pytest.fixture()
def mock_confirm(mocker):
    return mocker.patch(
        "snapcraft.commands.remote.confirm_with_user", return_value=True
    )


@pytest.fixture()
def mock_remote_build_run(mocker):
    _mock_remote_build_run = mocker.patch(
        "snapcraft.commands.remote.RemoteBuildCommand._run"
    )
    return _mock_remote_build_run


@pytest.fixture()
def mock_remote_builder(mocker):
    _mock_remote_builder = mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService"
    )
    _mock_remote_builder._is_setup = True
    return _mock_remote_builder


@pytest.fixture()
def mock_remote_builder_start_builds(mocker):
    _mock_start_builds = mocker.patch(
        "snapcraft.services.remotebuild.RemoteBuild.start_builds"
    )
    return _mock_start_builds


@pytest.fixture()
def mock_remote_builder_fake_build_process(mocker):
    mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.monitor_builds"
    )

    mocker.patch("snapcraft.services.remotebuild.RemoteBuild.fetch_logs")

    mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.fetch_artifacts",
        return_value=[Path("test.snap")],
    )

    mocker.patch("craft_application.services.remotebuild.RemoteBuildService.cleanup")


@pytest.fixture()
def mock_run_remote_build(mocker):
    return mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.start_builds"
    )


@pytest.fixture()
def mock_run_legacy(mocker):
    return mocker.patch("snapcraft_legacy.cli.legacy.legacy_run")


#############
# CLI tests #
#############


@pytest.mark.parametrize("base", const.CURRENT_BASES)
@pytest.mark.parametrize("project_name", ["something", "something-else"])
def test_set_project(
    mocker, snapcraft_yaml, base, mock_confirm, fake_services, project_name
):
    """Check that a project name gets set if the user provides a project."""
    mocker.patch("sys.argv", ["snapcraft", "remote-build", "--project", project_name])

    snapcraft_yaml_dict = {"base": base, "build-base": "devel", "grade": "devel"}
    snapcraft_yaml(**snapcraft_yaml_dict)
    app = application.create_app()
    remote_build = app.services.remote_build
    remote_build.is_project_private = lambda: False

    app.run()

    assert remote_build._project_name == project_name
    mock_confirm.assert_called_once()


@pytest.mark.parametrize("base", const.CURRENT_BASES)
@pytest.mark.parametrize("project_name", ["something", "something_else"])
def test_no_confirmation_for_private_project(
    mocker, snapcraft_yaml, base, mock_confirm, fake_services, project_name
):
    """If a user uploads to a private project, we don't need a confirmation prompt."""
    mocker.patch("sys.argv", ["snapcraft", "remote-build", "--project", project_name])

    snapcraft_yaml_dict = {"base": base, "build-base": "devel", "grade": "devel"}
    snapcraft_yaml(**snapcraft_yaml_dict)
    app = application.create_app()
    remote_build = app.services.remote_build
    remote_build.is_project_private = lambda: True

    app.run()

    assert remote_build._project_name == project_name
    mock_confirm.assert_not_called()


@pytest.mark.parametrize("base", const.CURRENT_BASES)
@pytest.mark.usefixtures("mock_argv")
def test_command_user_confirms_upload(
    snapcraft_yaml, base, mock_confirm, fake_services
):
    """Check if the confirmation prompt is shown."""
    snapcraft_yaml_dict = {"base": base, "build-base": "devel", "grade": "devel"}
    snapcraft_yaml(**snapcraft_yaml_dict)
    fake_services.remote_build.is_project_private = lambda: False
    app = application.create_app()

    app.run()

    mock_confirm.assert_called_once_with(
        "All data sent to remote builders will be publicly available. "
        "Are you sure you want to continue?",
        default=False,
    )


@pytest.mark.parametrize("base", const.CURRENT_BASES)
@pytest.mark.usefixtures("mock_argv", "fake_services")
def test_command_user_denies_upload(
    capsys,
    snapcraft_yaml,
    base,
    mock_confirm,
):
    """Raise an error if the user denies the upload prompt."""
    snapcraft_yaml_dict = {"base": base, "build-base": "devel", "grade": "devel"}
    snapcraft_yaml(**snapcraft_yaml_dict)
    mock_confirm.return_value = False
    app = application.create_app()
    app.run()

    _, err = capsys.readouterr()

    assert (
        "Remote build needs explicit acknowledgement that data sent to build "
        "servers is public."
    ) in err


@pytest.mark.parametrize("base", const.CURRENT_BASES)
@pytest.mark.usefixtures("mock_argv", "fake_services")
def test_command_accept_upload(
    mocker, snapcraft_yaml, base, mock_confirm, mock_run_remote_build
):
    """Do not prompt user if `--launchpad-accept-public-upload` is provided."""
    mocker.patch.object(
        sys, "argv", ["snapcraft", "remote-build", "--launchpad-accept-public-upload"]
    )
    snapcraft_yaml_dict = {"base": base, "build-base": "devel", "grade": "devel"}
    snapcraft_yaml(**snapcraft_yaml_dict)
    app = application.create_app()
    app.run()

    mock_confirm.assert_not_called()
    mock_run_remote_build.assert_called_once()


@pytest.mark.parametrize("base", const.CURRENT_BASES)
@pytest.mark.usefixtures("mock_argv", "mock_confirm", "fake_services", "fake_sudo")
def test_remote_build_sudo_warns(emitter, snapcraft_yaml, base, mock_run_remote_build):
    "Check if a warning is shown when snapcraft is run with sudo."
    snapcraft_yaml_dict = {"base": base, "build-base": "devel", "grade": "devel"}
    snapcraft_yaml(**snapcraft_yaml_dict)
    app = application.create_app()
    app.run()

    emitter.assert_progress(
        "Running with 'sudo' may cause permission errors and is discouraged.",
        permanent=True,
    )
    mock_run_remote_build.assert_called_once()


@pytest.mark.usefixtures("mock_argv", "mock_confirm", "fake_services")
def test_cannot_load_snapcraft_yaml(capsys, mocker):
    """Raise an error if the snapcraft.yaml does not exist."""
    app = application.create_app()

    with pytest.raises(ClassicFallback):
        app.run()


@pytest.mark.parametrize("base", const.CURRENT_BASES)
@pytest.mark.usefixtures("mock_argv", "mock_confirm", "fake_services")
def test_launchpad_timeout_default(mocker, snapcraft_yaml, base, fake_services):
    """Check if no timeout is set by default."""
    snapcraft_yaml_dict = {"base": base, "build-base": "devel", "grade": "devel"}
    snapcraft_yaml(**snapcraft_yaml_dict)
    mock_start_builds = mocker.patch(
        "snapcraft.services.remotebuild.RemoteBuild.start_builds"
    )
    app = application.create_app()
    app.run()

    mock_start_builds.assert_called_once()
    assert app.services.remote_build._deadline is None


@pytest.mark.parametrize("base", const.CURRENT_BASES)
@pytest.mark.usefixtures("mock_argv", "mock_confirm", "fake_services")
def test_launchpad_timeout(mocker, snapcraft_yaml, base, fake_services):
    """Set the timeout for the remote builder."""
    mocker.patch.object(
        sys, "argv", ["snapcraft", "remote-build", "--launchpad-timeout", "100"]
    )
    snapcraft_yaml_dict = {"base": base, "build-base": "devel", "grade": "devel"}
    snapcraft_yaml(**snapcraft_yaml_dict)
    mock_start_builds = mocker.patch(
        "snapcraft.services.remotebuild.RemoteBuild.start_builds"
    )
    app = application.create_app()
    app.run()

    mock_start_builds.assert_called_once()
    assert app.services.remote_build._deadline is not None
    assert app.services.remote_build._deadline > time.monotonic_ns() + 90 * 10**9


#######################
# Control logic tests #
#######################


@pytest.mark.parametrize("base", const.CURRENT_BASES)
@pytest.mark.usefixtures("mock_argv", "mock_confirm", "fake_services")
def test_run_core22_and_later(snapcraft_yaml, base, mock_remote_build_run):
    """Bases that are core22 and later will use craft-application remote-build."""
    snapcraft_yaml_dict = {"base": base, "build-base": "devel", "grade": "devel"}
    snapcraft_yaml(**snapcraft_yaml_dict)
    application.main()

    mock_remote_build_run.assert_called_once()


@pytest.mark.parametrize("base", const.LEGACY_BASES)
@pytest.mark.usefixtures("mock_confirm", "mock_argv")
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
@pytest.mark.usefixtures("mock_confirm", "mock_argv")
def test_run_in_repo_newer_than_core22(
    emitter,
    mocker,
    snapcraft_yaml,
    base,
    new_dir,
    fake_services,
):
    """Bases newer than core22 run craft-application remote-build regardless of being in a repo."""
    # initialize a git repo
    GitRepo(new_dir)
    snapcraft_yaml_dict = {"base": base, "build-base": "devel", "grade": "devel"}
    snapcraft_yaml(**snapcraft_yaml_dict)

    mock_start_builds = mocker.patch(
        "snapcraft.services.remotebuild.RemoteBuild.start_builds"
    )
    application.main()

    mock_start_builds.assert_called_once()


@pytest.mark.xfail(reason="not implemented in craft-application")
@pytest.mark.parametrize("base", const.CURRENT_BASES)
@pytest.mark.usefixtures(
    "mock_confirm",
    "mock_argv",
)
def test_run_in_shallow_repo_unsupported(
    capsys,
    new_dir,
    snapcraft_yaml,
    base,
    mock_remote_builder_start_builds,
    fake_services,
):
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


def test_no_architecture_all(
    mocker, snapcraft_yaml, fake_services, mock_remote_builder_fake_build_process
):
    """Test that remote builds error out early with an architecture 'all'."""
    project = snapcraft_yaml(
        base="core22", architectures=[{"build-on": "amd64", "build-for": "all"}]
    )
    fake_services.project = models.Project.unmarshal(project)
    cmd = commands.RemoteBuildCommand(
        {"app": application.APP_METADATA, "services": fake_services}
    )

    with pytest.raises(
        craft_cli.CraftError, match="Remote build does not support architecture 'all'"
    ):
        cmd.run(
            argparse.Namespace(
                project=None,
                launchpad_accept_public_upload=True,
            )
        )


@pytest.mark.parametrize("base", const.CURRENT_BASES)
def test_no_platform_defined_no_platform_or_build_for(
    mocker,
    snapcraft_yaml,
    base,
    fake_services,
    mock_confirm,
    mock_remote_builder_fake_build_process,
):
    """Test that the remote-build command uses the host architecture if no platform is defined."""
    snapcraft_yaml_dict = {
        "base": base,
        "build-base": "devel",
        "grade": "devel",
    }
    snapcraft_yaml(**snapcraft_yaml_dict)
    mocker.patch.object(
        sys,
        "argv",
        ["snapcraft", "remote-build"],
    )
    mock_start_builds = mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.start_builds"
    )
    app = application.create_app()
    app.run()

    mock_start_builds.assert_called_once_with(
        ANY, architectures=[get_host_architecture()]
    )


@pytest.mark.parametrize("base", const.CURRENT_BASES - {"core22"})
def test_platform_defined_no_platform_or_build_for(
    mocker,
    snapcraft_yaml,
    base,
    fake_services,
    mock_confirm,
    mock_remote_builder_fake_build_process,
):
    """remote-build command uses all platforms if no platform or build-for is given."""
    snapcraft_yaml_dict = {
        "base": base,
        "build-base": "devel",
        "grade": "devel",
        "platforms": {
            "rpi4": {"build-on": "arm64", "build-for": "arm64"},
            "x86-64": {"build-on": "amd64", "build-for": "amd64"},
        },
    }
    snapcraft_yaml(**snapcraft_yaml_dict)
    mocker.patch.object(
        sys,
        "argv",
        ["snapcraft", "remote-build"],
    )
    mock_start_builds = mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.start_builds"
    )
    app = application.create_app()
    app.run()

    mock_start_builds.assert_called_once()
    assert const.SnapArch.arm64 in mock_start_builds.call_args[1]["architectures"]
    assert const.SnapArch.amd64 in mock_start_builds.call_args[1]["architectures"]


@pytest.mark.parametrize("base", const.CURRENT_BASES - {"core22"})
@pytest.mark.parametrize(
    ["platform", "arch"],
    [("rpi4", const.SnapArch.arm64), ("x86-64", const.SnapArch.amd64)],
)
def test_platform(
    mocker,
    snapcraft_yaml,
    base,
    fake_services,
    mock_confirm,
    mock_remote_builder_fake_build_process,
    platform,
    arch,
):
    """Test that the remote-build command uses the platform if --platform is given."""
    snapcraft_yaml_dict = {
        "base": base,
        "build-base": "devel",
        "grade": "devel",
        "platforms": {
            "rpi4": {"build-on": "arm64", "build-for": "arm64"},
            "x86-64": {"build-on": "amd64", "build-for": "amd64"},
        },
    }
    snapcraft_yaml(**snapcraft_yaml_dict)
    mocker.patch.object(
        sys,
        "argv",
        ["snapcraft", "remote-build", "--platform", platform],
    )
    mock_start_builds = mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.start_builds"
    )
    app = application.create_app()
    app.run()

    mock_start_builds.assert_called_once_with(ANY, architectures=[arch])


@pytest.mark.xfail(
    reason="craft-application catches before it gets to the remote-build command"
)
@pytest.mark.parametrize("base", const.CURRENT_BASES - {"core22"})
def test_platform_error(
    emitter,
    mocker,
    snapcraft_yaml,
    base,
    fake_services,
    mock_confirm,
):
    """Test that the remote-build command errors if the given platform is not found."""
    snapcraft_yaml_dict = {
        "base": base,
        "build-base": "devel",
        "grade": "devel",
        "platforms": {
            "rpi4": {"build-on": "arm64", "build-for": "arm64"},
            "x86-64": {"build-on": "amd64", "build-for": "amd64"},
        },
    }
    snapcraft_yaml(**snapcraft_yaml_dict)
    mocker.patch.object(
        sys,
        "argv",
        ["snapcraft", "remote-build", "--platform", "nonexistent"],
    )

    app = application.create_app()
    assert app.run() == 1
    emitter.assert_progress(
        "Platform 'nonexistent' not found in the project definition.", permanent=True
    )


@pytest.mark.parametrize("base", const.CURRENT_BASES - {"core22"})
@pytest.mark.parametrize(
    ["build_for", "arch"],
    [("amd64", const.SnapArch.amd64), ("arm64", const.SnapArch.arm64)],
)
def test_build_for(
    mocker,
    snapcraft_yaml,
    base,
    fake_services,
    mock_confirm,
    mock_remote_builder_fake_build_process,
    build_for,
    arch,
):
    """Test that the remote-build command uses the build-for if --build-for is given."""
    snapcraft_yaml_dict = {
        "base": base,
        "build-base": "devel",
        "grade": "devel",
        "platforms": {
            "rpi4": {"build-on": "arm64", "build-for": "arm64"},
            "x86-64": {"build-on": "amd64", "build-for": "amd64"},
        },
    }
    snapcraft_yaml(**snapcraft_yaml_dict)
    mocker.patch.object(
        sys,
        "argv",
        ["snapcraft", "remote-build", "--build-for", build_for],
    )
    mock_start_builds = mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.start_builds"
    )
    app = application.create_app()
    app.run()

    mock_start_builds.assert_called_once_with(ANY, architectures=[arch])


@pytest.mark.parametrize("base", const.CURRENT_BASES)
@pytest.mark.parametrize(
    ["build_for", "arch"],
    [("amd64", const.SnapArch.amd64), ("arm64", const.SnapArch.arm64)],
)
def test_build_for_no_platforms(
    mocker,
    snapcraft_yaml,
    base,
    fake_services,
    mock_confirm,
    mock_remote_builder_fake_build_process,
    build_for,
    arch,
):
    """remote-build command uses the build-for if --build-for is given without platforms."""
    snapcraft_yaml_dict = {
        "base": base,
        "build-base": "devel",
        "grade": "devel",
    }
    snapcraft_yaml(**snapcraft_yaml_dict)
    mocker.patch.object(
        sys,
        "argv",
        ["snapcraft", "remote-build", "--build-for", build_for],
    )
    mock_start_builds = mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.start_builds"
    )
    app = application.create_app()
    app.run()

    mock_start_builds.assert_called_once_with(ANY, architectures=[arch])


@pytest.mark.parametrize("base", const.CURRENT_BASES - {"core22"})
def test_build_for_error(
    capsys,
    mocker,
    snapcraft_yaml,
    base,
    fake_services,
    mock_confirm,
    mock_remote_builder_fake_build_process,
):
    """Test that the remote-build command errors if the given build-for is not found."""
    snapcraft_yaml_dict = {
        "base": base,
        "build-base": "devel",
        "grade": "devel",
        "platforms": {
            "rpi4": {"build-on": "arm64", "build-for": "arm64"},
            "x86-64": {"build-on": "amd64", "build-for": "amd64"},
        },
    }
    snapcraft_yaml(**snapcraft_yaml_dict)
    mocker.patch.object(
        sys,
        "argv",
        ["snapcraft", "remote-build", "--build-for", "nonexistent"],
    )
    mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.start_builds"
    )
    app = application.create_app()
    assert app.run() == 1

    _, err = capsys.readouterr()

    assert "Architecture 'nonexistent' is not supported." in err


########################
# Remote builder tests #
########################


@pytest.mark.parametrize("base", const.CURRENT_BASES)
@pytest.mark.usefixtures("mock_confirm")
def test_monitor_build(mocker, emitter, snapcraft_yaml, base, fake_services):
    """Test the monitor_build method and the progress emitter."""
    mocker.patch.object(sys, "argv", ["snapcraft", "remote-build"])
    snapcraft_yaml_dict = {"base": base, "build-base": "devel", "grade": "devel"}
    snapcraft_yaml(**snapcraft_yaml_dict)

    mock_monitor_builds = mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.monitor_builds",
        side_effect=[
            [
                {"amd64": launchpad.models.BuildState.PENDING},
                {"amd64": launchpad.models.BuildState.BUILDING},
                {"amd64": launchpad.models.BuildState.UPLOADING},
                {"amd64": launchpad.models.BuildState.SUCCESS},
            ]
        ],
    )

    mock_start_builds = mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.start_builds"
    )

    mock_fetch_artifacts = mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.fetch_artifacts",
        return_value=[Path("test.snap")],
    )

    mock_cleanup = mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.cleanup"
    )

    app = application.create_app()
    app.services.remote_build._name = get_build_id(
        app.services.app.name, app.project.name, app.project_dir
    )
    app.services.remote_build._is_setup = True
    app.services.remote_build.request.download_files_with_progress = Mock()
    app.run()

    mock_start_builds.assert_called_once()
    mock_monitor_builds.assert_called_once()
    mock_fetch_artifacts.assert_called_once()
    mock_cleanup.assert_called_once()

    emitter.assert_progress("Stopped: amd64")
    emitter.assert_progress("Building: amd64")
    emitter.assert_progress("Uploading: amd64")
    emitter.assert_progress("Succeeded: amd64")


@pytest.mark.parametrize("base", const.CURRENT_BASES)
@pytest.mark.usefixtures("mock_confirm")
def test_monitor_build_error(mocker, emitter, snapcraft_yaml, base, fake_services):
    """Test the monitor_build cleanup when an error occurs."""
    mocker.patch.object(sys, "argv", ["snapcraft", "remote-build"])
    snapcraft_yaml_dict = {"base": base, "build-base": "devel", "grade": "devel"}
    snapcraft_yaml(**snapcraft_yaml_dict)

    mock_start_builds = mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.start_builds"
    )

    mock_monitor_builds = mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.monitor_builds",
        side_effect=Exception(),
    )

    mock_fetch_logs = mocker.patch(
        "snapcraft.services.remotebuild.RemoteBuild.fetch_logs"
    )

    mock_cleanup = mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.cleanup"
    )

    app = application.create_app()
    app.services.remote_build._name = get_build_id(
        app.services.app.name, app.project.name, app.project_dir
    )
    app.services.remote_build._is_setup = True
    app.services.remote_build.request.download_files_with_progress = Mock()

    assert app.run() == 1

    mock_start_builds.assert_called_once()
    mock_monitor_builds.assert_called_once()
    mock_fetch_logs.assert_not_called()
    mock_cleanup.assert_called_once()

    emitter.assert_progress("Cleaning up")


@pytest.mark.parametrize("base", const.CURRENT_BASES)
@pytest.mark.usefixtures("mock_confirm")
def test_monitor_build_interrupt(mocker, emitter, snapcraft_yaml, base, fake_services):
    """Test the monitor_build cleanup when a keyboard interrupt occurs."""
    mocker.patch.object(sys, "argv", ["snapcraft", "remote-build"])
    snapcraft_yaml_dict = {"base": base, "build-base": "devel", "grade": "devel"}
    snapcraft_yaml(**snapcraft_yaml_dict)

    mock_start_builds = mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.start_builds"
    )

    mock_monitor_builds = mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.monitor_builds",
        side_effect=KeyboardInterrupt(),
    )

    mock_fetch_logs = mocker.patch(
        "snapcraft.services.remotebuild.RemoteBuild.fetch_logs"
    )

    mock_cleanup = mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.cleanup"
    )

    app = application.create_app()
    app.services.remote_build._name = get_build_id(
        app.services.app.name, app.project.name, app.project_dir
    )
    app.services.remote_build._is_setup = True
    app.services.remote_build.request.download_files_with_progress = Mock()

    assert app.run() == 0

    mock_start_builds.assert_called_once()
    mock_monitor_builds.assert_called_once()
    mock_fetch_logs.assert_not_called()
    mock_cleanup.assert_called_once()

    emitter.assert_progress("Cancelling builds.")
    emitter.assert_progress("Cleaning up")


@pytest.mark.parametrize("base", const.CURRENT_BASES)
@pytest.mark.usefixtures("mock_confirm")
def test_monitor_build_timeout(mocker, emitter, snapcraft_yaml, base, fake_services):
    """Test the monitor_build timeout."""
    mocker.patch.object(sys, "argv", ["snapcraft", "remote-build"])
    snapcraft_yaml_dict = {"base": base, "build-base": "devel", "grade": "devel"}
    snapcraft_yaml(**snapcraft_yaml_dict)

    mock_start_builds = mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.start_builds"
    )

    mock_monitor_builds = mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.monitor_builds",
        side_effect=TimeoutError(),
    )

    mock_fetch_logs = mocker.patch(
        "snapcraft.services.remotebuild.RemoteBuild.fetch_logs"
    )

    mock_cleanup = mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.cleanup"
    )

    app = application.create_app()
    app.services.remote_build._name = get_build_id(
        app.services.app.name, app.project.name, app.project_dir
    )

    assert app.run() == 75

    mock_start_builds.assert_called_once()
    mock_monitor_builds.assert_called_once()
    mock_fetch_logs.assert_not_called()
    mock_cleanup.assert_not_called()

    emitter.assert_message(
        "Timed out waiting for build.\nTo resume, run "
        f"'{app.services.app.name} remote-build --recover "
        f"--build-id={app.services.remote_build._name}'"
    )


@pytest.mark.parametrize("base", const.CURRENT_BASES)
@pytest.mark.usefixtures("mock_confirm")
def test_monitor_build_failure(mocker, emitter, snapcraft_yaml, base, fake_services):
    """Test the monitor_build cleanup when a build fails."""
    mocker.patch.object(sys, "argv", ["snapcraft", "remote-build"])
    snapcraft_yaml_dict = {"base": base, "build-base": "devel", "grade": "devel"}
    snapcraft_yaml(**snapcraft_yaml_dict)

    mock_start_builds = mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.start_builds"
    )

    mock_monitor_builds = mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.monitor_builds",
        return_value=[
            {"amd64": BuildState.PENDING},
            {"amd64": BuildState.FAILED},
        ],
    )

    mock_fetch_logs = mocker.patch(
        "snapcraft.services.remotebuild.RemoteBuild.fetch_logs"
    )

    mock_cleanup = mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.cleanup"
    )

    app = application.create_app()
    app.services.remote_build._name = get_build_id(
        app.services.app.name, app.project.name, app.project_dir
    )
    app.services.remote_build._is_setup = True
    app.services.remote_build.request.download_files_with_progress = Mock()

    assert app.run() == 1

    mock_start_builds.assert_called_once()
    mock_monitor_builds.assert_called_once()
    mock_fetch_logs.assert_called_once()
    mock_cleanup.assert_called_once()

    emitter.assert_progress("Cleaning up")


@pytest.mark.parametrize("base", const.CURRENT_BASES)
@pytest.mark.usefixtures("mock_confirm")
def test_monitor_build_success_no_artifacts(
    mocker, emitter, snapcraft_yaml, base, fake_services
):
    """Test the cleanup when a build succeeds but doesn't generate artifacts."""
    mocker.patch.object(sys, "argv", ["snapcraft", "remote-build"])
    snapcraft_yaml_dict = {"base": base, "build-base": "devel", "grade": "devel"}
    snapcraft_yaml(**snapcraft_yaml_dict)

    mock_start_builds = mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.start_builds"
    )

    mock_monitor_builds = mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.monitor_builds",
        return_value=[
            {"amd64": BuildState.PENDING},
            {"amd64": BuildState.SUCCESS},
        ],
    )

    mock_fetch_logs = mocker.patch(
        "snapcraft.services.remotebuild.RemoteBuild.fetch_logs",
    )
    mock_fetch_artifacts = mocker.patch(
        "snapcraft.services.remotebuild.RemoteBuild.fetch_artifacts", return_value=[]
    )

    mock_cleanup = mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.cleanup"
    )

    app = application.create_app()
    app.services.remote_build._name = get_build_id(
        app.services.app.name, app.project.name, app.project_dir
    )
    app.services.remote_build._is_setup = True
    app.services.remote_build.request.download_files_with_progress = Mock()

    assert app.run() == 1

    mock_start_builds.assert_called_once()
    mock_monitor_builds.assert_called_once()
    mock_fetch_logs.assert_called_once()
    mock_fetch_artifacts.assert_called_once()
    mock_cleanup.assert_called_once()

    emitter.assert_progress(
        "No build artifacts downloaded from Launchpad.", permanent=True
    )
    emitter.assert_progress("Cleaning up")


@pytest.mark.parametrize("base", const.CURRENT_BASES)
@pytest.mark.usefixtures("mock_confirm")
def test_monitor_build_success_no_logs(
    mocker, emitter, snapcraft_yaml, base, fake_services
):
    """Test the cleanup when a build succeeds but doesn't generate logs."""
    mocker.patch.object(sys, "argv", ["snapcraft", "remote-build"])
    snapcraft_yaml_dict = {"base": base, "build-base": "devel", "grade": "devel"}
    snapcraft_yaml(**snapcraft_yaml_dict)

    mock_start_builds = mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.start_builds"
    )

    mock_monitor_builds = mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.monitor_builds",
        return_value=[
            {"amd64": BuildState.PENDING},
            {"amd64": BuildState.SUCCESS},
        ],
    )

    mock_fetch_logs = mocker.patch(
        "snapcraft.services.remotebuild.RemoteBuild.fetch_logs", return_value=[]
    )

    mock_cleanup = mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.cleanup"
    )

    app = application.create_app()
    app.services.remote_build._name = get_build_id(
        app.services.app.name, app.project.name, app.project_dir
    )
    app.services.remote_build._is_setup = True
    app.services.remote_build.request.download_files_with_progress = Mock()

    assert app.run() == 1

    mock_start_builds.assert_called_once()
    mock_monitor_builds.assert_called_once()
    mock_fetch_logs.assert_called_once()
    mock_cleanup.assert_called_once()

    emitter.assert_progress("No log files downloaded from Launchpad.", permanent=True)
    emitter.assert_progress("Cleaning up")


@pytest.mark.parametrize("base", const.CURRENT_BASES)
@pytest.mark.usefixtures("mock_confirm")
def test_recover_build(mocker, snapcraft_yaml, base, fake_services):
    """Recover a build when `--recover` is provided."""
    mocker.patch.object(sys, "argv", ["snapcraft", "remote-build", "--recover"])
    snapcraft_yaml_dict = {"base": base, "build-base": "devel", "grade": "devel"}
    snapcraft_yaml(**snapcraft_yaml_dict)

    mock_resume_builds = mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.resume_builds"
    )

    mock_monitor_builds = mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.monitor_builds"
    )

    mock_fetch_logs = mocker.patch(
        "snapcraft.services.remotebuild.RemoteBuild.fetch_logs"
    )

    mock_fetch_artifacts = mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.fetch_artifacts",
        return_value=[Path("test.snap")],
    )

    mock_cleanup = mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.cleanup"
    )

    app = application.create_app()
    app.run()

    mock_resume_builds.assert_called_once()
    mock_monitor_builds.assert_called_once()
    mock_fetch_logs.assert_called_once()
    mock_fetch_artifacts.assert_called_once()
    mock_cleanup.assert_called_once()


@pytest.mark.parametrize("base", const.CURRENT_BASES)
@pytest.mark.usefixtures("mock_confirm", "mock_argv")
def test_remote_build(mocker, snapcraft_yaml, base, fake_services):
    """Test the remote-build command."""
    snapcraft_yaml_dict = {"base": base, "build-base": "devel", "grade": "devel"}
    snapcraft_yaml(**snapcraft_yaml_dict)

    mock_start_builds = mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.start_builds"
    )

    mock_monitor_builds = mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.monitor_builds"
    )

    mock_fetch_logs = mocker.patch(
        "snapcraft.services.remotebuild.RemoteBuild.fetch_logs"
    )

    mock_fetch_artifacts = mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.fetch_artifacts",
        return_value=[Path("test.snap")],
    )

    mock_cleanup = mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.cleanup"
    )

    app = application.create_app()
    app.run()

    mock_start_builds.assert_called_once()
    mock_monitor_builds.assert_called_once()
    mock_fetch_logs.assert_called_once()
    mock_fetch_artifacts.assert_called_once()
    mock_cleanup.assert_called_once()


@pytest.mark.parametrize("base", const.CURRENT_BASES)
@pytest.mark.usefixtures("mock_confirm", "mock_argv")
def test_remote_build_error(emitter, mocker, snapcraft_yaml, base, fake_services):
    """Test the remote-build command when an error occurs."""
    snapcraft_yaml_dict = {"base": base, "build-base": "devel", "grade": "devel"}
    snapcraft_yaml(**snapcraft_yaml_dict)

    mock_start_builds = mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.start_builds",
        side_effect=RemoteBuildError("test error"),
    )

    mock_monitor_builds = mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.monitor_builds"
    )

    mock_cleanup = mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.cleanup"
    )

    app = application.create_app()
    assert app.run() == 1

    mock_start_builds.assert_called_once()
    mock_monitor_builds.assert_not_called()
    mock_cleanup.assert_called_once()

    emitter.assert_progress("Starting build failed.", permanent=True)
    emitter.assert_progress("Cleaning up")
