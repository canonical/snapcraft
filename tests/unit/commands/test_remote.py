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
import time
from pathlib import Path
from unittest.mock import ANY, Mock, call

import pytest
from craft_application import launchpad
from craft_application.errors import RemoteBuildError
from craft_application.remote.utils import get_build_id
from yaml import safe_dump

from snapcraft import application, cli
from snapcraft.const import SnapArch
from snapcraft.errors import ClassicFallback
from snapcraft.parts.yaml_utils import CURRENT_BASES, ESM_BASES, LEGACY_BASES
from snapcraft.remote import GitRepo
from snapcraft.utils import get_host_architecture

# remote-build control logic may check if the working dir is a git repo,
# so execute all tests inside a test directory
pytestmark = pytest.mark.usefixtures("new_dir")


@pytest.fixture()
def create_snapcraft_yaml(request, snapcraft_yaml):
    """Create a snapcraft.yaml file with a particular base."""
    snapcraft_yaml(base=request.param)


@pytest.fixture()
def use_core22_remote_build(monkeypatch):
    """Fixture to force using the core22 remote-build code."""
    monkeypatch.setenv("SNAPCRAFT_REMOTE_BUILD_STRATEGY", "disable-fallback")


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
def mock_core22_confirm(mocker):
    return mocker.patch(
        "snapcraft.commands.core22.remote.confirm_with_user", return_value=True
    )


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
def mock_core22_remote_builder(mocker):
    _mock_remote_builder = mocker.patch(
        "snapcraft.commands.core22.remote.RemoteBuilder"
    )
    _mock_remote_builder.return_value.has_outstanding_build.return_value = False
    return _mock_remote_builder


@pytest.fixture()
def mock_run_core22_or_fallback_remote_build(mocker):
    return mocker.patch(
        "snapcraft.commands.core22.remote.RemoteBuildCommand._run_new_or_fallback_remote_build"
    )


@pytest.fixture()
def mock_run_remote_build(mocker):
    return mocker.patch(
        "craft_application.services.remotebuild.RemoteBuildService.start_builds"
    )


@pytest.fixture()
def mock_run_core22_remote_build(mocker):
    return mocker.patch(
        "snapcraft.commands.core22.remote.RemoteBuildCommand._run_new_remote_build"
    )


@pytest.fixture()
def mock_run_legacy(mocker):
    return mocker.patch("snapcraft.commands.core22.remote.run_legacy")


#############
# CLI tests #
#############


@pytest.mark.parametrize("base", CURRENT_BASES - {"core22"})
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


@pytest.mark.parametrize("base", CURRENT_BASES - {"core22"})
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


@pytest.mark.parametrize("base", CURRENT_BASES - {"core22"})
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


@pytest.mark.parametrize(
    "create_snapcraft_yaml", CURRENT_BASES | LEGACY_BASES, indirect=True
)
@pytest.mark.usefixtures("create_snapcraft_yaml", "mock_argv")
def test_command_user_confirms_upload_core22(
    mock_core22_confirm, mock_run_core22_or_fallback_remote_build
):
    """Run remote-build if the user confirms the upload prompt."""
    cli.run()

    mock_core22_confirm.assert_called_once_with(
        "All data sent to remote builders will be publicly available. "
        "Are you sure you want to continue?"
    )
    mock_run_core22_or_fallback_remote_build.assert_called_once()


@pytest.mark.parametrize("base", CURRENT_BASES - {"core22"})
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


@pytest.mark.parametrize(
    "create_snapcraft_yaml", CURRENT_BASES | LEGACY_BASES, indirect=True
)
@pytest.mark.usefixtures("create_snapcraft_yaml", "mock_argv")
def test_command_user_denies_upload_core22(
    capsys, mock_core22_confirm, mock_run_core22_or_fallback_remote_build
):
    """Raise an error if the user denies the upload prompt."""
    mock_core22_confirm.return_value = False

    cli.run()

    _, err = capsys.readouterr()
    assert (
        "Cannot upload data to build servers.\n"
        "Remote build needs explicit acknowledgement "
        "that data sent to build servers is public."
    ) in err


@pytest.mark.parametrize("base", CURRENT_BASES - {"core22"})
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


@pytest.mark.parametrize(
    "create_snapcraft_yaml", CURRENT_BASES | LEGACY_BASES, indirect=True
)
@pytest.mark.usefixtures("create_snapcraft_yaml")
def test_command_accept_upload_core22(
    mock_core22_confirm, mock_run_core22_or_fallback_remote_build, mocker
):
    """Do not prompt user if `--launchpad-accept-public-upload` is provided."""
    mocker.patch.object(
        sys, "argv", ["snapcraft", "remote-build", "--launchpad-accept-public-upload"]
    )

    cli.run()

    mock_core22_confirm.assert_not_called()
    mock_run_core22_or_fallback_remote_build.assert_called_once()


@pytest.mark.parametrize(
    "create_snapcraft_yaml", CURRENT_BASES | LEGACY_BASES, indirect=True
)
@pytest.mark.usefixtures(
    "create_snapcraft_yaml", "mock_core22_confirm", "use_core22_remote_build"
)
def test_command_new_build_arguments_mutually_exclusive_core22(capsys, mocker):
    """`--build-for` and `--build-on` are mutually exclusive in the new remote-build."""
    mocker.patch.object(
        sys,
        "argv",
        ["snapcraft", "remote-build", "--build-on", "amd64", "--build-for", "arm64"],
    )

    cli.run()

    _, err = capsys.readouterr()
    assert "Error: argument --build-for: not allowed with argument --build-on" in err


@pytest.mark.parametrize(
    "create_snapcraft_yaml", LEGACY_BASES | {"core22"}, indirect=True
)
@pytest.mark.usefixtures("create_snapcraft_yaml", "mock_core22_confirm")
def test_command_legacy_build_arguments_not_mutually_exclusive_core22(
    mocker, mock_run_legacy
):
    """`--build-for` and `--build-on` are not mutually exclusive for legacy."""
    mocker.patch.object(
        sys,
        "argv",
        ["snapcraft", "remote-build", "--build-on", "amd64", "--build-for", "arm64"],
    )

    cli.run()

    mock_run_legacy.assert_called_once()


@pytest.mark.parametrize(
    "create_snapcraft_yaml", CURRENT_BASES | LEGACY_BASES, indirect=True
)
@pytest.mark.usefixtures("create_snapcraft_yaml", "mock_core22_confirm")
def test_command_build_on_warning_core22(
    emitter, mocker, mock_run_core22_or_fallback_remote_build
):
    """Warn when `--build-on` is passed."""
    mocker.patch.object(
        sys, "argv", ["snapcraft", "remote-build", "--build-on", "arch"]
    )

    cli.run()

    emitter.assert_message("Use --build-for instead of --build-on")
    mock_run_core22_or_fallback_remote_build.assert_called_once()


@pytest.mark.parametrize("base", CURRENT_BASES - {"core22"})
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


@pytest.mark.parametrize(
    "create_snapcraft_yaml", CURRENT_BASES | LEGACY_BASES, indirect=True
)
@pytest.mark.usefixtures(
    "create_snapcraft_yaml", "mock_core22_confirm", "fake_sudo", "mock_argv"
)
def test_remote_build_sudo_warns_core22(
    emitter, mock_run_core22_or_fallback_remote_build
):
    """Warn when snapcraft is run with sudo."""
    cli.run()

    emitter.assert_message(
        "Running with 'sudo' may cause permission errors and is discouraged."
    )
    mock_run_core22_or_fallback_remote_build.assert_called_once()


@pytest.mark.usefixtures("mock_argv", "mock_confirm", "fake_services")
def test_cannot_load_snapcraft_yaml(capsys, mocker):
    """Raise an error if the snapcraft.yaml does not exist."""
    app = application.create_app()

    with pytest.raises(ClassicFallback):
        app.run()


@pytest.mark.usefixtures("mock_argv", "mock_core22_confirm")
def test_cannot_load_snapcraft_yaml_core22(capsys):
    """Raise an error if the snapcraft.yaml does not exist."""
    cli.run()

    _, err = capsys.readouterr()
    assert (
        "Could not find snap/snapcraft.yaml. "
        "Are you sure you are in the right directory?" in err
    )


@pytest.mark.parametrize("base", CURRENT_BASES - {"core22"})
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


@pytest.mark.parametrize(
    "create_snapcraft_yaml", CURRENT_BASES | LEGACY_BASES, indirect=True
)
@pytest.mark.usefixtures(
    "create_snapcraft_yaml",
    "mock_core22_confirm",
    "use_core22_remote_build",
    "mock_argv",
)
def test_launchpad_timeout_default_core22(mock_core22_remote_builder):
    """Use the default timeout `0` when `--launchpad-timeout` is not provided."""
    cli.run()

    mock_core22_remote_builder.assert_called_with(
        app_name="snapcraft",
        build_id=None,
        project_name="mytest",
        architectures=ANY,
        project_dir=Path(),
        timeout=0,
    )


@pytest.mark.parametrize("base", CURRENT_BASES - {"core22"})
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


@pytest.mark.parametrize(
    "create_snapcraft_yaml", CURRENT_BASES | LEGACY_BASES, indirect=True
)
@pytest.mark.usefixtures(
    "create_snapcraft_yaml", "mock_core22_confirm", "use_core22_remote_build"
)
def test_launchpad_timeout_core22(mocker, mock_core22_remote_builder):
    """Pass the `--launchpad-timeout` to the remote builder."""
    mocker.patch.object(
        sys, "argv", ["snapcraft", "remote-build", "--launchpad-timeout", "100"]
    )

    cli.run()

    mock_core22_remote_builder.assert_called_with(
        app_name="snapcraft",
        build_id=None,
        project_name="mytest",
        architectures=ANY,
        project_dir=Path(),
        timeout=100,
    )


#########################################################
# Snapcraft project base tests (no effect since core24) #
#########################################################


@pytest.mark.usefixtures("mock_argv", "mock_core22_confirm")
@pytest.mark.parametrize("base", CURRENT_BASES | LEGACY_BASES)
def test_get_effective_base_core22(
    base, snapcraft_yaml, mock_run_core22_or_fallback_remote_build
):
    """Get the base from a snapcraft.yaml file."""
    snapcraft_yaml(base=base)

    cli.run()

    mock_run_core22_or_fallback_remote_build.assert_called_once_with(base)


@pytest.mark.usefixtures("mock_argv", "mock_core22_confirm")
@pytest.mark.parametrize(
    ("base", "build_base"),
    [
        ("core24", "devel"),
        ("core18", "core22"),
        ("bare", "core22"),
    ],
)
def test_get_effective_base_with_build_base_core22(
    base, build_base, snapcraft_yaml, mock_run_core22_or_fallback_remote_build
):
    """The effective base should be the build-base, when provided."""
    snapcraft_yaml(**{"base": base, "build-base": build_base})

    cli.run()

    if build_base == "devel":
        mock_run_core22_or_fallback_remote_build.assert_called_once_with(base)
    else:
        mock_run_core22_or_fallback_remote_build.assert_called_once_with(build_base)


@pytest.mark.usefixtures("mock_argv", "mock_core22_confirm")
@pytest.mark.parametrize("base", CURRENT_BASES | LEGACY_BASES | ESM_BASES)
def test_get_effective_base_type_core22(
    base, snapcraft_yaml, mock_run_core22_or_fallback_remote_build
):
    """The effective base should be the name when building a base."""
    snapcraft_yaml(**{"base": base, "name": base, "type": "base"})

    cli.run()

    mock_run_core22_or_fallback_remote_build.assert_called_once_with(base)


@pytest.mark.usefixtures("mock_argv", "mock_core22_confirm")
def test_get_effective_base_unknown_core22(capsys, snapcraft_yaml):
    """Raise an error for unknown bases."""
    snapcraft_yaml(base="core10")

    cli.run()

    _, err = capsys.readouterr()
    assert "Unknown base 'core10' in 'snap/snapcraft.yaml'." in err


@pytest.mark.usefixtures("mock_argv", "mock_core22_confirm")
def test_get_effective_base_none_core22(capsys, snapcraft_yaml):
    """Raise an error if there is no base in the snapcraft.yaml."""
    snapcraft_yaml()

    cli.run()

    _, err = capsys.readouterr()
    assert "Could not determine base from 'snap/snapcraft.yaml'." in err


@pytest.mark.usefixtures("mock_argv", "mock_core22_confirm")
def test_get_effective_base_core_esm_warning_core22(
    emitter, snapcraft_yaml, mock_run_core22_or_fallback_remote_build
):
    """Warn if core, an ESM base, is used."""
    snapcraft_yaml(base="core")

    cli.run()

    mock_run_core22_or_fallback_remote_build.assert_called_once_with("core")
    emitter.assert_progress(
        "WARNING: base 'core' was last supported on Snapcraft 4 available on the "
        "'4.x' channel.",
        permanent=True,
    )


@pytest.mark.usefixtures("mock_argv", "mock_core22_confirm")
def test_get_effective_base_core18_esm_warning_core22(
    emitter, snapcraft_yaml, mock_run_core22_or_fallback_remote_build
):
    """Warn if core18, an ESM base, is used."""
    snapcraft_yaml(base="core18")

    cli.run()

    mock_run_core22_or_fallback_remote_build.assert_called_once_with("core18")
    emitter.assert_progress(
        "WARNING: base 'core18' was last supported on Snapcraft 7 available on the "
        "'7.x' channel.",
        permanent=True,
    )


#######################
# Control logic tests #
#######################


@pytest.mark.parametrize("base", CURRENT_BASES - {"core22"})
@pytest.mark.usefixtures("mock_argv", "mock_confirm", "fake_services")
def test_run_core24_and_later(mocker, snapcraft_yaml, base, fake_services):
    """Bases that are core24 and later must use craft-application remote-build."""
    snapcraft_yaml_dict = {"base": base, "build-base": "devel", "grade": "devel"}
    snapcraft_yaml(**snapcraft_yaml_dict)
    mock_start_builds = mocker.patch(
        "snapcraft.services.remotebuild.RemoteBuild.start_builds"
    )
    application.main()

    mock_start_builds.assert_called_once()


@pytest.mark.parametrize("base", {"core22"})
@pytest.mark.usefixtures("mock_core22_confirm", "mock_argv")
def test_run_core22(
    emitter,
    snapcraft_yaml,
    base,
    use_core22_remote_build,
    mock_run_core22_remote_build,
    mock_run_legacy,
    fake_services,
):
    """Bases that is core22 should use core22 remote-build."""
    snapcraft_yaml_dict = {"base": base}
    snapcraft_yaml(**snapcraft_yaml_dict)
    application.main()

    mock_run_core22_remote_build.assert_called_once()
    mock_run_legacy.assert_not_called()
    emitter.assert_debug(
        "Running new remote-build because environment variable "
        "'SNAPCRAFT_REMOTE_BUILD_STRATEGY' is 'disable-fallback'"
    )


@pytest.mark.parametrize("base", LEGACY_BASES | {"core22"})
@pytest.mark.usefixtures("mock_core22_confirm", "mock_argv")
def test_run_core22_and_older(
    emitter, snapcraft_yaml, base, mock_run_legacy, fake_services
):
    """core22 and older bases can use fallback remote-build."""
    snapcraft_yaml_dict = {"base": base}
    snapcraft_yaml(**snapcraft_yaml_dict)
    application.main()

    mock_run_legacy.assert_called_once()
    emitter.assert_debug("Running fallback remote-build")


@pytest.mark.parametrize(
    "create_snapcraft_yaml", CURRENT_BASES - {"core22"}, indirect=True
)
@pytest.mark.parametrize(
    "envvar", ["force-fallback", "disable-fallback", "badvalue", None]
)
@pytest.mark.usefixtures("create_snapcraft_yaml", "mock_core22_confirm", "mock_argv")
def test_run_envvar_core22(envvar, emitter, mock_run_core22_remote_build, monkeypatch):
    """Bases newer than core22 run new remote-build regardless of envvar."""
    if envvar:
        monkeypatch.setenv("SNAPCRAFT_REMOTE_BUILD_STRATEGY", envvar)
    else:
        monkeypatch.delenv("SNAPCRAFT_REMOTE_BUILD_STRATEGY", raising=False)

    cli.run()

    mock_run_core22_remote_build.assert_called_once()
    emitter.assert_debug("Running new remote-build because base is newer than core22")


@pytest.mark.parametrize(
    "create_snapcraft_yaml", LEGACY_BASES | {"core22"}, indirect=True
)
@pytest.mark.usefixtures("create_snapcraft_yaml", "mock_core22_confirm", "mock_argv")
def test_run_envvar_disable_fallback_core22(
    emitter, mock_run_core22_remote_build, monkeypatch
):
    """core22 and older bases run new remote-build if envvar is `disable-fallback`."""
    monkeypatch.setenv("SNAPCRAFT_REMOTE_BUILD_STRATEGY", "disable-fallback")

    cli.run()

    mock_run_core22_remote_build.assert_called_once()
    emitter.assert_debug(
        "Running new remote-build because environment variable "
        "'SNAPCRAFT_REMOTE_BUILD_STRATEGY' is 'disable-fallback'"
    )


@pytest.mark.parametrize(
    "create_snapcraft_yaml", LEGACY_BASES | {"core22"}, indirect=True
)
@pytest.mark.usefixtures("create_snapcraft_yaml", "mock_core22_confirm", "mock_argv")
def test_run_envvar_force_fallback_core22(emitter, mock_run_legacy, monkeypatch):
    """core22 and older bases run legacy remote-build if envvar is `force-fallback`."""
    monkeypatch.setenv("SNAPCRAFT_REMOTE_BUILD_STRATEGY", "force-fallback")

    cli.run()

    mock_run_legacy.assert_called_once()
    emitter.assert_debug(
        "Running fallback remote-build because environment variable "
        "'SNAPCRAFT_REMOTE_BUILD_STRATEGY' is 'force-fallback'"
    )


@pytest.mark.parametrize(
    "create_snapcraft_yaml", LEGACY_BASES | {"core22"}, indirect=True
)
@pytest.mark.usefixtures("create_snapcraft_yaml", "mock_core22_confirm", "mock_argv")
def test_run_envvar_force_fallback_unset_core22(emitter, mock_run_legacy, monkeypatch):
    """core22 and older bases run legacy remote-build if envvar is unset."""
    monkeypatch.delenv("SNAPCRAFT_REMOTE_BUILD_STRATEGY", raising=False)

    cli.run()

    mock_run_legacy.assert_called_once()
    emitter.assert_debug("Running fallback remote-build")


@pytest.mark.parametrize(
    "create_snapcraft_yaml", LEGACY_BASES | {"core22"}, indirect=True
)
@pytest.mark.usefixtures("create_snapcraft_yaml", "mock_core22_confirm", "mock_argv")
def test_run_envvar_force_fallback_empty_core22(emitter, mock_run_legacy, monkeypatch):
    """core22 and older bases run legacy remote-build if envvar is empty."""
    monkeypatch.setenv("SNAPCRAFT_REMOTE_BUILD_STRATEGY", "")

    cli.run()

    mock_run_legacy.assert_called_once()
    emitter.assert_debug("Running fallback remote-build")


@pytest.mark.parametrize(
    "create_snapcraft_yaml", LEGACY_BASES | {"core22"}, indirect=True
)
@pytest.mark.usefixtures("create_snapcraft_yaml", "mock_core22_confirm", "mock_argv")
def test_run_envvar_invalid_core22(capsys, emitter, mock_run_legacy, monkeypatch):
    """core22 and older bases raise an error if the envvar is invalid."""
    monkeypatch.setenv("SNAPCRAFT_REMOTE_BUILD_STRATEGY", "badvalue")

    cli.run()

    _, err = capsys.readouterr()
    assert (
        "Unknown value 'badvalue' in environment variable "
        "'SNAPCRAFT_REMOTE_BUILD_STRATEGY'. Valid values are 'disable-fallback' and "
        "'force-fallback'"
    ) in err


@pytest.mark.parametrize(
    "create_snapcraft_yaml", LEGACY_BASES | {"core22"}, indirect=True
)
@pytest.mark.usefixtures("create_snapcraft_yaml", "mock_core22_confirm", "mock_argv")
def test_run_in_repo_core22(emitter, mock_run_core22_remote_build, new_dir):
    """core22 and older bases run new remote-build if in a git repo."""
    # initialize a git repo
    GitRepo(new_dir)

    cli.run()

    mock_run_core22_remote_build.assert_called_once()
    emitter.assert_debug(
        "Running new remote-build because project is in a git repository"
    )


@pytest.mark.parametrize(
    "create_snapcraft_yaml", LEGACY_BASES | {"core22"}, indirect=True
)
@pytest.mark.usefixtures("create_snapcraft_yaml", "mock_core22_confirm", "mock_argv")
def test_run_not_in_repo_core22(emitter, mock_run_legacy):
    """core22 and older bases run legacy remote-build if not in a git repo."""
    cli.run()

    mock_run_legacy.assert_called_once()
    emitter.assert_debug("Running fallback remote-build")


@pytest.mark.parametrize("base", CURRENT_BASES - {"core22"})
@pytest.mark.usefixtures("mock_confirm", "mock_argv")
def test_run_in_repo_newer_than_core22(
    emitter, mocker, snapcraft_yaml, base, new_dir, fake_services
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


@pytest.mark.parametrize(
    "create_snapcraft_yaml", LEGACY_BASES | {"core22"}, indirect=True
)
@pytest.mark.usefixtures("create_snapcraft_yaml", "mock_core22_confirm", "mock_argv")
def test_run_in_shallow_repo_core22(emitter, mock_run_legacy, new_dir):
    """core22 and older bases fall back to legacy remote-build if in a shallow git repo."""
    root_path = Path(new_dir)
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
    cli.run()

    mock_run_legacy.assert_called_once()
    emitter.assert_debug("Current git repository is shallow cloned.")
    emitter.assert_debug("Running fallback remote-build")


@pytest.mark.xfail(reason="not implemented in craft-application")
@pytest.mark.parametrize("base", CURRENT_BASES - {"core22"})
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


@pytest.mark.parametrize(
    "create_snapcraft_yaml", CURRENT_BASES - {"core22"}, indirect=True
)
@pytest.mark.usefixtures(
    "create_snapcraft_yaml",
    "mock_core22_confirm",
    "mock_argv",
    "use_core22_remote_build",
)
def test_run_in_shallow_repo_unsupported_core22(capsys, new_dir):
    """devel / core24 and newer bases run new remote-build in a shallow git repo."""
    root_path = Path(new_dir)
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
    ret = cli.run()
    assert ret != 0
    _, err = capsys.readouterr()

    assert (
        "Remote builds are not supported for projects in shallowly cloned "
        "git repositories."
    ) in err


######################
# Architecture tests #
######################


@pytest.mark.parametrize("base", CURRENT_BASES | LEGACY_BASES)
@pytest.mark.parametrize(
    ["archs", "expected_archs"],
    [
        # single arch as scalar
        ([{"build-on": "arm64", "build-for": "arm64"}], ["arm64"]),
        # single arch as list
        ([{"build-on": ["arm64"], "build-for": ["arm64"]}], ["arm64"]),
        # no build-for as scalar
        ([{"build-on": "arm64"}], ["arm64"]),
        # no build-for as list
        ([{"build-on": ["arm64"]}], ["arm64"]),
        # multiple archs as scalars
        (
            [
                {"build-on": "amd64", "build-for": "amd64"},
                {"build-on": "arm64", "build-for": "arm64"},
            ],
            ["amd64", "arm64"],
        ),
        # multiple archs as lists
        (
            [
                {"build-on": ["amd64"], "build-for": ["amd64"]},
                {"build-on": ["arm64"], "build-for": ["arm64"]},
            ],
            ["amd64", "arm64"],
        ),
        # multiple build-ons
        (
            [
                {"build-on": ["amd64", "arm64"], "build-for": "amd64"},
                {"build-on": ["armhf", "powerpc"], "build-for": "arm64"},
            ],
            ["amd64", "arm64", "armhf", "powerpc"],
        ),
    ],
)
@pytest.mark.usefixtures("mock_argv", "mock_core22_confirm", "use_core22_remote_build")
def test_determine_architectures_from_snapcraft_yaml_core22(
    archs, expected_archs, base, snapcraft_yaml, mock_core22_remote_builder
):
    """Parse `build-on` architectures from a snapcraft.yaml file."""
    snapcraft_yaml(base=base, architectures=archs)

    cli.run()

    mock_core22_remote_builder.assert_called_with(
        app_name="snapcraft",
        build_id=None,
        project_name="mytest",
        architectures=expected_archs,
        project_dir=Path(),
        timeout=0,
    )


@pytest.mark.parametrize(
    "create_snapcraft_yaml", CURRENT_BASES | LEGACY_BASES, indirect=True
)
@pytest.mark.usefixtures(
    "create_snapcraft_yaml",
    "mock_argv",
    "mock_core22_confirm",
    "use_core22_remote_build",
)
def test_determine_architectures_host_arch_core22(mocker, mock_core22_remote_builder):
    """Use host architecture if not defined in the snapcraft.yaml."""
    mocker.patch(
        "snapcraft.commands.core22.remote.get_host_architecture", return_value="arm64"
    )

    cli.run()

    mock_core22_remote_builder.assert_called_with(
        app_name="snapcraft",
        build_id=None,
        project_name="mytest",
        architectures=["arm64"],
        project_dir=Path(),
        timeout=0,
    )


@pytest.mark.parametrize("build_flag", ["--build-for", "--build-on"])
@pytest.mark.parametrize(
    ("archs", "expected_archs"),
    [
        ("amd64", ["amd64"]),
        ("amd64,arm64", ["amd64", "arm64"]),
        ("amd64,amd64,arm64 ", ["amd64", "amd64", "arm64"]),
    ],
)
@pytest.mark.parametrize(
    "create_snapcraft_yaml", CURRENT_BASES | LEGACY_BASES, indirect=True
)
@pytest.mark.usefixtures(
    "create_snapcraft_yaml", "mock_core22_confirm", "use_core22_remote_build"
)
def test_determine_architectures_provided_by_user_duplicate_arguments_core22(
    build_flag, archs, expected_archs, mocker, mock_core22_remote_builder
):
    """Argparse should only consider the last argument provided for build flags."""
    mocker.patch.object(
        sys,
        "argv",
        # `--build-{for|on} armhf` should get silently ignored by argparse
        ["snapcraft", "remote-build", build_flag, "armhf", build_flag, archs],
    )

    cli.run()

    mock_core22_remote_builder.assert_called_with(
        app_name="snapcraft",
        build_id=None,
        project_name="mytest",
        architectures=expected_archs,
        project_dir=Path(),
        timeout=0,
    )


@pytest.mark.parametrize("build_flag", ["--build-for", "--build-on"])
@pytest.mark.parametrize(
    ("archs", "expected_archs"),
    [
        ("amd64", ["amd64"]),
        ("amd64,arm64", ["amd64", "arm64"]),
        ("amd64, arm64", ["amd64", "arm64"]),
        ("amd64,arm64 ", ["amd64", "arm64"]),
        ("amd64,arm64,armhf", ["amd64", "arm64", "armhf"]),
        (" amd64 , arm64 , armhf ", ["amd64", "arm64", "armhf"]),
        # launchpad will accept and ignore duplicates
        (" amd64 , arm64 , arm64 ", ["amd64", "arm64", "arm64"]),
    ],
)
@pytest.mark.parametrize(
    "create_snapcraft_yaml", CURRENT_BASES | LEGACY_BASES, indirect=True
)
@pytest.mark.usefixtures(
    "create_snapcraft_yaml", "mock_core22_confirm", "use_core22_remote_build"
)
def test_determine_architectures_provided_by_user_core22(
    build_flag, archs, expected_archs, mocker, mock_core22_remote_builder
):
    """Use architectures provided by the user."""
    mocker.patch.object(sys, "argv", ["snapcraft", "remote-build", build_flag, archs])

    cli.run()

    mock_core22_remote_builder.assert_called_with(
        app_name="snapcraft",
        build_id=None,
        project_name="mytest",
        architectures=expected_archs,
        project_dir=Path(),
        timeout=0,
    )


@pytest.mark.parametrize("base", CURRENT_BASES | LEGACY_BASES)
@pytest.mark.usefixtures("mock_core22_confirm", "use_core22_remote_build")
def test_determine_architectures_error_core22(base, capsys, snapcraft_yaml, mocker):
    """Error if `--build-for` is provided and archs are in the snapcraft.yaml."""
    mocker.patch.object(
        sys, "argv", ["snapcraft", "remote-build", "--build-for", "amd64"]
    )
    snapcraft_yaml(
        base=base, architectures=[{"build-on": "arm64", "build-for": "arm64"}]
    )

    cli.run()

    _, err = capsys.readouterr()
    assert (
        "Cannot use `--build-on` because architectures are already defined in "
        "snapcraft.yaml."
    ) in err


@pytest.mark.parametrize("base", CURRENT_BASES - {"core22"})
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


@pytest.mark.parametrize("base", CURRENT_BASES - {"core22"})
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
    assert SnapArch.arm64 in mock_start_builds.call_args[1]["architectures"]
    assert SnapArch.amd64 in mock_start_builds.call_args[1]["architectures"]


@pytest.mark.parametrize("base", CURRENT_BASES - {"core22"})
@pytest.mark.parametrize(
    ["platform", "arch"], [("rpi4", SnapArch.arm64), ("x86-64", SnapArch.amd64)]
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
@pytest.mark.parametrize("base", CURRENT_BASES - {"core22"})
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


@pytest.mark.parametrize("base", CURRENT_BASES - {"core22"})
@pytest.mark.parametrize(
    ["build_for", "arch"], [("amd64", SnapArch.amd64), ("arm64", SnapArch.arm64)]
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


@pytest.mark.parametrize("base", CURRENT_BASES - {"core22"})
@pytest.mark.parametrize(
    ["build_for", "arch"], [("amd64", SnapArch.amd64), ("arm64", SnapArch.arm64)]
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


@pytest.mark.parametrize("base", CURRENT_BASES - {"core22"})
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
    assert app.run() == 78

    _, err = capsys.readouterr()

    assert "build-for 'nonexistent' is not supported." in err


##################
# Build id tests #
##################


@pytest.mark.parametrize(
    "create_snapcraft_yaml", CURRENT_BASES | LEGACY_BASES, indirect=True
)
@pytest.mark.usefixtures(
    "create_snapcraft_yaml", "mock_core22_confirm", "use_core22_remote_build"
)
def test_build_id_provided_core22(mocker, mock_core22_remote_builder):
    """Pass the build id provided as an argument."""
    mocker.patch.object(
        sys, "argv", ["snapcraft", "remote-build", "--build-id", "test-build-id"]
    )

    cli.run()

    mock_core22_remote_builder.assert_called_with(
        app_name="snapcraft",
        build_id="test-build-id",
        project_name="mytest",
        architectures=ANY,
        project_dir=Path(),
        timeout=0,
    )


@pytest.mark.parametrize(
    "create_snapcraft_yaml", CURRENT_BASES | LEGACY_BASES, indirect=True
)
@pytest.mark.usefixtures(
    "create_snapcraft_yaml",
    "mock_core22_confirm",
    "mock_argv",
    "use_core22_remote_build",
)
def test_build_id_not_provided_core22(mock_core22_remote_builder):
    """Pass `None` for the build id if it is not provided as an argument."""

    cli.run()

    mock_core22_remote_builder.assert_called_with(
        app_name="snapcraft",
        build_id=None,
        project_name="mytest",
        architectures=ANY,
        project_dir=Path(),
        timeout=0,
    )


@pytest.mark.parametrize("base", CURRENT_BASES | LEGACY_BASES)
@pytest.mark.usefixtures("mock_core22_confirm", "mock_argv", "use_core22_remote_build")
def test_build_id_no_project_name_error_core22(base, capsys):
    """Raise an error if there is no name in the snapcraft.yaml file."""
    content = {
        "base": base,
        "version": "0.1",
        "summary": "test",
        "description": "test",
        "grade": "stable",
        "confinement": "strict",
        "parts": {
            "part1": {
                "plugin": "nil",
            }
        },
    }
    yaml_path = Path("snapcraft.yaml")
    yaml_path.write_text(safe_dump(content, indent=2), encoding="utf-8")

    cli.run()

    _, err = capsys.readouterr()
    assert "Could not get project name from 'snapcraft.yaml'." in err


########################
# Remote builder tests #
########################


@pytest.mark.parametrize(
    "create_snapcraft_yaml", CURRENT_BASES | LEGACY_BASES, indirect=True
)
@pytest.mark.usefixtures(
    "create_snapcraft_yaml", "mock_core22_confirm", "use_core22_remote_build"
)
def test_status_core22(mocker, mock_core22_remote_builder):
    """Print the status when `--status` is provided."""
    mocker.patch.object(sys, "argv", ["snapcraft", "remote-build", "--status"])

    cli.run()

    assert mock_core22_remote_builder.mock_calls[-1] == call().print_status()


@pytest.mark.parametrize("base", CURRENT_BASES - {"core22"})
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


@pytest.mark.parametrize("base", CURRENT_BASES - {"core22"})
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


@pytest.mark.parametrize("base", CURRENT_BASES - {"core22"})
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


@pytest.mark.parametrize(
    "create_snapcraft_yaml", CURRENT_BASES | LEGACY_BASES, indirect=True
)
@pytest.mark.usefixtures(
    "create_snapcraft_yaml",
    "mock_core22_confirm",
    "mock_core22_remote_builder",
    "use_core22_remote_build",
)
def test_recover_no_build_core22(emitter, mocker):
    """Warn if no build is found when `--recover` is provided."""
    mocker.patch.object(sys, "argv", ["snapcraft", "remote-build", "--recover"])

    cli.run()

    emitter.assert_progress("No build found", permanent=True)


@pytest.mark.parametrize("base", CURRENT_BASES - {"core22"})
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


@pytest.mark.parametrize(
    "create_snapcraft_yaml", CURRENT_BASES | LEGACY_BASES, indirect=True
)
@pytest.mark.usefixtures(
    "create_snapcraft_yaml", "mock_core22_confirm", "use_core22_remote_build"
)
def test_recover_build_core22(emitter, mocker, mock_core22_remote_builder):
    """Recover a build when `--recover` is provided."""
    mocker.patch.object(sys, "argv", ["snapcraft", "remote-build", "--recover"])
    mock_core22_remote_builder.return_value.has_outstanding_build.return_value = True

    cli.run()

    assert mock_core22_remote_builder.mock_calls[-3:] == [
        call().print_status(),
        call().monitor_build(),
        call().clean_build(),
    ]


@pytest.mark.parametrize(
    "create_snapcraft_yaml", CURRENT_BASES | LEGACY_BASES, indirect=True
)
@pytest.mark.usefixtures(
    "create_snapcraft_yaml",
    "mock_argv",
    "mock_core22_confirm",
    "use_core22_remote_build",
)
def test_recover_build_user_confirms_core22(
    emitter, mocker, mock_core22_remote_builder
):
    """Recover a build when a user confirms."""
    mock_core22_remote_builder.return_value.has_outstanding_build.return_value = True

    cli.run()

    assert mock_core22_remote_builder.mock_calls[-3:] == [
        call().print_status(),
        call().monitor_build(),
        call().clean_build(),
    ]


@pytest.mark.parametrize(
    "create_snapcraft_yaml", CURRENT_BASES | LEGACY_BASES, indirect=True
)
@pytest.mark.usefixtures(
    "create_snapcraft_yaml", "mock_argv", "use_core22_remote_build"
)
def test_recover_build_user_denies_core22(emitter, mocker, mock_core22_remote_builder):
    """Clean and start a new build when a user denies to recover an existing build."""
    mocker.patch(
        # confirm data upload, deny build recovery
        "snapcraft.commands.core22.remote.confirm_with_user",
        side_effect=[True, False],
    )
    mock_core22_remote_builder.return_value.has_outstanding_build.return_value = True

    cli.run()

    assert mock_core22_remote_builder.mock_calls[-3:] == [
        call().start_build(),
        call().monitor_build(),
        call().clean_build(),
    ]


@pytest.mark.parametrize("base", CURRENT_BASES - {"core22"})
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


@pytest.mark.parametrize("base", CURRENT_BASES - {"core22"})
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


@pytest.mark.parametrize(
    "create_snapcraft_yaml", CURRENT_BASES | LEGACY_BASES, indirect=True
)
@pytest.mark.usefixtures(
    "create_snapcraft_yaml",
    "mock_argv",
    "mock_core22_confirm",
    "use_core22_remote_build",
)
def test_remote_build_core22(emitter, mocker, mock_core22_remote_builder):
    """Clean and start a new build."""
    cli.run()

    assert mock_core22_remote_builder.mock_calls[-3:] == [
        call().start_build(),
        call().monitor_build(),
        call().clean_build(),
    ]
