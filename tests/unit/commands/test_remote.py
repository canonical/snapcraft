# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022-2023 Canonical Ltd.
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
from unittest.mock import ANY, call

import pytest
from yaml import safe_dump

from snapcraft import cli
from snapcraft.parts.yaml_utils import CURRENT_BASES, ESM_BASES, LEGACY_BASES
from snapcraft.remote import GitRepo

# remote-build control logic may check if the working dir is a git repo,
# so execute all tests inside a test directory
pytestmark = pytest.mark.usefixtures("new_dir")


@pytest.fixture()
def create_snapcraft_yaml(request, snapcraft_yaml):
    """Create a snapcraft.yaml file with a particular base."""
    snapcraft_yaml(base=request.param)


@pytest.fixture()
def use_new_remote_build(monkeypatch):
    """Fixture to force using the new remote-build code."""
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
def mock_remote_builder(mocker):
    _mock_remote_builder = mocker.patch("snapcraft.commands.remote.RemoteBuilder")
    _mock_remote_builder.return_value.has_outstanding_build.return_value = False
    return _mock_remote_builder


@pytest.fixture()
def mock_run_new_or_fallback_remote_build(mocker):
    return mocker.patch(
        "snapcraft.commands.remote.RemoteBuildCommand._run_new_or_fallback_remote_build"
    )


@pytest.fixture()
def mock_run_new_remote_build(mocker):
    return mocker.patch(
        "snapcraft.commands.remote.RemoteBuildCommand._run_new_remote_build"
    )


@pytest.fixture()
def mock_run_legacy(mocker):
    return mocker.patch("snapcraft.commands.remote.run_legacy")


#############
# CLI tests #
#############


@pytest.mark.parametrize(
    "create_snapcraft_yaml", CURRENT_BASES | LEGACY_BASES, indirect=True
)
@pytest.mark.usefixtures("create_snapcraft_yaml", "mock_argv")
def test_command_user_confirms_upload(
    mock_confirm, mock_run_new_or_fallback_remote_build
):
    """Run remote-build if the user confirms the upload prompt."""
    cli.run()

    mock_confirm.assert_called_once_with(
        "All data sent to remote builders will be publicly available. "
        "Are you sure you want to continue?"
    )
    mock_run_new_or_fallback_remote_build.assert_called_once()


@pytest.mark.parametrize(
    "create_snapcraft_yaml", CURRENT_BASES | LEGACY_BASES, indirect=True
)
@pytest.mark.usefixtures("create_snapcraft_yaml", "mock_argv")
def test_command_user_denies_upload(
    capsys, mock_confirm, mock_run_new_or_fallback_remote_build
):
    """Raise an error if the user denies the upload prompt."""
    mock_confirm.return_value = False

    cli.run()

    _, err = capsys.readouterr()
    assert (
        "Cannot upload data to build servers.\n"
        "Remote build needs explicit acknowledgement "
        "that data sent to build servers is public."
    ) in err


@pytest.mark.parametrize(
    "create_snapcraft_yaml", CURRENT_BASES | LEGACY_BASES, indirect=True
)
@pytest.mark.usefixtures("create_snapcraft_yaml")
def test_command_accept_upload(
    mock_confirm, mock_run_new_or_fallback_remote_build, mocker
):
    """Do not prompt user if `--launchpad-accept-public-upload` is provided."""
    mocker.patch.object(
        sys, "argv", ["snapcraft", "remote-build", "--launchpad-accept-public-upload"]
    )

    cli.run()

    mock_confirm.assert_not_called()
    mock_run_new_or_fallback_remote_build.assert_called_once()


@pytest.mark.parametrize(
    "create_snapcraft_yaml", CURRENT_BASES | LEGACY_BASES, indirect=True
)
@pytest.mark.usefixtures("create_snapcraft_yaml", "mock_confirm")
def test_command_build_on_warning(
    emitter, mocker, mock_run_new_or_fallback_remote_build
):
    """Warn when `--build-on` is passed."""
    mocker.patch.object(
        sys, "argv", ["snapcraft", "remote-build", "--build-on", "arch"]
    )

    cli.run()

    emitter.assert_message("Use --build-for instead of --build-on")
    mock_run_new_or_fallback_remote_build.assert_called_once()


@pytest.mark.parametrize(
    "create_snapcraft_yaml", CURRENT_BASES | LEGACY_BASES, indirect=True
)
@pytest.mark.usefixtures(
    "create_snapcraft_yaml", "mock_confirm", "fake_sudo", "mock_argv"
)
def test_remote_build_sudo_warns(emitter, mock_run_new_or_fallback_remote_build):
    """Warn when snapcraft is run with sudo."""
    cli.run()

    emitter.assert_message(
        "Running with 'sudo' may cause permission errors and is discouraged."
    )
    mock_run_new_or_fallback_remote_build.assert_called_once()


@pytest.mark.usefixtures("mock_argv", "mock_confirm")
def test_cannot_load_snapcraft_yaml(capsys):
    """Raise an error if the snapcraft.yaml does not exist."""
    cli.run()

    _, err = capsys.readouterr()
    assert (
        "Could not find snap/snapcraft.yaml. "
        "Are you sure you are in the right directory?" in err
    )


@pytest.mark.parametrize(
    "create_snapcraft_yaml", CURRENT_BASES | LEGACY_BASES, indirect=True
)
@pytest.mark.usefixtures(
    "create_snapcraft_yaml", "mock_confirm", "use_new_remote_build", "mock_argv"
)
def test_launchpad_timeout_default(mock_remote_builder):
    """Use the default timeout `0` when `--launchpad-timeout` is not provided."""
    cli.run()

    mock_remote_builder.assert_called_with(
        app_name="snapcraft",
        build_id=None,
        project_name="mytest",
        architectures=ANY,
        project_dir=Path(),
        timeout=0,
    )


@pytest.mark.parametrize(
    "create_snapcraft_yaml", CURRENT_BASES | LEGACY_BASES, indirect=True
)
@pytest.mark.usefixtures(
    "create_snapcraft_yaml", "mock_confirm", "use_new_remote_build"
)
def test_launchpad_timeout(mocker, mock_remote_builder):
    """Pass the `--launchpad-timeout` to the remote builder."""
    mocker.patch.object(
        sys, "argv", ["snapcraft", "remote-build", "--launchpad-timeout", "100"]
    )

    cli.run()

    mock_remote_builder.assert_called_with(
        app_name="snapcraft",
        build_id=None,
        project_name="mytest",
        architectures=ANY,
        project_dir=Path(),
        timeout=100,
    )


################################
# Snapcraft project base tests #
################################


@pytest.mark.usefixtures("mock_argv", "mock_confirm")
@pytest.mark.parametrize("base", CURRENT_BASES | LEGACY_BASES)
def test_get_effective_base(
    base, snapcraft_yaml, mock_run_new_or_fallback_remote_build
):
    """Get the base from a snapcraft.yaml file."""
    snapcraft_yaml(base=base)

    cli.run()

    mock_run_new_or_fallback_remote_build.assert_called_once_with(base)


@pytest.mark.usefixtures("mock_argv", "mock_confirm")
@pytest.mark.parametrize(
    ("base", "build_base"),
    [
        ("core24", "devel"),
        ("core18", "core22"),
        ("bare", "core22"),
    ],
)
def test_get_effective_base_with_build_base(
    base, build_base, snapcraft_yaml, mock_run_new_or_fallback_remote_build
):
    """The effective base should be the build-base, when provided."""
    snapcraft_yaml(**{"base": base, "build-base": build_base})

    cli.run()

    mock_run_new_or_fallback_remote_build.assert_called_once_with(build_base)


@pytest.mark.usefixtures("mock_argv", "mock_confirm")
@pytest.mark.parametrize("base", CURRENT_BASES | LEGACY_BASES | ESM_BASES)
def test_get_effective_base_type(
    base, snapcraft_yaml, mock_run_new_or_fallback_remote_build
):
    """The effective base should be the name when building a base."""
    snapcraft_yaml(**{"base": base, "name": base, "type": "base"})

    cli.run()

    mock_run_new_or_fallback_remote_build.assert_called_once_with(base)


@pytest.mark.usefixtures("mock_argv", "mock_confirm")
def test_get_effective_base_unknown(capsys, snapcraft_yaml):
    """Raise an error for unknown bases."""
    snapcraft_yaml(base="core10")

    cli.run()

    _, err = capsys.readouterr()
    assert "Unknown base 'core10' in 'snap/snapcraft.yaml'." in err


@pytest.mark.usefixtures("mock_argv", "mock_confirm")
def test_get_effective_base_none(capsys, snapcraft_yaml):
    """Raise an error if there is no base in the snapcraft.yaml."""
    snapcraft_yaml()

    cli.run()

    _, err = capsys.readouterr()
    assert "Could not determine base from 'snap/snapcraft.yaml'." in err


@pytest.mark.usefixtures("mock_argv", "mock_confirm")
def test_get_effective_base_core_esm_warning(
    emitter, snapcraft_yaml, mock_run_new_or_fallback_remote_build
):
    """Warn if core, an ESM base, is used."""
    snapcraft_yaml(base="core")

    cli.run()

    mock_run_new_or_fallback_remote_build.assert_called_once_with("core")
    emitter.assert_progress(
        "WARNING: base 'core' was last supported on Snapcraft 4 available on the "
        "'4.x' channel.",
        permanent=True,
    )


@pytest.mark.usefixtures("mock_argv", "mock_confirm")
def test_get_effective_base_core18_esm_warning(
    emitter, snapcraft_yaml, mock_run_new_or_fallback_remote_build
):
    """Warn if core18, an ESM base, is used."""
    snapcraft_yaml(base="core18")

    cli.run()

    mock_run_new_or_fallback_remote_build.assert_called_once_with("core18")
    emitter.assert_progress(
        "WARNING: base 'core18' was last supported on Snapcraft 7 available on the "
        "'7.x' channel.",
        permanent=True,
    )


#######################
# Control logic tests #
#######################


@pytest.mark.parametrize(
    "create_snapcraft_yaml", CURRENT_BASES - {"core22"}, indirect=True
)
@pytest.mark.usefixtures("create_snapcraft_yaml", "mock_confirm", "mock_argv")
def test_run_newer_than_core_22(emitter, mock_run_new_remote_build):
    """Bases newer than core22 must use new remote-build."""
    cli.run()

    mock_run_new_remote_build.assert_called_once()
    emitter.assert_debug("Running new remote-build because base is newer than core22")


@pytest.mark.parametrize(
    "create_snapcraft_yaml", LEGACY_BASES | {"core22"}, indirect=True
)
@pytest.mark.usefixtures("create_snapcraft_yaml", "mock_confirm", "mock_argv")
def test_run_core22_and_older(emitter, mock_run_legacy):
    """core22 and older bases can use fallback remote-build."""
    cli.run()

    mock_run_legacy.assert_called_once()
    emitter.assert_debug("Running fallback remote-build")


@pytest.mark.parametrize(
    "create_snapcraft_yaml", CURRENT_BASES - {"core22"}, indirect=True
)
@pytest.mark.parametrize(
    "envvar", ["force-fallback", "disable-fallback", "badvalue", None]
)
@pytest.mark.usefixtures("create_snapcraft_yaml", "mock_confirm", "mock_argv")
def test_run_envvar_newer_than_core22(
    envvar, emitter, mock_run_new_remote_build, monkeypatch
):
    """Bases newer than core22 run new remote-build regardless of envvar."""
    if envvar:
        monkeypatch.setenv("SNAPCRAFT_REMOTE_BUILD_STRATEGY", envvar)
    else:
        monkeypatch.delenv("SNAPCRAFT_REMOTE_BUILD_STRATEGY", raising=False)

    cli.run()

    mock_run_new_remote_build.assert_called_once()
    emitter.assert_debug("Running new remote-build because base is newer than core22")


@pytest.mark.parametrize(
    "create_snapcraft_yaml", LEGACY_BASES | {"core22"}, indirect=True
)
@pytest.mark.usefixtures("create_snapcraft_yaml", "mock_confirm", "mock_argv")
def test_run_envvar_disable_fallback(emitter, mock_run_new_remote_build, monkeypatch):
    """core22 and older bases run new remote-build if envvar is `disable-fallback`."""
    monkeypatch.setenv("SNAPCRAFT_REMOTE_BUILD_STRATEGY", "disable-fallback")

    cli.run()

    mock_run_new_remote_build.assert_called_once()
    emitter.assert_debug(
        "Running new remote-build because environment variable "
        "'SNAPCRAFT_REMOTE_BUILD_STRATEGY' is 'disable-fallback'"
    )


@pytest.mark.parametrize(
    "create_snapcraft_yaml", LEGACY_BASES | {"core22"}, indirect=True
)
@pytest.mark.usefixtures("create_snapcraft_yaml", "mock_confirm", "mock_argv")
def test_run_envvar_force_fallback(emitter, mock_run_legacy, monkeypatch):
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
@pytest.mark.usefixtures("create_snapcraft_yaml", "mock_confirm", "mock_argv")
def test_run_envvar_force_fallback_unset(emitter, mock_run_legacy, monkeypatch):
    """core22 and older bases run legacy remote-build if envvar is unset."""
    monkeypatch.delenv("SNAPCRAFT_REMOTE_BUILD_STRATEGY", raising=False)

    cli.run()

    mock_run_legacy.assert_called_once()
    emitter.assert_debug("Running fallback remote-build")


@pytest.mark.parametrize(
    "create_snapcraft_yaml", LEGACY_BASES | {"core22"}, indirect=True
)
@pytest.mark.usefixtures("create_snapcraft_yaml", "mock_confirm", "mock_argv")
def test_run_envvar_force_fallback_empty(emitter, mock_run_legacy, monkeypatch):
    """core22 and older bases run legacy remote-build if envvar is empty."""
    monkeypatch.setenv("SNAPCRAFT_REMOTE_BUILD_STRATEGY", "")

    cli.run()

    mock_run_legacy.assert_called_once()
    emitter.assert_debug("Running fallback remote-build")


@pytest.mark.parametrize(
    "create_snapcraft_yaml", LEGACY_BASES | {"core22"}, indirect=True
)
@pytest.mark.usefixtures("create_snapcraft_yaml", "mock_confirm", "mock_argv")
def test_run_envvar_invalid(capsys, emitter, mock_run_legacy, monkeypatch):
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
@pytest.mark.usefixtures("create_snapcraft_yaml", "mock_confirm", "mock_argv")
def test_run_in_repo(emitter, mock_run_new_remote_build, new_dir):
    """core22 and older bases run new remote-build if in a git repo."""
    # initialize a git repo
    GitRepo(new_dir)

    cli.run()

    mock_run_new_remote_build.assert_called_once()
    emitter.assert_debug(
        "Running new remote-build because project is in a git repository"
    )


@pytest.mark.parametrize(
    "create_snapcraft_yaml", LEGACY_BASES | {"core22"}, indirect=True
)
@pytest.mark.usefixtures("create_snapcraft_yaml", "mock_confirm", "mock_argv")
def test_run_not_in_repo(emitter, mock_run_legacy):
    """core22 and older bases run legacy remote-build if not in a git repo."""
    cli.run()

    mock_run_legacy.assert_called_once()
    emitter.assert_debug("Running fallback remote-build")


@pytest.mark.parametrize(
    "create_snapcraft_yaml", CURRENT_BASES - {"core22"}, indirect=True
)
@pytest.mark.usefixtures("create_snapcraft_yaml", "mock_confirm", "mock_argv")
def test_run_in_repo_newer_than_core22(
    emitter, mock_run_new_remote_build, monkeypatch, new_dir
):
    """Bases newer than core22 run new remote-build regardless of being in a repo."""
    # initialize a git repo
    GitRepo(new_dir)

    cli.run()

    mock_run_new_remote_build.assert_called_once()
    emitter.assert_debug("Running new remote-build because base is newer than core22")


@pytest.mark.parametrize(
    "create_snapcraft_yaml", LEGACY_BASES | {"core22"}, indirect=True
)
@pytest.mark.usefixtures("create_snapcraft_yaml", "mock_confirm", "mock_argv")
def test_run_in_shallow_repo(emitter, mock_run_legacy, new_dir):
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


@pytest.mark.parametrize(
    "create_snapcraft_yaml", CURRENT_BASES - {"core22"}, indirect=True
)
@pytest.mark.usefixtures(
    "create_snapcraft_yaml", "mock_confirm", "mock_argv", "use_new_remote_build"
)
def test_run_in_shallow_repo_unsupported(capsys, new_dir):
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

    assert "Remote build for shallow cloned git repos are no longer supported" in err


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
@pytest.mark.usefixtures("mock_argv", "mock_confirm", "use_new_remote_build")
def test_determine_architectures_from_snapcraft_yaml(
    archs, expected_archs, base, snapcraft_yaml, mock_remote_builder
):
    """Parse `build-on` architectures from a snapcraft.yaml file."""
    snapcraft_yaml(base=base, architectures=archs)

    cli.run()

    mock_remote_builder.assert_called_with(
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
    "create_snapcraft_yaml", "mock_argv", "mock_confirm", "use_new_remote_build"
)
def test_determine_architectures_host_arch(mocker, mock_remote_builder):
    """Use host architecture if not defined in the snapcraft.yaml."""
    mocker.patch(
        "snapcraft.commands.remote.get_host_architecture", return_value="arm64"
    )

    cli.run()

    mock_remote_builder.assert_called_with(
        app_name="snapcraft",
        build_id=None,
        project_name="mytest",
        architectures=["arm64"],
        project_dir=Path(),
        timeout=0,
    )


@pytest.mark.parametrize(
    ("args", "expected_archs"),
    [
        (["--build-for", "amd64"], ["amd64"]),
        (["--build-for", "amd64", "arm64"], ["amd64", "arm64"]),
        # launchpad will accept and ignore duplicates
        (["--build-for", "amd64", "amd64"], ["amd64", "amd64"]),
    ],
)
@pytest.mark.parametrize(
    "create_snapcraft_yaml", CURRENT_BASES | LEGACY_BASES, indirect=True
)
@pytest.mark.usefixtures(
    "create_snapcraft_yaml", "mock_confirm", "use_new_remote_build"
)
def test_determine_architectures_provided_by_user(
    args, expected_archs, mocker, mock_remote_builder
):
    """Use architectures provided by the user."""
    mocker.patch.object(sys, "argv", ["snapcraft", "remote-build"] + args)

    cli.run()

    mock_remote_builder.assert_called_with(
        app_name="snapcraft",
        build_id=None,
        project_name="mytest",
        architectures=expected_archs,
        project_dir=Path(),
        timeout=0,
    )


@pytest.mark.parametrize("base", CURRENT_BASES | LEGACY_BASES)
@pytest.mark.usefixtures("mock_confirm", "use_new_remote_build")
def test_determine_architectures_error(base, capsys, snapcraft_yaml, mocker):
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


##################
# Build id tests #
##################


@pytest.mark.parametrize(
    "create_snapcraft_yaml", CURRENT_BASES | LEGACY_BASES, indirect=True
)
@pytest.mark.usefixtures(
    "create_snapcraft_yaml", "mock_confirm", "use_new_remote_build"
)
def test_build_id_provided(mocker, mock_remote_builder):
    """Pass the build id provided as an argument."""
    mocker.patch.object(
        sys, "argv", ["snapcraft", "remote-build", "--build-id", "test-build-id"]
    )

    cli.run()

    mock_remote_builder.assert_called_with(
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
    "create_snapcraft_yaml", "mock_confirm", "mock_argv", "use_new_remote_build"
)
def test_build_id_not_provided(mock_remote_builder):
    """Pass `None` for the build id if it is not provided as an argument."""

    cli.run()

    mock_remote_builder.assert_called_with(
        app_name="snapcraft",
        build_id=None,
        project_name="mytest",
        architectures=ANY,
        project_dir=Path(),
        timeout=0,
    )


@pytest.mark.parametrize("base", CURRENT_BASES | LEGACY_BASES)
@pytest.mark.usefixtures("mock_confirm", "mock_argv", "use_new_remote_build")
def test_build_id_no_project_name_error(base, capsys):
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
    "create_snapcraft_yaml", "mock_confirm", "use_new_remote_build"
)
def test_status(mocker, mock_remote_builder):
    """Print the status when `--status` is provided."""
    mocker.patch.object(sys, "argv", ["snapcraft", "remote-build", "--status"])

    cli.run()

    assert mock_remote_builder.mock_calls[-1] == call().print_status()


@pytest.mark.parametrize(
    "create_snapcraft_yaml", CURRENT_BASES | LEGACY_BASES, indirect=True
)
@pytest.mark.usefixtures(
    "create_snapcraft_yaml",
    "mock_confirm",
    "mock_remote_builder",
    "use_new_remote_build",
)
def test_recover_no_build(emitter, mocker):
    """Warn if no build is found when `--recover` is provided."""
    mocker.patch.object(sys, "argv", ["snapcraft", "remote-build", "--recover"])

    cli.run()

    emitter.assert_progress("No build found", permanent=True)


@pytest.mark.parametrize(
    "create_snapcraft_yaml", CURRENT_BASES | LEGACY_BASES, indirect=True
)
@pytest.mark.usefixtures(
    "create_snapcraft_yaml", "mock_confirm", "use_new_remote_build"
)
def test_recover_build(emitter, mocker, mock_remote_builder):
    """Recover a build when `--recover` is provided."""
    mocker.patch.object(sys, "argv", ["snapcraft", "remote-build", "--recover"])
    mock_remote_builder.return_value.has_outstanding_build.return_value = True

    cli.run()

    assert mock_remote_builder.mock_calls[-3:] == [
        call().print_status(),
        call().monitor_build(),
        call().clean_build(),
    ]


@pytest.mark.parametrize(
    "create_snapcraft_yaml", CURRENT_BASES | LEGACY_BASES, indirect=True
)
@pytest.mark.usefixtures(
    "create_snapcraft_yaml", "mock_argv", "mock_confirm", "use_new_remote_build"
)
def test_recover_build_user_confirms(emitter, mocker, mock_remote_builder):
    """Recover a build when a user confirms."""
    mock_remote_builder.return_value.has_outstanding_build.return_value = True

    cli.run()

    assert mock_remote_builder.mock_calls[-3:] == [
        call().print_status(),
        call().monitor_build(),
        call().clean_build(),
    ]


@pytest.mark.parametrize(
    "create_snapcraft_yaml", CURRENT_BASES | LEGACY_BASES, indirect=True
)
@pytest.mark.usefixtures("create_snapcraft_yaml", "mock_argv", "use_new_remote_build")
def test_recover_build_user_denies(emitter, mocker, mock_remote_builder):
    """Clean and start a new build when a user denies to recover an existing build."""
    mocker.patch(
        # confirm data upload, deny build recovery
        "snapcraft.commands.remote.confirm_with_user",
        side_effect=[True, False],
    )
    mock_remote_builder.return_value.has_outstanding_build.return_value = True

    cli.run()

    assert mock_remote_builder.mock_calls[-3:] == [
        call().start_build(),
        call().monitor_build(),
        call().clean_build(),
    ]


@pytest.mark.parametrize(
    "create_snapcraft_yaml", CURRENT_BASES | LEGACY_BASES, indirect=True
)
@pytest.mark.usefixtures(
    "create_snapcraft_yaml", "mock_argv", "mock_confirm", "use_new_remote_build"
)
def test_remote_build(emitter, mocker, mock_remote_builder):
    """Clean and start a new build."""
    cli.run()

    assert mock_remote_builder.mock_calls[-3:] == [
        call().start_build(),
        call().monitor_build(),
        call().clean_build(),
    ]
