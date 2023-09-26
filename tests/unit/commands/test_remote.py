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
import sys

import pytest

from snapcraft import cli
from snapcraft.parts.yaml_utils import CURRENT_BASES, ESM_BASES, LEGACY_BASES
from snapcraft.remote import GitRepo
from snapcraft_legacy.internal.remote_build.errors import AcceptPublicUploadError

# remote-build control logic may check if the working dir is a git repo,
# so execute all tests inside a test directory
pytestmark = pytest.mark.usefixtures("new_dir")


@pytest.fixture()
def create_snapcraft_yaml(request, snapcraft_yaml):
    """Create a snapcraft.yaml file with a particular base."""
    snapcraft_yaml(base=request.param)


@pytest.fixture
def fake_sudo(monkeypatch):
    monkeypatch.setenv("SUDO_USER", "fake")
    monkeypatch.setattr("os.geteuid", lambda: 0)


@pytest.fixture()
def mock_argv(mocker):
    """Mock `snapcraft remote-build` cli."""
    return mocker.patch.object(sys, "argv", ["snapcraft", "remote-build"])


@pytest.fixture
def mock_confirm(mocker):
    return mocker.patch(
        "snapcraft.commands.remote.confirm_with_user", return_value=True
    )


@pytest.fixture()
def mock_run_new_or_fallback_remote_build(mocker):
    return mocker.patch(
        "snapcraft.commands.remote.RemoteBuildCommand._run_new_or_fallback_remote_build"
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
    mock_confirm, mock_run_new_or_fallback_remote_build
):
    """Raise an error if the user denies the upload prompt."""
    mock_confirm.return_value = False

    with pytest.raises(AcceptPublicUploadError):
        cli.run()

    mock_confirm.assert_called_once_with(
        "All data sent to remote builders will be publicly available. "
        "Are you sure you want to continue?"
    )
    mock_run_new_or_fallback_remote_build.assert_not_called()


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
@pytest.mark.parametrize("base", CURRENT_BASES | LEGACY_BASES)
def test_get_effective_base_type(
    base, snapcraft_yaml, mock_run_new_or_fallback_remote_build
):
    """The effective base should be the name when building a base."""
    snapcraft_yaml(**{"base": base, "name": base, "type": "base"})

    cli.run()

    mock_run_new_or_fallback_remote_build.assert_called_once_with(base)


@pytest.mark.usefixtures("mock_argv", "mock_confirm")
def test_get_effective_base_cannot_load_snapcraft_yaml(capsys):
    """Raise an error if the snapcraft.yaml does not exist."""
    cli.run()

    _, err = capsys.readouterr()
    assert (
        "Could not find snap/snapcraft.yaml. "
        "Are you sure you are in the right directory?" in err
    )


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
@pytest.mark.parametrize("base", ESM_BASES)
def test_get_effective_base_esm(base, capsys, snapcraft_yaml):
    """Raise an error if an ESM base is used."""
    snapcraft_yaml(base=base)

    cli.run()

    _, err = capsys.readouterr()
    assert f"{base!r} is not supported on this version of Snapcraft." in err


#######################
# Control logic tests #
#######################


@pytest.mark.parametrize(
    "create_snapcraft_yaml", CURRENT_BASES - {"core22"}, indirect=True
)
@pytest.mark.usefixtures("create_snapcraft_yaml", "mock_confirm", "mock_argv")
def test_run_newer_than_core_22(emitter, mock_run_legacy):
    """Bases newer than core22 must use new remote-build."""
    cli.run()

    # this should fail when new remote-build code is used (#4323)
    mock_run_legacy.assert_called_once()
    emitter.assert_debug(
        "Using fallback remote-build because new remote-build is not available."
    )


@pytest.mark.parametrize(
    "create_snapcraft_yaml", LEGACY_BASES | {"core22"}, indirect=True
)
@pytest.mark.usefixtures("create_snapcraft_yaml", "mock_confirm", "mock_argv")
def test_run_core22_and_older(emitter, mock_run_legacy):
    """core22 and older bases can use fallback remote-build."""
    cli.run()

    mock_run_legacy.assert_called_once()
    emitter.assert_debug("Running fallback remote-build.")


@pytest.mark.parametrize(
    "create_snapcraft_yaml", CURRENT_BASES - {"core22"}, indirect=True
)
@pytest.mark.parametrize(
    "envvar", ["force-fallback", "disable-fallback", "badvalue", None]
)
@pytest.mark.usefixtures("create_snapcraft_yaml", "mock_confirm", "mock_argv")
def test_run_envvar_newer_than_core22(envvar, emitter, mock_run_legacy, monkeypatch):
    """Bases newer than core22 run new remote-build regardless of envvar."""
    if envvar:
        monkeypatch.setenv("SNAPCRAFT_REMOTE_BUILD_STRATEGY", envvar)
    else:
        monkeypatch.delenv("SNAPCRAFT_REMOTE_BUILD_STRATEGY", raising=False)

    cli.run()

    mock_run_legacy.assert_called_once()
    emitter.assert_debug(
        "Using fallback remote-build because new remote-build is not available."
    )


@pytest.mark.parametrize(
    "create_snapcraft_yaml", LEGACY_BASES | {"core22"}, indirect=True
)
@pytest.mark.usefixtures("create_snapcraft_yaml", "mock_confirm", "mock_argv")
def test_run_envvar_disable_fallback(emitter, mock_run_legacy, monkeypatch):
    """core22 and older bases run new remote-build if envvar is `disable-fallback`."""
    monkeypatch.setenv("SNAPCRAFT_REMOTE_BUILD_STRATEGY", "disable-fallback")

    cli.run()

    mock_run_legacy.assert_called_once()
    emitter.assert_debug(
        "Environment variable 'SNAPCRAFT_REMOTE_BUILD_STRATEGY' is 'disable-fallback' "
        "but running fallback remote-build because new remote-build is not available."
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
        "'SNAPCRAFT_REMOTE_BUILD_STRATEGY' is 'force-fallback'."
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
    emitter.assert_debug("Running fallback remote-build.")


@pytest.mark.parametrize(
    "create_snapcraft_yaml", LEGACY_BASES | {"core22"}, indirect=True
)
@pytest.mark.usefixtures("create_snapcraft_yaml", "mock_confirm", "mock_argv")
def test_run_envvar_force_fallback_empty(emitter, mock_run_legacy, monkeypatch):
    """core22 and older bases run legacy remote-build if envvar is empty."""
    monkeypatch.setenv("SNAPCRAFT_REMOTE_BUILD_STRATEGY", "")

    cli.run()

    mock_run_legacy.assert_called_once()
    emitter.assert_debug("Running fallback remote-build.")


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
        "'force-fallback'."
    ) in err


@pytest.mark.parametrize(
    "create_snapcraft_yaml", LEGACY_BASES | {"core22"}, indirect=True
)
@pytest.mark.usefixtures("create_snapcraft_yaml", "mock_confirm", "mock_argv")
def test_run_in_repo(emitter, mock_run_legacy, new_dir):
    """core22 and older bases run new remote-build if in a git repo."""
    # initialize a git repo
    GitRepo(new_dir)

    cli.run()

    # this should fail when new remote-build code is used (#4323)
    mock_run_legacy.assert_called_once()
    emitter.assert_debug(
        "Project is in a git repository but running fallback remote-build "
        "because new remote-build is not available."
    )


@pytest.mark.parametrize(
    "create_snapcraft_yaml", LEGACY_BASES | {"core22"}, indirect=True
)
@pytest.mark.usefixtures("create_snapcraft_yaml", "mock_confirm", "mock_argv")
def test_run_not_in_repo(emitter, mock_run_legacy):
    """core22 and older bases run legacy remote-build if not in a git repo."""
    cli.run()

    mock_run_legacy.assert_called_once()
    emitter.assert_debug("Running fallback remote-build.")


@pytest.mark.parametrize(
    "create_snapcraft_yaml", CURRENT_BASES - {"core22"}, indirect=True
)
@pytest.mark.usefixtures("create_snapcraft_yaml", "mock_confirm", "mock_argv")
def test_run_in_repo_newer_than_core22(emitter, mock_run_legacy, monkeypatch, new_dir):
    """Bases newer than core22 run new remote-build regardless of being in a repo."""
    # initialize a git repo
    GitRepo(new_dir)

    cli.run()

    # this should fail when new remote-build code is used (#4323)
    mock_run_legacy.assert_called_once()
    emitter.assert_debug(
        "Using fallback remote-build because new remote-build is not available."
    )
