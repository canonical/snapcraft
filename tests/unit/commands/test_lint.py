# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2023 Canonical Ltd.
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
from pathlib import Path
from subprocess import CalledProcessError
from unittest.mock import Mock, call

import pytest
from craft_providers.bases import BuilddBaseAlias

from snapcraft import cli


@pytest.fixture
def mock_ensure_provider_is_available(mocker):
    return mocker.patch(
        "snapcraft.parts.lifecycle.providers.ensure_provider_is_available"
    )


@pytest.fixture
def mock_get_base_configuration(mocker):
    return mocker.patch("snapcraft.parts.lifecycle.providers.get_base_configuration")


@pytest.fixture
def mock_is_managed_mode(mocker):
    return mocker.patch("snapcraft.commands.lint.is_managed_mode", return_value=False)


@pytest.fixture()
def mock_provider(mocker, mock_instance, fake_provider):
    _mock_provider = Mock(wraps=fake_provider)
    mocker.patch(
        "snapcraft.commands.lint.providers.get_provider", return_value=_mock_provider
    )
    return _mock_provider


def test_lint_default(
    emitter,
    mock_ensure_provider_is_available,
    mock_get_base_configuration,
    mock_instance,
    mock_is_managed_mode,
    mock_provider,
    mocker,
    tmp_path,
):
    """Test the lint command."""
    # create a snap file
    snap_file = tmp_path / "test-snap.snap"
    snap_file.touch()
    mocker.patch.object(sys, "argv", ["snapcraft", "lint", str(snap_file)])
    # create an assertion file
    assert_file = tmp_path / "test-snap.assert"
    assert_file.touch()

    cli.run()

    mock_ensure_provider_is_available.assert_called_once()
    mock_get_base_configuration.assert_called_once_with(
        alias=BuilddBaseAlias.JAMMY,
        http_proxy=None,
        https_proxy=None,
        instance_name="snapcraft-linter",
    )
    assert mock_instance.push_file.mock_calls == [
        call(source=snap_file, destination=Path("/root/test-snap.snap")),
        call(source=assert_file, destination=Path("/root/test-snap.assert")),
    ]
    mock_instance.execute_run.assert_called_once_with(
        ["snapcraft", "lint", "/root/test-snap.snap"], check=True
    )
    emitter.assert_interactions(
        [
            call("progress", "Running linter.", permanent=True),
            call(
                "debug", f"Found assertion file {str(tmp_path / 'test-snap.assert')!r}."
            ),
            call("progress", "Checking build provider availability."),
            call("progress", "Launching instance."),
        ]
    )


def test_lint_http_https_proxy(
    emitter,
    mock_ensure_provider_is_available,
    mock_get_base_configuration,
    mock_instance,
    mock_is_managed_mode,
    mock_provider,
    mocker,
    tmp_path,
):
    """Pass the http and https proxies into the instance."""
    # create a snap file
    snap_file = tmp_path / "test-snap.snap"
    snap_file.touch()
    mocker.patch.object(
        sys,
        "argv",
        [
            "snapcraft",
            "lint",
            str(snap_file),
            "--http-proxy",
            "test-http-proxy",
            "--https-proxy",
            "test-https-proxy",
        ],
    )

    cli.run()

    mock_get_base_configuration.assert_called_once_with(
        alias=BuilddBaseAlias.JAMMY,
        http_proxy="test-http-proxy",
        https_proxy="test-https-proxy",
        instance_name="snapcraft-linter",
    )


def test_lint_assert_file_missing(
    emitter,
    mock_ensure_provider_is_available,
    mock_get_base_configuration,
    mock_instance,
    mock_is_managed_mode,
    mock_provider,
    mocker,
    tmp_path,
):
    """Do not push non-existent assertion files in the instance."""
    # create a snap file
    snap_file = tmp_path / "test-snap.snap"
    snap_file.touch()
    mocker.patch.object(sys, "argv", ["snapcraft", "lint", str(snap_file)])

    cli.run()

    mock_instance.push_file.assert_called_once_with(
        source=snap_file,
        destination=Path("/root/test-snap.snap"),
    )
    mock_instance.execute_run.assert_called_once_with(
        ["snapcraft", "lint", "/root/test-snap.snap"], check=True
    )
    emitter.assert_debug(
        f"Assertion file {str(tmp_path / 'test-snap.assert')!r} does not exist."
    )


def test_lint_assert_file_not_valid(
    emitter,
    mock_ensure_provider_is_available,
    mock_get_base_configuration,
    mock_instance,
    mock_is_managed_mode,
    mock_provider,
    mocker,
    tmp_path,
):
    """Do not push invalid assertion files in the instance."""
    # create a snap file
    snap_file = tmp_path / "test-snap.snap"
    snap_file.touch()
    mocker.patch.object(sys, "argv", ["snapcraft", "lint", str(snap_file)])
    # make the assertion filepath a directory
    assert_file = tmp_path / "test-snap.assert"
    assert_file.mkdir()

    cli.run()

    mock_instance.push_file.assert_called_once_with(
        source=snap_file,
        destination=Path("/root/test-snap.snap"),
    )
    mock_instance.execute_run.assert_called_once_with(
        ["snapcraft", "lint", "/root/test-snap.snap"], check=True
    )
    emitter.assert_debug(
        f"Assertion file {str(tmp_path / 'test-snap.assert')!r} is not a valid file."
    )


def test_lint_default_snap_file_missing(capsys, mocker, tmp_path):
    """Raise an error if the snap file does not exist."""
    # do not create a snap file
    snap_file = tmp_path / "test-snap.snap"
    mocker.patch.object(sys, "argv", ["snapcraft", "lint", str(snap_file)])

    cli.run()

    out, err = capsys.readouterr()
    assert not out
    assert f"snap file {str(snap_file)!r} does not exist" in err


def test_lint_default_snap_file_not_valid(capsys, mocker, tmp_path):
    """Raise an error if the snap file is not valid."""
    # make the snap filepath a directory
    snap_file = tmp_path / "test-snap.snap"
    snap_file.mkdir()
    mocker.patch.object(sys, "argv", ["snapcraft", "lint", str(snap_file)])

    cli.run()

    out, err = capsys.readouterr()
    assert not out
    assert f"snap file {str(snap_file)!r} is not a valid file" in err


def test_lint_execute_run(
    capsys,
    emitter,
    mock_ensure_provider_is_available,
    mock_get_base_configuration,
    mock_instance,
    mock_is_managed_mode,
    mock_provider,
    mocker,
    tmp_path,
):
    """Raise an error if running snapcraft in the instance fails."""
    # create a snap file
    snap_file = tmp_path / "test-snap.snap"
    snap_file.touch()
    mocker.patch.object(sys, "argv", ["snapcraft", "lint", str(snap_file)])
    mock_instance.execute_run.side_effect = CalledProcessError(cmd="err", returncode=1)

    cli.run()

    out, err = capsys.readouterr()
    assert not out
    assert "failed to execute 'snapcraft lint /root/test-snap.snap' in instance" in err
