# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2026 Canonical Ltd
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

import argparse
import re
from pathlib import Path
from typing import Any
from unittest.mock import Mock, call

import pytest
from craft_cli.pytest_plugin import RecordingEmitter
from pytest_mock import MockerFixture

from snapcraft.commands import StoreSignBuildCommand
from snapcraft.store.errors import (
    KeyNotRegisteredError,
    StoreBuildAssertionPermissionError,
)

_BASIC_SNAP_YAML = {
    "name": "basic",
    "summary": "test summary",
    "description": "test description",
    "version": "0.1",
}


@pytest.fixture(autouse=True)
def fake_get_data_from_snap_file(mocker: MockerFixture) -> Mock:
    return mocker.patch(
        "snapcraft.utils.get_data_from_snap_file", return_value=(_BASIC_SNAP_YAML, None)
    )


@pytest.fixture
def fake_sign_build(fake_app_config: dict[str, Any]) -> StoreSignBuildCommand:
    return StoreSignBuildCommand(fake_app_config)


@pytest.fixture
def fake_snap_file(tmp_path: Path) -> Path:
    snap_file = tmp_path / "test_snap.snap"
    snap_file.touch()
    return snap_file


@pytest.fixture(autouse=True)
def fake_isatty(mocker: MockerFixture):
    yield mocker.patch("snapcraft.utils.sys.stdin.isatty", return_value=True)


@pytest.fixture(autouse=True)
def fake_store_client(mocker: MockerFixture) -> Mock:
    fake_client = mocker.patch("snapcraft.store.StoreClientCLI", autospec=True)
    fake_client.return_value.get_account_info.return_value = {
        "id": "1234",
        "snaps": {"16": {"basic": {"snap-id": "test-snap-id"}}},
        "account-keys": [{"name": "good", "public-key-sha3-384": "good_hash"}],
    }
    return fake_client.return_value


@pytest.fixture
def fake_check_output(mocker: MockerFixture) -> Mock:
    return mocker.patch("subprocess.check_output")


@pytest.fixture(autouse=True)
def fake_check_for_key(mocker: MockerFixture) -> Mock:
    return mocker.patch(
        "snapcraft.commands.keys._maybe_prompt_for_key",
        return_value={"name": "good", "sha3-384": "good_hash"},
    )


def test_sign_build_nonexisting_snap(fake_sign_build: StoreSignBuildCommand) -> None:
    namespace = argparse.Namespace(snap_file=Path("nonexisting.snap"))

    with pytest.raises(
        FileNotFoundError,
        match=re.escape("The file 'nonexisting.snap' does not exist."),
    ):
        fake_sign_build.run(namespace)


def test_sign_build_missing_permissions(
    fake_sign_build: StoreSignBuildCommand,
    fake_snap_file: Path,
    fake_get_data_from_snap_file: Mock,
) -> None:
    fake_get_data_from_snap_file.return_value = (
        _BASIC_SNAP_YAML | {"name": "unowned"},
        None,
    )
    namespace = argparse.Namespace(snap_file=fake_snap_file)

    with pytest.raises(StoreBuildAssertionPermissionError):
        fake_sign_build.run(namespace)


def test_sign_build_unregistered_key(
    fake_sign_build: StoreSignBuildCommand,
    fake_snap_file: Path,
    fake_check_for_key: Mock,
) -> None:
    fake_check_for_key.return_value = {"name": "not-uploaded", "sha3-384": "da-hash"}
    namespace = argparse.Namespace(snap_file=fake_snap_file, key_name=None, local=False)

    with pytest.raises(KeyNotRegisteredError):
        fake_sign_build.run(namespace)


def test_sign_build_local(
    fake_sign_build: StoreSignBuildCommand,
    fake_snap_file: Path,
    fake_check_output: Mock,
    tmp_path: Path,
    emitter: RecordingEmitter,
) -> None:
    namespace = argparse.Namespace(snap_file=fake_snap_file, key_name=None, local=True)
    fake_check_output.return_value = b"success!"
    expected_snap_build = tmp_path / (fake_snap_file.name + "-build")

    fake_sign_build.run(namespace)

    assert expected_snap_build.exists()
    assert expected_snap_build.read_bytes() == b"success!"
    emitter.assert_message(f"Build assertion {expected_snap_build} saved to disk.")
    assert (
        call("message", f"Build assertion {expected_snap_build} uploaded to the store.")
        not in emitter.interactions
    )


def test_sign_build_remote(
    fake_sign_build: StoreSignBuildCommand,
    fake_snap_file: Path,
    fake_check_output: Mock,
    fake_store_client: Mock,
    tmp_path: Path,
    emitter: RecordingEmitter,
) -> None:
    namespace = argparse.Namespace(snap_file=fake_snap_file, key_name=None, local=False)
    fake_check_output.return_value = b"success!"
    expected_snap_build = tmp_path / (fake_snap_file.name + "-build")

    fake_sign_build.run(namespace)

    assert expected_snap_build.exists()
    assert expected_snap_build.read_bytes() == b"success!"
    emitter.assert_messages(
        [
            f"Build assertion {expected_snap_build} saved to disk.",
            f"Build assertion {expected_snap_build} uploaded to the store.",
        ]
    )
    fake_store_client.push_snap_build.assert_called_once_with(
        "test-snap-id", "success!"
    )


def test_sign_build_already_existed(
    fake_sign_build: StoreSignBuildCommand,
    fake_snap_file: Path,
    tmp_path: Path,
    emitter: RecordingEmitter,
    mocker: MockerFixture,
) -> None:
    namespace = argparse.Namespace(snap_file=fake_snap_file, key_name=None, local=True)
    existing_snap_build = tmp_path / (fake_snap_file.name + "-build")
    existing_snap_build.write_bytes(b"already here")
    spy_generate = mocker.spy(fake_sign_build, "_generate_snap_build")

    fake_sign_build.run(namespace)

    assert existing_snap_build.exists()
    assert existing_snap_build.read_bytes() == b"already here"
    emitter.assert_progress(
        "A signed build assertion for this snap already exists.", permanent=True
    )
    spy_generate.assert_not_called()
