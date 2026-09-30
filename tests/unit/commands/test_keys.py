# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2026 Canonical Ltd.
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

"""Tests for key management commands."""

import argparse
import json
import os
import re
import subprocess
import sys
from pathlib import Path
from typing import Any
from unittest.mock import Mock, call

import craft_store
import pytest
from craft_cli.pytest_plugin import RecordingEmitter
from pytest_mock import MockerFixture

from snapcraft import cli, commands, store
from snapcraft.commands import keys
from snapcraft.commands.keys import StoreSignBuildCommand, _get_usable_keys
from snapcraft.store.errors import (
    KeyNotRegisteredError,
    StoreBuildAssertionPermissionError,
)
from tests.unit.store.utils import FakeResponse

_TEST_KEY_1 = {"name": "test-key-1", "sha3-384": "abc123"}
_TEST_KEY_2 = {"name": "test-key-2", "sha3-384": "deadbeef"}


@pytest.fixture
def mock_subprocess_check_call(mocker):
    return mocker.patch("subprocess.check_call")


@pytest.fixture
def mock_subprocess_check_output(mocker):
    return mocker.patch("subprocess.check_output")


@pytest.fixture
def mock_isatty(mocker):
    yield mocker.patch("snapcraft.utils.sys.stdin.isatty", return_value=True)


@pytest.fixture
def fake_store_client(mocker):
    """Fake StoreClientCLI with a get_account_info endpoint."""
    mock_cls = mocker.patch("snapcraft.store.StoreClientCLI")
    mock_cls.return_value.get_account_info.return_value = {
        "account_id": "test-account-id",
        "account_keys": [],
    }
    return mock_cls.return_value


@pytest.fixture
def fake_get_usable_keys(mocker):
    """Mock _get_usable_keys to yield no keys."""
    return mocker.patch(
        "snapcraft.commands.keys._get_usable_keys",
        return_value=[],
    )


class TestGetUsableKeys:
    """Tests for the _get_usable_keys() function."""

    @pytest.mark.parametrize(
        ("name", "expected"),
        [
            pytest.param(
                None,
                [
                    _TEST_KEY_1,
                    _TEST_KEY_2,
                ],
                id="no-name",
            ),
            pytest.param(
                "test-key-1",
                [
                    _TEST_KEY_1,
                ],
                id="matching-name",
            ),
            pytest.param(
                "unknown-key",
                [],
                id="non-matching-name",
            ),
        ],
    )
    def test_get_usable_keys(self, mocker, name, expected):
        mocker.patch(
            "subprocess.check_output",
            return_value=json.dumps(
                [
                    _TEST_KEY_1,
                    _TEST_KEY_2,
                ]
            ),
        )

        keys = list(_get_usable_keys(name=name))

        assert keys == expected

    @pytest.mark.parametrize("name", [None, "test-key-1"])
    def test_get_usable_keys_no_keys(self, mocker, name):
        mocker.patch("subprocess.check_output", return_value=json.dumps(None))

        keys = list(_get_usable_keys(name=name))

        assert keys == []

    def test_get_usable_keys_snapd_error(self, mocker):
        mocker.patch(
            "subprocess.check_output",
            side_effect=subprocess.CalledProcessError(1, ["snap", "keys", "--json"]),
        )

        with pytest.raises(subprocess.CalledProcessError):
            list(_get_usable_keys())

    def test_get_usable_keys_snap_not_found(self, mocker):
        mocker.patch(
            "subprocess.check_output",
            side_effect=FileNotFoundError("snap: command not found"),
        )

        with pytest.raises(FileNotFoundError):
            list(_get_usable_keys())

    def test_get_usable_keys_invalid_json(self, mocker):
        mocker.patch("subprocess.check_output", return_value="not valid json{{")

        with pytest.raises(json.JSONDecodeError):
            list(_get_usable_keys())

    def test_get_usable_keys_key_error(self, mocker):
        """Error if snapd output is missing the 'name' key."""
        mocker.patch(
            "subprocess.check_output", return_value=json.dumps([{"sha3-384": "abc123"}])
        )

        with pytest.raises(KeyError):
            list(_get_usable_keys(name="test-key-1"))

    def test_get_usable_keys_type_error(self, mocker):
        """Error if snapd output isn't a list of key dicts."""
        mocker.patch("subprocess.check_output", return_value=json.dumps([42, 99]))

        with pytest.raises(TypeError):
            list(_get_usable_keys(name="test-key-1"))


class TestKeysCommand:
    """Tests for the 'keys' command."""

    def test_keys(
        self, fake_store_client, fake_app_config, emitter, fake_get_usable_keys
    ):
        """Test local keys, registered keys, and keys not on the system."""
        fake_get_usable_keys.return_value = [_TEST_KEY_1, _TEST_KEY_2]
        fake_store_client.get_account_info.return_value = {
            "account_keys": [
                {
                    "name": _TEST_KEY_1["name"],
                    "public-key-sha3-384": _TEST_KEY_1["sha3-384"],
                },
                {"name": "not-on-system-key", "public-key-sha3-384": "def456"},
            ]
        }

        cmd = commands.StoreKeysCommand(fake_app_config)

        cmd.run(argparse.Namespace())

        emitter.assert_message("The following keys are available on this system:")
        emitter.assert_message(
            "    Name        SHA3-384 fingerprint\n"
            "*   test-key-1  abc123\n"
            "-   test-key-2  deadbeef                (not registered)"
        )
        emitter.assert_message(
            "The following SHA3-384 key fingerprints have been registered but are not available on this system:\n"
            "- def456"
        )

    @pytest.mark.usefixtures("fake_get_usable_keys", "fake_store_client")
    def test_keys_no_keys(self, fake_app_config, emitter):
        """No local keys and no registered keys."""
        cmd = commands.StoreKeysCommand(fake_app_config)

        cmd.run(argparse.Namespace())

        emitter.assert_message(
            "No keys have been created on this system. "
            "See 'snapcraft create-key --help' to create a key."
        )
        emitter.assert_message(
            "No keys have been registered with this account. "
            "See 'snapcraft register-key --help' to register a key."
        )

    def test_list_keys_error(self, mocker, capsys):
        """Error on removed 'list-keys' command."""
        mocker.patch.object(sys, "argv", ["cmd", "list-keys"])

        cli.run()

        out, err = capsys.readouterr()
        assert not out
        assert (
            "The 'list-keys' command was renamed to 'keys'.\n"
            "Recommended resolution: Use 'keys' instead."
        ) in err


class TestCreateKeyCommand:
    """Tests for the 'create-key' command."""

    @pytest.mark.usefixtures("fake_store_client", "fake_get_usable_keys")
    def test_create_key(self, mock_subprocess_check_call, fake_app_config):
        cmd = commands.StoreCreateKeyCommand(fake_app_config)

        cmd.run(argparse.Namespace(key_name="test-key-1"))

        assert mock_subprocess_check_call.mock_calls == [
            call(["snap", "create-key", "test-key-1"])
        ]

    def test_create_key_already_exists(
        self,
        mock_subprocess_check_call,
        fake_store_client,
        fake_app_config,
        fake_get_usable_keys,
    ):
        fake_get_usable_keys.return_value = [_TEST_KEY_1]

        cmd = commands.StoreCreateKeyCommand(fake_app_config)

        with pytest.raises(store.errors.KeyAlreadyExistsError):
            cmd.run(argparse.Namespace(key_name="test-key-1"))

        mock_subprocess_check_call.assert_not_called()
        fake_store_client.get_account_info.assert_not_called()

    @pytest.mark.usefixtures("fake_get_usable_keys")
    def test_create_key_already_registered(
        self, fake_store_client, mock_subprocess_check_call, fake_app_config
    ):
        fake_store_client.get_account_info.return_value = {
            "account_keys": [{"name": "test-key-1"}]
        }

        cmd = commands.StoreCreateKeyCommand(fake_app_config)

        with pytest.raises(store.errors.KeyAlreadyRegisteredError):
            cmd.run(argparse.Namespace(key_name="test-key-1"))

        mock_subprocess_check_call.assert_not_called()

    @pytest.mark.usefixtures("fake_get_usable_keys")
    def test_create_key_not_logged_in(
        self, mock_subprocess_check_call, fake_store_client, fake_app_config
    ):
        """Ignore 401 from store."""
        fake_store_client.get_account_info.side_effect = (
            craft_store.errors.StoreServerError(
                response=FakeResponse(
                    content=json.dumps(
                        {
                            "error_list": [
                                {"code": "unauthorized", "message": "Unauthorized"}
                            ]
                        }
                    ).encode(),
                    status_code=401,
                )
            )
        )

        cmd = commands.StoreCreateKeyCommand(fake_app_config)

        cmd.run(argparse.Namespace(key_name="test-key-1"))

        assert mock_subprocess_check_call.mock_calls == [
            call(["snap", "create-key", "test-key-1"])
        ]

    @pytest.mark.usefixtures("fake_get_usable_keys")
    def test_create_key_store_error(
        self, mock_subprocess_check_call, fake_store_client, fake_app_config
    ):
        """A non-401 store error is re-raised."""
        fake_store_client.get_account_info.side_effect = (
            craft_store.errors.StoreServerError(
                response=FakeResponse(
                    content=json.dumps(
                        {
                            "error_list": [
                                {
                                    "code": "server-error",
                                    "message": "Internal Server Error",
                                }
                            ]
                        }
                    ).encode(),
                    status_code=500,
                )
            )
        )

        cmd = commands.StoreCreateKeyCommand(fake_app_config)

        with pytest.raises(craft_store.errors.StoreServerError):
            cmd.run(argparse.Namespace(key_name="test-key-1"))

        mock_subprocess_check_call.assert_not_called()


class TestRegisterKeyCommand:
    """Tests for the 'register-key' command."""

    def test_register_key(
        self,
        fake_store_client,
        fake_app_config,
        fake_get_usable_keys,
        mock_subprocess_check_output,
        monkeypatch,
        emitter,
    ):
        monkeypatch.setenv(store.constants.ENVIRONMENT_STORE_CREDENTIALS, "creds")
        fake_get_usable_keys.return_value = [_TEST_KEY_1]
        mock_subprocess_check_output.return_value = "test-serialized-assertion"
        cmd = commands.StoreRegisterKeyCommand(fake_app_config)

        cmd.run(argparse.Namespace(key_name="test-key-1"))

        fake_store_client.login.assert_called_once_with(
            acls=["modify_account_key"],
            ttl=86400,
        )
        assert mock_subprocess_check_output.mock_calls == [
            call(
                ["snap", "export-key", "--account=test-account-id", "test-key-1"],
                universal_newlines=True,
            )
        ]
        fake_store_client.register_key.assert_called_once_with(
            "test-serialized-assertion"
        )
        emitter.assert_message(
            "Done. The key 'test-key-1' ('abc123') may be used to sign your assertions."
        )
        # restore credentials after running
        assert os.environ[store.constants.ENVIRONMENT_STORE_CREDENTIALS] == "creds"

    def test_register_key_no_key(
        self,
        fake_store_client,
        fake_app_config,
        fake_get_usable_keys,
    ):
        """Error when the provided key doesn't exist."""
        fake_get_usable_keys.return_value = []
        cmd = commands.StoreRegisterKeyCommand(fake_app_config)

        with pytest.raises(store.errors.NoSuchKeyError):
            cmd.run(argparse.Namespace(key_name="missing-key"))

        fake_store_client.login.assert_not_called()
        fake_store_client.register_key.assert_not_called()

    def test_register_key_no_keys_exist(
        self,
        fake_store_client,
        fake_app_config,
        fake_get_usable_keys,
    ):
        """Error when no keys exist and no key name is provided."""
        fake_get_usable_keys.return_value = []
        cmd = commands.StoreRegisterKeyCommand(fake_app_config)

        with pytest.raises(store.errors.NoKeysError):
            cmd.run(argparse.Namespace(key_name=None))

        fake_store_client.login.assert_not_called()
        fake_store_client.register_key.assert_not_called()

    @pytest.mark.parametrize(
        ("selection", "key_name"),
        [
            ("1", _TEST_KEY_1["name"]),
            ("2", _TEST_KEY_2["name"]),
        ],
    )
    def test_register_key_selects_from_multiple_keys(
        self,
        mocker,
        fake_store_client,
        fake_app_config,
        fake_get_usable_keys,
        mock_subprocess_check_output,
        mock_isatty,
        selection,
        key_name,
        emitter,
    ):
        fake_get_usable_keys.return_value = [_TEST_KEY_1, _TEST_KEY_2]
        mock_subprocess_check_output.return_value = "test-serialized-assertion"
        mocker.patch("craft_cli.emit.prompt", return_value=selection)
        cmd = commands.StoreRegisterKeyCommand(fake_app_config)

        cmd.run(argparse.Namespace(key_name=None))

        assert mock_subprocess_check_output.mock_calls == [
            call(
                ["snap", "export-key", "--account=test-account-id", key_name],
                universal_newlines=True,
            )
        ]
        fake_store_client.register_key.assert_called_once_with(
            "test-serialized-assertion"
        )
        emitter.assert_progress("Select a key:\n", permanent=True)
        emitter.assert_progress(
            "  Number  Name        SHA3-384 fingerprint\n"
            "       1  test-key-1  abc123\n"
            "       2  test-key-2  deadbeef\n",
            permanent=True,
        )


class TestSignBuildCommand:
    _BASIC_SNAP_YAML = {
        "name": "basic",
        "summary": "test summary",
        "description": "test description",
        "version": "0.1",
    }

    @pytest.fixture(autouse=True)
    def fake_get_data_from_snap_file(self, mocker: MockerFixture) -> Mock:
        return mocker.patch(
            "snapcraft.utils.get_data_from_snap_file",
            return_value=(self._BASIC_SNAP_YAML, None),
        )

    @pytest.fixture
    def fake_sign_build(self, fake_app_config: dict[str, Any]) -> StoreSignBuildCommand:
        return StoreSignBuildCommand(fake_app_config)

    @pytest.fixture
    def fake_snap_file(self, tmp_path: Path) -> Path:
        snap_file = tmp_path / "test_snap.snap"
        snap_file.touch()
        return snap_file

    @pytest.fixture(autouse=True)
    def fake_isatty(self, mocker: MockerFixture):
        yield mocker.patch("snapcraft.utils.sys.stdin.isatty", return_value=True)

    @pytest.fixture(autouse=True)
    def fake_store_client(self, mocker: MockerFixture) -> Mock:
        fake_client = mocker.patch("snapcraft.store.StoreClientCLI", autospec=True)
        fake_client.return_value.get_account_info.return_value = {
            "id": "1234",
            "snaps": {"16": {"basic": {"snap-id": "test-snap-id"}}},
            "account-keys": [{"name": "good", "public-key-sha3-384": "good_hash"}],
        }
        return fake_client.return_value

    @pytest.fixture
    def fake_check_output(self, mocker: MockerFixture) -> Mock:
        return mocker.patch("subprocess.check_output")

    @pytest.fixture(autouse=True)
    def fake_check_for_key(self, mocker: MockerFixture) -> Mock:
        return mocker.patch(
            "snapcraft.commands.keys._maybe_prompt_for_key",
            return_value={"name": "good", "sha3-384": "good_hash"},
        )

    def test_sign_build_nonexisting_snap(
        self, fake_sign_build: StoreSignBuildCommand
    ) -> None:
        namespace = argparse.Namespace(snap_file=Path("nonexisting.snap"))

        with pytest.raises(
            FileNotFoundError,
            match=re.escape("The file 'nonexisting.snap' does not exist."),
        ):
            fake_sign_build.run(namespace)

    def test_sign_build_missing_permissions(
        self,
        fake_sign_build: StoreSignBuildCommand,
        fake_snap_file: Path,
        fake_get_data_from_snap_file: Mock,
    ) -> None:
        fake_get_data_from_snap_file.return_value = (
            self._BASIC_SNAP_YAML | {"name": "unowned"},
            None,
        )
        namespace = argparse.Namespace(snap_file=fake_snap_file)

        with pytest.raises(StoreBuildAssertionPermissionError):
            fake_sign_build.run(namespace)

    def test_sign_build_unregistered_key(
        self,
        fake_sign_build: StoreSignBuildCommand,
        fake_snap_file: Path,
        fake_check_for_key: Mock,
    ) -> None:
        fake_check_for_key.return_value = {
            "name": "not-uploaded",
            "sha3-384": "da-hash",
        }
        namespace = argparse.Namespace(
            snap_file=fake_snap_file, key_name=None, local=False
        )

        with pytest.raises(KeyNotRegisteredError):
            fake_sign_build.run(namespace)

    def test_sign_build_local(
        self,
        fake_sign_build: StoreSignBuildCommand,
        fake_snap_file: Path,
        fake_check_output: Mock,
        tmp_path: Path,
        emitter: RecordingEmitter,
    ) -> None:
        namespace = argparse.Namespace(
            snap_file=fake_snap_file, key_name=None, local=True
        )
        fake_check_output.return_value = b"success!"
        expected_snap_build = tmp_path / (fake_snap_file.name + "-build")

        fake_sign_build.run(namespace)

        assert expected_snap_build.exists()
        assert expected_snap_build.read_bytes() == b"success!"
        emitter.assert_message(f"Build assertion {expected_snap_build} saved to disk.")
        assert (
            call(
                "message",
                f"Build assertion {expected_snap_build} uploaded to the store.",
            )
            not in emitter.interactions
        )

    def test_sign_build_remote(
        self,
        fake_sign_build: StoreSignBuildCommand,
        fake_snap_file: Path,
        fake_check_output: Mock,
        fake_store_client: Mock,
        tmp_path: Path,
        emitter: RecordingEmitter,
    ) -> None:
        namespace = argparse.Namespace(
            snap_file=fake_snap_file, key_name=None, local=False
        )
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
        self,
        fake_sign_build: StoreSignBuildCommand,
        fake_snap_file: Path,
        tmp_path: Path,
        emitter: RecordingEmitter,
        mocker: MockerFixture,
    ) -> None:
        namespace = argparse.Namespace(
            snap_file=fake_snap_file, key_name=None, local=True
        )
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


def test_select_key_skips_invalid_input(mocker):
    """Retry key selection until a valid key is chosen."""
    mocker.patch(
        "craft_cli.emit.prompt",
        side_effect=["x", "", "99", "0", "1"],
    )

    result = keys._select_key([_TEST_KEY_1, _TEST_KEY_2])

    assert result == _TEST_KEY_1
