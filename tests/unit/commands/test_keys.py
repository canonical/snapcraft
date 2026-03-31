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
import subprocess
from unittest.mock import call

import craft_store
import pytest

from snapcraft import commands, store
from snapcraft.commands.keys import _get_usable_keys
from tests.unit.store.utils import FakeResponse

_TEST_KEY_1 = {"name": "test-key-1", "sha3-384": "abc123"}
_TEST_KEY_2 = {"name": "test-key-2", "sha3-384": "deadbeef"}


@pytest.fixture
def mock_subprocess_check(mocker):
    return mocker.patch("subprocess.check_call")


@pytest.fixture
def fake_store_client(mocker):
    """Fake StoreClientCLI with a get_account_info endpoint."""
    mock_cls = mocker.patch("snapcraft.store.StoreClientCLI")
    mock_cls.return_value.get_account_info.return_value = {"account_keys": []}
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


class TestCreateKeyCommand:
    """Tests for the 'create-key' command."""

    @pytest.mark.usefixtures("fake_store_client", "fake_get_usable_keys")
    def test_create_key(self, mock_subprocess_check, fake_app_config):
        cmd = commands.StoreCreateKeyCommand(fake_app_config)

        cmd.run(argparse.Namespace(key_name="test-key-1"))

        assert mock_subprocess_check.mock_calls == [
            call(["snap", "create-key", "test-key-1"])
        ]

    def test_create_key_already_exists(
        self,
        mock_subprocess_check,
        fake_store_client,
        fake_app_config,
        fake_get_usable_keys,
    ):
        fake_get_usable_keys.return_value = [_TEST_KEY_1]

        cmd = commands.StoreCreateKeyCommand(fake_app_config)

        with pytest.raises(store.errors.KeyAlreadyExistsError):
            cmd.run(argparse.Namespace(key_name="test-key-1"))

        mock_subprocess_check.assert_not_called()
        fake_store_client.get_account_info.assert_not_called()

    @pytest.mark.usefixtures("fake_get_usable_keys")
    def test_create_key_already_registered(
        self, fake_store_client, mock_subprocess_check, fake_app_config
    ):
        fake_store_client.get_account_info.return_value = {
            "account_keys": [{"name": "test-key-1"}]
        }

        cmd = commands.StoreCreateKeyCommand(fake_app_config)

        with pytest.raises(store.errors.KeyAlreadyRegisteredError):
            cmd.run(argparse.Namespace(key_name="test-key-1"))

        mock_subprocess_check.assert_not_called()

    @pytest.mark.usefixtures("fake_get_usable_keys")
    def test_create_key_not_logged_in(
        self, mock_subprocess_check, fake_store_client, fake_app_config
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

        assert mock_subprocess_check.mock_calls == [
            call(["snap", "create-key", "test-key-1"])
        ]

    @pytest.mark.usefixtures("fake_get_usable_keys")
    def test_create_key_store_error(
        self, mock_subprocess_check, fake_store_client, fake_app_config
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

        mock_subprocess_check.assert_not_called()
