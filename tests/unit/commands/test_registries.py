# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2024 Canonical Ltd.
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

"""Unit tests for registries commands."""

import sys
from argparse import Namespace

import pytest

from snapcraft import application, commands, const


@pytest.fixture
def mock_list_assertions(mocker):
    return mocker.patch("snapcraft.services.registries.Registries.list_assertions")


@pytest.fixture
def mock_edit_assertion(mocker):
    return mocker.patch("snapcraft.services.registries.Registries.edit_assertion")


@pytest.mark.usefixtures("memory_keyring")
@pytest.mark.parametrize("output_format", const.OUTPUT_FORMATS)
@pytest.mark.parametrize("name", [None, "test"])
def test_list_registries(mock_list_assertions, output_format, name):
    """Test `snapcraft list-registries`."""
    app = application.create_app()
    cmd = commands.StoreListRegistriesCommand(app.app_config)

    cmd.run(Namespace(format=output_format, name=name))

    mock_list_assertions.assert_called_once_with(name=name, output_format=output_format)


@pytest.mark.skip("Needs the 'list-confdb' command exposed (#5183).")
@pytest.mark.usefixtures("memory_keyring")
@pytest.mark.parametrize("name", [None, "test"])
def test_list_registries_default_format(mocker, mock_list_assertions, name):
    """Default format is 'table'."""
    cmd = ["snapcraft", "list-registries"]
    if name:
        cmd.extend(["--name", name])
    mocker.patch.object(sys, "argv", cmd)

    app = application.create_app()
    app.run()

    mock_list_assertions.assert_called_once_with(name=name, output_format="table")


@pytest.mark.parametrize("key_name", [None, "test-key"])
@pytest.mark.usefixtures("memory_keyring")
def test_edit_registries(key_name, mocker, mock_edit_assertion):
    """Test `snapcraft edit-registries`."""
    cmd = ["snapcraft", "edit-registries", "test-account-id", "test-name"]
    if key_name:
        cmd.extend(["--key-name", key_name])
    mocker.patch.object(sys, "argv", cmd)

    app = application.create_app()

    cmd = commands.StoreEditRegistriesCommand(app.app_config)
    cmd.run(
        Namespace(account_id="test-account-id", name="test-name", key_name=key_name)
    )

    mock_edit_assertion.assert_called_once_with(
        name="test-name", account_id="test-account-id", key_name=key_name
    )
