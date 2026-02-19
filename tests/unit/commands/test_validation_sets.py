# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022,2024 Canonical Ltd.
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
import sys

import pytest

from snapcraft import application, commands, errors


@pytest.fixture
def mock_edit_assertion(mocker):
    return mocker.patch(
        "snapcraft.services.validationsets.ValidationSets.edit_assertion"
    )


@pytest.fixture
def mock_list_assertions(mocker):
    return mocker.patch(
        "snapcraft.services.validationsets.ValidationSets.list_assertions"
    )


@pytest.mark.parametrize("key_name", [None, "test-key"])
@pytest.mark.usefixtures("memory_keyring")
def test_edit_validation_sets(key_name, mocker, mock_edit_assertion):
    """Test `snapcraft edit-validation-sets`."""
    cmd = ["snapcraft", "edit-validation-sets", "test-account-id", "test-name", "10"]
    if key_name:
        cmd.extend(["--key-name", key_name])
    mocker.patch.object(sys, "argv", cmd)

    app = application.create_app()
    app.run()

    mock_edit_assertion.assert_called_once_with(
        name="test-name",
        account_id="test-account-id",
        key_name=key_name,
        sequence=10,
    )


@pytest.mark.usefixtures("memory_keyring")
@pytest.mark.parametrize("name", [None, "test"])
@pytest.mark.parametrize("sequence", [None, "latest", "all"])
def test_list_validation_sets(capsys, mocker, name, sequence, mock_list_assertions):
    """Test `snapcraft validation-sets`."""
    cmd = ["snapcraft", "validation-sets"]
    if name:
        cmd.extend(["--name", name])
    if sequence:
        cmd.extend(["--sequence", sequence])
    mocker.patch.object(sys, "argv", cmd)
    kwargs = {"sequence": sequence} if sequence else {}

    app = application.create_app()
    app.run()

    mock_list_assertions.assert_called_once_with(
        name=name,
        output_format="table",
        **kwargs,
    )


def test_list_validation_sets_error(fake_app_config):
    """Error on removed 'list-validation-sets' command."""
    cmd = commands.StoreListValidationSetsCommand(fake_app_config)
    expected = re.escape(
        "The 'list-validation-sets' command was renamed to 'validation-sets'."
    )

    with pytest.raises(errors.RemovedCommand, match=expected):
        cmd.run(
            argparse.Namespace(
                account_id="test",
                set_name="cert1",
                sequence="9",
                key_name=None,
            )
        )
