# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022,2024,2026 Canonical Ltd.
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

"""Tests for validation sets commands."""

import argparse
import os
import sys

import pytest

from snapcraft import application, commands, const


@pytest.fixture
def mock_list_assertions(mocker):
    return mocker.patch(
        "snapcraft.services.validationsets.ValidationSets.list_assertions"
    )


@pytest.fixture
def mock_edit_assertion(mocker):
    return mocker.patch(
        "snapcraft.services.validationsets.ValidationSets.edit_assertion"
    )


class TestValidationSetsCommand:
    """Tests for the 'validation-sets' command."""

    @pytest.mark.parametrize("sequence", [None, "latest", "all"])
    @pytest.mark.parametrize("name", [None, "test-vs"])
    def test_validation_sets(
        self, mock_list_assertions, fake_app_config, name, sequence
    ):
        kwargs = {"sequence": sequence} if sequence else {}
        cmd = commands.StoreValidationSetsCommand(fake_app_config)

        cmd.run(argparse.Namespace(name=name, sequence=sequence, format="table"))

        mock_list_assertions.assert_called_once_with(
            name=name, output_format="table", **kwargs
        )

    @pytest.mark.parametrize("output_format", const.OUTPUT_FORMATS)
    def test_validation_sets_output_format(
        self, mock_list_assertions, fake_app_config, output_format
    ):
        cmd = commands.StoreValidationSetsCommand(fake_app_config)

        cmd.run(argparse.Namespace(name=None, sequence=None, format=output_format))

        mock_list_assertions.assert_called_once_with(
            name=None, output_format=output_format
        )

    def test_list_validation_sets_error(self, mocker, capsys):
        """Error on removed 'list-validation-sets' command."""
        mocker.patch.object(sys, "argv", ["cmd", "list-validation-sets"])

        app = application.create_app()
        return_code = app.run()

        out, err = capsys.readouterr()
        assert not out
        assert (
            "The 'list-validation-sets' command was renamed to 'validation-sets'.\n"
            "Recommended resolution: Use 'validation-sets' instead."
        ) in err
        assert return_code == os.EX_USAGE


class TestEditValidationSetsCommand:
    """Tests for the 'edit-validation-sets' command."""

    @pytest.mark.parametrize("key_name", [None, "test-key"])
    def test_edit_validation_sets(self, mock_edit_assertion, fake_app_config, key_name):
        cmd = commands.StoreEditValidationSetsCommand(fake_app_config)

        cmd.run(
            argparse.Namespace(
                account_id="test-account-id",
                name="test-name",
                sequence=10,
                key_name=key_name,
            )
        )

        mock_edit_assertion.assert_called_once_with(
            name="test-name",
            account_id="test-account-id",
            key_name=key_name,
            sequence=10,
        )
