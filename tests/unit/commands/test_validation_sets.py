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
import sys

import pytest

from snapcraft import cli, commands, const, store


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


@pytest.fixture
def fake_store_client(mocker):
    """Fake StoreClientCLI with a test snap in account info."""
    mock_cls = mocker.patch("snapcraft.store.StoreClientCLI")
    mock_cls.return_value.get_account_info.return_value = {
        "snaps": {
            store.constants.DEFAULT_SERIES: {
                "test-snap": {"snap-id": "test-snap-id"},
            }
        }
    }
    mock_cls.return_value.list_validations.return_value = []
    return mock_cls.return_value


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

        cli.run()

        out, err = capsys.readouterr()
        assert not out
        assert (
            "The 'list-validation-sets' command was renamed to 'validation-sets'.\n"
            "Recommended resolution: Use 'validation-sets' instead."
        ) in err


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


class TestGatedCommand:
    """Tests for the 'gated' command."""

    @pytest.mark.parametrize(
        "timestamp",
        [
            "2026-01-01T00:00:00Z",
            # microseconds are removed
            "2026-01-01T00:00:00.123456Z",
        ],
    )
    def test_gated(self, fake_store_client, fake_app_config, emitter, timestamp):
        """Gated snap with validations emits a table."""
        fake_store_client.list_validations.return_value = [
            {
                "approved-snap-name": "test-snap",
                "approved-snap-revision": "42",
                "required": True,
                "timestamp": timestamp,
            }
        ]

        cmd = commands.StoreGatedCommand(fake_app_config)

        cmd.run(argparse.Namespace(snap_name="test-snap"))

        fake_store_client.list_validations.assert_called_once_with("test-snap-id")
        emitter.assert_message(
            "Name         Revision  Required    Approved\n"
            "test-snap          42  True        2026-01-01T00:00:00Z"
        )

    @pytest.mark.parametrize(
        "timestamp",
        [
            "2026-01-01T00:00:00Z",
            # microseconds are removed
            "2026-01-01T00:00:00.123456Z",
        ],
    )
    def test_gated_no_revision(
        self, fake_store_client, fake_app_config, emitter, timestamp
    ):
        """Validate when there is no revision."""
        fake_store_client.list_validations.return_value = [
            {
                "approved-snap-name": "test-snap",
                "approved-snap-revision": "-",
                "required": True,
                "timestamp": timestamp,
            }
        ]
        cmd = commands.StoreGatedCommand(fake_app_config)

        cmd.run(argparse.Namespace(snap_name="test-snap"))

        emitter.assert_message(
            "Name       Revision    Required    Approved\n"
            "test-snap  -           True        2026-01-01T00:00:00Z"
        )

    def test_gated_no_validations(self, fake_store_client, fake_app_config, emitter):
        cmd = commands.StoreGatedCommand(fake_app_config)

        cmd.run(argparse.Namespace(snap_name="test-snap"))

        emitter.assert_message("There are no validations for snap 'test-snap'")

    def test_gated_snap_not_found(self, fake_store_client, fake_app_config):
        """Error when snap doesn't exist."""
        cmd = commands.StoreGatedCommand(fake_app_config)

        with pytest.raises(store.errors.SnapNotFoundError):
            cmd.run(argparse.Namespace(snap_name="missing-snap"))

        fake_store_client.list_validations.assert_not_called()
