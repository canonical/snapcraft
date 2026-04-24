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

"""Tests for validation commands."""

import argparse
import datetime

import pytest

from snapcraft import commands, models, store
from snapcraft.services import Assertion as AssertionService


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
            models.ValidationAssertion(
                type="validation",
                authority_id="test-authority",
                series="16",
                snap_id="test-snap-id",
                approved_snap_id="test-snap-id-2",
                approved_snap_revision="42",
                approved_snap_name="test-snap",
                required=True,
                timestamp=timestamp,
                revoked=False,
            )
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
            models.ValidationAssertion(
                type="validation",
                authority_id="test-authority",
                series="16",
                snap_id="test-snap-id",
                approved_snap_id="test-snap-id-2",
                approved_snap_revision="-",
                approved_snap_name="test-snap",
                required=True,
                timestamp=timestamp,
                revoked=False,
            )
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


class TestValidateCommand:
    """Tests for the 'validate' command."""

    @pytest.fixture
    def mock_sign_assertion(self, mocker):
        return mocker.patch.object(
            AssertionService, "sign_assertion", return_value=b"signed-assertion"
        )

    @pytest.fixture
    def fake_store_client(self, mocker):
        mock_cls = mocker.patch("snapcraft.store.StoreClientCLI")
        mock_cls.return_value.get_account_info.return_value = {
            "account_id": "test-account-id",
            "snaps": {
                store.constants.DEFAULT_SERIES: {
                    "test-snap": {"snap-id": "test-snap-id"},
                }
            },
        }
        mock_cls.return_value.list_validations.return_value = []
        mock_cls.return_value.get_snap_info.return_value = {
            "snap-id": "approved-snap-id"
        }
        return mock_cls.return_value

    @pytest.mark.parametrize("key_name", [None, "test-key"])
    @pytest.mark.parametrize("revoke", [True, False])
    def test_validate(
        self,
        new_dir,
        fake_store_client,
        mock_sign_assertion,
        fake_app_config,
        key_name,
        revoke,
    ):
        """Successfully validate a snap."""
        cmd = commands.StoreValidateCommand(fake_app_config)

        cmd.run(
            argparse.Namespace(
                snap_name="test-snap",
                validations=["approved-snap=10"],
                key_name=key_name,
                revoke=revoke,
            )
        )

        assertion, actual_key_name = mock_sign_assertion.call_args[0]
        assert isinstance(assertion, models.ValidationAssertion)
        assert assertion.type == "validation"
        assert assertion.authority_id == "test-account-id"
        assert assertion.series == store.constants.DEFAULT_SERIES
        assert assertion.snap_id == "test-snap-id"
        assert assertion.approved_snap_id == "approved-snap-id"
        assert assertion.approved_snap_revision == "10"
        assert assertion.revoked == revoke
        assert assertion.revision is None
        assert actual_key_name is key_name
        fake_store_client.post_validation.assert_called_once_with(
            "test-snap-id", b"signed-assertion"
        )
        assert (
            new_dir / "project/test-snap-approved-snap-r10.assertion"
        ).read_bytes() == b"signed-assertion"

    def test_validate_increment_existing_revision(
        self,
        new_dir,
        fake_store_client,
        mock_sign_assertion,
        fake_app_config,
    ):
        """Increment the revision when one already exists."""
        fake_store_client.list_validations.return_value = [
            models.ValidationAssertion(
                type="validation",
                authority_id="test-account-id",
                series="16",
                snap_id="test-snap-id",
                approved_snap_id="approved-snap-id",
                approved_snap_revision="10",
                timestamp="2026-04-02T12:06:42Z",
                revoked=False,
                revision=3,
            )
        ]
        cmd = commands.StoreValidateCommand(fake_app_config)

        cmd.run(
            argparse.Namespace(
                snap_name="test-snap",
                validations=["approved-snap=10"],
                key_name=None,
                revoke=False,
            )
        )

        assertion, _ = mock_sign_assertion.call_args[0]
        assert assertion.revision == 4

    def test_validate_timestamp(
        self,
        new_dir,
        fake_store_client,
        mock_sign_assertion,
        fake_app_config,
        mocker,
    ):
        """Timestamps must be in the format expected by the store."""
        fake_datetime = datetime.datetime(
            2026, 4, 23, 20, 43, 10, 123456, tzinfo=datetime.timezone.utc
        )
        mocker.patch(
            "snapcraft.commands.validations.datetime.datetime"
        ).now.return_value = fake_datetime
        cmd = commands.StoreValidateCommand(fake_app_config)

        cmd.run(
            argparse.Namespace(
                snap_name="test-snap",
                validations=["approved-snap=10"],
                key_name=None,
                revoke=False,
            )
        )

        assertion, _ = mock_sign_assertion.call_args[0]
        assert assertion.timestamp == "2026-04-23T20:43:10.123456Z"

    @pytest.mark.parametrize(
        "validation",
        [
            pytest.param("test-snap=1", id="simple"),
            pytest.param("mysnap=100", id="multiple-digits"),
            pytest.param("a=0", id="minimal"),
            pytest.param("SNAP-NAME=999", id="uppercase"),
        ],
    )
    def test_validation_regex(self, fake_app_config, validation):
        """Test valid validation strings pass the regex check."""
        cmd = commands.StoreValidateCommand(fake_app_config)

        cmd._check_validations([validation])

    @pytest.mark.parametrize(
        "validation",
        [
            pytest.param("approved-snap=notanumber", id="non-numeric-revision"),
            pytest.param("=5", id="empty-name"),
            pytest.param("snap=", id="empty-revision"),
            pytest.param("snap", id="missing-equals"),
            pytest.param("snap=1.5", id="decimal-revision"),
            pytest.param("snap=-1", id="negative-revision"),
            pytest.param("snap=1=2", id="multiple-equals"),
            pytest.param("", id="empty-string"),
        ],
    )
    def test_validation_regex_error(self, fake_app_config, validation):
        """Error for invalid validation format."""
        cmd = commands.StoreValidateCommand(fake_app_config)

        with pytest.raises(store.errors.InvalidValidationRequestsError):
            cmd._check_validations([validation])

    def test_validate_snap_not_found(self, fake_store_client, fake_app_config):
        """Error when snap doesn't exist."""
        cmd = commands.StoreValidateCommand(fake_app_config)

        with pytest.raises(store.errors.SnapNotFoundError):
            cmd.run(
                argparse.Namespace(
                    snap_name="missing-snap",
                    validations=["approved-snap=10"],
                    key_name=None,
                    revoke=False,
                )
            )

    def test_validate_fallback_to_snap_id(
        self,
        new_dir,
        fake_store_client,
        mock_sign_assertion,
        fake_app_config,
    ):
        """Use the snap name when the snap can't be found."""
        fake_store_client.get_snap_info.side_effect = store.errors.SnapNotFoundError(
            snap_name="unknown-snap-id"
        )
        cmd = commands.StoreValidateCommand(fake_app_config)

        cmd.run(
            argparse.Namespace(
                snap_name="test-snap",
                validations=["unknown-snap-id=5"],
                key_name=None,
                revoke=False,
            )
        )

        assertion, _ = mock_sign_assertion.call_args[0]
        assert assertion.approved_snap_id == "unknown-snap-id"
