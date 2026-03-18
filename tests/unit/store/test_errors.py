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

import json
import re

import pytest

from snapcraft.store.errors import NoSnapIdError, SnapNotFoundError, StoreMetadataError

from .utils import FakeResponse


class TestStoreMetadataError:
    """Tests for StoreMetadataError."""

    def test_404(self):
        response = FakeResponse(status_code=404, content=json.dumps({}).encode())

        err = StoreMetadataError("test-snap", response, {})

        assert str(err) == (
            "Sorry, updating the information in the store has failed, first run "
            "`snapcraft register test-snap` and then "
            "`snapcraft upload <snap-file>`."
        )

    def test_409(self):
        response = FakeResponse(
            status_code=409,
            content=json.dumps(
                {
                    "error_list": [
                        {
                            "code": "conflict",
                            "extra": {"name": "summary", "current": "store summary"},
                        }
                    ]
                }
            ).encode(),
        )

        err = StoreMetadataError("test-snap", response, {"summary": "local summary"})

        assert str(err) == "Metadata not uploaded!"
        assert err.details == (
            "Conflict in 'summary' field:\n"
            "    In snapcraft.yaml: 'local summary'\n"
            "    In the Store:      'store summary'"
        )
        assert err.resolution == (
            "You can repeat the upload-metadata command with "
            "--force to force the local values into the Store"
        )

    def test_409_multiple_conflicts(self):
        response = FakeResponse(
            status_code=409,
            content=json.dumps(
                {
                    "error_list": [
                        {
                            "code": "conflict",
                            "extra": {"name": "summary", "current": "store summary"},
                        },
                        {
                            "code": "conflict",
                            "extra": {
                                "name": "description",
                                "current": "store description",
                            },
                        },
                    ]
                }
            ).encode(),
        )

        err = StoreMetadataError(
            "test-snap",
            response,
            {"summary": "local summary", "description": "local description"},
        )

        assert str(err) == "Metadata not uploaded!"
        assert err.details == (
            "Conflict in 'description' field:\n"
            "    In snapcraft.yaml: 'local description'\n"
            "    In the Store:      'store description'\n"
            "Conflict in 'summary' field:\n"
            "    In snapcraft.yaml: 'local summary'\n"
            "    In the Store:      'store summary'"
        )
        assert err.resolution == (
            "You can repeat the upload-metadata command with "
            "--force to force the local values into the Store"
        )

    def test_409_no_conflicts(self):
        """409 with no conflict entries produces no details."""
        response = FakeResponse(
            status_code=409,
            content=json.dumps({"error_list": []}).encode(),
        )

        err = StoreMetadataError("test-snap", response, {})

        assert str(err) == "Metadata not uploaded!"
        assert err.details is None
        assert err.resolution == (
            "You can repeat the upload-metadata command with "
            "--force to force the local values into the Store"
        )

    def test_error_list(self):
        """Parse 'error_list' if it's in the response."""
        response = FakeResponse(
            status_code=400,
            content=json.dumps(
                {"error_list": [{"message": "Invalid field: unknown"}]}
            ).encode(),
        )

        err = StoreMetadataError("test-snap", response, {})

        assert str(err) == "Invalid field: unknown"

    def test_generic_error(self):
        response = FakeResponse(
            status_code=500, content=json.dumps({"text": "internal error"}).encode()
        )

        err = StoreMetadataError("test-snap", response, {})

        assert str(err) == "Store error 500: 'internal error'"

    def test_invalid_json_response(self):
        """Catch and re-raise invalid json errors."""
        response = FakeResponse(status_code=500, content=b"not json")

        # should not raise
        err = StoreMetadataError("test-snap", response, {})

        assert str(err) == "Store error 500: ''"


class TestSnapNotFoundError:
    """Tests for SnapNotFoundError."""

    def test_missing_snap_name_and_id(self):
        with pytest.raises(
            RuntimeError,
            match=re.escape("Both 'snap_name' and 'snap_id' cannot be None."),
        ):
            SnapNotFoundError()

    def test_snap_name_only(self):
        err = SnapNotFoundError(snap_name="test-snap")

        assert str(err) == "Snap 'test-snap' was not found."
        assert err.resolution == "Ensure you have proper access rights for 'test-snap'."

    def test_snap_id(self):
        err = SnapNotFoundError(snap_id="test-id")

        assert str(err) == "Cannot find snap with snap_id 'test-id'."
        assert err.resolution == "Ensure you have proper access rights for 'test-id'."

    def test_channel_only(self):
        err = SnapNotFoundError(snap_name="test-snap", channel="latest/stable")

        assert str(err) == "Snap 'test-snap' was not found on channel 'latest/stable'."
        assert err.resolution == (
            "Ensure you have proper access rights for 'test-snap'.\n"
            "Also ensure the correct channel was used."
        )

    def test_arch_only(self):
        err = SnapNotFoundError(snap_name="test-snap", arch="riscv64")

        assert str(err) == "Snap 'test-snap' for architecture 'riscv64' was not found."
        assert err.resolution == (
            "Ensure you have proper access rights for 'test-snap'.\n"
            "Also ensure the correct architecture was used."
        )

    def test_channel_and_arch(self):
        err = SnapNotFoundError(
            snap_name="test-snap", channel="latest/stable", arch="riscv64"
        )

        assert str(err) == (
            "Snap 'test-snap' for architecture 'riscv64' was "
            "not found on channel 'latest/stable'."
        )
        assert err.resolution == (
            "Ensure you have proper access rights for 'test-snap'.\n"
            "Also ensure the correct channel and architecture was used."
        )


class TestNoSnapIdError:
    """Tests for NoSnapIdError."""

    def test_message(self):
        err = NoSnapIdError("test-snap")

        assert str(err) == (
            "Failed to get snap ID for snap 'test-snap'. This is an error in the store."
        )
        assert err.resolution == (
            "Please open a new topic in the 'store' category in "
            "the forum: https://forum.snapcraft.io/c/store"
        )
