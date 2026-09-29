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

import hashlib
import json
from unittest.mock import ANY, Mock, call

import craft_store
import pytest

from snapcraft.store import constants
from snapcraft.store._metadata import StoreMetadataHandler
from snapcraft.store.errors import StoreMetadataError

from .utils import FakeResponse


@pytest.fixture
def mock_request():
    return Mock()


@pytest.fixture
def handler(mock_request):
    return StoreMetadataHandler(
        base_url=constants.STORE_URL,
        request_method=mock_request,
        snap_id="test-snap-id",
        snap_name="test-snap",
    )


class TestUpload:
    """Tests for upload()."""

    @pytest.mark.parametrize(
        ("force", "method"),
        [
            (False, "POST"),
            (True, "PUT"),
        ],
    )
    def test_upload(self, handler, mock_request, force, method):
        metadata = {"summary": "test summary", "description": "test description"}

        handler.upload(metadata, force=force)

        assert mock_request.mock_calls == [
            call(
                method,
                "https://dashboard.snapcraft.io/dev/api/snaps/test-snap-id/metadata",
                json=metadata,
                headers={
                    "Content-Type": "application/json",
                    "Accept": "application/json",
                },
            )
        ]

    def test_upload_error(self, handler, mock_request):
        """Raise StoreMetadataError on server error."""
        mock_request.side_effect = craft_store.errors.StoreServerError(
            FakeResponse(
                status_code=400,
                content=json.dumps({"error_list": [{"message": "bad field"}]}).encode(),
            )
        )

        with pytest.raises(StoreMetadataError):
            handler.upload({"summary": "test summary"}, force=False)

    def test_upload_conflict_error(self, handler, mock_request):
        """Raise StoreMetadataError with conflict details on 409."""
        mock_request.side_effect = craft_store.errors.StoreServerError(
            FakeResponse(
                status_code=409,
                content=json.dumps(
                    {
                        "error_list": [
                            {
                                "code": "conflict",
                                "extra": {
                                    "name": "summary",
                                    "current": "store summary",
                                },
                            }
                        ]
                    }
                ).encode(),
            )
        )

        with pytest.raises(StoreMetadataError) as raised:
            handler.upload({"summary": "local summary"}, force=False)

        assert "Conflict in 'summary' field" in str(raised.value.details)
        assert "In snapcraft.yaml: 'local summary'" in str(raised.value.details)
        assert "In the Store:      'store summary'" in str(raised.value.details)


class TestUploadBinary:
    """Tests for upload_binary()."""

    @pytest.mark.parametrize(
        ("force", "method"),
        [
            (False, "POST"),
            (True, "PUT"),
        ],
    )
    def test_upload_binary(self, handler, mock_request, tmp_path, force, method):
        icon = tmp_path / "icon.png"
        icon.write_bytes(b"fake-icon-data")

        mock_request.return_value = FakeResponse(
            status_code=200, content=json.dumps([]).encode()
        )

        handler.upload_binary({"icon": icon.open("rb")}, force=force)

        assert mock_request.mock_calls == [
            call(
                "GET",
                "https://dashboard.snapcraft.io/dev/api/snaps/test-snap-id/binary-metadata",
                headers={"Accept": "application/json"},
            ),
            call(
                method,
                "https://dashboard.snapcraft.io/dev/api/snaps/test-snap-id/binary-metadata",
                data={"info": ANY},
                files={"icon": ANY},
                headers={"Accept": "application/json"},
            ),
        ]
        assert json.loads(mock_request.mock_calls[1].kwargs["data"]["info"]) == [
            {"type": "icon", "hash": ANY, "key": "icon", "filename": str(icon)}
        ]
        assert mock_request.mock_calls[1].kwargs["files"]["icon"].name == str(icon)

    def test_upload_binary_no_icon_no_request(self, handler, mock_request):
        """No upload request when there is no existing or new icon."""
        mock_request.return_value = FakeResponse(
            status_code=200, content=json.dumps([]).encode()
        )

        handler.upload_binary({}, force=False)

        # only the GET for current binary metadata, no upload
        assert len(mock_request.mock_calls) == 1
        assert mock_request.mock_calls[0].args[0] == "GET"

    def test_upload_binary_unchanged_icon_no_request(
        self, handler, mock_request, tmp_path
    ):
        """No upload request when the icon matches what is in the store."""
        icon = tmp_path / "icon.png"
        icon.write_bytes(b"fake-icon-data")
        icon_hash = hashlib.sha256(b"fake-icon-data").hexdigest()

        mock_request.return_value = FakeResponse(
            status_code=200,
            content=json.dumps([{"type": "icon", "hash": icon_hash}]).encode(),
        )

        handler.upload_binary({"icon": icon.open("rb")}, force=False)

        assert len(mock_request.mock_calls) == 1
        assert mock_request.mock_calls[0].args[0] == "GET"

    def test_upload_binary_error(self, handler, mock_request, tmp_path):
        """Raise StoreMetadataError on upload server error."""
        icon = tmp_path / "icon.png"
        icon.write_bytes(b"fake-icon-data")

        mock_request.side_effect = [
            FakeResponse(status_code=200, content=json.dumps([]).encode()),
            craft_store.errors.StoreServerError(
                FakeResponse(
                    status_code=400,
                    content=json.dumps(
                        {"error_list": [{"message": "Invalid field: icon"}]}
                    ).encode(),
                )
            ),
        ]

        with pytest.raises(StoreMetadataError):
            handler.upload_binary({"icon": icon.open("rb")}, force=False)

    def test_current_binary_metadata_error(self, handler, mock_request):
        """Raise StoreMetadataError if the GET for current metadata fails."""
        mock_request.return_value = FakeResponse(
            status_code=500, content=json.dumps({}).encode()
        )

        with pytest.raises(StoreMetadataError):
            handler.upload_binary({}, force=False)

    @pytest.mark.parametrize(
        "screenshots", [[], [{"type": "screenshot", "hash": "0badcode"}]]
    )
    def test_upload_binary_remove_existing_icon(
        self, handler, mock_request, screenshots
    ):
        """Remove the existing store icon and keep screenshots."""
        icon = [{"type": "icon", "hash": "deadbeef"}]
        mock_request.return_value = FakeResponse(
            status_code=200,
            content=json.dumps([*icon, *screenshots]).encode(),
        )

        handler.upload_binary({"icon": None}, force=False)

        assert len(mock_request.mock_calls) == 2
        post_call = mock_request.mock_calls[1]
        assert post_call.args[0] == "POST"
        assert (
            post_call.args[1]
            == "https://dashboard.snapcraft.io/dev/api/snaps/test-snap-id/binary-metadata"
        )
        _, info_body = post_call.kwargs["files"]["info"]
        assert json.loads(info_body) == [*screenshots]
