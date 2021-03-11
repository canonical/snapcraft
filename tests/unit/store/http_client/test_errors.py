# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2021 Canonical Ltd
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

import requests
import urllib3
from unittest import mock

from snapcraft.storeapi.http_clients import errors


def _fake_error_response(status_code, reason):
    response = mock.Mock()
    response.status_code = status_code
    response.reason = reason
    return response


class TestSnapcraftException:
    scenarios = (
        (
            "InvalidCredentialsError",
            {
                "exception_class": errors.InvalidCredentialsError,
                "kwargs": {"message": "macaroon expired"},
                "expected_message": (
                    "Invalid credentials: macaroon expired. "
                    'Have you run "snapcraft login"?'
                ),
            },
        ),
        (
            "StoreAuthenticationError",
            {
                "exception_class": errors.StoreAuthenticationError,
                "kwargs": {"message": "invalid password"},
                "expected_message": ("Authentication error: invalid password"),
            },
        ),
        (
            "StoreNetworkError generic error",
            {
                "exception_class": errors.StoreNetworkError,
                "kwargs": {
                    "exception": requests.exceptions.ConnectionError("bad error")
                },
                "expected_message": "There seems to be a network error: bad error",
            },
        ),
        (
            "StoreNetworkError max retry error",
            {
                "exception_class": errors.StoreNetworkError,
                "kwargs": {
                    "exception": requests.exceptions.ConnectionError(
                        urllib3.exceptions.MaxRetryError(
                            pool="test-pool", url="test-url"
                        )
                    )
                },
                "expected_message": (
                    "There seems to be a network error: maximum retries exceeded "
                    "trying to reach the store.\n"
                    "Check your network connection, and check the store status at "
                    "https://status.snapcraft.io/"
                ),
            },
        ),
        (
            "StoreServerError 500",
            {
                "exception_class": errors.StoreServerError,
                "kwargs": {
                    "response": _fake_error_response(500, "internal server error")
                },
                "expected_message": (
                    "The Snap Store encountered an error while processing your "
                    "request: internal server error (code 500).\nThe operational "
                    "status of the Snap Store can be checked at "
                    "https://status.snapcraft.io/"
                ),
            },
        ),
        (
            "StoreServerError 501",
            {
                "exception_class": errors.StoreServerError,
                "kwargs": {"response": _fake_error_response(501, "not implemented")},
                "expected_message": (
                    "The Snap Store encountered an error while processing your "
                    "request: not implemented (code 501).\nThe operational "
                    "status of the Snap Store can be checked at "
                    "https://status.snapcraft.io/"
                ),
            },
        ),
    )

    def test_error_formatting(self, exception_class, expected_message, kwargs):
        assert str(exception_class(**kwargs)) == expected_message
