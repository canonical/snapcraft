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

import contextlib
import logging
import urllib3
from simplejson.scanner import JSONDecodeError

from snapcraft.internal.errors import SnapcraftError

logger = logging.getLogger(__name__)


_STORE_STATUS_URL = "https://status.snapcraft.io/"


# TODO: migrate to storeapi private exception to ready craft-store.
class HttpClientError(SnapcraftError):
    """Base class http client errors.

    :cvar fmt: A format string that daughter classes override
    """

    def __init__(self, **kwargs):
        with contextlib.suppress(KeyError, AttributeError):
            logger.debug("Store error response: {}".format(kwargs["response"].__dict__))
        super().__init__(**kwargs)


class StoreServerError(HttpClientError):

    fmt = "{what}: {error_text} (code {error_code}).\n{action}"

    def __init__(self, response):
        what = "The Snap Store encountered an error while processing your request"
        error_code = response.status_code
        error_text = response.reason
        action = "The operational status of the Snap Store can be checked at {}".format(
            _STORE_STATUS_URL
        )
        self.response = response

        super().__init__(
            response=response,
            what=what,
            error_text=error_text,
            error_code=error_code,
            action=action,
        )


class StoreNetworkError(HttpClientError):

    fmt = "There seems to be a network error: {message}"

    def __init__(self, exception):
        message = str(exception)
        with contextlib.suppress(IndexError):
            underlying_exception = exception.args[0]
            if isinstance(underlying_exception, urllib3.exceptions.MaxRetryError):
                message = (
                    "maximum retries exceeded trying to reach the store.\n"
                    "Check your network connection, and check the store "
                    "status at {}".format(_STORE_STATUS_URL)
                )
        super().__init__(message=message)


class InvalidCredentialsError(HttpClientError):

    fmt = 'Invalid credentials: {message}. Have you run "snapcraft login"?'

    def __init__(self, message):
        super().__init__(message=message)


class StoreAuthenticationError(HttpClientError):

    fmt = "Authentication error: {message}"

    def __init__(self, message, response=None):
        # Unfortunately the store doesn't give us a consistent error response,
        # so we'll check the ones of which we're aware.
        with contextlib.suppress(AttributeError, JSONDecodeError):
            response_json = response.json()
            extra_error_message = ""
            if "error_message" in response_json:
                extra_error_message = response_json["error_message"]
            elif "message" in response_json:
                extra_error_message = response_json["message"]

            if extra_error_message:
                message += ": {}".format(extra_error_message)

        super().__init__(response=response, message=message)


class StoreTwoFactorAuthenticationRequired(StoreAuthenticationError):
    def __init__(self):
        super().__init__("Two-factor authentication required.")


class InvalidLoginConfig(HttpClientError):

    fmt = "Invalid login config: {error}"

    def __init__(self, error):
        super().__init__(error=error)
