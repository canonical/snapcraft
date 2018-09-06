# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2016-2017 Canonical Ltd
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
from http.client import responses
import logging
from requests.packages import urllib3
from simplejson.scanner import JSONDecodeError
from typing import List  # noqa

from snapcraft.internal.errors import SnapcraftError
from snapcraft import formatting_utils


logger = logging.getLogger(__name__)

_STORE_STATUS_URL = "https://status.snapcraft.io/"
_FORUM_URL = "https://forum.snapcraft.io/c/store"


class StoreError(SnapcraftError):
    """Base class for all storeapi exceptions.

    :cvar fmt: A format string that daughter classes override

    """

    def __init__(self, **kwargs):
        with contextlib.suppress(KeyError, AttributeError):
            logger.debug("Store error response: {}".format(kwargs["response"].__dict__))
        super().__init__(**kwargs)


class InvalidCredentialsError(StoreError):

    fmt = "Invalid credentials: {message}. " 'Have you run "snapcraft login"?'

    def __init__(self, message):
        super().__init__(message=message)


class LoginRequiredError(StoreError):

    fmt = "{message}"

    def __init__(self, extra_information=""):
        message = "Cannot continue without logging in successfully"
        if extra_information:
            message += ": {}".format(extra_information)
        super().__init__(message=message)


class StoreNetworkError(StoreError):

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


class SnapNotFoundError(StoreError):

    __FMT_ARCH_CHANNEL = (
        "Snap {name!r} for {arch!r} cannot be found in the {channel!r} channel."
    )
    __FMT_CHANNEL = "Snap {name!r} was not found in the {channel!r} channel."
    __FMT_SERIES_ARCH = "Snap {name!r} for {arch!r} was not found in {series!r} series."
    __FMT_SERIES = "Snap {name!r} was not found in {series!r} series."

    fmt = "Snap {name!r} was not found."

    def __init__(self, name, channel=None, arch=None, series=None):
        if channel and arch:
            self.fmt = self.__FMT_ARCH_CHANNEL
        elif channel:
            self.fmt = self.__FMT_CHANNEL
        elif series and arch:
            self.fmt = self.__FMT_SERIES_ARCH
        elif series:
            self.fmt = self.__FMT_SERIES

        super().__init__(name=name, channel=channel, arch=arch, series=series)


class NoSnapIdError(StoreError):

    fmt = (
        "Failed to get snap ID for snap {snap_name!r}. This is an error in "
        "the store, please open a new topic in the 'store' category in the "
        "forum: {forum_url}"
    )

    def __init__(self, snap_name):
        super().__init__(snap_name=snap_name, forum_url=_FORUM_URL)


class SHAMismatchError(StoreError):

    fmt = "SHA512 checksum for {path} is not {expected_sha}."

    def __init__(self, path, expected_sha):
        super().__init__(path=path, expected_sha=expected_sha)


class StoreAuthenticationError(StoreError):

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


class StoreMacaroonNeedsRefreshError(StoreError):

    fmt = "Authentication macaroon needs to be refreshed."


class DeveloperAgreementSignError(StoreError):

    fmt = (
        "There was an error while signing developer agreement.\n"
        "Reason: {reason!r}\n"
        "Text: {text!r}"
    )

    def __init__(self, response):
        super().__init__(response=response, reason=response.reason, text=response.text)


class NeedTermsSignedError(StoreError):

    fmt = (
        "Developer Terms of Service agreement must be signed "
        "before continuing: {message}"
    )

    def __init__(self, message):
        super().__init__(message=message)


class StoreAccountInformationError(StoreError):

    fmt = "Error fetching account information from store: {error}"

    def __init__(self, response):
        error = "{} {}".format(response.status_code, response.reason)
        extra = []
        try:
            response_json = response.json()
            if "error_list" in response_json:
                error = " ".join(
                    error["message"] for error in response_json["error_list"]
                )
                extra = [
                    error["extra"]
                    for error in response_json["error_list"]
                    if "extra" in error
                ]
        except JSONDecodeError:
            pass
        super().__init__(response=response, error=error, extra=extra)


class StoreKeyRegistrationError(StoreError):

    fmt = "Key registration failed: {error}"

    def __init__(self, response):
        error = "{} {}".format(response.status_code, response.reason)
        try:
            response_json = response.json()
            if "error_list" in response_json:
                error = " ".join(
                    error["message"] for error in response_json["error_list"]
                )
        except JSONDecodeError:
            pass
        super().__init__(response=response, error=error)


class StoreRegistrationError(StoreError):
    """Captures store name registration errors.

    Overrides `SnapcraftError` setup and string representation to
    support new-style (multiple) Store error responses.

    See https://myapps.developer.ubuntu.com/docs/api/snap.html#errors
    """

    __FMT_ALREADY_REGISTERED = (
        "The name {snap_name!r} is already taken.\n\n"
        "We can if needed rename snaps to ensure they match the expectations "
        "of most users. If you are the publisher most users expect for "
        "{snap_name!r} then claim the name at {register_name_url!r}"
    )

    __FMT_ALREADY_OWNED = "You already own the name {snap_name!r}."

    __FMT_RESERVED = (
        "The name {snap_name!r} is reserved.\n\n"
        "If you are the publisher most users expect for "
        "{snap_name!r} then please claim the name at {register_name_url!r}\n\n"
        "Otherwise, please register another name."
    )

    __FMT_RETRY_WAIT = (
        "You must wait {retry_after} seconds before trying to register "
        "your next snap."
    )

    __FMT_INVALID = "{message}"

    fmt = "Registration failed."

    __error_messages = {
        "already_registered": __FMT_ALREADY_REGISTERED,
        "already_owned": __FMT_ALREADY_OWNED,
        "reserved_name": __FMT_RESERVED,
        "register_window": __FMT_RETRY_WAIT,
        "invalid": __FMT_INVALID,
    }

    def __init__(self, snap_name, response):
        self._errors = []

        try:
            response_json = response.json()
        except JSONDecodeError:
            response_json = {}

        # Annotate 'snap_name' to be used when formatting errors.
        response_json["snap_name"] = snap_name

        # Extract the new-format error structure.
        error_list = response_json.pop("error_list", None)

        # Cope with legacy errors (missing 'error_list', single error and
        # top-level 'code').
        if error_list is None:
            error_code = response_json.get("code")
            # we default to self.fmt in case error_code is not mapped yet.
            fmt = self.__error_messages.get(error_code, self.fmt)
            self._errors.append(fmt.format(**response_json))
            return

        # Support new style formatted errors.
        for error in error_list:
            fmt = self.__error_messages.get(error["code"], self.fmt)
            # Augment 'error' with remaining top-level keys. Should be a
            # no-op once they are moved to their specific error record.
            error.update(response_json)
            self._errors.append(fmt.format(**error))

    def __str__(self):
        """Simply join formatted error as lines."""
        return "\n".join(self._errors)


class StoreUploadError(StoreError):

    fmt = (
        "There was an error uploading the package.\n"
        "Reason: {reason!r}\n"
        "Text: {text!r}"
    )

    def __init__(self, response):
        super().__init__(response=response, reason=response.reason, text=response.text)


class StorePushError(StoreError):

    __FMT_NOT_REGISTERED = (
        "You are not the publisher or allowed to push revisions for this "
        "snap. To become the publisher, run `snapcraft register {snap_name}` "
        "and try to push again."
    )

    fmt = "Received {status_code!r}: {text!r}"

    def __init__(self, snap_name, response):
        try:
            response_json = response.json()
        except (AttributeError, JSONDecodeError):
            response_json = {}

        if response.status_code == 404:
            self.fmt = self.__FMT_NOT_REGISTERED
        elif response.status_code == 401 or response.status_code == 403:
            try:
                response_json["text"] = response.text
            except AttributeError:
                response_json["text"] = "error while pushing"

        super().__init__(
            response=response,
            snap_name=snap_name,
            status_code=response.status_code,
            **response_json
        )


class StoreReviewError(StoreError):

    __FMT_NEED_MANUAL_REVIEW = (
        "The Store automatic review failed.\n"
        "A human will soon review your snap, but if you can't wait please "
        "write in the snapcraft forum asking for the manual review "
        "explicitly.\n"
        "If you need to disable confinement, please consider using devmode, "
        "but note that devmode revision will only be allowed to be released "
        "in edge and beta channels.\n"
        "Please check the errors and some hints below:"
    )

    __FMT_PROCESSING_ERROR = "The store was unable to accept this snap."

    __FMT_PROCESSING_UPLOAD_DELTA_ERROR = (
        "There has been a problem while processing a snap delta."
    )

    __messages = {
        "need_manual_review": __FMT_NEED_MANUAL_REVIEW,
        "processing_error": __FMT_PROCESSING_ERROR,
        "processing_upload_delta_error": __FMT_PROCESSING_UPLOAD_DELTA_ERROR,
    }

    def __init__(self, result):
        self.fmt = self.__messages[result["code"]]
        additional = []
        errors = result.get("errors")
        if errors:
            for error in errors:
                message = error.get("message")
                if message:
                    additional.append("  - {message}".format(message=message))
        if additional:
            self.additional = "\n".join(additional)
            self.fmt += "\n{additional}"
        self.code = result["code"]
        super().__init__()


class StoreServerError(StoreError):

    fmt = "{what}: {error_text} (code {error_code}).\n{action}"

    def __init__(self, response):
        what = "The Snap Store encountered an error while processing your request"
        error_code = response.status_code
        error_text = responses[error_code].lower()
        action = (
            "The operational status of the Snap Store can be checked at "
            "{}".format(_STORE_STATUS_URL)
        )

        super().__init__(
            response=response,
            what=what,
            error_text=error_text,
            error_code=error_code,
            action=action,
        )


class StoreReleaseError(StoreError):

    fmt = "{message}"

    __FMT_NOT_REGISTERED = (
        "Sorry, try `snapcraft register {snap_name}` before trying to "
        "release or choose an existing revision."
    )

    __FMT_BAD_REQUEST = "{code}: {message}\n"

    __FMT_UNAUTHORIZED_OR_FORBIDDEN = "Received {status_code!r}: {text}"

    def __init__(self, snap_name, response):
        self.fmt_errors = {
            400: self.__fmt_error_400,
            401: self.__fmt_error_401_or_403,
            403: self.__fmt_error_401_or_403,
            404: self.__fmt_error_404,
        }

        fmt_error = self.fmt_errors.get(response.status_code, self.__fmt_error_unknown)

        super().__init__(
            response=response, message=fmt_error(response).format(snap_name=snap_name)
        )

    def __to_json(self, response):
        try:
            response_json = response.json()
        except (AttributeError, JSONDecodeError):
            response_json = {}

        return response_json

    def __fmt_error_400(self, response):
        response_json = self.__to_json(response)
        try:
            fmt = ""
            for error in response_json["error_list"]:
                # Escape curly braces to avoid formatting errors.
                message = error["message"].replace("{", "{{").replace("}", "}}")
                code = error["code"].replace("{", "{{").replace("}", "}}")
                fmt += self.__FMT_BAD_REQUEST.format(code=code, message=message)
            # Strip the last new line from the error message
            fmt = fmt.rstrip("\n")

        except (AttributeError, KeyError):
            fmt = self.__fmt_error_unknown(response)
        return fmt

    def __fmt_error_401_or_403(self, response):
        try:
            text = response.text

            with contextlib.suppress(AttributeError, JSONDecodeError):
                response_json = response.json()
                if "error_list" in response_json:
                    text = _error_list_to_message(response_json)

        except AttributeError:
            text = "error while releasing"

        return self.__FMT_UNAUTHORIZED_OR_FORBIDDEN.format(
            status_code=response.status_code, text=text
        )

    def __fmt_error_404(self, response):
        return self.__FMT_NOT_REGISTERED

    def __fmt_error_unknown(self, response):
        response_json = self.__to_json(response)

        try:
            fmt = "{errors}".format(**response_json)

        except AttributeError:
            fmt = "{}".format(response)

        return fmt


class StoreMetadataError(StoreError):

    __FMT_NOT_FOUND = (
        "Sorry, updating the information on the store has failed, first run "
        "`snapcraft register {snap_name}` and then "
        "`snapcraft push <snap-file>`."
    )

    fmt = "Received {status_code!r}: {text!r}"

    def __init__(self, snap_name, response, metadata):
        try:
            response_json = response.json()
        except (AttributeError, JSONDecodeError):
            response_json = {}

        if response.status_code == 404:
            self.fmt = self.__FMT_NOT_FOUND
        elif response.status_code == 409:
            conflicts = [
                (error["extra"]["name"], error)
                for error in response_json["error_list"]
                if error["code"] == "conflict"
            ]
            parts = ["Metadata not pushed!"]
            for field_name, error in sorted(conflicts):
                sent = metadata.get(field_name)
                parts.extend(
                    (
                        "Conflict in {!r} field:".format(field_name),
                        "    In snapcraft.yaml: {!r}".format(sent),
                        "    In the Store:      {!r}".format(error["message"]),
                    )
                )
            parts.append(
                "You can repeat the push-metadata command with "
                "--force to force the local values into the Store"
            )
            self.parts = "\n".join(parts)
            self.fmt = "{parts}"
        elif "error_list" in response_json:
            response_json["text"] = response_json["error_list"][0]["message"]

        super().__init__(
            response=response,
            snap_name=snap_name,
            status_code=response.status_code,
            **response_json
        )


class StoreValidationError(StoreError):

    fmt = "Received error {status_code!r}: {text!r}"

    def __init__(self, snap_id, response, message=None):
        try:
            response_json = response.json()
            error = response.json()["error_list"][0]
            response_json["text"] = error.get("message")
            response_json["extra"] = error.get("extra")
        except (AttributeError, JSONDecodeError):
            response_json = {"text": message or response}

        super().__init__(
            response=response, status_code=response.status_code, **response_json
        )


class StoreSnapBuildError(StoreError):

    fmt = "Could not assert build: {error}"

    def __init__(self, response):
        error = "{} {}".format(response.status_code, response.reason)
        try:
            response_json = response.json()
            if "error_list" in response_json:
                error = " ".join(
                    error["message"] for error in response_json["error_list"]
                )
        except JSONDecodeError:
            pass

        super().__init__(response=response, error=error)


class StoreSnapRevisionsError(StoreError):

    fmt = (
        "Error fetching revisions of snap id {snap_id!r} for {arch!r} "
        "in {series!r} series: {error}."
    )

    def __init__(self, response, snap_id, series, arch):
        error = "{} {}".format(response.status_code, response.reason)
        try:
            response_json = response.json()
            if "error_list" in response_json:
                error = " ".join(
                    error["message"] for error in response_json["error_list"]
                )
        except JSONDecodeError:
            pass

        super().__init__(
            response=response,
            snap_id=snap_id,
            arch=arch or "any arch",
            series=series or "any",
            error=error,
        )


class StoreDeltaApplicationError(StoreError):

    fmt = "{message}"

    def __init__(self, message):
        super().__init__(message=message)


class StoreSnapStatusError(StoreSnapRevisionsError):

    fmt = (
        "Error fetching status of snap id {snap_id!r} for {arch!r} "
        "in {series!r} series: {error}."
    )


class StoreChannelClosingError(StoreError):

    fmt = "Could not close channel: {error}"

    def __init__(self, response):
        try:
            e = response.json()["error_list"][0]
            error = "{}".format(e["message"])
        except (JSONDecodeError, KeyError, IndexError):
            error = "{} {}".format(response.status_code, response.reason)

        super().__init__(response=response, error=error)


class StoreChannelClosingPermissionError(StoreError):

    fmt = (
        "Your account lacks permission to close channels for this snap. Make "
        "sure the logged in account has upload permissions on {snap_name!r} "
        "in series {snap_series!r}."
    )

    def __init__(self, snap_name, snap_series):
        super().__init__(snap_name=snap_name, snap_series=snap_series)


class StoreBuildAssertionPermissionError(StoreError):

    fmt = (
        "Your account lacks permission to assert builds for this snap. Make "
        "sure you are logged in as the publisher of {snap_name!r} "
        "for series {snap_series!r}."
    )

    def __init__(self, snap_name, snap_series):
        super().__init__(snap_name=snap_name, snap_series=snap_series)


class StoreAssertionError(StoreError):

    fmt = "Error signing {endpoint} assertion for {snap_name}: {error!s}"


class MissingSnapdError(StoreError):

    fmt = (
        "The snapd package is not installed. In order to use {command!r}, "
        "you must run 'apt install snapd'."
    )

    def __init__(self, command):
        super().__init__(command=command)


class KeyAlreadyRegisteredError(StoreError):

    fmt = "You have already registered a key named {key_name!r}"

    def __init__(self, key_name):
        super().__init__(key_name=key_name)


class NoKeysError(StoreError):

    fmt = (
        "You have no usable keys.\nPlease create at least one key with "
        "`snapcraft create-key` for use with snap."
    )


class NoSuchKeyError(StoreError):

    fmt = (
        "You have no usable key named {key_name!r}.\nSee the keys available "
        "in your system with `snapcraft keys`."
    )

    def __init__(self, key_name):
        super().__init__(key_name=key_name)


class KeyNotRegisteredError(StoreError):

    fmt = (
        "The key {key_name!r} is not registered in the Store.\nPlease "
        "register it with `snapcraft register-key {key_name!r}` before "
        "signing and pushing signatures to the Store."
    )

    def __init__(self, key_name):
        super().__init__(key_name=key_name)


class InvalidValidationRequestsError(StoreError):

    fmt = "Invalid validation requests (format must be name=revision): {requests}"

    def __init__(self, requests):
        requests_str = " ".join(requests)
        super().__init__(requests=requests_str)


class SignBuildAssertionError(StoreError):

    fmt = "Failed to sign build assertion for {snap_name!r}"

    def __init__(self, snap_name):
        super().__init__(snap_name=snap_name)


class InvalidLoginConfig(StoreError):

    fmt = "Invalid login config: {error}"

    def __init__(self, error):
        super().__init__(error=error)


def _error_list_to_message(response_json):
    """Handle error list.

    The error format is given here:
    https://dashboard.snapcraft.io/docs/api/snap.html#format
    """
    messages = []
    for error_list_item in response_json["error_list"]:
        messages.append(_error_list_item_to_message(error_list_item, response_json))

    return ", and ".join(messages)


def _error_list_item_to_message(error_list_item, response_json):
    """Handle error list item according to code.

    The list of codes are here:
    https://dashboard.snapcraft.io/docs/api/snap.html#codes
    """
    code = error_list_item["code"]
    message = ""
    if code == "macaroon-permission-required":
        message = _handle_macaroon_permission_required(response_json)

    if message:
        return message
    else:
        return error_list_item["message"]


def _handle_macaroon_permission_required(response_json):
    if "permission" in response_json and "channels" in response_json:
        if response_json["permission"] == "channel":
            channels = response_json["channels"]
            return "Lacking permission to release to channel(s) {}".format(
                formatting_utils.humanize_list(channels, "and")
            )

    return ""
