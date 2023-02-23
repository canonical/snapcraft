# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2020-2022 Canonical Ltd
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

from textwrap import dedent

from snapcraft_legacy.storeapi import errors
from snapcraft_legacy.storeapi.channels import Channel
from snapcraft_legacy.storeapi.status import SnapStatusChannelDetails


class TestSnapNotFoundException:
    scenarios = (
        (
            "SnapNotFoundError snap_name",
            dict(
                exception_class=errors.SnapNotFoundError,
                kwargs=dict(snap_name="not-found"),
                expected_brief="Snap 'not-found' was not found.",
                expected_resolution="Ensure you have proper access rights for 'not-found'.",
                expected_details=None,
                expected_docs_url=None,
                expected_reportable=False,
            ),
        ),
        (
            "SnapNotFoundError snap_id",
            dict(
                exception_class=errors.SnapNotFoundError,
                kwargs=dict(snap_id="1234567890"),
                expected_brief="Cannot find snap with snap_id '1234567890'.",
                expected_resolution="Ensure you have proper access rights for '1234567890'.",
                expected_details=None,
                expected_docs_url=None,
                expected_reportable=False,
            ),
        ),
        (
            "SnapNotFoundError snap_name, channel and arch",
            dict(
                exception_class=errors.SnapNotFoundError,
                kwargs=dict(
                    snap_name="not-found", channel="default/stable", arch="foo"
                ),
                expected_brief="Snap 'not-found' for architecture 'foo' was not found on channel 'default/stable'.",
                expected_resolution=dedent(
                    """\
                Ensure you have proper access rights for 'not-found'.
                Also ensure the correct channel and architecture was used."""
                ),
                expected_details=None,
                expected_docs_url=None,
                expected_reportable=False,
            ),
        ),
        (
            "SnapNotFoundError snap_name and channel",
            dict(
                exception_class=errors.SnapNotFoundError,
                kwargs=dict(snap_name="not-found", channel="default/stable"),
                expected_brief="Snap 'not-found' was not found on channel 'default/stable'.",
                expected_resolution=dedent(
                    """\
                Ensure you have proper access rights for 'not-found'.
                Also ensure the correct channel was used."""
                ),
                expected_details=None,
                expected_docs_url=None,
                expected_reportable=False,
            ),
        ),
        (
            "SnapNotFoundError snap_name and arch",
            dict(
                exception_class=errors.SnapNotFoundError,
                kwargs=dict(snap_name="not-found", arch="foo"),
                expected_brief="Snap 'not-found' for architecture 'foo' was not found.",
                expected_resolution=dedent(
                    """\
                Ensure you have proper access rights for 'not-found'.
                Also ensure the correct architecture was used."""
                ),
                expected_details=None,
                expected_docs_url=None,
                expected_reportable=False,
            ),
        ),
    )

    def test_snapcraft_exception_handling(
        self,
        exception_class,
        kwargs,
        expected_brief,
        expected_resolution,
        expected_details,
        expected_docs_url,
        expected_reportable,
    ):
        exception = exception_class(**kwargs)
        assert exception.get_brief() == expected_brief
        assert exception.get_resolution() == expected_resolution
        assert exception.get_details() == expected_details
        assert exception.get_docs_url() == expected_docs_url
        assert exception.get_reportable() == expected_reportable


class TestErrorFormatting:
    scenarios = [
        (
            "NoSnapIdError",
            dict(
                exception_class=errors.NoSnapIdError,
                kwargs=dict(snap_name="test-1"),
                expected_message=(
                    "Failed to get snap ID for snap 'test-1'. This is an error in "
                    "the store, please open a new topic in the 'store' category in the "
                    "forum: https://forum.snapcraft.io/c/store"
                ),
            ),
        ),
        (
            "DownloadNotFoundError",
            dict(
                exception_class=errors.DownloadNotFoundError,
                kwargs=dict(path="test-1"),
                expected_message="Downloaded file not found 'test-1'.",
            ),
        ),
        (
            "SHAMismatchError",
            dict(
                exception_class=errors.SHAMismatchError,
                kwargs=dict(path="test-1", expected="test-2", calculated="test-3"),
                expected_message=(
                    "The SHA3-384 checksum for 'test-1' was 'test-3': expected 'test-2'."
                ),
            ),
        ),
        (
            "StoreDeltaApplicationError",
            dict(
                exception_class=errors.StoreDeltaApplicationError,
                kwargs=dict(message="test-1"),
                expected_message="test-1",
            ),
        ),
        (
            "StoreChannelClosingPermissionError",
            dict(
                exception_class=errors.StoreChannelClosingPermissionError,
                kwargs=dict(snap_name="test-1", snap_series="test-2"),
                expected_message=(
                    "Your account lacks permission to close channels for this snap. Make "
                    "sure the logged in account has upload permissions on 'test-1' "
                    "in series 'test-2'."
                ),
            ),
        ),
        (
            "StoreBuildAssertionPermissionError",
            dict(
                exception_class=errors.StoreBuildAssertionPermissionError,
                kwargs=dict(snap_name="test-1", snap_series="test-2"),
                expected_message=(
                    "Your account lacks permission to assert builds for this snap. "
                    "Make sure you are logged in as the publisher of 'test-1' for series 'test-2'."
                ),
            ),
        ),
        (
            "StoreAssertionError",
            dict(
                exception_class=errors.StoreAssertionError,
                kwargs=dict(endpoint="test-1", snap_name="test-2", error="test-3"),
                expected_message="Error signing test-1 assertion for test-2: test-3",
            ),
        ),
        (
            "KeyAlreadyExistsError",
            dict(
                exception_class=errors.KeyAlreadyExistsError,
                kwargs=dict(key_name="test-1"),
                expected_message="The key 'test-1' already exists",
            ),
        ),
        (
            "KeyAlreadyRegisteredError",
            dict(
                exception_class=errors.KeyAlreadyRegisteredError,
                kwargs=dict(key_name="test-1"),
                expected_message="You have already registered a key named 'test-1'",
            ),
        ),
        (
            "NoKeysError",
            dict(
                exception_class=errors.NoKeysError,
                kwargs={},
                expected_message=(
                    "You have no usable keys.\nPlease create at least one key with "
                    "`snapcraft create-key` for use with snap."
                ),
            ),
        ),
        (
            "NoSuchKeyError",
            dict(
                exception_class=errors.NoSuchKeyError,
                kwargs=dict(key_name="test-1"),
                expected_message=(
                    "You have no usable key named 'test-1'.\nSee the keys available "
                    "in your system with `snapcraft keys`."
                ),
            ),
        ),
        (
            "KeyNotRegisteredError",
            dict(
                exception_class=errors.KeyNotRegisteredError,
                kwargs=dict(key_name="test-1"),
                expected_message=(
                    "The key 'test-1' is not registered in the Store.\nPlease "
                    "register it with `snapcraft register-key 'test-1'` before "
                    "signing and uploading signatures to the Store."
                ),
            ),
        ),
        (
            "InvalidValidationRequestsError",
            dict(
                exception_class=errors.InvalidValidationRequestsError,
                kwargs=dict(requests=["test-1"]),
                expected_message=(
                    "Invalid validation requests (format must be name=revision): test-1"
                ),
            ),
        ),
        (
            "SignBuildAssertionError",
            dict(
                exception_class=errors.SignBuildAssertionError,
                kwargs=dict(snap_name="test-1"),
                expected_message="Failed to sign build assertion for 'test-1'",
            ),
        ),
        (
            "ChannelNotAvailableOnArchError",
            dict(
                exception_class=errors.ChannelNotAvailableOnArchError,
                kwargs=dict(snap_name="test-1", channel=Channel, arch="test-2"),
                expected_message=(
                    f"No releases available for 'test-1' on channel {Channel!r} "
                    "for architecture 'test-2'.\n"
                    "Ensure the selected channel contains released revisions "
                    "for this architecture."
                ),
            ),
        ),
        (
            "InvalidChannelSet",
            dict(
                exception_class=errors.InvalidChannelSet,
                kwargs=dict(
                    snap_name="test-1",
                    channel=Channel(channel="stable"),
                    channel_outliers=[
                        SnapStatusChannelDetails(
                            snap_name="test-2",
                            arch="test-3",
                            payload={"test-4": "test-5"},
                        )
                    ],
                ),
                expected_message=(
                    f"The '{Channel(channel='stable')}' channel for 'test-1' does"
                    " not form a complete set.\n"
                    "There is no revision released for the following"
                    " architectures: \"'test-3'\".\n"
                    "Ensure the selected channel contains released revisions for all architectures."
                ),
            ),
        ),
    ]

    def test_error_formatting(self, exception_class, kwargs, expected_message):
        assert str(exception_class(**kwargs)) == expected_message
