# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2020 Canonical Ltd
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

import logging
import os
from unittest import mock

import fixtures
from testtools.matchers import Contains, Equals, FileExists, Not
from xdg import BaseDirectory

import tests
from snapcraft import file_utils, internal, storeapi
from snapcraft.internal import review_tools
from snapcraft.storeapi.errors import (
    StoreDeltaApplicationError,
    StoreUpDownError,
    StoreUploadError,
)

from . import FakeStoreCommandsBaseTestCase


class UploadCommandBaseTestCase(FakeStoreCommandsBaseTestCase):
    def setUp(self):
        super().setUp()

        self.snap_file = os.path.join(
            os.path.dirname(tests.__file__), "data", "test-snap.snap"
        )

        self.fake_review_tools_run = fixtures.MockPatch(
            "snapcraft.internal.review_tools.run"
        )
        self.useFixture(self.fake_review_tools_run)

        self.fake_review_tools_is_available = fixtures.MockPatch(
            "snapcraft.internal.review_tools.is_available", return_value=False
        )
        self.useFixture(self.fake_review_tools_is_available)


class UploadCommandTestCase(UploadCommandBaseTestCase):
    def test_upload_without_snap_must_raise_exception(self):
        result = self.run_command(["upload"])

        self.assertThat(result.exit_code, Equals(2))
        self.assertThat(result.output, Contains("Usage:"))

    def test_upload_a_snap(self):
        # Upload
        result = self.run_command(["upload", self.snap_file])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Contains("Revision 19 of 'basic' created."))
        self.fake_store_upload.mock.assert_called_once_with(
            snap_name="basic",
            snap_filename=self.snap_file,
            built_at=None,
            channels=None,
            delta_format=None,
            delta_hash=None,
            source_hash=None,
            target_hash=None,
        )

    def test_review_tools_not_available(self):
        result = self.run_command(["upload", self.snap_file])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Contains(
                "Install the review-tools from the Snap Store for enhanced "
                "checks before uploading this snap"
            ),
        )
        self.fake_review_tools_run.mock.assert_not_called()

    def test_upload_a_snap_review_tools_run_success(self):
        self.fake_review_tools_is_available.mock.return_value = True

        result = self.run_command(["upload", self.snap_file])

        self.assertThat(result.exit_code, Equals(0))
        self.fake_review_tools_run.mock.assert_called_once_with(
            snap_filename=self.snap_file
        )

    def test_upload_a_snap_review_tools_run_fail(self):
        self.fake_review_tools_is_available.mock.return_value = True
        self.fake_review_tools_run.mock.side_effect = review_tools.errors.ReviewError(
            {
                "snap.v2_functional": {"error": {}, "warn": {}},
                "snap.v2_security": {
                    "error": {
                        "security-snap-v2:security_issue": {
                            "text": "(NEEDS REVIEW) security message."
                        }
                    },
                    "warn": {},
                },
                "snap.v2_lint": {"error": {}, "warn": {}},
            }
        )

        result = self.run_command(["upload", self.snap_file])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Contains(
                "Review Tools did not fully pass for this snap.\n"
                "Specific measures might need to be taken on the Snap Store before "
                "this snap can be fully accepted.\n"
                "Security Issues:\n"
                "- (NEEDS REVIEW) security message"
            ),
        )
        self.fake_review_tools_run.mock.assert_called_once_with(
            snap_filename=self.snap_file
        )

    def test_upload_with_started_at(self):
        snap_file = os.path.join(
            os.path.dirname(tests.__file__), "data", "test-snap-with-started-at.snap"
        )

        # Upload
        result = self.run_command(["upload", snap_file])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Contains("Revision 19 of 'basic' created."))
        self.fake_store_upload.mock.assert_called_once_with(
            snap_name="basic",
            snap_filename=snap_file,
            built_at="2019-05-07T19:25:53.939041Z",
            channels=None,
            delta_format=None,
            delta_hash=None,
            source_hash=None,
            target_hash=None,
        )

    def test_upload_without_login_must_ask(self):
        self.fake_store_upload_precheck.mock.side_effect = [
            storeapi.http_clients.errors.InvalidCredentialsError("error"),
            None,
        ]

        result = self.run_command(
            ["upload", self.snap_file], input="\n\n\n\nuser@example.com\nsecret\n"
        )

        self.assertThat(
            result.output, Contains("You are required to login before continuing.")
        )

    def test_upload_nonexisting_snap_must_raise_exception(self):
        result = self.run_command(["upload", "test-unexisting-snap"])

        self.assertThat(result.exit_code, Equals(2))

    def test_upload_invalid_snap_must_raise_exception(self):
        snap_path = os.path.join(
            os.path.dirname(tests.__file__), "data", "invalid.snap"
        )

        raised = self.assertRaises(
            internal.errors.SnapDataExtractionError,
            self.run_command,
            ["upload", snap_path],
        )

        self.assertThat(str(raised), Contains("Cannot read data from snap"))

    def test_upload_unregistered_snap_must_ask(self):
        class MockResponse:
            status_code = 404

            def json(self):
                return dict(
                    error_list=[
                        {
                            "code": "resource-not-found",
                            "message": "Snap not found for name=basic",
                        }
                    ]
                )

        self.fake_store_upload_precheck.mock.side_effect = [
            StoreUploadError("basic", MockResponse()),
            None,
        ]

        result = self.run_command(["upload", self.snap_file], input="y\n")

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Contains("You are required to register this snap before continuing. "),
        )
        self.fake_store_register.mock.assert_called_once_with(
            "basic", is_private=False, series="16", store_id=None
        )

    def test_upload_unregistered_snap_must_raise_exception_if_not_registering(self):
        class MockResponse:
            status_code = 404

            def json(self):
                return dict(
                    error_list=[
                        {
                            "code": "resource-not-found",
                            "message": "Snap not found for name=basic",
                        }
                    ]
                )

        self.fake_store_upload_precheck.mock.side_effect = [
            StoreUploadError("basic", MockResponse()),
            None,
        ]

        raised = self.assertRaises(
            storeapi.errors.StoreUploadError,
            self.run_command,
            ["upload", self.snap_file],
        )

        self.assertThat(
            str(raised),
            Contains("This snap is not registered. Register the snap and try again."),
        )
        self.fake_store_register.mock.assert_not_called()

    def test_upload_with_updown_error(self):
        # We really don't know of a reason why this would fail
        # aside from a 5xx style error on the server.
        class MockResponse:
            text = "stub error"
            reason = "stub reason"

        self.fake_store_upload.mock.side_effect = StoreUpDownError(MockResponse())

        self.assertRaises(
            storeapi.errors.StoreUpDownError,
            self.run_command,
            ["upload", self.snap_file],
        )

    def test_upload_raises_deprecation_warning(self):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)

        # Push
        result = self.run_command(["push", self.snap_file])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Contains("Revision 19 of 'basic' created."))
        self.assertThat(
            fake_logger.output,
            Contains(
                "DEPRECATED: The 'push' set of commands have been replaced with 'upload'."
            ),
        )
        self.fake_store_upload.mock.assert_called_once_with(
            snap_name="basic",
            snap_filename=self.snap_file,
            built_at=None,
            channels=None,
            delta_format=None,
            delta_hash=None,
            source_hash=None,
            target_hash=None,
        )

    def test_upload_and_release_a_snap(self):
        self.useFixture
        # Upload
        result = self.run_command(["upload", self.snap_file, "--release", "beta"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Contains("Revision 19 of 'basic' created"))
        self.fake_store_upload.mock.assert_called_once_with(
            snap_name="basic",
            snap_filename=self.snap_file,
            built_at=None,
            channels=["beta"],
            delta_format=None,
            delta_hash=None,
            source_hash=None,
            target_hash=None,
        )

    def test_upload_and_release_a_snap_to_N_channels(self):
        # Upload
        result = self.run_command(
            ["upload", self.snap_file, "--release", "edge,beta,candidate"]
        )

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Contains("Revision 19 of 'basic' created"))
        self.fake_store_upload.mock.assert_called_once_with(
            snap_name="basic",
            snap_filename=self.snap_file,
            built_at=None,
            channels=["edge", "beta", "candidate"],
            delta_format=None,
            delta_hash=None,
            source_hash=None,
            target_hash=None,
        )

    def test_upload_displays_humanized_message(self):
        result = self.run_command(
            ["upload", self.snap_file, "--release", "edge,beta,candidate"]
        )

        self.assertThat(
            result.output,
            Contains(
                "After uploading, the resulting snap revision will be released to "
                "'beta', 'candidate', and 'edge' when it passes the Snap Store review."
            ),
        )


class UploadCommandDeltasTestCase(UploadCommandBaseTestCase):
    def setUp(self):
        super().setUp()

        self.latest_snap_revision = 8
        self.new_snap_revision = self.latest_snap_revision + 1

        self.mock_tracker.track.return_value = {
            "code": "ready_to_release",
            "processed": True,
            "can_release": True,
            "url": "/fake/url",
            "revision": self.new_snap_revision,
        }

    def test_upload_revision_cached_with_experimental_deltas(self):
        # Upload
        result = self.run_command(["upload", self.snap_file])

        self.assertThat(result.exit_code, Equals(0))
        snap_cache = os.path.join(
            BaseDirectory.xdg_cache_home,
            "snapcraft",
            "projects",
            "basic",
            "snap_hashes",
            "amd64",
        )
        cached_snap = os.path.join(
            snap_cache, file_utils.calculate_sha3_384(self.snap_file)
        )

        self.assertThat(cached_snap, FileExists())

    def test_upload_revision_uses_available_delta(self):
        # Upload
        result = self.run_command(["upload", self.snap_file])

        self.assertThat(result.exit_code, Equals(0))

        # Upload again
        result = self.run_command(["upload", self.snap_file])

        self.assertThat(result.exit_code, Equals(0))
        _, kwargs = self.fake_store_upload.mock.call_args
        self.assertThat(kwargs.get("delta_format"), Equals("xdelta3"))

    def test_upload_with_delta_generation_failure_falls_back(self):
        # Upload and ensure fallback is called
        with mock.patch(
            "snapcraft._store._upload_delta",
            side_effect=StoreDeltaApplicationError("error"),
        ):
            result = self.run_command(["upload", self.snap_file])

        self.assertThat(result.exit_code, Equals(0))
        self.fake_store_upload.mock.assert_called_once_with(
            snap_name="basic",
            snap_filename=self.snap_file,
            built_at=None,
            channels=None,
            delta_format=None,
            delta_hash=None,
            source_hash=None,
            target_hash=None,
        )

    def test_upload_with_delta_upload_failure_falls_back(self):
        # Upload
        result = self.run_command(["upload", self.snap_file])

        self.assertThat(result.exit_code, Equals(0))

        result = {
            "code": "processing_upload_delta_error",
            "errors": [{"message": "Delta service failed to apply delta within 60s"}],
        }
        self.mock_tracker.raise_for_code.side_effect = [
            storeapi.errors.StoreReviewError(result=result),
            None,
        ]

        # Upload and ensure fallback is called
        result = self.run_command(["upload", self.snap_file])

        self.assertThat(result.exit_code, Equals(0))
        self.fake_store_upload.mock.assert_has_calls(
            [
                mock.call(
                    snap_name="basic",
                    snap_filename=mock.ANY,
                    built_at=None,
                    channels=None,
                    delta_format="xdelta3",
                    delta_hash=mock.ANY,
                    source_hash=mock.ANY,
                    target_hash=mock.ANY,
                ),
                mock.call(
                    snap_name="basic",
                    snap_filename=self.snap_file,
                    built_at=None,
                    channels=None,
                    delta_format=None,
                    delta_hash=None,
                    source_hash=None,
                    target_hash=None,
                ),
            ]
        )

    def test_upload_with_disabled_delta_falls_back(self):
        # Upload
        result = self.run_command(["upload", self.snap_file])

        self.assertThat(result.exit_code, Equals(0))

        class _FakeResponse:
            status_code = 501
            reason = "disabled"

            def json(self):
                return {
                    "error_list": [
                        {
                            "code": "feature-disabled",
                            "message": "The delta upload support is currently disabled.",
                        }
                    ]
                }

        self.fake_store_upload.mock.side_effect = [
            storeapi.http_clients.errors.StoreServerError(_FakeResponse()),
            self.mock_tracker,
        ]

        # Upload and ensure fallback is called
        with mock.patch("snapcraft.storeapi._status_tracker.StatusTracker"):
            result = self.run_command(["upload", self.snap_file])
        self.assertThat(result.exit_code, Equals(0))
        self.fake_store_upload.mock.assert_has_calls(
            [
                mock.call(
                    snap_name="basic",
                    snap_filename=mock.ANY,
                    built_at=None,
                    channels=None,
                    delta_format="xdelta3",
                    delta_hash=mock.ANY,
                    source_hash=mock.ANY,
                    target_hash=mock.ANY,
                ),
                mock.call(
                    snap_name="basic",
                    snap_filename=self.snap_file,
                    built_at=None,
                    channels=None,
                    delta_format=None,
                    delta_hash=None,
                    source_hash=None,
                    target_hash=None,
                ),
            ]
        )


class UploadCommandDeltasWithPruneTestCase(UploadCommandBaseTestCase):
    def run_test(self, cached_snaps):
        snap_revision = 19

        self.mock_tracker.track.return_value = {
            "code": "ready_to_release",
            "processed": True,
            "can_release": True,
            "url": "/fake/url",
            "revision": snap_revision,
        }

        deb_arch = "amd64"

        snap_cache = os.path.join(
            BaseDirectory.xdg_cache_home,
            "snapcraft",
            "projects",
            "basic",
            "snap_hashes",
            deb_arch,
        )
        os.makedirs(snap_cache)

        for cached_snap in cached_snaps:
            cached_snap = cached_snap.format(deb_arch)
            open(os.path.join(snap_cache, cached_snap), "a").close()

        # Upload
        result = self.run_command(["upload", self.snap_file])

        self.assertThat(result.exit_code, Equals(0))

        real_cached_snap = os.path.join(
            snap_cache, file_utils.calculate_sha3_384(self.snap_file)
        )

        self.assertThat(os.path.join(snap_cache, real_cached_snap), FileExists())

        for snap in cached_snaps:
            snap = snap.format(deb_arch)
            self.assertThat(os.path.join(snap_cache, snap), Not(FileExists()))
        self.assertThat(len(os.listdir(snap_cache)), Equals(1))

    def test_delete_other_cache_files_with_valid_name(self):
        self.run_test(
            ["a-cached-snap_0.3_{}_8.snap", "another-cached-snap_1.0_fakearch_6.snap"]
        )

    def test_delete_other_cache_file_with_invalid_name(self):
        self.run_test(
            [
                "a-cached-snap_0.3_{}.snap",
                "cached-snap-without-revision_1.0_fakearch.snap",
                "another-cached-snap-without-version_fakearch.snap",
            ]
        )
