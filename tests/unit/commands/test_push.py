# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2017 Canonical Ltd
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
import os
from unittest import mock

from testtools.matchers import Contains, Equals, FileExists, Not
from xdg import BaseDirectory

from snapcraft import file_utils, storeapi
from snapcraft.storeapi.errors import (
    StoreDeltaApplicationError,
    StorePushError,
    StoreUploadError,
)
import tests
from . import CommandBaseTestCase


class PushCommandBaseTestCase(CommandBaseTestCase):
    def setUp(self):
        super().setUp()

        patcher = mock.patch("snapcraft.storeapi.StoreClient.push_precheck")
        self.mock_precheck = patcher.start()
        self.addCleanup(patcher.stop)

        self.snap_file = os.path.join(
            os.path.dirname(tests.__file__), "data", "test-snap.snap"
        )


class PushCommandTestCase(PushCommandBaseTestCase):
    def test_push_without_snap_must_raise_exception(self):
        result = self.run_command(["push"])

        self.assertThat(result.exit_code, Equals(2))
        self.assertThat(result.output, Contains("Usage:"))

    def test_push_a_snap(self):
        mock_tracker = mock.Mock(storeapi._status_tracker.StatusTracker)
        mock_tracker.track.return_value = {
            "code": "ready_to_release",
            "processed": True,
            "can_release": True,
            "url": "/fake/url",
            "revision": 9,
        }
        patcher = mock.patch.object(storeapi.StoreClient, "upload")
        mock_upload = patcher.start()
        self.addCleanup(patcher.stop)
        mock_upload.return_value = mock_tracker

        # Upload
        with mock.patch(
            "snapcraft.storeapi._status_tracker.StatusTracker"
        ) as mock_tracker:
            result = self.run_command(["push", self.snap_file])
        self.assertThat(result.exit_code, Equals(0))

        self.assertRegexpMatches(
            self.fake_logger.output,
            ".*push '.*test-snap.snap' to the store\.\n"
            "Revision 9 of 'basic' created\.",
        )
        mock_upload.assert_called_once_with("basic", self.snap_file)

    def test_push_without_login_must_raise_exception(self):
        raised = self.assertRaises(
            storeapi.errors.InvalidCredentialsError,
            self.run_command,
            ["push", self.snap_file],
        )

        self.assertThat(str(raised), Contains("Invalid credentials"))

    def test_push_nonexisting_snap_must_raise_exception(self):
        result = self.run_command(["push", "test-unexisting-snap"])
        self.assertThat(result.exit_code, Equals(2))

    def test_push_unregistered_snap_must_raise_exception(self):
        class MockResponse:
            status_code = 404
            error_list = [
                {
                    "code": "resource-not-found",
                    "message": "Snap not found for name=basic",
                }
            ]

        self.mock_precheck.side_effect = StorePushError("basic", MockResponse())

        raised = self.assertRaises(
            storeapi.errors.StorePushError, self.run_command, ["push", self.snap_file]
        )

        self.assertThat(
            str(raised),
            Contains(
                "You are not the publisher or allowed to push revisions for this "
                "snap. To become the publisher, run `snapcraft register "
                "basic` and try to push again."
            ),
        )

    def test_push_with_updown_error(self):
        # We really don't know of a reason why this would fail
        # aside from a 5xx style error on the server.
        class MockResponse:
            text = "stub error"
            reason = "stub reason"

        patcher = mock.patch.object(storeapi.StoreClient, "upload")
        mock_upload = patcher.start()
        self.addCleanup(patcher.stop)
        mock_upload.side_effect = StoreUploadError(MockResponse())

        self.assertRaises(
            storeapi.errors.StoreUploadError, self.run_command, ["push", self.snap_file]
        )

    def test_upload_raises_deprecation_warning(self):
        mock_tracker = mock.Mock(storeapi._status_tracker.StatusTracker)
        mock_tracker.track.return_value = {
            "code": "ready_to_release",
            "processed": True,
            "can_release": True,
            "url": "/fake/url",
            "revision": 9,
        }
        patcher = mock.patch.object(storeapi.StoreClient, "upload")
        mock_upload = patcher.start()
        self.addCleanup(patcher.stop)
        mock_upload.return_value = mock_tracker

        # Upload
        with mock.patch(
            "snapcraft.storeapi._status_tracker.StatusTracker"
        ) as mock_tracker:
            result = self.run_command(["upload", self.snap_file])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Contains("Revision 9 of 'basic' created."))
        mock_upload.assert_called_once_with("basic", self.snap_file)

    def test_push_and_release_a_snap(self):
        mock_tracker = mock.Mock(storeapi._status_tracker.StatusTracker)
        mock_tracker.track.return_value = {
            "code": "ready_to_release",
            "processed": True,
            "can_release": True,
            "url": "/fake/url",
            "revision": 9,
        }
        patcher = mock.patch.object(storeapi.StoreClient, "upload")
        mock_upload = patcher.start()
        self.addCleanup(patcher.stop)
        mock_upload.return_value = mock_tracker

        patcher = mock.patch.object(storeapi.StoreClient, "release")
        mock_release = patcher.start()
        self.addCleanup(patcher.stop)
        mock_release.return_value = {
            "opened_channels": ["beta"],
            "channel_map_tree": {
                "latest": {
                    "16": {
                        "amd64": [
                            {"channel": "stable", "info": "none"},
                            {"channel": "candidate", "info": "none"},
                            {
                                "revision": 9,
                                "channel": "beta",
                                "version": "0",
                                "info": "specific",
                            },
                            {"channel": "edge", "info": "tracking"},
                        ]
                    }
                }
            },
        }

        # Upload
        with mock.patch(
            "snapcraft.storeapi._status_tracker.StatusTracker"
        ) as mock_tracker:
            result = self.run_command(["push", self.snap_file, "--release", "beta"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Contains("Revision 9 of 'basic' created"))
        self.assertThat(result.output, Contains("The 'beta' channel is now open"))
        mock_upload.assert_called_once_with("basic", self.snap_file)
        mock_release.assert_called_once_with("basic", 9, ["beta"])

    def test_push_and_release_a_snap_to_N_channels(self):
        mock_tracker = mock.Mock(storeapi._status_tracker.StatusTracker)
        mock_tracker.track.return_value = {
            "code": "ready_to_release",
            "processed": True,
            "can_release": True,
            "url": "/fake/url",
            "revision": 9,
        }
        patcher = mock.patch.object(storeapi.StoreClient, "upload")
        mock_upload = patcher.start()
        self.addCleanup(patcher.stop)
        mock_upload.return_value = mock_tracker

        patcher = mock.patch.object(storeapi.StoreClient, "release")
        mock_release = patcher.start()
        self.addCleanup(patcher.stop)
        mock_release.return_value = {
            "opened_channels": ["beta,edge,candidate"],
            "channel_map_tree": {
                "latest": {
                    "16": {
                        "amd64": [
                            {"channel": "stable", "info": "none"},
                            {
                                "revision": 9,
                                "channel": "candidate",
                                "version": "0",
                                "info": "specific",
                            },
                            {
                                "revision": 9,
                                "channel": "beta",
                                "version": "0",
                                "info": "specific",
                            },
                            {
                                "revision": 9,
                                "channel": "edge",
                                "version": "0",
                                "info": "specific",
                            },
                        ]
                    }
                }
            },
        }

        # Upload
        with mock.patch(
            "snapcraft.storeapi._status_tracker.StatusTracker"
        ) as mock_tracker:
            result = self.run_command(
                ["push", self.snap_file, "--release", "edge,beta,candidate"]
            )

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Contains("Revision 9 of 'basic' created"))
        self.assertThat(
            result.output, Contains("The 'beta,edge,candidate' channel is now open")
        )

        mock_upload.assert_called_once_with("basic", self.snap_file)
        mock_release.assert_called_once_with("basic", 9, ["edge", "beta", "candidate"])

    def test_push_displays_humanized_message(self):
        mock_tracker = mock.Mock(storeapi._status_tracker.StatusTracker)
        mock_tracker.track.return_value = {
            "code": "ready_to_release",
            "processed": True,
            "can_release": True,
            "url": "/fake/url",
            "revision": 9,
        }
        patcher = mock.patch.object(storeapi.StoreClient, "upload")
        mock_upload = patcher.start()
        self.addCleanup(patcher.stop)
        mock_upload.return_value = mock_tracker

        patcher = mock.patch.object(storeapi.StoreClient, "release")
        mock_release = patcher.start()
        self.addCleanup(patcher.stop)
        mock_release.return_value = {
            "opened_channels": ["beta,edge,candidate"],
            "channel_map_tree": {
                "latest": {
                    "16": {
                        "amd64": [
                            {"channel": "stable", "info": "none"},
                            {
                                "revision": 9,
                                "channel": "candidate",
                                "version": "0",
                                "info": "specific",
                            },
                            {
                                "revision": 9,
                                "channel": "beta",
                                "version": "0",
                                "info": "specific",
                            },
                            {
                                "revision": 9,
                                "channel": "edge",
                                "version": "0",
                                "info": "specific",
                            },
                        ]
                    }
                }
            },
        }

        with mock.patch(
            "snapcraft.storeapi._status_tracker.StatusTracker"
        ) as mock_tracker:
            result = self.run_command(
                ["push", self.snap_file, "--release", "edge,beta,candidate"]
            )

        self.assertThat(
            result.output,
            Contains(
                "After pushing, an attempt will be made to release to "
                "'beta', 'candidate', and 'edge'"
            ),
        )


class PushCommandDeltasTestCase(PushCommandBaseTestCase):
    def setUp(self):
        super().setUp()

        self.latest_snap_revision = 8
        self.new_snap_revision = self.latest_snap_revision + 1

        mock_tracker = mock.Mock(storeapi._status_tracker.StatusTracker)
        mock_tracker.track.return_value = {
            "code": "ready_to_release",
            "processed": True,
            "can_release": True,
            "url": "/fake/url",
            "revision": self.new_snap_revision,
        }
        patcher = mock.patch.object(storeapi.StoreClient, "get_snap_revisions")
        mock_release = patcher.start()
        mock_release.return_value = [self.latest_snap_revision]
        self.addCleanup(patcher.stop)

        patcher = mock.patch.object(storeapi.StoreClient, "upload")
        self.mock_upload = patcher.start()
        self.addCleanup(patcher.stop)
        self.mock_upload.return_value = mock_tracker

    def test_push_revision_cached_with_experimental_deltas(self):
        # Upload
        with mock.patch("snapcraft.storeapi._status_tracker.StatusTracker"):
            result = self.run_command(["push", self.snap_file])
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

    def test_push_revision_uses_available_delta(self):
        # Push
        with mock.patch("snapcraft.storeapi._status_tracker.StatusTracker"):
            result = self.run_command(["push", self.snap_file])
        self.assertThat(result.exit_code, Equals(0))

        # Push again
        with mock.patch("snapcraft.storeapi._status_tracker.StatusTracker"):
            result = self.run_command(["push", self.snap_file])

        self.assertThat(result.exit_code, Equals(0))
        _, kwargs = self.mock_upload.call_args
        self.assertThat(kwargs.get("delta_format"), Equals("xdelta3"))

    def test_push_with_delta_generation_failure_falls_back(self):
        # Upload
        with mock.patch("snapcraft.storeapi._status_tracker.StatusTracker"):
            result = self.run_command(["push", self.snap_file])
        self.assertThat(result.exit_code, Equals(0))

        # Raise exception in delta upload
        patcher = mock.patch("snapcraft._store._push_delta")
        mock_push_delta = patcher.start()
        self.addCleanup(patcher.stop)
        mock_push_delta.side_effect = StoreDeltaApplicationError(
            "There has been a problem while processing a snap delta."
        )

        mock_tracker = mock.Mock(storeapi._status_tracker.StatusTracker)
        mock_tracker.track.return_value = {
            "code": "ready_to_release",
            "processed": True,
            "can_release": True,
            "url": "/fake/url",
            "revision": 9,
        }
        patcher = mock.patch.object(storeapi.StoreClient, "upload")
        mock_upload = patcher.start()
        self.addCleanup(patcher.stop)
        mock_upload.return_value = mock_tracker

        # Upload and ensure fallback is called
        with mock.patch("snapcraft.storeapi._status_tracker.StatusTracker"):
            result = self.run_command(["push", self.snap_file])
        self.assertThat(result.exit_code, Equals(0))
        mock_upload.assert_called_once_with("basic", self.snap_file)

    def test_push_with_delta_upload_failure_falls_back(self):
        # Upload
        with mock.patch("snapcraft.storeapi._status_tracker.StatusTracker"):
            result = self.run_command(["push", self.snap_file])
        self.assertThat(result.exit_code, Equals(0))

        mock_tracker = mock.Mock(storeapi._status_tracker.StatusTracker)
        mock_tracker.track.return_value = {
            "code": "ready_to_release",
            "processed": True,
            "can_release": True,
            "url": "/fake/url",
            "revision": 9,
        }
        result = {
            "code": "processing_upload_delta_error",
            "errors": [{"message": "Delta service failed to apply delta within 60s"}],
        }
        mock_tracker.raise_for_code.side_effect = [
            storeapi.errors.StoreReviewError(result=result),
            None,
        ]
        patcher = mock.patch.object(storeapi.StoreClient, "upload")
        mock_upload = patcher.start()
        self.addCleanup(patcher.stop)
        mock_upload.return_value = mock_tracker

        # Upload and ensure fallback is called
        with mock.patch("snapcraft.storeapi._status_tracker.StatusTracker"):
            result = self.run_command(["push", self.snap_file])
        self.assertThat(result.exit_code, Equals(0))
        mock_upload.assert_has_calls(
            [
                mock.call(
                    "basic",
                    mock.ANY,
                    delta_format="xdelta3",
                    delta_hash=mock.ANY,
                    source_hash=mock.ANY,
                    target_hash=mock.ANY,
                ),
                mock.call().track(),
                mock.call().raise_for_code(),
                mock.call("basic", self.snap_file),
                mock.call().track(),
                mock.call().raise_for_code(),
            ]
        )

    def test_push_with_disabled_delta_falls_back(self):
        # Upload
        with mock.patch("snapcraft.storeapi._status_tracker.StatusTracker"):
            result = self.run_command(["push", self.snap_file])
        self.assertThat(result.exit_code, Equals(0))

        original_upload = storeapi.StoreClient.upload
        called = False

        def _fake_upload(self, *args, **kwargs):
            nonlocal called
            if called:
                return original_upload(self, *args, **kwargs)
            else:
                called = True

                class _FakeResponse:
                    status_code = 501

                    def json(self):
                        return {
                            "error_list": [
                                {
                                    "code": "feature-disabled",
                                    "message": "The delta upload support is currently disabled.",
                                }
                            ]
                        }

                raise storeapi.errors.StoreServerError(_FakeResponse())

        patcher = mock.patch.object(
            storeapi.StoreClient, "upload", side_effect=_fake_upload
        )
        mock_upload = patcher.start()
        self.addCleanup(patcher.stop)

        # Upload and ensure fallback is called
        with mock.patch("snapcraft.storeapi._status_tracker.StatusTracker"):
            result = self.run_command(["push", self.snap_file])
        self.assertThat(result.exit_code, Equals(0))
        mock_upload.assert_has_calls(
            [
                mock.call(
                    "basic",
                    mock.ANY,
                    delta_format="xdelta3",
                    delta_hash=mock.ANY,
                    source_hash=mock.ANY,
                    target_hash=mock.ANY,
                ),
                mock.call("basic", self.snap_file),
            ]
        )


class PushCommandDeltasWithPruneTestCase(PushCommandBaseTestCase):

    scenarios = [
        (
            "delete other cache files with valid name",
            {
                "cached_snaps": [
                    "a-cached-snap_0.3_{}_8.snap",
                    "another-cached-snap_1.0_fakearch_6.snap",
                ]
            },
        ),
        (
            "delete other cache files with invalid name",
            {
                "cached_snaps": [
                    "a-cached-snap_0.3_{}.snap",
                    "cached-snap-without-revision_1.0_fakearch.snap",
                    "another-cached-snap-without-version_fakearch.snap",
                ]
            },
        ),
    ]

    def test_push_revision_prune_snap_cache(self):
        snap_revision = 9

        patcher = mock.patch.object(storeapi.StoreClient, "get_snap_revisions")
        mock_release = patcher.start()
        self.addCleanup(patcher.stop)
        mock_release.return_value = [snap_revision]

        mock_tracker = mock.Mock(storeapi._status_tracker.StatusTracker)
        mock_tracker.track.return_value = {
            "code": "ready_to_release",
            "processed": True,
            "can_release": True,
            "url": "/fake/url",
            "revision": snap_revision,
        }

        patcher = mock.patch.object(storeapi.StoreClient, "upload")
        mock_upload = patcher.start()
        self.addCleanup(patcher.stop)
        mock_upload.return_value = mock_tracker

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

        for cached_snap in self.cached_snaps:
            cached_snap = cached_snap.format(deb_arch)
            open(os.path.join(snap_cache, cached_snap), "a").close()

        # Upload
        with mock.patch("snapcraft.storeapi._status_tracker.StatusTracker"):
            result = self.run_command(["push", self.snap_file])
        self.assertThat(result.exit_code, Equals(0))

        real_cached_snap = os.path.join(
            snap_cache, file_utils.calculate_sha3_384(self.snap_file)
        )

        self.assertThat(os.path.join(snap_cache, real_cached_snap), FileExists())

        for snap in self.cached_snaps:
            snap = snap.format(deb_arch)
            self.assertThat(os.path.join(snap_cache, snap), Not(FileExists()))
        self.assertThat(len(os.listdir(snap_cache)), Equals(1))
