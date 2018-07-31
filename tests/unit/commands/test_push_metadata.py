# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2017-2018 Canonical Ltd
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

import fixtures
from testtools.matchers import Contains, Equals, Not

from snapcraft import storeapi
from snapcraft.storeapi.errors import StorePushError
import tests
from . import CommandBaseTestCase


class PushMetadataCommandTestCase(CommandBaseTestCase):
    def setUp(self):
        super().setUp()

        patcher = mock.patch("snapcraft.storeapi.StoreClient.push_precheck")
        patcher.start()
        self.addCleanup(patcher.stop)

        self.pushed_icon = None

        def _save_updated_icon(snap_name, metadata, force):
            self.pushed_icon = metadata["icon"].read() if metadata["icon"] else None

        patcher = mock.patch.object(
            storeapi.StoreClient, "push_binary_metadata", side_effect=_save_updated_icon
        )
        self.mock_binary_metadata = patcher.start()
        self.addCleanup(patcher.stop)

        self.snap_file = os.path.join(
            os.path.dirname(tests.__file__), "data", "test-snap-with-icon.snap"
        )

    def assert_expected_metadata_calls(self, force=False):
        # text metadata
        text_metadata = {
            "description": "Description of the most simple snap",
            "summary": "Summary of the most simple snap",
        }
        self.mock_metadata.assert_called_once_with("basic", text_metadata, force)
        # binary metadata
        args, _ = self.mock_binary_metadata.call_args
        self.assertEqual(args[0], "basic")
        expected_icon = (
            b'<svg width="256" height="256">\n'
            b'<rect width="256" height="256" style="fill:rgb(0,0,255)" />\n'
            b"</svg>"
        )
        self.assertEqual(self.pushed_icon, expected_icon)
        self.assertEqual(args[2], force)

    def test_without_snap_must_raise_exception(self):
        result = self.run_command(["push-metadata"])

        self.assertThat(result.exit_code, Equals(2))
        self.assertThat(result.output, Contains("Usage:"))

    def test_simple(self):
        patcher = mock.patch.object(storeapi.StoreClient, "push_metadata")
        self.mock_metadata = patcher.start()
        self.addCleanup(patcher.stop)

        # push metadata
        with mock.patch("snapcraft.storeapi._status_tracker.StatusTracker"):
            result = self.run_command(["push-metadata", self.snap_file])
        self.assertThat(result.exit_code, Equals(0))

        self.assertThat(
            result.output, Not(Contains("Pushing metadata to the Store (force=False)"))
        )
        self.assertThat(result.output, Contains("The metadata has been pushed"))
        self.assert_expected_metadata_calls(force=False)

    def test_simple_debug(self):
        patcher = mock.patch.object(storeapi.StoreClient, "push_metadata")
        self.mock_metadata = patcher.start()
        self.addCleanup(patcher.stop)

        self.useFixture(
            fixtures.EnvironmentVariable("SNAPCRAFT_ENABLE_DEVELOPER_DEBUG", "yes")
        )
        # push metadata
        with mock.patch("snapcraft.storeapi._status_tracker.StatusTracker"):
            result = self.run_command(["push-metadata", self.snap_file])
        self.assertThat(result.exit_code, Equals(0))

        self.assertThat(
            result.output, Contains("Pushing metadata to the Store (force=False)")
        )
        self.assertThat(result.output, Contains("The metadata has been pushed"))
        self.assert_expected_metadata_calls(force=False)

    def test_without_login_must_raise_exception(self):
        raised = self.assertRaises(
            storeapi.errors.InvalidCredentialsError,
            self.run_command,
            ["push-metadata", self.snap_file],
        )

        self.assertThat(str(raised), Contains("Invalid credentials"))

    def test_nonexisting_snap_must_raise_exception(self):
        result = self.run_command(["push-metadata", "test-unexisting-snap"])
        self.assertThat(result.exit_code, Equals(2))

    def test_unregistered_snap_must_raise_exception(self):
        class MockResponse:
            status_code = 404
            error_list = [
                {
                    "code": "resource-not-found",
                    "message": "Snap not found for name=basic",
                }
            ]

        patcher = mock.patch.object(storeapi.StoreClient, "push_precheck")
        mock_precheck = patcher.start()
        self.addCleanup(patcher.stop)
        mock_precheck.side_effect = StorePushError("basic", MockResponse())

        raised = self.assertRaises(
            storeapi.errors.StorePushError,
            self.run_command,
            ["push-metadata", self.snap_file],
        )

        self.assertThat(
            str(raised),
            Contains(
                "You are not the publisher or allowed to push revisions for this "
                "snap. To become the publisher, run `snapcraft register "
                "basic` and try to push again."
            ),
        )

    def test_forced(self):
        patcher = mock.patch.object(storeapi.StoreClient, "push_metadata")
        self.mock_metadata = patcher.start()
        self.addCleanup(patcher.stop)

        self.useFixture(
            fixtures.EnvironmentVariable("SNAPCRAFT_ENABLE_DEVELOPER_DEBUG", "yes")
        )

        result = self.run_command(["push-metadata", self.snap_file, "--force"])

        self.assertThat(
            result.output, Contains("Pushing metadata to the Store (force=True)")
        )
        self.assertThat(result.output, Contains("The metadata has been pushed"))
        self.assert_expected_metadata_calls(force=True)

    def test_snap_without_icon(self):
        patcher = mock.patch.object(storeapi.StoreClient, "push_metadata")
        self.mock_metadata = patcher.start()
        self.addCleanup(patcher.stop)

        snap_file = os.path.join(
            os.path.dirname(tests.__file__), "data", "test-snap.snap"
        )

        # push metadata
        with mock.patch("snapcraft.storeapi._status_tracker.StatusTracker"):
            result = self.run_command(["push-metadata", snap_file])
        self.assertThat(result.exit_code, Equals(0))

        self.assertThat(result.output, Contains("The metadata has been pushed"))
        # icon pushed to store is None
        self.assertIsNone(self.pushed_icon)
