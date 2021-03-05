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
from unittest import mock

import fixtures
from testtools.matchers import Contains, Equals, FileExists

import snapcraft.storeapi.errors

from . import StoreCommandsBaseTestCase


class ValidateCommandTestCase(StoreCommandsBaseTestCase):
    def setUp(self):
        super().setUp()

        patcher = mock.patch("snapcraft._store.Popen")
        self.popen_mock = patcher.start()
        rv_mock = mock.Mock()
        rv_mock.returncode = 0
        rv_mock.communicate.return_value = [b"foo", b""]
        self.popen_mock.return_value = rv_mock
        self.addCleanup(patcher.stop)

        self.client.login(email="dummy", password="test correct password")

    def test_validate_success(self):
        result = self.run_command(["validate", "core", "core=3", "test-snap=4"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output, Contains("Signing validations assertion for core=3")
        )
        self.assertThat(
            result.output, Contains("Signing validations assertion for test-snap=4")
        )

    def test_validate_with_key(self):
        result = self.run_command(
            ["validate", "core", "core=3", "test-snap=4", "--key-name=keyname"]
        )

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output, Contains("Signing validations assertion for core=3")
        )
        self.assertThat(
            result.output, Contains("Signing validations assertion for test-snap=4")
        )
        self.popen_mock.assert_called_with(
            ["snap", "sign", "-k", "keyname"], stderr=-1, stdin=-1, stdout=-1
        )

    def test_validate_from_branded_store(self):
        # Validating snaps from a branded store requires setting
        # `SNAPCRAFT_UBUNTU_STORE` environment variable to the store 'slug'.
        self.useFixture(
            fixtures.EnvironmentVariable("SNAPCRAFT_UBUNTU_STORE", "Test-Branded")
        )

        result = self.run_command(["validate", "core", "test-snap-branded-store=1"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Contains("Signing validations assertion for test-snap-branded-store=1"),
        )

    def test_validate_unknown_snap(self):
        raised = self.assertRaises(
            snapcraft.storeapi.errors.SnapNotFoundError,
            self.run_command,
            ["validate", "notfound", "core=3", "test-snap=4"],
        )

        self.assertThat(str(raised), Equals("Snap 'notfound' was not found."))

    def test_validate_bad_argument(self):
        raised = self.assertRaises(
            snapcraft.storeapi.errors.InvalidValidationRequestsError,
            self.run_command,
            ["validate", "core", "core=foo"],
        )

        self.assertThat(str(raised), Contains("format must be name=revision"))

    def test_validate_with_snap_name(self):
        self.fake_sign = fixtures.MockPatch(
            "snapcraft._store._sign_assertion", return_value=b""
        )
        self.useFixture(self.fake_sign)

        self.run_command(["validate", "core", "test-snap=3"])

        self.assertThat("core-test-snap-r3.assertion", FileExists())
        self.fake_sign.mock.assert_called_once_with(
            "test-snap=3",
            {
                "type": "validation",
                "authority-id": "abcd",
                "series": "16",
                "snap-id": "good",
                "approved-snap-id": "test-snap-snap-id",
                "approved-snap-revision": "3",
                "timestamp": mock.ANY,
                "revoked": "false",
            },
            None,
            "validations",
        )

    def test_revoke(self):
        self.fake_sign = fixtures.MockPatch(
            "snapcraft._store._sign_assertion", return_value=b""
        )
        self.useFixture(self.fake_sign)

        self.run_command(["validate", "core", "snap-1=3", "--revoke"])

        self.fake_sign.mock.assert_called_once_with(
            "snap-1=3",
            {
                "type": "validation",
                "authority-id": "abcd",
                "series": "16",
                "snap-id": "good",
                "approved-snap-id": "snap-id-1",
                "approved-snap-revision": "3",
                "timestamp": mock.ANY,
                "revoked": "true",
                "revision": "1",
            },
            None,
            "validations",
        )

    def test_no_revoke(self):
        self.fake_sign = fixtures.MockPatch(
            "snapcraft._store._sign_assertion", return_value=b""
        )
        self.useFixture(self.fake_sign)

        self.run_command(["validate", "core", "snap-1=3", "--no-revoke"])

        self.fake_sign.mock.assert_called_once_with(
            "snap-1=3",
            {
                "type": "validation",
                "authority-id": "abcd",
                "series": "16",
                "snap-id": "good",
                "approved-snap-id": "snap-id-1",
                "approved-snap-revision": "3",
                "timestamp": mock.ANY,
                "revoked": "false",
                "revision": "1",
            },
            None,
            "validations",
        )

    def test_validate_fallback_to_snap_id(self):
        self.fake_sign = fixtures.MockPatch(
            "snapcraft._store._sign_assertion", return_value=b""
        )
        self.useFixture(self.fake_sign)

        # XXXYYY is an imaginary snap-id.
        self.run_command(["validate", "core", "XXXYYY=3"])

        self.assertThat("core-XXXYYY-r3.assertion", FileExists())
        self.fake_sign.mock.assert_called_once_with(
            "XXXYYY=3",
            {
                "type": "validation",
                "authority-id": "abcd",
                "series": "16",
                "snap-id": "good",
                "approved-snap-id": "XXXYYY",
                "approved-snap-revision": "3",
                "timestamp": mock.ANY,
                "revoked": "false",
            },
            None,
            "validations",
        )

    def test_validate_with_revoke(self):
        self.fake_sign = fixtures.MockPatch(
            "snapcraft._store._sign_assertion", return_value=b""
        )
        self.useFixture(self.fake_sign)

        self.run_command(["validate", "core", "test-snap=3", "--revoke"])

        self.assertThat("core-test-snap-r3.assertion", FileExists())
        self.fake_sign.mock.assert_called_once_with(
            "test-snap=3",
            {
                "type": "validation",
                "authority-id": "abcd",
                "series": "16",
                "snap-id": "good",
                "approved-snap-id": "test-snap-snap-id",
                "approved-snap-revision": "3",
                "timestamp": mock.ANY,
                "revoked": "true",
            },
            None,
            "validations",
        )

    def test_validate_with_no_revoke(self):
        self.fake_sign = fixtures.MockPatch(
            "snapcraft._store._sign_assertion", return_value=b""
        )
        self.useFixture(self.fake_sign)

        self.run_command(["validate", "core", "test-snap=3", "--no-revoke"])

        self.assertThat("core-test-snap-r3.assertion", FileExists())
        self.fake_sign.mock.assert_called_once_with(
            "test-snap=3",
            {
                "type": "validation",
                "authority-id": "abcd",
                "series": "16",
                "snap-id": "good",
                "approved-snap-id": "test-snap-snap-id",
                "approved-snap-revision": "3",
                "timestamp": mock.ANY,
                "revoked": "false",
            },
            None,
            "validations",
        )
