# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018 Canonical Ltd
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
from testtools.matchers import Equals

from snapcraft.internal.repo import errors
from tests import unit


class ErrorFormattingTestCase(unit.TestCase):

    scenarios = (
        (
            "SnapdConnectionError",
            {
                "exception": errors.SnapdConnectionError,
                "kwargs": {"snap_name": "test", "url": "url"},
                "expected_message": (
                    "Failed to get information for snap 'test': "
                    "could not connect to 'url'."
                ),
            },
        ),
    )

    def test_error_formatting(self):
        self.assertThat(
            str(self.exception(**self.kwargs)), Equals(self.expected_message)
        )


class AptGPGKeyInstallErrorTests(unit.TestCase):

    scenarios = [
        (
            "AptGPGKeyInstallError basic",
            {
                "exception": errors.AptGPGKeyInstallError,
                "kwargs": {"output": "some error", "key": "fake key"},
                "expected_brief": "Failed to install GPG key: some error",
                "expected_resolution": "Verify any configured GPG keys.",
                "expected_details": "GPG key:\nfake key\n",
                "expected_docs_url": None,
                "expected_reportable": False,
            },
        ),
        (
            "AptGPGKeyInstallError basic with warning",
            {
                "exception": errors.AptGPGKeyInstallError,
                "kwargs": {
                    "output": "Warning: apt-key output should not be parsed (stdout is not a terminal)\nsome error",
                    "key": "fake key",
                },
                "expected_brief": "Failed to install GPG key: some error",
                "expected_resolution": "Verify any configured GPG keys.",
                "expected_details": "GPG key:\nfake key\n",
                "expected_docs_url": None,
                "expected_reportable": False,
            },
        ),
        (
            "AptGPGKeyInstallError basic with warning",
            {
                "exception": errors.AptGPGKeyInstallError,
                "kwargs": {
                    "output": "Warning: apt-key output should not be parsed (stdout is not a terminal)\nsome error",
                    "key": "fake key",
                },
                "expected_brief": "Failed to install GPG key: some error",
                "expected_resolution": "Verify any configured GPG keys.",
                "expected_details": "GPG key:\nfake key\n",
                "expected_docs_url": None,
                "expected_reportable": False,
            },
        ),
        (
            "AptGPGKeyInstallError keyserver no data",
            {
                "exception": errors.AptGPGKeyInstallError,
                "kwargs": {
                    "output": "gpg: keyserver receive failed: No data",
                    "key_id": "fake-key-id",
                    "key_server": "fake.key.server",
                },
                "expected_brief": "Failed to install GPG key: GPG key 'fake-key-id' not found on key server 'fake.key.server'",
                "expected_resolution": "Verify any configured GPG keys.",
                "expected_details": "GPG key ID: fake-key-id\nGPG key server: fake.key.server\n",
                "expected_docs_url": None,
                "expected_reportable": False,
            },
        ),
        (
            "AptGPGKeyInstallError keyserver failure",
            {
                "exception": errors.AptGPGKeyInstallError,
                "kwargs": {
                    "output": "gpg: keyserver receive failed: Server indicated a failure",
                    "key_id": "fake-key-id",
                    "key_server": "fake.key.server",
                },
                "expected_brief": "Failed to install GPG key: unable to establish connection to key server 'fake.key.server'",
                "expected_resolution": "Verify any configured GPG keys.",
                "expected_details": "GPG key ID: fake-key-id\nGPG key server: fake.key.server\n",
                "expected_docs_url": None,
                "expected_reportable": False,
            },
        ),
        (
            "AptGPGKeyInstallError keyserver timeout",
            {
                "exception": errors.AptGPGKeyInstallError,
                "kwargs": {
                    "output": "gpg: keyserver receive failed: Connection timed out",
                    "key_id": "fake-key-id",
                    "key_server": "fake.key.server",
                },
                "expected_brief": "Failed to install GPG key: unable to establish connection to key server 'fake.key.server' (connection timed out)",
                "expected_resolution": "Verify any configured GPG keys.",
                "expected_details": "GPG key ID: fake-key-id\nGPG key server: fake.key.server\n",
                "expected_docs_url": None,
                "expected_reportable": False,
            },
        ),
    ]

    def test_snapcraft_exception_handling(self):
        exception = self.exception(**self.kwargs)
        self.assertThat(exception.get_brief(), Equals(self.expected_brief))
        self.assertThat(exception.get_resolution(), Equals(self.expected_resolution))
        self.assertThat(exception.get_details(), Equals(self.expected_details))
        self.assertThat(exception.get_docs_url(), Equals(self.expected_docs_url))
        self.assertThat(exception.get_reportable(), Equals(self.expected_reportable))
