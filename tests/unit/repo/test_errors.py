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

from snapcraft.internal.meta.package_management import Repository
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


class SnapcraftExceptionTests(unit.TestCase):

    scenarios = [
        (
            "RepositoryError",
            {
                "exception": errors.RepositoryError,
                "kwargs": dict(
                    repo=Repository(
                        source="ppa:test-source",
                        gpg_public_key=None,
                        gpg_public_key_id="TEST-GPG-KEYID",
                        gpg_key_server="test.keyserver.com",
                    ),
                    message="some reason",
                ),
                "expected_brief": "Error adding repository: some reason",
                "expected_resolution": "Please ensure that the repository key configuration for Repository(source='ppa:test-source', gpg-public-key=None, gpg-public-key-id='TEST-GPG-KEYID', gpg-key-server=test.keyserver.com) is valid.",
                "expected_details": None,
                "expected_docs_url": "<TODO>",
                "expected_reportable": False,
            },
        ),
        (
            "RepositoryKeyError",
            {
                "exception": errors.RepositoryKeyError,
                "kwargs": dict(
                    repo=Repository(
                        source="ppa:test-source",
                        gpg_public_key=None,
                        gpg_public_key_id="TEST-GPG-KEYID",
                        gpg_key_server="test.keyserver.com",
                    ),
                    message="some reason",
                ),
                "expected_brief": "Error adding repository key: some reason",
                "expected_resolution": "Please ensure that the repository key configuration for Repository(source='ppa:test-source', gpg-public-key=None, gpg-public-key-id='TEST-GPG-KEYID', gpg-key-server=test.keyserver.com) is valid and that key server 'test.keyserver.com' is accessible.",
                "expected_details": None,
                "expected_docs_url": "<TODO>",
                "expected_reportable": False,
            },
        ),
    ]

    def test_snapcraft_exception_handling(self):
        exception = self.exception(**self.kwargs)
        self.assertEquals(self.expected_brief, exception.get_brief())
        self.assertEquals(self.expected_resolution, exception.get_resolution())
        self.assertEquals(self.expected_details, exception.get_details())
        self.assertEquals(self.expected_docs_url, exception.get_docs_url())
        self.assertEquals(self.expected_reportable, exception.get_reportable())
