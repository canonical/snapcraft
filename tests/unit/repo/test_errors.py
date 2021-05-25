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

from snapcraft.internal.repo import errors


class TestErrorFormatting:

    scenarios = (
        (
            "SnapdConnectionError",
            {
                "exception_class": errors.SnapdConnectionError,
                "kwargs": {"snap_name": "test", "url": "url"},
                "expected_message": (
                    "Failed to get information for snap 'test': "
                    "could not connect to 'url'."
                ),
            },
        ),
    )

    def test_error_formatting(self, exception_class, kwargs, expected_message):
        assert str(exception_class(**kwargs)) == expected_message


class TestAptGPGKeyInstallError:

    scenarios = [
        (
            "AptGPGKeyInstallError basic",
            {
                "exception_class": errors.AptGPGKeyInstallError,
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
                "exception_class": errors.AptGPGKeyInstallError,
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
                "exception_class": errors.AptGPGKeyInstallError,
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
                "exception_class": errors.AptGPGKeyInstallError,
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
                "exception_class": errors.AptGPGKeyInstallError,
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
                "exception_class": errors.AptGPGKeyInstallError,
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

    def test_snapcraft_exception_handling(
        self,
        exception_class,
        expected_brief,
        expected_details,
        expected_docs_url,
        expected_reportable,
        expected_resolution,
        kwargs,
    ):
        exception = exception_class(**kwargs)

        assert exception.get_brief() == expected_brief
        assert exception.get_resolution() == expected_resolution
        assert exception.get_details() == expected_details
        assert exception.get_docs_url() == expected_docs_url
        assert exception.get_reportable() == expected_reportable


def test_packages_not_found_error():
    exception = errors.PackagesNotFoundError(["foo"])

    assert (
        exception.get_brief()
        == "Failed to find installation candidate for packages: 'foo'"
    )
    assert (
        exception.get_resolution()
        == "Verify APT repository configuration and package names are correct."
    )
    assert exception.get_details() is None
    assert exception.get_docs_url() is None
    assert exception.get_reportable() is False


def test_multiple_packages_not_found_error():
    exception = errors.PackagesNotFoundError(["foo", "foo2", "foo3"])

    assert (
        exception.get_brief()
        == "Failed to find installation candidate for packages: 'foo', 'foo2', and 'foo3'"
    )
    assert (
        exception.get_resolution()
        == "Verify APT repository configuration and package names are correct."
    )
    assert exception.get_details() is None
    assert exception.get_docs_url() is None
    assert exception.get_reportable() is False
