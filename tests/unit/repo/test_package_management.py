# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2020 Canonical Ltd
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

from subprocess import PIPE, STDOUT, CalledProcessError
from unittest.mock import MagicMock, call, patch

import fixtures
from testtools.matchers import Equals

from snapcraft.internal.meta.package_management import PackageManagement, Repository
from snapcraft.internal.repo import errors, package_management
from tests import unit
from tests.fixture_setup.os_release import FakeOsRelease


class InstallAptRepositoryKeyIdTests(unit.TestCase):
    def setUp(self):
        super().setUp()
        self.useFixture(FakeOsRelease())

        self.fake_run = self.useFixture(
            fixtures.MockPatch("snapcraft.internal.repo.package_management.run")
        ).mock
        self.fake_run.return_value = MagicMock(stdout="imported: 1".encode())

    def test_no_key_id(self):
        repo = Repository(source="ppa:source", gpg_public_key_id=None)
        package_management._install_apt_repository_key_id(repo)

    def test_with_key_id(self):
        repo = Repository(source="ppa:source", gpg_public_key_id="KEYID")
        package_management._install_apt_repository_key_id(repo)

        self.fake_run.assert_has_calls(
            [
                call(
                    [
                        "sudo",
                        "apt-key",
                        "adv",
                        "--keyserver",
                        "keyserver.ubuntu.com",
                        "--recv-keys",
                        "KEYID",
                    ],
                    stdout=PIPE,
                    stderr=STDOUT,
                    check=True,
                )
            ]
        )

    def test_with_key_server(self):
        repo = Repository(
            source="ppa:source",
            gpg_public_key_id="KEYID",
            gpg_key_server="gpg.key.server",
        )
        package_management._install_apt_repository_key_id(repo)

        self.fake_run.assert_has_calls(
            [
                call(
                    [
                        "sudo",
                        "apt-key",
                        "adv",
                        "--keyserver",
                        "gpg.key.server",
                        "--recv-keys",
                        "KEYID",
                    ],
                    stdout=PIPE,
                    stderr=STDOUT,
                    check=True,
                )
            ]
        )


class InstallAptRepositoryKeyIdErrorTests(unit.TestCase):
    scenarios = [
        (
            "unknnown error",
            dict(error_message="unknown error", expected="unknown error"),
        ),
        (
            "unknnown error2",
            dict(error_message="unknown error2", expected="unknown error2"),
        ),
        (
            "ignore warning",
            dict(
                error_message="Warning: apt-key output should not be parsed (stdout is not a terminal)\nsome error",
                expected="some error",
            ),
        ),
        (
            "no data",
            dict(
                error_message="gpg: keyserver receive failed: No data",
                expected="GPG key ID 'KEYID' not found on key server 'gpg.key.server'",
            ),
        ),
        (
            "server filure",
            dict(
                error_message="gpg: keyserver receive failed: Server indicated a failure",
                expected="unable to establish connection to key server 'gpg.key.server'",
            ),
        ),
        (
            "timeout",
            dict(
                error_message="gpg: keyserver receive failed: Connection timed out",
                expected="unable to establish connection to key server 'gpg.key.server' (connection timed out)",
            ),
        ),
    ]

    def setUp(self):
        super().setUp()
        self.useFixture(FakeOsRelease())

        self.fake_run = self.useFixture(
            fixtures.MockPatch("snapcraft.internal.repo.package_management.run")
        ).mock
        self.fake_run.return_value = MagicMock(stdout=self.error_message.encode())

    def test_error(self):
        repo = Repository(
            source="ppa:source",
            gpg_public_key_id="KEYID",
            gpg_key_server="gpg.key.server",
        )

        error = CalledProcessError(1, ["apt-key"])
        error.output = self.error_message.encode()
        self.fake_run.side_effect = error

        error = self.assertRaises(
            errors.RepositoryKeyError,
            package_management._install_apt_repository_key_id,
            repo,
        )

        self.assertThat(error._repo, Equals(repo))
        self.assertThat(error._message, Equals(self.expected))


class InstallAptRepositoryKeyBlockTests(unit.TestCase):
    def setUp(self):
        super().setUp()
        self.useFixture(FakeOsRelease())

    @patch("snapcraft.internal.repo.package_management.run")
    def test_no_key_id(self, mock_run):
        gpg_public_key = None
        repo = Repository(source="ppa:source", gpg_public_key=gpg_public_key)
        package_management._install_apt_repository_key_block(repo)

        mock_run.assert_not_called()

    @patch("snapcraft.internal.repo.package_management.run")
    def test_with_key_id(self, mock_run):
        gpg_public_key = "-----BEGIN PGP PUBLIC KEY BLOCK-----\n...\n-----END PGP PUBLIC KEY BLOCK-----"
        repo = Repository(source="ppa:source", gpg_public_key=gpg_public_key)
        package_management._install_apt_repository_key_block(repo)

        mock_run.assert_has_calls(
            [
                call(
                    ["sudo", "apt-key", "add", "-"],
                    input=gpg_public_key.encode(),
                    stdout=PIPE,
                    stderr=STDOUT,
                    check=True,
                )
            ]
        )


class InstallAptRepositoryKeyBlockErrorTests(unit.TestCase):
    scenarios = [
        (
            "unknnown error",
            dict(error_message="unknown error", expected="unknown error"),
        ),
        (
            "unknnown error2",
            dict(error_message="unknown error2", expected="unknown error2"),
        ),
        (
            "ignore warning",
            dict(
                error_message="Warning: apt-key output should not be parsed (stdout is not a terminal)\nsome error",
                expected="some error",
            ),
        ),
    ]

    def setUp(self):
        super().setUp()
        self.useFixture(FakeOsRelease())

    @patch("snapcraft.internal.repo.package_management.run")
    def test_error_with_process_error(self, mock_run):
        gpg_public_key = "-----BEGIN PGP PUBLIC KEY BLOCK-----\n...\n-----END PGP PUBLIC KEY BLOCK-----"
        repo = Repository(source="ppa:source", gpg_public_key=gpg_public_key)

        error = CalledProcessError(1, ["apt-key"])
        error.output = self.error_message.encode()
        mock_run.side_effect = error

        error = self.assertRaises(
            errors.RepositoryKeyError,
            package_management._install_apt_repository_key_block,
            repo,
        )

        self.assertThat(error._repo, Equals(repo))
        self.assertThat(error._message, Equals(self.expected))


class InstallAptRepositoryTests(unit.TestCase):
    def setUp(self):
        super().setUp()

    @patch("snapcraft.internal.repo.package_management.run")
    def test_source(self, mock_run):
        repo = Repository(source="ppa:source")
        package_management._install_apt_repository(repo)

        mock_run.assert_has_calls(
            [
                call(
                    ["sudo", "apt-add-repository", "-y", "ppa:source"],
                    stdout=PIPE,
                    stderr=STDOUT,
                    check=True,
                )
            ]
        )


class InstallAptRepositoryScTests(unit.TestCase):
    scenarios = [
        (
            "minimal ppa",
            dict(
                source="test_ppa",
                gpg_public_key=None,
                gpg_public_key_id=None,
                gpg_key_server=None,
                expected_calls=[
                    call(
                        ["sudo", "apt-add-repository", "-y", "test_ppa"],
                        stdout=PIPE,
                        stderr=STDOUT,
                        check=True,
                    )
                ],
            ),
        ),
        (
            "minimal url",
            dict(
                source="deb http://archive.canonical.com/ubuntu bionic partner",
                gpg_public_key=None,
                gpg_public_key_id=None,
                gpg_key_server=None,
                expected_calls=[
                    call(
                        [
                            "sudo",
                            "apt-add-repository",
                            "-y",
                            "deb http://archive.canonical.com/ubuntu bionic partner",
                        ],
                        stdout=PIPE,
                        stderr=STDOUT,
                        check=True,
                    )
                ],
            ),
        ),
        (
            "url + gpg-public-key",
            dict(
                source="deb http://archive.canonical.com/ubuntu bionic partner",
                gpg_public_key="-----BEGIN PGP PUBLIC KEY BLOCK-----\n...\n-----END PGP PUBLIC KEY BLOCK-----",
                gpg_public_key_id=None,
                gpg_key_server=None,
                expected_calls=[
                    call(
                        ["sudo", "apt-key", "add", "-"],
                        input=b"-----BEGIN PGP PUBLIC KEY BLOCK-----\n...\n-----END PGP PUBLIC KEY BLOCK-----",
                        stdout=PIPE,
                        stderr=STDOUT,
                        check=True,
                    ),
                    call(
                        [
                            "sudo",
                            "apt-add-repository",
                            "-y",
                            "deb http://archive.canonical.com/ubuntu bionic partner",
                        ],
                        stdout=PIPE,
                        stderr=STDOUT,
                        check=True,
                    ),
                ],
            ),
        ),
        (
            "url + gpg-public-key-id",
            dict(
                source="deb http://archive.canonical.com/ubuntu bionic partner",
                gpg_public_key=None,
                gpg_public_key_id="0ab215679c571d1c8325275b9bdb3d89ce49ec21",
                gpg_key_server=None,
                expected_calls=[
                    call(
                        [
                            "sudo",
                            "apt-key",
                            "adv",
                            "--keyserver",
                            "keyserver.ubuntu.com",
                            "--recv-keys",
                            "0ab215679c571d1c8325275b9bdb3d89ce49ec21",
                        ],
                        stdout=PIPE,
                        stderr=STDOUT,
                        check=True,
                    ),
                    call(
                        [
                            "sudo",
                            "apt-add-repository",
                            "-y",
                            "deb http://archive.canonical.com/ubuntu bionic partner",
                        ],
                        stdout=PIPE,
                        stderr=STDOUT,
                        check=True,
                    ),
                ],
            ),
        ),
        (
            "url + gpg-public-key-id + gpg-key-server",
            dict(
                source="deb http://archive.canonical.com/ubuntu bionic partner",
                gpg_public_key="0ab215679c571d1c8325275b9bdb3d89ce49ec21",
                gpg_public_key_id=None,
                gpg_key_server="keyserver.ubuntu.com",
                expected_calls=[
                    call(
                        ["sudo", "apt-key", "add", "-"],
                        input=b"0ab215679c571d1c8325275b9bdb3d89ce49ec21",
                        stdout=PIPE,
                        stderr=STDOUT,
                        check=True,
                    ),
                    call(
                        [
                            "sudo",
                            "apt-add-repository",
                            "-y",
                            "deb http://archive.canonical.com/ubuntu bionic partner",
                        ],
                        stdout=PIPE,
                        stderr=STDOUT,
                        check=True,
                    ),
                ],
            ),
        ),
    ]

    def setUp(self):
        super().setUp()
        self.useFixture(FakeOsRelease())

        self.fake_run = self.useFixture(
            fixtures.MockPatch("snapcraft.internal.repo.package_management.run")
        ).mock
        self.fake_run.return_value = MagicMock(stdout="imported: 1".encode())

    def test_repo_install(self):
        repo = Repository(
            source=self.source,
            gpg_public_key=self.gpg_public_key,
            gpg_public_key_id=self.gpg_public_key_id,
            gpg_key_server=self.gpg_key_server,
        )

        package_management._install_repository(repo)

        self.assertThat(self.fake_run.mock_calls, Equals(self.expected_calls))


class GetBuildToolsTests(unit.TestCase):
    def setUp(self):
        super().setUp()

    @patch(
        "snapcraft.internal.repo.package_management.OsRelease.id", return_value="ubuntu"
    )
    def test_ubuntu(self, mock_id):
        build_tools = package_management._get_repo_build_tools()

        self.assertThat(build_tools, Equals(["software-properties-common"]))


class ConfigurePackageManagerTests(unit.TestCase):
    @patch(
        "snapcraft.internal.repo.package_management.OsRelease.id", return_value="centos"
    )
    def test_non_ubuntu_no_repos(self, mock_id):
        # Make sure there is no exception thrown.
        package_management.configure_package_manager(PackageManagement())

    @patch(
        "snapcraft.internal.repo.package_management.OsRelease.id", return_value="centos"
    )
    def test_non_ubuntu_with_repos(self, mock_id):
        # Make sure exception is thrown.
        self.assertRaises(
            RuntimeError,
            package_management.configure_package_manager,
            PackageManagement(repositories=[Repository(source="ppa:test-ppa")]),
        )

    @patch(
        "snapcraft.internal.repo.package_management.OsRelease.id", return_value="ubuntu"
    )
    @patch("snapcraft.internal.repo.package_management.Repo.install_build_packages")
    @patch("snapcraft.internal.repo.package_management.run")
    def test_ubuntu_with_repos(self, mock_run, mock_repo, mock_id):
        package_management.configure_package_manager(
            PackageManagement(
                repositories=[
                    Repository(source="ppa:test-ppa"),
                    Repository(source="ppa:test-ppa2"),
                ]
            )
        )

        self.assertThat(
            mock_repo.mock_calls,
            Equals([call.install_build_packages(["software-properties-common"])]),
        )
        self.assertThat(
            mock_run.mock_calls,
            Equals(
                [
                    call(
                        ["sudo", "apt-add-repository", "-y", "ppa:test-ppa"],
                        stderr=STDOUT,
                        stdout=PIPE,
                        check=True,
                    ),
                    call(
                        ["sudo", "apt-add-repository", "-y", "ppa:test-ppa2"],
                        stderr=STDOUT,
                        stdout=PIPE,
                        check=True,
                    ),
                ]
            ),
        )
