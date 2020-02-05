# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019 Canonical Ltd
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

from snapcraft.internal.meta import errors
from snapcraft.internal.meta.package_management import PackageManagement, Repository
from tests import unit


class ValidRepositoriesTests(unit.TestCase):
    scenarios = [
        (
            "minimal ppa",
            dict(
                source="test_ppa",
                gpg_public_key=None,
                gpg_public_key_id=None,
                gpg_key_server=None,
            ),
        ),
        (
            "minimal url",
            dict(
                source="deb http://archive.canonical.com/ubuntu bionic partner",
                gpg_public_key=None,
                gpg_public_key_id=None,
                gpg_key_server=None,
            ),
        ),
        (
            "url + gpg-public-key",
            dict(
                source="deb http://archive.canonical.com/ubuntu bionic partner",
                gpg_public_key="-----BEGIN PGP PUBLIC KEY BLOCK-----\n...\n-----END PGP PUBLIC KEY BLOCK-----",
                gpg_public_key_id=None,
                gpg_key_server=None,
            ),
        ),
        (
            "url + gpg-public-key-id",
            dict(
                source="deb http://archive.canonical.com/ubuntu bionic partner",
                gpg_public_key=None,
                gpg_public_key_id="0ab215679c571d1c8325275b9bdb3d89ce49ec21",
                gpg_key_server=None,
            ),
        ),
        (
            "url + gpg-public-key-id + gpg-key-server",
            dict(
                source="deb http://archive.canonical.com/ubuntu bionic partner",
                gpg_public_key="0ab215679c571d1c8325275b9bdb3d89ce49ec21",
                gpg_public_key_id=None,
                gpg_key_server="keyserver.ubuntu.com",
            ),
        ),
    ]

    def test_repo_init(self):
        repo = Repository(
            source=self.source,
            gpg_public_key=self.gpg_public_key,
            gpg_public_key_id=self.gpg_public_key_id,
            gpg_key_server=self.gpg_key_server,
        )
        repo.validate()

        self.assertThat(repo.source, Equals(self.source))
        self.assertThat(repo.gpg_public_key, Equals(self.gpg_public_key))
        self.assertThat(repo.gpg_public_key_id, Equals(self.gpg_public_key_id))
        self.assertThat(repo.gpg_key_server, Equals(self.gpg_key_server))

    def test_repo_from_dict(self):
        repo_dict = dict(source=self.source)
        if self.gpg_public_key:
            repo_dict["gpg-public-key"] = self.gpg_public_key
        if self.gpg_public_key_id:
            repo_dict["gpg-public-key-id"] = self.gpg_public_key_id
        if self.gpg_key_server:
            repo_dict["gpg-key-server"] = self.gpg_key_server

        repo = Repository.from_dict(repo_dict)
        repo.validate()

        self.assertThat(repo.source, Equals(self.source))
        self.assertThat(repo.gpg_public_key, Equals(self.gpg_public_key))
        self.assertThat(repo.gpg_public_key_id, Equals(self.gpg_public_key_id))
        self.assertThat(repo.gpg_key_server, Equals(self.gpg_key_server))

    def test_repo_from_object(self):
        repo_dict = dict(source=self.source)
        if self.gpg_public_key:
            repo_dict["gpg-public-key"] = self.gpg_public_key
        if self.gpg_public_key_id:
            repo_dict["gpg-public-key-id"] = self.gpg_public_key_id
        if self.gpg_key_server:
            repo_dict["gpg-key-server"] = self.gpg_key_server

        repo = Repository.from_object(repo_dict)
        repo.validate()

        self.assertThat(repo.source, Equals(self.source))
        self.assertThat(repo.gpg_public_key, Equals(self.gpg_public_key))
        self.assertThat(repo.gpg_public_key_id, Equals(self.gpg_public_key_id))
        self.assertThat(repo.gpg_key_server, Equals(self.gpg_key_server))


class InvalidRepositoryTests(unit.TestCase):
    def test_invalld_object_none(self):
        repo_object = None

        error = self.assertRaises(
            errors.RepositoryValidationError, Repository.from_object, repo_object
        )

        self.assertThat(
            error.get_brief(), Equals("Invalid repository: empty definition")
        )

    def test_invalid_object_missing_source(self):
        repo_object = {"source": ""}

        error = self.assertRaises(
            errors.RepositoryValidationError, Repository.from_object, repo_object
        )

        self.assertThat(
            error.get_brief(),
            Equals("Invalid repository: 'source' undefined for {'source': ''}"),
        )

    def test_invalid_object_empty_source(self):
        repo_object = {"source": ""}

        error = self.assertRaises(
            errors.RepositoryValidationError, Repository.from_object, repo_object
        )

        self.assertThat(
            error.get_brief(),
            Equals("Invalid repository: 'source' undefined for {'source': ''}"),
        )

    def test_invalid_object(self):
        repo_object = "invalid-repository"

        error = self.assertRaises(
            errors.RepositoryValidationError, Repository.from_object, repo_object
        )

        self.assertThat(
            error.get_brief(),
            Equals("Invalid repository: unknown syntax for 'invalid-repository'"),
        )


class PackageManagementTests(unit.TestCase):
    def test_from_object_none(self):
        pm_object = None

        pm = PackageManagement.from_object(pm_object)
        self.assertThat(pm.repositories, Equals([]))

    def test_from_object_list(self):
        pm_object = ["invalid-list"]

        error = self.assertRaises(
            errors.PackageManagementValidationError,
            PackageManagement.from_object,
            pm_object,
        )

        self.assertThat(
            error.get_brief(),
            Equals("Invalid package-management: unknown syntax for ['invalid-list']"),
        )

    def test_from_object_string(self):
        pm_object = "invalid-string"

        error = self.assertRaises(
            errors.PackageManagementValidationError,
            PackageManagement.from_object,
            pm_object,
        )

        self.assertThat(
            error.get_brief(),
            Equals("Invalid package-management: unknown syntax for 'invalid-string'"),
        )

    def test_from_object_empty(self):
        pm_object = dict()

        pm = PackageManagement.from_object(pm_object)
        self.assertThat(pm.repositories, Equals([]))

    def test_from_object_empty_list(self):
        pm_object = dict(repositories=[])

        pm = PackageManagement.from_object(pm_object)
        self.assertThat(pm.repositories, Equals([]))

    def test_from_object_one_repo(self):
        pm_object = dict(repositories=[dict(source="ppa:test-ppa")])

        pm = PackageManagement.from_object(pm_object)
        self.assertThat(pm.repositories, Equals([Repository(source="ppa:test-ppa")]))

    def test_from_object_two_repos(self):
        pm_object = dict(
            repositories=[dict(source="ppa:test-ppa"), dict(source="ppa:test-ppa2")]
        )

        pm = PackageManagement.from_object(pm_object)
        self.assertThat(
            pm.repositories,
            Equals(
                [Repository(source="ppa:test-ppa"), Repository(source="ppa:test-ppa2")]
            ),
        )
