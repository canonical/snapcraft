#
# Copyright (C) 2016 Canonical Ltd
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
import unittest

import fixtures
import requests
import testscenarios

from snapcraft import (
    config,
    storeapi,
)
from snapcraft.tests import store_tests


load_tests = testscenarios.load_tests_apply_scenarios


class TestDownloadLogin(store_tests.TestCase):

    scenarios = (('OAuth', dict(with_macaroons=False)),
                 ('macaroons', dict(with_macaroons=True)),
                 )

    def setUp(self):
        super().setUp()
        if self.with_macaroons:
            self.useFixture(
                fixtures.EnvironmentVariable('SNAPCRAFT_WITH_MACAROONS', '1'))
        else:
            self.useFixture(
                fixtures.EnvironmentVariable('SNAPCRAFT_WITH_MACAROONS', None))

    def test_download_without_credentials(self):

        self.assertRaises(storeapi.InvalidCredentials,
                          self.download, 'basic')


class TestSearchPackage(store_tests.TestCase):

    scenarios = (('OAuth', dict(with_macaroons=False)),
                 ('macaroons', dict(with_macaroons=True)),
                 )

    def setUp(self):
        super().setUp()
        if self.with_macaroons:
            self.useFixture(
                fixtures.EnvironmentVariable('SNAPCRAFT_WITH_MACAROONS', '1'))
        else:
            self.useFixture(
                fixtures.EnvironmentVariable('SNAPCRAFT_WITH_MACAROONS', None))
        self.addCleanup(self.logout)
        self.login()
        self.conf = config.Config()
        self.cpi = storeapi.CPIClient(self.conf)

    def test_search_known_package(self):
        # Rely on ubuntu-core as it must exists at all times (or we have other
        # problems than a failing test).
        # response = self.cpi.search_package('ubuntu-core', 'stable', 'amd64')
        # FIXME: Doh, no 'ubuntu-core' on staging... -- vila 2016-04-26
        pkgs = self.cpi.search_package('test-package', 'stable', 'amd64')
        self.assertEqual(1, len(pkgs))


class TestDownload(store_tests.TestCase):

    scenarios = (('OAuth', dict(with_macaroons=False)),
                 # ('macaroons', dict(with_macaroons=True)),
                 )

    def setUp(self):
        super().setUp()
        if self.with_macaroons:
            self.useFixture(
                fixtures.EnvironmentVariable('SNAPCRAFT_WITH_MACAROONS', '1'))
        else:
            self.useFixture(
                fixtures.EnvironmentVariable('SNAPCRAFT_WITH_MACAROONS', None))
        self.addCleanup(self.logout)
        self.login()

    def test_download_works(self):
        self.download('test-package', 'stable', 'downloaded.snap')
        self.assertTrue(os.path.exists('downloaded.snap'))
        self.assertIn('Downloading test-package',
                      self.logger.output)
        self.assertIn('Successfully downloaded test-package'
                      ' at downloaded.snap',
                      self.logger.output)

    def test_download_unknwon_package(self):
        exc = self.assertRaises(
            storeapi.SnapNotFound,
            self.download, 'gloo', 'bee')
        self.assertEqual('gloo', exc.name)
        self.assertEqual('bee', exc.channel)
        self.assertEqual('amd64', exc.arch)  # default value
