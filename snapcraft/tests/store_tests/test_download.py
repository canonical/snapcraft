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

from snapcraft import (
    config,
    storeapi,
)
from snapcraft.tests import store_tests


class TestDownloadLogin(store_tests.TestCase):

    def test_download_without_credentials(self):
        self.assertRaises(storeapi.InvalidCredentials,
                          self.download, 'basic')


class TestSearchPackage(store_tests.TestCase):

    def setUp(self):
        super().setUp()
        self.addCleanup(self.logout)
        self.login()
        self.conf = config.Config()
        self.cpi = storeapi.CPIClient(self.conf)

    def test_search_known_package(self):
        # We rely on a package that has been published for TEST_USER_EMAIL
        pkgs = self.cpi.search_package('femto', 'stable', 'amd64')
        self.assertEqual(1, len(pkgs))

    def test_search_unknown_package(self):
        pkgs = self.cpi.search_package('dont-exist', 'dont-exist', 'amd64')
        self.assertEqual(0, len(pkgs))


class TestDownload(store_tests.TestCase):

    def setUp(self):
        super().setUp()
        self.addCleanup(self.logout)
        self.login()

    def assertLog(self, expected):
        self.assertEqual(expected, self.logger.output)

    def test_download_unknwon_package(self):
        exc = self.assertRaises(
            storeapi.SnapNotFound,
            self.download, 'gloo', 'bee')
        self.assertEqual('gloo', exc.name)
        self.assertEqual('bee', exc.channel)
        self.assertEqual('amd64', exc.arch)  # default value

    def test_download_works(self):
        pkg_name = 'femto'
        self.download(pkg_name, 'stable', 'downloaded.snap')
        self.assertTrue(os.path.exists('downloaded.snap'))
        self.assertLog('''Getting details for {pkg_name}
Downloading {pkg_name}
Successfully downloaded {pkg_name} at downloaded.snap
'''.format(pkg_name=pkg_name))

    def test_download_twice_reuse_existing(self):
        pkg_name = 'femto'
        self.download(pkg_name, 'stable', 'downloaded.snap')
        self.assertTrue(os.path.exists('downloaded.snap'))
        self.assertLog('''Getting details for {pkg_name}
Downloading {pkg_name}
Successfully downloaded {pkg_name} at downloaded.snap
'''.format(pkg_name=pkg_name))
        # Try downloading again
        self.download(pkg_name, 'stable', 'downloaded.snap')
        self.assertLog('''Getting details for {pkg_name}
Downloading {pkg_name}
Successfully downloaded {pkg_name} at downloaded.snap
Getting details for {pkg_name}
Already downloaded {pkg_name} at downloaded.snap
'''.format(pkg_name=pkg_name))

    def test_redownload_on_mismatch(self):
        pkg_name = 'femto'
        self.download(pkg_name, 'stable', 'downloaded.snap')
        self.assertTrue(os.path.exists('downloaded.snap'))
        self.assertLog('''Getting details for {pkg_name}
Downloading {pkg_name}
Successfully downloaded {pkg_name} at downloaded.snap
'''.format(pkg_name=pkg_name))
        # Clobber the downloaded file
        with open('downloaded.snap', 'w') as f:
            f.write('Sabotage !')
        # Try downloading again
        self.download(pkg_name, 'stable', 'downloaded.snap')
        self.assertLog('''Getting details for {pkg_name}
Downloading {pkg_name}
Successfully downloaded {pkg_name} at downloaded.snap
Getting details for {pkg_name}
Downloading {pkg_name}
Successfully downloaded {pkg_name} at downloaded.snap
'''.format(pkg_name=pkg_name))
