# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2016 Canonical Ltd
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

from snapcraft.internal import sources

from snapcraft import tests


class TestDeb(tests.FakeFileHTTPServerBasedTestCase):

    def setUp(self):
        super().setUp()

        patcher = mock.patch('debian.debfile.DebFile')
        self.mock_deb = patcher.start()
        self.addCleanup(patcher.stop)

    def test_pull_debfile_must_download_and_extract(self):
        dest_dir = 'src'
        os.makedirs(dest_dir)
        deb_file_name = 'test.deb'
        source = 'http://{}:{}/{file_name}'.format(
            *self.server.server_address, file_name=deb_file_name)
        deb_source = sources.Deb(source, dest_dir)

        deb_source.pull()

        self.mock_deb.assert_called_once_with(
            os.path.join(deb_source.source_dir, deb_file_name))

    def test_extract_and_keep_debfile(self):
        deb_file_name = 'test.deb'
        source = 'http://{}:{}/{file_name}'.format(
            *self.server.server_address, file_name=deb_file_name)
        dest_dir = os.path.abspath(os.curdir)
        deb_source = sources.Deb(source, dest_dir)

        deb_source.download()
        deb_source.provision(dst=dest_dir, keep_deb=True)

        deb_download = os.path.join(deb_source.source_dir, deb_file_name)
        self.mock_deb.assert_called_once_with(
            os.path.join(deb_source.source_dir, deb_file_name))

        with open(deb_download, 'r') as deb_file:
            self.assertEqual('Test fake compressed file', deb_file.read())
