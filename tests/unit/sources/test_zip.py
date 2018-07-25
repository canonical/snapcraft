# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2018 Canonical Ltd
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

from testtools.matchers import Equals

from snapcraft.internal import sources
from tests import unit


class TestZip(unit.FakeFileHTTPServerBasedTestCase):
    @mock.patch("zipfile.ZipFile")
    def test_pull_zipfile_must_download_and_extract(self, mock_zip):
        dest_dir = "src"
        os.makedirs(dest_dir)
        zip_file_name = "test.zip"
        source = "http://{}:{}/{file_name}".format(
            *self.server.server_address, file_name=zip_file_name
        )
        zip_source = sources.Zip(source, dest_dir)

        zip_source.pull()

        mock_zip.assert_called_once_with(
            os.path.join(zip_source.source_dir, zip_file_name), "r"
        )

    @mock.patch("zipfile.ZipFile")
    def test_extract_and_keep_zipfile(self, mock_zip):
        zip_file_name = "test.zip"
        source = "http://{}:{}/{file_name}".format(
            *self.server.server_address, file_name=zip_file_name
        )
        dest_dir = os.path.abspath(os.curdir)
        zip_source = sources.Zip(source, dest_dir)

        zip_source.download()
        zip_source.provision(dst=dest_dir, keep_zip=True)

        zip_download = os.path.join(zip_source.source_dir, zip_file_name)
        mock_zip.assert_called_once_with(zip_download, "r")

        with open(zip_download, "r") as zip_file:
            self.assertThat(zip_file.read(), Equals("Test fake file"))

    def test_has_source_handler_entry(self):
        self.assertTrue(sources._source_handler["zip"] is sources.Zip)
