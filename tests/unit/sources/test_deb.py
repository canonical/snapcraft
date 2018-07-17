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
import sys
from unittest import mock

from testtools.matchers import Equals

from snapcraft.internal import sources
from tests import unit


class TestDeb(unit.FakeFileHTTPServerBasedTestCase):
    def setUp(self):
        super().setUp()

        patcher = mock.patch("debian.arfile.ArFile")
        self.mock_ar = patcher.start()
        self.mock_ar.return_value.getnames.return_value = [
            "data.tar.gz",
            "control.tar.gz",
            "debian-binary",
        ]
        self.addCleanup(patcher.stop)

        patcher = mock.patch("tarfile.open")
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_pull_debfile_must_download_and_extract(self):
        dest_dir = "src"
        os.makedirs(dest_dir)
        deb_file_name = "test.deb"
        source = "http://{}:{}/{file_name}".format(
            *self.server.server_address, file_name=deb_file_name
        )
        deb_source = sources.Deb(source, dest_dir)

        deb_source.pull()

        self.mock_ar.assert_called_once_with(
            os.path.join(deb_source.source_dir, deb_file_name)
        )

    def test_extract_and_keep_debfile(self):
        deb_file_name = "test.deb"
        source = "http://{}:{}/{file_name}".format(
            *self.server.server_address, file_name=deb_file_name
        )
        dest_dir = os.path.abspath(os.curdir)
        deb_source = sources.Deb(source, dest_dir)

        deb_source.download()
        deb_source.provision(dst=dest_dir, keep_deb=True)

        deb_download = os.path.join(deb_source.source_dir, deb_file_name)
        self.mock_ar.assert_called_once_with(
            os.path.join(deb_source.source_dir, deb_file_name)
        )

        with open(deb_download, "r") as deb_file:
            self.assertThat(deb_file.read(), Equals("Test fake file"))

    def test_has_source_handler_entry_on_linux(self):
        if sys.platform == "linux":
            self.assertTrue(sources._source_handler["deb"] is sources.Deb)
        else:
            self.assertRaises(KeyError, sources._source_handler["deb"])

    def test_invalid_deb(self):
        self.mock_ar.return_value.getnames.return_value = [
            "control.tar.gz",
            "debian-binary",
        ]

        deb_file_name = "test.deb"
        source = "http://{}:{}/{file_name}".format(
            *self.server.server_address, file_name=deb_file_name
        )
        dest_dir = os.path.abspath(os.curdir)
        deb_source = sources.Deb(source, dest_dir)

        deb_source.download()

        self.assertRaises(
            sources.errors.InvalidDebError,
            deb_source.provision,
            dst=dest_dir,
            keep_deb=True,
        )
