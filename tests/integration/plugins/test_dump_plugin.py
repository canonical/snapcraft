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
import fixtures
import tarfile
import threading

from pyftpdlib import authorizers, handlers, servers

from testtools.matchers import DirExists, FileContains, FileExists

from tests import integration


class FtpServerRunning(fixtures.Fixture):
    def __init__(self, directory):
        super().__init__()
        self.directory = directory

    def setUp(self):
        super().setUp()
        self._start_ftp_server()

    def _start_ftp_server(self):
        authorizer = authorizers.DummyAuthorizer()
        authorizer.add_anonymous(self.directory)
        handler = handlers.FTPHandler
        handler.authorizer = authorizer
        self.server = servers.FTPServer(("", 2121), handler)

        server_thread = threading.Thread(target=self.server.serve_forever)
        server_thread.start()
        self.addCleanup(self._stop_ftp_server, server_thread)

    def _stop_ftp_server(self, thread):
        self.server.close_all()
        thread.join()


class DumpPluginTestCase(integration.TestCase):
    def test_stage_dump_plugin(self):
        self.run_snapcraft("stage", "dump")

        expected_files = [
            "flat",
            os.path.join("flatdir", "flat2"),
            "onedeep",
            os.path.join("onedeepdir", "onedeep2"),
            "oneflat",
            "top-simple",
            "notop",
            "parent",
            "slash",
            "readonly_file",
        ]
        for expected_file in expected_files:
            self.assertThat(os.path.join(self.stage_dir, expected_file), FileExists())
        expected_dirs = ["dir-simple", "notopdir"]
        for expected_dir in expected_dirs:
            self.assertThat(os.path.join(self.stage_dir, expected_dir), DirExists())

        # Regression test for
        # https://bugs.launchpad.net/snapcraft/+bug/1500728
        self.run_snapcraft("pull")

    def test_download_file_with_content_encoding_set(self):
        """Download a file with Content-Encoding: gzip LP: #1611776"""
        self.run_snapcraft("pull", "compressed-content-encoding")

    def test_download_file_from_ftp_source(self):
        """Download a file from a FTP source, LP: #1602323"""
        ftp_dir = os.path.join(self.path, "ftp")
        os.mkdir(ftp_dir)
        ftp_server = FtpServerRunning(ftp_dir)
        self.useFixture(ftp_server)

        test_file_path = os.path.join(self.path, "test")
        with open(test_file_path, "w") as test_file:
            test_file.write("Hello ftp")
        with tarfile.open(os.path.join(ftp_dir, "test.tar.gz"), "w:gz") as tar:
            tar.add(test_file_path)

        self.run_snapcraft("pull", "ftp-source")
        self.assertThat(
            os.path.join(self.parts_dir, "ftp-part", "src", "test"),
            FileContains("Hello ftp"),
        )
