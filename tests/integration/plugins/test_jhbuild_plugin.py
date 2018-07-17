# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2018 Canonical Ltd
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

import http.server
import glob
import os
import urllib.parse

from testtools.matchers import FileExists

from tests import fixture_setup, integration


class TestFakeServer(http.server.HTTPServer):
    def __init__(self, server_address):
        super().__init__(server_address, http.server.SimpleHTTPRequestHandler)


class JHBuildPluginTestCase(integration.TestCase):
    def setUp(self):
        super().setUp()
        self.fake_server_fixture = fixture_setup.FakeServerRunning()
        self.fake_server_fixture.fake_server = TestFakeServer

    def start_fake_server(self):
        self.useFixture(self.fake_server_fixture)
        self.netloc = urllib.parse.urlparse(self.fake_server_fixture.url).netloc

    def test_snap(self):
        files = {
            "pull": [os.path.join(self.parts_dir, "jhbuild", "jhbuildrc")],
            "build": [
                os.path.join(self.parts_dir, "jhbuild", "jhbuildrc"),
                os.path.join(
                    self.parts_dir,
                    "jhbuild",
                    "install",
                    "usr",
                    "share",
                    "doc",
                    "dep-module",
                    "README",
                ),
                os.path.join(
                    self.parts_dir,
                    "jhbuild",
                    "install",
                    "usr",
                    "share",
                    "doc",
                    "main-module",
                    "README",
                ),
            ],
            "stage": [
                os.path.join(
                    self.stage_dir, "usr", "share", "doc", "dep-module", "README"
                ),
                os.path.join(
                    self.stage_dir, "usr", "share", "doc", "main-module", "README"
                ),
            ],
            "prime": [
                os.path.join(
                    self.prime_dir, "usr", "share", "doc", "dep-module", "README"
                ),
                os.path.join(
                    self.prime_dir, "usr", "share", "doc", "main-module", "README"
                ),
                os.path.join(self.prime_dir, "command-dep.wrapper"),
                os.path.join(self.prime_dir, "command-main.wrapper"),
            ],
            "snap": [],
        }

        # start the test http server
        self.start_fake_server()

        # copy the .modules file into cwd and replace the placeholder with
        # the local test http url.
        infile_path = os.path.join(
            self.snaps_dir, "jhbuild", "simple-jhbuild.modules.in"
        )
        outfile_path = os.path.join(self.path, "simple-jhbuild.modules")

        infile = open(infile_path, "r")
        outfile = open(outfile_path, "w")

        for line in infile:
            outfile.write(line.replace("__TESTFIXTURE__", "http://" + self.netloc))

        infile.close()
        outfile.close()

        # run the tests
        for step in ["pull", "build", "stage", "prime", "snap"]:
            self.run_snapcraft(step, "jhbuild")

            for path in files[step]:
                self.assertThat(os.path.join(self.path, path), FileExists())

        self.assertNotEqual([], glob.glob("test-jhbuild_*.snap"))
