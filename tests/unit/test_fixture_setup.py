# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2018 Canonical Ltd
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

import http.client
import http.server
import json
import os
import tempfile
import urllib.parse

import fixtures
from testtools.matchers import Equals

from tests import fixture_setup, unit


class TempCWDTestCase(unit.TestCase):
    def test_with_TEMPDIR_env_var(self):
        with tempfile.TemporaryDirectory() as test_tmp_dir:
            with fixtures.EnvironmentVariable("TMPDIR", test_tmp_dir):
                temp_cwd_fixture = fixture_setup.TempCWD()
                self.useFixture(temp_cwd_fixture)

        self.assertThat(os.path.dirname(temp_cwd_fixture.path), Equals(test_tmp_dir))


class TestFakeServer(http.server.HTTPServer):
    def __init__(self, server_address):
        super().__init__(server_address, TestFakeRequestHandler)


class TestFakeRequestHandler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(json.dumps("{}").encode())

    def log_message(*args):
        # Do not print anything during the tests.
        pass


class FakeServerRunningTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()
        self.fake_server_fixture = fixture_setup.FakeServerRunning()
        self.fake_server_fixture.fake_server = TestFakeServer

    def start_fake_server(self):
        self.useFixture(self.fake_server_fixture)
        self.netloc = urllib.parse.urlparse(self.fake_server_fixture.url).netloc

    def do_request(self, method, path):
        connection = http.client.HTTPConnection(self.netloc)
        self.addCleanup(connection.close)
        connection.request(method, path)
        response = connection.getresponse()
        return response.status

    def assert_server_not_running(self):
        self.assertRaises(Exception, self.do_request, "GET", "/")

    def test_server_must_start_and_stop(self):
        self.addCleanup(self.assert_server_not_running)
        self.start_fake_server()
        status = self.do_request("GET", "/")
        self.assertThat(status, Equals(200))
