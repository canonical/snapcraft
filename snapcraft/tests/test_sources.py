# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015 Canonical Ltd
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
import http.server
import threading

from snapcraft import sources
from snapcraft import tests


class FakeTarballHTTPRequestHandler(http.server.BaseHTTPRequestHandler):

    def do_GET(self):
        data = 'Test fake tarball file'
        self.send_response(200)
        self.send_header('Content-Length', len(data))
        self.send_header('Content-type', 'text/html')
        self.end_headers()
        self.wfile.write(data.encode())

    def log_message(self, *args):
        # Overwritten so the test does not write to stderr.
        pass


class TestTar(tests.TestCase):

    def test_pull_tarball_must_download_to_sourcedir(self):
        server = http.server.HTTPServer(('', 0), FakeTarballHTTPRequestHandler)
        server_thread = threading.Thread(target=server.serve_forever)
        self.addCleanup(server_thread.join)
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        server_thread.start()

        plugin_name = 'test_plugin'
        dest_dir = os.path.join('parts', plugin_name, 'src')
        os.makedirs(dest_dir)
        tar_file_name = 'test.tar'
        source = 'http://{}:{}/{file_name}'.format(
            *server.server_address, file_name=tar_file_name)
        tar_source = sources.Tar(source)

        tar_source.pull(dest_dir)

        with open(os.path.join(dest_dir, tar_file_name), 'r') as tar_file:
            self.assertEqual('Test fake tarball file', tar_file.read())
