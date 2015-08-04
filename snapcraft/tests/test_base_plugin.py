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

import http.server
import fixtures
import logging
import os
import threading

import snapcraft
from snapcraft.tests import TestCase


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


class TestBasePlugin(TestCase):

    def test_isurl(self):
        plugin = snapcraft.BasePlugin('mock', {})
        self.assertTrue(plugin.isurl('git://'))
        self.assertTrue(plugin.isurl('bzr://'))
        self.assertFalse(plugin.isurl('./'))
        self.assertFalse(plugin.isurl('/foo'))
        self.assertFalse(plugin.isurl('/fo:o'))

    def test_pull_git_with_tag_and_branch_must_raise_error(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        plugin = snapcraft.BasePlugin('test_plugin', 'dummy_options')
        with self.assertRaises(SystemExit) as raised:
            plugin.pull_git(
                'dummy_source', source_tag='test_tag',
                source_branch='test_branch')

        self.assertEqual(raised.exception.code, 1, 'Wrong exit code returned.')
        expected = (
            "You can't specify both source-tag and source-branch for a git "
            "source (part 'test_plugin').\n")
        self.assertEqual(expected, fake_logger.output)

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
        plugin = snapcraft.BasePlugin(plugin_name, 'dummy_options')
        plugin.pull_tarball(source)

        with open(os.path.join(dest_dir, tar_file_name), 'r') as tar_file:
            self.assertEqual('Test fake tarball file', tar_file.read())
