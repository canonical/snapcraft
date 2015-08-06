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


class TestBasePlugin(tests.TestCase):

    def test_isurl(self):
        plugin = snapcraft.BasePlugin('mock', {})
        self.assertTrue(plugin.isurl('git://'))
        self.assertTrue(plugin.isurl('bzr://'))
        self.assertFalse(plugin.isurl('./'))
        self.assertFalse(plugin.isurl('/foo'))
        self.assertFalse(plugin.isurl('/fo:o'))

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

    def test_get_source_with_unrecognized_source_must_raise_error(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        plugin = snapcraft.BasePlugin('test_plugin', 'dummy_options')
        with self.assertRaises(SystemExit) as raised:
            plugin.get_source('unrecognized://test_source')

        self.assertEqual(raised.exception.code, 1, 'Wrong exit code returned.')
        expected = (
            "Unrecognized source 'unrecognized://test_source' for part "
            "'test_plugin'.\n")
        self.assertEqual(expected, fake_logger.output)

    def test_makedirs_with_existing_dir(self):
        plugin = snapcraft.BasePlugin('dummy_plugin', 'dummy_options')
        plugin.makedirs(self.path)
        self.assertTrue(os.path.exists(self.path))

    def test_makedirs_with_unexisting_dir(self):
        path = os.path.join(self.path, 'unexisting')
        plugin = snapcraft.BasePlugin('dummy_plugin', 'dummy_options')
        plugin.makedirs(path)
        self.assertTrue(os.path.exists(path))


class GetSourceWithBranches(tests.TestCase):

    scenarios = [
        ('git with source branch and tag', {
            'source_type': 'git',
            'source_branch': 'test_branch',
            'source_tag': 'tag',
        }),
        ('hg with source branch and tag', {
            'source_type': 'mercurial',
            'source_branch': 'test_branch',
            'source_tag': 'tag',
        }),
    ]

    def test_get_source_with_branch_and_tag_must_raise_error(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        plugin = snapcraft.BasePlugin('test_plugin', 'dummy_options')
        with self.assertRaises(SystemExit) as raised:
            plugin.get_source(
                'dummy_source', source_type=self.source_type,
                source_branch=self.source_branch, source_tag=self.source_tag)

        self.assertEqual(raised.exception.code, 1, 'Wrong exit code returned.')
        expected = (
            "You can't specify both source-tag and source-branch for a {} "
            "source (part 'test_plugin').\n".format(self.source_type))
        self.assertEqual(expected, fake_logger.output)


class GetSourceTestCase(tests.TestCase):

    scenarios = [
        ('bzr with source branch', {
            'source_type': 'bzr',
            'source_branch': 'test_branch',
            'source_tag': None,
            'error': 'source-branch'}),
        ('tar with source branch', {
            'source_type': 'tar',
            'source_branch': 'test_branch',
            'source_tag': None,
            'error': 'source-branch'}),
        ('tar with source tag', {
            'source_type': 'tar',
            'source_branch': None,
            'source_tag': 'test_tag',
            'error': 'source-tag'})
    ]

    def test_get_source_with_branch_must_raise_error(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        plugin = snapcraft.BasePlugin('test_plugin', 'dummy_options')
        with self.assertRaises(SystemExit) as raised:
            plugin.get_source(
                'dummy_source', source_type=self.source_type,
                source_branch=self.source_branch, source_tag=self.source_tag)

        self.assertEqual(raised.exception.code, 1, 'Wrong exit code returned.')
        expected = (
            "You can't specify {} for a {} source "
            "(part 'test_plugin').\n".format(self.error, self.source_type))
        self.assertEqual(expected, fake_logger.output)
