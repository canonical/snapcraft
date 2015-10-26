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
import unittest.mock

import snapcraft.sources

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
        os.environ['no_proxy'] = '127.0.0.1'
        server = http.server.HTTPServer(
            ('127.0.0.1', 0), FakeTarballHTTPRequestHandler)
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
        tar_source = snapcraft.sources.Tar(source, dest_dir)

        tar_source.pull()

        with open(os.path.join(dest_dir, tar_file_name), 'r') as tar_file:
            self.assertEqual('Test fake tarball file', tar_file.read())


class SourceTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        patcher = unittest.mock.patch('snapcraft.common.run')
        self.mock_run = patcher.start()
        self.mock_run.return_value = True
        self.addCleanup(patcher.stop)

        patcher = unittest.mock.patch('os.rmdir')
        self.mock_rmdir = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = unittest.mock.patch('os.path.exists')
        self.mock_path_exists = patcher.start()
        self.mock_path_exists.return_value = False
        self.addCleanup(patcher.stop)


class TestBazaar(SourceTestCase):

    def test_pull(self):
        bzr = snapcraft.sources.Bazaar('lp:my-source', 'source_dir')

        bzr.pull()

        self.mock_rmdir.assert_called_once_with('source_dir')
        self.mock_run.assert_called_once_with(
            ['bzr', 'branch', 'lp:my-source', 'source_dir'], cwd=os.getcwd())

    def test_pull_tag(self):
        bzr = snapcraft.sources.Bazaar(
            'lp:my-source', 'source_dir', source_tag='tag')
        bzr.pull()

        self.mock_run.assert_called_once_with(
            ['bzr', 'branch', '-r', 'tag:tag', 'lp:my-source',
             'source_dir'], cwd=os.getcwd())

    def test_pull_existing_with_tag(self):
        self.mock_path_exists.return_value = True

        bzr = snapcraft.sources.Bazaar(
            'lp:my-source', 'source_dir', source_tag='tag')
        bzr.pull()

        self.mock_run.assert_called_once_with(
            ['bzr', 'pull', '-r', 'tag:tag', 'lp:my-source', '-d',
             'source_dir'], cwd=os.getcwd())

    def test_provision(self):
        bzr = snapcraft.sources.Bazaar('lp:my-source', 'source_dir')
        bzr.provision('dst')

        self.mock_run.assert_called_once_with(
            ['cp', '-Trfa', 'source_dir', 'dst'], cwd=os.getcwd())

    def test_init_with_source_branch_raises_exception(self):
        with self.assertRaises(
                snapcraft.sources.IncompatibleOptionsError) as raised:
            snapcraft.sources.Bazaar('lp:mysource', 'source_dir',
                                     source_branch='branch')

        expected_message = 'can\'t specify a source-branch for a bzr source'
        self.assertEqual(raised.exception.message, expected_message)


class TestGit(SourceTestCase):

    def test_pull(self):
        git = snapcraft.sources.Git('git://my-source', 'source_dir')

        git.pull()

        self.mock_run.assert_called_once_with(
            ['git', 'clone', 'git://my-source', 'source_dir'], cwd=os.getcwd())

    def test_pull_branch(self):
        git = snapcraft.sources.Git('git://my-source', 'source_dir',
                                    source_branch='my-branch')
        git.pull()

        self.mock_run.assert_called_once_with(
            ['git', 'clone', '--branch', 'my-branch', 'git://my-source',
             'source_dir'], cwd=os.getcwd())

    def test_pull_tag(self):
        git = snapcraft.sources.Git('git://my-source', 'source_dir',
                                    source_tag='tag')
        git.pull()

        self.mock_run.assert_called_once_with(
            ['git', 'clone', '--branch', 'tag', 'git://my-source',
             'source_dir'], cwd=os.getcwd())

    def test_pull_existing(self):
        self.mock_path_exists.return_value = True

        git = snapcraft.sources.Git('git://my-source', 'source_dir')
        git.pull()

        self.mock_run.assert_called_once_with(
            ['git', '-C', 'source_dir', 'pull', 'git://my-source',
             'HEAD'], cwd=os.getcwd())

    def test_pull_existing_with_tag(self):
        self.mock_path_exists.return_value = True

        git = snapcraft.sources.Git('git://my-source', 'source_dir',
                                    source_tag='tag')
        git.pull()

        self.mock_run.assert_called_once_with(
            ['git', '-C', 'source_dir', 'pull', 'git://my-source',
             'refs/tags/tag'], cwd=os.getcwd())

    def test_pull_existing_with_branch(self):
        self.mock_path_exists.return_value = True

        git = snapcraft.sources.Git('git://my-source', 'source_dir',
                                    source_branch='my-branch')
        git.pull()

        self.mock_run.assert_called_once_with(
            ['git', '-C', 'source_dir', 'pull', 'git://my-source',
             'refs/heads/my-branch'], cwd=os.getcwd())

    def test_provision(self):
        bzr = snapcraft.sources.Git('git://my-source', 'source_dir')
        bzr.provision('dst')

        self.mock_run.assert_called_once_with(
            ['cp', '-Trfa', 'source_dir', 'dst'], cwd=os.getcwd())

    def test_init_with_source_branch_and_tag_raises_exception(self):
        with self.assertRaises(
                snapcraft.sources.IncompatibleOptionsError) as raised:
            snapcraft.sources.Git('git://mysource', 'source_dir',
                                  source_tag='tag', source_branch='branch')

        expected_message = \
            'can\'t specify both source-tag and source-branch for a git source'
        self.assertEqual(raised.exception.message, expected_message)


class TestMercurial(SourceTestCase):

    def test_pull(self):
        hg = snapcraft.sources.Mercurial('hg://my-source', 'source_dir')
        hg.pull()

        self.mock_run.assert_called_once_with(
            ['hg', 'clone', 'hg://my-source', 'source_dir'], cwd=os.getcwd())

    def test_pull_branch(self):
        hg = snapcraft.sources.Mercurial('hg://my-source', 'source_dir',
                                         source_branch='my-branch')
        hg.pull()

        self.mock_run.assert_called_once_with(
            ['hg', 'clone', '-u', 'my-branch', 'hg://my-source',
             'source_dir'], cwd=os.getcwd())

    def test_pull_tag(self):
        hg = snapcraft.sources.Mercurial('hg://my-source', 'source_dir',
                                         source_tag='tag')
        hg.pull()

        self.mock_run.assert_called_once_with(
            ['hg', 'clone', '-u', 'tag', 'hg://my-source',
             'source_dir'], cwd=os.getcwd())

    def test_pull_existing(self):
        self.mock_path_exists.return_value = True

        hg = snapcraft.sources.Mercurial('hg://my-source', 'source_dir')
        hg.pull()

        self.mock_run.assert_called_once_with(
            ['hg', 'pull', 'hg://my-source'], cwd=os.getcwd())

    def test_pull_existing_with_tag(self):
        self.mock_path_exists.return_value = True

        hg = snapcraft.sources.Mercurial('hg://my-source', 'source_dir',
                                         source_tag='tag')
        hg.pull()

        self.mock_run.assert_called_once_with(
            ['hg', 'pull', '-r', 'tag', 'hg://my-source'], cwd=os.getcwd())

    def test_pull_existing_with_branch(self):
        self.mock_path_exists.return_value = True

        hg = snapcraft.sources.Mercurial('hg://my-source', 'source_dir',
                                         source_branch='my-branch')
        hg.pull()

        self.mock_run.assert_called_once_with(
            ['hg', 'pull', '-b', 'my-branch', 'hg://my-source'],
            cwd=os.getcwd())

    def test_provision(self):
        bzr = snapcraft.sources.Mercurial('hg://my-source', 'source_dir')
        bzr.provision('dst')

        self.mock_run.assert_called_once_with(
            ['cp', '-Trfa', 'source_dir', 'dst'], cwd=os.getcwd())

    def test_init_with_source_branch_and_tag_raises_exception(self):
        with self.assertRaises(
                snapcraft.sources.IncompatibleOptionsError) as raised:
            snapcraft.sources.Mercurial(
                'hg://mysource', 'source_dir', source_tag='tag',
                source_branch='branch')

        expected_message = (
            'can\'t specify both source-tag and source-branch for a mercurial '
            'source')
        self.assertEqual(raised.exception.message, expected_message)


class TestLocal(SourceTestCase):

    def setUp(self):
        super().setUp()

        patcher = unittest.mock.patch('os.path.isdir')
        self.mock_isdir = patcher.start()
        self.mock_isdir.return_value = True
        self.addCleanup(patcher.stop)

        patcher = unittest.mock.patch('os.symlink')
        self.mock_symlink = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = unittest.mock.patch('os.remove')
        self.mock_remove = patcher.start()
        self.addCleanup(patcher.stop)
        self.mock_symlink.return_value = True

        patcher = unittest.mock.patch('os.path.abspath')
        self.mock_abspath = patcher.start()
        self.mock_abspath.return_value = '/home/ubuntu/sources/snap/source'
        self.addCleanup(patcher.stop)

    def test_pull(self):
        local = snapcraft.sources.Local('.', 'source_dir')
        local.pull()

    def test_provision(self):
        local = snapcraft.sources.Local('.', 'source_dir')
        local.provision('dst')

        self.mock_rmdir.assert_called_once_with('dst')
        self.mock_symlink.assert_called_once_with(
            '/home/ubuntu/sources/snap/source', 'dst')

    def test_provision_when_target_is_file(self):
        self.mock_isdir.return_value = False

        local = snapcraft.sources.Local('.', 'source_dir')
        local.provision('dst')

        self.mock_remove.assert_called_once_with('dst')
        self.mock_symlink.assert_called_once_with(
            '/home/ubuntu/sources/snap/source', 'dst')


class TestUri(tests.TestCase):

    def test_get_tar_source_from_uri(self):
        sources = [
            'https://golang.tar.gz',
            'https://golang.tar.xz',
            'https://golang.tar.bz2',
            'https://golang.tar.tgz',
        ]

        for source in sources:
            with self.subTest(key=source):
                self.assertEqual(
                    snapcraft.sources._get_source_type_from_uri(source), 'tar')
