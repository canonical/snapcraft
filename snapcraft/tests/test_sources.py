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
import http.server
import threading
import unittest.mock

from snapcraft.internal import sources

from snapcraft import tests


class FakeTarballHTTPRequestHandler(http.server.BaseHTTPRequestHandler):

    def do_GET(self):
        data = 'Test fake compressed file'
        self.send_response(200)
        self.send_header('Content-Length', len(data))
        self.send_header('Content-type', 'text/html')
        self.end_headers()
        self.wfile.write(data.encode())

    def log_message(self, *args):
        # Overwritten so the test does not write to stderr.
        pass


class TestTar(tests.TestCase):

    @unittest.mock.patch('snapcraft.sources.Tar.provision')
    def test_pull_tarball_must_download_to_sourcedir(self, mock_prov):
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
        tar_source = sources.Tar(source, dest_dir)

        tar_source.pull()

        mock_prov.assert_called_once_with(dest_dir)
        with open(os.path.join(dest_dir, tar_file_name), 'r') as tar_file:
            self.assertEqual('Test fake compressed file', tar_file.read())


class TestZip(tests.TestCase):

    def setUp(self):
        super().setUp()
        os.environ['no_proxy'] = '127.0.0.1'
        self.server = http.server.HTTPServer(
            ('127.0.0.1', 0), FakeTarballHTTPRequestHandler)
        server_thread = threading.Thread(target=self.server.serve_forever)
        self.addCleanup(server_thread.join)
        self.addCleanup(self.server.server_close)
        self.addCleanup(self.server.shutdown)
        server_thread.start()

    @unittest.mock.patch('zipfile.ZipFile')
    def test_pull_zipfile_must_download_and_extract(self, mock_zip):
        dest_dir = 'src'
        os.makedirs(dest_dir)
        zip_file_name = 'test.zip'
        source = 'http://{}:{}/{file_name}'.format(
            *self.server.server_address, file_name=zip_file_name)
        zip_source = sources.Zip(source, dest_dir)

        zip_source.pull()

        mock_zip.assert_called_once_with(
            os.path.join(zip_source.source_dir, zip_file_name))

    @unittest.mock.patch('zipfile.ZipFile')
    def test_extract_and_keep_zipfile(self, mock_zip):
        zip_file_name = 'test.zip'
        source = 'http://{}:{}/{file_name}'.format(
            *self.server.server_address, file_name=zip_file_name)
        dest_dir = os.path.abspath(os.curdir)
        zip_source = sources.Zip(source, dest_dir)

        zip_source.download()
        zip_source.provision(dst=dest_dir, keep_zip=True)

        zip_download = os.path.join(zip_source.source_dir, zip_file_name)
        mock_zip.assert_called_once_with(zip_download)

        with open(zip_download, 'r') as zip_file:
            self.assertEqual('Test fake compressed file', zip_file.read())


class SourceTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        patcher = unittest.mock.patch('subprocess.check_call')
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
        bzr = sources.Bazaar('lp:my-source', 'source_dir')

        bzr.pull()

        self.mock_rmdir.assert_called_once_with('source_dir')
        self.mock_run.assert_called_once_with(
            ['bzr', 'branch', 'lp:my-source', 'source_dir'])

    def test_pull_tag(self):
        bzr = sources.Bazaar(
            'lp:my-source', 'source_dir', source_tag='tag')
        bzr.pull()

        self.mock_run.assert_called_once_with(
            ['bzr', 'branch', '-r', 'tag:tag', 'lp:my-source',
             'source_dir'])

    def test_pull_existing_with_tag(self):
        self.mock_path_exists.return_value = True

        bzr = sources.Bazaar(
            'lp:my-source', 'source_dir', source_tag='tag')
        bzr.pull()

        self.mock_run.assert_called_once_with(
            ['bzr', 'pull', '-r', 'tag:tag', 'lp:my-source', '-d',
             'source_dir'])

    def test_init_with_source_branch_raises_exception(self):
        with self.assertRaises(
                sources.IncompatibleOptionsError) as raised:
            sources.Bazaar('lp:mysource', 'source_dir', source_branch='branch')

        expected_message = 'can\'t specify a source-branch for a bzr source'
        self.assertEqual(raised.exception.message, expected_message)


class TestGit(SourceTestCase):

    def test_pull(self):
        git = sources.Git('git://my-source', 'source_dir')

        git.pull()

        self.mock_run.assert_called_once_with(
            ['git', 'clone', '--depth', '1', '--recursive', 'git://my-source',
             'source_dir'])

    def test_pull_branch(self):
        git = sources.Git('git://my-source', 'source_dir',
                          source_branch='my-branch')
        git.pull()

        self.mock_run.assert_called_once_with(
            ['git', 'clone', '--depth', '1', '--recursive', '--branch',
             'my-branch', 'git://my-source', 'source_dir'])

    def test_pull_tag(self):
        git = sources.Git('git://my-source', 'source_dir', source_tag='tag')
        git.pull()

        self.mock_run.assert_called_once_with(
            ['git', 'clone', '--depth', '1', '--recursive', '--branch', 'tag',
             'git://my-source', 'source_dir'])

    def test_pull_existing(self):
        self.mock_path_exists.return_value = True

        git = sources.Git('git://my-source', 'source_dir')
        git.pull()

        self.mock_run.assert_has_calls([
            unittest.mock.call(['git', '-C', 'source_dir', 'pull',
                                '--recurse-submodules=yes', 'git://my-source',
                                'HEAD']),
            unittest.mock.call(['git', '-C', 'source_dir', 'submodule',
                                'update'])
        ])

    def test_pull_existing_with_tag(self):
        self.mock_path_exists.return_value = True

        git = sources.Git('git://my-source', 'source_dir', source_tag='tag')
        git.pull()

        self.mock_run.assert_has_calls([
            unittest.mock.call(['git', '-C', 'source_dir', 'pull',
                                '--recurse-submodules=yes', 'git://my-source',
                                'refs/tags/tag']),
            unittest.mock.call(['git', '-C', 'source_dir', 'submodule',
                                'update'])
        ])

    def test_pull_existing_with_branch(self):
        self.mock_path_exists.return_value = True

        git = sources.Git('git://my-source', 'source_dir',
                          source_branch='my-branch')
        git.pull()

        self.mock_run.assert_has_calls([
            unittest.mock.call(['git', '-C', 'source_dir', 'pull',
                                '--recurse-submodules=yes', 'git://my-source',
                                'refs/heads/my-branch']),
            unittest.mock.call(['git', '-C', 'source_dir', 'submodule',
                                'update'])
        ])

    def test_init_with_source_branch_and_tag_raises_exception(self):
        with self.assertRaises(sources.IncompatibleOptionsError) as raised:
            sources.Git('git://mysource', 'source_dir',
                        source_tag='tag', source_branch='branch')

        expected_message = \
            'can\'t specify both source-tag and source-branch for a git source'
        self.assertEqual(raised.exception.message, expected_message)


class TestMercurial(SourceTestCase):

    def test_pull(self):
        hg = sources.Mercurial('hg://my-source', 'source_dir')
        hg.pull()

        self.mock_run.assert_called_once_with(
            ['hg', 'clone', 'hg://my-source', 'source_dir'])

    def test_pull_branch(self):
        hg = sources.Mercurial('hg://my-source', 'source_dir',
                               source_branch='my-branch')
        hg.pull()

        self.mock_run.assert_called_once_with(
            ['hg', 'clone', '-u', 'my-branch', 'hg://my-source',
             'source_dir'])

    def test_pull_tag(self):
        hg = sources.Mercurial('hg://my-source', 'source_dir',
                               source_tag='tag')
        hg.pull()

        self.mock_run.assert_called_once_with(
            ['hg', 'clone', '-u', 'tag', 'hg://my-source',
             'source_dir'])

    def test_pull_existing(self):
        self.mock_path_exists.return_value = True

        hg = sources.Mercurial('hg://my-source', 'source_dir')
        hg.pull()

        self.mock_run.assert_called_once_with(
            ['hg', 'pull', 'hg://my-source'])

    def test_pull_existing_with_tag(self):
        self.mock_path_exists.return_value = True

        hg = sources.Mercurial('hg://my-source', 'source_dir',
                               source_tag='tag')
        hg.pull()

        self.mock_run.assert_called_once_with(
            ['hg', 'pull', '-r', 'tag', 'hg://my-source'])

    def test_pull_existing_with_branch(self):
        self.mock_path_exists.return_value = True

        hg = sources.Mercurial('hg://my-source', 'source_dir',
                               source_branch='my-branch')
        hg.pull()

        self.mock_run.assert_called_once_with(
            ['hg', 'pull', '-b', 'my-branch', 'hg://my-source'])

    def test_init_with_source_branch_and_tag_raises_exception(self):
        with self.assertRaises(sources.IncompatibleOptionsError) as raised:
            sources.Mercurial(
                'hg://mysource', 'source_dir', source_tag='tag',
                source_branch='branch')

        expected_message = (
            'can\'t specify both source-tag and source-branch for a mercurial '
            'source')
        self.assertEqual(raised.exception.message, expected_message)


class TestLocal(tests.TestCase):

    def test_pull_with_existing_source_dir_creates_symlink(self):
        os.mkdir('source_dir')
        local = sources.Local('.', 'source_dir')
        local.pull()

        self.assertTrue(os.path.islink('source_dir'))
        self.assertEqual(os.path.realpath('source_dir'), self.path)

    def test_pull_with_existing_source_link_creates_symlink(self):
        os.symlink('dummy', 'source_dir')
        local = sources.Local('.', 'source_dir')
        local.pull()

        self.assertTrue(os.path.islink('source_dir'))
        self.assertEqual(os.path.realpath('source_dir'), self.path)

    def test_pull_with_existing_source_file_fails(self):
        open('source_dir', 'w').close()
        local = sources.Local('.', 'source_dir')

        with self.assertRaises(EnvironmentError) as raised:
            local.pull()

        self.assertEqual("Cannot pull to target 'source_dir'",
                         str(raised.exception))

    def test_pull_with_source_dir_with_content_fails(self):
        os.mkdir('source_dir')
        open(os.path.join('source_dir', 'file'), 'w').close()
        local = sources.Local('.', 'source_dir')

        with self.assertRaises(EnvironmentError) as raised:
            local.pull()

        self.assertEqual("Cannot pull to target 'source_dir'",
                         str(raised.exception))

    def test_pulling_twice_with_existing_source_dir_creates_symlink(self):
        os.mkdir('source_dir')
        local = sources.Local('.', 'source_dir')
        local.pull()
        local.pull()

        self.assertTrue(os.path.islink('source_dir'))
        self.assertEqual(os.path.realpath('source_dir'), self.path)


class TestUri(tests.TestCase):

    def test_get_tar_source_from_uri(self):
        test_sources = [
            'https://golang.tar.gz',
            'https://golang.tar.xz',
            'https://golang.tar.bz2',
            'https://golang.tar.tgz',
            'https://golang.tar',
        ]

        for source in test_sources:
            with self.subTest(key=source):
                self.assertEqual(
                    sources._get_source_type_from_uri(source), 'tar')

    @unittest.mock.patch('snapcraft.sources.Git.pull')
    def test_get_git_source_from_uri(self, mock_pull):
        test_sources = [
            'git://github.com:ubuntu-core/snapcraft.git',
            'git@github.com:ubuntu-core/snapcraft.git',
            'https://github.com:ubuntu-core/snapcraft.git',
        ]

        for source in test_sources:
            with self.subTest(key=source):
                options = tests.MockOptions(source=source)
                sources.get(
                    sourcedir='dummy',
                    builddir='dummy',
                    options=options)

                mock_pull.assert_called_once_with()
                mock_pull.reset_mock()

    @unittest.mock.patch('snapcraft.sources.Bazaar.pull')
    def test_get_bzr_source_from_uri(self, mock_pull):
        test_sources = [
            'lp:snapcraft_test_source',
            'bzr:dummy-source'
        ]

        for source in test_sources:
            with self.subTest(key=source):
                options = tests.MockOptions(source=source)
                sources.get(
                    sourcedir='dummy',
                    builddir='dummy',
                    options=options)

                mock_pull.assert_called_once_with()
                mock_pull.reset_mock()
