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

import fixtures

import tarfile
import zipfile

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
        self.useFixture(fixtures.EnvironmentVariable(
            'no_proxy', 'localhost,127.0.0.1'))
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

    def test_checksum(self):
        tar = tarfile.open("checksum.tar", "w")
        tar.close()

        # md5
        source_checksum = '1276481102f218c981e0324180bafd9f'
        sources.FileBase.check_checksum_determine_format(
            self, source_checksum, "checksum.tar")
        sources.FileBase.check_checksum(
            self, source_checksum, "checksum.tar")

        # sha1
        source_checksum = '34e163be8e43c5631d8b92e9c43ab0bf0fa62b9c'
        sources.FileBase.check_checksum_determine_format(
            self, source_checksum, "checksum.tar")
        sources.FileBase.check_checksum(
            self, source_checksum, "checksum.tar")

        # sha224
        source_checksum = ('82be34614e8ca3d20d1733e52ae9b5b12902c196eeb4ee36c'
                           '2625b5f')
        sources.FileBase.check_checksum_determine_format(
            self, source_checksum, "checksum.tar")
        sources.FileBase.check_checksum(
            self, source_checksum, "checksum.tar")

        # sha256
        source_checksum = ('84ff92691f909a05b224e1c56abb4864f01b4f8e3c854e4bb'
                           '4c7baf1d3f6d652')
        sources.FileBase.check_checksum_determine_format(
            self, source_checksum, "checksum.tar")
        sources.FileBase.check_checksum(
            self, source_checksum, "checksum.tar")

        # sha384
        source_checksum = ('a31859d12a1176d02c2552f1bea2cba40a3ec809a7b224425'
                           '04e7dd1c19201e6eeffce9a54d13760924ad73aad45049f')
        sources.FileBase.check_checksum_determine_format(
            self, source_checksum, "checksum.tar")
        sources.FileBase.check_checksum(
            self, source_checksum, "checksum.tar")

        # sha512
        source_checksum = ('1e543b135acb1da2d9ce119c11d6fa9de2c9ca2e97e55fdf2'
                           '481c2944779a3d6df4a7c74f87692072ada4d494bbc3018d7'
                           '545b3c631dac1bfb787f81e0b76530')
        sources.FileBase.check_checksum_determine_format(
            self, source_checksum, "checksum.tar")
        sources.FileBase.check_checksum(
            self, source_checksum, "checksum.tar")

    def test_non_matching_checksum(self):
        tar = tarfile.open("checksum.tar", "w")
        tar.close()

        source_checksum = '1234481102f218c981e0324180ba1234'

        sources.FileBase.check_checksum_determine_format(
            self, source_checksum, "checksum.tar")

        with self.assertRaises(sources.ChecksumDoesNotMatch) as raised:
            sources.FileBase.check_checksum(
                self, source_checksum, "checksum.tar")
        expected_message = ("the checksum ( 1234481102f218c981e0324180ba1234 "
                            ") doesn't match the file ( 1276481102f218c981e03"
                            "24180bafd9f )")
        self.assertEqual(raised.exception.message, expected_message)

class TestZip(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.useFixture(fixtures.EnvironmentVariable(
            'no_proxy', 'localhost,127.0.0.1'))
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

    def test_checksum_of_zip(self):
        zipfile_check = zipfile.ZipFile('checksum.zip', 'w')
        zipfile_check.close()

        # md5
        source_checksum = '76cdb2bad9582d23c1f6f4d868218d6c'
        sources.FileBase.check_checksum_determine_format(
            self, source_checksum, "checksum.zip")
        sources.FileBase.check_checksum(
            self, source_checksum, "checksum.zip")

        # sha1
        source_checksum = 'b04f3ee8f5e43fa3b162981b50bb72fe1acabb33'
        sources.FileBase.check_checksum_determine_format(
            self, source_checksum, "checksum.zip")
        sources.FileBase.check_checksum(
            self, source_checksum, "checksum.zip")

        # sha224
        source_checksum = ('a3cb5d98d33ad55b145b1d058d8ea50b3c212ad949ed85b6f'
                           '7392196')
        sources.FileBase.check_checksum_determine_format(
            self, source_checksum, "checksum.zip")
        sources.FileBase.check_checksum(
            self, source_checksum, "checksum.zip")

        # sha256
        source_checksum = ('8739c76e681f900923b900c9df0ef75cf421d39cabb54650c'
                           '4b9ad19b6a76d85')
        sources.FileBase.check_checksum_determine_format(
            self, source_checksum, "checksum.zip")
        sources.FileBase.check_checksum(
            self, source_checksum, "checksum.zip")

        # sha384
        source_checksum = ('35b38c9c2bfa0a9716fc424785c169b2f4b8cf9cd039ef63b'
                           '502194ee482c332866f218fad8c9d00928394663ee75794')
        sources.FileBase.check_checksum_determine_format(
            self, source_checksum, "checksum.zip")
        sources.FileBase.check_checksum(
            self, source_checksum, "checksum.zip")

        # sha512
        source_checksum = ('5e2f959f36b66df0580a94f384c5fc1ceeec4b2a3925f062d'
                           '7b68f21758b86581ac2adcfdde73a171a28496e758ef1b23c'
                           'a4951c05455cdae9357cc3b5a5825f')
        sources.FileBase.check_checksum_determine_format(
            self, source_checksum, "checksum.zip")
        sources.FileBase.check_checksum(
            self, source_checksum, "checksum.zip")

    def test_non_matching_checksum(self):
        zipfile = tarfile.open("checksum.zip", "w")
        zipfile.close()

        source_checksum = '1234481102f218c981e0324180ba1234'

        sources.FileBase.check_checksum_determine_format(
            self, source_checksum, "checksum.zip")
        with self.assertRaises(sources.ChecksumDoesNotMatch) as raised:
            sources.FileBase.check_checksum(
                self, source_checksum, "checksum.zip")
        expected_message = ("the checksum ( 1234481102f218c981e0324180ba1234 "
                            ") doesn't match the file ( 1276481102f218c981e03"
                            "24180bafd9f )")
        self.assertEqual(raised.exception.message, expected_message)

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

    def test_init_with_source_checksum_raises_exception(self):
        with self.assertRaises(
                sources.IncompatibleOptionsError) as raised:
            sources.Bazaar('lp:mysource', 'source_dir',
                           source_checksum='checksum')

        expected_message = 'can\'t specify a source-checksum for a bzr source'
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

    def test_init_with_source_checksum_raises_exception(self):
        with self.assertRaises(sources.IncompatibleOptionsError) as raised:
            sources.Git('git://mysource', 'source_dir',
                        source_checksum='checksum')

        expected_message = \
            'can\'t specify source-checksum for a git source'
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

    def test_init_with_source_checksum_raises_exception(self):
        with self.assertRaises(sources.IncompatibleOptionsError) as raised:
            sources.Mercurial(
                'hg://mysource', 'source_dir', source_checksum='checksum')

        expected_message = (
            'can\'t specify source-checksum for a mercurial source')
        self.assertEqual(raised.exception.message, expected_message)


class TestSubversion(SourceTestCase):

    def test_pull_remote(self):
        svn = sources.Subversion('svn://my-source', 'source_dir')
        svn.pull()
        self.mock_run.assert_called_once_with(
            ['svn', 'checkout', 'svn://my-source', 'source_dir'])

    def test_pull_local_absolute_path(self):
        svn = sources.Subversion(self.path, 'source_dir')
        svn.pull()
        self.mock_run.assert_called_once_with(
            ['svn', 'checkout', 'file://'+self.path, 'source_dir'])

    def test_pull_local_relative_path(self):
        os.mkdir("my-source")
        svn = sources.Subversion('my-source', 'source_dir')
        svn.pull()
        self.mock_run.assert_called_once_with(
            ['svn', 'checkout',
             'file://{}'.format(os.path.join(self.path, 'my-source')),
             'source_dir'])

    def test_pull_existing(self):
        self.mock_path_exists.return_value = True
        svn = sources.Subversion('svn://my-source', 'source_dir')
        svn.pull()
        self.mock_run.assert_called_once_with(
            ['svn', 'update'], cwd=svn.source_dir)

    def test_init_with_source_tag_raises_exception(self):
        with self.assertRaises(sources.IncompatibleOptionsError) as raised:
            sources.Subversion(
                'svn://mysource', 'source_dir', source_tag='tag')
        expected_message = (
            "Can't specify source-tag for a Subversion source")
        self.assertEqual(raised.exception.message, expected_message)

    def test_init_with_source_branch_raises_exception(self):
        with self.assertRaises(sources.IncompatibleOptionsError) as raised:
            sources.Subversion(
                'svn://mysource', 'source_dir', source_branch='branch')
        expected_message = (
            "Can't specify source-branch for a Subversion source")
        self.assertEqual(raised.exception.message, expected_message)

    def test_init_with_source_branch_and_tag_raises_exception(self):
        with self.assertRaises(sources.IncompatibleOptionsError) as raised:
            sources.Subversion(
                'svn://mysource', 'source_dir', source_tag='tag',
                source_branch='branch')

        expected_message = (
            "Can't specify source-tag OR source-branch for a Subversion "
            "source")
        self.assertEqual(raised.exception.message, expected_message)

    def test_init_with_source_checksum_raises_exception(self):
        with self.assertRaises(sources.IncompatibleOptionsError) as raised:
            sources.Subversion(
                'svn://mysource', 'source_dir', source_checksum='checksum')

        expected_message = (
            "Can't specify source-checksum for a Subversion source")
        self.assertEqual(raised.exception.message, expected_message)


class TestLocal(tests.TestCase):

    def test_pull_with_existing_empty_source_dir_creates_hardlinks(self):
        os.makedirs(os.path.join('src', 'dir'))
        open(os.path.join('src', 'dir', 'file'), 'w').close()

        os.mkdir('destination')

        local = sources.Local('src', 'destination')
        local.pull()

        # Verify that the directories are not symlinks, but the file is a
        # hardlink.
        self.assertFalse(os.path.islink('destination'))
        self.assertFalse(os.path.islink(os.path.join('destination', 'dir')))
        self.assertGreater(
            os.stat(os.path.join('destination', 'dir', 'file')).st_nlink, 1)

    def test_pull_with_existing_source_link_creates_symlink(self):
        os.makedirs(os.path.join('src', 'dir'))
        open(os.path.join('src', 'dir', 'file'), 'w').close()

        # Note that this is a symlink now instead of a directory
        os.symlink('dummy', 'destination')

        local = sources.Local('src', 'destination')
        local.pull()

        self.assertFalse(os.path.islink('destination'))
        self.assertFalse(os.path.islink(os.path.join('destination', 'dir')))
        self.assertGreater(
            os.stat(os.path.join('destination', 'dir', 'file')).st_nlink, 1)

    def test_pull_with_existing_source_file_wipes_and_creates_hardlinks(self):
        os.makedirs(os.path.join('src', 'dir'))
        open(os.path.join('src', 'dir', 'file'), 'w').close()

        # Note that this is a file now instead of a directory
        open('destination', 'w').close()

        local = sources.Local('src', 'destination')
        local.pull()

        self.assertFalse(os.path.isfile('destination'))
        self.assertFalse(os.path.islink('destination'))
        self.assertFalse(os.path.islink(os.path.join('destination', 'dir')))
        self.assertGreater(
            os.stat(os.path.join('destination', 'dir', 'file')).st_nlink, 1)

    def test_pulling_twice_with_existing_source_dir_recreates_hardlinks(self):
        os.makedirs(os.path.join('src', 'dir'))
        open(os.path.join('src', 'dir', 'file'), 'w').close()

        os.mkdir('destination')

        local = sources.Local('src', 'destination')
        local.pull()
        local.pull()

        # Verify that the directories are not symlinks, but the file is a
        # hardlink.
        self.assertFalse(os.path.islink('destination'))
        self.assertFalse(os.path.islink(os.path.join('destination', 'dir')))
        self.assertGreater(
            os.stat(os.path.join('destination', 'dir', 'file')).st_nlink, 1)

    def test_pull_ignores_snapcraft_specific_data(self):
        # Make the snapcraft-specific directories
        os.makedirs(os.path.join('src', 'parts'))
        os.makedirs(os.path.join('src', 'stage'))
        os.makedirs(os.path.join('src', 'prime'))

        # Make the snapcraft.yaml (and hidden one) and a built snap
        open(os.path.join('src', 'snapcraft.yaml'), 'w').close()
        open(os.path.join('src', '.snapcraft.yaml'), 'w').close()
        open(os.path.join('src', 'foo.snap'), 'w').close()

        # Now make some real files
        os.makedirs(os.path.join('src', 'dir'))
        open(os.path.join('src', 'dir', 'file'), 'w').close()

        os.mkdir('destination')

        local = sources.Local('src', 'destination')
        local.pull()

        # Verify that the snapcraft-specific stuff got filtered out
        self.assertFalse(os.path.exists(os.path.join('destination', 'parts')))
        self.assertFalse(os.path.exists(os.path.join('destination', 'stage')))
        self.assertFalse(os.path.exists(os.path.join('destination', 'prime')))
        self.assertFalse(
            os.path.exists(os.path.join('destination', 'snapcraft.yaml')))
        self.assertFalse(
            os.path.exists(os.path.join('destination', '.snapcraft.yaml')))
        self.assertFalse(
            os.path.exists(os.path.join('destination', 'foo.snap')))

        # Verify that the real stuff made it in.
        self.assertFalse(os.path.islink('destination'))
        self.assertFalse(os.path.islink(os.path.join('destination', 'dir')))
        self.assertGreater(
            os.stat(os.path.join('destination', 'dir', 'file')).st_nlink, 1)


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

    @unittest.mock.patch('snapcraft.sources.Subversion.pull')
    def test_get_svn_source_from_uri(self, mock_pull):
        test_sources = [
            'svn://sylpheed.sraoss.jp/sylpheed/trunk'
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
