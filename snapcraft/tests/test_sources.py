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

import copy
import os
import http.server
import shutil
import threading
import unittest.mock

import fixtures
import libarchive

from snapcraft.internal import (
    common,
    errors,
    sources
)
from snapcraft import tests


class FakeFileHTTPRequestHandler(http.server.BaseHTTPRequestHandler):

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

    scenarios = [
        ('TERM=dumb', dict(term='dumb')),
        ('TERM=vt100', dict(term='vt100')),
    ]

    @unittest.mock.patch('snapcraft.sources.Tar.provision')
    def test_pull_tarball_must_download_to_sourcedir(self, mock_prov):
        self.useFixture(fixtures.EnvironmentVariable('TERM', self.term))
        self.useFixture(fixtures.EnvironmentVariable(
            'no_proxy', 'localhost,127.0.0.1'))
        server = http.server.HTTPServer(
            ('127.0.0.1', 0), FakeFileHTTPRequestHandler)
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
        self.useFixture(fixtures.EnvironmentVariable(
            'no_proxy', 'localhost,127.0.0.1'))
        self.server = http.server.HTTPServer(
            ('127.0.0.1', 0), FakeFileHTTPRequestHandler)
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


class TestDeb(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.useFixture(fixtures.EnvironmentVariable(
            'no_proxy', 'localhost,127.0.0.1'))
        self.server = http.server.HTTPServer(
            ('127.0.0.1', 0), FakeFileHTTPRequestHandler)
        server_thread = threading.Thread(target=self.server.serve_forever)
        self.addCleanup(server_thread.join)
        self.addCleanup(self.server.server_close)
        self.addCleanup(self.server.shutdown)
        server_thread.start()

        patcher = unittest.mock.patch('apt_inst.DebFile')
        self.mock_deb = patcher.start()
        self.addCleanup(patcher.stop)

    def test_pull_debfile_must_download_and_extract(self):
        dest_dir = 'src'
        os.makedirs(dest_dir)
        deb_file_name = 'test.deb'
        source = 'http://{}:{}/{file_name}'.format(
            *self.server.server_address, file_name=deb_file_name)
        deb_source = sources.Deb(source, dest_dir)

        deb_source.pull()

        self.mock_deb.assert_called_once_with(
            os.path.join(deb_source.source_dir, deb_file_name))

    def test_extract_and_keep_debfile(self):
        deb_file_name = 'test.deb'
        source = 'http://{}:{}/{file_name}'.format(
            *self.server.server_address, file_name=deb_file_name)
        dest_dir = os.path.abspath(os.curdir)
        deb_source = sources.Deb(source, dest_dir)

        deb_source.download()
        deb_source.provision(dst=dest_dir, keep_deb=True)

        deb_download = os.path.join(deb_source.source_dir, deb_file_name)
        self.mock_deb.assert_called_once_with(
            os.path.join(deb_source.source_dir, deb_file_name))

        with open(deb_download, 'r') as deb_file:
            self.assertEqual('Test fake compressed file', deb_file.read())


class TestRpm(tests.TestCase):

    def test_pull_rpm_file_must_extract(self):
        rpm_file_name = 'test.rpm'
        dest_dir = 'src'
        os.makedirs(dest_dir)

        test_file_path = os.path.join(self.path, 'test.txt')
        open(test_file_path, 'w').close()
        rpm_file_path = os.path.join(self.path, rpm_file_name)
        os.chdir(self.path)
        with libarchive.file_writer(rpm_file_path, 'cpio', 'gzip') as rpm:
            rpm.add_files('test.txt')

        rpm_source = sources.Rpm(rpm_file_path, dest_dir)
        rpm_source.pull()

        self.assertEqual(os.listdir(dest_dir), ['test.txt'])

    def test_extract_and_keep_rpmfile(self):
        rpm_file_name = 'test.rpm'
        dest_dir = 'src'
        os.makedirs(dest_dir)

        test_file_path = os.path.join(self.path, 'test.txt')
        open(test_file_path, 'w').close()
        rpm_file_path = os.path.join(self.path, rpm_file_name)
        os.chdir(self.path)
        with libarchive.file_writer(rpm_file_path, 'cpio', 'gzip') as rpm:
            rpm.add_files('test.txt')

        rpm_source = sources.Rpm(rpm_file_path, dest_dir)
        # This is the first step done by pull. We don't call pull to call the
        # second step with a different keep_rpm value.
        shutil.copy2(rpm_source.source, rpm_source.source_dir)
        rpm_source.provision(dst=dest_dir, keep_rpm=True)

        test_output_files = ['test.txt', rpm_file_name]
        self.assertCountEqual(os.listdir(dest_dir), test_output_files)


class SourceTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        patcher = unittest.mock.patch('subprocess.check_call')
        self.mock_run = patcher.start()
        self.mock_run.return_value = True
        self.addCleanup(patcher.stop)

        patcher = unittest.mock.patch(
            'snapcraft.internal.sources._check_for_command')
        self.mock_check = patcher.start()
        self.mock_check.side_effect = None
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

    def test_pull_commit(self):
        bzr = sources.Bazaar(
            'lp:my-source', 'source_dir', source_commit='2')
        bzr.pull()

        self.mock_run.assert_called_once_with(
            ['bzr', 'branch', '-r', '2', 'lp:my-source',
             'source_dir'])

    def test_pull_existing_with_commit(self):
        self.mock_path_exists.return_value = True

        bzr = sources.Bazaar(
            'lp:my-source', 'source_dir', source_commit='2')
        bzr.pull()

        self.mock_run.assert_called_once_with(
            ['bzr', 'pull', '-r', '2', 'lp:my-source', '-d',
             'source_dir'])

    def test_init_with_source_branch_raises_exception(self):
        with self.assertRaises(
                sources.IncompatibleOptionsError) as raised:
            sources.Bazaar('lp:mysource', 'source_dir', source_branch='branch')

        expected_message = 'can\'t specify a source-branch for a bzr source'
        self.assertEqual(raised.exception.message, expected_message)

    def test_init_with_source_depth_raises_exception(self):
        with self.assertRaises(sources.IncompatibleOptionsError) as raised:
            sources.Bazaar('lp://mysource', 'source_dir', source_depth=2)

        expected_message = (
            'can\'t specify source-depth for a bzr source')
        self.assertEqual(raised.exception.message, expected_message)

    def test_init_with_source_tag_and_commit_raises_exception(self):
        with self.assertRaises(sources.IncompatibleOptionsError) as raised:
            sources.Bazaar('lp://mysource', 'source_dir', source_tag="tag",
                           source_commit="2")

        expected_message = (
            'can\'t specify both source-tag and source-commit for '
            'a bzr source')
        self.assertEqual(raised.exception.message, expected_message)


class TestGit(SourceTestCase):

    def test_pull(self):
        git = sources.Git('git://my-source', 'source_dir')

        git.pull()

        self.mock_run.assert_called_once_with(
            ['git', 'clone', '--recursive', 'git://my-source',
             'source_dir'])

    def test_pull_with_depth(self):
        git = sources.Git('git://my-source', 'source_dir', source_depth=2)

        git.pull()

        self.mock_run.assert_called_once_with(
            ['git', 'clone', '--recursive', '--depth', '2', 'git://my-source',
             'source_dir'])

    def test_pull_branch(self):
        git = sources.Git('git://my-source', 'source_dir',
                          source_branch='my-branch')
        git.pull()

        self.mock_run.assert_called_once_with(
            ['git', 'clone', '--recursive', '--branch',
             'my-branch', 'git://my-source', 'source_dir'])

    def test_pull_tag(self):
        git = sources.Git('git://my-source', 'source_dir', source_tag='tag')
        git.pull()

        self.mock_run.assert_called_once_with(
            ['git', 'clone', '--recursive', '--branch', 'tag',
             'git://my-source', 'source_dir'])

    def test_pull_commit(self):
        git = sources.Git(
            'git://my-source', 'source_dir',
            source_commit='2514f9533ec9b45d07883e10a561b248497a8e3c')
        git.pull()

        self.mock_run.assert_has_calls([
            unittest.mock.call(['git', 'clone', '--recursive',
                                'git://my-source', 'source_dir']),
            unittest.mock.call(['git', '-C', 'source_dir', 'checkout',
                                '2514f9533ec9b45d07883e10a561b248497a8e3c'])
        ])

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

    def test_pull_existing_with_commit(self):
        self.mock_path_exists.return_value = True

        git = sources.Git(
            'git://my-source', 'source_dir',
            source_commit='2514f9533ec9b45d07883e10a561b248497a8e3c')
        git.pull()

        self.mock_run.assert_has_calls([
            unittest.mock.call(['git', '-C', 'source_dir', 'pull',
                                '--recurse-submodules=yes', 'git://my-source',
                                '2514f9533ec9b45d07883e10a561b248497a8e3c']),
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

    def test_init_with_source_branch_and_commit_raises_exception(self):
        with self.assertRaises(sources.IncompatibleOptionsError) as raised:
            sources.Git(
                'git://mysource', 'source_dir',
                source_commit='2514f9533ec9b45d07883e10a561b248497a8e3c',
                source_branch='branch')

        expected_message = \
            'can\'t specify both source-branch and source-commit for ' \
            'a git source'
        self.assertEqual(raised.exception.message, expected_message)

    def test_init_with_source_tag_and_commit_raises_exception(self):
        with self.assertRaises(sources.IncompatibleOptionsError) as raised:
            sources.Git(
                'git://mysource', 'source_dir',
                source_commit='2514f9533ec9b45d07883e10a561b248497a8e3c',
                source_tag='tag')

        expected_message = \
            'can\'t specify both source-tag and source-commit for ' \
            'a git source'
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

    def test_pull_commit(self):
        hg = sources.Mercurial('hg://my-source', 'source_dir',
                               source_commit='2')
        hg.pull()

        self.mock_run.assert_called_once_with(
            ['hg', 'clone', '-u', '2', 'hg://my-source',
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

    def test_pull_existing_with_commit(self):
        self.mock_path_exists.return_value = True

        hg = sources.Mercurial('hg://my-source', 'source_dir',
                               source_commit='2')
        hg.pull()

        self.mock_run.assert_called_once_with(
            ['hg', 'pull', '-r', '2', 'hg://my-source'])

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

    def test_init_with_source_commit_and_tag_raises_exception(self):
        with self.assertRaises(sources.IncompatibleOptionsError) as raised:
            sources.Mercurial(
                'hg://mysource', 'source_dir', source_commit='2',
                source_tag='tag')

        expected_message = (
            'can\'t specify both source-tag and source-commit for a mercurial '
            'source')
        self.assertEqual(raised.exception.message, expected_message)

    def test_init_with_source_commit_and_branch_raises_exception(self):
        with self.assertRaises(sources.IncompatibleOptionsError) as raised:
            sources.Mercurial(
                'hg://mysource', 'source_dir', source_commit='2',
                source_branch='branch')

        expected_message = (
            'can\'t specify both source-branch and source-commit for '
            'a mercurial source')
        self.assertEqual(raised.exception.message, expected_message)

    def test_init_with_source_depth_raises_exception(self):
        with self.assertRaises(sources.IncompatibleOptionsError) as raised:
            sources.Mercurial('hg://mysource', 'source_dir', source_depth=2)

        expected_message = (
            'can\'t specify source-depth for a mercurial source')
        self.assertEqual(raised.exception.message, expected_message)


class TestSubversion(SourceTestCase):

    def test_pull_remote(self):
        svn = sources.Subversion('svn://my-source', 'source_dir')
        svn.pull()
        self.mock_run.assert_called_once_with(
            ['svn', 'checkout', 'svn://my-source', 'source_dir'])

    def test_pull_remote_commit(self):
        svn = sources.Subversion('svn://my-source', 'source_dir',
                                 source_commit="2")
        svn.pull()
        self.mock_run.assert_called_once_with(
            ['svn', 'checkout', 'svn://my-source', 'source_dir', '-r', '2'])

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

    def test_init_with_source_depth_raises_exception(self):
        with self.assertRaises(sources.IncompatibleOptionsError) as raised:
            sources.Subversion('svn://mysource', 'source_dir', source_depth=2)

        expected_message = (
            'can\'t specify source-depth for a Subversion source')
        self.assertEqual(raised.exception.message, expected_message)


class TestLocal(tests.TestCase):

    def test_pull_with_source_a_parent_of_current_dir(self):
        snapcraft_files_before_pull = copy.copy(common.SNAPCRAFT_FILES)

        # Verify that the snapcraft root dir does not get copied into itself.
        os.makedirs('subdir')

        cwd = os.getcwd()
        os.chdir('subdir')
        local = sources.Local('..', 'foo')
        local.pull()
        os.chdir(cwd)

        self.assertTrue(
            'subdir' not in os.listdir(os.path.join('subdir', 'foo')))

        # Regression test for https://bugs.launchpad.net/snapcraft/+bug/1614913
        # Verify that SNAPCRAFT_FILES was not modified by the pull when there
        # are files to ignore.
        self.assertEqual(
            snapcraft_files_before_pull, common.SNAPCRAFT_FILES)

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

    def setUp(self):
        super().setUp()

        patcher = unittest.mock.patch(
            'snapcraft.internal.sources._check_for_command')
        patcher.start()
        self.addCleanup(patcher.stop)

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


class CommandCheckTestCase(tests.TestCase):

    def test__check_for_command_not_installed(self):
        with self.assertRaises(errors.MissingCommandError):
            sources._check_for_command('missing-command')

    def test__check_for_command_installed(self):
        sources._check_for_command('sh')
