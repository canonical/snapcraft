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
import shutil
import tarfile
import unittest.mock

import fixtures
import libarchive

from snapcraft.internal import sources
from snapcraft import tests
from . import SourceTestCase


class TestTar(tests.FakeFileHTTPServerBasedTestCase):

    scenarios = [
        ('TERM=dumb', dict(term='dumb')),
        ('TERM=vt100', dict(term='vt100')),
    ]

    def setUp(self):
        self.useFixture(fixtures.EnvironmentVariable('TERM', self.term))
        super().setUp()

    @unittest.mock.patch('snapcraft.sources.Tar.provision')
    def test_pull_tarball_must_download_to_sourcedir(self, mock_prov):
        plugin_name = 'test_plugin'
        dest_dir = os.path.join('parts', plugin_name, 'src')
        os.makedirs(dest_dir)
        tar_file_name = 'test.tar'
        source = 'http://{}:{}/{file_name}'.format(
            *self.server.server_address, file_name=tar_file_name)
        tar_source = sources.Tar(source, dest_dir)

        tar_source.pull()

        mock_prov.assert_called_once_with(dest_dir)
        with open(os.path.join(dest_dir, tar_file_name), 'r') as tar_file:
            self.assertEqual('Test fake compressed file', tar_file.read())

    def test_strip_common_prefix(self):
        # Create tar file for testing
        os.makedirs(os.path.join('src', 'test_prefix'))
        file_to_tar = os.path.join('src', 'test_prefix', 'test.txt')
        open(file_to_tar, 'w').close()
        tar = tarfile.open(os.path.join('src', 'test.tar'), 'w')
        tar.add(file_to_tar)
        tar.close()

        tar_source = sources.Tar(os.path.join('src', 'test.tar'), 'dst')
        os.mkdir('dst')
        tar_source.pull()

        # The 'test_prefix' part of the path should have been removed
        self.assertTrue(os.path.exists(os.path.join('dst', 'test.txt')))

    def test_strip_common_prefix_symlink(self):
        # Create tar file for testing
        os.makedirs(os.path.join('src', 'test_prefix'))
        file_to_tar = os.path.join('src', 'test_prefix', 'test.txt')
        open(file_to_tar, 'w').close()

        file_to_link = os.path.join('src', 'test_prefix', 'link.txt')
        os.symlink("./test.txt", file_to_link)
        self.assertTrue(os.path.islink(file_to_link))

        def check_for_symlink(tarinfo):
            self.assertTrue(tarinfo.issym())
            self.assertEqual(file_to_link, tarinfo.name)
            self.assertEqual(file_to_tar, os.path.normpath(
                os.path.join(
                    os.path.dirname(file_to_tar), tarinfo.linkname)))
            return tarinfo

        tar = tarfile.open(os.path.join('src', 'test.tar'), 'w')
        tar.add(file_to_tar)
        tar.add(file_to_link, filter=check_for_symlink)
        tar.close()

        tar_source = sources.Tar(os.path.join('src', 'test.tar'), 'dst')
        os.mkdir('dst')
        tar_source.pull()

        # The 'test_prefix' part of the path should have been removed
        self.assertTrue(os.path.exists(os.path.join('dst', 'test.txt')))
        self.assertTrue(os.path.exists(os.path.join('dst', 'link.txt')))

    def test_strip_common_prefix_hardlink(self):
        # Create tar file for testing
        os.makedirs(os.path.join('src', 'test_prefix'))
        file_to_tar = os.path.join('src', 'test_prefix', 'test.txt')
        open(file_to_tar, 'w').close()

        file_to_link = os.path.join('src', 'test_prefix', 'link.txt')
        os.link(file_to_tar, file_to_link)
        self.assertTrue(os.path.exists(file_to_link))

        def check_for_hardlink(tarinfo):
            self.assertTrue(tarinfo.islnk())
            self.assertFalse(tarinfo.issym())
            self.assertEqual(file_to_link, tarinfo.name)
            self.assertEqual(file_to_tar, tarinfo.linkname)
            return tarinfo

        tar = tarfile.open(os.path.join('src', 'test.tar'), 'w')
        tar.add(file_to_tar)
        tar.add(file_to_link, filter=check_for_hardlink)
        tar.close()

        tar_source = sources.Tar(os.path.join('src', 'test.tar'), 'dst')
        os.mkdir('dst')
        tar_source.pull()

        # The 'test_prefix' part of the path should have been removed
        self.assertTrue(os.path.exists(os.path.join('dst', 'test.txt')))
        self.assertTrue(os.path.exists(os.path.join('dst', 'link.txt')))


class TestZip(tests.FakeFileHTTPServerBasedTestCase):

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
        raised = self.assertRaises(
            sources.errors.IncompatibleOptionsError,
            sources.Git,
            'git://mysource', 'source_dir',
            source_tag='tag', source_branch='branch')

        expected_message = \
            'can\'t specify both source-tag and source-branch for a git source'
        self.assertEqual(raised.message, expected_message)

    def test_init_with_source_branch_and_commit_raises_exception(self):
        raised = self.assertRaises(
            sources.errors.IncompatibleOptionsError,
            sources.Git,
            'git://mysource', 'source_dir',
            source_commit='2514f9533ec9b45d07883e10a561b248497a8e3c',
            source_branch='branch')

        expected_message = \
            'can\'t specify both source-branch and source-commit for ' \
            'a git source'
        self.assertEqual(raised.message, expected_message)

    def test_init_with_source_tag_and_commit_raises_exception(self):
        raised = self.assertRaises(
            sources.errors.IncompatibleOptionsError,
            sources.Git,
            'git://mysource', 'source_dir',
            source_commit='2514f9533ec9b45d07883e10a561b248497a8e3c',
            source_tag='tag')

        expected_message = \
            'can\'t specify both source-tag and source-commit for ' \
            'a git source'
        self.assertEqual(raised.message, expected_message)


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
        raised = self.assertRaises(
            sources.errors.IncompatibleOptionsError,
            sources.Mercurial,
            'hg://mysource', 'source_dir', source_tag='tag',
            source_branch='branch')

        expected_message = (
            'can\'t specify both source-tag and source-branch for a mercurial '
            'source')
        self.assertEqual(raised.message, expected_message)

    def test_init_with_source_commit_and_tag_raises_exception(self):
        raised = self.assertRaises(
            sources.errors.IncompatibleOptionsError,
            sources.Mercurial,
            'hg://mysource', 'source_dir', source_commit='2',
            source_tag='tag')

        expected_message = (
            'can\'t specify both source-tag and source-commit for a mercurial '
            'source')
        self.assertEqual(raised.message, expected_message)

    def test_init_with_source_commit_and_branch_raises_exception(self):
        raised = self.assertRaises(
            sources.errors.IncompatibleOptionsError,
            sources.Mercurial,
            'hg://mysource', 'source_dir', source_commit='2',
            source_branch='branch')

        expected_message = (
            'can\'t specify both source-branch and source-commit for '
            'a mercurial source')
        self.assertEqual(raised.message, expected_message)

    def test_init_with_source_depth_raises_exception(self):
        raised = self.assertRaises(
            sources.errors.IncompatibleOptionsError,
            sources.Mercurial,
            'hg://mysource', 'source_dir', source_depth=2)

        expected_message = (
            'can\'t specify source-depth for a mercurial source')
        self.assertEqual(raised.message, expected_message)


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
        raised = self.assertRaises(
            sources.errors.IncompatibleOptionsError,
            sources.Subversion,
            'svn://mysource', 'source_dir', source_tag='tag')
        expected_message = (
            "Can't specify source-tag for a Subversion source")
        self.assertEqual(raised.message, expected_message)

    def test_init_with_source_branch_raises_exception(self):
        raised = self.assertRaises(
            sources.errors.IncompatibleOptionsError,
            sources.Subversion,
            'svn://mysource', 'source_dir', source_branch='branch')
        expected_message = (
            "Can't specify source-branch for a Subversion source")
        self.assertEqual(raised.message, expected_message)

    def test_init_with_source_branch_and_tag_raises_exception(self):
        raised = self.assertRaises(
            sources.errors.IncompatibleOptionsError,
            sources.Subversion,
            'svn://mysource', 'source_dir', source_tag='tag',
            source_branch='branch')

        expected_message = (
            "Can't specify source-tag OR source-branch for a Subversion "
            "source")
        self.assertEqual(raised.message, expected_message)

    def test_init_with_source_depth_raises_exception(self):
        raised = self.assertRaises(
            sources.errors.IncompatibleOptionsError,
            sources.Subversion,
            'svn://mysource', 'source_dir', source_depth=2)

        expected_message = (
            'can\'t specify source-depth for a Subversion source')
        self.assertEqual(raised.message, expected_message)


class TestUri(tests.TestCase):

    scenarios = [
        ('tar.gz', dict(result='tar', source='https://golang.tar.gz')),
        ('tar.gz', dict(result='tar', source='https://golang.tar.xz')),
        ('tar.bz2', dict(result='tar', source='https://golang.tar.bz2')),
        ('tgz', dict(result='tar', source='https://golang.tgz')),
        ('tar', dict(result='tar', source='https://golang.tar')),
        ('git:', dict(result='git',
                      source='git://github.com:snapcore/snapcraft.git')),
        ('git@', dict(result='git',
                      source='git@github.com:snapcore/snapcraft.git')),
        ('.git', dict(result='git',
                      source='https://github.com:snapcore/snapcraft.git')),
        ('lp:', dict(result='bzr', source='lp:snapcraft_test_source')),
        ('bzr:', dict(result='bzr', source='bzr:dummy_source')),
        ('svn:', dict(result='subversion',
                      source='svn://sylpheed.jp/sylpheed/trunk')),

    ]

    def test_get_source_typefrom_uri(self):
        self.assertEqual(sources._get_source_type_from_uri(self.source),
                         self.result)


class SourceWithBranchTestCase(tests.TestCase):

    scenarios = [
        ('bzr with source branch', {
            'source_type': 'bzr',
            'source_branch': 'test_branch',
            'source_tag': None,
            'source_commit': None,
            'error': 'source-branch'}),
        ('tar with source branch', {
            'source_type': 'tar',
            'source_branch': 'test_branch',
            'source_tag': None,
            'source_commit': None,
            'error': 'source-branch'}),
        ('tar with source tag', {
            'source_type': 'tar',
            'source_branch': None,
            'source_tag': 'test_tag',
            'source_commit': None,
            'error': 'source-tag'}),
        ('tar with source commit', {
            'source_type': 'tar',
            'source_branch': None,
            'source_tag': None,
            'source_commit': 'commit',
            'error': 'source-commit'}),
        ('deb with source branch', {
            'source_type': 'deb',
            'source_branch': 'test_branch',
            'source_tag': None,
            'source_commit': None,
            'error': 'source-branch'}),
        ('deb with source tag', {
            'source_type': 'deb',
            'source_branch': None,
            'source_tag': 'test_tag',
            'source_commit': None,
            'error': 'source-tag'}),
        ('deb with source commit', {
            'source_type': 'deb',
            'source_branch': None,
            'source_tag': None,
            'source_commit': 'commit',
            'error': 'source-commit'})
    ]

    def test_get_source_with_branch_must_raise_error(self):
        handler = sources.get_source_handler('https://source.com',
                                             source_type=self.source_type)
        raised = self.assertRaises(
            sources.errors.IncompatibleOptionsError,
            handler,
            'https://source.com',
            source_dir='.',
            source_branch=self.source_branch,
            source_tag=self.source_tag,
            source_commit=self.source_commit)

        self.assertEqual(
            str(raised),
            'can\'t specify a {} for a {} source'.format(
                self.error, self.source_type))


class SourceWithBranchAndTagTestCase(tests.TestCase):

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
        handler = sources.get_source_handler('https://source.com',
                                             source_type=self.source_type)
        raised = self.assertRaises(
            sources.errors.IncompatibleOptionsError,
            handler,
            'https://source.com',
            source_dir='.',
            source_branch=self.source_branch,
            source_tag=self.source_tag)

        self.assertEqual(
            str(raised),
            'can\'t specify both source-tag and source-branch for a {} '
            'source'.format(self.source_type))


class GetSourceTestClass(tests.TestCase):

    def test_get(self):

        class Options:
            source = '.'

        sources.get('src', 'useless-arg', Options())

        self.assertTrue(os.path.isdir('src'))
