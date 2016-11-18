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
import unittest.mock

import snapcraft
from snapcraft.internal import common, sources
from snapcraft import tests


class TestBasePlugin(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.project_options = snapcraft.ProjectOptions()

    def test_get_source_with_unrecognized_source_must_raise_exception(self):
        options = tests.MockOptions('unrecognized://test_source')
        plugin = snapcraft.BasePlugin('test_plugin', options,
                                      self.project_options)
        with self.assertRaises(ValueError) as raised:
            plugin.pull()

        self.assertEqual(raised.exception.__str__(),
                         'no handler to manage source')

    def test_parallel_build_count_returns_1_when_disabled(self):
        options = tests.MockOptions(disable_parallel=True)
        plugin = snapcraft.BasePlugin('test_plugin', options,
                                      self.project_options)
        self.assertEqual(plugin.parallel_build_count, 1)

    def test_parallel_build_count_returns_build_count_from_project(self):
        options = tests.MockOptions(disable_parallel=False)
        plugin = snapcraft.BasePlugin('test_plugin', options,
                                      self.project_options)
        unittest.mock.patch.object(
            self.project_options, 'parallel_build_count', 2)
        self.assertEqual(plugin.parallel_build_count, 2)

    @unittest.mock.patch('os.path.isdir')
    def test_local_non_dir_source_path_must_raise_exception(self, mock_isdir):
        options = tests.MockOptions('file')
        mock_isdir.return_value = False
        plugin = snapcraft.BasePlugin('test_plugin', options,
                                      self.project_options)
        with self.assertRaises(ValueError) as raised:
            plugin.pull()

        mock_isdir.assert_called_once_with('file')

        self.assertEqual(raised.exception.__str__(),
                         'local source is not a directory')

    def test_build_with_subdir_copies_sourcedir(self):
        options = tests.MockOptions(source_subdir='src')
        plugin = snapcraft.BasePlugin('test-part', options,
                                      self.project_options)

        subdir = os.path.join(plugin.sourcedir, plugin.options.source_subdir)
        os.makedirs(subdir)
        open(os.path.join(plugin.sourcedir, 'file1'), 'w').close()
        open(os.path.join(subdir, 'file2'), 'w').close()

        self.assertEqual(
            os.path.join(plugin.build_basedir, options.source_subdir),
            plugin.builddir)

        plugin.build()

        self.assertTrue(
            os.path.exists(os.path.join(plugin.build_basedir, 'file1')))
        self.assertTrue(os.path.exists(os.path.join(plugin.builddir, 'file2')))

    def test_build_without_subdir_copies_sourcedir(self):
        plugin = snapcraft.BasePlugin('test-part', options=None)

        os.makedirs(plugin.sourcedir)
        open(os.path.join(plugin.sourcedir, 'file'), 'w').close()

        self.assertEqual(plugin.build_basedir, plugin.builddir)

        plugin.build()

        self.assertTrue(
            os.path.exists(os.path.join(plugin.build_basedir, 'file')))

    def test_part_name_with_forward_slash_is_one_directory(self):
        plugin = snapcraft.BasePlugin('test/part', options=None)

        os.makedirs(plugin.sourcedir)

        self.assertIn('test\N{BIG SOLIDUS}part', os.listdir('parts'))

    @unittest.mock.patch('snapcraft.internal.common.run')
    def test_run_without_specifying_cwd(self, mock_run):
        plugin = snapcraft.BasePlugin('test/part', options=None)
        plugin.run(['ls'])

        mock_run.assert_called_once_with(['ls'], cwd=plugin.builddir)

    @unittest.mock.patch('snapcraft.internal.common.run')
    def test_run_specifying_a_cwd(self, mock_run):
        plugin = snapcraft.BasePlugin('test/part', options=None)
        plugin.run(['ls'], cwd=plugin.sourcedir)

        mock_run.assert_called_once_with(['ls'], cwd=plugin.sourcedir)

    @unittest.mock.patch('snapcraft.internal.common.run_output')
    def test_run_output_without_specifying_cwd(self, mock_run):
        plugin = snapcraft.BasePlugin('test/part', options=None)
        plugin.run_output(['ls'])

        mock_run.assert_called_once_with(['ls'], cwd=plugin.builddir)

    @unittest.mock.patch('snapcraft.internal.common.run_output')
    def test_run_output_specifying_a_cwd(self, mock_run):
        plugin = snapcraft.BasePlugin('test/part', options=None)
        plugin.run_output(['ls'], cwd=plugin.sourcedir)

        mock_run.assert_called_once_with(['ls'], cwd=plugin.sourcedir)


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

    @unittest.mock.patch('snapcraft.internal.sources._check_for_command')
    def test_get_source_with_branch_and_tag_must_raise_error(self, mock_check):
        mock_check.side_effect = None
        options = tests.MockOptions('lp:source', self.source_type,
                                    self.source_branch, self.source_tag)
        plugin = snapcraft.BasePlugin('test_plugin', options)
        with self.assertRaises(sources.IncompatibleOptionsError) as raised:
            plugin.pull()

        self.assertEqual(
            raised.exception.__str__(),
            'can\'t specify both source-tag and source-branch for a {} '
            'source'.format(self.source_type))


class GetSourceTestCase(tests.TestCase):

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

    @unittest.mock.patch('snapcraft.internal.sources._check_for_command')
    def test_get_source_with_branch_must_raise_error(self, mock_check):
        mock_check.side_effect = None
        options = tests.MockOptions('lp:this', self.source_type,
                                    self.source_branch, self.source_tag,
                                    None, None, self.source_commit)
        plugin = snapcraft.BasePlugin('test_plugin', options)

        with self.assertRaises(sources.IncompatibleOptionsError) as raised:
            plugin.pull()

        self.assertEqual(
            raised.exception.__str__(),
            'can\'t specify a {} for a {} source'.format(
                self.error, self.source_type))


class BuildTestCase(tests.TestCase):

    @unittest.mock.patch('snapcraft.internal.sources._check_for_command')
    def test_do_not_follow_links(self, mock_check):
        mock_check.side_effect = None
        options = tests.MockOptions(source='.')
        plugin = snapcraft.BasePlugin('test_plugin', options)

        os.makedirs(plugin.partdir)
        os.symlink(os.path.abspath('.'), plugin.sourcedir)

        # Create a file and a symlink to it
        open('file', mode='w').close()
        os.symlink('file', 'symlinkfile')

        # Create a directory and a symlink to it
        os.mkdir('dir')
        os.symlink('dir', 'symlinkdir')
        plugin.build()

        # Make sure this is still a link
        self.assertTrue(os.path.islink(plugin.sourcedir))

        build_file_path = os.path.join(
            plugin.builddir, 'file')
        build_symlinkfile_path = os.path.join(
            plugin.builddir, 'symlinkfile')

        self.assertTrue(os.path.isfile(build_file_path))
        self.assertTrue(os.path.islink(build_symlinkfile_path))

        build_dir_path = os.path.join(
            plugin.builddir, 'dir')
        build_symlinkdir_path = os.path.join(
            plugin.builddir, 'symlinkdir')

        self.assertTrue(os.path.isdir(build_dir_path))
        self.assertTrue(os.path.islink(build_symlinkdir_path))

    def test_build_ignores_snapcraft_files_in_source_dir(self):
        plugin = snapcraft.BasePlugin('test-part', options=None)

        os.makedirs(plugin.sourcedir)
        os.makedirs(plugin.builddir)

        for file_ in common.SNAPCRAFT_FILES:
            open(os.path.join(plugin.sourcedir, file_), 'w').close()
        open(os.path.join(plugin.sourcedir, 'my-snap.snap'), 'w').close()
        open(os.path.join(plugin.sourcedir, 'my-snap'), 'w').close()

        plugin.build()

        for file_ in common.SNAPCRAFT_FILES:
            self.assertFalse(
                os.path.exists(os.path.join(plugin.builddir, file_)))
        self.assertFalse(
            os.path.exists(os.path.join(plugin.builddir, 'my-snap.snap')),
            os.listdir(plugin.builddir))

        # Make sure we don't filter things out incorrectly
        self.assertTrue(
            os.path.exists(os.path.join(plugin.builddir, 'my-snap')),
            os.listdir(plugin.builddir))


class CleanBuildTestCase(tests.TestCase):

    def test_clean_build(self):
        options = tests.MockOptions(source='src')
        plugin = snapcraft.BasePlugin('test_plugin', options)

        os.makedirs(plugin.sourcedir)
        source_file = os.path.join(plugin.sourcedir, 'source')
        open(source_file, 'w').close()

        os.makedirs(plugin.build_basedir)
        open(os.path.join(plugin.build_basedir, 'built'), 'w').close()

        os.makedirs(plugin.installdir)
        open(os.path.join(plugin.installdir, 'installed'), 'w').close()

        plugin.clean_build()

        # Make sure the source file hasn't been touched
        self.assertTrue(os.path.isfile(source_file))

        # Make sure the build directory is gone
        self.assertFalse(os.path.exists(plugin.build_basedir))

        # Make sure the install directory is gone
        self.assertFalse(os.path.exists(plugin.installdir))


class CleanPullTestCase(tests.TestCase):

    def test_clean_pull_directory(self):
        options = tests.MockOptions(source='src')
        plugin = snapcraft.BasePlugin('test_plugin', options)

        os.makedirs(plugin.sourcedir)
        source_file = os.path.join(plugin.sourcedir, 'source')
        open(source_file, 'w').close()

        plugin.clean_pull()

        # The source directory should now be gone
        self.assertFalse(os.path.exists(plugin.sourcedir))

    def test_clean_pull_symlink(self):
        options = tests.MockOptions(source='src')
        plugin = snapcraft.BasePlugin('test_plugin', options)

        real_source_directory = os.path.join(os.getcwd(), 'src')
        os.mkdir(real_source_directory)
        os.makedirs(plugin.partdir)
        os.symlink(real_source_directory, plugin.sourcedir)

        plugin.clean_pull()

        # The source symlink should now be gone, but the real source should
        # still be there.
        self.assertFalse(os.path.exists(plugin.sourcedir))
        self.assertTrue(os.path.isdir(real_source_directory))
