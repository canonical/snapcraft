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
import tempfile
import unittest.mock

import snapcraft
from snapcraft import sources
from snapcraft import tests


class MockOptions:

    def __init__(self, source, source_type=None, source_branch=None,
                 source_tag=None):
        self.source = source
        self.source_type = source_type
        self.source_branch = source_branch
        self.source_tag = source_tag


class TestBasePlugin(tests.TestCase):

    def test_get_source_with_unrecognized_source_must_raise_exception(self):
        options = MockOptions('unrecognized://test_source')
        plugin = snapcraft.BasePlugin('test_plugin', options)
        with self.assertRaises(ValueError) as raised:
            plugin.pull()

        self.assertEqual(raised.exception.__str__(),
                         'no handler to manage source')

    @unittest.mock.patch('os.path.isdir')
    def test_local_non_dir_source_path_must_raise_exception(self, mock_isdir):
        options = MockOptions('file')
        mock_isdir.return_value = False
        plugin = snapcraft.BasePlugin('test_plugin', options)
        with self.assertRaises(ValueError) as raised:
            plugin.pull()

        mock_isdir.assert_called_once_with('file')

        self.assertEqual(raised.exception.__str__(),
                         'local source is not a directory')

    def test_build_with_subdir_copies_subdir(self):
        class Options:
            source_subdir = 'src'

        plugin = snapcraft.BasePlugin('test-part', Options())

        tmpdir = tempfile.TemporaryDirectory()
        self.addCleanup(tmpdir.cleanup)
        plugin.sourcedir = tmpdir.name
        subdir = os.path.join(plugin.sourcedir, plugin.options.source_subdir)
        os.mkdir(subdir)
        open(os.path.join(subdir, 'file'), 'w').close()

        tmpdir = tempfile.TemporaryDirectory()
        self.addCleanup(tmpdir.cleanup)
        plugin.builddir = tmpdir.name

        plugin.build()

        self.assertTrue(os.path.exists(os.path.join(plugin.builddir, 'file')))

    def test_build_without_subdir_copies_sourcedir(self):
        class Options:
            pass

        plugin = snapcraft.BasePlugin('test-part', Options())

        tmpdir = tempfile.TemporaryDirectory()
        self.addCleanup(tmpdir.cleanup)
        plugin.sourcedir = tmpdir.name
        subdir = os.path.join(plugin.sourcedir, 'src')
        os.mkdir(subdir)
        open(os.path.join(subdir, 'file'), 'w').close()

        tmpdir = tempfile.TemporaryDirectory()
        self.addCleanup(tmpdir.cleanup)
        plugin.builddir = tmpdir.name

        plugin.build()

        self.assertTrue(os.path.exists(
            os.path.join(plugin.builddir, 'src', 'file')))


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
        options = MockOptions('lp:source', self.source_type,
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
        options = MockOptions('lp:this', self.source_type, self.source_branch,
                              self.source_tag)
        plugin = snapcraft.BasePlugin('test_plugin', options)

        with self.assertRaises(sources.IncompatibleOptionsError) as raised:
            plugin.pull()

        self.assertEqual(
            raised.exception.__str__(),
            'can\'t specify a {} for a {} source'.format(
                self.error, self.source_type))


class BuildTestCase(tests.TestCase):

    def test_do_not_follow_links(self):
        options = MockOptions('.', None, None, None)
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
