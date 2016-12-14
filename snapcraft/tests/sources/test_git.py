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

from unittest import mock

from snapcraft.internal import sources

from snapcraft.tests.sources import SourceTestCase


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
            mock.call(['git', 'clone', '--recursive', 'git://my-source',
                       'source_dir']),
            mock.call(['git', '-C', 'source_dir', 'checkout',
                       '2514f9533ec9b45d07883e10a561b248497a8e3c'])
        ])

    def test_pull_existing(self):
        self.mock_path_exists.return_value = True

        git = sources.Git('git://my-source', 'source_dir')
        git.pull()

        self.mock_run.assert_has_calls([
            mock.call(['git', '-C', 'source_dir', 'pull',
                       '--recurse-submodules=yes', 'git://my-source', 'HEAD']),
            mock.call(['git', '-C', 'source_dir', 'submodule', 'update'])
        ])

    def test_pull_existing_with_tag(self):
        self.mock_path_exists.return_value = True

        git = sources.Git('git://my-source', 'source_dir', source_tag='tag')
        git.pull()

        self.mock_run.assert_has_calls([
            mock.call(['git', '-C', 'source_dir', 'pull',
                       '--recurse-submodules=yes', 'git://my-source',
                       'refs/tags/tag']),
            mock.call(['git', '-C', 'source_dir', 'submodule', 'update'])
        ])

    def test_pull_existing_with_commit(self):
        self.mock_path_exists.return_value = True

        git = sources.Git(
            'git://my-source', 'source_dir',
            source_commit='2514f9533ec9b45d07883e10a561b248497a8e3c')
        git.pull()

        self.mock_run.assert_has_calls([
            mock.call(['git', '-C', 'source_dir', 'pull',
                       '--recurse-submodules=yes', 'git://my-source',
                       '2514f9533ec9b45d07883e10a561b248497a8e3c']),
            mock.call(['git', '-C', 'source_dir', 'submodule', 'update'])
        ])

    def test_pull_existing_with_branch(self):
        self.mock_path_exists.return_value = True

        git = sources.Git('git://my-source', 'source_dir',
                          source_branch='my-branch')
        git.pull()

        self.mock_run.assert_has_calls([
            mock.call(['git', '-C', 'source_dir', 'pull',
                       '--recurse-submodules=yes', 'git://my-source',
                       'refs/heads/my-branch']),
            mock.call(['git', '-C', 'source_dir', 'submodule', 'update'])
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
