# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2016 Canonical Ltd
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
import subprocess
from unittest import mock

from snapcraft.internal import sources

from snapcraft.tests.sources import SourceTestCase
from snapcraft import tests


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
            mock.call(['git', '-C', 'source_dir', 'fetch',
                       '--prune', '--recurse-submodules=yes']),
            mock.call(['git', '-C', 'source_dir', 'reset', '--hard',
                       'origin/master']),
            mock.call(['git', '-C', 'source_dir', 'submodule', 'update',
                       '--recursive', '--remote'])
        ])

    def test_pull_existing_with_tag(self):
        self.mock_path_exists.return_value = True

        git = sources.Git('git://my-source', 'source_dir', source_tag='tag')
        git.pull()

        self.mock_run.assert_has_calls([
            mock.call(['git', '-C', 'source_dir', 'fetch', '--prune',
                       '--recurse-submodules=yes']),
            mock.call(['git', '-C', 'source_dir', 'reset', '--hard',
                       'refs/tags/tag']),
            mock.call(['git', '-C', 'source_dir', 'submodule', 'update',
                       '--recursive', '--remote'])
        ])

    def test_pull_existing_with_commit(self):
        self.mock_path_exists.return_value = True

        git = sources.Git(
            'git://my-source', 'source_dir',
            source_commit='2514f9533ec9b45d07883e10a561b248497a8e3c')
        git.pull()

        self.mock_run.assert_has_calls([
            mock.call(['git', '-C', 'source_dir', 'fetch', '--prune',
                       '--recurse-submodules=yes']),
            mock.call(['git', '-C', 'source_dir', 'reset', '--hard',
                       '2514f9533ec9b45d07883e10a561b248497a8e3c']),
            mock.call(['git', '-C', 'source_dir', 'submodule', 'update',
                       '--recursive', '--remote'])
        ])

    def test_pull_existing_with_branch(self):
        self.mock_path_exists.return_value = True

        git = sources.Git('git://my-source', 'source_dir',
                          source_branch='my-branch')
        git.pull()

        self.mock_run.assert_has_calls([
            mock.call(['git', '-C', 'source_dir', 'fetch', '--prune',
                       '--recurse-submodules=yes']),
            mock.call(['git', '-C', 'source_dir', 'reset', '--hard',
                       'refs/heads/my-branch']),
            mock.call(['git', '-C', 'source_dir', 'submodule', 'update',
                       '--recursive', '--remote'])
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


class TestGitConflicts(tests.TestCase):
    """Test that git pull errors don't kill the parser"""

    def call(self, cmd):
        subprocess.check_call(
            cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def rm_dir(self, dir):
        if os.path.exists(dir):
            shutil.rmtree(dir)

    def clean_dir(self, dir):
        self.rm_dir(dir)
        os.mkdir(dir)
        self.addCleanup(self.rm_dir, dir)

    def clone_repo(self, repo, tree):
        self.clean_dir(tree)
        self.call(['git', 'clone', repo, tree])
        os.chdir(tree)
        self.call(['git', 'config', '--local', 'user.name',
                   '"Example Dev"'])
        self.call(['git', 'config', '--local', 'user.email',
                   'dev@example.com'])

    def add_file(self, filename, body, message):
        with open(filename, 'w') as fp:
            fp.write(body)

        self.call(['git', 'add', filename])
        self.call(['git', 'commit', '-am', message])

    def check_file_contents(self, path, expected):
        body = None
        with open(path) as fp:
            body = fp.read()
        self.assertEqual(body, expected)

    def test_git_conflicts(self):

        repo = '/tmp/conflict-test.git'
        working_tree = '/tmp/git-conflict-test'
        conflicting_tree = '{}-conflict'.format(working_tree)
        git = sources.Git(repo, working_tree, silent=True)

        self.clean_dir(repo)
        self.clean_dir(working_tree)
        self.clean_dir(conflicting_tree)

        os.chdir(repo)
        self.call(['git', 'init', '--bare'])

        self.clone_repo(repo, working_tree)

        # check out the original repo
        self.clone_repo(repo, conflicting_tree)

        # add a file to the repo
        os.chdir(working_tree)
        self.add_file('fake', 'fake 1', 'fake 1')
        self.call(['git', 'push', repo])

        git.pull()

        os.chdir(conflicting_tree)
        self.add_file('fake', 'fake 2', 'fake 2')
        self.call(['git', 'push', '-f', repo])

        os.chdir(working_tree)
        git.pull()

        body = None
        with open(os.path.join(working_tree, 'fake')) as fp:
            body = fp.read()

        self.assertEqual(body, 'fake 2')

    def test_git_submodules(self):
        """Test that updates to submodules are pulled"""
        repo = '/tmp/submodules.git'
        sub_repo = '/tmp/subrepo'
        working_tree = '/tmp/git-submodules'
        sub_working_tree = '/tmp/git-submodules-sub'
        git = sources.Git(repo, working_tree, silent=True)

        self.clean_dir(repo)
        self.clean_dir(sub_repo)
        self.clean_dir(working_tree)
        self.clean_dir(sub_working_tree)

        os.chdir(sub_repo)
        self.call(['git', 'init', '--bare'])

        self.clone_repo(sub_repo, sub_working_tree)
        self.add_file('sub-file', 'sub-file', 'sub-file')
        self.call(['git', 'push', sub_repo])

        os.chdir(repo)
        self.call(['git', 'init', '--bare'])

        self.clone_repo(repo, working_tree)
        self.call(['git', 'submodule', 'add', sub_repo])
        self.call(['git', 'commit', '-am', 'added submodule'])
        self.call(['git', 'push', repo])

        git.pull()

        self.check_file_contents(os.path.join(working_tree,
                                              'subrepo', 'sub-file'),
                                 'sub-file')

        # add a file to the repo
        os.chdir(sub_working_tree)
        self.add_file('fake', 'fake 1', 'fake 1')
        self.call(['git', 'push', sub_repo])

        os.chdir(working_tree)
        git.pull()

        self.check_file_contents(os.path.join(working_tree, 'subrepo', 'fake'),
                                 'fake 1')
