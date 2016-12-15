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

from snapcraft.internal import sources

from snapcraft.tests.sources import SourceTestCase


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
