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
        raised = self.assertRaises(
            sources.errors.IncompatibleOptionsError,
            sources.Bazaar,
            'lp:mysource', 'source_dir', source_branch='branch')

        expected_message = 'can\'t specify a source-branch for a bzr source'
        self.assertEqual(raised.message, expected_message)

    def test_init_with_source_depth_raises_exception(self):
        raised = self.assertRaises(
            sources.errors.IncompatibleOptionsError,
            sources.Bazaar,
            'lp://mysource', 'source_dir', source_depth=2)

        expected_message = (
            'can\'t specify source-depth for a bzr source')
        self.assertEqual(raised.message, expected_message)

    def test_init_with_source_tag_and_commit_raises_exception(self):
        raised = self.assertRaises(
            sources.errors.IncompatibleOptionsError,
            sources.Bazaar,
            'lp://mysource', 'source_dir', source_tag="tag",
            source_commit="2")

        expected_message = (
            'can\'t specify both source-tag and source-commit for '
            'a bzr source')
        self.assertEqual(raised.message, expected_message)
