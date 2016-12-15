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

from snapcraft.internal import sources

from snapcraft.tests.sources import SourceTestCase


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
