# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2017 Canonical Ltd
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
from snapcraft import tests


class BazaarBaseTestCase(tests.sources.SourceTestCase):

    def call(self, cmd):
        """Call a command ignoring output."""
        subprocess.check_call(
            cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def call_with_output(self, cmd):
        """Return command output converted to a string."""
        return subprocess.check_output(cmd).decode('utf-8').strip()

    def rm_dir(self, dir):
        if os.path.exists(dir):
            shutil.rmtree(dir)

    def clean_dir(self, dir):
        self.rm_dir(dir)
        os.mkdir(dir)
        self.addCleanup(self.rm_dir, dir)


class TestBazaar(BazaarBaseTestCase):

    def setUp(self):
        super().setUp()

        # Mock _get_source_details() since not all tests have a
        # full repo checkout
        patcher = mock.patch('snapcraft.sources.Bazaar._get_source_details')
        self.mock_get_source_details = patcher.start()
        self.mock_get_source_details.return_value = ""
        self.addCleanup(patcher.stop)

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

    def test_source_checksum_raises_exception(self):
        raised = self.assertRaises(
            sources.errors.IncompatibleOptionsError,
            sources.Bazaar,
            'lp://mysource', 'source_dir',
            source_checksum="md5/d9210476aac5f367b14e513bdefdee08")

        expected_message = (
            "can't specify a source-checksum for a bzr source")
        self.assertEqual(raised.message, expected_message)


class BazaarDetailsTestCase(BazaarBaseTestCase):

    def setUp(self):
        self.working_tree = 'bzr-test'
        self.source_dir = 'bzr-checkout'
        self.clean_dir(self.working_tree)
        self.clean_dir(self.source_dir)
        os.chdir(self.working_tree)
        self.call(['bzr', 'init'])
        self.call(['bzr', 'whoami', 'Test User <test.user@example.com>'])
        with open('testing', 'w') as fp:
            fp.write('testing')
        self.call(['bzr', 'add', 'testing'])
        self.call(['bzr', 'commit', '-m', 'testing'])
        self.call(['bzr', 'tag', 'test-tag'])
        self.expected_commit = self.call_with_output(['bzr', 'revno', '.'])
        self.expected_tag = 'test-tag'

        os.chdir('..')

        self.bzr = sources.Bazaar(self.working_tree, self.source_dir,
                                  silent=True)
        self.bzr.pull()

        self.source_details = self.bzr._get_source_details()

        super().setUp()

    def test_bzr_details_commit(self):
        self.assertEqual(self.expected_commit, self.source_details['commit'])

    def test_bzr_details_tag(self):
        self.bzr = sources.Bazaar(self.working_tree, self.source_dir,
                                  source_tag='test-tag', silent=True)
        self.bzr.pull()

        self.source_details = self.bzr._get_source_details()
        self.assertEqual(self.expected_tag, self.source_details['tag'])
