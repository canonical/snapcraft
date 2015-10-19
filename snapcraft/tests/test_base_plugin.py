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

import fixtures
import logging
import unittest.mock

import snapcraft
from snapcraft import tests


class MockOptions:

    def __init__(self, source, source_type=None, source_branch=None,
                 source_tag=None):
        self.source = source
        self.source_type = source_type
        self.source_branch = source_branch
        self.source_tag = source_tag


class TestBasePlugin(tests.TestCase):

    def test_get_source_with_unrecognized_source_must_raise_error(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        options = MockOptions('unrecognized://test_source')
        plugin = snapcraft.BasePlugin('test_plugin', options)
        with self.assertRaises(SystemExit) as raised:
            plugin.pull()

        self.assertEqual(raised.exception.code, 1, 'Wrong exit code returned.')
        expected = (
            "Unrecognized source 'unrecognized://test_source' for part "
            "'test_plugin': No handler to manage source.\n")
        self.assertEqual(expected, fake_logger.output)

    @unittest.mock.patch('os.path.isdir')
    def test_local_non_dir_source_path_must_raise_error(self, mock_isdir):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        options = MockOptions('file')
        mock_isdir.return_value = False
        plugin = snapcraft.BasePlugin('test_plugin', options)
        with self.assertRaises(SystemExit) as raised:
            plugin.pull()

        mock_isdir.assert_called_once_with('file')
        self.assertEqual(raised.exception.code, 1, 'Wrong exit code returned.')
        expected = (
            "Unrecognized source 'file' for part 'test_plugin': "
            "Local source is not a directory.\n")
        self.assertEqual(expected, fake_logger.output)


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
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        options = MockOptions('lp:source', self.source_type,
                              self.source_branch, self.source_tag)
        plugin = snapcraft.BasePlugin('test_plugin', options)
        with self.assertRaises(SystemExit) as raised:
            plugin.pull()

        self.assertEqual(raised.exception.code, 1, 'Wrong exit code returned.')
        expected = (
            'Issues while setting up sources for part \'test_plugin\': '
            'can\'t specify both source-tag and source-branch for a {} '
            "source.\n".format(self.source_type))
        self.assertEqual(expected, fake_logger.output)


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
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        options = MockOptions('lp:this', self.source_type, self.source_branch,
                              self.source_tag)
        plugin = snapcraft.BasePlugin('test_plugin', options)

        with self.assertRaises(SystemExit) as raised:
            plugin.pull()

        self.assertEqual(raised.exception.code, 1, 'Wrong exit code returned.')
        expected = (
            'Issues while setting up sources for part \'test_plugin\': can\'t '
            'specify a {} for a {} source.\n'
            .format(self.error, self.source_type))
        self.assertEqual(expected, fake_logger.output)
