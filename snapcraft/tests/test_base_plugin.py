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
import os

import snapcraft
from snapcraft import tests


class TestBasePlugin(tests.TestCase):

    def test_isurl(self):
        plugin = snapcraft.BasePlugin('mock', {})
        self.assertTrue(plugin.isurl('git://'))
        self.assertTrue(plugin.isurl('bzr://'))
        self.assertFalse(plugin.isurl('./'))
        self.assertFalse(plugin.isurl('/foo'))
        self.assertFalse(plugin.isurl('/fo:o'))

    def test_get_source_with_unrecognized_source_must_raise_error(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        plugin = snapcraft.BasePlugin('test_plugin', 'dummy_options')
        with self.assertRaises(SystemExit) as raised:
            plugin.get_source('unrecognized://test_source')

        self.assertEqual(raised.exception.code, 1, 'Wrong exit code returned.')
        expected = (
            "Unrecognized source 'unrecognized://test_source' for part "
            "'test_plugin'.\n")
        self.assertEqual(expected, fake_logger.output)

    def test_makedirs_with_existing_dir(self):
        plugin = snapcraft.BasePlugin('dummy_plugin', 'dummy_options')
        plugin.makedirs(self.path)
        self.assertTrue(os.path.exists(self.path))

    def test_makedirs_with_unexisting_dir(self):
        path = os.path.join(self.path, 'unexisting')
        plugin = snapcraft.BasePlugin('dummy_plugin', 'dummy_options')
        plugin.makedirs(path)
        self.assertTrue(os.path.exists(path))

    def test_get_tar_source_from_uri(self):
        sources = [
            'https://golang.tar.gz',
            'https://golang.tar.xz',
            'https://golang.tar.bz2',
            'https://golang.tar.tgz',
        ]

        for source in sources:
            with self.subTest(key=source):
                self.assertEqual(
                    snapcraft._get_source_type_from_uri(source), 'tar')


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

        plugin = snapcraft.BasePlugin('test_plugin', 'dummy_options')
        with self.assertRaises(SystemExit) as raised:
            plugin.get_source(
                'dummy_source', source_type=self.source_type,
                source_branch=self.source_branch, source_tag=self.source_tag)

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

        plugin = snapcraft.BasePlugin('test_plugin', 'dummy_options')
        with self.assertRaises(SystemExit) as raised:
            plugin.get_source(
                'dummy_source', source_type=self.source_type,
                source_branch=self.source_branch, source_tag=self.source_tag)

        self.assertEqual(raised.exception.code, 1, 'Wrong exit code returned.')
        expected = (
            'Issues while setting up sources for part \'test_plugin\': can\'t '
            'specify a {} for a {} source.\n'
            .format(self.error, self.source_type))
        self.assertEqual(expected, fake_logger.output)
