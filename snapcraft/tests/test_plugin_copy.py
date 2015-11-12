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

import os.path
from unittest.mock import Mock

import fixtures

from snapcraft.plugins.copy import CopyPlugin
from snapcraft.tests import TestCase


class TestCopyPlugin(TestCase):

    def setUp(self):
        super().setUp()
        self.mock_options = Mock()
        self.mock_options.files = {}
        # setup the expected target dir in our tempdir
        parts_prefix = os.path.join('parts', 'copy')
        self.dst_prefix = os.path.join(parts_prefix, 'install')
        os.makedirs(self.dst_prefix)
        self.src_prefix = os.path.join(parts_prefix, 'src')
        os.makedirs(self.src_prefix)

    def test_copy_plugin_any_missing_src_raises_exception(self):
        # ensure that a bad file causes a warning and fails the build even
        # if there is a good file last
        self.mock_options.files = {
            'src': 'dst',
            'zzz': 'zzz',
        }
        open('zzz', 'w').close()
        c = CopyPlugin('copy', self.mock_options)

        with self.assertRaises(EnvironmentError) as raised:
            c.build()

        self.assertEqual(raised.exception.__str__(),
                         'file "src" missing')

    def test_copy_plugin_copies(self):
        self.useFixture(fixtures.FakeLogger())

        self.mock_options.files = {
            'src': 'dst',
        }
        open(os.path.join(self.src_prefix, 'src'), 'w').close()

        c = CopyPlugin('copy', self.mock_options)
        c.build()
        self.assertTrue(os.path.exists(os.path.join(self.dst_prefix, 'dst')))

    def test_copy_plugin_creates_prefixes(self):
        self.useFixture(fixtures.FakeLogger())

        self.mock_options.files = {
            'src': 'dir/dst',
        }
        open(os.path.join(self.src_prefix, 'src'), 'w').close()

        c = CopyPlugin('copy', self.mock_options)
        c.build()
        self.assertTrue(os.path.exists(os.path.join(self.dst_prefix,
                                                    'dir', 'dst')))

    def test_copy_plugin_glob(self):
        self.useFixture(fixtures.FakeLogger())

        self.mock_options.files = {
            '*.txt': '.',
        }

        for filename in ('file-a.txt', 'file-b.txt', 'file-c.notxt'):
            with open(os.path.join(self.src_prefix, filename), 'w') as \
                    datafile:
                datafile.write('data')

        c = CopyPlugin('copy', self.mock_options)
        c.build()

        self.assertTrue(os.path.exists(
            os.path.join(self.dst_prefix, 'file-a.txt')))
        self.assertTrue(os.path.exists(
            os.path.join(self.dst_prefix, 'file-b.txt')))
        self.assertFalse(os.path.exists(
            os.path.join(self.dst_prefix, 'file-c.notxt')))
