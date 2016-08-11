# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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

from unittest import mock

import snapcraft
from snapcraft import tests
from snapcraft.plugins import shell


class ShellPluginTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        class Options:
            build_cmds = []
            disable_parallel = False

        self.options = Options()
        self.project_options = snapcraft.ProjectOptions()

    def test_schema(self):
        schema = shell.ShellPlugin.schema()
        properties = schema['properties']
        self.assertIn('build-cmds', properties,
                      'Expected "build-cmds" to be included in properties')
        self.assertIn('type', properties['build-cmds'],
                      'Expected "type" to be included in "build-cmds"')
        self.assertEqual(properties['build-cmds']['type'], 'array',
                         'Expected "build-cmds" "type" to be "array"')
        self.assertEqual(properties['build-cmds']['minitems'], 1,
                         'Expected "build-cmds" "minitems" to be 1')
        self.assertEqual(properties['build-cmds']['default'], [],
                         'Expected "build-cmds" "default" to be empty list')
        self.assertIn('items', properties['build-cmds'],
                      'Expected "items" to be included in "build-cmds"')
        self.assertIsInstance(properties['build-cmds']['items'], dict,
                              'Expected "build-cmds" "items" to be a dict')
        self.assertEqual(properties['build-cmds']['items']['type'], 'string',
                         'Expected "build-cmds" "items" "type" to be "string"')

    @mock.patch.object(shell.ShellPlugin, 'run')
    def test_build(self, mock_run):
        cmds = [
            ('singleword', ['singleword']),
            ('three words three', ['three', 'words', 'three']),
            ('make install', ['make', 'install']),
            ('./waf', ['./waf'])
        ]
        self.options.build_cmds = list(cmd for (cmd, _) in cmds)
        plugin = shell.ShellPlugin(
            'test-part', self.options, self.project_options)
        os.makedirs(plugin.sourcedir)
        plugin.build()
        self.assertEqual(len(cmds), mock_run.call_count)
        for ((_, expected), actual) in zip(cmds, mock_run.call_args_list):
            self.assertEqual(mock.call(expected), actual)
