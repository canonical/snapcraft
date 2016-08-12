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
        self.options.build_cmds = ['singleword', 'one two three',
                                   'make install', './waf']
        plugin = shell.ShellPlugin('testpart', self.options,
                                   self.project_options)
        os.makedirs(plugin.sourcedir)
        plugin.build()
        env_data = {
            'PARTNAME': plugin.name,
            'BUILDDIR': plugin.builddir,
            'SRCDIR': plugin.sourcedir,
            'DESTDIR': plugin.installdir,
        }
        self.assertEqual(len(self.options.build_cmds), mock_run.call_count)
        for (cmd, (call_args, call_kwargs)) in zip(
                self.options.build_cmds,
                [list(call) for call in mock_run.call_args_list]):
            self.assertEqual(len(call_kwargs), 0)
            self.assertEqual(len(call_args), 1)
            self.assertEqual(len(call_args[0]), 3)
            (actual_bash, actual_dash_c, actual_cmd) = call_args[0]
            self.assertEqual(actual_bash, 'bash')
            self.assertEqual(actual_dash_c, '-c')
            self.assertTrue(actual_cmd.endswith(cmd))
            self.assertEqual({
                name: value.strip('"') for (name, value) in
                [mapping.split('=') for mapping in
                 (actual_cmd[:-len(cmd)]).split(';')
                 if len(mapping) != 0]
            }, env_data)
