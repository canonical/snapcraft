
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
from snapcraft.plugins import script


class ScriptPluginTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        class Options:
            script_name = 'build.sh'
            stages = ['pull', 'build']
            build_arguments = ['test-arg1']
            pull_arguments = ['test-arg2']

        self.options = Options()
        self.project_options = snapcraft.ProjectOptions()

        patcher = mock.patch('snapcraft.repo.Ubuntu')
        self.ubuntu_mock = patcher.start()
        self.addCleanup(patcher.stop)

    def test_schema(self):
        schema = script.ScriptPlugin.schema()

        properties = schema['properties']
        self.assertTrue('script-name' in properties,
                        'Expected "script-name" to be included in properties')

        script_name = properties['script-name']
        self.assertTrue('type' in script_name,
                        'Expected "type" to be included in "script-name"')

        script_type = script_name['type']
        self.assertEqual(script_type, 'string',
                         'Expected "stages" "type" to be "string", but it '
                         'was "{}"'.format(script_type))

        self.assertTrue('stages' in properties,
                        'Expected "stages" to be included in properties')

        stages = properties['stages']
        self.assertTrue('type' in stages,
                        'Expected "type" to be included in "stages"')

        stages_type = stages['type']
        self.assertEqual(stages_type, 'array',
                         'Expected "stages" "type" to be "array", but it '
                         'was "{}"'.format(stages_type))

        self.assertTrue('build-arguments' in properties,
                        'Expected "build-arguments" to be included '
                        'in properties')

        build_arguments = properties['build-arguments']
        self.assertTrue('type' in build_arguments,
                        'Expected "type" to be included in "stages"')

        build_arguments_type = build_arguments['type']
        self.assertEqual(stages_type, 'array',
                         'Expected "build-arguments" "type" to be "array", but'
                         ' it was "{}"'.format(build_arguments_type))

        self.assertTrue('pull-arguments' in properties,
                        'Expected "pull-arguments" to be included '
                        'in properties')

        pull_arguments = properties['pull-arguments']
        self.assertTrue('type' in pull_arguments,
                        'Expected "type" to be included in "pull-arguments"')

        pull_arguments_type = stages['type']
        self.assertEqual(pull_arguments_type, 'array',
                         'Expected "pull-arguments" "type" to be "array", '
                         'but it was "{}"'.format(pull_arguments_type))

        required = schema['required']
        self.assertEqual(2, len(required))
        self.assertTrue('source' in required)
        self.assertTrue('script-name' in required)
        build_properties = schema['build-properties']
        self.assertEqual(3, len(build_properties))
        self.assertTrue('stages' in build_properties)
        self.assertTrue('build-arguments' in build_properties)
        self.assertTrue('pull-arguments' in build_properties)

    @mock.patch.object(script.ScriptPlugin, 'run')
    def test_build(self, run_mock):
        self.options.build_arguments = []
        plugin = script.ScriptPlugin('test-part', self.options,
                                     self.project_options)
        os.makedirs(plugin.sourcedir)
        script_dir = os.path.join(plugin.sourcedir, self.options.script_name)

        plugin.build()

        self.assertEqual(1, run_mock.call_count)
        run_mock.assert_has_calls([
            mock.call([script_dir,
                       '{}'.format(plugin.installdir)])
        ])

    @mock.patch.object(script.ScriptPlugin, 'run')
    def test_build_arguments(self, run_mock):
        plugin = script.ScriptPlugin('test-part', self.options,
                                     self.project_options)
        os.makedirs(plugin.sourcedir)
        script_dir = os.path.join(plugin.sourcedir, self.options.script_name)

        plugin.build()

        self.assertEqual(1, run_mock.call_count)
        run_mock.assert_has_calls([
            mock.call([script_dir,
                       self.options.build_arguments[0],
                       '{}'.format(plugin.installdir)])
        ])

    @mock.patch.object(script.ScriptPlugin, 'run')
    def test_pull(self, run_mock):
        plugin = script.ScriptPlugin('test-part', self.options,
                                     self.project_options)
        os.makedirs(plugin.sourcedir)
        script_dir = os.path.join(plugin.sourcedir, self.options.script_name)

        plugin.pull()

        self.assertEqual(1, run_mock.call_count)
        run_mock.assert_has_calls([
            mock.call([script_dir,
                       self.options.pull_arguments[0]])
        ])

    @mock.patch.object(script.ScriptPlugin, 'run')
    def test_stages(self, run_mock):
        self.options.stages = ['build']
        plugin = script.ScriptPlugin('test-part', self.options,
                                     self.project_options)
        os.makedirs(plugin.sourcedir)

        plugin.pull()

        self.assertEqual(0, run_mock.call_count)
        run_mock.assert_has_calls([
            # no calls
        ])

        self.options.stages = ['pull']
        plugin = script.ScriptPlugin('test-part', self.options,
                                     self.project_options)

        plugin.build()

        self.assertEqual(0, run_mock.call_count)
        run_mock.assert_has_calls([
            # no calls
        ])
