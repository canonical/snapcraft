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

from os import path
from unittest import mock

import fixtures
from testtools.matchers import HasLength

import snapcraft
from snapcraft.plugins import gulp, nodejs
from snapcraft import tests


class GulpPluginTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        self.project_options = snapcraft.ProjectOptions()

        patcher = mock.patch('snapcraft.internal.common.run')
        self.run_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('snapcraft.sources.Tar')
        self.tar_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('sys.stdout')
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_pull_local_sources(self):
        class Options:
            source = '.'
            gulp_tasks = []
            node_engine = '4'

        plugin = gulp.GulpPlugin('test-part', Options(), self.project_options)

        os.makedirs(plugin.sourcedir)

        plugin.pull()

        self.assertFalse(self.run_mock.called, 'run() was called')
        self.tar_mock.assert_has_calls([
            mock.call(
                nodejs.get_nodejs_release(plugin.options.node_engine),
                path.join(os.path.abspath('.'), 'parts', 'test-part', 'npm')),
            mock.call().download()])

    def test_build(self):
        self.useFixture(tests.fixture_setup.CleanEnvironment())
        self.useFixture(fixtures.EnvironmentVariable(
            'PATH', '/bin'))

        class Options:
            source = '.'
            gulp_tasks = []
            node_engine = '4'

        plugin = gulp.GulpPlugin('test-part', Options(), self.project_options)

        os.makedirs(plugin.builddir)
        open(os.path.join(plugin.builddir, 'package.json'), 'w').close()

        plugin.build()

        path = '{}:/bin'.format(os.path.join(plugin._npm_dir, 'bin'))
        self.run_mock.assert_has_calls([
            mock.call(['npm', 'install', '-g', 'gulp-cli'],
                      cwd=plugin.builddir,
                      env={'PATH': path,
                           'NPM_CONFIG_PREFIX': plugin._npm_dir}),
            mock.call(['npm', 'install', '--only-development'],
                      cwd=plugin.builddir,
                      env={'PATH': path,
                           'NPM_CONFIG_PREFIX': plugin._npm_dir}),
        ])

        self.tar_mock.assert_has_calls([
            mock.call(
                nodejs.get_nodejs_release(plugin.options.node_engine),
                os.path.join(plugin._npm_dir)),
            mock.call().provision(
                plugin._npm_dir, clean_target=False, keep_tarball=True)])

    @mock.patch('platform.machine')
    def test_unsupported_arch_raises_exception(self, machine_mock):
        machine_mock.return_value = 'fantasy-arch'

        class Options:
            source = None
            gulp_tasks = []
            node_engine = '4'

        raised = self.assertRaises(
            EnvironmentError,
            gulp.GulpPlugin, 'test-part', Options(), self.project_options)

        self.assertEqual(raised.__str__(),
                         'architecture not supported (fantasy-arch)')

    def test_schema(self):
        schema = gulp.GulpPlugin.schema()

        properties = schema['properties']
        self.assertTrue('gulp-tasks' in properties,
                        'Expected "gulp-tasks" to be included in '
                        'properties')
        gulp_tasks = properties['gulp-tasks']

        self.assertTrue(
            'type' in gulp_tasks,
            'Expected "type" to be included in "gulp-tasks"')
        self.assertEqual(gulp_tasks['type'], 'array',
                         'Expected "gulp-tasks" "type" to be "array", but '
                         'it was "{}"'.format(gulp_tasks['type']))

        self.assertTrue(
            'minitems' in gulp_tasks,
            'Expected "minitems" to be included in "gulp-tasks"')
        self.assertEqual(gulp_tasks['minitems'], 1,
                         'Expected "gulp-tasks" "minitems" to be 1, but '
                         'it was "{}"'.format(gulp_tasks['minitems']))

        self.assertTrue(
            'uniqueItems' in gulp_tasks,
            'Expected "uniqueItems" to be included in "gulp-tasks"')
        self.assertTrue(
            gulp_tasks['uniqueItems'],
            'Expected "gulp-tasks" "uniqueItems" to be "True"')

        self.assertTrue('node-engine' in properties,
                        'Expected "node-engine" to be included in '
                        'properties')
        node_engine_type = properties['node-engine']['type']
        self.assertEqual(node_engine_type, 'string',
                         'Expected "node_engine" "type" to be '
                         '"string", but it was "{}"'
                         .format(node_engine_type))

    def test_get_build_properties(self):
        expected_build_properties = ['gulp-tasks']
        resulting_build_properties = gulp.GulpPlugin.get_build_properties()

        self.assertThat(resulting_build_properties,
                        HasLength(len(expected_build_properties)))

        for property in expected_build_properties:
            self.assertIn(property, resulting_build_properties)

    def test_get_pull_properties(self):
        expected_pull_properties = ['node-engine']
        resulting_pull_properties = gulp.GulpPlugin.get_pull_properties()

        self.assertThat(resulting_pull_properties,
                        HasLength(len(expected_pull_properties)))

        for property in expected_pull_properties:
            self.assertIn(property, resulting_pull_properties)

    def test_clean_pull_step(self):
        class Options:
            source = '.'
            gulp_tasks = []
            node_engine = '4'

        plugin = gulp.GulpPlugin('test-part', Options(), self.project_options)

        os.makedirs(plugin.sourcedir)

        plugin.pull()

        self.assertTrue(os.path.exists(plugin._npm_dir))

        plugin.clean_pull()

        self.assertFalse(os.path.exists(plugin._npm_dir))
