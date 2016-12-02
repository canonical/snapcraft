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

        with self.assertRaises(EnvironmentError) as raised:
            gulp.GulpPlugin('test-part', Options(), self.project_options)

        self.assertEqual(raised.exception.__str__(),
                         'architecture not supported (fantasy-arch)')

    def test_schema(self):
        self.maxDiff = None
        plugin_schema = {
            '$schema': 'http://json-schema.org/draft-04/schema#',
            'additionalProperties': False,
            'properties': {
                'gulp-tasks': {'default': [],
                               'items': {'type': 'string'},
                               'minitems': 1,
                               'type': 'array',
                               'uniqueItems': True},
                'node-engine': {'default': '4.4.4', 'type': 'string'}},
            'pull-properties': ['node-engine'],
            'build-properties': ['gulp-tasks'],
            'required': ['gulp-tasks'],
            'type': 'object'}

        self.assertEqual(gulp.GulpPlugin.schema(), plugin_schema)

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
