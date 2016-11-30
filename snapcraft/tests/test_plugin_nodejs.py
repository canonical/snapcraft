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

import os

from os import path
from unittest import mock

import snapcraft
from snapcraft.plugins import nodejs
from snapcraft import tests


class NodePluginTestCase(tests.TestCase):

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
            node_packages = []
            node_engine = '4'
            npm_run = []

        plugin = nodejs.NodePlugin('test-part', Options(),
                                   self.project_options)

        os.makedirs(plugin.sourcedir)

        plugin.pull()

        self.assertFalse(self.run_mock.called, 'run() was called')
        self.tar_mock.assert_has_calls([
            mock.call(
                nodejs.get_nodejs_release(plugin.options.node_engine),
                path.join(os.path.abspath('.'), 'parts', 'test-part', 'npm')),
            mock.call().download()])

    def test_build_local_sources(self):
        class Options:
            source = '.'
            node_packages = []
            node_engine = '4'
            npm_run = []

        plugin = nodejs.NodePlugin('test-part', Options(),
                                   self.project_options)

        os.makedirs(plugin.builddir)
        open(os.path.join(plugin.builddir, 'package.json'), 'w').close()

        plugin.build()

        self.run_mock.assert_has_calls([
            mock.call(['npm', '--cache-min=Infinity', 'install'],
                      cwd=plugin.builddir),
            mock.call(['npm', '--cache-min=Infinity', 'install', '--global'],
                      cwd=plugin.builddir)])
        self.tar_mock.assert_has_calls([
            mock.call(
                nodejs.get_nodejs_release(plugin.options.node_engine),
                path.join(os.path.abspath('.'), 'parts', 'test-part', 'npm')),
            mock.call().provision(
                plugin.installdir, clean_target=False, keep_tarball=True)])

    def test_pull_and_build_node_packages_sources(self):
        class Options:
            source = None
            node_packages = ['my-pkg']
            node_engine = '4'
            npm_run = []

        plugin = nodejs.NodePlugin('test-part', Options(),
                                   self.project_options)

        os.makedirs(plugin.sourcedir)

        plugin.pull()
        plugin.build()

        self.run_mock.assert_has_calls([
            mock.call(['npm', '--cache-min=Infinity', 'install', '--global',
                       'my-pkg'], cwd=plugin.builddir)])
        self.tar_mock.assert_has_calls([
            mock.call(
                nodejs.get_nodejs_release(plugin.options.node_engine),
                path.join(os.path.abspath('.'), 'parts', 'test-part', 'npm')),
            mock.call().download(),
            mock.call().provision(
                plugin.installdir, clean_target=False, keep_tarball=True)])

    def test_build_executes_npm_run_commands(self):
        class Options:
            source = '.'
            node_packages = []
            node_engine = '4'
            npm_run = ['command_one', 'avocado']

        plugin = nodejs.NodePlugin('test-part', Options(),
                                   self.project_options)

        os.makedirs(plugin.sourcedir)
        open(os.path.join(plugin.sourcedir, 'package.json'), 'w').close()

        plugin.build()

        self.run_mock.assert_has_calls([
            mock.call(['npm', 'run', 'command_one'],
                      cwd=plugin.builddir),
            mock.call(['npm', 'run', 'avocado'],
                      cwd=plugin.builddir)])

    @mock.patch('platform.machine')
    def test_unsupported_arch_raises_exception(self, machine_mock):
        machine_mock.return_value = 'fantasy-arch'

        class Options:
            source = None
            node_packages = []
            node_engine = '4'
            npm_run = []

        with self.assertRaises(EnvironmentError) as raised:
            nodejs.NodePlugin('test-part', Options(),
                              self.project_options)

        self.assertEqual(raised.exception.__str__(),
                         'architecture not supported (fantasy-arch)')

    def test_schema(self):
        self.maxDiff = None
        plugin_schema = {
            '$schema': 'http://json-schema.org/draft-04/schema#',
            'additionalProperties': False,
            'properties': {
                'node-engine': {'default': '4.4.4', 'type': 'string'},
                'node-packages': {'default': [],
                                  'items': {'type': 'string'},
                                  'minitems': 1,
                                  'type': 'array',
                                  'uniqueItems': True},
                'npm-run': {'default': [],
                            'items': {'type': 'string'},
                            'minitems': 1,
                            'type': 'array',
                            'uniqueItems': False}},
            'pull-properties': ['node-engine'],
            'build-properties': ['node-packages', 'npm-run'],
            'type': 'object'}

        self.assertEqual(nodejs.NodePlugin.schema(), plugin_schema)

    @mock.patch('snapcraft.BasePlugin.schema')
    def test_required_not_in_parent_schema(self, schema_mock):
        schema_mock.return_value = {
            'properties': {},
            'pull-properties': [],
            'build-properties': []
        }
        self.assertTrue('required' not in nodejs.NodePlugin.schema())

    def test_clean_pull_step(self):
        class Options:
            source = '.'
            node_packages = []
            node_engine = '4'
            npm_run = []

        plugin = nodejs.NodePlugin('test-part', Options(),
                                   self.project_options)

        os.makedirs(plugin.sourcedir)

        plugin.pull()

        self.assertTrue(os.path.exists(plugin._npm_dir))

        plugin.clean_pull()

        self.assertFalse(os.path.exists(plugin._npm_dir))


class NodeReleaseTestCase(tests.TestCase):

    scenarios = [
        ('i686', dict(
            machine='i686',
            engine='4.4.4',
            expected_url=(
                'https://nodejs.org/dist/v4.4.4/'
                'node-v4.4.4-linux-x86.tar.gz'))),
        ('x86_64', dict(
            machine='x86_64',
            engine='4.4.4',
            expected_url=(
                'https://nodejs.org/dist/v4.4.4/'
                'node-v4.4.4-linux-x64.tar.gz'))),
        ('armv7l', dict(
             machine='armv7l',
             engine='4.4.4',
             expected_url=(
                 'https://nodejs.org/dist/v4.4.4/'
                 'node-v4.4.4-linux-armv7l.tar.gz'))),
        ('aarch64', dict(
            machine='aarch64',
            engine='4.4.4',
            expected_url=(
                'https://nodejs.org/dist/v4.4.4/'
                'node-v4.4.4-linux-arm64.tar.gz'))),
    ]

    @mock.patch('platform.machine')
    def test_get_nodejs_release(self, machine_mock):
        machine_mock.return_value = self.machine
        node_url = nodejs.get_nodejs_release(self.engine)
        self.assertEqual(node_url, self.expected_url)
