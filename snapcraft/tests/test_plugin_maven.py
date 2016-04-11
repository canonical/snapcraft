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

import fixtures

import snapcraft
from snapcraft import tests
from snapcraft.plugins import maven


class MavenPluginTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        class Options:
            maven_options = []

        self.options = Options()
        self.project_options = snapcraft.ProjectOptions()

        patcher = mock.patch('snapcraft.repo.Ubuntu')
        self.ubuntu_mock = patcher.start()
        self.addCleanup(patcher.stop)

    def test_schema(self):
        schema = maven.MavenPlugin.schema()

        properties = schema['properties']
        self.assertTrue('maven-options' in properties,
                        'Expected "maven-options" to be included in '
                        'properties')

        maven_options = properties['maven-options']

        self.assertTrue(
            'type' in maven_options,
            'Expected "type" to be included in "maven-options"')
        self.assertEqual(maven_options['type'], 'array',
                         'Expected "maven-options" "type" to be "array", but '
                         'it was "{}"'.format(maven_options['type']))

        self.assertTrue(
            'minitems' in maven_options,
            'Expected "minitems" to be included in "maven-options"')
        self.assertEqual(maven_options['minitems'], 1,
                         'Expected "maven-options" "minitems" to be 1, but '
                         'it was "{}"'.format(maven_options['minitems']))

        self.assertTrue(
            'uniqueItems' in maven_options,
            'Expected "uniqueItems" to be included in "maven-options"')
        self.assertTrue(
            maven_options['uniqueItems'],
            'Expected "maven-options" "uniqueItems" to be "True"')

    @mock.patch.object(maven.MavenPlugin, 'run')
    @mock.patch('glob.glob')
    def test_build(self, glob_mock, run_mock):
        plugin = maven.MavenPlugin('test-part', self.options,
                                   self.project_options)
        os.makedirs(plugin.sourcedir)
        glob_mock.return_value = [
            os.path.join(plugin.builddir, 'target', 'dummy')]

        plugin.build()

        run_mock.assert_has_calls([
            mock.call(['mvn', 'package']),
        ])

    @mock.patch.object(maven.MavenPlugin, 'run')
    @mock.patch('glob.glob')
    def test_build_with_snapcraft_proxy(self, glob_mock, run_mock):
        env_vars = (
            ('SNAPCRAFT_SETUP_PROXIES', '1',),
            ('http_proxy', 'http://localhost:3132'),
            ('no_proxy', None),
        )
        for v in env_vars:
            self.useFixture(fixtures.EnvironmentVariable(v[0], v[1]))

        plugin = maven.MavenPlugin('test-part', self.options,
                                   self.project_options)

        settings_path = os.path.join(plugin.partdir, 'm2', 'settings.xml')
        os.makedirs(plugin.sourcedir)
        glob_mock.return_value = [
            os.path.join(plugin.builddir, 'target', 'dummy')]

        plugin.build()

        run_mock.assert_has_calls([
            mock.call(['mvn', 'package', '-s', settings_path]),
        ])

        self.assertTrue(
            os.path.exists(settings_path),
            'expected {!r} to exist'.format(settings_path))

        with open(settings_path) as f:
            settings_contents = f.read()

        expected_contents = (
            '<settings xmlns="http://maven.apache.org/SETTINGS/1.0.0"\n'
            '          xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"\n'
            '          xsi:schemaLocation="http://maven.apache.org/SETTINGS/'
            '1.0.0 http://maven.apache.org/xsd/settings-1.0.0.xsd">\n'
            '  <interactiveMode>false</interactiveMode>\n'
            '  <proxies>\n'
            '    <proxy>\n'
            '      <id>proxy</id>\n'
            '      <active>true</active>\n'
            '      <protocol>http</protocol>\n'
            '      <host>localhost</host>\n'
            '      <port>3132</port>\n'
            '      <nonProxyHosts>localhost</nonProxyHosts>\n'
            '    </proxy>\n'
            '  </proxies>\n'
            '</settings>\n')
        self.assertEqual(settings_contents, expected_contents)

    @mock.patch.object(maven.MavenPlugin, 'run')
    @mock.patch('glob.glob')
    def test_build_with_proxy_and_no_proxy(self, glob_mock, run_mock):
        env_vars = (
            ('SNAPCRAFT_SETUP_PROXIES', '1',),
            ('http_proxy', 'http://localhost:3132'),
            ('no_proxy', 'internal'),
        )
        for v in env_vars:
            self.useFixture(fixtures.EnvironmentVariable(v[0], v[1]))

        plugin = maven.MavenPlugin('test-part', self.options,
                                   self.project_options)

        settings_path = os.path.join(plugin.partdir, 'm2', 'settings.xml')
        os.makedirs(plugin.sourcedir)
        glob_mock.return_value = [
            os.path.join(plugin.builddir, 'target', 'dummy')]

        plugin.build()

        run_mock.assert_has_calls([
            mock.call(['mvn', 'package', '-s', settings_path]),
        ])

        self.assertTrue(
            os.path.exists(settings_path),
            'expected {!r} to exist'.format(settings_path))

        with open(settings_path) as f:
            settings_contents = f.read()

        expected_contents = (
            '<settings xmlns="http://maven.apache.org/SETTINGS/1.0.0"\n'
            '          xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"\n'
            '          xsi:schemaLocation="http://maven.apache.org/SETTINGS/'
            '1.0.0 http://maven.apache.org/xsd/settings-1.0.0.xsd">\n'
            '  <interactiveMode>false</interactiveMode>\n'
            '  <proxies>\n'
            '    <proxy>\n'
            '      <id>proxy</id>\n'
            '      <active>true</active>\n'
            '      <protocol>http</protocol>\n'
            '      <host>localhost</host>\n'
            '      <port>3132</port>\n'
            '      <nonProxyHosts>internal</nonProxyHosts>\n'
            '    </proxy>\n'
            '  </proxies>\n'
            '</settings>\n')
        self.assertEqual(settings_contents, expected_contents)

    @mock.patch.object(maven.MavenPlugin, 'run')
    @mock.patch('glob.glob')
    def test_build_with_proxy_and_no_proxies(self, glob_mock, run_mock):
        env_vars = (
            ('SNAPCRAFT_SETUP_PROXIES', '1',),
            ('http_proxy', 'http://localhost:3132'),
            ('no_proxy', 'internal, pseudo-dmz'),
        )
        for v in env_vars:
            self.useFixture(fixtures.EnvironmentVariable(v[0], v[1]))

        plugin = maven.MavenPlugin('test-part', self.options,
                                   self.project_options)

        settings_path = os.path.join(plugin.partdir, 'm2', 'settings.xml')
        os.makedirs(plugin.sourcedir)
        glob_mock.return_value = [
            os.path.join(plugin.builddir, 'target', 'dummy')]

        plugin.build()

        run_mock.assert_has_calls([
            mock.call(['mvn', 'package', '-s', settings_path]),
        ])

        self.assertTrue(
            os.path.exists(settings_path),
            'expected {!r} to exist'.format(settings_path))

        with open(settings_path) as f:
            settings_contents = f.read()

        expected_contents = (
            '<settings xmlns="http://maven.apache.org/SETTINGS/1.0.0"\n'
            '          xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"\n'
            '          xsi:schemaLocation="http://maven.apache.org/SETTINGS/'
            '1.0.0 http://maven.apache.org/xsd/settings-1.0.0.xsd">\n'
            '  <interactiveMode>false</interactiveMode>\n'
            '  <proxies>\n'
            '    <proxy>\n'
            '      <id>proxy</id>\n'
            '      <active>true</active>\n'
            '      <protocol>http</protocol>\n'
            '      <host>localhost</host>\n'
            '      <port>3132</port>\n'
            '      <nonProxyHosts>internal|pseudo-dmz</nonProxyHosts>\n'
            '    </proxy>\n'
            '  </proxies>\n'
            '</settings>\n')
        self.assertEqual(settings_contents, expected_contents)
