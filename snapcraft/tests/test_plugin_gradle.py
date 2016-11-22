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
from snapcraft.plugins import gradle


class BaseGradlePluginTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        class Options:
            gradle_options = []
        self.options = Options()

        self.project_options = snapcraft.ProjectOptions()

        patcher = mock.patch('snapcraft.repo.Ubuntu')
        self.ubuntu_mock = patcher.start()
        self.addCleanup(patcher.stop)

        # unset http and https proxies.
        self.useFixture(fixtures.EnvironmentVariable('http_proxy', None))
        self.useFixture(fixtures.EnvironmentVariable('https_proxy', None))


class GradlePluginTestCase(BaseGradlePluginTestCase):

    def test_schema(self):
        schema = gradle.GradlePlugin.schema()

        properties = schema['properties']
        self.assertTrue('gradle-options' in properties,
                        'Expected "gradle-options" to be included in '
                        'properties')

        gradle_options = properties['gradle-options']

        self.assertTrue(
            'type' in gradle_options,
            'Expected "type" to be included in "gradle-options"')
        self.assertEqual(gradle_options['type'], 'array',
                         'Expected "gradle-options" "type" to be "array", but '
                         'it was "{}"'.format(gradle_options['type']))

        self.assertTrue(
            'minitems' in gradle_options,
            'Expected "minitems" to be included in "gradle-options"')
        self.assertEqual(gradle_options['minitems'], 1,
                         'Expected "gradle-options" "minitems" to be 1, but '
                         'it was "{}"'.format(gradle_options['minitems']))

        self.assertTrue(
            'uniqueItems' in gradle_options,
            'Expected "uniqueItems" to be included in "gradle-options"')
        self.assertTrue(
            gradle_options['uniqueItems'],
            'Expected "gradle-options" "uniqueItems" to be "True"')

    @mock.patch.object(gradle.GradlePlugin, 'run')
    def test_build(self, run_mock):
        plugin = gradle.GradlePlugin('test-part', self.options,
                                     self.project_options)

        def side(l):
            os.makedirs(os.path.join(plugin.builddir,
                        'build', 'libs'))
            open(os.path.join(plugin.builddir,
                 'build', 'libs', 'dummy.jar'), 'w').close()

        run_mock.side_effect = side
        os.makedirs(plugin.sourcedir)

        plugin.build()

        run_mock.assert_has_calls([
            mock.call(['./gradlew', 'jar']),
        ])

    @mock.patch.object(gradle.GradlePlugin, 'run')
    def test_build_war(self, run_mock):
        plugin = gradle.GradlePlugin('test-part', self.options,
                                     self.project_options)

        def side(l):
            os.makedirs(os.path.join(plugin.builddir,
                        'build', 'libs'))
            open(os.path.join(plugin.builddir,
                 'build', 'libs', 'dummy.war'), 'w').close()

        run_mock.side_effect = side
        os.makedirs(plugin.sourcedir)

        plugin.build()

        run_mock.assert_has_calls([
            mock.call(['./gradlew', 'jar']),
        ])

    @mock.patch.object(gradle.GradlePlugin, 'run')
    def test_build_fail(self, run_mock):
        plugin = gradle.GradlePlugin('test-part', self.options,
                                     self.project_options)

        os.makedirs(plugin.sourcedir)
        with self.assertRaises(RuntimeError):
            plugin.build()

        run_mock.assert_has_calls([
            mock.call(['./gradlew', 'jar']),
        ])


class GradleProxyTestCase(BaseGradlePluginTestCase):

    scenarios = [
        ('http proxy url', dict(
            env_var=('http_proxy', 'http://test_proxy'),
            expected_args=['-Dhttp.proxyHost=test_proxy'])),
        ('http proxy url and port', dict(
            env_var=('http_proxy', 'http://test_proxy:3000'),
            expected_args=['-Dhttp.proxyHost=test_proxy',
                           '-Dhttp.proxyPort=3000'])),
        ('https proxy url', dict(
            env_var=('https_proxy', 'https://test_proxy'),
            expected_args=['-Dhttps.proxyHost=test_proxy'])),
        ('https proxy url and port', dict(
            env_var=('https_proxy', 'https://test_proxy:3000'),
            expected_args=['-Dhttps.proxyHost=test_proxy',
                           '-Dhttps.proxyPort=3000'])),
    ]

    @mock.patch.object(gradle.GradlePlugin, 'run')
    def test_build_with_http_proxy(self, run_mock):
        var, value = self.env_var
        self.useFixture(fixtures.EnvironmentVariable(var, value))
        plugin = gradle.GradlePlugin('test-part', self.options,
                                     self.project_options)

        def side(l):
            os.makedirs(os.path.join(plugin.builddir,
                        'build', 'libs'))
            open(os.path.join(plugin.builddir,
                 'build', 'libs', 'dummy.war'), 'w').close()

        run_mock.side_effect = side
        os.makedirs(plugin.sourcedir)

        plugin.build()

        run_mock.assert_has_calls([
            mock.call(['./gradlew'] + self.expected_args + ['jar'])
        ])
