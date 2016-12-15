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
from snapcraft.plugins import scons


class SconsPluginTestCase(tests.TestCase):
    """Plugin to provide snapcraft support for the scons build system"""

    def setUp(self):
        super(SconsPluginTestCase, self).setUp()

        class Options:
            """Internal Options Class matching the Scons plugin"""
            scons_options = []

        self.options = Options()
        self.project_options = snapcraft.ProjectOptions()

        patcher = mock.patch('snapcraft.repo.Ubuntu')
        self.ubuntu_mock = patcher.start()
        self.addCleanup(patcher.stop)

    def test_schema(self):
        """Test validity of the Scons Plugin schema"""
        schema = scons.SconsPlugin.schema()

        # Verify the presence of all properties
        properties = schema['properties']
        self.assertTrue('scons-options' in properties,
                        'Expected "scons-options" to be included in '
                        'properties')

        maven_options = properties['scons-options']

        self.assertTrue(
            'type' in maven_options,
            'Expected "type" to be included in "scons-options"')
        self.assertEqual(maven_options['type'], 'array',
                         'Expected "scons-options" "type" to be "array", but '
                         'it was "{}"'.format(maven_options['type']))

        self.assertTrue(
            'minitems' in maven_options,
            'Expected "minitems" to be included in "scons-options"')
        self.assertEqual(maven_options['minitems'], 1,
                         'Expected "scons-options" "minitems" to be 1, but '
                         'it was "{}"'.format(maven_options['minitems']))

        self.assertTrue(
            'uniqueItems' in maven_options,
            'Expected "uniqueItems" to be included in "scons-options"')
        self.assertTrue(
            maven_options['uniqueItems'],
            'Expected "scons-options" "uniqueItems" to be "True"')

    def test_get_build_properties(self):
        plugin = scons.SconsPlugin('test-part', self.options,
                                   self.project_options)
        scons_build_properties = ['scons-options']
        for prop in scons_build_properties:
            self.assertTrue(prop in plugin.get_build_properties(),
                            'Expected "' + prop + '" to be included in '
                            'properties')

    def scons_build(self):
        """Helper to call a full build"""
        plugin = scons.SconsPlugin('test-part', self.options,
                                   self.project_options)
        os.makedirs(plugin.sourcedir)

        # Create fake scons
        open(os.path.join(plugin.sourcedir, 'scons'), 'w').close()

        plugin.build()

        return plugin

    @mock.patch.object(scons.SconsPlugin, 'run')
    def test_build_with_destdir(self, run_mock):
        """Test building via scons and check for known calls and destdir"""
        plugin = self.scons_build()
        env = os.environ.copy()
        env['DESTDIR'] = plugin.installdir

        self.assertEqual(2, run_mock.call_count)
        run_mock.assert_has_calls([
            mock.call(['scons']),
            mock.call(['scons', 'install'], env=env)
        ])
