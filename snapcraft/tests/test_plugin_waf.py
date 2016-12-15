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
from testtools.matchers import HasLength

import snapcraft
from snapcraft import tests
from snapcraft.plugins import waf


class WafPluginTestCase(tests.TestCase):
    """Plugin to provide snapcraft support for the waf build system"""

    def setUp(self):
        super(WafPluginTestCase, self).setUp()

        class Options:
            """Internal Options Class matching the Waf plugin"""
            configflags = []

        self.options = Options()
        self.project_options = snapcraft.ProjectOptions()

        patcher = mock.patch('snapcraft.repo.Ubuntu')
        self.ubuntu_mock = patcher.start()
        self.addCleanup(patcher.stop)

    def test_schema(self):
        """Test validity of the Waf Plugin schema"""
        schema = waf.WafPlugin.schema()

        # Verify the presence of all properties
        properties = schema['properties']
        self.assertTrue('configflags' in properties,
                        'Expected "configflags" to be included in properties')

        # Check configflags property
        configflags = properties['configflags']
        for item in ['type', 'minitems', 'uniqueItems', 'items', 'default']:
            self.assertTrue(item in configflags,
                            'Expected "{}" to be included in "configflags"'
                            .format(item))

        configflags_type = configflags['type']
        self.assertEqual(configflags_type, 'array',
                         'Expected "configflags" "type" to be "array", but it '
                         'was "{}"'.format(configflags_type))

        configflags_minitems = configflags['minitems']
        self.assertEqual(configflags_minitems, 1,
                         'Expected "configflags" "minitems" to be 1, but '
                         'it was {}'.format(configflags_minitems))

        self.assertTrue(configflags['uniqueItems'])

        configflags_default = configflags['default']
        self.assertEqual(configflags_default, [],
                         'Expected "configflags" "default" to be [], but '
                         'it was {}'.format(configflags_default))

        configflags_items = configflags['items']
        self.assertTrue('type' in configflags_items,
                        'Expected "type" to be included in "configflags" '
                        '"items"')

        configflags_items_type = configflags_items['type']
        self.assertEqual(configflags_items_type, 'string',
                         'Expected "configflags" "items" "type" to be '
                         '"string", but it was "{}"'
                         .format(configflags_items_type))

        self.assertTrue('build-properties' in schema,
                        'Expected schema to include "build-properties"')

    def test_get_build_properties(self):
        expected_build_properties = ['configflags']
        resulting_build_properties = waf.WafPlugin.get_build_properties()

        self.assertThat(resulting_build_properties,
                        HasLength(len(expected_build_properties)))

        for property in expected_build_properties:
            self.assertIn(property, resulting_build_properties)

    def waf_build(self):
        """Helper to call a full build"""
        plugin = waf.WafPlugin('test-part', self.options,
                               self.project_options)
        os.makedirs(plugin.sourcedir)

        # Create fake waf
        open(os.path.join(plugin.sourcedir, 'waf'), 'w').close()

        plugin.build()

        return plugin

    @mock.patch.object(waf.WafPlugin, 'run')
    def test_build_with_destdir(self, run_mock):
        """Test building via waf and check for known calls and destdir"""
        plugin = self.waf_build()

        self.assertEqual(4, run_mock.call_count)
        run_mock.assert_has_calls([
            mock.call(['./waf', 'distclean']),
            mock.call(['./waf', 'configure']),
            mock.call(['./waf', 'build']),
            mock.call(['./waf', 'install',
                       '--destdir={}'.format(plugin.installdir)])
        ])
