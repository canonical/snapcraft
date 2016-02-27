
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

from unittest import mock

from snapcraft import tests
from snapcraft.plugins import cmake


class CMakeTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        patcher = mock.patch('snapcraft.common.run')
        self.run_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('sys.stdout')
        patcher.start()
        self.addCleanup(patcher.stop)

        class Options():
            configflags = []
            install_via = 'destdir'

        self.options = Options()

    def test_schema(self):
        schema = cmake.CMakePlugin.schema()

        # Verify the presence of all properties
        properties = schema['properties']
        self.assertTrue('configflags' in properties,
                        'Expected "configflags" to be included in properties')
        self.assertTrue('install-via' in properties,
                        'Expected "install-via" to be included in properties')

        # Check configflags property
        configflags = properties['configflags']
        self.assertTrue('type' in configflags,
                        'Expected "type" to be included in "configflags"')
        self.assertTrue('minitems' in configflags,
                        'Expected "minitems" to be included in "configflags"')
        self.assertTrue('uniqueItems' in configflags,
                        'Expected "uniqueItems" to be included in '
                        '"configflags"')
        self.assertTrue('items' in configflags,
                        'Expected "items" to be included in "configflags"')
        self.assertTrue('default' in configflags,
                        'Expected "default" to be included in "configflags"')

        configflags_type = configflags['type']
        self.assertEqual(configflags_type, 'array',
                         'Expected "configflags" "type" to be "array", but it '
                         'was "{}"'.format(configflags_type))

        configflags_uniqueItems = configflags['uniqueItems']
        self.assertTrue(configflags_uniqueItems,
                        'Expected "configflags" "uniqueItems" to be True')

        configflags_default = configflags['default']
        self.assertEqual(configflags_default, [],
                         'Expected "configflags" "default" to be "[]", but '
                         'it was "{}"'.format(configflags_default))

        configflags_items = configflags['items']
        self.assertTrue('type' in configflags_items,
                        'Expected "type" to be included in "configflags" '
                        '"items"')

        configflags_items_type = configflags_items['type']
        self.assertEqual(configflags_items_type, 'string',
                         'Expected "configflags" "item" "type" to be '
                         '"string", but it was "{}"'.format(
                            configflags_items_type))

        # Check install-via property
        installvia = properties['install-via']
        self.assertTrue('enum' in installvia,
                        'Expected "enum" to be included in "install-via"')
        self.assertTrue('default' in installvia,
                        'Expected "default" to be included in "install-via"')

        installvia_enum = installvia['enum']
        # Using sets for order independence in the comparison
        self.assertEqual(set(['destdir', 'prefix']), set(installvia_enum))

        installvia_default = installvia['default']
        self.assertEqual(installvia_default, 'destdir',
                         'Expected "install-via" "default" to be "destdir", '
                         'but it was "{}"'.format(installvia_default))

    def test_install_via_invalid_enum(self):
        self.options.install_via = 'invalid'
        with self.assertRaises(RuntimeError) as raised:
            cmake.CMakePlugin('test-part', self.options)

        self.assertEqual(str(raised.exception),
                         'Unsupported installation method: "invalid"')

    def test_build_referencing_sourcedir_if_no_subdir(self):
        plugin = cmake.CMakePlugin('test-part', self.options)
        os.makedirs(plugin.builddir)
        plugin.build()

        self.run_mock.assert_has_calls([
            mock.call(['cmake', plugin.sourcedir, '-DCMAKE_INSTALL_PREFIX='],
                      cwd=plugin.builddir),
            mock.call(['make', 'install',
                       'DESTDIR={}'.format(plugin.installdir)],
                      cwd=plugin.builddir)])

    def test_build_referencing_sourcedir_with_subdir(self):
        self.options.source_subdir = 'subdir'

        plugin = cmake.CMakePlugin('test-part', self.options)
        os.makedirs(plugin.builddir)
        plugin.build()

        sourcedir = os.path.join(
            plugin.sourcedir, plugin.options.source_subdir)
        self.run_mock.assert_has_calls([
            mock.call(['cmake', sourcedir, '-DCMAKE_INSTALL_PREFIX='],
                      cwd=plugin.builddir),
            mock.call(['make', 'install',
                       'DESTDIR={}'.format(plugin.installdir)],
                      cwd=plugin.builddir)])

    def test_build_referencing_sourcedir_if_no_subdir_via_prefix(self):
        self.options.install_via = 'prefix'

        plugin = cmake.CMakePlugin('test-part', self.options)
        os.makedirs(plugin.builddir)
        plugin.build()

        self.run_mock.assert_has_calls([
            mock.call(['cmake', plugin.sourcedir,
                       '-DCMAKE_INSTALL_PREFIX={}'.format(plugin.installdir)],
                      cwd=plugin.builddir),
            mock.call(['make', 'install'], cwd=plugin.builddir)])

    def test_build_referencing_sourcedir_with_subdir_via_prefix(self):
        self.options.source_subdir = 'subdir'
        self.options.install_via = 'prefix'

        plugin = cmake.CMakePlugin('test-part', self.options)
        os.makedirs(plugin.builddir)
        plugin.build()

        sourcedir = os.path.join(
            plugin.sourcedir, plugin.options.source_subdir)
        self.run_mock.assert_has_calls([
            mock.call(['cmake', sourcedir,
                       '-DCMAKE_INSTALL_PREFIX={}'.format(plugin.installdir)],
                      cwd=plugin.builddir),
            mock.call(['make', 'install'], cwd=plugin.builddir)])
