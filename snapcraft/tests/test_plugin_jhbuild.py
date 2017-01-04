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

import snapcraft
import snapcraft.plugins.jhbuild
import unittest.mock


class JHBuildPluginTestCase(snapcraft.tests.TestCase):

    def _test_plugin(self):
        class Options:
            modules = ['gtk+']
            snap_name = 'test'

        return snapcraft.plugins.jhbuild.JHBuildPlugin('test', Options())

    def test_schema(self):
        schema = snapcraft.plugins.jhbuild.JHBuildPlugin.schema()

        for key in schema['properties']:
            self.assertIn('type', schema['properties'][key])

        for key in schema['required']:
            self.assertIn(key, schema['properties'])

        for key in schema['pull-properties']:
            self.assertIn(key, schema['properties'])

        for key in schema['build-properties']:
            self.assertIn(key, schema['properties'])

    def test_run_with_env(self):
        plugin = self._test_plugin()

        plugin.run(['printenv', 'ANSWER'], env={'ANSWER': 42})

        self.assertEqual(
            '42',
            plugin.run_output(['printenv', 'ANSWER'], env={'ANSWER': 42})
        )

    def test_run_with_input(self):
        plugin = self._test_plugin()

        plugin.run(['cat'], input='42')

        self.assertEqual(
            '42',
            plugin.run_output(['cat'], input='42')
        )

    def test_apt_file(self):
        plugin = self._test_plugin()
        updated = False

        def apt_file(*args, **kwargs):
            nonlocal updated

            if args == (['apt-file', 'update'],):
                updated = True
            elif args == (['apt-file', 'search', '/usr/include/stdlib.h'],):
                return (
                    'no: /usr/include/stdlib.h.no\n' +
                    'libc6-dev: /usr/include/stdlib.h\n' +
                    'no: /usr/include/stdlib.h.no\n'
                )
            elif args == (['apt-file', 'search', '/usr/bin/test'],):
                return (
                    'no: /usr/bin/test.no\n' +
                    'coreutils: /usr/bin/test\n' +
                    'no: /usr/bin/test.no\n'
                )
            elif args == (['apt-file', 'search', '/glib-2.0.pc'],):
                return (
                    'no: /usr/lib/pkgconfig/glib-2.0.pc.no\n' +
                    'libglib2.0-dev: /usr/lib/pkgconfig/glib-2.0.pc\n' +
                    'no: /usr/lib/pkgconfig/glib-2.0.pc.no\n'
                )

        plugin.run_output = unittest.mock.MagicMock(name='apt-file')
        plugin.run_output.side_effect = apt_file

        self.assertEqual(
            {'libc6-dev'},
            plugin.find_build_packages('c_include:stdlib.h')
        )

        self.assertTrue(updated)

        self.assertEqual(
            {'coreutils'},
            plugin.find_build_packages('path:test')
        )

        self.assertEqual(
            {'libglib2.0-dev'},
            plugin.find_build_packages('pkgconfig:glib-2.0')
        )

        self.assertEqual(
            {'python-libxml2'},
            plugin.find_build_packages('python2:libxml2')
        )

    def test_is_virtual_package(self):
        plugin = self._test_plugin()

        self.assertTrue(plugin.is_virtual_package('<www-browser>'))
        self.assertFalse(plugin.is_virtual_package('chromium-browser'))

    def test_is_build_package(self):
        plugin = self._test_plugin()

        self.assertTrue(plugin.is_build_package('libglib2.0-dev'))
        self.assertFalse(plugin.is_build_package('libglib2.0-0'))

    def test_find_provider(self):
        plugin = self._test_plugin()

        self.assertEqual(
            'libtiff5-dev',
            plugin.find_provider('libtiff-dev')
        )

    def test_find_dependencies(self):
        plugin = self._test_plugin()

        self.assertEqual(
            {'libgcc1'},
            plugin.find_dependencies('libc6')
        )

    def test_find_stage_packages(self):
        plugin = self._test_plugin()

        for package in plugin.find_stage_packages('libglib2.0-dev'):
            self.assertFalse(plugin.is_virtual_package(package))
            self.assertFalse(plugin.is_build_package(package))
