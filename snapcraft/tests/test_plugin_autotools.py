# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
# Copyright (C) 2016 Harald Sitter <sitter@kde.org>
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
import stat

from unittest import mock

import snapcraft
from snapcraft import tests
from snapcraft.plugins import autotools


class AutotoolsPluginTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        class Options:
            configflags = []
            install_via = 'destdir'
            disable_parallel = False

        self.options = Options()
        self.project_options = snapcraft.ProjectOptions()

        patcher = mock.patch('snapcraft.repo.Ubuntu')
        self.ubuntu_mock = patcher.start()
        self.addCleanup(patcher.stop)

    def test_schema(self):
        schema = autotools.AutotoolsPlugin.schema()

        # Verify the presence of all properties
        properties = schema['properties']
        self.assertTrue('configflags' in properties,
                        'Expected "configflags" to be included in properties')
        self.assertTrue('install-via' in properties,
                        'Expected "install-via" to be included in properties')

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

        self.assertTrue('build-properties' in schema,
                        'Expected schema to include "build-properties"')
        build_properties = schema['build-properties']
        self.assertEqual(2, len(build_properties))
        self.assertTrue('configflags' in build_properties)
        self.assertTrue('install-via' in build_properties)

    def test_install_via_invalid_enum(self):
        self.options.install_via = 'invalid'
        with self.assertRaises(RuntimeError) as raised:
            autotools.AutotoolsPlugin('test-part', self.options,
                                      self.project_options)

        self.assertEqual(str(raised.exception),
                         'Unsupported installation method: "invalid"')

    def build_with_configure(self):
        plugin = autotools.AutotoolsPlugin('test-part', self.options,
                                           self.project_options)
        os.makedirs(plugin.builddir)

        # Create both configure and autogen.sh.
        # Configure should take precedence.
        open(os.path.join(plugin.builddir, 'configure'), 'w').close()
        open(os.path.join(plugin.builddir, 'autogen.sh'), 'w').close()

        plugin.build()

        return plugin

    @mock.patch.object(autotools.AutotoolsPlugin, 'run')
    def test_build_configure_with_destdir(self, run_mock):
        plugin = self.build_with_configure()

        self.assertEqual(3, run_mock.call_count)
        run_mock.assert_has_calls([
            mock.call(['./configure', '--prefix=']),
            mock.call(['make', '-j2']),
            mock.call(['make', 'install',
                       'DESTDIR={}'.format(plugin.installdir)])
        ])

    @mock.patch.object(autotools.AutotoolsPlugin, 'run')
    def test_build_configure_with_prefix(self, run_mock):
        self.options.install_via = 'prefix'
        plugin = self.build_with_configure()

        self.assertEqual(3, run_mock.call_count)
        run_mock.assert_has_calls([
            mock.call(['./configure', '--prefix={}'.format(
                plugin.installdir)]),
            mock.call(['make', '-j2']),
            mock.call(['make', 'install'])
        ])

    def build_with_autogen(self, files=None, dirs=None):
        plugin = autotools.AutotoolsPlugin('test-part', self.options,
                                           self.project_options)
        os.makedirs(plugin.builddir)

        if not files:
            files = ['autogen.sh']
        if not dirs:
            dirs = []

        # No configure-- only autogen.sh. Make sure it's executable.
        for filename in files:
            open(os.path.join(plugin.builddir, filename), 'w').close()
            os.chmod(os.path.join(plugin.builddir, filename),
                     stat.S_IRUSR | stat.S_IWUSR | stat.S_IXUSR)
        for directory in dirs:
            os.makedirs(os.path.join(plugin.builddir, directory))

        plugin.build()

        return plugin

    @mock.patch.object(autotools.AutotoolsPlugin, 'run')
    def test_build_autogen_with_bootstrap_dir(self, run_mock):
        plugin = self.build_with_autogen(files=['README'], dirs=['bootstrap'])

        self.assertEqual(4, run_mock.call_count)
        run_mock.assert_has_calls([
            mock.call(['autoreconf', '-i']),
            mock.call(['./configure', '--prefix=']),
            mock.call(['make', '-j2']),
            mock.call(['make', 'install',
                       'DESTDIR={}'.format(plugin.installdir)])
        ])

    @mock.patch.object(autotools.AutotoolsPlugin, 'run')
    def test_build_autogen_with_destdir(self, run_mock):
        plugin = self.build_with_autogen()

        self.assertEqual(4, run_mock.call_count)
        run_mock.assert_has_calls([
            mock.call(['env', 'NOCONFIGURE=1', './autogen.sh']),
            mock.call(['./configure', '--prefix=']),
            mock.call(['make', '-j2']),
            mock.call(['make', 'install',
                       'DESTDIR={}'.format(plugin.installdir)])
        ])

    @mock.patch.object(autotools.AutotoolsPlugin, 'run')
    def test_build_bootstrap_with_destdir(self, run_mock):
        plugin = self.build_with_autogen(files=['bootstrap'])

        self.assertEqual(4, run_mock.call_count)
        run_mock.assert_has_calls([
            mock.call(['env', 'NOCONFIGURE=1', './bootstrap']),
            mock.call(['./configure', '--prefix=']),
            mock.call(['make', '-j2']),
            mock.call(['make', 'install',
                       'DESTDIR={}'.format(plugin.installdir)])
        ])

    @mock.patch.object(autotools.AutotoolsPlugin, 'run')
    def test_build_bootstrap_and_autogen_with_destdir(self, run_mock):
        plugin = self.build_with_autogen(files=['bootstrap', 'autogen.sh'])

        self.assertEqual(4, run_mock.call_count)
        run_mock.assert_has_calls([
            mock.call(['env', 'NOCONFIGURE=1', './autogen.sh']),
            mock.call(['./configure', '--prefix=']),
            mock.call(['make', '-j2']),
            mock.call(['make', 'install',
                       'DESTDIR={}'.format(plugin.installdir)])
        ])

    @mock.patch.object(autotools.AutotoolsPlugin, 'run')
    def test_build_autogen_with_prefix(self, run_mock):
        self.options.install_via = 'prefix'
        plugin = self.build_with_autogen()

        self.assertEqual(4, run_mock.call_count)
        run_mock.assert_has_calls([
            mock.call(['env', 'NOCONFIGURE=1', './autogen.sh']),
            mock.call(['./configure', '--prefix={}'.format(
                plugin.installdir)]),
            mock.call(['make', '-j2']),
            mock.call(['make', 'install'])
        ])

    def build_with_autoreconf(self):
        plugin = autotools.AutotoolsPlugin('test-part', self.options,
                                           self.project_options)
        os.makedirs(plugin.sourcedir)

        # No configure or autogen.sh.

        plugin.build()

        return plugin

    @mock.patch.object(autotools.AutotoolsPlugin, 'run')
    def test_build_autoreconf_with_destdir(self, run_mock):
        plugin = self.build_with_autoreconf()

        self.assertEqual(4, run_mock.call_count)
        run_mock.assert_has_calls([
            mock.call(['autoreconf', '-i']),
            mock.call(['./configure', '--prefix=']),
            mock.call(['make', '-j2']),
            mock.call(['make', 'install',
                       'DESTDIR={}'.format(plugin.installdir)])
        ])

    @mock.patch.object(autotools.AutotoolsPlugin, 'run')
    def test_build_autoreconf_with_prefix(self, run_mock):
        self.options.install_via = 'prefix'
        plugin = self.build_with_autoreconf()

        self.assertEqual(4, run_mock.call_count)
        run_mock.assert_has_calls([
            mock.call(['autoreconf', '-i']),
            mock.call(['./configure', '--prefix={}'.format(
                plugin.installdir)]),
            mock.call(['make', '-j2']),
            mock.call(['make', 'install'])
        ])

    @mock.patch.object(autotools.AutotoolsPlugin, 'run')
    def test_build_autoreconf_with_disable_parallel(self, run_mock):
        self.options.disable_parallel = True
        plugin = self.build_with_autoreconf()

        self.assertEqual(4, run_mock.call_count)
        run_mock.assert_has_calls([
            mock.call(['autoreconf', '-i']),
            mock.call(['./configure', '--prefix=']),
            mock.call(['make', '-j1']),
            mock.call(['make', 'install',
                       'DESTDIR={}'.format(plugin.installdir)])
        ])

    @mock.patch('sys.stdout')
    def test_build_nonexecutable_autogen(self, stdout_mock):
        plugin = autotools.AutotoolsPlugin('test-part', self.options,
                                           self.project_options)
        os.makedirs(plugin.sourcedir)

        # Make a non-executable autogen.sh
        with open(os.path.join(plugin.sourcedir, 'autogen.sh'), 'w') as f:
            f.write('#!/bin/sh')

        patcher = mock.patch.object(autotools.AutotoolsPlugin, 'run')
        run_mock = patcher.start()

        # We want to mock out every run() call except the one to autogen
        def _run(cmd):
            if './autogen.sh' in cmd:
                patcher.stop()
                output = plugin.run(cmd)
                patcher.start()
                return output

        run_mock.side_effect = _run

        # An exception will be raised if build can't handle the non-executable
        # autogen.
        plugin.build()

    def test_fileset_ignores(self):
        plugin = autotools.AutotoolsPlugin('test-part', self.options,
                                           self.project_options)
        expected_fileset = [
            '-**/*.la',
        ]
        fileset = plugin.snap_fileset()
        self.assertListEqual(expected_fileset, fileset)
