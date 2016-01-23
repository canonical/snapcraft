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
import stat

from unittest import mock

from snapcraft import tests
from snapcraft.plugins import autotools


class AutotoolsPluginTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        class Options:
            configflags = []

        self.options = Options()

        patcher = mock.patch('snapcraft.repo.Ubuntu')
        self.ubuntu_mock = patcher.start()
        self.addCleanup(patcher.stop)

    def test_schema(self):
        schema = autotools.AutotoolsPlugin.schema()

        properties = schema['properties']
        self.assertTrue('configflags' in properties,
                        'Expected "configflags" to be included in properties')

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

    @mock.patch.object(autotools.AutotoolsPlugin, 'run')
    def test_build_configure(self, run_mock):
        plugin = autotools.AutotoolsPlugin('test-part', self.options)
        os.makedirs(plugin.sourcedir)

        # Create both configure and autogen.sh.
        # Configure should take precedence.
        open(os.path.join(plugin.sourcedir, 'configure'), 'w').close()
        open(os.path.join(plugin.sourcedir, 'autogen.sh'), 'w').close()

        plugin.build()

        self.assertEqual(3, run_mock.call_count)
        run_mock.assert_has_calls([
            mock.call(['./configure', '--prefix=']),
            mock.call(['make']),
            mock.call(['make', 'install',
                       'DESTDIR={}'.format(plugin.installdir)])
        ])

    @mock.patch.object(autotools.AutotoolsPlugin, 'run')
    def test_build_autogen(self, run_mock):
        plugin = autotools.AutotoolsPlugin('test-part', self.options)
        os.makedirs(plugin.sourcedir)

        # No configure-- only autogen.sh. Make sure it's executable.
        open(os.path.join(plugin.sourcedir, 'autogen.sh'), 'w').close()
        os.chmod(os.path.join(plugin.sourcedir, 'autogen.sh'),
                 stat.S_IRUSR | stat.S_IWUSR | stat.S_IXUSR)

        plugin.build()

        self.assertEqual(4, run_mock.call_count)
        run_mock.assert_has_calls([
            mock.call(['env', 'NOCONFIGURE=1', './autogen.sh']),
            mock.call(['./configure', '--prefix=']),
            mock.call(['make']),
            mock.call(['make', 'install',
                       'DESTDIR={}'.format(plugin.installdir)])
        ])

    @mock.patch.object(autotools.AutotoolsPlugin, 'run')
    def test_build_autoreconf(self, run_mock):
        plugin = autotools.AutotoolsPlugin('test-part', self.options)
        os.makedirs(plugin.sourcedir)

        # No configure or autogen.sh.

        plugin.build()

        self.assertEqual(4, run_mock.call_count)
        run_mock.assert_has_calls([
            mock.call(['autoreconf', '-i']),
            mock.call(['./configure', '--prefix=']),
            mock.call(['make']),
            mock.call(['make', 'install',
                       'DESTDIR={}'.format(plugin.installdir)])
        ])

    @mock.patch('sys.stdout')
    def test_build_nonexecutable_autogen(self, stdout_mock):
        plugin = autotools.AutotoolsPlugin('test-part', self.options)
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

        try:
            plugin.build()
        except:
            self.fail('Expected build() to be able to handle non-executable '
                      'autogen.sh')
