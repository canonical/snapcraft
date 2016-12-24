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

import logging
import os
from unittest import mock

import fixtures
from testtools.matchers import HasLength

import snapcraft
from snapcraft import tests
from snapcraft.plugins import kbuild


class KBuildPluginTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        class Options:
            build_parameters = []
            kconfigfile = None
            kdefconfig = []
            kconfigs = []

        self.options = Options()
        self.project_options = snapcraft.ProjectOptions()

        patcher = mock.patch('snapcraft.repo.Ubuntu')
        self.ubuntu_mock = patcher.start()
        self.addCleanup(patcher.stop)

    def test_schema(self):
        schema = kbuild.KBuildPlugin.schema()

        properties = schema['properties']
        self.assertEqual(properties['kdefconfig']['type'], 'array')
        self.assertEqual(properties['kdefconfig']['default'], ['defconfig'])

        self.assertEqual(properties['kconfigfile']['type'], 'string')
        self.assertEqual(properties['kconfigfile']['default'], None)

        self.assertEqual(properties['kconfigs']['type'], 'array')
        self.assertEqual(properties['kconfigs']['default'], [])
        self.assertEqual(properties['kconfigs']['minitems'], 1)
        self.assertEqual(properties['kconfigs']['items']['type'], 'string')
        self.assertTrue(properties['kconfigs']['uniqueItems'])

    def test_get_build_properties(self):
        expected_build_properties = ['kdefconfig', 'kconfigfile', 'kconfigs']
        resulting_build_properties = kbuild.KBuildPlugin.get_build_properties()

        self.assertThat(resulting_build_properties,
                        HasLength(len(expected_build_properties)))

        for property in expected_build_properties:
            self.assertIn(property, resulting_build_properties)

    @mock.patch('subprocess.check_call')
    @mock.patch.object(kbuild.KBuildPlugin, 'run')
    def test_build_with_kconfigfile(self, run_mock, check_call_mock):
        self.options.kconfigfile = 'config'
        with open(self.options.kconfigfile, 'w') as f:
            f.write('ACCEPT=y\n')

        plugin = kbuild.KBuildPlugin('test-part', self.options,
                                     self.project_options)

        os.makedirs(plugin.builddir)

        plugin.build()

        self.assertEqual(1, check_call_mock.call_count)
        check_call_mock.assert_has_calls([
            mock.call('yes "" | make -j2 oldconfig', shell=True,
                      cwd=plugin.builddir),
        ])

        self.assertEqual(2, run_mock.call_count)
        run_mock.assert_has_calls([
            mock.call(['make', '-j2']),
            mock.call(['make', '-j2',
                       'CONFIG_PREFIX={}'.format(plugin.installdir),
                       'install'])
        ])

        config_file = os.path.join(plugin.builddir, '.config')
        self.assertTrue(os.path.exists(config_file))

        with open(config_file) as f:
            config_contents = f.read()

        self.assertEqual(config_contents, 'ACCEPT=y\n')

    @mock.patch('subprocess.check_call')
    @mock.patch.object(kbuild.KBuildPlugin, 'run')
    def test_build_verbose_with_kconfigfile(self, run_mock, check_call_mock):
        fake_logger = fixtures.FakeLogger(level=logging.DEBUG)
        self.useFixture(fake_logger)

        self.options.kconfigfile = 'config'
        with open(self.options.kconfigfile, 'w') as f:
            f.write('ACCEPT=y\n')

        plugin = kbuild.KBuildPlugin('test-part', self.options,
                                     self.project_options)

        os.makedirs(plugin.builddir)

        plugin.build()

        self.assertEqual(1, check_call_mock.call_count)
        check_call_mock.assert_has_calls([
            mock.call('yes "" | make -j2 V=1 oldconfig', shell=True,
                      cwd=plugin.builddir),
        ])

        self.assertEqual(2, run_mock.call_count)
        run_mock.assert_has_calls([
            mock.call(['make', '-j2', 'V=1']),
            mock.call(['make', '-j2', 'V=1',
                       'CONFIG_PREFIX={}'.format(plugin.installdir),
                       'install'])
        ])

        config_file = os.path.join(plugin.builddir, '.config')
        self.assertTrue(os.path.exists(config_file))

        with open(config_file) as f:
            config_contents = f.read()

        self.assertEqual(config_contents, 'ACCEPT=y\n')

    @mock.patch('subprocess.check_call')
    @mock.patch.object(kbuild.KBuildPlugin, 'run')
    def test_build_with_kconfigfile_and_kconfigs(
            self, run_mock, check_call_mock):
        self.options.kconfigfile = 'config'
        self.options.kconfigs = [
            'SOMETHING=y',
            'ACCEPT=n',
        ]

        with open(self.options.kconfigfile, 'w') as f:
            f.write('ACCEPT=y\n')

        plugin = kbuild.KBuildPlugin('test-part', self.options,
                                     self.project_options)

        os.makedirs(plugin.builddir)

        plugin.build()

        self.assertEqual(1, check_call_mock.call_count)
        check_call_mock.assert_has_calls([
            mock.call('yes "" | make -j2 oldconfig', shell=True,
                      cwd=plugin.builddir),
        ])

        self.assertEqual(2, run_mock.call_count)
        run_mock.assert_has_calls([
            mock.call(['make', '-j2']),
            mock.call(['make', '-j2',
                       'CONFIG_PREFIX={}'.format(plugin.installdir),
                       'install'])
        ])

        config_file = os.path.join(plugin.builddir, '.config')
        self.assertTrue(os.path.exists(config_file))

        with open(config_file) as f:
            config_contents = f.read()

        expected_config = """SOMETHING=y
ACCEPT=n

ACCEPT=y

SOMETHING=y
ACCEPT=n
"""
        self.assertEqual(config_contents, expected_config)

    @mock.patch('subprocess.check_call')
    @mock.patch.object(kbuild.KBuildPlugin, 'run')
    def test_build_with_defconfig_and_kconfigs(
            self, run_mock, check_call_mock):
        self.options.kdefconfig = ['defconfig']
        self.options.kconfigs = [
            'SOMETHING=y',
            'ACCEPT=n',
        ]

        plugin = kbuild.KBuildPlugin('test-part', self.options,
                                     self.project_options)

        config_file = os.path.join(plugin.builddir, '.config')

        def fake_defconfig(*args, **kwargs):
            if os.path.exists(config_file):
                return
            with open(config_file, 'w') as f:
                f.write('ACCEPT=y\n')

        run_mock.side_effect = fake_defconfig

        os.makedirs(plugin.builddir)

        plugin.build()

        self.assertEqual(1, check_call_mock.call_count)
        check_call_mock.assert_has_calls([
            mock.call('yes "" | make -j2 oldconfig', shell=True,
                      cwd=plugin.builddir),
        ])

        self.assertEqual(3, run_mock.call_count)
        run_mock.assert_has_calls([
            mock.call(['make', '-j2']),
            mock.call(['make', '-j2',
                       'CONFIG_PREFIX={}'.format(plugin.installdir),
                       'install'])
        ])

        self.assertTrue(os.path.exists(config_file))

        with open(config_file) as f:
            config_contents = f.read()

        expected_config = """SOMETHING=y
ACCEPT=n

ACCEPT=y

SOMETHING=y
ACCEPT=n
"""
        self.assertEqual(config_contents, expected_config)
