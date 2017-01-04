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
import copy
from unittest import mock
from testtools.matchers import HasLength

import snapcraft
from snapcraft import tests
from snapcraft.plugins import ant


class AntPluginTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        class Options:
            ant_properties = {}
            ant_build_targets = None
        self.options = Options()

        self.project_options = snapcraft.ProjectOptions()

        patcher = mock.patch('snapcraft.repo.Ubuntu')
        self.ubuntu_mock = patcher.start()
        self.addCleanup(patcher.stop)

    def test_schema(self):
        schema = ant.AntPlugin.schema()

        properties = schema['properties']
        for expected in ['ant-properties', 'ant-build-targets']:
            self.assertTrue(
                expected in properties,
                'Expected {!r} to be included in properties'.format(expected))

        properties_type = schema['properties']['ant-properties']['type']
        self.assertEqual(properties_type, 'object',
                         'Expected "ant-properties" "type" to be "object", '
                         'but it was "{}"'.format(properties_type))
        build_targets_type = schema['properties']['ant-build-targets']['type']
        self.assertEqual(build_targets_type, 'array',
                         'Expected "ant-build-targets" "type" to be "object", '
                         'but it was "{}"'.format(build_targets_type))

    def test_get_build_properties(self):
        expected_build_properties = ['ant-build-targets', 'ant-properties']
        resulting_build_properties = ant.AntPlugin.get_build_properties()

        self.assertThat(resulting_build_properties,
                        HasLength(len(expected_build_properties)))

        for property in expected_build_properties:
            self.assertIn(property, resulting_build_properties)

    @mock.patch.object(ant.AntPlugin, 'run')
    def test_build(self, run_mock):
        plugin = ant.AntPlugin('test-part', self.options,
                               self.project_options)

        def side(l):
            os.makedirs(os.path.join(plugin.builddir, 'target'))
            open(os.path.join(plugin.builddir,
                 'target', 'dummy.jar'), 'w').close()

        run_mock.side_effect = side
        os.makedirs(plugin.sourcedir)

        plugin.build()

        run_mock.assert_has_calls([
            mock.call(['ant']),
        ])

    @mock.patch.object(ant.AntPlugin, 'run')
    def test_build_with_options(self, run_mock):
        options = copy.deepcopy(self.options)
        plugin = ant.AntPlugin('test-part', options,
                               self.project_options)
        options.ant_build_targets = ['artifacts', 'jar']
        options.ant_properties = {'basedir': '.',
                                  'dist.dir': plugin.installdir}

        os.makedirs(plugin.sourcedir)
        plugin.build()

        destination = '-Ddist.dir={}'.format(plugin.installdir)
        basedir = '-Dbasedir=.'
        args = run_mock.call_args[0][0]
        self.assertEqual(args[0], 'ant')
        self.assertEqual(args[1], 'artifacts')
        self.assertEqual(args[2], 'jar')
        self.assertIn(destination, args)
        self.assertIn(basedir, args)

    def test_env(self):
        plugin = ant.AntPlugin('test-part', self.options,
                               self.project_options)

        os.makedirs(os.path.join(plugin.installdir, 'jar'))
        open(os.path.join(plugin.installdir,
             'jar', 'lib1.jar'), 'w').close()
        open(os.path.join(plugin.installdir,
             'jar', 'lib2.jar'), 'w').close()
        plugin.env(plugin.partdir)
