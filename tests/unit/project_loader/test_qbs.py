# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Dan Chapman <dpniel@ubuntu.com>
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
import shutil
from unittest import mock
from testtools.matchers import HasLength

import snapcraft
from snapcraft import tests
from snapcraft.plugins import qbs


class QbsPluginTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        self.project_options = snapcraft.ProjectOptions()

        class Options:
            qbs_options = []
            qt_version = 'qt5'
            qbs_profile = 'clang'
            qbs_build_variant = 'release'

        self.options = Options()

    def test_schema(self):
        schema = qbs.QbsPlugin.schema()

        # Check all properties exist.
        properties = schema['properties']
        self.assertTrue('qbs-options' in properties,
                        'Expected "options" to be in properties')
        self.assertTrue('qt-version' in properties,
                        'Expected "qt-version" to be in properties')
        self.assertTrue('qbs-profile' in properties,
                        'Expected "profile" to be in properties')
        self.assertTrue('qbs-build-variant' in properties,
                        'Expected "build-variant" to be in properties')

        # Check the options property
        options = properties['qbs-options']
        for item in ['type', 'minitems', 'uniqueItems', 'items', 'default']:
            self.assertTrue(item in options,
                            'Expected {!r} to be included in "options"'
                            .format(item))

        options_type = options['type']
        self.assertEqual(options_type, 'array',
                         'Expected "options" "type" to be "array", but it '
                         'was "{}"'.format(options_type))

        options_minitems = options['minitems']
        self.assertEqual(options_minitems, 1,
                         'Expected "options" "minitems" to be 1, but '
                         'it was {}'.format(options_minitems))

        self.assertTrue(options['uniqueItems'])

        options_default = options['default']
        self.assertEqual(options_default, [],
                         'Expected "options" "default" to be [], but '
                         'it was {}'.format(options_default))

        options_items = options['items']
        self.assertTrue('type' in options_items,
                        'Expected "type" to be included in "options" '
                        '"items"')

        options_items_type = options_items['type']
        self.assertEqual(options_items_type, 'string',
                         'Expected "options" "items" "type" to be '
                         '"string", but it was "{}"'
                         .format(options_items_type))

        # check the qt-version property
        qt_version = properties['qt-version']
        self.assertTrue('enum' in qt_version,
                        'Expected "enum" to be included in "qt_version"')
        self.assertTrue('default' in qt_version,
                        '"default" was unexpectedly included in "qt_version"')

        qt_version_enum = qt_version['enum']
        # Using sets for order independence in the comparison
        self.assertEqual(set(['qt4', 'qt5']), set(qt_version_enum))

        # Check the profile property
        profile = properties['qbs-profile']
        self.assertTrue('enum' in profile,
                        'Expected "enum" to be included in "profile"')
        self.assertTrue('default' in profile,
                        'Expected "default" to be included in "profile"')
        profile_enum = profile['enum']
        self.assertEqual(set(['gcc', 'clang']), set(profile_enum))
        profile_default = profile['default']
        self.assertEqual('gcc', profile_default)
        # check the build-variant property
        build_variant = properties['qbs-build-variant']
        self.assertTrue('enum' in build_variant,
                        'Expected "enum" to be included in "profile"')
        self.assertTrue('default' in build_variant,
                        'Expected "default" to be included in "profile"')
        build_variant_enum = build_variant['enum']
        self.assertEqual(set(['debug', 'release']), set(build_variant_enum))
        build_variant_default = build_variant['default']
        self.assertEqual('release', build_variant_default)
        # Check build and pull properties
        self.assertTrue('build-properties' in schema,
                        'Expected schema to include "build-properties"')

        self.assertTrue('pull-properties' in schema,
                        'Expected schema to include "pull-properties"')

    def test_get_build_properties(self):
        expected_build_properties = ['qbs-options', 'qbs-build-variant']
        resulting_build_properties = qbs.QbsPlugin.get_build_properties()

        self.assertThat(resulting_build_properties,
                        HasLength(len(expected_build_properties)))

        for property in expected_build_properties:
            self.assertIn(property, resulting_build_properties)

    def test_get_pull_properties(self):
        expected_pull_properties = ['qt-version', 'qbs-profile']
        resulting_pull_properties = qbs.QbsPlugin.get_pull_properties()

        self.assertThat(resulting_pull_properties,
                        HasLength(len(expected_pull_properties)))

        for property in expected_pull_properties:
            self.assertIn(property, resulting_pull_properties)

    def test_unsupported_qt_version(self):
        self.options.qt_version = 'qt3'
        raised = self.assertRaises(
            RuntimeError,
            qbs.QbsPlugin,
            'test-part', self.options, self.project_options)

        self.assertEqual(str(raised),
                         "Unsupported Qt version: 'qt3'")

    @mock.patch.object(qbs.QbsPlugin, 'run')
    def test_build_nil(self, run_mock):
        self.options.qt_version = None
        plugin = qbs.QbsPlugin('test-part', self.options,
                               self.project_options)

        os.makedirs(plugin.sourcedir)
        plugin.build()

        run_mock.assert_has_calls([
            mock.call(['qbs', 'setup-toolchains', '--detect'], env=mock.ANY),
            mock.call(['qbs', 'build', '-d', plugin.builddir,
                       '-f', plugin.sourcedir, 'release',
                       'qbs.installRoot:' + plugin.installdir,
                       'profile:clang'], env=mock.ANY)
        ])

    @mock.patch.object(qbs.QbsPlugin, 'run')
    def test_build_qt(self, run_mock):
        self.options.qt_version = 'qt5'
        plugin = qbs.QbsPlugin('test-part', self.options,
                               self.project_options)

        os.makedirs(plugin.sourcedir)

        # Check to see if there is a qmake on path
        # or create one if there isn't
        qmake_path = self._find_or_create_qmake(plugin)
        self.assertIsNotNone(qmake_path)

        plugin.build()

        run_mock.assert_has_calls([
            mock.call(['qbs', 'setup-toolchains', '--detect'], env=mock.ANY),
            mock.call(['qbs', 'setup-qt', qmake_path,
                       'snapcraft-qbs-qt5-clang'], env=mock.ANY),
            mock.call(['qbs', 'config',
                       'profiles.snapcraft-qbs-qt5-clang.baseProfile',
                       self.options.qbs_profile], env=mock.ANY),
            mock.call(['qbs', 'build', '-d', plugin.builddir,
                       '-f', plugin.sourcedir, 'release',
                       'qbs.installRoot:' + plugin.installdir,
                       'profile:snapcraft-qbs-qt5-clang'], env=mock.ANY),
        ])

    @mock.patch.object(qbs.QbsPlugin, 'run')
    def test_build_with_options(self, run_mock):
        self.options.qt_version = None
        self.options.qbs_options = ['project.someBoolProperty:true']
        plugin = qbs.QbsPlugin('test-part', self.options,
                               self.project_options)

        os.makedirs(plugin.sourcedir)
        plugin.build()

        run_mock.assert_has_calls([
            mock.call(['qbs', 'setup-toolchains', '--detect'], env=mock.ANY),
            mock.call(['qbs', 'build', '-d', plugin.builddir,
                       '-f', plugin.sourcedir, 'release',
                       'qbs.installRoot:' + plugin.installdir,
                       'profile:clang', 'project.someBoolProperty:true'],
                      env=mock.ANY),
        ])

    def _find_or_create_qmake(self, plugin):
        qmake_path = shutil.which('qmake')
        if qmake_path is None:
            tmp_path = os.pathsep + plugin.sourcedir
            # Create PATH if it isn't set. This seems to
            # occur in the travis docker containers.
            if 'PATH' not in os.environ.keys():
                os.environ.setdefault('PATH', os.defpath + tmp_path)
            else:
                os.environ['PATH'] += tmp_path

            qmake_path = os.path.join(plugin.sourcedir, 'qmake')
            open(qmake_path, 'w').close()
            os.chmod(qmake_path, 0o755)
        return qmake_path
