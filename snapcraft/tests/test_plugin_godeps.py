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
from snapcraft.plugins import godeps
from snapcraft import tests


class GodepsPluginTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        self.project_options = snapcraft.ProjectOptions()

        patcher = mock.patch('snapcraft.internal.common.run')
        self.run_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('sys.stdout')
        patcher.start()
        self.addCleanup(patcher.stop)

        class Options:
            source = 'src'
            go_importpath = 'github.com/foo/bar'
            godeps_file = 'dependencies.tsv'

        self.options = Options()

    def test_schema(self):
        schema = godeps.GodepsPlugin.schema()

        properties = schema['properties']
        for expected in ['godeps-file', 'go-importpath']:
            self.assertTrue(
                expected in properties,
                'Expected {!r} to be included in properties'.format(
                    expected))

        # Check godeps-file
        godeps_file = properties['godeps-file']
        for expected in ['type', 'default']:
            self.assertTrue(
                expected in godeps_file,
                "Expected {!r} to be included in 'godeps-file'".format(
                    expected))

        godeps_file_type = godeps_file['type']
        self.assertEqual(godeps_file_type, 'string',
                         'Expected "godeps-file" "type" to be "string", but '
                         'it was "{}"'.format(godeps_file_type))

        godeps_file_default = godeps_file['default']
        self.assertEqual(godeps_file_default, 'dependencies.tsv',
                         'Expected "godeps-file" "default" to be '
                         '"dependencies.tsv", but it was "{}"'.format(
                             godeps_file_default))

        # Check go-importpath
        go_importpath = properties['go-importpath']
        for expected in ['type']:
            self.assertTrue(
                expected in go_importpath,
                "Expected {!r} to be included in 'go-importpath'".format(
                    expected))

        go_importpath_type = go_importpath['type']
        self.assertEqual(go_importpath_type, 'string',
                         'Expected "go-importpath" "type" to be "string", but '
                         'it was "{}"'.format(go_importpath_type))

        # go-importpath should be required
        self.assertTrue('go-importpath' in schema['required'],
                        'Expeced "go-importpath" to be required')

        # Verify both are pull dependencies
        self.assertTrue('godeps-file' in schema['pull-properties'])
        self.assertTrue('go-importpath' in schema['pull-properties'])

    def test_build_environment(self):
        plugin = godeps.GodepsPlugin(
            'test-part', self.options, self.project_options)

        os.makedirs(plugin.options.source)
        os.makedirs(os.path.join(plugin.installdir, 'lib'))
        os.makedirs(os.path.join(plugin.installdir, 'usr', 'lib'))
        os.makedirs(os.path.join(plugin.project.stage_dir, 'lib'))
        os.makedirs(os.path.join(plugin.project.stage_dir, 'usr', 'lib'))
        plugin.pull()

        self.assertEqual(2, self.run_mock.call_count)
        for call_args in self.run_mock.call_args_list:
            env = call_args[1]['env']
            self.assertTrue(
                'GOPATH' in env, 'Expected environment to include GOPATH')
            self.assertEqual(env['GOPATH'], plugin._gopath)

            self.assertTrue(
                'PATH' in env, 'Expected environment to include PATH')
            self.assertTrue(
                os.path.join(plugin._gopath, 'bin') in env['PATH'],
                'Expected $PATH to include $GOPATH/bin')

            self.assertTrue(
                'CGO_LDFLAGS' in env,
                'Expected environment to include CGO_LDFLAGS')
            expected_flags = [
                '-L{}/lib'.format(plugin.installdir),
                '-L{}/usr/lib'.format(plugin.installdir),
                '-L{}/lib'.format(plugin.project.stage_dir),
                '-L{}/usr/lib'.format(plugin.project.stage_dir),
            ]
            for flag in expected_flags:
                self.assertTrue(
                    flag in env['CGO_LDFLAGS'],
                    'Expected $CGO_LDFLAGS to include {!r}, but it was '
                    '"{}"'.format(flag, env['CGO_LDFLAGS']))

    def test_pull(self):
        plugin = godeps.GodepsPlugin(
            'test-part', self.options, self.project_options)

        os.makedirs(plugin.options.source)

        plugin.pull()

        self.assertEqual(2, self.run_mock.call_count)
        self.run_mock.assert_has_calls([
            mock.call(['go', 'get', 'github.com/rogpeppe/godeps'],
                      cwd=plugin._gopath_src, env=mock.ANY),
            mock.call(['godeps', '-t', '-u', os.path.join(
                plugin.sourcedir, self.options.godeps_file)],
                cwd=plugin._gopath_src, env=mock.ANY),
        ])

        self.assertTrue(os.path.exists(plugin._gopath))
        self.assertTrue(os.path.exists(plugin._gopath_src))
        self.assertFalse(os.path.exists(plugin._gopath_pkg))
        self.assertFalse(os.path.exists(plugin._gopath_bin))

        sourcedir = os.path.join(
            plugin._gopath_src, 'github.com', 'foo', 'bar')
        self.assertTrue(os.path.islink(sourcedir))
        self.assertEqual(plugin.sourcedir, os.readlink(sourcedir))

    def test_clean_pull(self):
        plugin = godeps.GodepsPlugin(
            'test-part', self.options, self.project_options)

        os.makedirs(plugin.options.source)

        plugin.pull()

        self.assertTrue(os.path.exists(plugin.sourcedir))
        self.assertTrue(os.path.exists(plugin._gopath))

        plugin.clean_pull()

        self.assertFalse(os.path.exists(plugin.sourcedir))
        self.assertFalse(os.path.exists(plugin._gopath))

    def test_build(self):
        plugin = godeps.GodepsPlugin(
            'test-part', self.options, self.project_options)

        os.makedirs(plugin.options.source)

        plugin.pull()

        os.makedirs(plugin._gopath_bin)
        os.makedirs(plugin.builddir)

        self.run_mock.reset_mock()

        open(os.path.join(plugin._gopath_bin, 'test-binary'), 'w').close()
        open(os.path.join(plugin._gopath_bin, 'godeps'), 'w').close()
        plugin.build()

        self.assertEqual(1, self.run_mock.call_count)
        self.run_mock.assert_has_calls([
            mock.call(
                ['go', 'install', './github.com/foo/bar/...'],
                cwd=plugin._gopath_src, env=mock.ANY),
        ])

        self.assertTrue(os.path.exists(plugin._gopath))
        self.assertTrue(os.path.exists(plugin._gopath_src))
        self.assertTrue(os.path.exists(plugin._gopath_bin))
        self.assertTrue(os.path.exists(
            os.path.join(plugin.installdir, 'bin', 'test-binary')))

        # Assert that the godeps binary was NOT copied
        self.assertFalse(os.path.exists(
            os.path.join(plugin.installdir, 'bin', 'godeps')))

    def test_clean_build(self):
        plugin = godeps.GodepsPlugin(
            'test-part', self.options, self.project_options)

        os.makedirs(plugin.options.source)

        plugin.pull()

        os.makedirs(plugin._gopath_bin)
        os.makedirs(plugin._gopath_pkg)
        os.makedirs(plugin.builddir)

        plugin.build()

        self.assertTrue(os.path.exists(plugin._gopath))
        self.assertTrue(os.path.exists(plugin._gopath_src))
        self.assertTrue(os.path.exists(plugin._gopath_pkg))
        self.assertTrue(os.path.exists(plugin._gopath_bin))

        plugin.clean_build()

        self.assertTrue(os.path.exists(plugin._gopath))
        self.assertTrue(os.path.exists(plugin._gopath_src))
        self.assertFalse(os.path.exists(plugin._gopath_pkg))
        self.assertFalse(os.path.exists(plugin._gopath_bin))
