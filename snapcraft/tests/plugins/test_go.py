# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2016 Canonical Ltd
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
from snapcraft.plugins import go
from snapcraft import tests


class GoPluginTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        self.project_options = snapcraft.ProjectOptions()

        patcher = mock.patch('snapcraft.internal.common.run')
        self.run_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('sys.stdout')
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_schema(self):
        schema = go.GoPlugin.schema()

        properties = schema['properties']
        for expected in ['go-packages', 'go-importpath', 'go-buildtags']:
            self.assertTrue(
                expected in properties,
                'Expected {!r} to be included in properties'.format(
                    expected))

        # Check go-packages
        go_packages = properties['go-packages']
        for expected in [
                'type', 'default', 'minitems', 'uniqueItems', 'items']:
            self.assertTrue(
                expected in go_packages,
                "Expected {!r} to be included in 'go-packages'".format(
                    expected))

        go_packages_type = go_packages['type']
        self.assertEqual(go_packages_type, 'array',
                         'Expected "go-packages" "type" to be "array", but '
                         'it was "{}"'.format(go_packages_type))

        go_packages_default = go_packages['default']
        self.assertEqual(go_packages_default, [],
                         'Expected "go-packages" "default" to be '
                         '"d[]", but it was "{}"'.format(
                             go_packages_default))

        go_packages_minitems = go_packages['minitems']
        self.assertEqual(go_packages_minitems, 1,
                         'Expected "go-packages" "minitems" to be 1, but '
                         'it was {}'.format(go_packages_minitems))

        self.assertTrue(go_packages['uniqueItems'])

        go_packages_items = go_packages['items']
        self.assertTrue('type' in go_packages_items,
                        'Expected "type" to be included in "go-packages" '
                        '"items"')

        go_packages_items_type = go_packages_items['type']
        self.assertEqual(go_packages_items_type, 'string',
                         'Expected "go-packages" "item" "type" to be '
                         '"string", but it was "{}"'
                         .format(go_packages_items_type))

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

        go_importpath_default = go_importpath['default']
        self.assertEqual(go_importpath_default, '',
                         'Expected "go-default" "default" to be "''", but '
                         'it was "{}"'.format(go_importpath_default))

        # Check go-buildtags
        go_buildtags = properties['go-buildtags']
        for expected in [
                'type', 'default', 'minitems', 'uniqueItems', 'items']:
            self.assertTrue(
                expected in go_buildtags,
                "Expected {!r} to be included in 'go-buildtags'".format(
                    expected))

        go_buildtags_type = go_buildtags['type']
        self.assertEqual(go_buildtags_type, 'array',
                         'Expected "go-buildtags" "type" to be "array", but '
                         'it was "{}"'.format(go_buildtags_type))

        go_buildtags_default = go_buildtags['default']
        self.assertEqual(go_buildtags_default, [],
                         'Expected "go-buildtags" "default" to be "[]", but '
                         'it was "{}"'.format(go_buildtags_type))

        go_buildtags_minitems = go_buildtags['minitems']
        self.assertEqual(go_buildtags_minitems, 1,
                         'Expected "go-buildtags" "minitems" to be 1, but '
                         'it was {}'.format(go_buildtags_minitems))

        self.assertTrue(go_buildtags['uniqueItems'])

        go_buildtags_items = go_buildtags['items']
        self.assertTrue('type' in go_buildtags_items,
                        'Expected "type" to be included in "go-buildtags" '
                        '"items"')

        go_buildtags_items_type = go_buildtags_items['type']
        self.assertEqual(go_buildtags_items_type, 'string',
                         'Expected "go-buildtags" "item" "type" to be '
                         '"string", but it was "{}"'
                         .format(go_packages_items_type))

        # Check required properties
        self.assertNotIn('required', schema)

    def test_get_pull_properties(self):
        expected_pull_properties = ['go-packages']
        resulting_pull_properties = go.GoPlugin.get_pull_properties()

        self.assertThat(resulting_pull_properties,
                        HasLength(len(expected_pull_properties)))

        for property in expected_pull_properties:
            self.assertIn(property, resulting_pull_properties)

    def test_get_build_properties(self):
        expected_build_properties = ['go-packages', 'go-buildtags']
        resulting_build_properties = go.GoPlugin.get_build_properties()

        self.assertThat(resulting_build_properties,
                        HasLength(len(expected_build_properties)))

        for property in expected_build_properties:
            self.assertIn(property, resulting_build_properties)

    def test_pull_local_sources(self):
        class Options:
            source = 'dir'
            go_packages = []
            go_importpath = ''

        plugin = go.GoPlugin('test-part', Options(), self.project_options)

        os.makedirs(plugin.sourcedir)
        open(os.path.join(plugin.sourcedir, 'main.go'), 'w').close()

        plugin.pull()

        self.run_mock.assert_has_calls([
            mock.call(['go', 'get', '-t', '-d', './dir/...'],
                      cwd=plugin._gopath_src, env=mock.ANY)])

        self.assertTrue(os.path.exists(plugin._gopath))
        self.assertTrue(os.path.exists(plugin._gopath_src))
        self.assertFalse(os.path.exists(plugin._gopath_bin))

    def test_no_local_source_with_go_packages(self):
        class Options:
            source = None
            go_packages = ['github.com/gotools/vet']
            go_importpath = ''

        plugin = go.GoPlugin('test-part', Options(), self.project_options)

        os.makedirs(plugin.sourcedir)

        plugin.pull()

        self.run_mock.assert_has_calls([
            mock.call(['go', 'get', '-t', '-d', plugin.options.go_packages[0]],
                      env=mock.ANY, cwd=plugin._gopath_src)])

        self.assertTrue(os.path.exists(plugin._gopath))
        self.assertTrue(os.path.exists(plugin._gopath_src))
        self.assertFalse(os.path.exists(plugin._gopath_bin))

    def test_pull_with_local_sources_or_go_packages(self):
        class Options:
            source = None
            go_packages = []
            go_importpath = ''

        plugin = go.GoPlugin('test-part', Options(), self.project_options)
        plugin.pull()

        self.run_mock.assert_has_calls([])

        self.assertTrue(os.path.exists(plugin._gopath))
        self.assertTrue(os.path.exists(plugin._gopath_src))
        self.assertFalse(os.path.exists(plugin._gopath_bin))

    def test_build_with_local_sources(self):
        class Options:
            source = 'dir'
            go_packages = []
            go_importpath = ''
            go_buildtags = ''

        plugin = go.GoPlugin('test-part', Options(), self.project_options)

        os.makedirs(plugin.sourcedir)
        open(os.path.join(plugin.sourcedir, 'main.go'), 'w').close()

        plugin.pull()

        os.makedirs(plugin._gopath_bin)
        os.makedirs(plugin.builddir)

        self.run_mock.reset_mock()
        plugin.build()

        self.run_mock.assert_called_once_with(
            ['go', 'install', './dir/...'],
            cwd=plugin._gopath_src, env=mock.ANY)

        self.assertTrue(os.path.exists(plugin._gopath))
        self.assertTrue(os.path.exists(plugin._gopath_src))
        self.assertTrue(os.path.exists(plugin._gopath_bin))

    def test_build_go_packages(self):
        class Options:
            source = ''
            go_packages = ['github.com/gotools/vet']
            go_importpath = ''
            go_buildtags = ''

        plugin = go.GoPlugin('test-part', Options(), self.project_options)

        os.makedirs(plugin.sourcedir)

        plugin.pull()

        os.makedirs(plugin._gopath_bin)
        os.makedirs(plugin.builddir)
        # fake some binaries
        open(os.path.join(plugin._gopath_bin, 'vet'), 'w').close()

        self.run_mock.reset_mock()
        plugin.build()

        self.run_mock.assert_called_once_with(
            ['go', 'install', plugin.options.go_packages[0]],
            cwd=plugin._gopath_src, env=mock.ANY)

        self.assertTrue(os.path.exists(plugin._gopath))
        self.assertTrue(os.path.exists(plugin._gopath_src))
        self.assertTrue(os.path.exists(plugin._gopath_bin))
        vet_binary = os.path.join(plugin.installdir, 'bin', 'vet')
        self.assertTrue(os.path.exists(vet_binary))

    def test_build_with_no_local_sources_or_go_packages(self):
        class Options:
            source = ''
            go_packages = []
            go_importpath = ''
            go_buildtags = ''

        plugin = go.GoPlugin('test-part', Options(), self.project_options)

        os.makedirs(plugin.sourcedir)

        plugin.pull()

        os.makedirs(plugin._gopath_bin)
        os.makedirs(plugin.builddir)

        plugin.build()

        self.run_mock.assert_has_calls([])

        self.assertTrue(os.path.exists(plugin._gopath))
        self.assertTrue(os.path.exists(plugin._gopath_src))
        self.assertTrue(os.path.exists(plugin._gopath_bin))

    def test_clean_build(self):
        class Options:
            source = 'dir'
            go_packages = []
            go_importpath = ''
            go_buildtags = ''

        plugin = go.GoPlugin('test-part', Options(), self.project_options)

        plugin.pull()

        os.makedirs(plugin._gopath_bin)
        os.makedirs(plugin._gopath_pkg)
        os.makedirs(plugin.builddir)

        plugin.build()

        self.assertTrue(os.path.exists(plugin._gopath))
        self.assertTrue(os.path.exists(plugin._gopath_src))
        self.assertTrue(os.path.exists(plugin._gopath_bin))

        plugin.clean_build()

        self.assertTrue(os.path.exists(plugin._gopath))
        self.assertTrue(os.path.exists(plugin._gopath_src))
        self.assertFalse(os.path.exists(plugin._gopath_bin))
        self.assertFalse(os.path.exists(plugin._gopath_pkg))

    def test_clean_pull(self):
        class Options:
            source = 'dir'
            go_packages = []
            go_importpath = ''
            go_buildtags = ''

        plugin = go.GoPlugin('test-part', Options(), self.project_options)

        os.makedirs(plugin.sourcedir)
        open(os.path.join(plugin.sourcedir, 'main.go'), 'w').close()

        plugin.pull()

        self.assertTrue(os.path.exists(plugin._gopath))

        plugin.clean_pull()

        self.assertFalse(os.path.exists(plugin._gopath))

    def test_build_with_local_sources_and_go_importpath(self):
        class Options:
            source = 'dir'
            go_packages = []
            go_importpath = 'github.com/snapcore/launcher'
            go_buildtags = ''

        plugin = go.GoPlugin('test-part', Options(), self.project_options)

        os.makedirs(plugin.sourcedir)
        open(os.path.join(plugin.sourcedir, 'main.go'), 'w').close()

        plugin.pull()

        os.makedirs(plugin._gopath_bin)
        os.makedirs(plugin.builddir)

        plugin.build()

        self.run_mock.assert_has_calls([
            mock.call(['go', 'get', '-t', '-d',
                       './github.com/snapcore/launcher/...'],
                      cwd=plugin._gopath_src, env=mock.ANY),
            mock.call(['go', 'install', './github.com/snapcore/launcher/...'],
                      cwd=plugin._gopath_src, env=mock.ANY),
        ])

        self.assertTrue(os.path.exists(
            os.path.join(plugin._gopath_src, plugin.options.go_importpath)))

    def test_build_environment(self):
        class Options:
            source = 'dir'
            go_packages = []
            go_importpath = ''
            go_buildtags = ''

        plugin = go.GoPlugin('test-part', Options(), self.project_options)

        os.makedirs(plugin.sourcedir)
        open(os.path.join(plugin.sourcedir, 'main.go'), 'w').close()
        os.makedirs(os.path.join(plugin.installdir, 'lib'))
        os.makedirs(os.path.join(plugin.installdir, 'usr', 'lib'))
        os.makedirs(os.path.join(plugin.project.stage_dir, 'lib'))
        os.makedirs(os.path.join(plugin.project.stage_dir, 'usr', 'lib'))
        plugin.pull()

        self.assertEqual(1, self.run_mock.call_count)
        for call_args in self.run_mock.call_args_list:
            env = call_args[1]['env']
            self.assertTrue(
                'GOPATH' in env, 'Expected environment to include GOPATH')
            self.assertEqual(env['GOPATH'], plugin._gopath)

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

    def test_build_with_buildtag(self):
        class Options:
            source = 'dir'
            go_importpath = ''
            go_packages = []
            go_buildtags = ['testbuildtag1', 'testbuildtag2']

        plugin = go.GoPlugin('test-part', Options(), self.project_options)

        os.makedirs(plugin.options.source)
        os.makedirs(plugin.sourcedir)

        plugin.pull()

        os.makedirs(plugin._gopath_bin)
        os.makedirs(plugin.builddir)

        self.run_mock.reset_mock()
        plugin.build()

        self.run_mock.assert_called_once_with(
            ['go', 'install',
             '-tags=testbuildtag1,testbuildtag2', './dir/...'],
            cwd=plugin._gopath_src, env=mock.ANY)
