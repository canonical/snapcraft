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
        self.assertTrue('godeps' in properties,
                        'Expected "godeps" to be included in properties')

        godeps = properties['godeps']
        self.assertTrue('type' in godeps,
                        'Expected "type" to be included in "godeps"')
        self.assertTrue('default' in godeps,
                        'Expected "default" to be included in "godeps"')

        godeps_type = godeps['type']
        self.assertEqual(godeps_type, 'string',
                         'Expected "godeps" "type" to be "string", but it '
                         'was "{}"'.format(godeps_type))

        godeps_type = godeps['default']
        self.assertEqual(godeps_type, '',
                         'Expected "godeps" "default" to be "", but it '
                         'was "{}"'.format(godeps_type))

    def test_environment(self):
        class Options:
            source = 'http://github.com/testplug'
            go_packages = []
            go_importpath = ''
            godeps = ''

        plugin = go.GoPlugin('test', Options(), self.project_options)
        self.assertEqual(plugin.env('myroot'), [
            'GOPATH=myroot/go',
            'CGO_LDFLAGS="$CGO_LDFLAGS -Lmyroot/lib -Lmyroot/usr/lib '
            '-Lmyroot/lib/{0} '
            '-Lmyroot/usr/lib/{0} $LDFLAGS"'.format(
                plugin.project.arch_triplet)])

    def test_pull_local_sources(self):
        class Options:
            source = 'dir'
            go_packages = []
            go_importpath = ''
            godeps = ''

        plugin = go.GoPlugin('test-part', Options(), self.project_options)

        os.makedirs(plugin.options.source)
        os.makedirs(plugin.sourcedir)

        plugin.pull()

        self.run_mock.assert_has_calls([
            mock.call(['env', 'GOPATH={}'.format(plugin._gopath),
                       'go', 'get', '-t', '-d', './dir/...'],
                      cwd=plugin._gopath_src)])

        self.assertTrue(os.path.exists(plugin._gopath))
        self.assertTrue(os.path.exists(plugin._gopath_src))
        self.assertFalse(os.path.exists(plugin._gopath_bin))

    def test_pull_remote_sources(self):
        class Options:
            source = None
            go_packages = ['github.com/gotools/vet']
            go_importpath = ''
            godeps = ''

        plugin = go.GoPlugin('test-part', Options(), self.project_options)

        os.makedirs(plugin.sourcedir)

        plugin.pull()

        self.run_mock.assert_has_calls([
            mock.call(['env', 'GOPATH={}'.format(plugin._gopath),
                       'go', 'get', '-t', '-d', plugin.options.go_packages[0]],
                      cwd=plugin._gopath_src)])

        self.assertTrue(os.path.exists(plugin._gopath))
        self.assertTrue(os.path.exists(plugin._gopath_src))
        self.assertFalse(os.path.exists(plugin._gopath_bin))

    def test_pull_godeps_dependencies(self):
        class Options:
            source = None
            go_packages = []
            go_importpath = ''
            godeps = 'dependencies.tsv'

        plugin = go.GoPlugin('test-part', Options(), self.project_options)

        os.makedirs(plugin.sourcedir)

        plugin.pull()

        self.run_mock.assert_has_calls([
            mock.call(['env', 'GOPATH={}'.format(plugin._gopath),
                       'go', 'get', 'launchpad.net/godeps'],
                      cwd=plugin._gopath_src),
            mock.call(['env', 'GOPATH={}'.format(plugin._gopath), 'godeps',
                       '-u', '{}/dependencies.tsv'.format(plugin.sourcedir)],
                      cwd=plugin._gopath_src),
        ])

        self.assertTrue(os.path.exists(plugin._gopath))
        self.assertTrue(os.path.exists(plugin._gopath_src))
        self.assertFalse(os.path.exists(plugin._gopath_bin))

    def test_pull_with_no_local_or_remote_sources(self):
        class Options:
            source = None
            go_packages = []
            go_importpath = ''
            godeps = ''

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
            godeps = ''

        plugin = go.GoPlugin('test-part', Options(), self.project_options)

        os.makedirs(plugin.options.source)
        os.makedirs(plugin.sourcedir)

        plugin.pull()

        os.makedirs(plugin._gopath_bin)
        os.makedirs(plugin.builddir)

        plugin.build()

        self.run_mock.assert_has_calls([
            mock.call(['env', 'GOPATH={}'.format(plugin._gopath),
                       'go', 'get', '-t', '-d', './dir/...'],
                      cwd=plugin._gopath_src),
            mock.call(['env', 'GOPATH={}'.format(plugin._gopath),
                       'go', 'install', './dir/...'],
                      cwd=plugin._gopath_src),
        ])

        self.assertTrue(os.path.exists(plugin._gopath))
        self.assertTrue(os.path.exists(plugin._gopath_src))
        self.assertTrue(os.path.exists(plugin._gopath_bin))

    def test_build_with_remote_sources(self):
        class Options:
            source = None
            go_packages = ['github.com/gotools/vet']
            go_importpath = ''
            godeps = ''

        plugin = go.GoPlugin('test-part', Options(), self.project_options)

        os.makedirs(plugin.sourcedir)

        plugin.pull()

        os.makedirs(plugin._gopath_bin)
        os.makedirs(plugin.builddir)
        # fake some binaries
        open(os.path.join(plugin._gopath_bin, 'vet'), 'w').close()

        plugin.build()

        self.run_mock.assert_has_calls([
            mock.call(['env', 'GOPATH={}'.format(plugin._gopath),
                       'go', 'get', '-t', '-d', plugin.options.go_packages[0]],
                      cwd=plugin._gopath_src),
            mock.call(['env', 'GOPATH={}'.format(plugin._gopath),
                       'go', 'install', plugin.options.go_packages[0]],
                      cwd=plugin._gopath_src),
        ])

        self.assertTrue(os.path.exists(plugin._gopath))
        self.assertTrue(os.path.exists(plugin._gopath_src))
        self.assertTrue(os.path.exists(plugin._gopath_bin))
        vet_binary = os.path.join(plugin.installdir, 'bin', 'vet')
        self.assertTrue(os.path.exists(vet_binary))

    def test_build_with_no_local_or_remote_sources(self):
        class Options:
            source = None
            go_packages = []
            go_importpath = ''
            godeps = ''

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
            godeps = ''

        plugin = go.GoPlugin('test-part', Options(), self.project_options)

        os.makedirs(plugin.options.source)
        os.makedirs(plugin.sourcedir)

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
            godeps = ''

        plugin = go.GoPlugin('test-part', Options(), self.project_options)

        os.makedirs(plugin.options.source)
        os.makedirs(plugin.sourcedir)

        plugin.pull()

        self.assertTrue(os.path.exists(plugin._gopath))

        plugin.clean_pull()

        self.assertFalse(os.path.exists(plugin._gopath))

    def test_build_with_local_sources_and_go_importpath(self):
        class Options:
            source = 'dir'
            go_packages = []
            go_importpath = 'github.com/snapcore/launcher'
            godeps = ''

        plugin = go.GoPlugin('test-part', Options(), self.project_options)

        os.makedirs(plugin.options.source)
        os.makedirs(plugin.sourcedir)

        plugin.pull()

        os.makedirs(plugin._gopath_bin)
        os.makedirs(plugin.builddir)

        plugin.build()

        self.run_mock.assert_has_calls([
            mock.call(['env', 'GOPATH={}'.format(plugin._gopath),
                       'go', 'get', '-t', '-d',
                       './github.com/snapcore/launcher/...'],
                      cwd=plugin._gopath_src),
            mock.call(['env', 'GOPATH={}'.format(plugin._gopath),
                       'go', 'install', './github.com/snapcore/launcher/...'],
                      cwd=plugin._gopath_src),
        ])

        self.assertTrue(os.path.exists(
            os.path.join(plugin._gopath_src, plugin.options.go_importpath)))
