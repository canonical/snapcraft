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

    def test_pull_local_sources(self):
        class Options:
            source = 'dir'
            go_packages = []
            go_importpath = ''

        plugin = go.GoPlugin('test-part', Options(), self.project_options)

        os.makedirs(plugin.options.source)
        os.makedirs(plugin.sourcedir)

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

        plugin = go.GoPlugin('test-part', Options(), self.project_options)

        os.makedirs(plugin.options.source)
        os.makedirs(plugin.sourcedir)

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

        plugin = go.GoPlugin('test-part', Options(), self.project_options)

        os.makedirs(plugin.options.source)
        os.makedirs(plugin.sourcedir)

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

        plugin = go.GoPlugin('test-part', Options(), self.project_options)

        os.makedirs(plugin.options.source)
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
