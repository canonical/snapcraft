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

from snapcraft.plugins import go
from snapcraft import common
from snapcraft import tests


class verify_only_gopath():
    def __init__(self, test, gopath):
        self.test = test
        self.gopath = gopath

    def __eq__(self, env):
        self.test.assertEqual(env['GOPATH'], self.gopath)

        for variable in ['GOARCH', 'GOBIN', 'GOCHAR', 'GOEXE', 'GOHOSTARCH',
                         'GOHOSTOS', 'GOOS', 'GORACE', 'GOROOT', 'GOTOOLDIR',
                         'TERM', 'CC', 'GOGCCFLAGS', 'CXX', 'CGO_ENABLED']:
            self.test.assertTrue(
                variable not in env,
                'Expected "{}" to not be defined'.format(variable))

        return True


class GoPluginTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        patcher = mock.patch('subprocess.check_call')
        self.check_call_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('sys.stdout')
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_environment(self):
        class Options:
            source = 'http://github.com/testplug'
            go_packages = []

        plugin = go.GoPlugin('test', Options())
        self.assertEqual(plugin.env('myroot'), [
            'GOPATH=myroot/go',
            'CGO_LDFLAGS="$CGO_LDFLAGS -Lmyroot/lib -Lmyroot/usr/lib '
            '-Lmyroot/lib/{0} '
            '-Lmyroot/usr/lib/{0} $LDFLAGS"'.format(
                common.get_arch_triplet())])

    def test_pull_local_sources(self):
        class Options:
            source = 'dir'
            go_packages = []

        plugin = go.GoPlugin('test-part', Options())

        os.makedirs(plugin.options.source)
        os.makedirs(plugin.sourcedir)

        plugin.pull()

        self.check_call_mock.assert_called_once_with(
            ['go', 'get', '-t', '-d', './dir/...'], cwd=plugin._gopath_src,
            env=verify_only_gopath(self, plugin._gopath))

        self.assertTrue(os.path.exists(plugin._gopath))
        self.assertTrue(os.path.exists(plugin._gopath_src))
        self.assertFalse(os.path.exists(plugin._gopath_bin))

    def test_pull_remote_sources(self):
        class Options:
            source = None
            go_packages = ['github.com/gotools/vet']

        plugin = go.GoPlugin('test-part', Options())

        os.makedirs(plugin.sourcedir)

        plugin.pull()

        self.check_call_mock.assert_called_once_with(
            ['go', 'get', '-t', '-d', plugin.options.go_packages[0]],
            cwd=plugin._gopath_src, env=verify_only_gopath(self,
                                                           plugin._gopath))

        self.assertTrue(os.path.exists(plugin._gopath))
        self.assertTrue(os.path.exists(plugin._gopath_src))
        self.assertFalse(os.path.exists(plugin._gopath_bin))

    def test_pull_with_no_local_or_remote_sources(self):
        class Options:
            source = None
            go_packages = []

        plugin = go.GoPlugin('test-part', Options())
        plugin.pull()

        self.check_call_mock.assert_has_calls([])

        self.assertTrue(os.path.exists(plugin._gopath))
        self.assertTrue(os.path.exists(plugin._gopath_src))
        self.assertFalse(os.path.exists(plugin._gopath_bin))

    def test_build_with_local_sources(self):
        class Options:
            source = 'dir'
            go_packages = []

        plugin = go.GoPlugin('test-part', Options())

        os.makedirs(plugin.options.source)
        os.makedirs(plugin.sourcedir)

        plugin.pull()

        os.makedirs(plugin._gopath_bin)
        os.makedirs(plugin.builddir)

        plugin.build()

        self.check_call_mock.assert_has_calls([
            mock.call(['go', 'get', '-t', '-d', './dir/...'],
                      cwd=plugin._gopath_src,
                      env=verify_only_gopath(self, plugin._gopath)),
            mock.call(['go', 'install', './dir/...'],
                      cwd=plugin._gopath_src,
                      env=verify_only_gopath(self, plugin._gopath)),
        ])

        self.assertTrue(os.path.exists(plugin._gopath))
        self.assertTrue(os.path.exists(plugin._gopath_src))
        self.assertTrue(os.path.exists(plugin._gopath_bin))

    def test_build_with_remote_sources(self):
        class Options:
            source = None
            go_packages = ['github.com/gotools/vet']

        plugin = go.GoPlugin('test-part', Options())

        os.makedirs(plugin.sourcedir)

        plugin.pull()

        os.makedirs(plugin._gopath_bin)
        os.makedirs(plugin.builddir)
        # fake some binaries
        open(os.path.join(plugin._gopath_bin, 'vet'), 'w').close()

        plugin.build()

        self.check_call_mock.assert_has_calls([
            mock.call(['go', 'get', '-t', '-d', plugin.options.go_packages[0]],
                      cwd=plugin._gopath_src,
                      env=verify_only_gopath(self, plugin._gopath)),
            mock.call(['go', 'install', plugin.options.go_packages[0]],
                      cwd=plugin._gopath_src,
                      env=verify_only_gopath(self, plugin._gopath)),
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

        plugin = go.GoPlugin('test-part', Options())

        os.makedirs(plugin.sourcedir)

        plugin.pull()

        os.makedirs(plugin._gopath_bin)
        os.makedirs(plugin.builddir)

        plugin.build()

        self.check_call_mock.assert_has_calls([])

        self.assertTrue(os.path.exists(plugin._gopath))
        self.assertTrue(os.path.exists(plugin._gopath_src))
        self.assertTrue(os.path.exists(plugin._gopath_bin))
