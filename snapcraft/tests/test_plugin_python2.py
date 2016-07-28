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
from snapcraft import tests
from snapcraft.plugins import python2


class Python2PluginTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        class Options:
            requirements = ''
            python_packages = []

        self.options = Options()
        self.project_options = snapcraft.ProjectOptions()

    @mock.patch.object(python2.Python2Plugin, 'run')
    @mock.patch.object(python2.Python2Plugin, 'run_output',
                       return_value='python2.7')
    def test_pip_relative_site_packages_symlink(self, run_output_mock,
                                                run_mock):
        plugin = python2.Python2Plugin('test-part', self.options,
                                       self.project_options)
        os.makedirs(plugin.sourcedir)
        os.makedirs(os.path.join(
            plugin.installdir, 'usr', 'lib', 'python2.7', 'dist-packages'))
        os.makedirs(os.path.join(
            plugin.installdir, 'usr', 'include', 'python2.7'))

        open(os.path.join(plugin.sourcedir, 'setup.py'), 'w').close()

        plugin._pip()

        link = os.readlink(os.path.join(plugin.installdir, 'usr', 'lib',
                                        'python2.7', 'site-packages'))
        self.assertEqual(link, 'dist-packages',
                         'Expected site-packages to be a relative link to '
                         '"dist-packages", but it was a link to "{}"'
                         .format(link))

    def test_get_python2_include_missing_raises_exception(self):
        with self.assertRaises(EnvironmentError) as raised:
            python2._get_python2_include('/foo')
        self.assertEqual(str(raised.exception),
                         'python development headers not installed')

    def test_fileset_ignores(self):
        plugin = python2.Python2Plugin('test-part', self.options,
                                       self.project_options)
        expected_fileset = [
            '-usr/bin/pip*',
            '-usr/lib/python*/dist-packages/easy-install.pth',
            '-usr/lib/python*/dist-packages/__pycache__/*.pyc',
            '-usr/lib/python*/*.pyc',
            '-usr/lib/python*/*/*.pyc',
            '-usr/lib/python*/*/*/*.pyc',
            '-usr/lib/python*/*/*/*/*.pyc',
            '-usr/lib/python*/*/*/*/*/*.pyc',
            '-usr/lib/python*/*/*/*/*/*/*.pyc',
            '-usr/lib/python*/*/*/*/*/*/*/*.pyc',
            '-usr/lib/python*/*/*/*/*/*/*/*/*.pyc',
            '-usr/lib/python*/*/*/*/*/*/*/*/*/*.pyc',
            '-usr/lib/python*/*/*/*/*/*/*/*/*/*/*.pyc',
        ]
        fileset = plugin.snap_fileset()
        self.assertListEqual(expected_fileset, fileset)

    @mock.patch.object(python2.Python2Plugin, 'run')
    def test_build_fixes_python_shebangs(self, run_mock):
        plugin = python2.Python2Plugin('test-part', self.options,
                                       self.project_options)
        os.makedirs(plugin.sourcedir)
        open(os.path.join(plugin.sourcedir, 'setup.py'), 'w').close()
        os.makedirs(os.path.join(plugin.installdir, 'bin'))
        os.makedirs(os.path.join(
            plugin.installdir, 'usr', 'lib', 'python2', 'dist-packages'))
        os.makedirs(os.path.join(
            plugin.installdir, 'usr', 'include', 'python2'))

        # Place a few files with bad shebangs, and some files that shouldn't be
        # changed.
        files = [
            {
                'path': os.path.join(plugin.installdir, 'example.py'),
                'contents': '#!/foo/bar/baz/python',
                'expected': '#!/usr/bin/env python',
            },
            {
                'path': os.path.join(plugin.installdir, 'bin/another_example'),
                'contents': '#!/foo/baz/python2',
                'expected': '#!/usr/bin/env python2',
            },
            {
                'path': os.path.join(plugin.installdir, 'foo'),
                'contents': 'foo',
                'expected': 'foo',
            }
        ]

        for file_info in files:
            with open(file_info['path'], 'w') as f:
                f.write(file_info['contents'])

        plugin.build()

        for file_info in files:
            with open(os.path.join(plugin.installdir,
                                   file_info['path']), 'r') as f:
                self.assertEqual(f.read(), file_info['expected'])
