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
from snapcraft.plugins import python3


class Python3PluginTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        class Options:
            requirements = ''
            python_packages = []

        self.options = Options()
        self.project_options = snapcraft.ProjectOptions()

    @mock.patch.object(python3.Python3Plugin, 'run')
    @mock.patch.object(python3.Python3Plugin, 'run_output',
                       return_value='python3.4')
    def test_pip_relative_site_packages_symlink(self, run_output_mock,
                                                run_mock):
        plugin = python3.Python3Plugin('test-part', self.options,
                                       self.project_options)
        os.makedirs(plugin.sourcedir)
        os.makedirs(os.path.join(plugin.installdir, 'usr', 'lib', 'python3.4'))
        os.makedirs(os.path.join(plugin.installdir, 'usr', 'lib', 'python3',
                                 'dist-packages'))

        open(os.path.join(plugin.sourcedir, 'setup.py'), 'w').close()

        plugin._pip()

        link = os.readlink(os.path.join(plugin.installdir, 'usr', 'lib',
                                        'python3.4', 'site-packages'))
        expected = os.path.join('..', 'python3', 'dist-packages')
        self.assertEqual(link, expected,
                         'Expected site-packages to be a relative link to '
                         '"{}", but it was a link to "{}"'.format(expected,
                                                                  link))

    @mock.patch.object(python3.Python3Plugin, 'run')
    def test_build_fixes_python_shebangs(self, run_mock):
        plugin = python3.Python3Plugin('test-part', self.options,
                                       self.project_options)
        os.makedirs(plugin.sourcedir)
        open(os.path.join(plugin.sourcedir, 'setup.py'), 'w').close()
        os.makedirs(os.path.join(plugin.installdir, 'bin'))

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
