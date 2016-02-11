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

from snapcraft import tests
from snapcraft.plugins import python2


class Python2PluginTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        class Options:
            requirements = ''
            python_packages = []

        self.options = Options()

    @mock.patch.object(python2.Python2Plugin, 'run')
    @mock.patch.object(python2.Python2Plugin, 'run_output',
                       return_value='python2.7')
    def test_pip_relative_site_packages_symlink(self, run_output_mock,
                                                run_mock):
        plugin = python2.Python2Plugin('test-part', self.options)
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
