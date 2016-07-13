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
from snapcraft import tests
from snapcraft.plugins import ant


class AntPluginTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        class Options:
            ant_options = []
        self.options = Options()

        self.project_options = snapcraft.ProjectOptions()

        patcher = mock.patch('snapcraft.repo.Ubuntu')
        self.ubuntu_mock = patcher.start()
        self.addCleanup(patcher.stop)

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
    def test_build_fail(self, run_mock):
        plugin = ant.AntPlugin('test-part', self.options,
                               self.project_options)

        os.makedirs(plugin.sourcedir)
        with self.assertRaises(RuntimeError):
            plugin.build()

        run_mock.assert_has_calls([
            mock.call(['ant']),
        ])

    def test_env(self):
        plugin = ant.AntPlugin('test-part', self.options,
                               self.project_options)

        os.makedirs(os.path.join(plugin.installdir, 'jar'))
        open(os.path.join(plugin.installdir,
             'jar', 'lib1.jar'), 'w').close()
        open(os.path.join(plugin.installdir,
             'jar', 'lib2.jar'), 'w').close()
        plugin.env(plugin.partdir)
