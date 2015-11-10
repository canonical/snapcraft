
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
from snapcraft.plugins import cmake


class CMakeTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        patcher = mock.patch('snapcraft.common.run')
        self.run_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('sys.stdout')
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_build_referencing_sourcedir_if_no_subdir(self):
        class Options:
            configflags = []

        plugin = cmake.CMakePlugin('test-part', Options())
        os.makedirs(plugin.builddir)
        plugin.build()

        self.run_mock.assert_has_calls([
            mock.call(['cmake', plugin.sourcedir, '-DCMAKE_INSTALL_PREFIX='],
                      cwd=plugin.builddir),
            mock.call(['make', 'install',
                       'DESTDIR={}'.format(plugin.installdir)],
                      cwd=plugin.builddir)])

    def test_build_referencing_sourcedir_with_subdir(self):
        class Options:
            configflags = []
            source_subdir = 'subdir'

        plugin = cmake.CMakePlugin('test-part', Options())
        os.makedirs(plugin.builddir)
        plugin.build()

        sourcedir = os.path.join(
            plugin.sourcedir, plugin.options.source_subdir)
        self.run_mock.assert_has_calls([
            mock.call(['cmake', sourcedir, '-DCMAKE_INSTALL_PREFIX='],
                      cwd=plugin.builddir),
            mock.call(['make', 'install',
                       'DESTDIR={}'.format(plugin.installdir)],
                      cwd=plugin.builddir)])
