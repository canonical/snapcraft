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
import unittest.mock

import snapcraft
from snapcraft import tests


class TestBasePlugin(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.project_options = snapcraft.ProjectOptions()

    def test_parallel_build_count_returns_1_when_disabled(self):
        options = tests.MockOptions(disable_parallel=True)
        plugin = snapcraft.BasePlugin('test_plugin', options,
                                      self.project_options)
        self.assertEqual(plugin.parallel_build_count, 1)

    def test_parallel_build_count_returns_build_count_from_project(self):
        options = tests.MockOptions(disable_parallel=False)
        plugin = snapcraft.BasePlugin('test_plugin', options,
                                      self.project_options)
        unittest.mock.patch.object(
            self.project_options, 'parallel_build_count', 2)
        self.assertEqual(plugin.parallel_build_count, 2)

    def test_part_name_with_forward_slash_is_one_directory(self):
        plugin = snapcraft.BasePlugin('test/part', options=None)

        os.makedirs(plugin.sourcedir)

        self.assertIn('test\N{BIG SOLIDUS}part', os.listdir('parts'))

    @unittest.mock.patch('snapcraft.internal.common.run')
    def test_run_without_specifying_cwd(self, mock_run):
        plugin = snapcraft.BasePlugin('test/part', options=None)
        plugin.run(['ls'])

        mock_run.assert_called_once_with(['ls'], cwd=plugin.builddir)

    @unittest.mock.patch('snapcraft.internal.common.run')
    def test_run_specifying_a_cwd(self, mock_run):
        plugin = snapcraft.BasePlugin('test/part', options=None)
        plugin.run(['ls'], cwd=plugin.sourcedir)

        mock_run.assert_called_once_with(['ls'], cwd=plugin.sourcedir)

    @unittest.mock.patch('snapcraft.internal.common.run_output')
    def test_run_output_without_specifying_cwd(self, mock_run):
        plugin = snapcraft.BasePlugin('test/part', options=None)
        plugin.run_output(['ls'])

        mock_run.assert_called_once_with(['ls'], cwd=plugin.builddir)

    @unittest.mock.patch('snapcraft.internal.common.run_output')
    def test_run_output_specifying_a_cwd(self, mock_run):
        plugin = snapcraft.BasePlugin('test/part', options=None)
        plugin.run_output(['ls'], cwd=plugin.sourcedir)

        mock_run.assert_called_once_with(['ls'], cwd=plugin.sourcedir)
