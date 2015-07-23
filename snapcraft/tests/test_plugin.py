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

import logging
import os
import sys
import tempfile

import fixtures
from unittest.mock import (
    Mock,
    patch,
)

import snapcraft.tests.mock_plugin
from snapcraft.plugin import PluginHandler, PluginError
from snapcraft.tests import TestCase


class TestPlugin(TestCase):

    def get_test_plugin(self):
        return PluginHandler('mock', 'mock-part', {}, load_config=False, load_code=False)

    def test_is_dirty(self):
        p = self.get_test_plugin()
        p.statefile = tempfile.NamedTemporaryFile().name
        self.addCleanup(os.remove, p.statefile)
        p.code = Mock()
        # pull once
        p.pull()
        p.code.pull.assert_called()
        # pull again, not dirty no need to pull
        p.code.pull.reset_mock()
        p.pull()
        self.assertFalse(p.code.pull.called)

    def test_collect_snap_files(self):
        p = self.get_test_plugin()

        tmpdirObject = tempfile.TemporaryDirectory()
        self.addCleanup(tmpdirObject.cleanup)
        tmpdir = tmpdirObject.name

        p.installdir = tmpdir + '/install'
        os.makedirs(tmpdir + '/install/1/1a/1b')
        os.makedirs(tmpdir + '/install/2/2a')
        os.makedirs(tmpdir + '/install/3')
        open(tmpdir + '/install/a', mode='w').close()
        open(tmpdir + '/install/b', mode='w').close()
        open(tmpdir + '/install/1/a', mode='w').close()
        open(tmpdir + '/install/3/a', mode='w').close()

        p.stagedir = tmpdir + '/stage'
        os.makedirs(tmpdir + '/stage/1/1a/1b')
        os.makedirs(tmpdir + '/stage/2/2a')
        os.makedirs(tmpdir + '/stage/2/2b')
        os.makedirs(tmpdir + '/stage/3')
        open(tmpdir + '/stage/a', mode='w').close()
        open(tmpdir + '/stage/b', mode='w').close()
        open(tmpdir + '/stage/c', mode='w').close()
        open(tmpdir + '/stage/1/a', mode='w').close()
        open(tmpdir + '/stage/2/2b/a', mode='w').close()
        open(tmpdir + '/stage/3/a', mode='w').close()

        self.assertEqual(p.collect_snap_files([], []), (set(), set()))

        self.assertEqual(p.collect_snap_files(['*'], []), (
            set(['1', '1/1a', '1/1a/1b', '2', '2/2a', '3']),
            set(['a', 'b', '1/a', '3/a'])))

        self.assertEqual(p.collect_snap_files(['*'], ['1']), (
            set(['2', '2/2a', '3']),
            set(['a', 'b', '3/a'])))

        self.assertEqual(p.collect_snap_files(['a'], ['*']), (set(), set()))

        self.assertEqual(p.collect_snap_files(['*'], ['*/*']), (
            set(['1', '2', '3']),
            set(['a', 'b'])))

        self.assertEqual(p.collect_snap_files(['1', '2'], ['*/a']), (
            set(['1', '1/1a', '1/1a/1b', '2', '2/2a']),
            set()))

    def test_notify_stage_must_log_information(self):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)

        plugin = self.get_test_plugin()
        plugin.notify_stage('test stage')

        self.assertEqual('test stage mock-part\n', fake_logger.output)

    def test_local_plugins(self):
        """Ensure local plugins are loaded from parts/plugins"""
        def mock_import_modules(module_name):
            # called with the name only and sys.path set
            self.assertEqual(module_name, "x-mock")
            self.assertTrue(sys.path[0].endswith("parts/plugins"))
            return snapcraft.tests.mock_plugin
        with patch("importlib.import_module", side_effect=mock_import_modules):
            PluginHandler(
                "x-mock", "mock-part", {}, load_config=False, load_code=True)
        # sys.path is cleaned afterwards
        self.assertFalse(sys.path[0].endswith("parts/plugins"))

    def test_non_local_plugins(self):
        """Ensure regular plugins are loaded from snapcraft only"""
        def mock_import_modules(module_name):
            # called with the full snapcraft path
            self.assertEqual(module_name, "snapcraft.plugins.mock")
            return snapcraft.tests.mock_plugin
        with patch("importlib.import_module", side_effect=mock_import_modules):
            PluginHandler(
                "mock", "mock-part", {}, load_config=False, load_code=True)

    def test_collect_snap_files_with_abs_path_raises(self):
        # ensure that absolute path raise an error
        # (os.path.join will throw an error otherwise)
        with patch("importlib.import_module", return_value=snapcraft.tests.mock_plugin):
            p = PluginHandler("mock", "mock-part", {}, load_config=False)
        with self.assertRaises(PluginError):
            p.collect_snap_files(['*'], ['/1'])
