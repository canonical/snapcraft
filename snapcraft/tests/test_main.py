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

import logging
from unittest import mock

import fixtures

import snapcraft.main

from snapcraft.tests import TestCase


class TestMain(TestCase):

    @mock.patch('snapcraft.internal.lifecycle.snap')
    def test_default_command_is_snap(self, mock_cmd):
        with mock.patch('snapcraft.ProjectOptions') as mock_project_options:
            snapcraft.main.main([])
            mock_project_options.assert_called_once_with(
                debug=False, parallel_builds=True, target_deb_arch=None,
                use_geoip=False)
            self.assertTrue(mock_cmd.called, mock_cmd.called)

    @mock.patch('snapcraft.internal.lifecycle.snap')
    def test_command_with_geoip(self, mock_cmd):
        with mock.patch('snapcraft.ProjectOptions') as mock_project_options:
            snapcraft.main.main(['--enable-geoip'])
            self.assertTrue(mock_cmd.called, mock_cmd.called)
            mock_project_options.assert_called_once_with(
                debug=False, parallel_builds=True, target_deb_arch=None,
                use_geoip=True)

    def test_command_error(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)
        with mock.patch('snapcraft.topic_help') as mock_cmd:
            mock_cmd.side_effect = Exception('some error')

            raised = self.assertRaises(
                SystemExit,
                snapcraft.main.main, ['help', 'topics'])

        self.assertEqual(1, raised.code)
        self.assertEqual(fake_logger.output, 'some error\n')

    @mock.patch('snapcraft.internal.log.configure')
    def test_command_error_debug(self, mock_log_configure):
        with mock.patch('snapcraft.topic_help') as mock_cmd:
            mock_cmd.side_effect = Exception('some error')

            # When verbose, the exception should be re-raised instead of
            # SystemExit. Note that this test works since SystemExit doesn't
            # inherit from Exception.
            cm = self.assertRaises(
                Exception,
                snapcraft.main.main, ['help', 'topics', '--debug'])

        self.assertEqual(str(cm), 'some error')
        mock_log_configure.assert_called_once_with(
            log_level=logging.DEBUG)

    @mock.patch('snapcraft.internal.lifecycle.snap')
    def test_command_with_debug(self, mock_cmd):
        with mock.patch('snapcraft.ProjectOptions') as mock_project_options:
            snapcraft.main.main(['--debug'])
            mock_project_options.assert_called_once_with(
                debug=True, parallel_builds=True, target_deb_arch=None,
                use_geoip=False)

    @mock.patch('snapcraft.internal.lifecycle.snap')
    def test_command_with_parallel_builds(self, mock_cmd):
        with mock.patch('snapcraft.ProjectOptions') as mock_project_options:
            snapcraft.main.main([])
            mock_project_options.assert_called_once_with(
                debug=False, parallel_builds=True, target_deb_arch=None,
                use_geoip=False)

    @mock.patch('snapcraft.internal.lifecycle.snap')
    def test_command_disable_parallel_build(self, mock_cmd):
        with mock.patch('snapcraft.ProjectOptions') as mock_project_options:
            snapcraft.main.main(['--no-parallel-build'])
            mock_project_options.assert_called_once_with(
                debug=False, parallel_builds=False, target_deb_arch=None,
                use_geoip=False)

    @mock.patch('snapcraft.internal.lifecycle.snap')
    def test_command_with_target_deb_arch(self, mock_cmd):
        with mock.patch('snapcraft.ProjectOptions') as mock_project_options:
            snapcraft.main.main(['--target-arch', 'arm64'])
            mock_project_options.assert_called_once_with(
                debug=False, parallel_builds=True, target_deb_arch='arm64',
                use_geoip=False)
