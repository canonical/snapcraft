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

import io
import logging
from unittest import mock

from snapcraft.internal import log
from snapcraft import tests


@mock.patch('os.isatty', return_value=True)
@mock.patch('sys.stdout', new_callable=io.StringIO)
@mock.patch('sys.stderr', new_callable=io.StringIO)
class LogTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        self.info_color = log._ColoredFormatter.LEVEL_COLORS['INFO']
        self.warning_color = log._ColoredFormatter.LEVEL_COLORS['WARNING']
        self.error_color = log._ColoredFormatter.LEVEL_COLORS['ERROR']
        self.critical_color = log._ColoredFormatter.LEVEL_COLORS['CRITICAL']

    def test_configure_must_send_messages_to_stdout(
            self, mock_stderr, mock_stdout, mock_isatty):
        logger_name = self.id()
        log.configure(logger_name)
        logger = logging.getLogger(logger_name)
        # Overwrite the level to log everything.
        logger.setLevel(logging.DEBUG)

        logger.debug('Test debug')
        logger.info('Test info')
        logger.warning('Test warning')

        expected_out = ('Test debug\n'
                        '{}Test info\033[0m\n'
                        '{}Test warning\033[0m\n').format(
            self.info_color, self.warning_color)
        self.assertEqual(expected_out, mock_stdout.getvalue())
        self.assertEqual('', mock_stderr.getvalue())

    def test_configure_must_send_errors_to_stderr(
            self, mock_stderr, mock_stdout, mock_isatty):
        logger_name = self.id()
        log.configure(logger_name)
        logger = logging.getLogger(logger_name)
        # Overwrite the level to log everything.
        logger.setLevel(logging.DEBUG)

        logger.error('Test error')
        logger.critical('Test critical')

        expected_err = ('{}Test error\033[0m\n'
                        '{}Test critical\033[0m\n').format(
            self.error_color, self.critical_color)
        self.assertEqual(expected_err, mock_stderr.getvalue())
        self.assertEqual('', mock_stdout.getvalue())

    def test_configure_must_log_info_and_higher(
            self, mock_stderr, mock_stdout, mock_isatty):
        logger_name = self.id()
        log.configure(logger_name)
        logger = logging.getLogger(logger_name)

        logger.debug('Test debug')
        logger.info('Test info')
        logger.warning('Test warning')
        logger.error('Test error')
        logger.critical('Test critical')

        expected_out = ('{}Test info\033[0m\n'
                        '{}Test warning\033[0m\n').format(
            self.info_color, self.warning_color)
        expected_err = ('{}Test error\033[0m\n'
                        '{}Test critical\033[0m\n').format(
            self.error_color, self.critical_color)
        self.assertEqual(expected_out, mock_stdout.getvalue())
        self.assertEqual(expected_err, mock_stderr.getvalue())

    def test_configure_must_support_debug(
            self, mock_stderr, mock_stdout, mock_isatty):
        logger_name = self.id()
        log.configure(logger_name, log_level=logging.DEBUG)
        logger = logging.getLogger(logger_name)

        logger.debug('Test debug')
        logger.info('Test info')
        logger.warning('Test warning')
        logger.error('Test error')
        logger.critical('Test critical')

        expected_out = ('Test debug\n'
                        '{}Test info\033[0m\n'
                        '{}Test warning\033[0m\n').format(
            self.info_color, self.warning_color)
        expected_err = ('{}Test error\033[0m\n'
                        '{}Test critical\033[0m\n').format(
            self.error_color, self.critical_color)
        self.assertEqual(expected_out, mock_stdout.getvalue())
        self.assertEqual(expected_err, mock_stderr.getvalue())

    def test_configure_must_support_no_tty(
            self, mock_stderr, mock_stdout, mock_isatty):
        mock_isatty.return_value = False
        logger_name = self.id()
        log.configure(logger_name, log_level=logging.DEBUG)
        logger = logging.getLogger(logger_name)

        logger.debug('Test debug')
        logger.info('Test info')
        logger.warning('Test warning')
        logger.error('Test error')
        logger.critical('Test critical')

        expected_out = ('Test debug\n'
                        'Test info\n'
                        'Test warning\n').format(
            self.info_color, self.warning_color)
        expected_err = ('Test error\n'
                        'Test critical\n').format(
            self.error_color, self.critical_color)
        self.assertEqual(expected_out, mock_stdout.getvalue())
        self.assertEqual(expected_err, mock_stderr.getvalue())
