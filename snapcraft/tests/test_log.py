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

from snapcraft import (
    log,
    tests
)


@mock.patch('sys.stdout', new_callable=io.StringIO)
@mock.patch('sys.stderr', new_callable=io.StringIO)
class LogTestCase(tests.TestCase):

    def test_configure_must_send_messages_to_stdout(
            self, mock_stderr, mock_stdout):
        logger_name = self.id()
        log.configure(logger_name)
        logger = logging.getLogger(logger_name)
        # Overwrite the level to log everything.
        logger.setLevel(logging.DEBUG)

        logger.debug('Test debug')
        logger.info('Test info')
        logger.warning('Test warning')

        expected_out = (
            '\033[1mTest debug\033[0m\n'
            '\033[1mTest info\033[0m\n'
            '\033[1mTest warning\033[0m\n')
        self.assertEqual(expected_out, mock_stdout.getvalue())
        self.assertEqual('', mock_stderr.getvalue())

    def test_configure_must_send_errors_to_stderr(
            self, mock_stderr, mock_stdout):
        logger_name = self.id()
        log.configure(logger_name)
        logger = logging.getLogger(logger_name)
        # Overwrite the level to log everything.
        logger.setLevel(logging.DEBUG)

        logger.error('Test error')
        logger.critical('Test critical')

        expected_err = (
            '\033[1mTest error\033[0m\n'
            '\033[1mTest critical\033[0m\n')
        self.assertEqual(expected_err, mock_stderr.getvalue())
        self.assertEqual('', mock_stdout.getvalue())

    def test_configure_must_log_info_and_higher(
            self, mock_stderr, mock_stdout):
        logger_name = self.id()
        log.configure(logger_name)
        logger = logging.getLogger(logger_name)

        logger.debug('Test debug')
        logger.info('Test info')
        logger.warning('Test warning')
        logger.error('Test error')
        logger.critical('Test critical')

        expected_out = (
            '\033[1mTest info\033[0m\n'
            '\033[1mTest warning\033[0m\n')
        expected_err = (
            '\033[1mTest error\033[0m\n'
            '\033[1mTest critical\033[0m\n')
        self.assertEqual(expected_out, mock_stdout.getvalue())
        self.assertEqual(expected_err, mock_stderr.getvalue())
