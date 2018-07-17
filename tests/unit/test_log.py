# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2018 Canonical Ltd
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

from testtools.matchers import Contains, Equals, Not

from snapcraft.internal import log
from tests import fixture_setup, unit


class LogTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.info_color = log._ColoredFormatter.LEVEL_COLORS["INFO"]
        self.warning_color = log._ColoredFormatter.LEVEL_COLORS["WARNING"]
        self.error_color = log._ColoredFormatter.LEVEL_COLORS["ERROR"]
        self.critical_color = log._ColoredFormatter.LEVEL_COLORS["CRITICAL"]

    def test_configure_must_send_messages_to_stdout(self):
        logger_name = self.id()
        log.configure(logger_name)
        logger = logging.getLogger(logger_name)
        # Overwrite the level to log everything.
        logger.setLevel(logging.DEBUG)

        logger.debug("Test debug")
        logger.info("Test info")
        logger.warning("Test warning")

        stdout = self.fake_terminal.getvalue()
        self.assertThat(stdout, Contains("Test debug"))
        expected_info = "{}Test info\033[0m".format(self.info_color)
        self.assertThat(stdout, Contains(expected_info))
        expected_warning = "{}Test warning\033[0m".format(self.warning_color)
        self.assertThat(stdout, Contains(expected_warning))
        self.assertThat(self.fake_terminal.getvalue(stderr=True), Equals(""))

    def test_configure_must_send_errors_to_stderr(self):
        logger_name = self.id()
        log.configure(logger_name)
        logger = logging.getLogger(logger_name)
        # Overwrite the level to log everything.
        logger.setLevel(logging.DEBUG)

        logger.error("Test error")
        logger.critical("Test critical")

        stderr = self.fake_terminal.getvalue(stderr=True)
        expected_error = "{}Test error\033[0m".format(self.error_color)
        self.assertThat(stderr, Contains(expected_error))
        expected_crit = "{}Test critical\033[0m".format(self.critical_color)
        self.assertThat(stderr, Contains(expected_crit))
        self.assertThat(self.fake_terminal.getvalue(), Equals(""))

    def test_configure_must_log_info_and_higher(self):
        logger_name = self.id()
        log.configure(logger_name)
        logger = logging.getLogger(logger_name)

        logger.debug("Test debug")
        logger.info("Test info")
        logger.warning("Test warning")
        logger.error("Test error")
        logger.critical("Test critical")

        stdout = self.fake_terminal.getvalue()
        self.assertThat(stdout, Not(Contains("Test debug")))
        expected_info = "{}Test info\033[0m".format(self.info_color)
        self.assertThat(stdout, Contains(expected_info))
        expected_warning = "{}Test warning\033[0m".format(self.warning_color)
        self.assertThat(stdout, Contains(expected_warning))

        stderr = self.fake_terminal.getvalue(stderr=True)
        expected_error = "{}Test error\033[0m".format(self.error_color)
        self.assertThat(stderr, Contains(expected_error))
        expected_crit = "{}Test critical\033[0m".format(self.critical_color)
        self.assertThat(stderr, Contains(expected_crit))

    def test_configure_must_support_debug(self):
        logger_name = self.id()
        log.configure(logger_name, log_level=logging.DEBUG)
        logger = logging.getLogger(logger_name)

        logger.debug("Test debug")
        logger.info("Test info")
        logger.warning("Test warning")
        logger.error("Test error")
        logger.critical("Test critical")

        stdout = self.fake_terminal.getvalue()
        self.assertThat(stdout, Contains("Test debug"))
        expected_info = "{}Test info\033[0m".format(self.info_color)
        self.assertThat(stdout, Contains(expected_info))
        expected_warning = "{}Test warning\033[0m".format(self.warning_color)
        self.assertThat(stdout, Contains(expected_warning))

        stderr = self.fake_terminal.getvalue(stderr=True)
        expected_error = "{}Test error\033[0m".format(self.error_color)
        self.assertThat(stderr, Contains(expected_error))
        expected_crit = "{}Test critical\033[0m".format(self.critical_color)
        self.assertThat(stderr, Contains(expected_crit))

    def test_configure_must_support_no_tty(self):
        self.fake_terminal = fixture_setup.FakeTerminal(isatty=False)
        self.useFixture(self.fake_terminal)
        logger_name = self.id()
        log.configure(logger_name, log_level=logging.DEBUG)
        logger = logging.getLogger(logger_name)

        logger.debug("Test debug")
        logger.info("Test info")
        logger.warning("Test warning")
        logger.error("Test error")
        logger.critical("Test critical")

        stdout = self.fake_terminal.getvalue()
        self.assertThat(stdout, Contains("Test debug"))
        self.assertThat(stdout, Contains("Test info"))
        self.assertThat(stdout, Not(Contains(self.info_color)))
        self.assertThat(stdout, Contains("Test warning"))
        self.assertThat(stdout, Not(Contains(self.warning_color)))
        self.assertThat(stdout, Not(Contains("\033[0m")))

        stderr = self.fake_terminal.getvalue(stderr=True)
        self.assertThat(stderr, Contains("Test error"))
        self.assertThat(stderr, Not(Contains(self.error_color)))
        self.assertThat(stderr, Contains("Test critical"))
        self.assertThat(stderr, Not(Contains(self.critical_color)))
