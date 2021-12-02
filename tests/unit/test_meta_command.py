# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019 Canonical Ltd
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

import fixtures
from testtools.matchers import Equals

from snapcraft.meta import command, errors
from tests import unit


def _create_file(file_path: str, *, mode=0o755, contents="") -> None:
    os.makedirs(os.path.dirname(file_path), exist_ok=True)
    with open(file_path, "w") as f:
        if contents:
            f.write(contents)
    os.chmod(file_path, mode)


class CommandWithoutWrapperAllowedTest(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.fake_logger = fixtures.FakeLogger(level=logging.WARNING)
        self.useFixture(self.fake_logger)

    def tearDown(self):
        super().tearDown()

        self.assertThat(self.fake_logger.output, Equals(""))

    def test_command(self):
        _create_file(os.path.join(self.path, "foo"))
        cmd = command.Command(app_name="foo", command_name="command", command="foo")

        cmd.prime_command(prime_dir=self.path)

        self.assertThat(cmd.command, Equals("foo"))

    def test_command_with_args(self):
        _create_file(os.path.join(self.path, "foo"))
        cmd = command.Command(
            app_name="foo", command_name="command", command="foo bar -baz"
        )
        cmd.prime_command(prime_dir=self.path)

        self.assertThat(cmd.command, Equals("foo bar -baz"))


class CommandWithoutWrapperAllowedTestErrors(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.fake_logger = fixtures.FakeLogger(level=logging.WARNING)
        self.useFixture(self.fake_logger)

    def test_command_not_executable(self):
        _create_file(os.path.join(self.path, "foo"), mode=0o644)
        cmd = command.Command(app_name="foo", command_name="command", command="foo")

        self.assertRaises(
            errors.InvalidAppCommandNotExecutable,
            cmd.prime_command,
            prime_dir=self.path,
        )
        self.assertThat(self.fake_logger.output, Equals(""))

    def test_command_not_found(self):
        cmd = command.Command(app_name="foo", command_name="command", command="foo")

        self.assertRaises(
            errors.InvalidAppCommandNotFound,
            cmd.prime_command,
            prime_dir=self.path,
        )
        self.assertThat(self.fake_logger.output.strip(), Equals(""))
