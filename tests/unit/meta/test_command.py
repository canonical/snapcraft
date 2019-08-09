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

import os

from testtools.matchers import Equals, Is, FileContains, FileExists

from snapcraft.internal.meta import command, errors
from tests import unit


def _create_file(file_path: str, *, mode=0o755) -> None:
    os.makedirs(os.path.dirname(file_path), exist_ok=True)
    open(file_path, "w").close()
    os.chmod(file_path, mode)


class CommandWithoutWrapperTest(unit.TestCase):
    def test_command(self):
        _create_file(os.path.join(self.path, "foo"))
        cmd = command.Command(
            app_name="foo",
            command_name="command",
            command="foo",
            prime_dir=self.path,
            can_use_wrapper=False,
        )

        self.assertThat(cmd.get_command(), Equals("foo"))

    def test_command_with_args(self):
        _create_file(os.path.join(self.path, "foo"))
        cmd = command.Command(
            app_name="foo",
            command_name="command",
            command="foo bar -baz",
            prime_dir=self.path,
            can_use_wrapper=False,
        )

        self.assertThat(cmd.get_command(), Equals("foo bar -baz"))

    def test_command_does_not_match_snapd_pattern(self):
        _create_file(os.path.join(self.path, "foo"))
        cmd = command.Command(
            app_name="foo",
            command_name="command",
            command="foo /!option",
            prime_dir=self.path,
            can_use_wrapper=False,
        )

        self.assertRaises(errors.InvalidAppCommandFormatError, cmd.get_command)

    def test_command_starts_with_slash(self):
        cmd = command.Command(
            app_name="foo",
            command_name="command",
            command="/foo",
            prime_dir=self.path,
            can_use_wrapper=False,
        )

        self.assertRaises(errors.InvalidAppCommandFormatError, cmd.get_command)

    def test_command_relative_command_found_in_slash(self):
        cmd = command.Command(
            app_name="foo",
            command_name="command",
            command="sh",
            prime_dir=self.path,
            can_use_wrapper=False,
        )

        self.assertRaises(errors.InvalidAppCommandFormatError, cmd.get_command)

    def test_command_not_executable(self):
        _create_file(os.path.join(self.path, "foo"), mode=0o644)
        cmd = command.Command(
            app_name="foo",
            command_name="command",
            command="foo",
            prime_dir=self.path,
            can_use_wrapper=False,
        )

        self.assertRaises(errors.InvalidAppCommandNotExecutable, cmd.get_command)


class CommandWithWrapperTest(unit.TestCase):
    def test_command(self):
        _create_file(os.path.join(self.path, "foo"))
        cmd = command.Command(
            app_name="foo",
            command_name="command",
            command="foo",
            prime_dir=self.path,
            can_use_wrapper=True,
        )

        self.assertThat(cmd.get_command(), Equals("foo"))

    def test_command_with_args(self):
        _create_file(os.path.join(self.path, "foo"))
        cmd = command.Command(
            app_name="foo",
            command_name="command",
            command="foo bar -baz",
            prime_dir=self.path,
            can_use_wrapper=False,
        )

        self.expectThat(cmd.get_command(), Equals("foo bar -baz"))
        self.expectThat(cmd.generate_wrapper(), Is(None))

    def test_command_does_not_match_snapd_pattern(self):
        _create_file(os.path.join(self.path, "foo"))
        cmd = command.Command(
            app_name="foo",
            command_name="command",
            command="foo /!option",
            prime_dir=self.path,
            can_use_wrapper=True,
        )

        app_command = cmd.get_command()
        wrapper_path = cmd.generate_wrapper()
        self.expectThat(app_command, Equals("command-foo.wrapper"))
        self.assertThat(wrapper_path, FileExists())
        self.assertThat(
            wrapper_path, FileContains('#!/bin/sh\nexec $SNAP/foo /!option "$@"\n')
        )

    def test_command_starts_with_slash(self):
        cmd = command.Command(
            app_name="foo",
            command_name="command",
            command="/foo",
            prime_dir=self.path,
            can_use_wrapper=True,
        )

        app_command = cmd.get_command()
        wrapper_path = cmd.generate_wrapper()
        self.expectThat(app_command, Equals("command-foo.wrapper"))
        self.assertThat(wrapper_path, FileExists())
        self.assertThat(wrapper_path, FileContains('#!/bin/sh\nexec /foo "$@"\n'))

    def test_command_relative_command_found_in_slash(self):
        cmd = command.Command(
            app_name="foo",
            command_name="command",
            command="sh",
            prime_dir=self.path,
            can_use_wrapper=True,
        )

        app_command = cmd.get_command()
        wrapper_path = cmd.generate_wrapper()
        self.expectThat(app_command, Equals("command-foo.wrapper"))
        self.assertThat(wrapper_path, FileExists())
        self.assertThat(wrapper_path, FileContains('#!/bin/sh\nexec /bin/sh "$@"\n'))

    def test_command_not_executable(self):
        _create_file(os.path.join(self.path, "foo"), mode=0o644)
        cmd = command.Command(
            app_name="foo",
            command_name="command",
            command="foo",
            prime_dir=self.path,
            can_use_wrapper=True,
        )

        self.assertRaises(errors.InvalidAppCommandNotExecutable, cmd.get_command)

    def test_generate_wrapper_called_before_get_command(self):
        cmd = command.Command(
            app_name="foo",
            command_name="command",
            command="/foo",
            prime_dir=self.path,
            can_use_wrapper=True,
        )

        self.assertRaises(RuntimeError, cmd.generate_wrapper)
