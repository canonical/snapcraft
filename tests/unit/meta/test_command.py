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
import shutil

import fixtures
from testtools.matchers import Equals, FileContains, FileExists, Is

from snapcraft.internal.meta import command, errors
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

        cmd.prime_command(
            can_use_wrapper=False, massage_command=True, prime_dir=self.path,
        )

        self.assertThat(cmd.command, Equals("foo"))

    def test_command_with_args(self):
        _create_file(os.path.join(self.path, "foo"))
        cmd = command.Command(
            app_name="foo", command_name="command", command="foo bar -baz"
        )
        cmd.prime_command(
            can_use_wrapper=False, massage_command=True, prime_dir=self.path,
        )

        self.assertThat(cmd.command, Equals("foo bar -baz"))


def test_interpretered_command_from_host(monkeypatch, tmp_path):
    monkeypatch.setattr(shutil, "which", lambda x: "/bin/python3")

    _create_file(os.path.join(tmp_path, "foo"), contents="#!/usr/bin/env python3\n")
    cmd = command.Command(
        app_name="foo", command_name="command", command="foo bar -baz"
    )
    cmd.prime_command(
        can_use_wrapper=True, massage_command=True, prime_dir=tmp_path.as_posix(),
    )

    assert cmd.command == "command-foo.wrapper"
    assert cmd.wrapped_command == "/bin/python3 $SNAP/foo bar -baz"


def test_interpretered_command_from_prime(tmp_path):
    _create_file(os.path.join(tmp_path, "bin", "python3"))
    _create_file(os.path.join(tmp_path, "foo"), contents="#!/usr/bin/env python3\n")

    cmd = command.Command(
        app_name="foo", command_name="command", command="foo bar -baz"
    )
    cmd.prime_command(
        can_use_wrapper=True, massage_command=True, prime_dir=tmp_path.as_posix(),
    )

    assert cmd.command == "bin/python3 $SNAP/foo bar -baz"


def test_interpretered_command_from_root(tmp_path):
    _create_file(os.path.join(tmp_path, "foo"), contents="#!/bin/sh\n")

    cmd = command.Command(
        app_name="foo", command_name="command", command="foo bar -baz"
    )
    cmd.prime_command(
        can_use_wrapper=True, massage_command=True, prime_dir=tmp_path.as_posix(),
    )

    assert cmd.command == "foo bar -baz"


class CommandWithoutWrapperAllowedTestErrors(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.fake_logger = fixtures.FakeLogger(level=logging.WARNING)
        self.useFixture(self.fake_logger)

    def test_command_starts_with_slash(self):
        cmd = command.Command(app_name="foo", command_name="command", command="/foo")

        self.assertRaises(
            errors.InvalidAppCommandFormatError,
            cmd.prime_command,
            can_use_wrapper=False,
            massage_command=True,
            prime_dir=self.path,
        )
        self.assertThat(self.fake_logger.output, Equals(""))

    def test_command_relative_command_found_in_slash(self):
        cmd = command.Command(app_name="foo", command_name="command", command="sh")

        self.assertRaises(
            errors.InvalidAppCommandFormatError,
            cmd.prime_command,
            can_use_wrapper=False,
            massage_command=True,
            prime_dir=self.path,
        )
        self.assertThat(
            self.fake_logger.output.strip(),
            Equals(
                "The command 'sh' for 'sh' was resolved to '/bin/sh'.\n"
                "The command 'sh' has been changed to '/bin/sh'."
            ),
        )

    def test_command_does_not_match_snapd_pattern(self):
        _create_file(os.path.join(self.path, "foo"))
        cmd = command.Command(
            app_name="foo", command_name="command", command="foo /!option"
        )

        self.assertRaises(
            errors.InvalidAppCommandFormatError,
            cmd.prime_command,
            can_use_wrapper=False,
            massage_command=True,
            prime_dir=self.path,
        )
        self.assertThat(self.fake_logger.output, Equals(""))

    def test_command_not_executable(self):
        _create_file(os.path.join(self.path, "foo"), mode=0o644)
        cmd = command.Command(app_name="foo", command_name="command", command="foo")

        self.assertRaises(
            errors.InvalidAppCommandNotExecutable,
            cmd.prime_command,
            can_use_wrapper=False,
            massage_command=True,
            prime_dir=self.path,
        )
        self.assertThat(self.fake_logger.output, Equals(""))

    def test_command_not_found(self):
        cmd = command.Command(app_name="foo", command_name="command", command="foo")

        self.assertRaises(
            errors.InvalidAppCommandNotFound,
            cmd.prime_command,
            can_use_wrapper=False,
            massage_command=False,
            prime_dir=self.path,
        )
        self.assertThat(self.fake_logger.output.strip(), Equals(""))


class CommandWithWrapperTest(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.fake_logger = fixtures.FakeLogger(level=logging.WARNING)
        self.useFixture(self.fake_logger)

    def test_command(self):
        _create_file(os.path.join(self.path, "foo"))
        cmd = command.Command(app_name="foo", command_name="command", command="foo")
        cmd.prime_command(
            can_use_wrapper=True, massage_command=True, prime_dir=self.path,
        )

        self.assertThat(cmd.command, Equals("foo"))
        self.assertThat(self.fake_logger.output, Equals(""))

    def test_command_with_dollar_snap_and_does_not_match_snapd_pattern(self):
        _create_file(os.path.join(self.path, "foo"))
        cmd = command.Command(
            app_name="foo", command_name="command", command="$SNAP/foo !option"
        )
        cmd.prime_command(
            can_use_wrapper=True, massage_command=True, prime_dir=self.path,
        )

        self.assertThat(cmd.command, Equals("command-foo.wrapper"))
        self.assertThat(
            self.fake_logger.output.strip(),
            Equals(
                "Found unneeded '$SNAP/' in command '$SNAP/foo !option'.\n"
                "The command '$SNAP/foo !option' has been changed to 'foo !option'.\n"
                "A shell wrapper will be generated for command 'foo !option' "
                "as it does not conform with the command pattern expected "
                "by the runtime. Commands must be relative to the prime "
                "directory and can only consist of alphanumeric characters, "
                "spaces, and the following special characters: / . _ # : $ -"
            ),
        )

    def test_command_with_args(self):
        _create_file(os.path.join(self.path, "foo"))
        cmd = command.Command(
            app_name="foo", command_name="command", command="foo bar -baz"
        )

        cmd.prime_command(
            can_use_wrapper=True, massage_command=True, prime_dir=self.path,
        )
        wrapper_path = cmd.write_wrapper(prime_dir=self.path)

        self.expectThat(cmd.command, Equals("foo bar -baz"))
        self.expectThat(wrapper_path, Is(None))
        self.assertThat(self.fake_logger.output, Equals(""))

    def test_command_does_not_match_snapd_pattern(self):
        _create_file(os.path.join(self.path, "foo"))
        cmd = command.Command(
            app_name="foo", command_name="command", command="foo /!option"
        )

        cmd.prime_command(
            can_use_wrapper=True, massage_command=True, prime_dir=self.path,
        )
        wrapper_path = cmd.write_wrapper(prime_dir=self.path)

        self.expectThat(cmd.command, Equals("command-foo.wrapper"))
        self.assertThat(wrapper_path, FileExists())
        self.assertThat(
            wrapper_path, FileContains('#!/bin/sh\nexec $SNAP/foo /!option "$@"\n')
        )
        self.assertThat(
            self.fake_logger.output.strip(),
            Equals(
                "A shell wrapper will be generated for command 'foo /!option' "
                "as it does not conform with the command pattern expected "
                "by the runtime. Commands must be relative to the prime "
                "directory and can only consist of alphanumeric characters, "
                "spaces, and the following special characters: / . _ # : $ -"
            ),
        )

    def test_command_starts_with_slash(self):
        cmd = command.Command(app_name="foo", command_name="command", command="/foo")

        cmd.prime_command(
            can_use_wrapper=True, massage_command=True, prime_dir=self.path,
        )
        wrapper_path = cmd.write_wrapper(prime_dir=self.path)

        self.expectThat(cmd.command, Equals("command-foo.wrapper"))
        self.assertThat(wrapper_path, FileExists())
        self.assertThat(wrapper_path, FileContains('#!/bin/sh\nexec /foo "$@"\n'))
        self.assertThat(
            self.fake_logger.output.strip(),
            Equals(
                "A shell wrapper will be generated for command '/foo' "
                "as it does not conform with the command pattern expected "
                "by the runtime. Commands must be relative to the prime "
                "directory and can only consist of alphanumeric characters, "
                "spaces, and the following special characters: / . _ # : $ -"
            ),
        )

    def test_command_relative_command_found_in_slash(self):
        cmd = command.Command(app_name="foo", command_name="command", command="sh")

        cmd.prime_command(
            can_use_wrapper=True, massage_command=True, prime_dir=self.path,
        )
        wrapper_path = cmd.write_wrapper(prime_dir=self.path)

        self.expectThat(cmd.command, Equals("command-foo.wrapper"))
        self.assertThat(wrapper_path, FileExists())
        self.assertThat(wrapper_path, FileContains('#!/bin/sh\nexec /bin/sh "$@"\n'))
        self.assertThat(
            self.fake_logger.output.strip(),
            Equals(
                "The command 'sh' for 'sh' was resolved to '/bin/sh'.\n"
                "The command 'sh' has been changed to '/bin/sh'.\n"
                "A shell wrapper will be generated for command '/bin/sh' "
                "as it does not conform with the command pattern expected "
                "by the runtime. Commands must be relative to the prime "
                "directory and can only consist of alphanumeric characters, "
                "spaces, and the following special characters: / . _ # : $ -"
            ),
        )

    def test_command_not_executable(self):
        _create_file(os.path.join(self.path, "foo"), mode=0o644)
        cmd = command.Command(app_name="foo", command_name="command", command="foo")

        self.assertRaises(
            errors.InvalidAppCommandNotExecutable,
            cmd.prime_command,
            can_use_wrapper=True,
            massage_command=True,
            prime_dir=self.path,
        )
        self.assertThat(self.fake_logger.output.strip(), Equals(""))

    def test_command_not_found(self):
        cmd = command.Command(app_name="foo", command_name="command", command="foo")

        self.assertRaises(
            errors.InvalidAppCommandNotFound,
            cmd.prime_command,
            can_use_wrapper=True,
            massage_command=True,
            prime_dir=self.path,
        )
        self.assertThat(self.fake_logger.output.strip(), Equals(""))
