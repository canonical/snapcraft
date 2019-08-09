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

import contextlib
import os

from testtools.matchers import Equals

from snapcraft.internal.meta import command, errors
from tests import unit


class CommandMangleTest(unit.TestCase):
    scenarios = (
        (
            "no change",
            dict(command_path="foo", command="foo bar", expected_command="foo bar"),
        ),
        (
            "no change subpath",
            dict(
                command_path="bin/foo",
                command="bin/foo bar",
                expected_command="bin/foo bar",
            ),
        ),
        (
            "starts with $SNAP",
            dict(
                command_path="foo", command="$SNAP/foo bar", expected_command="foo bar"
            ),
        ),
        (
            "quoted environment variable",
            dict(
                command_path="foo", command='foo "$bar"', expected_command='foo "$bar"'
            ),
        ),
        (
            "starts with $SNAP with subpath",
            dict(
                command_path="bin/foo",
                command="$SNAP/bin/foo bar",
                expected_command="bin/foo bar",
            ),
        ),
        (
            "find bin",
            dict(
                command_path="bin/foo",
                command="foo bar",
                expected_command="bin/foo bar",
            ),
        ),
        (
            "find sbin",
            dict(
                command_path="sbin/foo",
                command="foo bar",
                expected_command="sbin/foo bar",
            ),
        ),
        (
            "find usr/bin",
            dict(
                command_path="usr/bin/foo",
                command="foo bar",
                expected_command="usr/bin/foo bar",
            ),
        ),
        (
            "find usr/sbin",
            dict(
                command_path="usr/sbin/foo",
                command="foo bar",
                expected_command="usr/sbin/foo bar",
            ),
        ),
        ("find in root", dict(command="sh bar", expected_command="/bin/sh bar")),
        (
            "find in non stardard path",
            dict(
                command_path="bar/foo",
                command="foo bar",
                expected_command="bar/foo bar",
            ),
        ),
        (
            "find in non stardard path and preferred over root",
            dict(
                command_path="bar/sh", command="sh bar", expected_command="bar/sh bar"
            ),
        ),
        (
            "relative shebang",
            dict(
                command_path="foo",
                command="foo bar",
                expected_command="bin/python $SNAP/foo bar",
                shebang="#!/usr/bin/env $SNAP/bin/python",
                interpreter="bin/python",
            ),
        ),
        (
            "root shebang",
            dict(
                command_path="bin/foo",
                command="bin/foo bar",
                expected_command="bin/foo bar",
                shebang="/bin/python",
                interpreter="/bin/python",
            ),
        ),
        (
            "pathless shebang",
            dict(
                command_path="bin/foo",
                command="bin/foo bar",
                expected_command="bin/python $SNAP/bin/foo bar",
                shebang="#!/usr/bin/env python",
                interpreter="bin/python",
            ),
        ),
        (
            "pathless shebang with $SNAP leading command",
            dict(
                command_path="bin/foo",
                command="$SNAP/bin/foo bar",
                expected_command="bin/python $SNAP/bin/foo bar",
                shebang="#!/usr/bin/env python",
                interpreter="bin/python",
            ),
        ),
        (
            "with environment variable",
            dict(
                command_path="bin/foo",
                command="bin/foo bar $SNAP_DATA",
                expected_command="bin/foo bar $SNAP_DATA",
            ),
        ),
        (
            "single quotes preserved",
            dict(
                command_path="bin/foo",
                command="bin/foo bar '$SNAP_DATA'",
                expected_command="bin/foo bar '$SNAP_DATA'",
            ),
        ),
    )

    def setUp(self):
        super().setUp()

        self.prime_dir = os.path.join(self.path, "prime")
        with contextlib.suppress(AttributeError):
            command_path = os.path.join(self.prime_dir, self.command_path)
            os.makedirs(os.path.dirname(command_path))
            with open(command_path, "w") as command_file:
                with contextlib.suppress(AttributeError):
                    print(self.shebang, file=command_file)
            os.chmod(command_path, 0o755)

        with contextlib.suppress(AttributeError):
            if not self.interpreter.startswith("/"):
                interpreter_path = os.path.join(self.prime_dir, self.interpreter)
                os.makedirs(os.path.dirname(interpreter_path), exist_ok=True)
                open(interpreter_path, "w").close()
                os.chmod(interpreter_path, 0o755)

    def test_mangle(self):
        self.assertThat(
            command._massage_command(command=self.command, prime_dir=self.prime_dir),
            Equals(self.expected_command),
        )


class CommandMangleFindErrorTest(unit.TestCase):
    def test_find_binary_not_found(self):
        self.assertRaises(
            errors.PrimedCommandNotFoundError,
            command._massage_command,
            command="not-found",
            prime_dir=self.path,
        )

    def test_binary_not_executable(self):
        os.mkdir("bin")
        open(os.path.join("bin", "not-executable"), "w").close()

        self.assertRaises(
            errors.PrimedCommandNotFoundError,
            command._massage_command,
            command="not-executable",
            prime_dir=self.path,
        )
