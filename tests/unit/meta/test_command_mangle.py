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

import pytest

from snapcraft.internal.meta import command, errors


class TestCommandMangle:
    scenarios = (
        (
            "no change",
            dict(
                command_path="foo",
                command_value="foo bar",
                expected_command="foo bar",
                expected_log=None,
                shebang="",
                interpreter_path=None,
            ),
        ),
        (
            "no change subpath",
            dict(
                command_path="bin/foo",
                command_value="bin/foo bar",
                expected_command="bin/foo bar",
                expected_log=None,
                shebang="",
                interpreter_path=None,
            ),
        ),
        (
            "starts with $SNAP",
            dict(
                command_path="foo",
                command_value="$SNAP/foo bar",
                expected_command="foo bar",
                expected_log="Stripped '$SNAP/' from command '$SNAP/foo bar'.",
                shebang="",
                interpreter_path=None,
            ),
        ),
        (
            "quoted environment variable",
            dict(
                command_path="foo",
                command_value='foo "$bar"',
                expected_command='foo "$bar"',
                expected_log=None,
                shebang="",
                interpreter_path=None,
            ),
        ),
        (
            "starts with $SNAP with subpath",
            dict(
                command_path="bin/foo",
                command_value="$SNAP/bin/foo bar",
                expected_command="bin/foo bar",
                expected_log="Stripped '$SNAP/' from command '$SNAP/bin/foo bar'.",
                shebang="",
                interpreter_path=None,
            ),
        ),
        (
            "find bin",
            dict(
                command_path="bin/foo",
                command_value="foo bar",
                expected_command="bin/foo bar",
                expected_log=None,
                shebang="",
                interpreter_path=None,
            ),
        ),
        (
            "find sbin",
            dict(
                command_path="sbin/foo",
                command_value="foo bar",
                expected_command="sbin/foo bar",
                expected_log=None,
                shebang="",
                interpreter_path=None,
            ),
        ),
        (
            "find usr/bin",
            dict(
                command_path="usr/bin/foo",
                command_value="foo bar",
                expected_command="usr/bin/foo bar",
                expected_log=None,
                shebang="",
                interpreter_path=None,
            ),
        ),
        (
            "find usr/sbin",
            dict(
                command_path="usr/sbin/foo",
                command_value="foo bar",
                expected_command="usr/sbin/foo bar",
                expected_log=None,
                shebang="",
                interpreter_path=None,
            ),
        ),
        (
            "find in root",
            dict(
                command_path=None,
                command_value="sh bar",
                expected_command="/bin/sh bar",
                expected_log=(
                    "The command 'sh bar' was not found in the prime directory, "
                    "it has been changed to '/bin/sh'."
                ),
                shebang="",
                interpreter_path=None,
            ),
        ),
        (
            "find in non standard path",
            dict(
                command_path="bar/foo",
                command_value="foo bar",
                expected_command="bar/foo bar",
                expected_log=None,
                shebang="",
                interpreter_path=None,
            ),
        ),
        (
            "find in non stardard path and preferred over root",
            dict(
                command_path="bar/sh",
                command_value="sh bar",
                expected_command="bar/sh bar",
                expected_log=None,
                shebang="",
                interpreter_path=None,
            ),
        ),
        (
            "relative shebang",
            dict(
                command_path="foo",
                command_value="foo bar",
                expected_command="bin/python $SNAP/foo bar",
                expected_log=None,
                shebang="#!/usr/bin/env $SNAP/bin/python",
                interpreter_path="bin/python",
            ),
        ),
        (
            "root shebang",
            dict(
                command_path="bin/foo",
                command_value="bin/foo bar",
                expected_command="bin/foo bar",
                expected_log=None,
                shebang="/bin/python",
                interpreter_path="/bin/python",
            ),
        ),
        (
            "pathless shebang",
            dict(
                command_path="bin/foo",
                command_value="bin/foo bar",
                expected_command="bin/python $SNAP/bin/foo bar",
                expected_log=None,
                shebang="#!/usr/bin/env python",
                interpreter_path="bin/python",
            ),
        ),
        (
            "pathless shebang with $SNAP leading command",
            dict(
                command_path="bin/foo",
                command_value="$SNAP/bin/foo bar",
                expected_command="bin/python $SNAP/bin/foo bar",
                expected_log="Stripped '$SNAP/' from command '$SNAP/bin/foo bar'.",
                shebang="#!/usr/bin/env python",
                interpreter_path="bin/python",
            ),
        ),
        (
            "with environment variable",
            dict(
                command_path="bin/foo",
                command_value="bin/foo bar $SNAP_DATA",
                expected_command="bin/foo bar $SNAP_DATA",
                expected_log=None,
                shebang="",
                interpreter_path=None,
            ),
        ),
        (
            "single quotes preserved",
            dict(
                command_path="bin/foo",
                command_value="bin/foo bar '$SNAP_DATA'",
                expected_command="bin/foo bar '$SNAP_DATA'",
                expected_log=None,
                shebang="",
                interpreter_path=None,
            ),
        ),
    )

    def test_mangle(
        self,
        caplog,
        tmp_work_path,
        command_path,
        command_value,
        expected_command,
        expected_log,
        shebang,
        interpreter_path,
    ):
        caplog.set_level(logging.INFO)

        if command_path is not None:
            command_path = tmp_work_path / command_path
            command_path.parent.mkdir(parents=True, exist_ok=True)
            with command_path.open("w") as command_file:
                print(shebang, file=command_file)
            command_path.chmod(0o755)

        if interpreter_path is not None and not interpreter_path.startswith("/"):
            interpreter_path = tmp_work_path / interpreter_path
            interpreter_path.parent.mkdir(parents=True, exist_ok=True)
            interpreter_path.touch()
            interpreter_path.chmod(0o755)

        assert (
            command._massage_command(
                command=command_value, prime_dir=tmp_work_path.as_posix()
            )
            == expected_command
        )

        if expected_log is not None:
            assert caplog.records[0].message == expected_log
        else:
            assert len(caplog.records) == 0


def test_find_binary_not_found(tmp_path):
    with pytest.raises(errors.PrimedCommandNotFoundError):
        command._massage_command(command="not-found", prime_dir=tmp_path.as_posix())


def test_binary_not_executable(tmp_work_path):
    exec_path = tmp_work_path / "bin" / "not-executable"
    exec_path.parent.mkdir()
    exec_path.touch()

    with pytest.raises(errors.PrimedCommandNotFoundError):
        command._massage_command(
            command="not-executable", prime_dir=tmp_work_path.as_posix()
        )
