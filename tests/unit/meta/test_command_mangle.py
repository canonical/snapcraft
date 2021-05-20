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

from snapcraft.internal.meta import command


class TestCommandMangle:
    scenarios = (
        (
            "no change",
            dict(
                command_path="foo",
                command_value="foo bar",
                expected_command="foo bar",
                expected_logs=[],
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
                expected_logs=[],
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
                expected_logs=[
                    "Found unneeded '$SNAP/' in command '$SNAP/foo bar'.",
                    "The command '$SNAP/foo bar' has been changed to 'foo bar'.",
                ],
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
                expected_logs=[],
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
                expected_logs=[
                    "Found unneeded '$SNAP/' in command '$SNAP/bin/foo bar'.",
                    "The command '$SNAP/bin/foo bar' has been changed to 'bin/foo bar'.",
                ],
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
                expected_logs=[
                    "The command 'foo' for 'foo bar' was resolved to 'bin/foo'.",
                    "The command 'foo bar' has been changed to 'bin/foo bar'.",
                ],
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
                expected_logs=[
                    "The command 'foo' for 'foo bar' was resolved to 'sbin/foo'.",
                    "The command 'foo bar' has been changed to 'sbin/foo bar'.",
                ],
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
                expected_logs=[
                    "The command 'foo' for 'foo bar' was resolved to 'usr/bin/foo'.",
                    "The command 'foo bar' has been changed to 'usr/bin/foo bar'.",
                ],
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
                expected_logs=[
                    "The command 'foo' for 'foo bar' was resolved to 'usr/sbin/foo'.",
                    "The command 'foo bar' has been changed to 'usr/sbin/foo bar'.",
                ],
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
                expected_logs=[
                    "The command 'sh' for 'sh bar' was resolved to '/bin/sh'.",
                    "The command 'sh bar' has been changed to '/bin/sh bar'.",
                ],
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
                expected_logs=[
                    "The command 'foo' for 'foo bar' was resolved to 'bar/foo'.",
                    "The command 'foo bar' has been changed to 'bar/foo bar'.",
                ],
                shebang="",
                interpreter_path=None,
            ),
        ),
        (
            "find in non standard path and preferred over root",
            dict(
                command_path="bar/sh",
                command_value="sh bar",
                expected_command="bar/sh bar",
                expected_logs=[
                    "The command 'sh' for 'sh bar' was resolved to 'bar/sh'.",
                    "The command 'sh bar' has been changed to 'bar/sh bar'.",
                ],
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
                expected_logs=[
                    "The command 'foo bar' has been changed to 'bin/python $SNAP/foo bar' to safely account for the interpreter.",
                ],
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
                expected_logs=[],
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
                expected_logs=[
                    "The interpreter 'python' for 'bin/foo bar' was resolved to 'bin/python'.",
                    "The command 'bin/foo bar' has been changed to 'bin/python $SNAP/bin/foo bar' to safely account for the interpreter.",
                ],
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
                expected_logs=[
                    "The interpreter 'python' for '$SNAP/bin/foo bar' was resolved to 'bin/python'.",
                    "Found unneeded '$SNAP/' in command '$SNAP/bin/foo bar'.",
                    "The command '$SNAP/bin/foo bar' has been changed to 'bin/python $SNAP/bin/foo bar' to safely account for the interpreter.",
                ],
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
                expected_logs=[],
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
                expected_logs=[],
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
        expected_logs,
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
            command._SnapCommandResolver.resolve_snap_command_entry(
                command=command_value, prime_path=tmp_work_path
            )
            == expected_command
        )

        log_messages = [r.message for r in caplog.records]
        assert log_messages == expected_logs


def test_find_binary_not_found(tmp_path):
    assert (
        command._SnapCommandResolver.resolve_snap_command_entry(
            command="not-found", prime_path=tmp_path
        )
        is None
    )


def test_find_ambigious_command_multiple_targets(tmp_work_path, caplog):
    caplog.set_level(logging.INFO)
    command_paths = [tmp_work_path / "bin/xc", tmp_work_path / "usr/bin/xc"]
    for command_path in command_paths:
        command_path.parent.mkdir(parents=True, exist_ok=True)
        command_path.touch()
        command_path.chmod(0o755)

    assert (
        command._SnapCommandResolver.resolve_snap_command_entry(
            command="xc", prime_path=tmp_work_path
        )
        == "bin/xc"
    )

    log_messages = [r.message for r in caplog.records]
    assert log_messages == [
        "Multiple binaries matching for ambiguous command 'xc': ['bin/xc', 'usr/bin/xc']",
        "The command 'xc' for 'xc' was resolved to 'bin/xc'.",
        "The command 'xc' has been changed to 'bin/xc'.",
    ]
