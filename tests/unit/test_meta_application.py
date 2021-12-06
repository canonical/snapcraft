# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019-2021 Canonical Ltd
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

from testtools.matchers import Equals

from snapcraft import yaml_utils
from snapcraft.meta import application, errors
from tests import unit


class AppCommandTest(unit.TestCase):
    def setUp(self):
        super().setUp()
        for exe in (
            "test-command",
            "test-stop-command",
            "test-command-chain",
            "prepend-command-chain",
        ):
            open(exe, "w").close()
            os.chmod(exe, 0o755)

    def test_app_no_change(self):
        app = application.Application.from_dict(
            app_name="foo",
            app_dict={
                "command": "test-command",
                "stop-command": "test-stop-command",
                "daemon": "simple",
                "install-mode": "disable",
                "command-chain": ["test-command-chain"],
            },
        )

        app.prime_commands(base="core18", prime_dir=self.path)

        self.expectThat(
            app.to_dict(),
            Equals(
                {
                    "command": "test-command",
                    "stop-command": "test-stop-command",
                    "daemon": "simple",
                    "install-mode": "disable",
                    "command-chain": ["test-command-chain"],
                }
            ),
        )

    def test_not_massaged_core20(self):
        app = application.Application.from_dict(
            app_name="foo", app_dict={"command": "$SNAP/test-command"}
        )

        self.assertRaises(
            errors.InvalidAppCommandNotFound,
            app.prime_commands,
            base="core20",
            prime_dir=self.path,
        )

    def test_socket_mode_change_to_octal(self):
        app = application.Application.from_dict(
            app_name="foo",
            app_dict={
                "command": "test-command",
                "daemon": "simple",
                "sockets": {
                    "sock1": {"listen-stream": 8080},
                    "sock2": {
                        "listen-stream": "$SNAP_COMMON/sock2",
                        "socket-mode": 1000,
                    },
                },
            },
        )

        self.expectThat(
            type(app.to_dict()["sockets"]["sock2"]["socket-mode"]),
            Equals(yaml_utils.OctInt),
        )

    def test_no_command_chain(self):
        app = application.Application.from_dict(
            app_name="foo", app_dict={"command": "test-command"}
        )

        app.prime_commands(base="core18", prime_dir=self.path)

        self.assertThat(app.to_dict(), Equals({"command": "test-command"}))


class InvalidCommandChainTest(unit.TestCase):
    def test_command_chain_path_not_found(self):
        app = application.Application.from_dict(
            app_name="foo", app_dict={"command-chain": "file-not-found"}
        )

        self.assertRaises(
            errors.InvalidCommandChainError,
            app.validate_command_chain_executables,
            prime_dir=self.path,
        )

    def test_command_chain_path_not_executable(self):
        open("file-not-executable", "w").close()

        app = application.Application.from_dict(
            app_name="foo", app_dict={"command-chain": "file-not-executable"}
        )

        self.assertRaises(
            errors.InvalidCommandChainError,
            app.validate_command_chain_executables,
            prime_dir=self.path,
        )


class AppPassthroughTests(unit.TestCase):
    def test_no_passthrough(self):
        app = application.Application(
            app_name="foo", command_chain=["test-command-chain"], passthrough=None,
        )

        app_dict = app.to_dict()

        self.assertThat(app_dict, Equals({"command-chain": ["test-command-chain"]}))

    def test_passthrough_to_dict(self):
        app = application.Application(
            app_name="foo",
            command_chain=["test-command-chain"],
            passthrough={"test-property": "test-value"},
        )

        app_dict = app.to_dict()

        self.assertThat(
            app_dict,
            Equals(
                {"command-chain": ["test-command-chain"], "test-property": "test-value"}
            ),
        )

    def test_passthrough_to_dict_from_dict(self):
        app = application.Application.from_dict(
            app_name="foo",
            app_dict={
                "command-chain": ["test-command-chain"],
                "passthrough": {"test-property": "test-value"},
            },
        )

        app_dict = app.to_dict()

        self.assertThat(
            app_dict,
            Equals(
                {"command-chain": ["test-command-chain"], "test-property": "test-value"}
            ),
        )
