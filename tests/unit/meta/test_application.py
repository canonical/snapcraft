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
import pathlib

from testtools.matchers import Contains, Equals, FileExists, Not

from snapcraft import yaml_utils
from snapcraft.internal.meta import application, desktop, errors
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

        app.write_command_wrappers(prime_dir=self.path)
        self.expectThat("command-foo.wrapper", Not(FileExists()))
        self.expectThat("stop-command-foo.wrapper", Not(FileExists()))

    def test_app_with_wrapper(self):
        app = application.Application.from_dict(
            app_name="foo",
            app_dict={
                "command": "/test-command",
                "stop-command": "/test-stop-command",
                "daemon": "simple",
            },
        )
        app.prime_commands(base="core18", prime_dir=self.path)
        self.assertThat(
            app.to_dict(),
            Equals(
                {
                    "command": "command-foo.wrapper",
                    "stop-command": "stop-command-foo.wrapper",
                    "daemon": "simple",
                }
            ),
        )

        app.write_command_wrappers(prime_dir=self.path)
        self.expectThat("command-foo.wrapper", FileExists())
        self.expectThat("stop-command-foo.wrapper", FileExists())

    def test_massaged_core(self):
        app = application.Application.from_dict(
            app_name="foo", app_dict={"command": "$SNAP/test-command"}
        )
        app.prime_commands(base="core", prime_dir=self.path)

        self.assertThat(app.to_dict(), Equals({"command": "test-command"}))

    def test_massaged_core18(self):
        app = application.Application.from_dict(
            app_name="foo", app_dict={"command": "$SNAP/test-command"}
        )
        app.prime_commands(base="core18", prime_dir=self.path)

        self.assertThat(app.to_dict(), Equals({"command": "test-command"}))

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


class TestWrapperUse:
    scenarios = (
        (
            "wrapper allowed for plain command on core18",
            dict(extra_app_properties={}, base="core18", expect_wrappers=True),
        ),
        (
            "wrapper allowed for plain command on core",
            dict(extra_app_properties={}, base="core", expect_wrappers=True),
        ),
        (
            "wrapper not allowed for not core or core18 base",
            dict(extra_app_properties={}, base="core20", expect_wrappers=False),
        ),
        (
            "wrapper not allowed with command-chain",
            dict(
                extra_app_properties={"command-chain": ["command-chain"]},
                base="core18",
                expect_wrappers=False,
            ),
        ),
        (
            "wrapper not allowed with none adapter",
            dict(
                extra_app_properties={"adapter": "none"},
                base="core18",
                expect_wrappers=False,
            ),
        ),
    )

    def test_wrapper(self, tmp_work_path, extra_app_properties, base, expect_wrappers):
        app_properties = dict(command="foo")
        app_properties.update(extra_app_properties)

        for exe in ["foo"] + app_properties.get("command-chain", list()):
            exe_path = pathlib.Path(exe)
            exe_path.touch()
            exe_path.chmod(0o755)

        app = application.Application.from_dict(app_name="foo", app_dict=app_properties)

        assert app.can_use_wrapper(base=base) == expect_wrappers


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


class DesktopFileTest(unit.TestCase):
    def test_desktop_file(self):
        desktop_file_path = "foo.desktop"
        with open(desktop_file_path, "w") as desktop_file:
            print("[Desktop Entry]", file=desktop_file)
            print("Exec=in-snap-exe", file=desktop_file)
        open("command-chain", "w").close()
        os.chmod("command-chain", 0o755)

        app = application.Application.from_dict(
            app_name="foo", app_dict=dict(command="/foo", desktop=desktop_file_path)
        )

        desktop_file = desktop.DesktopFile(
            snap_name="foo",
            app_name=app.app_name,
            filename=app.desktop,
            prime_dir=self.path,
        )

        desktop_file.write(gui_dir="gui")

        expected_desktop_file_path = os.path.join("gui", "foo.desktop")

        self.expectThat(app.to_dict(), Not(Contains("desktop")))
        self.expectThat(expected_desktop_file_path, FileExists())


class AppPassthroughTests(unit.TestCase):
    def test_no_passthrough(self):
        app = application.Application(
            app_name="foo",
            adapter=application.ApplicationAdapter.NONE,
            command_chain=["test-command-chain"],
            passthrough=None,
        )

        app_dict = app.to_dict()

        self.assertThat(app_dict, Equals({"command-chain": ["test-command-chain"]}))

    def test_passthrough_to_dict(self):
        app = application.Application(
            app_name="foo",
            adapter=application.ApplicationAdapter.NONE,
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
                "adapter": "none",
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
