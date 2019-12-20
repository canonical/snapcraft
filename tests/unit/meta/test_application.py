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
from testtools.matchers import Contains, Equals, FileContains, FileExists, Not

from snapcraft import yaml_utils
from snapcraft.internal.meta import application, errors, desktop
from tests import unit


def _create_file(file_path: str, *, mode=0o755) -> None:
    os.makedirs(os.path.dirname(file_path), exist_ok=True)
    open(file_path, "w").close()
    os.chmod(file_path, mode)


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
                    "command-chain": ["test-command-chain"],
                }
            ),
        )

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

        self.expectThat("command-foo.wrapper", FileExists())
        self.expectThat("stop-command-foo.wrapper", FileExists())

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

        app.validate()
        self.assertThat(app.to_dict(), Equals({"command": "test-command"}))

    def test_command_chain_none_adapter(self):
        app = application.Application.from_dict(
            app_name="foo",
            app_dict={
                "command": "test-command",
                "command-chain": ["command-chain"],
                "adapter": "none",
            },
        )

        app.prime_commands(base="core18", prime_dir=self.path)

        self.assertRaises(errors.CommandChainWithIncompatibleAdapterError, app.validate)

    def test_command_chain_legacy_adapter(self):
        app = application.Application.from_dict(
            app_name="foo",
            app_dict={
                "command": "test-command",
                "command-chain": ["command-chain"],
                "adapter": "legacy",
            },
        )

        app.prime_commands(base="core18", prime_dir=self.path)

        # This used to be an error condition, but since it's the schema's
        # default, we expect it to behave reasonably. That is,
        # do not error if using command-chain.  Only "none" should error.
        app.validate()
        self.assertThat(
            app.to_dict(),
            Equals({"command": "test-command", "command-chain": ["command-chain"]}),
        )

    def test_command_chain_full_adapter(self):
        app = application.Application.from_dict(
            app_name="foo",
            app_dict={
                "command": "test-command",
                "command-chain": ["command-chain"],
                "adapter": "full",
            },
        )

        app.prime_commands(base="core18", prime_dir=self.path)

        app.validate()
        self.assertThat(
            app.to_dict(),
            Equals({"command": "test-command", "command-chain": ["command-chain"]}),
        )


class WrapperUseTest(unit.TestCase):
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

    def setUp(self):
        super().setUp()

        self.app_properties = dict(command="foo")
        self.app_properties.update(self.extra_app_properties)

        for exe in ["foo"] + self.app_properties.get("command-chain", list()):
            open(exe, "w").close()
            os.chmod(exe, 0o755)

    def test_wrapper(self):
        app = application.Application.from_dict(
            app_name="foo", app_dict=self.app_properties
        )

        app.validate()
        self.assertThat(
            app.can_use_wrapper(base=self.base), Equals(self.expect_wrappers)
        )


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
        app = application.Application(app_name="foo", command="foo")

        app.prime_commands(base=None, prime_dir=self.path)

        self.assertThat(app.command, Equals("foo"))

    def test_command_with_args(self):
        _create_file(os.path.join(self.path, "foo"))
        app = application.Application(app_name="foo", command="foo bar -baz")

        app.prime_commands(base=None, prime_dir=self.path)

        self.assertThat(app.command, Equals("foo bar -baz"))

    def test_massage_bases(self):
        app = application.Application(app_name="foo", command="foo")

        self.assertThat(app.can_massage_commands(base=None), Equals(False))
        self.assertThat(app.can_massage_commands(base="core"), Equals(True))
        self.assertThat(app.can_massage_commands(base="core18"), Equals(True))
        self.assertThat(app.can_massage_commands(base="unknown"), Equals(False))

    def test_can_use_wrapper_bases(self):
        app = application.Application(app_name="foo", command="foo")

        self.assertThat(app.can_use_wrapper(base=None), Equals(False))
        self.assertThat(app.can_use_wrapper(base="core"), Equals(True))
        self.assertThat(app.can_use_wrapper(base="core18"), Equals(True))
        self.assertThat(app.can_use_wrapper(base="unknown"), Equals(False))

    def test_can_use_wrapper_command_chain(self):
        adapter = application.ApplicationAdapter.LEGACY
        app = application.Application(
            app_name="foo", command="foo", command_chain=None, adapter=adapter
        )
        self.assertThat(app.can_use_wrapper(base="core"), Equals(True))

        app.command_chain = ["command", "chain"]
        self.assertThat(app.can_use_wrapper(base="core"), Equals(False))


class CommandWithWrapperTest(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.fake_logger = fixtures.FakeLogger(level=logging.WARNING)
        self.useFixture(self.fake_logger)

    def test_command(self):
        _create_file(os.path.join(self.path, "foo"))
        app = application.Application(app_name="foo", command="foo")

        app.prime_commands(base="core", prime_dir=self.path)

        self.assertThat(app.command, Equals("foo"))
        self.assertThat(self.fake_logger.output, Equals(""))

    def test_command_with_dollar_snap_and_does_not_match_snapd_pattern(self):
        _create_file(os.path.join(self.path, "foo"))
        app = application.Application(app_name="foo", command="$SNAP/foo !option")

        app.prime_commands(base="core", prime_dir=self.path)

        self.assertThat(app.command, Equals("command-foo.wrapper"))
        self.assertThat(
            self.fake_logger.output.strip(),
            Equals(
                "Stripped '$SNAP/' from command '$SNAP/foo !option'."
                "\n"
                "A shell wrapper will be generated for command 'foo !option' "
                "as it does not conform with the command pattern expected "
                "by the runtime. Commands must be relative to the prime "
                "directory and can only consist of alphanumeric characters, "
                "spaces, and the following special characters: / . _ # : $ -"
            ),
        )

    def test_command_with_args(self):
        _create_file(os.path.join(self.path, "foo"))
        app = application.Application(app_name="foo", command="foo bar -baz")

        app.prime_commands(base="core", prime_dir=self.path)

        wrapper_path = os.path.join(self.path, "command-foo.wrapper")

        self.expectThat(app.command, Equals("foo bar -baz"))
        self.expectThat(wrapper_path, Not(FileExists()))
        self.assertThat(self.fake_logger.output, Equals(""))

    def test_command_does_not_match_snapd_pattern(self):
        _create_file(os.path.join(self.path, "foo"))
        app = application.Application(app_name="foo", command="foo /!option")

        app.prime_commands(base="core", prime_dir=self.path)

        wrapper_path = os.path.join(self.path, "command-foo.wrapper")

        self.expectThat(app.command, Equals("command-foo.wrapper"))
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
        app = application.Application(app_name="foo", command="/foo")

        app.prime_commands(base="core", prime_dir=self.path)

        wrapper_path = os.path.join(self.path, "command-foo.wrapper")

        self.expectThat(app.command, Equals("command-foo.wrapper"))
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
        app = application.Application(app_name="foo", command="sh")

        app.prime_commands(base="core", prime_dir=self.path)

        wrapper_path = os.path.join(self.path, "command-foo.wrapper")

        self.expectThat(app.command, Equals("command-foo.wrapper"))
        self.assertThat(wrapper_path, FileExists())
        self.assertThat(wrapper_path, FileContains('#!/bin/sh\nexec /bin/sh "$@"\n'))
        self.assertThat(
            self.fake_logger.output.strip(),
            Equals(
                "The command 'sh' was not found in the prime directory, it has "
                "been changed to '/bin/sh'."
                "\n"
                "A shell wrapper will be generated for command '/bin/sh' "
                "as it does not conform with the command pattern expected "
                "by the runtime. Commands must be relative to the prime "
                "directory and can only consist of alphanumeric characters, "
                "spaces, and the following special characters: / . _ # : $ -"
            ),
        )

    def test_command_not_executable(self):
        _create_file(os.path.join(self.path, "foo"), mode=0o644)
        app = application.Application(app_name="foo", command="foo")

        self.assertRaises(
            errors.InvalidAppCommandNotExecutable,
            app.prime_commands,
            base="core",
            prime_dir=self.path,
        )
        self.assertThat(self.fake_logger.output.strip(), Equals(""))
