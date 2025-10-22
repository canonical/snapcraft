# Copyright 2025 Canonical Ltd.
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
"""Tests for app-related models."""

from typing import Any

import pydantic
import pytest

from snapcraft.models import Project


@pytest.fixture
def app_yaml_data(project_yaml_data):
    def _app_yaml_data(**kwargs) -> dict[str, Any]:
        data = project_yaml_data()
        data["apps"] = {"app1": {"command": "/bin/true", **kwargs}}
        return data

    yield _app_yaml_data


@pytest.fixture
def socket_yaml_data(app_yaml_data):
    def _socket_yaml_data(**kwargs) -> dict[str, Any]:
        data = app_yaml_data()
        data["apps"]["app1"]["sockets"] = {"socket1": {**kwargs}}
        return data

    yield _socket_yaml_data


class TestAppValidation:
    """Validate apps."""

    def test_app_command(self, app_yaml_data):
        data = app_yaml_data(command="test-command")
        project = Project.unmarshal(data)
        assert project.apps is not None
        assert project.apps["app1"].command == "test-command"

    @pytest.mark.parametrize(
        "autostart",
        ["myapp.desktop", "_invalid"],
    )
    def test_app_autostart(self, autostart, app_yaml_data):
        data = app_yaml_data(autostart=autostart)

        if autostart != "_invalid":
            project = Project.unmarshal(data)
            assert project.apps is not None
            assert project.apps["app1"].autostart == autostart
        else:
            error = (
                "apps.app1.autostart\n  Value error, '_invalid' is not a valid "
                "desktop file name"
            )
            with pytest.raises(pydantic.ValidationError, match=error):
                Project.unmarshal(data)

    def test_app_common_id(self, app_yaml_data):
        data = app_yaml_data(common_id="test-common-id")
        project = Project.unmarshal(data)
        assert project.apps is not None
        assert project.apps["app1"].common_id == "test-common-id"

    def test_app_completer(self, app_yaml_data):
        data = app_yaml_data(completer="test-completer")
        project = Project.unmarshal(data)
        assert project.apps is not None
        assert project.apps["app1"].completer == "test-completer"

    def test_app_stop_command(self, app_yaml_data):
        data = app_yaml_data(stop_command="test-stop-command")
        project = Project.unmarshal(data)
        assert project.apps is not None
        assert project.apps["app1"].stop_command == "test-stop-command"

    def test_app_post_stop_command(self, app_yaml_data):
        data = app_yaml_data(post_stop_command="test-post-stop-command")
        project = Project.unmarshal(data)
        assert project.apps is not None
        assert project.apps["app1"].post_stop_command == "test-post-stop-command"

    @pytest.mark.parametrize(
        "start_timeout", ["10", "10ns", "10us", "10ms", "10s", "10m"]
    )
    def test_app_start_timeout_valid(self, start_timeout, app_yaml_data):
        data = app_yaml_data(start_timeout=start_timeout)
        project = Project.unmarshal(data)
        assert project.apps is not None
        assert project.apps["app1"].start_timeout == start_timeout

    @pytest.mark.parametrize(
        "start_timeout",
        ["10 s", "10 seconds", "1:00", "invalid"],
    )
    def test_app_start_timeout_invalid(self, start_timeout, app_yaml_data):
        data = app_yaml_data(start_timeout=start_timeout)

        error = f"'{start_timeout}' is not a valid time value"
        with pytest.raises(pydantic.ValidationError, match=error):
            Project.unmarshal(data)

    @pytest.mark.parametrize(
        "stop_timeout", ["10", "10ns", "10us", "10ms", "10s", "10m"]
    )
    def test_app_stop_timeout_valid(self, stop_timeout, app_yaml_data):
        data = app_yaml_data(stop_timeout=stop_timeout)
        project = Project.unmarshal(data)
        assert project.apps is not None
        assert project.apps["app1"].stop_timeout == stop_timeout

    @pytest.mark.parametrize(
        "stop_timeout",
        ["10 s", "10 seconds", "1:00", "invalid"],
    )
    def test_app_stop_timeout_invalid(self, stop_timeout, app_yaml_data):
        data = app_yaml_data(stop_timeout=stop_timeout)

        error = f"'{stop_timeout}' is not a valid time value"
        with pytest.raises(pydantic.ValidationError, match=error):
            Project.unmarshal(data)

    @pytest.mark.parametrize(
        "watchdog_timeout", ["10", "10ns", "10us", "10ms", "10s", "10m"]
    )
    def test_app_watchdog_timeout_valid(self, watchdog_timeout, app_yaml_data):
        data = app_yaml_data(watchdog_timeout=watchdog_timeout)
        project = Project.unmarshal(data)
        assert project.apps is not None
        assert project.apps["app1"].watchdog_timeout == watchdog_timeout

    @pytest.mark.parametrize(
        "watchdog_timeout",
        ["10 s", "10 seconds", "1:00", "invalid"],
    )
    def test_app_watchdog_timeout_invalid(self, watchdog_timeout, app_yaml_data):
        data = app_yaml_data(watchdog_timeout=watchdog_timeout)

        error = f"'{watchdog_timeout}' is not a valid time value"
        with pytest.raises(pydantic.ValidationError, match=error):
            Project.unmarshal(data)

    def test_app_reload_command(self, app_yaml_data):
        data = app_yaml_data(reload_command="test-reload-command")
        project = Project.unmarshal(data)
        assert project.apps is not None
        assert project.apps["app1"].reload_command == "test-reload-command"

    @pytest.mark.parametrize(
        "restart_delay", ["10", "10ns", "10us", "10ms", "10s", "10m"]
    )
    def test_app_restart_delay_valid(self, restart_delay, app_yaml_data):
        data = app_yaml_data(restart_delay=restart_delay)
        project = Project.unmarshal(data)
        assert project.apps is not None
        assert project.apps["app1"].restart_delay == restart_delay

    @pytest.mark.parametrize(
        "restart_delay",
        ["10 s", "10 seconds", "1:00", "invalid"],
    )
    def test_app_restart_delay_invalid(self, restart_delay, app_yaml_data):
        data = app_yaml_data(restart_delay=restart_delay)

        error = (
            f"apps.app1.restart_delay\n  Value error, '{restart_delay}' is not a "
            "valid time value"
        )
        with pytest.raises(pydantic.ValidationError, match=error):
            Project.unmarshal(data)

    def test_app_timer(self, app_yaml_data):
        data = app_yaml_data(timer="test-timer")
        project = Project.unmarshal(data)
        assert project.apps is not None
        assert project.apps["app1"].timer == "test-timer"

    @pytest.mark.parametrize(
        "daemon",
        ["simple", "forking", "oneshot", "notify", "dbus", "_invalid"],
    )
    def test_app_daemon(self, daemon, app_yaml_data):
        data = app_yaml_data(daemon=daemon)

        if daemon != "_invalid":
            project = Project.unmarshal(data)
            assert project.apps is not None
            assert project.apps["app1"].daemon == daemon
        else:
            error = "apps.app1.daemon\n  Input should be 'simple', 'forking', 'oneshot', 'notify' or 'dbus'"
            with pytest.raises(pydantic.ValidationError, match=error):
                Project.unmarshal(data)

    @pytest.mark.parametrize(
        "after",
        [
            "i am a string",
            ["i", "am", "a", "list"],
        ],
    )
    def test_app_after(self, after, app_yaml_data):
        data = app_yaml_data(after=after)

        if after == "i am a string":
            error = "apps.app1.after\n  Input should be a valid list"
            with pytest.raises(pydantic.ValidationError, match=error):
                Project.unmarshal(data)
        else:
            project = Project.unmarshal(data)
            assert project.apps is not None
            assert project.apps["app1"].after == after

    def test_app_duplicate_after(self, app_yaml_data):
        data = app_yaml_data(after=["duplicate", "duplicate"])

        error = "apps.app1.after\n  Value error, duplicate values in list"
        with pytest.raises(pydantic.ValidationError, match=error):
            Project.unmarshal(data)

    @pytest.mark.parametrize(
        "before",
        [
            "i am a string",
            ["i", "am", "a", "list"],
        ],
    )
    def test_app_before(self, before, app_yaml_data):
        data = app_yaml_data(before=before)

        if before == "i am a string":
            error = "apps.app1.before\n  Input should be a valid list"
            with pytest.raises(pydantic.ValidationError, match=error):
                Project.unmarshal(data)
        else:
            project = Project.unmarshal(data)
            assert project.apps is not None
            assert project.apps["app1"].before == before

    def test_app_duplicate_before(self, app_yaml_data):
        data = app_yaml_data(before=["duplicate", "duplicate"])

        error = "apps.app1.before\n  Value error, duplicate values in list"
        with pytest.raises(pydantic.ValidationError, match=error):
            Project.unmarshal(data)

    @pytest.mark.parametrize(
        "refresh_mode", ["endure", "restart", "ignore-running", "_invalid"]
    )
    def test_app_refresh_mode(self, refresh_mode, app_yaml_data):
        data = app_yaml_data(refresh_mode=refresh_mode)

        if refresh_mode != "_invalid":
            project = Project.unmarshal(data)
            assert project.apps is not None
            assert project.apps["app1"].refresh_mode == refresh_mode
        else:
            error = (
                "apps.app1.refresh_mode\n  Input should be 'endure', 'restart' "
                "or 'ignore-running'"
            )
            with pytest.raises(pydantic.ValidationError, match=error):
                Project.unmarshal(data)

    @pytest.mark.parametrize(
        "stop_mode",
        [
            "sigterm",
            "sigterm-all",
            "sighup",
            "sighup-all",
            "sigusr1",
            "sigusr1-all",
            "sigusr2",
            "sigusr2-all",
            "sigint",
            "sigint-all",
            "_invalid",
        ],
    )
    def test_app_stop_mode(self, stop_mode, app_yaml_data):
        data = app_yaml_data(stop_mode=stop_mode)

        if stop_mode != "_invalid":
            project = Project.unmarshal(data)
            assert project.apps is not None
            assert project.apps["app1"].stop_mode == stop_mode
        else:
            error = (
                "apps.app1.stop_mode\n  Input should be 'sigterm', 'sigterm-all', "
                "'sighup', 'sighup-all', 'sigusr1', 'sigusr1-all', "
                "'sigusr2', 'sigusr2-all', 'sigint' or 'sigint-all'"
            )
            with pytest.raises(pydantic.ValidationError, match=error):
                Project.unmarshal(data)

    @pytest.mark.parametrize(
        "restart_condition",
        [
            "on-success",
            "on-failure",
            "on-abnormal",
            "on-abort",
            "on-watchdog",
            "always",
            "never",
            "_invalid",
        ],
    )
    def test_app_restart_condition(self, restart_condition, app_yaml_data):
        data = app_yaml_data(restart_condition=restart_condition)

        if restart_condition != "_invalid":
            project = Project.unmarshal(data)
            assert project.apps is not None
            assert project.apps["app1"].restart_condition == restart_condition
        else:
            error = (
                "apps.app1.restart_condition\n  Input should be 'on-success', "
                "'on-failure', 'on-abnormal', 'on-abort', 'on-watchdog', "
                "'always' or 'never'"
            )
            with pytest.raises(pydantic.ValidationError, match=error):
                Project.unmarshal(data)

    @pytest.mark.parametrize("install_mode", ["enable", "disable", "_invalid"])
    def test_app_install_mode(self, install_mode, app_yaml_data):
        data = app_yaml_data(install_mode=install_mode)

        if install_mode != "_invalid":
            project = Project.unmarshal(data)
            assert project.apps is not None
            assert project.apps["app1"].install_mode == install_mode
        else:
            error = "apps.app1.install_mode\n  Input should be 'enable' or 'disable'"
            with pytest.raises(pydantic.ValidationError, match=error):
                Project.unmarshal(data)

    def test_app_valid_aliases(self, app_yaml_data):
        data = app_yaml_data(aliases=["i", "am", "a", "list"])

        project = Project.unmarshal(data)
        assert project.apps is not None
        assert project.apps["app1"].aliases == ["i", "am", "a", "list"]

    @pytest.mark.parametrize(
        "aliases",
        [
            "i am a string",
            ["_invalid!"],
        ],
    )
    def test_app_invalid_aliases(self, aliases, app_yaml_data):
        data = app_yaml_data(aliases=aliases)

        if isinstance(aliases, list):
            error = (
                f"apps.app1.aliases\n  Value error, '{aliases[0]}' is not a valid alias"
            )
            with pytest.raises(pydantic.ValidationError, match=error):
                Project.unmarshal(data)
        else:
            error = "apps.app1.aliases\n  Input should be a valid list"
            with pytest.raises(pydantic.ValidationError, match=error):
                Project.unmarshal(data)

    def test_app_duplicate_aliases(self, app_yaml_data):
        data = app_yaml_data(aliases=["duplicate", "duplicate"])

        error = "apps.app1.aliases\n  Value error, duplicate values in list"
        with pytest.raises(pydantic.ValidationError, match=error):
            Project.unmarshal(data)

    @pytest.mark.parametrize(
        "environment",
        [
            {"SINGLE_VARIABLE": "foo"},
            {"FIRST_VARIABLE": "foo", "SECOND_VARIABLE": "bar"},
        ],
    )
    def test_app_environment_valid(self, environment, app_yaml_data):
        data = app_yaml_data(environment=environment)
        project = Project.unmarshal(data)
        assert project.apps is not None
        for variable in environment:
            assert variable in project.apps["app1"].environment

    @pytest.mark.parametrize(
        "environment",
        [
            "i am a string",
            ["i", "am", "a", "list"],
            [{"i": "am"}, {"a": "list"}, {"of": "dictionaries"}],
        ],
    )
    def test_app_environment_invalid(self, environment, app_yaml_data):
        data = app_yaml_data(environment=environment)

        error = "apps.app1.environment\n  Input should be a valid dictionary"
        with pytest.raises(pydantic.ValidationError, match=error):
            Project.unmarshal(data)

    @pytest.mark.parametrize(
        "command_chain",
        [
            "i am a string",
            ["_invalid!"],
            ["snap/command-chain/snapcraft-runner"],
            ["i", "am", "a", "list"],
        ],
    )
    def test_app_command_chain(self, command_chain, app_yaml_data):
        data = app_yaml_data(command_chain=command_chain)

        if command_chain == "i am a string":
            error = "apps.app1.command_chain\n  Input should be a valid list"
            with pytest.raises(pydantic.ValidationError, match=error):
                Project.unmarshal(data)
        elif command_chain == ["_invalid!"]:
            error = (
                f"apps.app1.command_chain\n  Value error, '{command_chain[0]}' is not a "
                "valid command chain"
            )
            with pytest.raises(pydantic.ValidationError, match=error):
                Project.unmarshal(data)
        else:
            project = Project.unmarshal(data)
            assert project.apps is not None
            assert project.apps["app1"].command_chain == command_chain

    @pytest.mark.parametrize(
        "listen_stream", [1, 100, 65535, "/tmp/mysocket.sock", "@snap.foo"]
    )
    def test_app_sockets_valid_listen_stream(self, listen_stream, socket_yaml_data):
        data = socket_yaml_data(listen_stream=listen_stream)

        project = Project.unmarshal(data)
        assert project.apps is not None
        assert project.apps["app1"].sockets is not None
        assert project.apps["app1"].sockets["socket1"].listen_stream == listen_stream

    @pytest.mark.parametrize("listen_stream", [-1, 0, 65536])
    def test_app_sockets_invalid_int_listen_stream(
        self, listen_stream, socket_yaml_data
    ):
        data = socket_yaml_data(listen_stream=listen_stream)

        error = (
            f"apps.app1.sockets.socket1.listen_stream\n  Value error, {listen_stream} is not an "
            "integer between 1 and 65535"
        )
        with pytest.raises(pydantic.ValidationError, match=error):
            Project.unmarshal(data)

    @pytest.mark.parametrize("listen_stream", ["@foo"])
    def test_app_sockets_invalid_socket_listen_stream(
        self, listen_stream, socket_yaml_data
    ):
        data = socket_yaml_data(listen_stream=listen_stream)

        error = (
            f"apps.app1.sockets.socket1.listen_stream\n  Value error, {listen_stream!r} is not a "
            "valid socket path"
        )
        with pytest.raises(pydantic.ValidationError, match=error):
            Project.unmarshal(data)

    def test_app_sockets_missing_listen_stream(self, socket_yaml_data):
        data = socket_yaml_data()

        error = "apps.app1.sockets.socket1.listen-stream\n  Field required"
        with pytest.raises(pydantic.ValidationError, match=error):
            Project.unmarshal(data)

    @pytest.mark.parametrize(
        "key",
        [
            "command",
            "stop_command",
            "post_stop_command",
            "reload_command",
            "bus_name",
        ],
    )
    def test_app_command_lexicon_good(
        self,
        app_yaml_data,
        key: str,
    ):
        """Verify that command validation lets in a valid command."""
        command = {key: "mkbird --chirps 5"}
        data = app_yaml_data(**command)
        proj = Project.unmarshal(data)

        # Ensure the happy path
        assert proj.apps is not None
        assert getattr(proj.apps["app1"], key) == "mkbird --chirps 5"

    @pytest.mark.parametrize(
        "key",
        [
            "command",
            "stop_command",
            "post_stop_command",
            "reload_command",
            "bus_name",
        ],
    )
    @pytest.mark.parametrize(
        "value",
        [
            pytest.param(
                "bin/mkbird --chirps=5",
                id="has_bad_char",
            ),
            pytest.param('mkbird --chirps=1337 --name="81U3J@Y"', id="many_bad"),
        ],
    )
    def test_app_command_lexicon_bad(self, app_yaml_data, key: str, value: str):
        """Verify that invalid characters in command fields raise an error."""
        command = {key: value}
        data = app_yaml_data(**command)

        err_msg = "App commands must consist of only alphanumeric characters, spaces, and the following characters: / . _ # : $ -"

        with pytest.raises(pydantic.ValidationError) as val_err:
            Project.unmarshal(data)

        assert err_msg in str(val_err.value)

    @pytest.mark.parametrize("socket_mode", [1, "_invalid"])
    def test_app_sockets_valid_socket_mode(self, socket_mode, socket_yaml_data):
        data = socket_yaml_data(listen_stream="test", socket_mode=socket_mode)

        if socket_mode != "_invalid":
            project = Project.unmarshal(data)
            assert project.apps is not None
            assert project.apps["app1"].sockets is not None
            assert project.apps["app1"].sockets["socket1"].socket_mode == socket_mode
        else:
            error = "apps.app1.sockets.socket1.socket_mode\n  Input should be a valid integer"
            with pytest.raises(pydantic.ValidationError, match=error):
                Project.unmarshal(data)
