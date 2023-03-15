# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022-2023 Canonical Ltd.
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

from typing import Any, Dict

import pydantic
import pytest

from snapcraft import errors
from snapcraft.projects import (
    MANDATORY_ADOPTABLE_FIELDS,
    Architecture,
    ContentPlug,
    GrammarAwareProject,
    Hook,
    Project,
)
from snapcraft.utils import get_host_architecture


@pytest.fixture
def project_yaml_data():
    def _project_yaml_data(
        *, name: str = "name", version: str = "0.1", summary: str = "summary", **kwargs
    ) -> Dict[str, Any]:
        return {
            "name": name,
            "version": version,
            "base": "core22",
            "summary": summary,
            "description": "description",
            "grade": "stable",
            "confinement": "strict",
            "parts": {},
            **kwargs,
        }

    yield _project_yaml_data


@pytest.fixture
def app_yaml_data(project_yaml_data):
    def _app_yaml_data(**kwargs) -> Dict[str, Any]:
        data = project_yaml_data()
        data["apps"] = {"app1": {"command": "/bin/true", **kwargs}}
        return data

    yield _app_yaml_data


@pytest.fixture
def socket_yaml_data(app_yaml_data):
    def _socket_yaml_data(**kwargs) -> Dict[str, Any]:
        data = app_yaml_data()
        data["apps"]["app1"]["sockets"] = {"socket1": {**kwargs}}
        return data

    yield _socket_yaml_data


class TestProjectDefaults:
    """Ensure unspecified items have the correct default value."""

    def test_project_defaults(self, project_yaml_data):
        project = Project.unmarshal(project_yaml_data())

        assert project.build_base == project.base
        assert project.compression == "xz"
        assert project.contact is None
        assert project.donation is None
        assert project.issues is None
        assert project.source_code is None
        assert project.website is None
        assert project.type is None
        assert project.icon is None
        assert project.layout is None
        assert project.license is None
        assert project.package_repositories == []
        assert project.assumes == []
        assert project.hooks is None
        assert project.passthrough is None
        assert project.apps is None
        assert project.plugs is None
        assert project.slots is None
        assert project.epoch is None
        assert project.environment is None
        assert project.adopt_info is None
        assert project.architectures == [
            Architecture(
                build_on=[get_host_architecture()], build_for=[get_host_architecture()]
            )
        ]
        assert project.ua_services is None
        assert project.system_usernames is None
        assert project.provenance is None

    def test_app_defaults(self, project_yaml_data):
        data = project_yaml_data(apps={"app1": {"command": "/bin/true"}})
        project = Project.unmarshal(data)
        assert project.apps is not None

        app = project.apps["app1"]
        assert app is not None

        assert app.command == "/bin/true"
        assert app.autostart is None
        assert app.common_id is None
        assert app.bus_name is None
        assert app.completer is None
        assert app.stop_command is None
        assert app.post_stop_command is None
        assert app.start_timeout is None
        assert app.stop_timeout is None
        assert app.watchdog_timeout is None
        assert app.reload_command is None
        assert app.restart_delay is None
        assert app.timer is None
        assert app.daemon is None
        assert app.after == []
        assert app.before == []
        assert app.refresh_mode is None
        assert app.stop_mode is None
        assert app.restart_condition is None
        assert app.install_mode is None
        assert app.slots is None
        assert app.plugs is None
        assert app.aliases is None
        assert app.environment is None
        assert app.command_chain == []


class TestProjectValidation:
    """Validate top-level project items."""

    @pytest.mark.parametrize("field", ["name", "confinement", "parts"])
    def test_mandatory_fields(self, field, project_yaml_data):
        data = project_yaml_data()
        data.pop(field)
        error = f"field {field!r} required in top-level configuration"
        with pytest.raises(errors.ProjectValidationError, match=error):
            Project.unmarshal(data)

    @pytest.mark.parametrize(
        "snap_type,requires_base",
        [
            ("app", True),
            ("gadget", True),
            ("base", False),
            ("kernel", False),
            ("snapd", False),
        ],
    )
    def test_mandatory_base(self, snap_type, requires_base, project_yaml_data):
        data = project_yaml_data(type=snap_type)
        data.pop("base")

        if requires_base:
            error = "Snap base must be declared when type is not"
            with pytest.raises(errors.ProjectValidationError, match=error):
                Project.unmarshal(data)
        else:
            project = Project.unmarshal(data)
            assert project.base is None

    def test_mandatory_adoptable_fields_definition(self):
        assert MANDATORY_ADOPTABLE_FIELDS == (
            "version",
            "summary",
            "description",
        )

    @pytest.mark.parametrize("field", MANDATORY_ADOPTABLE_FIELDS)
    def test_adoptable_fields(self, field, project_yaml_data):
        data = project_yaml_data()
        data.pop(field)
        error = f"Snap {field} is required if not using adopt-info"
        with pytest.raises(errors.ProjectValidationError, match=error):
            Project.unmarshal(data)

    @pytest.mark.parametrize("field", MANDATORY_ADOPTABLE_FIELDS)
    def test_adoptable_field_not_required(self, field, project_yaml_data):
        data = project_yaml_data()
        data.pop(field)
        data["adopt-info"] = "part1"
        project = Project.unmarshal(data)
        assert getattr(project, field) is None

    @pytest.mark.parametrize("field", MANDATORY_ADOPTABLE_FIELDS)
    def test_adoptable_field_assignment(self, field, project_yaml_data):
        data = project_yaml_data()
        project = Project.unmarshal(data)
        setattr(project, field, None)

    @pytest.mark.parametrize(
        "name",
        [
            "name",
            "name-with-dashes",
            "name0123",
            "0123name",
            "a234567890123456789012345678901234567890",
        ],
    )
    def test_project_name_valid(self, name, project_yaml_data):
        project = Project.unmarshal(project_yaml_data(name=name))
        assert project.name == name

    @pytest.mark.parametrize(
        "name,error",
        [
            ("name_with_underscores", "Snap names can only use"),
            ("name-with-UPPERCASE", "Snap names can only use"),
            ("name with spaces", "Snap names can only use"),
            ("-name-starts-with-hyphen", "Snap names cannot start with a hyphen"),
            ("name-ends-with-hyphen-", "Snap names cannot end with a hyphen"),
            ("name-has--two-hyphens", "Snap names cannot have two hyphens in a row"),
            ("123456", "Snap names can only use"),
            (
                "a2345678901234567890123456789012345678901",
                "ensure this value has at most 40 characters",
            ),
        ],
    )
    def test_project_name_invalid(self, name, error, project_yaml_data):
        with pytest.raises(errors.ProjectValidationError, match=error):
            Project.unmarshal(project_yaml_data(name=name))

    @pytest.mark.parametrize(
        "version",
        [
            "1",
            "1.0",
            "1.0.1-5.2~build0.20.04:1+1A",
            "git",
            "1~",
            "1+",
            "12345678901234567890123456789012",
        ],
    )
    def test_project_version_valid(self, version, project_yaml_data):
        project = Project.unmarshal(project_yaml_data(version=version))
        assert project.version == version

    @pytest.mark.parametrize(
        "version,error",
        [
            ("1_0", "Snap versions consist of"),  # _ is an invalid character
            ("1=1", "Snap versions consist of"),  # = is an invalid character
            (".1", "Snap versions consist of"),  # cannot start with period
            (":1", "Snap versions consist of"),  # cannot start with colon
            ("+1", "Snap versions consist of"),  # cannot start with plus sign
            ("~1", "Snap versions consist of"),  # cannot start with tilde
            ("-1", "Snap versions consist of"),  # cannot start with hyphen
            ("1.", "Snap versions consist of"),  # cannot end with period
            ("1:", "Snap versions consist of"),  # cannot end with colon
            ("1-", "Snap versions consist of"),  # cannot end with hyphen
            (
                "123456789012345678901234567890123",
                "ensure this value has at most 32 characters",
            ),  # too large
        ],
    )
    def test_project_version_invalid(self, version, error, project_yaml_data):
        with pytest.raises(errors.ProjectValidationError, match=error):
            Project.unmarshal(project_yaml_data(version=version))

    @pytest.mark.parametrize(
        "snap_type",
        ["app", "gadget", "kernel", "snapd", "base", "_invalid"],
    )
    def test_project_type(self, snap_type, project_yaml_data):
        data = project_yaml_data(type=snap_type)
        if snap_type in ["base", "kernel", "snapd"]:
            data.pop("base")

        if snap_type != "_invalid":
            project = Project.unmarshal(data)
            assert project.type == snap_type
        else:
            error = ".*unexpected value; permitted: 'app', 'base', 'gadget', 'kernel', 'snapd'"
            with pytest.raises(errors.ProjectValidationError, match=error):
                Project.unmarshal(data)

    @pytest.mark.parametrize(
        "confinement", ["strict", "devmode", "classic", "_invalid"]
    )
    def test_project_confinement(self, confinement, project_yaml_data):
        data = project_yaml_data(confinement=confinement)

        if confinement != "_invalid":
            project = Project.unmarshal(data)
            assert project.confinement == confinement
        else:
            error = ".*unexpected value; permitted: 'classic', 'devmode', 'strict'"
            with pytest.raises(errors.ProjectValidationError, match=error):
                Project.unmarshal(data)

    @pytest.mark.parametrize("grade", ["devel", "stable", "_invalid"])
    def test_project_grade(self, grade, project_yaml_data):
        data = project_yaml_data(grade=grade)

        if grade != "_invalid":
            project = Project.unmarshal(data)
            assert project.grade == grade
        else:
            error = ".*unexpected value; permitted: 'stable', 'devel'"
            with pytest.raises(errors.ProjectValidationError, match=error):
                Project.unmarshal(data)

    @pytest.mark.parametrize("grade", ["devel", "stable", "_invalid"])
    def test_project_grade_assignment(self, grade, project_yaml_data):
        data = project_yaml_data()

        project = Project.unmarshal(data)
        if grade != "_invalid":
            project.grade = grade
        else:
            error = ".*unexpected value; permitted: 'stable', 'devel'"
            with pytest.raises(pydantic.ValidationError, match=error):
                project.grade = grade

    def test_project_summary_valid(self, project_yaml_data):
        summary = "x" * 78
        project = Project.unmarshal(project_yaml_data(summary=summary))
        assert project.summary == summary

    def test_project_summary_invalid(self, project_yaml_data):
        summary = "x" * 79
        error = "ensure this value has at most 78 characters"
        with pytest.raises(errors.ProjectValidationError, match=error):
            Project.unmarshal(project_yaml_data(summary=summary))

    @pytest.mark.parametrize(
        "epoch",
        [
            "0",
            "1",
            "1*",
            "12345",
            "12345*",
        ],
    )
    def test_project_epoch_valid(self, epoch, project_yaml_data):
        project = Project.unmarshal(project_yaml_data(epoch=epoch))
        assert project.epoch == epoch

    @pytest.mark.parametrize(
        "epoch",
        [
            "",
            "invalid",
            "0*",
            "012345",
            "-1",
            "*1",
            "1**",
        ],
    )
    def test_project_epoch_invalid(self, epoch, project_yaml_data):
        error = "Epoch is a positive integer followed by an optional asterisk"
        with pytest.raises(errors.ProjectValidationError, match=error):
            Project.unmarshal(project_yaml_data(epoch=epoch))

    def test_project_package_repository(self, project_yaml_data):
        repos = [
            {
                "type": "apt",
                "ppa": "test/somerepo",
            },
            {
                "type": "apt",
                "url": "https://some/url",
                "key-id": "ABCDE12345" * 4,
            },
        ]
        project = Project.unmarshal(project_yaml_data(package_repositories=repos))
        assert project.package_repositories == repos

    def test_project_package_repository_missing_fields(self, project_yaml_data):
        repos = [
            {
                "type": "apt",
            },
        ]
        error = r".*- field 'url' required .*\n- field 'key-id' required"
        with pytest.raises(errors.ProjectValidationError, match=error):
            Project.unmarshal(project_yaml_data(package_repositories=repos))

    def test_project_package_repository_extra_fields(self, project_yaml_data):
        repos = [
            {
                "type": "apt",
                "extra": "something",
            },
        ]
        error = r".*- extra field 'extra' not permitted"
        with pytest.raises(errors.ProjectValidationError, match=error):
            Project.unmarshal(project_yaml_data(package_repositories=repos))

    @pytest.mark.parametrize(
        "environment",
        [
            {"SINGLE_VARIABLE": "foo"},
            {"FIRST_VARIABLE": "foo", "SECOND_VARIABLE": "bar"},
        ],
    )
    def test_project_environment_valid(self, environment, project_yaml_data):
        project = Project.unmarshal(project_yaml_data(environment=environment))
        for variable in environment:
            assert variable in project.environment

    @pytest.mark.parametrize(
        "environment",
        [
            "i am a string",
            ["i", "am", "a", "list"],
            [{"i": "am"}, {"a": "list"}, {"of": "dictionaries"}],
        ],
    )
    def test_project_environment_invalid(self, environment, project_yaml_data):
        error = ".*value is not a valid dict"
        with pytest.raises(errors.ProjectValidationError, match=error):
            Project.unmarshal(project_yaml_data(environment=environment))

    @pytest.mark.parametrize(
        "plugs",
        [
            {"empty-plug": None},
            {"string-plug": "home"},
            {"dict-plug": {"string-parameter": "foo", "bool-parameter": True}},
        ],
    )
    def test_project_plugs_valid(self, plugs, project_yaml_data):
        project = Project.unmarshal(project_yaml_data(plugs=plugs))
        assert project.plugs == plugs

    @pytest.mark.parametrize(
        "plugs",
        [
            "i am a string",
            ["i", "am", "a", "list"],
            [{"i": "am"}, {"a": "list"}, {"of": "dictionaries"}],
        ],
    )
    def test_project_plugs_invalid(self, plugs, project_yaml_data):
        error = ".*value is not a valid dict"
        with pytest.raises(errors.ProjectValidationError, match=error):
            Project.unmarshal(project_yaml_data(plugs=plugs))

    def test_project_content_plugs_valid(self, project_yaml_data):
        content_plug_data = {
            "content-interface": {
                "interface": "content",
                "target": "test-target",
                "content": "test-content",
                "default-provider": "test-provider",
            }
        }
        content_plug = ContentPlug(**content_plug_data["content-interface"])

        project = Project.unmarshal(project_yaml_data(plugs=content_plug_data))
        assert project.plugs is not None
        assert project.plugs["content-interface"] == content_plug

    def test_project_content_plugs_missing_target(self, project_yaml_data):
        content_plug = {
            "content-interface": {
                "interface": "content",
                "content": "test-content",
                "default-provider": "test-provider",
            }
        }
        error = ".*'content-interface' must have a 'target' parameter"

        with pytest.raises(errors.ProjectValidationError, match=error):
            Project.unmarshal(project_yaml_data(plugs=content_plug))

    def test_project_get_content_snaps(self, project_yaml_data):
        content_plug_data = {
            "content-interface": {
                "interface": "content",
                "target": "test-target",
                "content": "test-content",
                "default-provider": "test-provider",
            }
        }

        project = Project.unmarshal(project_yaml_data(plugs=content_plug_data))
        assert project.get_content_snaps() == ["test-provider"]

    @pytest.mark.parametrize("decl_type", ["symlink", "bind", "bind-file", "type"])
    def test_project_layout(self, decl_type, project_yaml_data):
        project = Project.unmarshal(
            project_yaml_data(layout={"foo": {decl_type: "bar"}})
        )
        assert project.layout is not None
        assert project.layout["foo"][decl_type] == "bar"

    def test_project_layout_invalid(self, project_yaml_data):
        error = (
            "Bad snapcraft.yaml content:\n"
            "- unexpected value; permitted: 'symlink', 'bind', 'bind-file', 'type'"
        )
        with pytest.raises(errors.ProjectValidationError, match=error):
            Project.unmarshal(project_yaml_data(layout={"foo": {"invalid": "bar"}}))

    @pytest.mark.parametrize(
        "slots",
        [
            {"test-slot": {"interface": "some-value"}},
            {
                "db-socket": {
                    "interface": "content",
                    "content": "db-socket",
                    "write": ["$SNAP_COMMON/postgres/sockets"],
                },
            },
        ],
    )
    def test_slot_valid(self, slots, project_yaml_data):
        project = Project.unmarshal(project_yaml_data(slots=slots))
        assert project.slots == slots

    def test_project_build_base_devel_grade_devel(self, project_yaml_data):
        """When build_base is `devel`, the grade must be `devel`."""
        project = Project.unmarshal(
            project_yaml_data(build_base="devel", grade="devel")
        )

        assert project.grade == "devel"

    @pytest.mark.parametrize("build_base", {"core22", "devel"})
    def test_project_grade_not_defined(self, build_base, project_yaml_data):
        """Do not validate the grade if it is not defined, regardless of build_base."""
        data = project_yaml_data(build_base=build_base)
        data.pop("grade")

        project = Project.unmarshal(data)

        assert project.build_base == build_base
        assert not project.grade

    def test_project_build_base_devel_grade_stable_error(self, project_yaml_data):
        """Raise an error if build_base is `devel` and grade is `stable`."""
        error = (
            "Bad snapcraft.yaml content:\n"
            "- grade must be 'devel' when build-base is 'devel'"
        )

        with pytest.raises(errors.ProjectValidationError, match=error):
            Project.unmarshal(project_yaml_data(build_base="devel", grade="stable"))


class TestHookValidation:
    """Validate hooks."""

    @pytest.mark.parametrize(
        "hooks",
        [
            {"configure": {}},
            {
                "configure": {
                    "command-chain": ["test-1", "test-2"],
                    "environment": {
                        "FIRST_VARIABLE": "test-3",
                        "SECOND_VARIABLE": "test-4",
                    },
                    "plugs": ["home", "network"],
                }
            },
        ],
    )
    def test_project_hooks_valid(self, hooks, project_yaml_data):
        configure_hook_data = Hook(**hooks["configure"])
        project = Project.unmarshal(project_yaml_data(hooks=hooks))

        assert project.hooks is not None
        assert project.hooks["configure"] == configure_hook_data

    def test_project_hooks_command_chain_invalid(self, project_yaml_data):
        hook = {"configure": {"command-chain": ["_invalid!"]}}
        error = "'_invalid!' is not a valid command chain"

        with pytest.raises(errors.ProjectValidationError, match=error):
            Project.unmarshal(project_yaml_data(hooks=hook))

    @pytest.mark.parametrize(
        "environment",
        [
            "i am a string",
            ["i", "am", "a", "list"],
            [{"i": "am"}, {"a": "list"}, {"of": "dictionaries"}],
        ],
    )
    def test_project_hooks_environment_invalid(self, environment, project_yaml_data):
        hooks = {"configure": {"environment": environment}}

        error = ".*value is not a valid dict"
        with pytest.raises(errors.ProjectValidationError, match=error):
            Project.unmarshal(project_yaml_data(hooks=hooks))

    def test_project_hooks_plugs_empty(self, project_yaml_data):
        hook = {"configure": {"plugs": []}}
        error = ".*'plugs' field cannot be empty"

        with pytest.raises(errors.ProjectValidationError, match=error):
            Project.unmarshal(project_yaml_data(hooks=hook))


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
            error = ".*'_invalid' is not a valid desktop file name"
            with pytest.raises(errors.ProjectValidationError, match=error):
                Project.unmarshal(data)

    def test_app_common_id(self, app_yaml_data):
        data = app_yaml_data(common_id="test-common-id")
        project = Project.unmarshal(data)
        assert project.apps is not None
        assert project.apps["app1"].common_id == "test-common-id"

    @pytest.mark.parametrize(
        "bus_name",
        ["test-bus-name", "_invalid!"],
    )
    def test_app_bus_name(self, bus_name, app_yaml_data):
        data = app_yaml_data(bus_name=bus_name)

        if bus_name != "_invalid!":
            project = Project.unmarshal(data)
            assert project.apps is not None
            assert project.apps["app1"].bus_name == bus_name
        else:
            error = ".*'_invalid!' is not a valid bus name"
            with pytest.raises(errors.ProjectValidationError, match=error):
                Project.unmarshal(data)

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

        error = f".*'{start_timeout}' is not a valid time value"
        with pytest.raises(errors.ProjectValidationError, match=error):
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

        error = f".*'{stop_timeout}' is not a valid time value"
        with pytest.raises(errors.ProjectValidationError, match=error):
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

        error = f".*'{watchdog_timeout}' is not a valid time value"
        with pytest.raises(errors.ProjectValidationError, match=error):
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

        error = f".*'{restart_delay}' is not a valid time value"
        with pytest.raises(errors.ProjectValidationError, match=error):
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
            error = ".*unexpected value; permitted: 'simple', 'forking', 'oneshot'"
            with pytest.raises(errors.ProjectValidationError, match=error):
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
            error = ".*value is not a valid list"
            with pytest.raises(errors.ProjectValidationError, match=error):
                Project.unmarshal(data)
        else:
            project = Project.unmarshal(data)
            assert project.apps is not None
            assert project.apps["app1"].after == after

    def test_app_duplicate_after(self, app_yaml_data):
        data = app_yaml_data(after=["duplicate", "duplicate"])

        error = ".*duplicate entries in 'after' not permitted"
        with pytest.raises(errors.ProjectValidationError, match=error):
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
            error = ".*value is not a valid list"
            with pytest.raises(errors.ProjectValidationError, match=error):
                Project.unmarshal(data)
        else:
            project = Project.unmarshal(data)
            assert project.apps is not None
            assert project.apps["app1"].before == before

    def test_app_duplicate_before(self, app_yaml_data):
        data = app_yaml_data(before=["duplicate", "duplicate"])

        error = ".*duplicate entries in 'before' not permitted"
        with pytest.raises(errors.ProjectValidationError, match=error):
            Project.unmarshal(data)

    @pytest.mark.parametrize("refresh_mode", ["endure", "restart", "_invalid"])
    def test_app_refresh_mode(self, refresh_mode, app_yaml_data):
        data = app_yaml_data(refresh_mode=refresh_mode)

        if refresh_mode != "_invalid":
            project = Project.unmarshal(data)
            assert project.apps is not None
            assert project.apps["app1"].refresh_mode == refresh_mode
        else:
            error = ".*unexpected value; permitted: 'endure', 'restart'"
            with pytest.raises(errors.ProjectValidationError, match=error):
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
            error = ".*unexpected value; permitted: 'sigterm', 'sigterm-all', 'sighup'"
            with pytest.raises(errors.ProjectValidationError, match=error):
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
            error = ".*unexpected value; permitted: 'on-success', 'on-failure', 'on-abnormal'"
            with pytest.raises(errors.ProjectValidationError, match=error):
                Project.unmarshal(data)

    @pytest.mark.parametrize("install_mode", ["enable", "disable", "_invalid"])
    def test_app_install_mode(self, install_mode, app_yaml_data):
        data = app_yaml_data(install_mode=install_mode)

        if install_mode != "_invalid":
            project = Project.unmarshal(data)
            assert project.apps is not None
            assert project.apps["app1"].install_mode == install_mode
        else:
            error = ".*unexpected value; permitted: 'enable', 'disable'"
            with pytest.raises(errors.ProjectValidationError, match=error):
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
            error = f".*'{aliases[0]}' is not a valid alias"
            with pytest.raises(errors.ProjectValidationError, match=error):
                Project.unmarshal(data)
        else:
            error = ".*value is not a valid list"
            with pytest.raises(errors.ProjectValidationError, match=error):
                Project.unmarshal(data)

    def test_app_duplicate_aliases(self, app_yaml_data):
        data = app_yaml_data(aliases=["duplicate", "duplicate"])

        error = ".*duplicate entries in 'aliases' not permitted"
        with pytest.raises(errors.ProjectValidationError, match=error):
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

        error = ".*value is not a valid dict"
        with pytest.raises(errors.ProjectValidationError, match=error):
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
            error = ".*value is not a valid list"
            with pytest.raises(errors.ProjectValidationError, match=error):
                Project.unmarshal(data)
        elif command_chain == ["_invalid!"]:
            error = f".*'{command_chain[0]}' is not a valid command chain"
            with pytest.raises(errors.ProjectValidationError, match=error):
                Project.unmarshal(data)
        else:
            project = Project.unmarshal(data)
            assert project.apps is not None
            assert project.apps["app1"].command_chain == command_chain

    @pytest.mark.parametrize("listen_stream", [1, 100, 65535, "/tmp/mysocket.sock"])
    def test_app_sockets_valid_listen_stream(self, listen_stream, socket_yaml_data):
        data = socket_yaml_data(listen_stream=listen_stream)

        project = Project.unmarshal(data)
        assert project.apps is not None
        assert project.apps["app1"].sockets is not None
        assert project.apps["app1"].sockets["socket1"].listen_stream == listen_stream

    @pytest.mark.parametrize("listen_stream", [-1, 0, 65536])
    def test_app_sockets_invalid_listen_stream(self, listen_stream, socket_yaml_data):
        data = socket_yaml_data(listen_stream=listen_stream)

        error = f".*{listen_stream} is not an integer between 1 and 65535"
        with pytest.raises(errors.ProjectValidationError, match=error):
            Project.unmarshal(data)

    def test_app_sockets_missing_listen_stream(self, socket_yaml_data):
        data = socket_yaml_data()

        error = ".*field 'listen-stream' required"
        with pytest.raises(errors.ProjectValidationError, match=error):
            Project.unmarshal(data)

    @pytest.mark.parametrize("socket_mode", [1, "_invalid"])
    def test_app_sockets_valid_socket_mode(self, socket_mode, socket_yaml_data):
        data = socket_yaml_data(listen_stream="test", socket_mode=socket_mode)

        if socket_mode != "_invalid":
            project = Project.unmarshal(data)
            assert project.apps is not None
            assert project.apps["app1"].sockets is not None
            assert project.apps["app1"].sockets["socket1"].socket_mode == socket_mode
        else:
            error = ".*value is not a valid integer"
            with pytest.raises(errors.ProjectValidationError, match=error):
                Project.unmarshal(data)

    @pytest.mark.parametrize(
        "system_username",
        [
            {"snap_daemon": {"scope": "shared"}},
            {"snap_microk8s": {"scope": "shared"}},
            {"snap_aziotedge": {"scope": "shared"}},
            {"snap_aziotdu": {"scope": "shared"}},
            {"snap_daemon": "shared"},
            {"snap_microk8s": "shared"},
            {"snap_aziotedge": "shared"},
            {"snap_aziotdu": "shared"},
        ],
    )
    def test_project_system_usernames_valid(self, system_username, project_yaml_data):
        project = Project.unmarshal(project_yaml_data(system_usernames=system_username))
        assert project.system_usernames == system_username

    @pytest.mark.parametrize(
        "system_username",
        [
            0,
            "string",
        ],
    )
    def test_project_system_usernames_invalid(self, system_username, project_yaml_data):
        error = "- value is not a valid dict"
        with pytest.raises(errors.ProjectValidationError, match=error):
            Project.unmarshal(project_yaml_data(system_usernames=system_username))

    def test_project_provenance(self, project_yaml_data):
        """Verify provenance is parsed."""
        project = Project.unmarshal(project_yaml_data(provenance="test-provenance-1"))
        assert project.provenance == "test-provenance-1"

    @pytest.mark.parametrize("provenance", ["invalid$", "invalid_invalid"])
    def test_project_provenance_invalid(self, provenance, project_yaml_data):
        """Verify invalid provenance values raises an error."""
        error = "provenance must consist of alphanumeric characters and/or hyphens."
        with pytest.raises(errors.ProjectValidationError, match=error):
            Project.unmarshal(project_yaml_data(provenance=provenance))


class TestGrammarValidation:
    """Basic grammar validation testing."""

    def test_grammar_trivial(self, project_yaml_data):
        data = project_yaml_data(
            parts={
                "p1": {
                    "plugin": "nil",
                }
            }
        )
        GrammarAwareProject.validate_grammar(data)

    def test_grammar_without_grammar(self, project_yaml_data):
        data = project_yaml_data(
            parts={
                "p1": {
                    "plugin": "nil",
                    "sources": ".",
                    "build-environment": [
                        {"FOO": "1"},
                        {"BAR": "2"},
                    ],
                    "build-packages": ["a", "b"],
                    "build-snaps": ["d", "e"],
                    "stage-packages": ["foo", "bar"],
                    "stage-snaps": ["baz", "quux"],
                }
            }
        )
        GrammarAwareProject.validate_grammar(data)

    def test_grammar_simple(self, project_yaml_data):
        data = project_yaml_data(
            parts={
                "p1": {
                    "plugin": "nil",
                    "sources": [
                        {"on arm64": "this"},
                        {"else": "that"},
                    ],
                    "build-environment": [
                        {
                            "on amd64": [
                                {"FOO": "1"},
                                {"BAR": "2"},
                            ]
                        },
                    ],
                    "build-packages": [{"to arm64,amd64": ["a", "b"]}, "else fail"],
                    "build-snaps": [
                        {"on somearch": ["d", "e"]},
                    ],
                    "stage-packages": [
                        "pkg1",
                        "pkg2",
                        {"to somearch": ["foo", "bar"]},
                    ],
                    "stage-snaps": [
                        {"on arch to otherarch": ["baz", "quux"]},
                    ],
                }
            }
        )
        GrammarAwareProject.validate_grammar(data)

    def test_grammar_recursive(self, project_yaml_data):
        data = project_yaml_data(
            parts={
                "p1": {
                    "plugin": "nil",
                    "sources": [
                        {"on arm64": [{"to amd64": "this"}, "else fail"]},
                        {"else": "that"},
                    ],
                }
            }
        )
        GrammarAwareProject.validate_grammar(data)

    def test_grammar_try(self, project_yaml_data):
        data = project_yaml_data(
            parts={
                "p1": {
                    "plugin": "nil",
                    "source": [
                        {"try": "this"},
                        {"else": "that"},
                    ],
                }
            }
        )

        error = r".*- 'try' was removed from grammar, use 'on <arch>' instead"
        with pytest.raises(errors.ProjectValidationError, match=error):
            GrammarAwareProject.validate_grammar(data)

    def test_grammar_type_error(self, project_yaml_data):
        data = project_yaml_data(
            parts={
                "p1": {
                    "plugin": "nil",
                    "source": [
                        {"on amd64": [25]},
                    ],
                }
            }
        )

        error = r".*- value must be a string: \[25\]"
        with pytest.raises(errors.ProjectValidationError, match=error):
            GrammarAwareProject.validate_grammar(data)

    def test_grammar_syntax_error(self, project_yaml_data):
        data = project_yaml_data(
            parts={
                "p1": {
                    "plugin": "nil",
                    "source": [
                        {"on amd64,,arm64": "foo"},
                    ],
                }
            }
        )

        error = r".*- syntax error in 'on' selector"
        with pytest.raises(errors.ProjectValidationError, match=error):
            GrammarAwareProject.validate_grammar(data)


def test_get_snap_project_with_base(snapcraft_yaml):
    project = Project.unmarshal(snapcraft_yaml(base="core22"))

    assert project.get_extra_build_snaps() == ["core22"]


def test_get_snap_project_with_content_plugs(snapcraft_yaml, new_dir):
    yaml_data = {
        "name": "mytest",
        "version": "0.1",
        "base": "core22",
        "summary": "Just some test data",
        "description": "This is just some test data.",
        "grade": "stable",
        "confinement": "strict",
        "parts": {"part1": {"plugin": "nil"}},
        "plugs": {
            "test-plug-1": {
                "content": "content-interface",
                "interface": "content",
                "target": "$SNAP/content",
                "default-provider": "test-snap-1",
            },
            "test-plug-2": {
                "content": "content-interface",
                "interface": "content",
                "target": "$SNAP/content",
                "default-provider": "test-snap-2",
            },
        },
    }

    project = Project(**yaml_data)

    assert project.get_extra_build_snaps() == [
        "core22",
        "test-snap-1",
        "test-snap-2",
    ]


def test_get_snap_project_with_content_plugs_does_not_add_extension(
    snapcraft_yaml, new_dir
):
    yaml_data = {
        "name": "mytest",
        "version": "0.1",
        "base": "core22",
        "summary": "Just some test data",
        "description": "This is just some test data.",
        "grade": "stable",
        "confinement": "strict",
        "plugs": {
            "test-plug-1": {
                "content": "content-interface",
                "interface": "content",
                "target": "$SNAP/content",
                "default-provider": "test-snap-1",
            },
            "test-plug-2": {
                "content": "content-interface",
                "interface": "content",
                "target": "$SNAP/content",
                "default-provider": "test-snap-2",
            },
        },
        "parts": {
            "part1": {"plugin": "nil", "build-snaps": ["test-snap-2", "test-snap-3"]}
        },
    }

    project = Project(**yaml_data)

    assert project.get_extra_build_snaps() == [
        "core22",
        "test-snap-1",
    ]


class TestArchitecture:
    """Validate architectures."""

    def test_architecture_valid_list_of_strings(self, project_yaml_data):
        """Architectures can be defined as a list of strings (shorthand notation)."""
        data = project_yaml_data(architectures=["amd64", "armhf"])
        architectures = Project.unmarshal(data).architectures

        assert isinstance(architectures[0], Architecture)
        assert isinstance(architectures[1], Architecture)
        assert architectures[0].build_on == ["amd64"]
        assert architectures[0].build_for == ["amd64"]
        assert architectures[1].build_on == ["armhf"]
        assert architectures[1].build_for == ["armhf"]

    def test_architecture_valid_dictionary_of_strings(self, project_yaml_data):
        """`build-on` and `build-for` fields can be strings."""
        data = project_yaml_data(
            architectures=[
                {"build-on": "amd64", "build-for": "amd64"},
                {"build-on": "armhf", "build-for": "armhf"},
            ]
        )
        architectures = Project.unmarshal(data).architectures

        assert isinstance(architectures[0], Architecture)
        assert isinstance(architectures[1], Architecture)
        assert architectures[0].build_on == ["amd64"]
        assert architectures[0].build_for == ["amd64"]
        assert architectures[1].build_on == ["armhf"]
        assert architectures[1].build_for == ["armhf"]

    def test_architecture_valid_dictionary_of_lists(self, project_yaml_data):
        """`build-on` and `build-for` fields can be lists."""
        data = project_yaml_data(
            architectures=[
                {"build-on": ["amd64"], "build-for": ["amd64"]},
                {"build-on": ["armhf"], "build-for": ["armhf"]},
            ]
        )
        architectures = Project.unmarshal(data).architectures

        assert isinstance(architectures[0], Architecture)
        assert isinstance(architectures[1], Architecture)
        assert architectures[0].build_on == ["amd64"]
        assert architectures[0].build_for == ["amd64"]
        assert architectures[1].build_on == ["armhf"]
        assert architectures[1].build_for == ["armhf"]

    def test_architecture_invalid_string(self, project_yaml_data):
        """A single string is not valid."""
        data = project_yaml_data(architectures="amd64")

        with pytest.raises(errors.ProjectValidationError) as error:
            Project.unmarshal(data)

        assert "value is not a valid list" in str(error.value)

    def test_architecture_multiple_build_on(self, project_yaml_data):
        """Multiple architectures can be defined in a single `build-on`."""
        data = project_yaml_data(
            architectures=[
                {"build-on": ["amd64", "armhf"], "build-for": ["amd64"]},
            ]
        )
        architectures = Project.unmarshal(data).architectures

        assert isinstance(architectures[0], Architecture)
        assert architectures[0].build_on == ["amd64", "armhf"]
        assert architectures[0].build_for == ["amd64"]

    def test_architecture_implicit_build_for(self, project_yaml_data):
        """`build-for` is implicitly defined as the same as `build-for`."""
        data = project_yaml_data(
            architectures=[
                {"build-on": ["arm64"]},
            ]
        )
        architectures = Project.unmarshal(data).architectures

        assert isinstance(architectures[0], Architecture)
        assert architectures[0].build_on == ["arm64"]
        assert architectures[0].build_for == ["arm64"]

    def test_architecture_unknown_property(self, project_yaml_data):
        """Additional fields in the architectures node is invalid."""
        data = project_yaml_data(
            architectures=[
                {
                    "bad-property": ["amd64"],
                    "build-on": ["amd64"],
                    "build-for": ["amd64"],
                }
            ]
        )

        with pytest.raises(errors.ProjectValidationError) as error:
            Project.unmarshal(data)

        assert "extra field 'bad-property' not permitted" in str(error.value)

    def test_architecture_missing_build_on(self, project_yaml_data):
        """`build-on` is a required field."""
        data = project_yaml_data(architectures=[{"build-for": ["amd64"]}])

        with pytest.raises(errors.ProjectValidationError) as error:
            Project.unmarshal(data)

        assert "field 'build-on' required" in str(error.value)

    def test_architecture_build_on_all_and_others(self, project_yaml_data):
        """
        `all` cannot be used in the `build-on` field if another
            architecture in `build-on` is defined.
        """
        data = project_yaml_data(
            architectures=[{"build-on": ["all", "amd64"], "build-for": ["amd64"]}]
        )

        with pytest.raises(errors.ProjectValidationError) as error:
            Project.unmarshal(data)

        assert "'all' cannot be used for 'build-on'" in str(error.value)

    def test_architecture_invalid_multiple_build_for(self, project_yaml_data):
        """Only a single item can be defined for `build-for`."""
        data = project_yaml_data(
            architectures=[{"build-on": ["amd64"], "build-for": ["all", "amd64"]}]
        )

        with pytest.raises(errors.ProjectValidationError) as error:
            Project.unmarshal(data)

        assert "only one architecture can be defined for 'build-for'" in str(
            error.value
        )

    def test_architecture_invalid_multiple_implicit_build_for(self, project_yaml_data):
        """Only a single item can be defined for `build-for`."""
        data = project_yaml_data(architectures=[{"build-on": ["amd64", "armhf"]}])

        with pytest.raises(errors.ProjectValidationError) as error:
            Project.unmarshal(data)

        assert "only one architecture can be defined for 'build-for'" in str(
            error.value
        )

    def test_architecture_invalid_build_on_all_build_for_all(self, project_yaml_data):
        """`build-on: all` and `build-for: all` is invalid."""
        data = project_yaml_data(
            architectures=[
                {"build-on": ["all"], "build-for": ["all"]},
            ]
        )

        with pytest.raises(errors.ProjectValidationError) as error:
            Project.unmarshal(data)

        assert "'all' cannot be used for 'build-on'" in str(error.value)

    def test_architecture_invalid_build_on_all_implicit(self, project_yaml_data):
        """`build-on: all` is invalid, even when build-for is missing."""
        data = project_yaml_data(
            architectures=[
                {"build-on": ["all"]},
            ]
        )

        with pytest.raises(errors.ProjectValidationError) as error:
            Project.unmarshal(data)

        assert "'all' cannot be used for 'build-on'" in str(error.value)

    def test_architecture_invalid_build_on_all_build_for_architecture(
        self, project_yaml_data
    ):
        """`build-on: all` is invalid, even when build-for is valid."""
        data = project_yaml_data(
            architectures=[
                {"build-on": ["all"], "build-for": ["amd64"]},
            ]
        )

        with pytest.raises(errors.ProjectValidationError) as error:
            Project.unmarshal(data)

        assert "'all' cannot be used for 'build-on'" in str(error.value)

    def test_architecture_build_on_architecture_build_for_all(self, project_yaml_data):
        """`build-on: arch` and `build-for: all` is valid."""
        data = project_yaml_data(
            architectures=[
                {"build-on": ["amd64"], "build-for": ["all"]},
            ]
        )
        architectures = Project.unmarshal(data).architectures

        assert isinstance(architectures[0], Architecture)
        assert architectures[0].build_on == ["amd64"]
        assert architectures[0].build_for == ["all"]

    def test_architecture_build_on_all_and_other_architectures(self, project_yaml_data):
        """`all` cannot be used for `build-on`, even when another `build-on` is defined."""
        data = project_yaml_data(
            architectures=[
                {"build-on": ["all"], "build-for": ["amd64"]},
                {"build-on": ["armhf"], "build-for": ["armhf"]},
            ]
        )

        with pytest.raises(errors.ProjectValidationError) as error:
            Project.unmarshal(data)

        assert "'all' cannot be used for 'build-on'" in str(error.value)

    def test_architecture_build_for_all_and_other_architectures(
        self, project_yaml_data
    ):
        """`all` cannot be used for `build-for` when another `build-for` is defined."""
        data = project_yaml_data(
            architectures=[
                {"build-on": ["amd64"], "build-for": ["all"]},
                {"build-on": ["armhf"], "build-for": ["amd64"]},
            ]
        )

        with pytest.raises(errors.ProjectValidationError) as error:
            Project.unmarshal(data)

        assert (
            "one of the items has 'all' in 'build-for', but there are"
            " 2 items: upon release they will conflict."
            "'all' should only be used if there is a single item" in str(error.value)
        )

    def test_architecture_multiple_build_on_all(self, project_yaml_data):
        """`all` cannot be used for multiple `build-on` fields."""
        data = project_yaml_data(
            architectures=[
                {"build-on": ["all"], "build-for": ["amd64"]},
                {"build-on": ["all"], "build-for": ["armhf"]},
            ]
        )

        with pytest.raises(errors.ProjectValidationError) as error:
            Project.unmarshal(data)

        assert "'all' cannot be used for 'build-on'" in str(error.value)

    def test_architecture_multiple_build_for_all(self, project_yaml_data):
        """`all` cannot be used for multiple `build-for` fields."""
        data = project_yaml_data(
            architectures=[
                {"build-on": ["amd64"], "build-for": ["all"]},
                {"build-on": ["armhf"], "build-for": ["all"]},
            ]
        )

        with pytest.raises(errors.ProjectValidationError) as error:
            Project.unmarshal(data)

        assert (
            "one of the items has 'all' in 'build-for', but there are"
            " 2 items: upon release they will conflict."
            "'all' should only be used if there is a single item" in str(error.value)
        )

    def test_architecture_multiple_build_on_same_architecture(self, project_yaml_data):
        """The same architecture can be defined in multiple `build-on` fields."""
        data = project_yaml_data(
            architectures=[
                {"build-on": ["amd64"], "build-for": ["amd64"]},
                {"build-on": ["amd64", "arm64"], "build-for": ["arm64"]},
            ]
        )

        architectures = Project.unmarshal(data).architectures

        assert isinstance(architectures[0], Architecture)
        assert architectures[0].build_on == ["amd64"]
        assert architectures[0].build_for == ["amd64"]
        assert isinstance(architectures[1], Architecture)
        assert architectures[1].build_on == ["amd64", "arm64"]
        assert architectures[1].build_for == ["arm64"]

    def test_architecture_multiple_build_for_same_architecture(self, project_yaml_data):
        """The same architecture cannot be defined in multiple `build-for` fields."""
        data = project_yaml_data(
            architectures=[
                {"build-on": ["amd64"], "build-for": ["amd64"]},
                {"build-on": ["armhf"], "build-for": ["amd64"]},
            ]
        )

        with pytest.raises(errors.ProjectValidationError) as error:
            Project.unmarshal(data)

        assert "multiple items will build snaps that claim to run on amd64" in str(
            error.value
        )

    def test_architecture_multiple_build_for_same_architecture_implicit(
        self, project_yaml_data
    ):
        """
        The same architecture cannot be defined in multiple `build-for` fields,
        even if implicit values are used to define `build-for`.
        """
        data = project_yaml_data(
            architectures=[
                {"build-on": ["amd64"]},
                {"build-on": ["armhf"], "build-for": ["amd64"]},
            ]
        )

        with pytest.raises(errors.ProjectValidationError) as error:
            Project.unmarshal(data)

        assert "multiple items will build snaps that claim to run on amd64" in str(
            error.value
        )

    def test_project_get_build_on(self, project_yaml_data):
        """Test `get_build_on()` returns the build-on string."""
        data = project_yaml_data(
            architectures=[
                {"build-on": ["arm64"], "build-for": ["armhf"]},
            ]
        )
        project = Project.unmarshal(data)
        assert project.get_build_on() == "arm64"

    def test_project_get_build_for(self, project_yaml_data):
        """Test `get_build_for()`."""
        data = project_yaml_data(
            architectures=[
                {"build-on": ["arm64"], "build-for": ["armhf"]},
            ]
        )
        project = Project.unmarshal(data)
        assert project.get_build_for() == "armhf"
