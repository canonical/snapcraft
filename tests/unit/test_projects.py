# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
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

import pytest

from snapcraft import errors
from snapcraft.projects import Project


@pytest.fixture
def yaml_data():
    def _yaml_data(
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
            "parts": [],
            **kwargs,
        }

    yield _yaml_data


class TestProjectDefaults:
    """Ensure unspecified items have the correct default value."""

    def test_project_defaults(self, yaml_data):
        project = Project.unmarshal(yaml_data())

        assert project.build_base == project.base
        assert project.compression == "xz"
        assert project.contact is None
        assert project.donation is None
        assert project.issues is None
        assert project.source_code is None
        assert project.website is None
        assert project.type == "app"
        assert project.icon is None
        assert project.layout is None
        assert project.license is None
        assert project.architectures == []
        assert project.package_repositories == []
        assert project.assumes == []
        assert project.hooks is None
        assert project.passthrough is None
        assert project.apps is None
        assert project.plugs is None
        assert project.slots is None
        assert project.epoch is None

    def test_app_defaults(self, yaml_data):
        data = yaml_data(apps={"app1": {"command": "/bin/true"}})
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

    @pytest.mark.parametrize(
        "field",
        [
            "name",
            "summary",
            "description",
            "grade",
            "confinement",
            "parts",
        ],
    )
    def test_mandatory_fields(self, field, yaml_data):
        data = yaml_data()
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
    def test_mandatory_base(self, snap_type, requires_base, yaml_data):
        data = yaml_data(type=snap_type)
        data.pop("base")

        if requires_base:
            error = "Snap base must be declared when type is not"
            with pytest.raises(errors.ProjectValidationError, match=error):
                Project.unmarshal(data)
        else:
            project = Project.unmarshal(data)
            assert project.base is None

    def test_mandatory_version(self, yaml_data):
        data = yaml_data()
        data.pop("version")
        error = "Snap version is required if not using adopt-info"
        with pytest.raises(errors.ProjectValidationError, match=error):
            Project.unmarshal(data)

    def test_version_not_required(self, yaml_data):
        data = yaml_data()
        data.pop("version")
        data["adopt-info"] = "part1"
        project = Project.unmarshal(data)
        assert project.version is None

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
    def test_project_name_valid(self, name, yaml_data):
        project = Project.unmarshal(yaml_data(name=name))
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
    def test_project_name_invalid(self, name, error, yaml_data):
        with pytest.raises(errors.ProjectValidationError, match=error):
            Project.unmarshal(yaml_data(name=name))

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
    def test_project_version_valid(self, version, yaml_data):
        project = Project.unmarshal(yaml_data(version=version))
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
    def test_project_version_invalid(self, version, error, yaml_data):
        with pytest.raises(errors.ProjectValidationError, match=error):
            Project.unmarshal(yaml_data(version=version))

    @pytest.mark.parametrize(
        "snap_type",
        ["app", "gadget", "kernel", "snapd", "base", "_invalid"],
    )
    def test_project_type(self, snap_type, yaml_data):
        data = yaml_data(type=snap_type)
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
        "confinement",
        ["strict", "devmode", "classic", "_invalid"],
    )
    def test_project_confinement(self, confinement, yaml_data):
        data = yaml_data(confinement=confinement)

        if confinement != "_invalid":
            project = Project.unmarshal(data)
            assert project.confinement == confinement
        else:
            error = ".*unexpected value; permitted: 'classic', 'devmode', 'strict'"
            with pytest.raises(errors.ProjectValidationError, match=error):
                Project.unmarshal(data)

    @pytest.mark.parametrize(
        "grade",
        ["devel", "stable", "_invalid"],
    )
    def test_project_grade(self, grade, yaml_data):
        data = yaml_data(grade=grade)

        if grade != "_invalid":
            project = Project.unmarshal(data)
            assert project.grade == grade
        else:
            error = ".*unexpected value; permitted: 'stable', 'devel'"
            with pytest.raises(errors.ProjectValidationError, match=error):
                Project.unmarshal(data)

    def test_project_summary_valid(self, yaml_data):
        summary = "x" * 78
        project = Project.unmarshal(yaml_data(summary=summary))
        assert project.summary == summary

    def test_project_summary_invalid(self, yaml_data):
        summary = "x" * 79
        error = "ensure this value has at most 78 characters"
        with pytest.raises(errors.ProjectValidationError, match=error):
            Project.unmarshal(yaml_data(summary=summary))

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
    def test_project_epoch_valid(self, epoch, yaml_data):
        project = Project.unmarshal(yaml_data(epoch=epoch))
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
    def test_project_epoch_invalid(self, epoch, yaml_data):
        error = "Epoch is a positive integer followed by an optional asterisk"
        with pytest.raises(errors.ProjectValidationError, match=error):
            Project.unmarshal(yaml_data(epoch=epoch))

    def test_project_package_repository(self, yaml_data):
        repos = [
            {
                "type": "apt",
                "ppa": "test/somerepo",
            },
            {
                "type": "apt",
                "url": "https://some/url",
                "key_id": "KEYID12345" * 4,
            },
        ]
        project = Project.unmarshal(yaml_data(package_repositories=repos))
        assert project.package_repositories == repos

    def test_project_package_repository_missing_fields(self, yaml_data):
        repos = [
            {
                "type": "apt",
            },
        ]
        error = r".*\n- field 'url' required .*\n- field 'key-id' required"
        with pytest.raises(errors.ProjectValidationError, match=error):
            Project.unmarshal(yaml_data(package_repositories=repos))

    def test_project_package_repository_extra_fields(self, yaml_data):
        repos = [
            {
                "type": "apt",
                "extra": "something",
            },
        ]
        error = r".*\n- extra field 'extra' not permitted"
        with pytest.raises(errors.ProjectValidationError, match=error):
            Project.unmarshal(yaml_data(package_repositories=repos))


class TestAppValidation:
    """Validate apps."""

    @pytest.mark.parametrize(
        "daemon",
        ["simple", "forking", "oneshot", "notify", "dbus", "_invalid"],
    )
    def test_app_daemon(self, daemon, yaml_data):
        data = yaml_data(apps={"app1": {"command": "/bin/true", "daemon": daemon}})

        if daemon != "_invalid":
            project = Project.unmarshal(data)
            assert project.apps is not None
            assert project.apps["app1"].daemon == daemon
        else:
            error = ".*unexpected value; permitted: 'simple', 'forking', 'oneshot'"
            with pytest.raises(errors.ProjectValidationError, match=error):
                Project.unmarshal(data)

    @pytest.mark.parametrize("refresh_mode", ["endure", "restart", "_invalid"])
    def test_app_refresh_mode(self, refresh_mode, yaml_data):
        data = yaml_data(
            apps={"app1": {"command": "/bin/true", "refresh-mode": refresh_mode}}
        )

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
            "_invalid",
        ],
    )
    def test_app_stop_mode(self, stop_mode, yaml_data):
        data = yaml_data(
            apps={"app1": {"command": "/bin/true", "stop-mode": stop_mode}}
        )

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
    def test_app_restart_condition(self, restart_condition, yaml_data):
        data = yaml_data(
            apps={
                "app1": {"command": "/bin/true", "restart-condition": restart_condition}
            }
        )

        if restart_condition != "_invalid":
            project = Project.unmarshal(data)
            assert project.apps is not None
            assert project.apps["app1"].restart_condition == restart_condition
        else:
            error = ".*unexpected value; permitted: 'on-success', 'on-failure', 'on-abnormal'"
            with pytest.raises(errors.ProjectValidationError, match=error):
                Project.unmarshal(data)

    @pytest.mark.parametrize("install_mode", ["enable", "disable", "_invalid"])
    def test_app_install_mode(self, install_mode, yaml_data):
        data = yaml_data(
            apps={"app1": {"command": "/bin/true", "install-mode": install_mode}}
        )

        if install_mode != "_invalid":
            project = Project.unmarshal(data)
            assert project.apps is not None
            assert project.apps["app1"].install_mode == install_mode
        else:
            error = ".*unexpected value; permitted: 'enable', 'disable'"
            with pytest.raises(errors.ProjectValidationError, match=error):
                Project.unmarshal(data)
