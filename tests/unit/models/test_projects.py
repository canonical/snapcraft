# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022-2024 Canonical Ltd.
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

import itertools
from typing import Any, Dict, cast

import pydantic
import pytest
from craft_application.errors import CraftValidationError
from craft_application.models import BuildInfo, UniqueStrList
from craft_providers.bases import BaseName

import snapcraft.models
from snapcraft import const, errors, providers, utils
from snapcraft.models import (
    MANDATORY_ADOPTABLE_FIELDS,
    Architecture,
    ComponentProject,
    ContentPlug,
    GrammarAwareProject,
    Hook,
    Platform,
    Project,
)
from snapcraft.models.project import apply_root_packages
from snapcraft.utils import get_host_architecture

# required project data for core24 snaps
CORE24_DATA = {"base": "core24", "grade": "devel"}


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
        assert project.package_repositories is None
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
                build_on=cast(UniqueStrList, [get_host_architecture()]),
                build_for=cast(UniqueStrList, [get_host_architecture()]),
            )
        ]
        assert project.ua_services is None
        assert project.system_usernames is None
        assert project.provenance is None
        assert project.components is None

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
        error = f"Required field '{field}' is not set and 'adopt-info' not used."
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
        data["adopt-info"] = "part1"
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
            "x" * 32,
        ],
    )
    def test_project_version_valid(self, version, project_yaml_data):
        project = Project.unmarshal(project_yaml_data(version=version))
        assert project.version == version

    def test_project_version_invalid(self, project_yaml_data):
        # We only test one invalid version as this model is inherited
        # from Craft Application.
        with pytest.raises(errors.ProjectValidationError) as raised:

            Project.unmarshal(project_yaml_data(version="1=1"))

        assert str(raised.value) == (
            "Bad snapcraft.yaml content:\n- string does not match regex "
            '"^[a-zA-Z0-9](?:[a-zA-Z0-9:.+~-]*[a-zA-Z0-9+~])?$" '
            "(in field 'version')"
        )

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
                project.grade = grade  # type: ignore

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

    def test_project_default_provider_with_channel(self, project_yaml_data):
        content_plug_data = {
            "content-interface": {
                "interface": "content",
                "target": "test-target",
                "content": "test-content",
                "default-provider": "test-provider/edge",
            }
        }

        error = (
            "Specifying a snap channel in 'default_provider' is not supported: "
            "test-provider/edge"
        )

        with pytest.raises(errors.ProjectValidationError, match=error):
            Project.unmarshal(project_yaml_data(plugs=content_plug_data))

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

    @pytest.mark.parametrize("build_base", ["core22", "devel"])
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

    @pytest.mark.parametrize(
        ("base", "expected_base"),
        [("bare", None), *providers.SNAPCRAFT_BASE_TO_PROVIDER_BASE.items()],
    )
    def test_provider_base(self, base, expected_base, project_yaml_data):
        providers_base = Project._providers_base(base)

        assert providers_base == expected_base

    def test_provider_base_error(self, project_yaml_data):
        with pytest.raises(CraftValidationError) as raised:
            Project._providers_base("unknown")

        assert "Unknown base 'unknown'" in str(raised.value)

    def test_project_global_plugs_warning(self, project_yaml_data, emitter):
        data = project_yaml_data(plugs={"desktop": None, "desktop-legacy": None})
        Project.unmarshal(data)
        expected_message = (
            "Warning: implicit plug assignment in 'desktop' and 'desktop-legacy'. "
            "Plugs should be assigned to the app to which they apply, and not "
            "implicitly assigned via the global 'plugs:' stanza "
            "which is intended for configuration only."
            "\n(Reference: https://snapcraft.io/docs/snapcraft-top-level-metadata"
            "#heading--plugs-and-slots-for-an-entire-snap)"
        )
        emitter.assert_message(expected_message)

    def test_project_global_slots_warning(self, project_yaml_data, emitter):
        data = project_yaml_data(slots={"home": None, "removable-media": None})
        Project.unmarshal(data)
        expected_message = (
            "Warning: implicit slot assignment in 'home' and 'removable-media'. "
            "Slots should be assigned to the app to which they apply, and not "
            "implicitly assigned via the global 'slots:' stanza "
            "which is intended for configuration only."
            "\n(Reference: https://snapcraft.io/docs/snapcraft-top-level-metadata"
            "#heading--plugs-and-slots-for-an-entire-snap)"
        )
        emitter.assert_message(expected_message)

    def test_links_scalar(self, project_yaml_data):
        data = project_yaml_data(
            contact="https://matrix.to/#/#nickvision:matrix.org",
            donation="https://github.com/sponsors/nlogozzo",
            issues="https://github.com/NickvisionApps/Parabolic/issues",
            source_code="https://github.com/NickvisionApps/Parabolic",
            website="https://github.com/NickvisionApps/Parabolic",
        )
        project = Project.unmarshal(data)
        assert project.contact == ["https://matrix.to/#/#nickvision:matrix.org"]
        assert project.donation == ["https://github.com/sponsors/nlogozzo"]
        assert project.issues == ["https://github.com/NickvisionApps/Parabolic/issues"]
        assert project.source_code == ["https://github.com/NickvisionApps/Parabolic"]
        assert project.website == ["https://github.com/NickvisionApps/Parabolic"]

    def test_links_list(self, project_yaml_data):
        data = project_yaml_data(
            contact=[
                "https://matrix.to/#/#nickvision:matrix.org",
                "hello@example.org",
            ],
            donation=[
                "https://github.com/sponsors/nlogozzo",
                "https://paypal.me/nlogozzo",
            ],
            issues=[
                "https://github.com/NickvisionApps/Parabolic/issues",
                "https://github.com/NickvisionApps/Denaro/issues",
            ],
            source_code=[
                "https://github.com/NickvisionApps/Parabolic",
                "https://github.com/NickvisionApps/Denaro",
            ],
            website=[
                "https://github.com/NickvisionApps/Parabolic",
                "https://github.com/NickvisionApps/Denaro",
            ],
        )
        project = Project.unmarshal(data)
        assert project.contact == [
            "https://matrix.to/#/#nickvision:matrix.org",
            "hello@example.org",
        ]
        assert project.donation == [
            "https://github.com/sponsors/nlogozzo",
            "https://paypal.me/nlogozzo",
        ]
        assert project.issues == [
            "https://github.com/NickvisionApps/Parabolic/issues",
            "https://github.com/NickvisionApps/Denaro/issues",
        ]
        assert project.source_code == [
            "https://github.com/NickvisionApps/Parabolic",
            "https://github.com/NickvisionApps/Denaro",
        ]
        assert project.website == [
            "https://github.com/NickvisionApps/Parabolic",
            "https://github.com/NickvisionApps/Denaro",
        ]


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


class TestPlatforms:
    """Validate platforms."""

    VALID_PLATFORM_ARCHITECTURES = [
        # single architecture in a list
        *(list(x) for x in itertools.combinations(const.SnapArch, 1)),
        # two architectures in a list
        *(list(x) for x in itertools.combinations(const.SnapArch, 2)),
    ]

    @pytest.mark.parametrize("build_on", VALID_PLATFORM_ARCHITECTURES)
    @pytest.mark.parametrize("build_for", [[arch] for arch in const.SnapArch])
    def test_platform_validation_lists(self, build_on, build_for, project_yaml_data):
        """Unmarshal build-on and build-for lists."""
        platform_data = Platform(**{"build-on": build_on, "build-for": build_for})

        assert platform_data.build_for == build_for
        assert platform_data.build_on == build_on

    @pytest.mark.parametrize("build_on", const.SnapArch)
    @pytest.mark.parametrize("build_for", const.SnapArch)
    def test_platform_validation_strings(self, build_on, build_for, project_yaml_data):
        """Unmarshal and vectorize build-on and build-for strings."""
        platform_data = Platform(**{"build-on": build_on, "build-for": build_for})

        assert platform_data.build_for == [build_for]
        assert platform_data.build_on == [build_on]

    def test_platform_build_for_requires_build_on(self, project_yaml_data):
        """Raise an error if build-for is provided by build-on is not."""
        with pytest.raises(CraftValidationError) as raised:
            Platform(**{"build-for": [const.SnapArch.amd64]})

        assert "'build_for' expects 'build_on' to also be provided" in str(raised.value)

    def test_platforms_not_allowed_core22(self, project_yaml_data):
        with pytest.raises(errors.ProjectValidationError) as raised:
            Project.unmarshal(project_yaml_data(platforms={"amd64": None}))

        assert (
            "'platforms' keyword is not supported for base 'core22'. "
            "Use 'architectures' keyword instead." in str(raised.value)
        )

    @pytest.mark.parametrize(
        ("architectures", "expected"),
        [
            ([], {}),
            (
                ["amd64"],
                {
                    "amd64": Platform(
                        build_for=[const.SnapArch("amd64")],
                        build_on=[const.SnapArch("amd64")],
                    )
                },
            ),
            (
                [Architecture(build_on="amd64", build_for="riscv64")],
                {
                    "riscv64": Platform(
                        build_for=[const.SnapArch("riscv64")],
                        build_on=[const.SnapArch("amd64")],
                    )
                },
            ),
            (
                [
                    Architecture.unmarshal(
                        {"build_on": ["amd64"], "build_for": ["riscv64"]}
                    )
                ],
                {
                    "riscv64": Platform(
                        build_for=[const.SnapArch("riscv64")],
                        build_on=[const.SnapArch("amd64")],
                    )
                },
            ),
            (
                [
                    Architecture.unmarshal(
                        {"build_on": ["amd64", "arm64"], "build_for": ["riscv64"]}
                    ),
                    Architecture.unmarshal(
                        {"build_on": ["amd64", "arm64"], "build_for": ["arm64"]}
                    ),
                ],
                {
                    "riscv64": Platform(
                        build_for=[const.SnapArch("riscv64")],
                        build_on=[const.SnapArch("amd64"), const.SnapArch("arm64")],
                    ),
                    "arm64": Platform(
                        build_for=[const.SnapArch("arm64")],
                        build_on=[const.SnapArch("amd64"), const.SnapArch("arm64")],
                    ),
                },
            ),
        ],
    )
    def test_from_architectures(self, architectures, expected):
        assert Platform.from_architectures(architectures) == expected

    def test_from_architectures_no_all(self):
        """Test that 'from_architectures' does not support architecture 'all'."""
        with pytest.raises(errors.ArchAllInvalid):
            Platform.from_architectures(
                [Architecture(build_on="amd64", build_for="all")]
            )


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

        error = f".*{listen_stream} is not an integer between 1 and 65535"
        with pytest.raises(errors.ProjectValidationError, match=error):
            Project.unmarshal(data)

    @pytest.mark.parametrize("listen_stream", ["@foo"])
    def test_app_sockets_invalid_socket_listen_stream(
        self, listen_stream, socket_yaml_data
    ):
        data = socket_yaml_data(listen_stream=listen_stream)

        error = f".*{listen_stream!r} is not a valid socket path.*"
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

        assert isinstance(architectures, list)
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

        assert isinstance(architectures, list)
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

        assert isinstance(architectures, list)
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

        assert isinstance(architectures, list)
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

        assert isinstance(architectures, list)
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

        assert isinstance(architectures, list)
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

        assert isinstance(architectures, list)
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

    @pytest.mark.parametrize(
        "architectures",
        [
            "unknown",
            {"build-on": ["unknown"]},
            {"build-on": ["unknown"], "build-for": ["amd64"]},
            {"build-on": ["amd64"], "build-for": ["unknown"]},
        ],
    )
    def test_architecture_unsupported(self, architectures, project_yaml_data):
        """Raise an error for unsupported architectures."""
        data = project_yaml_data(architectures=[architectures])

        with pytest.raises(errors.ProjectValidationError) as error:
            Project.unmarshal(data)

        assert "Architecture 'unknown' is not supported." in str(error.value)

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

    def test_project_get_build_for_arch_triplet(self, project_yaml_data):
        """Get architecture triplet for the build-for architecture."""
        data = project_yaml_data(
            architectures=[
                {"build-on": ["arm64"], "build-for": ["armhf"]},
            ]
        )

        project = Project.unmarshal(data)
        arch_triplet = project.get_build_for_arch_triplet()

        assert arch_triplet == "arm-linux-gnueabihf"

    def test_project_get_build_for_arch_triplet_all(self, project_yaml_data):
        """When build-for = "all", the build-for arch triplet should be None."""
        data = project_yaml_data(
            architectures=[
                {"build-on": ["arm64"], "build-for": ["all"]},
            ]
        )

        project = Project.unmarshal(data)
        arch_triplet = project.get_build_for_arch_triplet()

        assert not arch_triplet

    def test_architectures_not_allowed(self, project_yaml_data):
        """'architectures' keyword is not allowed if base is not core22."""
        with pytest.raises(errors.ProjectValidationError) as raised:
            Project.unmarshal(project_yaml_data(**CORE24_DATA, architectures=["amd64"]))

        assert (
            "'architectures' keyword is not supported for base 'core24'. "
            "Use 'platforms' keyword instead."
        ) in str(raised.value)


class TestApplyRootPackages:
    """Test Transform the Project."""

    def test_apply_root_packages(self, project_yaml_data):
        """Test creating a part with root level build-packages and build-snaps."""
        data = project_yaml_data()
        data["build-packages"] = ["pkg1", "pkg2"]
        data["build-snaps"] = ["snap3", "snap4"]

        data_transformed = apply_root_packages(data)

        project = Project.unmarshal(data_transformed)

        assert project.parts["snapcraft/core"]["build-packages"] == ["pkg1", "pkg2"]
        assert project.parts["snapcraft/core"]["build-snaps"] == ["snap3", "snap4"]

    def test_root_packages_transform_no_affect(self, project_yaml_data):
        """Test that nothing is applied if there are not build-packages or build-snaps."""
        data = project_yaml_data()

        data_transformed = apply_root_packages(data)

        project = Project.unmarshal(data_transformed)

        assert project.build_packages is None
        assert project.build_snaps is None
        assert "snapcraft/core" not in project.parts


@pytest.mark.parametrize(
    ("platforms", "expected_build_infos"),
    [
        pytest.param(
            {"amd64": None},
            [
                BuildInfo(
                    build_on="amd64",
                    build_for="amd64",
                    base=BaseName(name="ubuntu", version="24.04"),
                    platform="amd64",
                )
            ],
            id="single_platform_as_arch",
        ),
        pytest.param(
            {
                "arm64": {
                    "build-on": ["arm64", "armhf"],
                    "build-for": ["arm64"],
                },
            },
            [
                BuildInfo(
                    build_on="arm64",
                    build_for="arm64",
                    base=BaseName(name="ubuntu", version="24.04"),
                    platform="arm64",
                ),
                BuildInfo(
                    build_on="armhf",
                    build_for="arm64",
                    base=BaseName(name="ubuntu", version="24.04"),
                    platform="arm64",
                ),
            ],
            id="multiple_build_on",
        ),
        pytest.param(
            {
                "amd64v2": {
                    "build-on": ["amd64"],
                    "build-for": "amd64",
                },
            },
            [
                BuildInfo(
                    build_on="amd64",
                    build_for="amd64",
                    base=BaseName(name="ubuntu", version="24.04"),
                    platform="amd64v2",
                )
            ],
            id="custom_platform_name",
        ),
    ],
)
def test_build_planner_get_build_plan(platforms, expected_build_infos):
    """Test `get_build_plan()` function with different platforms."""
    planner = snapcraft.models.project.SnapcraftBuildPlanner.parse_obj(
        {"name": "test-snap", "base": "core24", "platforms": platforms}
    )

    actual_build_infos = planner.get_build_plan()

    assert actual_build_infos == expected_build_infos


@pytest.mark.parametrize(
    ("architectures", "expected_build_infos"),
    [
        pytest.param(
            ["amd64"],
            [
                BuildInfo(
                    build_on="amd64",
                    build_for="amd64",
                    base=BaseName(name="ubuntu", version="22.04"),
                    platform="amd64",
                )
            ],
            id="single_platform_as_arch",
        ),
        pytest.param(
            [
                {
                    "build-on": ["arm64", "armhf"],
                    "build-for": ["arm64"],
                },
            ],
            [
                BuildInfo(
                    build_on="arm64",
                    build_for="arm64",
                    base=BaseName(name="ubuntu", version="22.04"),
                    platform="arm64",
                ),
                BuildInfo(
                    build_on="armhf",
                    build_for="arm64",
                    base=BaseName(name="ubuntu", version="22.04"),
                    platform="arm64",
                ),
            ],
            id="multiple_build_on",
        ),
        pytest.param(
            [
                {
                    "build-on": ["amd64"],
                    "build-for": "amd64",
                },
            ],
            [
                BuildInfo(
                    build_on="amd64",
                    build_for="amd64",
                    base=BaseName(name="ubuntu", version="22.04"),
                    platform="amd64",
                )
            ],
            id="fully_defined_arch",
        ),
        pytest.param(
            None,
            [
                BuildInfo(
                    build_on=utils.get_host_architecture(),
                    build_for=utils.get_host_architecture(),
                    base=BaseName(name="ubuntu", version="22.04"),
                    platform=utils.get_host_architecture(),
                )
            ],
            id="no_arch",
        ),
    ],
)
def test_build_planner_get_build_plan_core22(architectures, expected_build_infos):
    """Test `get_build_plan()` function with different platforms."""
    planner = snapcraft.models.project.SnapcraftBuildPlanner.parse_obj(
        {"name": "test-snap", "base": "core22", "architectures": architectures}
    )

    actual_build_infos = planner.get_build_plan()

    assert actual_build_infos == expected_build_infos


def test_get_build_plan_devel():
    """Test that "devel" build-bases are correctly reflected on the build plan"""
    planner = snapcraft.models.project.SnapcraftBuildPlanner.parse_obj(
        {
            "name": "test-snap",
            "base": "core24",
            "build-base": "devel",
            "platforms": {"amd64": None},
        }
    )

    build_plan = planner.get_build_plan()
    assert build_plan == [
        BuildInfo(
            build_on="amd64",
            build_for="amd64",
            base=BaseName(name="ubuntu", version="devel"),
            platform="amd64",
        )
    ]


def test_platform_default():
    """Default value for platforms is the host architecture."""
    planner = snapcraft.models.project.SnapcraftBuildPlanner.parse_obj(
        {"name": "test-snap", "base": "core24"}
    )

    actual_build_infos = planner.get_build_plan()

    assert actual_build_infos == [
        BuildInfo(
            build_on=get_host_architecture(),
            build_for=get_host_architecture(),
            base=BaseName(name="ubuntu", version="24.04"),
            platform=get_host_architecture(),
        )
    ]


def test_build_planner_get_build_plan_base(mocker):
    """Test `get_build_plan()` uses the correct base."""
    mock_get_effective_base = mocker.patch(
        "snapcraft.models.project.get_effective_base", return_value="core24"
    )
    planner = snapcraft.models.project.SnapcraftBuildPlanner.parse_obj(
        {
            "name": "test-snap",
            "base": "test-base",
            "build-base": "test-build-base",
            "platforms": {"amd64": None},
            "project_type": "test-type",
        }
    )

    actual_build_infos = planner.get_build_plan()

    assert actual_build_infos == [
        BuildInfo(
            platform="amd64",
            build_on="amd64",
            build_for="amd64",
            base=BaseName(name="ubuntu", version="24.04"),
        )
    ]
    mock_get_effective_base.assert_called_once_with(
        base="test-base",
        build_base="test-build-base",
        project_type="test-type",
        name="test-snap",
        translate_devel=False,
    )


def test_project_platform_error_has_context():
    """Platform validation errors include which platform entry is invalid."""
    with pytest.raises(CraftValidationError) as raised:
        snapcraft.models.project.SnapcraftBuildPlanner.parse_obj(
            {
                "name": "test-snap",
                "base": "test-base",
                "build-base": "test-build-base",
                "platforms": {"test-platform": {"build-for": ["amd64"]}},
                "project_type": "test-type",
            }
        )

    assert "'test-platform': 'build_for' expects 'build_on'" in str(raised.value)


def test_project_platform_mismatch():
    """Raise an error if platform name and build-for are valid but different archs."""
    with pytest.raises(CraftValidationError) as raised:
        snapcraft.models.project.SnapcraftBuildPlanner.parse_obj(
            {
                "name": "test-snap",
                "base": "test-base",
                "build-base": "test-build-base",
                "platforms": {"amd64": {"build-on": ["amd64"], "build-for": ["arm64"]}},
                "project_type": "test-type",
            }
        )

    assert (
        "if 'build_for' is provided and the platform entry label "
        "corresponds to a valid architecture, then both values must match. "
        "amd64 != arm64" in str(raised.value)
    )


def test_project_platform_unknown_name():
    """Raise an error if an empty platform is not a valid architecture."""
    with pytest.raises(CraftValidationError) as raised:
        snapcraft.models.project.SnapcraftBuildPlanner.parse_obj(
            {
                "name": "test-snap",
                "base": "test-base",
                "build-base": "test-build-base",
                "platforms": {"unknown": None},
                "project_type": "test-type",
            }
        )

    assert (
        "platform entry label must correspond to a valid architecture "
        "if 'build-for' is not provided." in str(raised.value)
    )


@pytest.mark.parametrize("project", [ComponentProject, Project])
class TestComponents:
    """Validate components."""

    @pytest.fixture
    def stub_component_data(self):
        data: dict[str, Any] = {
            "type": "test",
            "summary": "test summary",
            "description": "test description",
            "version": "1.0",
            "hooks": None,
        }
        return data

    def test_components_valid(self, project, project_yaml_data, stub_component_data):
        components = {"foo": stub_component_data, "bar": stub_component_data}

        test_project = project.unmarshal(project_yaml_data(components=components))

        assert test_project.components == components

    def test_component_type_valid(
        self, project, project_yaml_data, stub_component_data
    ):
        component = {"foo": stub_component_data}
        component["foo"]["type"] = "test"

        test_project = project.unmarshal(project_yaml_data(components=component))

        assert test_project.components
        assert test_project.components["foo"].type == "test"

    def test_component_type_invalid(
        self, project, project_yaml_data, stub_component_data
    ):
        component = {"foo": stub_component_data}
        component["foo"]["type"] = "invalid"
        error = ".*unexpected value; permitted: 'test'"

        with pytest.raises(errors.ProjectValidationError, match=error):
            project.unmarshal(project_yaml_data(components=component))

    @pytest.mark.parametrize(
        "name", ["name", "name-with-dashes", "x" * 40, "foo-snap-bar"]
    )
    def test_component_name_valid(
        self, project, name, project_yaml_data, stub_component_data
    ):
        component = {name: stub_component_data}

        test_project = project.unmarshal(project_yaml_data(components=component))

        assert test_project.components
        assert list(test_project.components.keys()) == [name]

    @pytest.mark.parametrize(
        "name,error",
        [
            (
                "snap-",
                "Component names cannot start with the reserved namespace 'snap-'",
            ),
            (
                "snap-foo",
                "Component names cannot start with the reserved namespace 'snap-'",
            ),
            ("123456", "Component names can only use"),
            ("name-ends-with-digits-0123", "Component names can only use"),
            ("456-name-starts-with-digits", "Component names can only use"),
            ("name-789-contains-digits", "Component names can only use"),
            ("name_with_underscores", "Component names can only use"),
            ("name-with-UPPERCASE", "Component names can only use"),
            ("name with spaces", "Component names can only use"),
            ("-name-starts-with-hyphen", "Component names cannot start with a hyphen"),
            ("name-ends-with-hyphen-", "Component names cannot end with a hyphen"),
            (
                "name-has--two-hyphens",
                "Component names cannot have two hyphens in a row",
            ),
            ("x" * 41, "ensure this value has at most 40 characters"),
        ],
    )
    def test_component_name_invalid(
        self, project, name, error, project_yaml_data, stub_component_data
    ):
        component = {name: stub_component_data}

        with pytest.raises(errors.ProjectValidationError, match=error):
            project.unmarshal(project_yaml_data(components=component))

    def test_component_summary_valid(
        self, project, project_yaml_data, stub_component_data
    ):
        component = {"foo": stub_component_data}
        summary = "x" * 78
        component["foo"]["summary"] = summary

        test_project = project.unmarshal(project_yaml_data(components=component))

        assert test_project.components
        assert test_project.components["foo"].summary == summary

    def test_component_summary_invalid(
        self, project, project_yaml_data, stub_component_data
    ):
        component = {"foo": stub_component_data}
        component["foo"]["summary"] = "x" * 79

        error = "ensure this value has at most 78 characters"
        with pytest.raises(errors.ProjectValidationError, match=error):
            project.unmarshal(project_yaml_data(components=component))

    @pytest.mark.parametrize(
        "version",
        [
            "1",
            "1.0",
            "1.0.1-5.2~build0.20.04:1+1A",
            "git",
            "1~",
            "1+",
            "x" * 32,
        ],
    )
    def test_component_version_valid(
        self, project, version, project_yaml_data, stub_component_data
    ):
        component = {"foo": stub_component_data}
        component["foo"]["version"] = version

        test_project = project.unmarshal(project_yaml_data(components=component))

        assert test_project.components
        assert test_project.components["foo"].version == version

    @pytest.mark.parametrize(
        "version,error",
        [
            pytest.param(
                "1_0",
                'string does not match regex "^[a-zA-Z0-9](?:[a-zA-Z0-9:.+~-]*[a-zA-Z0-9+~])?$"',
                id="'_' in version",
            ),
            pytest.param(
                "1=1",
                'string does not match regex "^[a-zA-Z0-9](?:[a-zA-Z0-9:.+~-]*[a-zA-Z0-9+~])?$"',
                id="'=' in version",
            ),
            pytest.param(
                ".1",
                'string does not match regex "^[a-zA-Z0-9](?:[a-zA-Z0-9:.+~-]*[a-zA-Z0-9+~])?$"',
                id="cannot start with '.'",
            ),
            pytest.param(
                ":1",
                'string does not match regex "^[a-zA-Z0-9](?:[a-zA-Z0-9:.+~-]*[a-zA-Z0-9+~])?$"',
                id="cannot start with ':'",
            ),
            pytest.param(
                "+1",
                'string does not match regex "^[a-zA-Z0-9](?:[a-zA-Z0-9:.+~-]*[a-zA-Z0-9+~])?$"',
                id="cannot start with '+'",
            ),
            pytest.param(
                "~1",
                'string does not match regex "^[a-zA-Z0-9](?:[a-zA-Z0-9:.+~-]*[a-zA-Z0-9+~])?$"',
                id="cannot start with '~'",
            ),
            pytest.param(
                "-1",
                'string does not match regex "^[a-zA-Z0-9](?:[a-zA-Z0-9:.+~-]*[a-zA-Z0-9+~])?$"',
                id="cannot start with '-'",
            ),
            pytest.param(
                "1.",
                'string does not match regex "^[a-zA-Z0-9](?:[a-zA-Z0-9:.+~-]*[a-zA-Z0-9+~])?$"',
                id="cannot end with '.'",
            ),
            pytest.param(
                "1:",
                'string does not match regex "^[a-zA-Z0-9](?:[a-zA-Z0-9:.+~-]*[a-zA-Z0-9+~])?$"',
                id="cannot end with ':'",
            ),
            pytest.param(
                "1-",
                'string does not match regex "^[a-zA-Z0-9](?:[a-zA-Z0-9:.+~-]*[a-zA-Z0-9+~])?$"',
                id="cannot end with '-'",
            ),
            pytest.param(
                "x" * 33,
                "ensure this value has at most 32 characters",
                id="too large",
            ),
            pytest.param(
                "",
                'string does not match regex "^[a-zA-Z0-9](?:[a-zA-Z0-9:.+~-]*[a-zA-Z0-9+~])?$"',
                id="empty string",
            ),
        ],
    )
    def test_component_version_invalid(
        self, project, version, error, project_yaml_data, stub_component_data
    ):
        component = {"foo": stub_component_data}
        component["foo"]["version"] = version

        with pytest.raises(errors.ProjectValidationError) as raised:
            project.unmarshal(project_yaml_data(components=component))

        assert error in str(raised.value)
        assert str(raised.value).endswith("(in field 'components.foo.version')")

    def test_get_component_names(self, project, project_yaml_data, stub_component_data):
        components = {"foo": stub_component_data, "bar-baz": stub_component_data}
        test_project = project.unmarshal(project_yaml_data(components=components))

        component_names = test_project.get_component_names()

        assert component_names == ["foo", "bar-baz"]

    def test_get_component_names_none(self, project, project_yaml_data):
        test_project = project.unmarshal(project_yaml_data())

        component_names = test_project.get_component_names()

        assert component_names == []

    def test_get_partitions(self, project, project_yaml_data, stub_component_data):
        components = {"foo": stub_component_data, "bar-baz": stub_component_data}
        test_project = project.unmarshal(project_yaml_data(components=components))

        partitions = test_project.get_partitions()

        assert partitions == ["default", "component/foo", "component/bar-baz"]

    def test_get_partitions_none(self, project, project_yaml_data):
        test_project = project.unmarshal(project_yaml_data())

        partitions = test_project.get_partitions()

        assert partitions is None
