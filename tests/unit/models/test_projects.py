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
from craft_application.models import BuildInfo, UniqueStrList, VersionStr
from craft_platforms import DebianArchitecture
from craft_providers.bases import BaseName

import snapcraft.models
from snapcraft import const, errors, providers
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


@pytest.fixture
def fake_project_with_numbers(project_yaml_data):
    """Returns a fake project with numbers in string fields.

    This includes numbers in fields that are validated by snapcraft and fields
    validated by craft-parts.
    """
    return project_yaml_data(
        # string
        version=1.0,
        # string
        icon=2,
        # list[str]
        website=[3.0, 4],
        # dict[str, str]
        environment={
            "float": 5.0,
            "int": 6,
        },
        parts={
            "p1": {
                "plugin": "nil",
                # string
                "source-type": 7,
                # string
                "source-commit": 8.0,
                # list[str]
                "build-snaps": [9, 10.0],
                # dict[str, str]
                "build-environment": [
                    {"float": 11.0},
                    {"int": 12},
                ],
            }
        },
    )


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
                build_on=cast(UniqueStrList, [str(DebianArchitecture.from_host())]),
                build_for=cast(UniqueStrList, [str(DebianArchitecture.from_host())]),
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

    def test_build_base_validation_reentrant(self, project_yaml_data):
        """Validators should be reentrant.

        Changing a field causes all validators to re-run, so validators should not
        fail when validating an existing model.

        This is a regression test for `base: core22` and `build-base: bare`, where
        the validators receive "build-base" when creating the model and "build_base"
        when re-validating.
        """
        data = project_yaml_data(
            base="bare",
            # build-base has to be parsed for the validator to allow 'architectures'
            build_base="core22",
            architectures=["amd64"],
        )

        project = Project.unmarshal(data)

        # changing any value will re-run the validators, which should not raise an error
        project.version = cast(VersionStr, "1.2.3")

    @pytest.mark.parametrize("field", ["name", "confinement", "parts"])
    def test_mandatory_fields(self, field, project_yaml_data):
        data = project_yaml_data()
        data.pop(field)
        error = f"{field}\n  Field required"
        with pytest.raises(pydantic.ValidationError, match=error):
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
            with pytest.raises(pydantic.ValidationError, match=error):
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
        with pytest.raises(pydantic.ValidationError, match=error):
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
            ("name_with_underscores", "snap names can only use"),
            ("name-with-UPPERCASE", "snap names can only use"),
            ("name with spaces", "snap names can only use"),
            ("-name-starts-with-hyphen", "snap names cannot start with a hyphen"),
            ("name-ends-with-hyphen-", "snap names cannot end with a hyphen"),
            ("name-has--two-hyphens", "snap names cannot have two hyphens in a row"),
            ("123456", "snap names can only use"),
            (
                "a2345678901234567890123456789012345678901",
                "String should have at most 40 characters",
            ),
        ],
    )
    def test_project_name_invalid(self, name, error, project_yaml_data):
        with pytest.raises(pydantic.ValidationError, match=error):
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
        """Test one invalid version as this is inherited from Craft Application."""
        error = "invalid version: Valid versions consist of"

        with pytest.raises(pydantic.ValidationError, match=error):
            Project.unmarshal(project_yaml_data(version="1=1"))

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
            error = "Input should be 'app', 'base', 'gadget', 'kernel' or 'snapd'"
            with pytest.raises(pydantic.ValidationError, match=error):
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
            error = "Input should be 'classic', 'devmode' or 'strict'"
            with pytest.raises(pydantic.ValidationError, match=error):
                Project.unmarshal(data)

    @pytest.mark.parametrize("grade", ["devel", "stable", "_invalid"])
    def test_project_grade(self, grade, project_yaml_data):
        data = project_yaml_data(grade=grade)

        if grade != "_invalid":
            project = Project.unmarshal(data)
            assert project.grade == grade
        else:
            error = "Input should be 'stable' or 'devel'"
            with pytest.raises(pydantic.ValidationError, match=error):
                Project.unmarshal(data)

    @pytest.mark.parametrize("grade", ["devel", "stable", "_invalid"])
    def test_project_grade_assignment(self, grade, project_yaml_data):
        data = project_yaml_data()

        project = Project.unmarshal(data)
        if grade != "_invalid":
            project.grade = grade
        else:
            error = "Input should be 'stable' or 'devel'"
            with pytest.raises(pydantic.ValidationError, match=error):
                project.grade = grade  # type: ignore

    def test_project_summary_valid(self, project_yaml_data):
        summary = "x" * 78
        project = Project.unmarshal(project_yaml_data(summary=summary))
        assert project.summary == summary

    def test_project_summary_invalid(self, project_yaml_data):
        summary = "x" * 79
        error = "String should have at most 78 characters"
        with pytest.raises(pydantic.ValidationError, match=error):
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
        with pytest.raises(pydantic.ValidationError, match=error):
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
        error = r"url\n  Field required.*\n.*\n.*key-id\n  Field required"
        with pytest.raises(pydantic.ValidationError, match=error):
            Project.unmarshal(project_yaml_data(package_repositories=repos))

    def test_project_package_repository_extra_fields(self, project_yaml_data):
        repos = [
            {
                "type": "apt",
                "extra": "something",
            },
        ]
        error = "Extra inputs are not permitted"
        with pytest.raises(pydantic.ValidationError, match=error):
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
        error = "Input should be a valid dictionary"
        with pytest.raises(pydantic.ValidationError, match=error):
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
        error = "Input should be a valid dictionary"
        with pytest.raises(pydantic.ValidationError, match=error):
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

        with pytest.raises(pydantic.ValidationError, match=error):
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

        with pytest.raises(pydantic.ValidationError, match=error):
            Project.unmarshal(project_yaml_data(plugs=content_plug_data))

    @pytest.mark.parametrize("decl_type", ["symlink", "bind", "bind-file", "type"])
    def test_project_layout(self, decl_type, project_yaml_data):
        project = Project.unmarshal(
            project_yaml_data(layout={"foo": {decl_type: "bar"}})
        )
        assert project.layout is not None
        assert project.layout["foo"][decl_type] == "bar"

    def test_project_layout_invalid(self, project_yaml_data):
        error = "Input should be 'symlink', 'bind', 'bind-file' or 'type'"
        with pytest.raises(pydantic.ValidationError, match=error):
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
        error = "grade must be 'devel' when build-base is 'devel'"

        with pytest.raises(pydantic.ValidationError, match=error):
            Project.unmarshal(project_yaml_data(build_base="devel", grade="stable"))

    @pytest.mark.parametrize(
        ("base", "expected_base"),
        [
            ("bare", None),
            *providers.SNAPCRAFT_BASE_TO_PROVIDER_BASE.items(),
            ("core22-desktop", providers.SNAPCRAFT_BASE_TO_PROVIDER_BASE["core22"]),
            ("core24-desktop", providers.SNAPCRAFT_BASE_TO_PROVIDER_BASE["core24"]),
        ],
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

    def test_coerce_numbers(self, fake_project_with_numbers):
        """Coerce numbers into strings."""
        project = Project.unmarshal(fake_project_with_numbers)

        assert project.version == "1.0"
        assert project.icon == "2"
        assert project.website == ["3.0", "4"]
        assert project.environment == {"float": "5.0", "int": "6"}
        # parts remain a dictionary with original types
        assert project.parts["p1"]["source-type"] == 7
        assert project.parts["p1"]["source-commit"] == 8.0
        assert project.parts["p1"]["build-snaps"] == [9, 10.0]
        assert project.parts["p1"]["build-environment"] == [
            {"float": 11.0},
            {"int": 12},
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

        with pytest.raises(pydantic.ValidationError, match=error):
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

        error = "Input should be a valid dictionary"
        with pytest.raises(pydantic.ValidationError, match=error):
            Project.unmarshal(project_yaml_data(hooks=hooks))

    def test_project_hooks_plugs_empty(self, project_yaml_data):
        hook = {"configure": {"plugs": []}}
        error = "'plugs' field cannot be empty"

        with pytest.raises(pydantic.ValidationError, match=error):
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
        error = r"build-on\n  Field required"
        with pytest.raises(pydantic.ValidationError, match=error):
            Platform(**{"build-for": [const.SnapArch.amd64]})  # type: ignore[reportArgumentType]

    def test_platforms_not_allowed_core22(self, project_yaml_data):
        error = (
            "'platforms' keyword is not supported for base 'core22'. "
            "Use 'architectures' keyword instead."
        )
        with pytest.raises(pydantic.ValidationError, match=error):
            Project.unmarshal(project_yaml_data(platforms={"amd64": None}))

    @pytest.mark.parametrize(
        ("architectures", "expected"),
        [
            pytest.param([], {}, id="empty"),
            pytest.param(
                ["amd64"],
                {
                    "amd64": Platform(
                        build_for=[const.SnapArch("amd64")],
                        build_on=[const.SnapArch("amd64")],
                    )
                },
                id="simple",
            ),
            pytest.param(
                [Architecture(build_on="amd64", build_for="riscv64")],
                {
                    "riscv64": Platform(
                        build_for=[const.SnapArch("riscv64")],
                        build_on=[const.SnapArch("amd64")],
                    )
                },
                id="cross-compile-from-object",
            ),
            pytest.param(
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
                id="cross-compile",
            ),
            pytest.param(
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
                id="complex",
            ),
            pytest.param(
                [Architecture.unmarshal({"build_on": ["s390x"], "build_for": ["all"]})],
                {
                    "all": Platform(
                        build_for=["all"],
                        build_on=[const.SnapArch("s390x")],
                    )
                },
                id="all",
            ),
        ],
    )
    def test_from_architectures(self, architectures, expected):
        assert Platform.from_architectures(architectures) == expected


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
        error = "Input should be a valid dictionary"
        with pytest.raises(pydantic.ValidationError, match=error):
            Project.unmarshal(project_yaml_data(system_usernames=system_username))

    def test_project_provenance(self, project_yaml_data):
        """Verify provenance is parsed."""
        project = Project.unmarshal(project_yaml_data(provenance="test-provenance-1"))
        assert project.provenance == "test-provenance-1"

    @pytest.mark.parametrize("provenance", ["invalid$", "invalid_invalid"])
    def test_project_provenance_invalid(self, provenance, project_yaml_data):
        """Verify invalid provenance values raises an error."""
        error = "provenance must consist of alphanumeric characters and/or hyphens."
        with pytest.raises(pydantic.ValidationError, match=error):
            Project.unmarshal(project_yaml_data(provenance=provenance))

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

    def test_grammar_all(self, project_yaml_data):
        data = project_yaml_data(
            parts={
                "p1": {
                    "stage-packages": [
                        "pkg1",
                        "pkg2",
                        {"to all": ["foo", "bar"]},
                    ],
                },
            },
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

        error = "'try' was removed from grammar, use 'on <arch>' instead"
        with pytest.raises(errors.ProjectValidationError, match=error):
            GrammarAwareProject.validate_grammar(data)

    def test_grammar_number_coercion(self, fake_project_with_numbers):
        """Ensure that grammar validation does not fail when coercing numbers into strings."""
        GrammarAwareProject.validate_grammar(fake_project_with_numbers)

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

        error = r"Input should be a valid string \(in field 'parts\.p1\.source\[0\]'\)"
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

        error = "syntax error in 'on' selector"
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

        error = "Input should be a valid list"
        with pytest.raises(pydantic.ValidationError, match=error):
            Project.unmarshal(data)

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

        error = r"bad-property\n  Extra inputs are not permitted"
        with pytest.raises(pydantic.ValidationError, match=error):
            Project.unmarshal(data)

    def test_architecture_missing_build_on(self, project_yaml_data):
        """`build-on` is a required field."""
        data = project_yaml_data(architectures=[{"build-for": ["amd64"]}])

        error = r"build-on\n  Field required"
        with pytest.raises(pydantic.ValidationError, match=error):
            Project.unmarshal(data)

    def test_architecture_build_on_all_and_others(self, project_yaml_data):
        """
        `all` cannot be used in the `build-on` field if another
            architecture in `build-on` is defined.
        """
        data = project_yaml_data(
            architectures=[{"build-on": ["all", "amd64"], "build-for": ["amd64"]}]
        )

        error = "'all' cannot be used for 'build-on'"
        with pytest.raises(pydantic.ValidationError, match=error):
            Project.unmarshal(data)

    def test_architecture_invalid_multiple_build_for(self, project_yaml_data):
        """Only a single item can be defined for `build-for`."""
        data = project_yaml_data(
            architectures=[{"build-on": ["amd64"], "build-for": ["all", "amd64"]}]
        )

        error = "only one architecture can be defined for 'build-for'"
        with pytest.raises(pydantic.ValidationError, match=error):
            Project.unmarshal(data)

    def test_architecture_invalid_multiple_implicit_build_for(self, project_yaml_data):
        """Only a single item can be defined for `build-for`.

        This is true even when 'build-for' is implicitly inferred from 'build-on'.
        """
        data = project_yaml_data(architectures=[{"build-on": ["amd64", "armhf"]}])

        error = "only one architecture can be defined for 'build-for'"
        with pytest.raises(pydantic.ValidationError, match=error):
            Project.unmarshal(data)

    def test_architecture_invalid_build_on_all_build_for_all(self, project_yaml_data):
        """`build-on: all` and `build-for: all` is invalid."""
        data = project_yaml_data(
            architectures=[
                {"build-on": ["all"], "build-for": ["all"]},
            ]
        )

        error = "'all' cannot be used for 'build-on'"
        with pytest.raises(pydantic.ValidationError, match=error):
            Project.unmarshal(data)

    def test_architecture_invalid_build_on_all_implicit(self, project_yaml_data):
        """`build-on: all` is invalid, even when build-for is missing."""
        data = project_yaml_data(
            architectures=[
                {"build-on": ["all"]},
            ]
        )

        error = "'all' cannot be used for 'build-on'"
        with pytest.raises(pydantic.ValidationError, match=error):
            Project.unmarshal(data)

    def test_architecture_invalid_build_on_all_build_for_architecture(
        self, project_yaml_data
    ):
        """`build-on: all` is invalid, even when build-for is valid."""
        data = project_yaml_data(
            architectures=[
                {"build-on": ["all"], "build-for": ["amd64"]},
            ]
        )

        error = "'all' cannot be used for 'build-on'"
        with pytest.raises(pydantic.ValidationError, match=error):
            Project.unmarshal(data)

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

        error = "'all' cannot be used for 'build-on'"
        with pytest.raises(pydantic.ValidationError, match=error):
            Project.unmarshal(data)

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

        error = (
            "one of the items has 'all' in 'build-for', but there are"
            " 2 items: upon release they will conflict."
            "'all' should only be used if there is a single item"
        )
        with pytest.raises(pydantic.ValidationError, match=error):
            Project.unmarshal(data)

    def test_architecture_multiple_build_on_all(self, project_yaml_data):
        """`all` cannot be used for multiple `build-on` fields."""
        data = project_yaml_data(
            architectures=[
                {"build-on": ["all"], "build-for": ["amd64"]},
                {"build-on": ["all"], "build-for": ["armhf"]},
            ]
        )

        error = "'all' cannot be used for 'build-on'"
        with pytest.raises(pydantic.ValidationError, match=error):
            Project.unmarshal(data)

    def test_architecture_multiple_build_for_all(self, project_yaml_data):
        """`all` cannot be used for multiple `build-for` fields."""
        data = project_yaml_data(
            architectures=[
                {"build-on": ["amd64"], "build-for": ["all"]},
                {"build-on": ["armhf"], "build-for": ["all"]},
            ]
        )

        error = (
            "one of the items has 'all' in 'build-for', but there are"
            " 2 items: upon release they will conflict."
            "'all' should only be used if there is a single item"
        )
        with pytest.raises(pydantic.ValidationError, match=error):
            Project.unmarshal(data)

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

        error = "multiple items will build snaps that claim to run on amd64"
        with pytest.raises(pydantic.ValidationError, match=error):
            Project.unmarshal(data)

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

        error = "multiple items will build snaps that claim to run on amd64"
        with pytest.raises(pydantic.ValidationError, match=error):
            Project.unmarshal(data)

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

        error = "Architecture 'unknown' is not supported"
        with pytest.raises(pydantic.ValidationError, match=error):
            Project.unmarshal(data)

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
        error = (
            "'architectures' keyword is not supported for base 'core24'. "
            "Use 'platforms' keyword instead."
        )

        with pytest.raises(pydantic.ValidationError, match=error):
            Project.unmarshal(project_yaml_data(**CORE24_DATA, architectures=["amd64"]))

    @pytest.mark.parametrize(
        ("data", "expected"),
        [
            ({"base": "core22", "architectures": ["amd64"]}, True),
            ({"base": "core22"}, False),
            # core24 and newer do not set this field
            ({"base": "core24"}, None),
        ],
    )
    def test_architectures_in_yaml(self, project_yaml_data, data, expected):
        """Check if architectures were present in the yaml before unmarshalling."""
        project_yaml = project_yaml_data(**data)

        project = Project.unmarshal(project_yaml)

        assert project._architectures_in_yaml is expected

        # adding architectures after unmarshalling does not change the field
        if project.base == "core22":
            project.architectures = [
                Architecture(build_on=["amd64"], build_for=["amd64"])
            ]
            assert project._architectures_in_yaml is expected


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
                "s390x": {
                    "build-on": "s390x",
                },
                "riscv64": {
                    "build-on": ["amd64", "riscv64"],
                },
            },
            [
                BuildInfo(
                    build_on="s390x",
                    build_for="s390x",
                    base=BaseName(name="ubuntu", version="24.04"),
                    platform="s390x",
                ),
                BuildInfo(
                    build_on="amd64",
                    build_for="riscv64",
                    base=BaseName(name="ubuntu", version="24.04"),
                    platform="riscv64",
                ),
                BuildInfo(
                    build_on="riscv64",
                    build_for="riscv64",
                    base=BaseName(name="ubuntu", version="24.04"),
                    platform="riscv64",
                ),
            ],
            id="implicit_build_for",
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
        pytest.param(
            {
                "platform1": {
                    "build-on": ["arm64", "armhf"],
                    "build-for": ["all"],
                },
            },
            [
                BuildInfo(
                    build_on="arm64",
                    build_for="all",
                    base=BaseName(name="ubuntu", version="24.04"),
                    platform="platform1",
                ),
                BuildInfo(
                    build_on="armhf",
                    build_for="all",
                    base=BaseName(name="ubuntu", version="24.04"),
                    platform="platform1",
                ),
            ],
            id="all",
        ),
    ],
)
def test_build_planner_get_build_plan(platforms, expected_build_infos):
    """Test `get_build_plan()` function with different platforms."""
    planner = snapcraft.models.project.SnapcraftBuildPlanner.model_validate(
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
                    "build-for": ["amd64"],
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
            [
                {
                    "build-on": "amd64",
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
            id="fully_defined_arch_as_string",
        ),
        pytest.param(
            None,
            [
                BuildInfo(
                    build_on=str(DebianArchitecture.from_host()),
                    build_for=str(DebianArchitecture.from_host()),
                    base=BaseName(name="ubuntu", version="22.04"),
                    platform=str(DebianArchitecture.from_host()),
                )
            ],
            id="no_arch",
        ),
        pytest.param(
            [
                {
                    "build-on": ["s390x"],
                    "build-for": ["all"],
                },
            ],
            [
                BuildInfo(
                    build_on="s390x",
                    build_for="all",
                    base=BaseName(name="ubuntu", version="22.04"),
                    platform="all",
                )
            ],
            id="all",
        ),
        pytest.param(
            [
                {
                    "build-on": "s390x",
                    "build-for": "all",
                },
            ],
            [
                BuildInfo(
                    build_on="s390x",
                    build_for="all",
                    base=BaseName(name="ubuntu", version="22.04"),
                    platform="all",
                )
            ],
            id="all_as_string",
        ),
    ],
)
def test_build_planner_get_build_plan_core22(architectures, expected_build_infos):
    """Test `get_build_plan()` function with different platforms."""
    planner = snapcraft.models.project.SnapcraftBuildPlanner.model_validate(
        {"name": "test-snap", "base": "core22", "architectures": architectures}
    )

    actual_build_infos = planner.get_build_plan()

    assert actual_build_infos == expected_build_infos


@pytest.mark.parametrize(
    ("platforms", "message"),
    [
        # this could be made a valid definition by adjusting the manual validators
        pytest.param(
            {"all": {"build-on": ["arm64", "armhf"]}},
            "platform entry label must correspond to a valid architecture if 'build-for' is not provided",
            id="no-build-for",
        ),
        # this is invalid because the platform 'all' will be used for 'build-on'
        pytest.param(
            {"all": None},
            "'all' cannot be used for 'build-on'",
            id="no-build-for-no-build-on",
        ),
    ],
)
def test_build_planner_all_as_platform_invalid(platforms, message):
    """The platform must be fully defined when using 'all'."""
    build_plan_data = {
        "name": "test-snap",
        "base": "core24",
        "platforms": platforms,
    }
    with pytest.raises(pydantic.ValidationError, match=message):
        snapcraft.models.project.SnapcraftBuildPlanner(**build_plan_data)


def test_build_planner_all_with_other_builds():
    """'build-for: all' cannot be combined with other builds."""
    build_plan_data = {
        "name": "test-snap",
        "base": "core24",
        "platforms": {
            "platform1": {
                "build-on": ["arm64", "armhf"],
                "build-for": ["arm64"],
            },
            "platform2": {
                "build-on": ["s390x"],
                "build-for": ["all"],
            },
        },
    }

    with pytest.raises(pydantic.ValidationError) as raised:
        snapcraft.models.project.SnapcraftBuildPlanner(**build_plan_data)

    assert (
        "one of the platforms has 'all' in 'build-for', but there are 2 platforms"
    ) in str(raised.value)


def test_build_planner_all_with_other_builds_core22():
    """'build-for: all' cannot be combined with other builds with core22 syntax."""
    build_plan_data = {
        "name": "test-snap",
        "base": "core22",
        "architectures": [
            {
                "build-on": ["s390x"],
                "build-for": ["all"],
            },
            {
                "build-on": ["arm64", "armhf"],
                "build-for": ["arm64"],
            },
        ],
    }

    build_plan = snapcraft.models.project.SnapcraftBuildPlanner(**build_plan_data)

    # architectures are only converted to Platforms when creating the build plan
    with pytest.raises(pydantic.ValidationError) as raised:
        build_plan.get_build_plan()

    assert (
        "one of the platforms has 'all' in 'build-for', but there are 2 platforms"
    ) in str(raised.value)


def test_get_build_plan_devel():
    """Test that "devel" build-bases are correctly reflected on the build plan"""
    planner = snapcraft.models.project.SnapcraftBuildPlanner.model_validate(
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
    planner = snapcraft.models.project.SnapcraftBuildPlanner.model_validate(
        {"name": "test-snap", "base": "core24"}
    )

    actual_build_infos = planner.get_build_plan()

    assert actual_build_infos == [
        BuildInfo(
            build_on=str(DebianArchitecture.from_host()),
            build_for=str(DebianArchitecture.from_host()),
            base=BaseName(name="ubuntu", version="24.04"),
            platform=str(DebianArchitecture.from_host()),
        )
    ]


def test_project_platform_error_has_context():
    """Platform validation errors include which platform entry is invalid."""
    error = r"build-on\n  Field required"
    with pytest.raises(pydantic.ValidationError, match=error):
        snapcraft.models.project.SnapcraftBuildPlanner.model_validate(
            {
                "name": "test-snap",
                "base": "test-base",
                "build-base": "test-build-base",
                "platforms": {"test-platform": {"build-for": ["amd64"]}},
                "project_type": "test-type",
            }
        )


def test_project_platform_mismatch():
    """Raise an error if platform name and build-for are valid but different archs."""
    with pytest.raises(pydantic.ValidationError) as raised:
        snapcraft.models.project.SnapcraftBuildPlanner.model_validate(
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
    with pytest.raises(pydantic.ValidationError) as raised:
        snapcraft.models.project.SnapcraftBuildPlanner.model_validate(
            {
                "name": "test-snap",
                "base": "test-base",
                "build-base": "test-build-base",
                "platforms": {"unknown": None},
                "project_type": "test-type",
            }
        )

    assert "'unknown' is not a valid Debian architecture." in str(raised.value)


@pytest.mark.parametrize("project", [ComponentProject, Project])
class TestComponents:
    """Validate components."""

    @pytest.fixture
    def stub_component_data(self):
        data = {
            "type": "test",
            "summary": "test summary",
            "description": "test description",
            "version": "1.0",
            "hooks": None,
        }
        return data

    def test_components_valid(self, project, project_yaml_data, stub_component_data):
        component_data = {"foo": stub_component_data, "bar": stub_component_data}
        components = {
            "foo": snapcraft.models.Component.unmarshal(stub_component_data),
            "bar": snapcraft.models.Component.unmarshal(stub_component_data),
        }

        test_project = project.unmarshal(project_yaml_data(components=component_data))

        assert test_project.components == components

    @pytest.mark.parametrize("component_type", ["test", "kernel-modules", "standard"])
    def test_component_type_valid(
        self, component_type, project, project_yaml_data, stub_component_data
    ):
        component = {"foo": stub_component_data}
        component["foo"]["type"] = component_type

        test_project = project.unmarshal(project_yaml_data(components=component))

        assert test_project.components
        assert test_project.components["foo"].type == component_type

    def test_component_type_invalid(
        self, project, project_yaml_data, stub_component_data
    ):
        component = {"foo": stub_component_data}
        component["foo"]["type"] = "invalid"

        error = "Input should be 'test'"
        with pytest.raises(pydantic.ValidationError, match=error):
            project.unmarshal(project_yaml_data(components=component))

    @pytest.mark.parametrize(
        "name",
        [
            "name",
            "name-with-dashes",
            "name-with-numbers-0123",
            "0123-name-with-numbers",
            "x" * 40,
            "foo-snap-bar",
        ],
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
            pytest.param(
                "snap-foo",
                "component names cannot start with the reserved prefix 'snap-'",
                id="reserved prefix",
            ),
            pytest.param("123456", "component names can only use", id="no letters"),
            ("name_with_underscores", "component names can only use"),
            ("name-with-UPPERCASE", "component names can only use"),
            ("name with spaces", "component names can only use"),
            ("name-with-$symbols", "component names can only use"),
            ("-name-starts-with-hyphen", "component names cannot start with a hyphen"),
            ("name-ends-with-hyphen-", "component names cannot end with a hyphen"),
            (
                "name-has--two-hyphens",
                "component names cannot have two hyphens in a row",
            ),
            ("x" * 41, "String should have at most 40 characters"),
        ],
    )
    def test_component_name_invalid(
        self, project, name, error, project_yaml_data, stub_component_data
    ):
        component = {name: stub_component_data}

        with pytest.raises(pydantic.ValidationError, match=error):
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

        error = "String should have at most 78 characters"
        with pytest.raises(pydantic.ValidationError, match=error):
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
                "invalid version: Valid versions consist of upper- and lower-case",
                id="'_' in version",
            ),
            pytest.param(
                "1=1",
                "invalid version: Valid versions consist of upper- and lower-case",
                id="'=' in version",
            ),
            pytest.param(
                ".1",
                "invalid version: Valid versions consist of upper- and lower-case",
                id="cannot start with '.'",
            ),
            pytest.param(
                ":1",
                "invalid version: Valid versions consist of upper- and lower-case",
                id="cannot start with ':'",
            ),
            pytest.param(
                "+1",
                "invalid version: Valid versions consist of upper- and lower-case",
                id="cannot start with '+'",
            ),
            pytest.param(
                "~1",
                "invalid version: Valid versions consist of upper- and lower-case",
                id="cannot start with '~'",
            ),
            pytest.param(
                "-1",
                "invalid version: Valid versions consist of upper- and lower-case",
                id="cannot start with '-'",
            ),
            pytest.param(
                "1.",
                "invalid version: Valid versions consist of upper- and lower-case",
                id="cannot end with '.'",
            ),
            pytest.param(
                "1:",
                "invalid version: Valid versions consist of upper- and lower-case",
                id="cannot end with ':'",
            ),
            pytest.param(
                "1-",
                "invalid version: Valid versions consist of upper- and lower-case",
                id="cannot end with '-'",
            ),
            pytest.param(
                "x" * 33,
                # TODO: can we fix this wording for strings?
                "Value should have at most 32 items after validation, not 33",
                id="too large",
            ),
            pytest.param(
                "",
                "invalid version: Valid versions consist of upper- and lower-case",
                id="empty string",
            ),
        ],
    )
    def test_component_version_invalid(
        self, project, version, error, project_yaml_data, stub_component_data
    ):
        component = {"foo": stub_component_data}
        component["foo"]["version"] = version

        with pytest.raises(pydantic.ValidationError, match=error):
            project.unmarshal(project_yaml_data(components=component))

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
