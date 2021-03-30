# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2021 Canonical Ltd
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

from textwrap import dedent
from unittest import mock

import pytest
from testtools import TestCase
from testtools.matchers import Contains, Equals

# required for schema format checkers
import snapcraft.internal.project_loader._config  # noqa: F401
import snapcraft.yaml_utils.errors
from snapcraft.project._schema import Validator

from . import ProjectBaseTest


def get_data():
    return {
        "name": "my-package-1",
        "base": "core18",
        "version": "1.0-snapcraft1~ppa1",
        "summary": "my summary less that 79 chars",
        "description": "description which can be pretty long",
        "adopt-info": "part1",
        "parts": {"part1": {"plugin": "project", "parse-info": ["test-metadata-file"]}},
    }


@pytest.fixture
def data():
    """Return snapcraft.yaml data to validate."""
    return get_data()


class ValidationBaseTest(TestCase):
    def setUp(self):
        super().setUp()

        self.data = get_data()


class ValidationTest(ValidationBaseTest):
    def test_summary_too_long(self):
        self.data["summary"] = "a" * 80
        raised = self.assertRaises(
            snapcraft.yaml_utils.errors.YamlValidationError,
            Validator(self.data).validate,
        )

        expected_message = (
            "The 'summary' property does not match the required schema: "
            "'{}' is too long (maximum length is 78)"
        ).format(self.data["summary"])
        self.assertThat(raised.message, Equals(expected_message), message=self.data)

    def test_apps_required_properties(self):
        self.data["apps"] = {"service1": {}}

        raised = self.assertRaises(
            snapcraft.yaml_utils.errors.YamlValidationError,
            Validator(self.data).validate,
        )

        expected_message = (
            "The 'apps/service1' property does not match the "
            "required schema: 'command' is a required "
            "property"
        )
        self.assertThat(raised.message, Equals(expected_message), message=self.data)

    def test_schema_file_not_found(self):
        mock_the_open = mock.mock_open()
        mock_the_open.side_effect = FileNotFoundError()

        with mock.patch("snapcraft.project._schema.open", mock_the_open, create=True):
            raised = self.assertRaises(
                snapcraft.yaml_utils.errors.YamlValidationError, Validator, self.data
            )

        expected_message = "snapcraft validation file is missing from installation path"
        self.assertThat(raised.message, Equals(expected_message))

    def test_valid_app_daemons(self):
        self.data["apps"] = {
            "service1": {"command": "binary1 start", "daemon": "simple"},
            "service2": {
                "command": "binary2",
                "stop-command": "binary2 --stop",
                "daemon": "simple",
            },
            "service3": {"command": "binary3", "daemon": "forking"},
            "service4": {
                "command": "binary4",
                "daemon": "simple",
                "restart-condition": "always",
            },
            "service5": {"command": "binary5", "daemon": "notify"},
            "service6": {
                "command": "binary6",
                "post-stop-command": "binary6 --post-stop",
                "daemon": "simple",
            },
            "service7": {
                "command": "binary7",
                "reload-command": "binary7 --reload",
                "daemon": "simple",
            },
            "service8": {
                "command": "binary8",
                "daemon": "dbus",
                "bus-name": "org.test.snapcraft",
            },
            "service9": {
                "command": "binary9",
                "daemon": "simple",
                "start-timeout": "1s",
            },
            "service10": {
                "command": "binary10",
                "daemon": "simple",
                "stop-timeout": "1s",
            },
            "service11": {
                "command": "binary11",
                "daemon": "simple",
                "restart-delay": "1s",
            },
            "service12": {
                "command": "binary12",
                "daemon": "simple",
                "watchdog-timeout": "1s",
            },
            "service13": {
                "command": "binary13",
                "daemon": "oneshot",
                "timer": "mon,10:00-12:00",
            },
            "service14": {
                "command": "binary14",
                "daemon": "simple",
                "restart-condition": "on-watchdog",
                "watchdog-timeout": "30s",
            },
            "service15": {
                "command": "binary15",
                "daemon": "simple",
                "install-mode": "enable",
            },
            "service16": {
                "command": "binary16",
                "daemon": "simple",
                "install-mode": "disable",
            },
        }

        Validator(self.data).validate()

    def test_invalid_restart_condition(self):
        self.data["apps"] = {
            "service1": {
                "command": "binary1",
                "daemon": "simple",
                "restart-condition": "on-tuesday",
            }
        }

        raised = self.assertRaises(
            snapcraft.yaml_utils.errors.YamlValidationError,
            Validator(self.data).validate,
        )

        self.assertThat(
            str(raised),
            Contains(
                "The 'apps/service1/restart-condition' property does not match "
                "the required schema: 'on-tuesday' is not one of ['on-success', "
                "'on-failure', 'on-abnormal', 'on-abort', 'on-watchdog', 'always', "
                "'never']"
            ),
        )

    def test_missing_required_property_and_missing_adopt_info(self):
        del self.data["summary"]
        del self.data["adopt-info"]

        raised = self.assertRaises(
            snapcraft.yaml_utils.errors.YamlValidationError,
            Validator(self.data).validate,
        )

        expected_message = (
            "'adopt-info' is a required property or 'summary' is a required property"
        )
        self.assertThat(raised.message, Equals(expected_message), message=self.data)

    def test_invalid_install_mode(self):
        self.data["apps"] = {
            "service": {
                "command": "binary",
                "daemon": "simple",
                "install-mode": "invalid",
            }
        }

        raised = self.assertRaises(
            snapcraft.yaml_utils.errors.YamlValidationError,
            Validator(self.data).validate,
        )

        expected_message = (
            "The 'apps/service/install-mode' property does not match the required schema: "
            "'invalid' is not one of ['enable', 'disable']"
        )
        self.assertThat(raised.message, Equals(expected_message), message=self.data)


@pytest.mark.parametrize(
    "option,value",
    [
        ("stop-command", "binary1 --stop"),
        ("post-stop-command", "binary1 --post-stop"),
        ("before", ["service1"]),
        ("after", ["service2"]),
    ],
)
def test_daemon_dependency(data, option, value):
    data["apps"] = {"service1": {"command": "binary1", option: value}}

    with pytest.raises(snapcraft.yaml_utils.errors.YamlValidationError) as error:
        Validator(data).validate()

    assert str(error.value).endswith(
        "The 'apps/service1' property does not match the required schema: "
        f"'daemon' is a dependency of {option!r}"
    )


@pytest.mark.parametrize("key", ["name", "parts"])
def test_required_properties(data, key):
    del data[key]

    with pytest.raises(snapcraft.yaml_utils.errors.YamlValidationError) as error:
        Validator(data).validate()

    assert f"{key!r} is a required property" in str(error.value)


class TestInvalidNames:
    e1 = "not a valid snap name. Snap names can only use ASCII lowercase letters, numbers, and hyphens, and must have at least one letter."
    e2 = "not a valid snap name. Snap names cannot start with a hyphen."
    e3 = "not a valid snap name. Snap names cannot end with a hyphen."
    e4 = "not a valid snap name. Snap names cannot have two hyphens in a row."
    e5 = "too long (maximum length is 40)"

    scenarios = [
        # snapcraft's existing unit tests
        ("existing test #1", dict(name="package@awesome", err=e1)),
        ("existing test #2", dict(name="something.another", err=e1)),
        ("existing test #3", dict(name="_hideme", err=e1)),
        ("existing test #4", dict(name="-no", err=e2)),
        ("existing test #5", dict(name="a:a", err=e1)),
        ("existing test #6", dict(name="123", err=e1)),
        # this one manages to fail every validation test except type
        ("order check", dict(name="-----------------------------------------", err=e5)),
        # from snapd's unit tests (except those covered by above)
        ("name cannot be empty", dict(name="", err=e1)),
        (
            "name cannot be too long",
            dict(name="aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa", err=e5),
        ),
        ("dashes alone are not a name", dict(name="-", err=e1)),
        ("dashes alone are not a name, take 2", dict(name="--", err=e1)),
        ("double dashes in a name are not ok", dict(name="a--a", err=e4)),
        ("name should not end with a dash", dict(name="a-", err=e3)),
        ("name cannot have any spaces in it, #1", dict(name="a ", err=e1)),
        ("name cannot have any spaces in it, #2", dict(name=" a", err=e1)),
        ("name cannot have any spaces in it, #3", dict(name="a a", err=e1)),
        ("a number alone is not a name", dict(name="0", err=e1)),
        ("just numbers and dashes", dict(name="1-2-3", err=e1)),
        ("plain ASCII #1", dict(name="реасе", err=e1)),
        ("plain ASCII #2", dict(name="日本語", err=e1)),
        ("plain ASCII #3", dict(name="한글", err=e1)),
        ("plain ASCII #4", dict(name="ру́сский язы́к", err=e1)),
        # from review-tools', except as covered
        (
            "stress the regexper",
            dict(name="u-9490371368748654323415773467328453675-", err=e3),
        ),
        ("review-tools bad", dict(name="foo?bar", err=e1)),
        ("review-tools bad1", dict(name="foo/bar", err=e1)),
        ("review-tools bad6", dict(name="foo-Bar", err=e1)),
    ]

    def test(self, data, name, err):
        data["name"] = name

        with pytest.raises(snapcraft.yaml_utils.errors.YamlValidationError) as error:
            Validator(data).validate()

        assert str(error.value).endswith(
            f"The 'name' property does not match the required schema: {name!r} is {err}"
        )


@pytest.mark.parametrize("snap_type", ["app", "base", "gadget", "kernel", "snapd"])
def test_valid_types(data, snap_type):
    data["type"] = snap_type
    if snap_type in ("base", "kernel", "snapd"):
        data.pop("base")

    Validator(data).validate()


_BASE_TYPE_MSG = (
    "must be one of base: <base> and type: <app|gadget>, "
    "base: bare (with a build-base), "
    "or type: <base|kernel|snapd> (without a base)"
)
_TYPE_ENUM_TMPL = (
    "The 'type' property does not match the required schema: '{}' is not one of "
    "['app', 'base', 'gadget', 'kernel', 'snapd']"
)


@pytest.mark.parametrize("snap_type", ["apps", "framework", "platform", "oem"])
def test_invalid_types(data, snap_type):
    data["type"] = snap_type

    with pytest.raises(snapcraft.yaml_utils.errors.YamlValidationError):
        Validator(data).validate()


def test_type_base_and_no_base(data):
    data.pop("base")
    data["type"] = "base"

    Validator(data).validate()


def test_type_base_and_base(data):
    data["type"] = "base"

    with pytest.raises(snapcraft.yaml_utils.errors.YamlValidationError) as error:
        Validator(data).validate()

    assert _BASE_TYPE_MSG in str(error.value)


def test_build_base_and_base(data):
    data["build-base"] = "fake-base"

    Validator(data).validate()


def test_build_base_and_type_base(data):
    data.pop("base")
    data["type"] = "base"
    data["build-base"] = "fake-base"

    Validator(data).validate()


def test_build_base_and_base_bare(data):
    data["base"] = "bare"
    data["build-base"] = "fake-base"

    Validator(data).validate()


@pytest.mark.parametrize(
    "name",
    [
        "1",
        "a",
        "aa",
        "aaa",
        "aaaa",
        "Aa",
        "aA",
        "1a",
        "a1",
        "1-a",
        "a-1",
        "a-a",
        "aa-a",
        "a-aa",
        "a-b-c",
        "0a-a",
        "a-0a",
    ],
)
def test_valid_app_names(data, name):
    data["apps"] = {name: {"command": "foo"}}

    Validator(data).validate()


@pytest.mark.parametrize(
    "name",
    [
        "qwe#rty",
        "qwe_rty",
        "queue rty",
        "queue  rty",
        "",
        "-",
        "--",
        "a--a",
        "a-",
        "a ",
        " a",
        "a a",
        "日本語",
        "한글",
        "ру́сский язы́к",
        "ໄຂ່​ອີ​ສ​ເຕີ້",
        ":a",
        "a:",
        "a:a",
        "_a",
        "a_",
        "a_a",
    ],
)
def test_invalid_app_names(data, name):
    data["apps"] = {name: {"command": "1"}}

    with pytest.raises(snapcraft.yaml_utils.errors.YamlValidationError) as error:
        Validator(data).validate()

    expected_message = (
        f"The 'apps' property does not match the required schema: {name!r} is "
        "not a valid app name. App names consist of upper- and lower-case "
        "alphanumeric characters and hyphens. They cannot start or end "
        "with a hyphen."
    )
    assert expected_message in str(error.value)


@pytest.mark.parametrize(
    "condition",
    [
        "always",
        "on-success",
        "on-failure",
        "on-abnormal",
        "on-abort",
        "on-watchdog",
        "never",
    ],
)
def test_valid_restart_conditions(data, condition):
    data["apps"] = {"service1": {"command": "binary1", "daemon": "simple"}}
    data["apps"]["service1"]["restart-condition"] = condition

    Validator(data).validate()


_REFRESH_MODES = ["endure", "restart"]


@pytest.mark.parametrize("mode", _REFRESH_MODES)
def test_valid_refresh_modes(data, mode):
    data["apps"] = {
        "service1": {"command": "binary1", "daemon": "simple", "refresh-mode": mode}
    }
    Validator(data).validate()


@pytest.mark.parametrize("mode", _REFRESH_MODES)
def test_refresh_mode_daemon_missing_errors(data, mode):
    data["apps"] = {"service1": {"command": "binary1", "refresh-mode": mode}}

    with pytest.raises(snapcraft.yaml_utils.errors.YamlValidationError):
        Validator(data).validate()


_STOP_MODES = [
    "sigterm",
    "sigterm-all",
    "sighup",
    "sighup-all",
    "sigusr1",
    "sigusr1-all",
    "sigusr2",
    "sigusr2-all",
]


@pytest.mark.parametrize("mode", _STOP_MODES)
def test_valid_modes(data, mode):
    data["apps"] = {
        "service1": {"command": "binary1", "daemon": "simple", "stop-mode": mode}
    }
    Validator(data).validate()


@pytest.mark.parametrize("mode", _STOP_MODES)
def test_daemon_missing_errors(data, mode):
    data["apps"] = {"service1": {"command": "binary1", "stop-mode": mode}}

    with pytest.raises(snapcraft.yaml_utils.errors.YamlValidationError):
        Validator(data).validate()


@pytest.mark.parametrize(
    "name",
    [
        "qwe#rty",
        "qwe_rty",
        "queue rty",
        "queue  rty",
        "Hi",
        "",
        "-",
        "--",
        "a--a",
        "a-",
        "a ",
        " a",
        "a a",
        "日本語",
        "한글",
        "ру́сский язы́к",
        "ໄຂ່​ອີ​ສ​ເຕີ້",
        ":a",
        "a:",
        "a:a",
        "_a",
        "a_",
        "a_a",
    ],
)
def test_invalid_hook_names(data, name):
    data["hooks"] = {name: {"plugs": ["network"]}}

    with pytest.raises(snapcraft.yaml_utils.errors.YamlValidationError) as error:
        Validator(data).validate()

    expected_message = (
        f"The 'hooks' property does not match the required schema: {name!r} is "
        "not a valid hook name. Hook names consist of lower-case "
        "alphanumeric characters and hyphens. They cannot start or end "
        "with a hyphen."
    )
    assert expected_message in str(error.value)


@pytest.mark.parametrize(
    "name", ["plugins", "qwe#rty", "qwe_rty", "queue rty", "queue  rty", "part/sub"]
)
def test_invalid_part_names(data, name):
    data["parts"] = {name: {"plugin": "nil"}}

    with pytest.raises(snapcraft.yaml_utils.errors.YamlValidationError) as error:
        Validator(data).validate()

    expected_message = (
        f"The 'parts' property does not match the required schema: {name!r} is "
        "not a valid part name. Part names consist of lower-case "
        "alphanumeric characters, hyphens and plus signs. "
        "As a special case, 'plugins' is also not a valid part name."
    )
    assert str(error.value).endswith(expected_message)


class TestInvalidArchitectures:

    scenarios = [
        (
            "single string",
            {"architectures": "amd64", "message": "'amd64' is not of type 'array'"},
        ),
        (
            "unknown object properties",
            {
                "architectures": [{"builds-on": ["amd64"], "runs-on": ["amd64"]}],
                "message": "'build-on' is a required property and additional "
                "properties are not allowed",
            },
        ),
        (
            "omit build-on",
            {
                "architectures": [{"run-on": ["amd64"]}],
                "message": "'build-on' is a required property",
            },
        ),
        (
            "build on all and others",
            {
                "architectures": [{"build-on": ["amd64", "all"]}],
                "message": "'all' can only be used within 'build-on' by itself, "
                "not with other architectures",
            },
        ),
        (
            "run on all and others",
            {
                "architectures": [{"build-on": ["amd64"], "run-on": ["amd64", "all"]}],
                "message": "'all' can only be used within 'run-on' by itself, "
                "not with other architectures",
            },
        ),
        (
            "run on all and more objects",
            {
                "architectures": [
                    {"build-on": ["amd64"], "run-on": ["all"]},
                    {"build-on": ["i396"], "run-on": ["i386"]},
                ],
                "message": "one of the items has 'all' in 'run-on', but there are "
                "2 items: upon release they will conflict. 'all' "
                "should only be used if there is a single item",
            },
        ),
        (
            "build on all and more objects",
            {
                "architectures": [{"build-on": ["all"]}, {"build-on": ["i396"]}],
                "message": "one of the items has 'all' in 'build-on', but there "
                "are 2 items: snapcraft doesn't know which one to use. "
                "'all' should only be used if there is a single item",
            },
        ),
        (
            "multiple builds run on same arch",
            {
                "architectures": [
                    {"build-on": ["amd64"], "run-on": ["amd64"]},
                    {"build-on": ["i396"], "run-on": ["amd64", "i386"]},
                ],
                "message": "multiple items will build snaps that claim to run on "
                "'amd64'",
            },
        ),
        (
            "multiple builds run on same arch with implicit run-on",
            {
                "architectures": [
                    {"build-on": ["amd64"]},
                    {"build-on": ["i396"], "run-on": ["amd64", "i386"]},
                ],
                "message": "multiple items will build snaps that claim to run on "
                "'amd64'",
            },
        ),
        (
            "mixing forms",
            {
                "architectures": [
                    "amd64",
                    {"build-on": ["i386"], "run-on": ["amd64", "i386"]},
                ],
                "message": "every item must either be a string or an object",
            },
        ),
        (
            "build on all run on specific",
            {
                "architectures": [
                    {"build-on": ["all"], "run-on": ["amd64"]},
                    {"build-on": ["all"], "run-on": ["i386"]},
                ],
                "message": "one of the items has 'all' in 'build-on', but there "
                "are 2 items: snapcraft doesn't know which one to use. "
                "'all' should only be used if there is a single item",
            },
        ),
        (
            "build on overlap",
            {
                "architectures": [
                    {"build-on": ["amd64", "i386"], "run-on": ["i386"]},
                    {"build-on": ["amd64"], "run-on": ["amd64"]},
                ],
                "message": "'amd64' is present in the 'build-on' of multiple "
                "items, which means snapcraft doesn't know which "
                "'run-on' to use when building on that architecture",
            },
        ),
    ]

    def test(self, data, architectures, message):
        data["architectures"] = architectures

        with pytest.raises(snapcraft.yaml_utils.errors.YamlValidationError) as error:
            Validator(data).validate()

        assert message in str(error.value)


class IconTest(ProjectBaseTest):
    def test_invalid_yaml_invalid_icon_extension(self):
        raised = self.assertValidationRaises(
            dedent(
                """\
            name: test
            base: core18
            version: "1"
            summary: test
            description: test
            icon: icon.foo
            confinement: strict
            grade: stable

            parts:
              part1:
                plugin: nil
            """
            )
        )

        self.assertThat(
            raised.message, Equals("icon 'icon.foo' must be either a .png or a .svg")
        )


@pytest.mark.parametrize("title", ["a title", "A Title", "T" * 40, "💩" * 40, "’" * 40])
def test_valid_title(data, title):
    data["title"] = title

    Validator(data).validate()


_EXPECTED_ERROR_TEMPLATE = {
    "number": (
        "Issues while validating snapcraft.yaml: The 'title' property "
        "does not match the required schema: {} is not of type 'string'"
    ),
    "length": (
        "Issues while validating snapcraft.yaml: The 'title' property "
        "does not match the required schema: {!r} is too long "
        "(maximum length is 40)"
    ),
}


@pytest.mark.parametrize(
    "title,error_template",
    [(1, "number"), ("T" * 41, "length"), ("💩" * 41, "length"), ("’" * 41, "length")],
)
def test_invalid_title(data, title, error_template):
    data["title"] = title

    with pytest.raises(snapcraft.yaml_utils.errors.YamlValidationError) as error:
        Validator(data).validate()

    assert _EXPECTED_ERROR_TEMPLATE[error_template].format(title) in str(error.value)


class OrganizeTest(ProjectBaseTest):
    def test_yaml_organize_value_none(self):
        raised = self.assertValidationRaises(
            dedent(
                """\
            name: test
            base: core18
            version: "1"
            summary: test
            description: nothing
            confinement: strict

            parts:
              part1:
                plugin: nil
                organize:
                  foo:
            """
            )
        )

        self.assertThat(
            str(raised),
            Contains(
                "The 'parts/part1/organize/foo' property does not match the "
                "required schema: None is not of type 'string'"
            ),
        )

    def test_yaml_organize_value_empty(self):
        raised = self.assertValidationRaises(
            dedent(
                """\
            name: test
            base: core18
            version: "1"
            summary: test
            description: nothing
            confinement: strict

            parts:
              part1:
                plugin: nil
                organize:
                  foo: ''
            """
            )
        )

        self.assertThat(
            str(raised),
            Contains(
                "The 'parts/part1/organize/foo' property does not match the "
                "required schema: '' is too short (minimum length is 1)"
            ),
        )


class VersionTest(ProjectBaseTest):
    def test_invalid_yaml_version_too_long(self):
        raised = self.assertValidationRaises(
            dedent(
                """\
            name: test
            base: core18
            version: 'abcdefghijklmnopqrstuvwxyz1234567' # Max is 32 in the store
            summary: test
            description: test
            confinement: strict
            grade: stable
            parts:
              part1:
                plugin: nil
        """
            )
        )  # noqa: E501

        self.assertThat(
            raised.message,
            Equals(
                "The 'version' property does not match the required "
                "schema: 'abcdefghijklmnopqrstuvwxyz1234567' is too long "
                "(maximum length is 32)"
            ),
        )


@pytest.mark.parametrize(
    "version", ["buttered-popcorn", "1.2.3", "v12.4:1:2~", "HeLlo", "v+"]
)
def test_valid_version(data, version):
    data["version"] = version

    Validator(data).validate()


@pytest.mark.parametrize(
    "version",
    [
        "'*'",
        "''",
        "':v'",
        "'.v'",
        "'+v'",
        "'~v'",
        "'_v'",
        "'-v'",
        "'v:'",
        "'v.'",
        "'v_'",
        "'v-'",
        "'underscores_are_bad'",
    ],
)
def test_invalid_version(data, version):
    data["version"] = version

    with pytest.raises(snapcraft.yaml_utils.errors.YamlValidationError) as error:
        Validator(data).validate()

    expected_message = (
        "The 'version' property does not match the required "
        f"schema: {version!r} is not a valid snap version string. Snap versions "
        "consist of upper- and lower-case alphanumeric characters, "
        "as well as periods, colons, plus signs, tildes, and "
        "hyphens. They cannot begin with a period, colon, plus "
        "sign, tilde, or hyphen. They cannot end with a period, "
        "colon, or hyphen."
    )
    assert expected_message in str(error.value)


def test_invalid_version_type(data):
    data["version"] = 0.1

    with pytest.raises(snapcraft.yaml_utils.errors.YamlValidationError) as error:
        Validator(data).validate()

    expected_message = (
        "The 'version' property does not match the required "
        "schema: snap versions need to be strings. They must "
        "also be wrapped in quotes when the value will be "
        "interpreted by the YAML parser as a non-string. "
        "Examples: '1', '1.2', '1.2.3', git (will be replaced "
        "by a git describe based version string)."
    )
    assert expected_message in str(error.value)


def test_invalid_version_length(data):
    data["version"] = "this.is.a.really.too.long.version"

    with pytest.raises(snapcraft.yaml_utils.errors.YamlValidationError) as error:
        Validator(data).validate()

    expected_message = (
        "The 'version' property does not match the required "
        "schema: 'this.is.a.really.too.long.version' is too long "
        "(maximum length is 32)"
    )
    assert expected_message in str(error.value)


class EnvironmentTest(ProjectBaseTest):
    def test_valid_environment(self):
        snapcraft_yaml = self.assertValidationPasses(
            dedent(
                dedent(
                    """\
            name: project-name
            base: core18
            version: "1"
            summary: test
            description: test
            confinement: strict
            environment:
                GLOBAL: "1"
                OTHER: valid-value

            apps:
              app1:
                command: app1
                environment:
                  LOCALE: C
                  PLUGIN_PATH: $SNAP_USER_DATA/plugins

            parts:
              main:
                plugin: nil
                source: .
            """
                )
            )
        )

        self.assertThat(
            snapcraft_yaml["environment"], Equals(dict(GLOBAL="1", OTHER="valid-value"))
        )
        self.assertThat(
            snapcraft_yaml["apps"]["app1"]["environment"],
            Equals(dict(LOCALE="C", PLUGIN_PATH="$SNAP_USER_DATA/plugins")),
        )

    def test_invalid_environment(self):
        raised = self.assertValidationRaises(
            dedent(
                """\
            name: project-name
            base: core18
            version: "1"
            summary: test
            description: test
            confinement: strict
            environment:
                INVALID:
                    - 1
                    - 2

            parts:
              main:
                source: .
                plugin: nil
            """
            )
        )

        self.assertRegex(
            raised.message,
            r"The 'environment/INVALID' property does not match the required "
            r"schema: \[1, 2\].*",
        )


@pytest.mark.parametrize("compression", ["lzo", "xz"])
def test_valid_compression(data, compression):
    data["compression"] = compression

    Validator(data).validate()


@pytest.mark.parametrize("compression", ["lzma", "gz", "rar"])
def test_invalid_compression(data, compression):
    data["compression"] = compression

    with pytest.raises(snapcraft.yaml_utils.errors.YamlValidationError) as error:
        Validator(data).validate()

    expected_message = (
        "The 'compression' property does not match the required "
        f"schema: {compression!r} is not one of ['lzo', 'xz']"
    )
    assert expected_message in str(error.value)


@pytest.mark.parametrize("confinement", ["strict", "devmode", "classic"])
def test_valid_confinement(data, confinement):
    data["confinement"] = confinement

    Validator(data).validate()


@pytest.mark.parametrize("confinement", ["foo", "strict-", "_devmode"])
def test_invalid_confinement(data, confinement):
    data["confinement"] = confinement

    with pytest.raises(snapcraft.yaml_utils.errors.YamlValidationError) as error:
        Validator(data).validate()

    expected_message = (
        "The 'confinement' property does not match the required "
        f"schema: {confinement!r} is not one of ['classic', 'devmode', "
        "'strict']"
    )
    assert expected_message in str(error.value)


@pytest.mark.parametrize("desc", ["test", "multi\nline\n"])
def test_valid_description(data, desc):
    data["description"] = desc
    Validator(data).validate()


@pytest.mark.parametrize("desc", [""])
def test_invalid_description(data, desc):
    data["description"] = desc

    with pytest.raises(snapcraft.yaml_utils.errors.YamlValidationError) as error:
        Validator(data).validate()

    expected_message = (
        "The 'description' property does not match the required "
        f"schema: {desc!r} is not a valid description string"
    )
    assert expected_message in str(error.value)


@pytest.mark.parametrize("grade", ["stable", "devel"])
def test_valid_grade(data, grade):
    data["grade"] = grade

    Validator(data).validate()


@pytest.mark.parametrize("grade", ["foo", "strict-", "_devmode"])
def test_invalid_grade(data, grade):
    data["grade"] = grade

    with pytest.raises(snapcraft.yaml_utils.errors.YamlValidationError) as error:
        Validator(data).validate()

    expected_message = (
        "The 'grade' property does not match the required "
        f"schema: {grade!r} is not one of ['stable', 'devel']"
    )
    assert expected_message in str(error.value)


@pytest.mark.parametrize("epoch", [0, "0", "1*", 1, "1", "400*", "1234"])
def test_valid_epoch(data, epoch):
    data["epoch"] = epoch

    Validator(data).validate()


@pytest.mark.parametrize(
    "epoch",
    [
        "0*",
        "_",
        "1-",
        "1+",
        "-1",
        "-1*",
        "a",
        "1a",
        "1**",
        '"01"',
        "1.2",
        '"1.2"',
        "[1]",
    ],
)
def test_invalid_epoch(data, epoch):
    data["epoch"] = epoch

    with pytest.raises(snapcraft.yaml_utils.errors.YamlValidationError) as error:
        Validator(data).validate()

    expected_message = (
        f"{epoch!r} is not a 'epoch' (epochs are positive integers "
        "followed by an optional asterisk)"
    )
    assert expected_message in str(error.value)


def test_valid_license(data):
    data["license"] = "MIT-0"

    Validator(data).validate()


def test_invalid_license(data):
    data["license"] = 1234

    with pytest.raises(snapcraft.yaml_utils.errors.YamlValidationError) as error:
        Validator(data).validate()

    expected_message = (
        "The 'license' property does not match the required schema: "
        "1234 is not of type 'string'"
    )
    assert expected_message in str(error.value)


@pytest.mark.parametrize("adapter", ["none", "legacy", "full"])
def test_valid_adapter(data, adapter):
    data["apps"] = {"foo": {"command": "foo", "adapter": adapter}}

    Validator(data).validate()


@pytest.mark.parametrize("adapter", ["NONE", "F", "Full"])
def test_invalid_adapter(data, adapter):
    data["apps"] = {"foo": {"command": "foo", "adapter": adapter}}

    with pytest.raises(snapcraft.yaml_utils.errors.YamlValidationError) as error:
        Validator(data).validate()

    expected_message = "The 'apps/foo/adapter' property does not match"
    assert expected_message in str(error.value)


@pytest.mark.parametrize(
    "build_environment",
    ["a string", ["a string"], [{"k1": "v1", "k2": "v2"}], [{"k1": 5}]],
)
def test_invalid_part_build_environment_key_type(data, build_environment):
    data["parts"]["part1"]["build-environment"] = build_environment

    with pytest.raises(snapcraft.yaml_utils.errors.YamlValidationError):
        Validator(data).validate()


@pytest.mark.parametrize(
    "command_chain", ["a string", [1], ["test chain"], ["test'chain"]]
)
def test_invalid_command_chain(data, command_chain):
    data["apps"] = {"foo": {"command": "foo", "command-chain": command_chain}}

    with pytest.raises(snapcraft.yaml_utils.errors.YamlValidationError) as error:
        Validator(data).validate()

    expected_message = "The 'apps/foo/command-chain"
    assert expected_message in str(error.value)


class SystemUsernamesTests(ProjectBaseTest):
    def test_yaml_valid_system_usernames_long(self):
        self.assertValidationPasses(
            dedent(
                """\
                name: test
                base: core18
                version: "1"
                summary: test
                description: nothing
                license: MIT
                parts:
                  part1:
                    plugin: nil
                system-usernames:
                  snap_daemon:
                    scope: shared
                """
            )
        )

    def test_yaml_valid_system_usernames_short(self):
        self.assertValidationPasses(
            dedent(
                """\
                name: test
                base: core18
                version: "1"
                summary: test
                description: nothing
                license: MIT
                parts:
                  part1:
                    plugin: nil
                system-usernames:
                  snap_daemon: shared
                """
            )
        )

    def test_invalid_yaml_invalid_username(self):
        raised = self.assertValidationRaises(
            dedent(
                """\
            name: test
            base: core18
            version: "1"
            summary: test
            description: nothing
            license: MIT
            parts:
              part1:
                plugin: nil
            system-usernames:
              snap_user: shared
            """
            )
        )

        self.assertThat(
            raised.message,
            Equals(
                "The 'system-usernames' property does not match the required schema: 'snap_user' is not a valid system-username."
            ),
        )

    def test_invalid_yaml_invalid_short_scope(self):
        raised = self.assertValidationRaises(
            dedent(
                """\
            name: test
            base: core18
            version: "1"
            summary: test
            description: nothing
            license: MIT
            parts:
              part1:
                plugin: nil
            system-usernames:
              snap_daemon: invalid-scope
            """
            )
        )

        self.assertThat(
            raised.message,
            Equals(
                "The 'system-usernames/snap_daemon' property does not match the required schema: 'invalid-scope' is not valid under any of the given schemas"
            ),
        )

    def test_invalid_yaml_invalid_long_scope(self):
        raised = self.assertValidationRaises(
            dedent(
                """\
            name: test
            base: core18
            version: "1"
            summary: test
            description: nothing
            license: MIT
            parts:
              part1:
                plugin: nil
            system-usernames:
              snap_daemon:
                scope: invalid-scope
            """
            )
        )

        self.assertThat(
            raised.message,
            Equals(
                "The 'system-usernames/snap_daemon' property does not match the required schema: OrderedDict([('scope', 'invalid-scope')]) is not valid under any of the given schemas"
            ),
        )


class PackageManagement(ProjectBaseTest):
    def test_yaml_valid_apt_repositories(self):
        self.assertValidationPasses(
            dedent(
                """\
                name: test
                base: core18
                version: "1"
                summary: test
                description: nothing
                license: MIT
                parts:
                  part1:
                    plugin: nil
                package-repositories:
                  - type: apt
                    ppa: user/test-ppa
                  - type: apt
                    architectures: [amd64, i386]
                    components: [main, multiverse]
                    formats: [deb, deb-src]
                    key-id: test-key-id
                    key-server: keyserver.ubuntu.com
                    url: http://archive.ubuntu.com/ubuntu
                    suites: [test, test-updates, test-security]
                  - type: apt
                    key-id: test-key-id
                    url: http://archive.ubuntu.com/ubuntu
                    suites: [bionic, bionic-updates]
                  - type: apt
                    key-id: test-key-id
                    path: foo
                    url: http://archive.ubuntu.com/ubuntu
                """
            )
        )


class TestInvalidAptConfigurations:
    scenarios = [
        (
            "ppa extra invalid field",
            dict(
                packages=[{"type": "apt", "ppa": "test/ppa", "invalid": "invalid"}],
                message_contains="The 'package-repositories[0]' property does not match the required schema:",
            ),
        ),
        (
            "deb extra invalid field",
            dict(
                packages=[
                    {
                        "type": "apt",
                        "components": ["main"],
                        "key-id": "test-key-id",
                        "url": "http://archive.ubuntu.com/ubuntu",
                        "suites": ["test", "test-updates", "test-security"],
                        "invalid": "invalid",
                    }
                ],
                message_contains="The 'package-repositories[0]' property does not match the required schema:",
            ),
        ),
        (
            "deb missing field: key-id",
            dict(
                packages=[
                    {
                        "type": "apt",
                        "components": ["main"],
                        "url": "http://archive.ubuntu.com/ubuntu",
                        "suites": ["test", "test-updates", "test-security"],
                    }
                ],
                message_contains="The 'package-repositories[0]' property does not match the required schema:",
            ),
        ),
        (
            "deb missing field: url",
            dict(
                packages=[
                    {
                        "type": "apt",
                        "components": ["main"],
                        "key-id": "test-key-id",
                        "suites": ["test", "test-updates", "test-security"],
                    }
                ],
                message_contains="The 'package-repositories[0]' property does not match the required schema:",
            ),
        ),
        (
            "deb invalid deb type",
            dict(
                packages=[
                    {
                        "type": "apt",
                        "format": ["invalid"],
                        "components": ["main"],
                        "key-id": "test-key-id",
                        "url": "http://archive.ubuntu.com/ubuntu",
                        "suites": ["test", "test-updates", "test-security"],
                    }
                ],
                message_contains="The 'package-repositories[0]' property does not match the required schema:",
            ),
        ),
        (
            "deb invalid key-id",
            dict(
                packages=[
                    {
                        "type": "apt",
                        "components": ["main"],
                        "key-id": "\\*\\*",
                        "url": "http://archive.ubuntu.com/ubuntu",
                        "suites": ["test", "test-updates", "test-security"],
                    }
                ],
                message_contains="The 'package-repositories[0]' property does not match the required schema:",
            ),
        ),
        (
            "deb empty architectures",
            dict(
                packages=[
                    {
                        "type": "apt",
                        "components": ["main"],
                        "key-id": "test-key-id",
                        "url": "http://archive.ubuntu.com/ubuntu",
                        "suites": ["test", "test-updates", "test-security"],
                        "architectures": [],
                    }
                ],
                message_contains="The 'package-repositories[0]' property does not match the required schema:",
            ),
        ),
        (
            "deb empty suites",
            dict(
                packages=[
                    {
                        "type": "apt",
                        "components": ["main"],
                        "key-id": "test-key-id",
                        "url": "http://archive.ubuntu.com/ubuntu",
                        "suites": [],
                    }
                ],
                message_contains="The 'package-repositories[0]' property does not match the required schema:",
            ),
        ),
        (
            "ppa duplicate",
            dict(
                packages=[
                    {"type": "apt", "ppa": "dupe/check"},
                    {"type": "apt", "ppa": "dupe/check"},
                ],
                message_contains="has non-unique elements",
            ),
        ),
        (
            "deb duplicate",
            dict(
                packages=[
                    {
                        "type": "apt",
                        "components": ["main"],
                        "key-id": "test-key-id",
                        "url": "http://archive.ubuntu.com/ubuntu",
                        "suites": ["test", "test-updates", "test-security"],
                    },
                    {
                        "type": "apt",
                        "components": ["main"],
                        "key-id": "test-key-id",
                        "url": "http://archive.ubuntu.com/ubuntu",
                        "suites": ["test", "test-updates", "test-security"],
                    },
                ],
                message_contains="has non-unique elements",
            ),
        ),
    ]

    def test_invalid(self, data, packages, message_contains):
        data["package-repositories"] = packages

        with pytest.raises(snapcraft.yaml_utils.errors.YamlValidationError) as error:
            Validator(data).validate()

        assert message_contains in str(error.value)
