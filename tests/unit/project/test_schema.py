# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2019 Canonical Ltd
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

from testscenarios.scenarios import multiply_scenarios
from testtools.matchers import Contains, Equals, MatchesAny, MatchesRegex

from . import ProjectBaseTest
from snapcraft.project import errors
from snapcraft.project._schema import Validator
from tests import unit


class ValidationBaseTest(unit.TestCase):
    def setUp(self):
        super().setUp()

        patcher = mock.patch("os.path.exists")
        self.mock_path_exists = patcher.start()
        self.mock_path_exists.return_value = True
        self.addCleanup(patcher.stop)

        self.data = {
            "name": "my-package-1",
            "base": "core18",
            "version": "1.0-snapcraft1~ppa1",
            "summary": "my summary less that 79 chars",
            "description": "description which can be pretty long",
            "adopt-info": "part1",
            "parts": {
                "part1": {"plugin": "project", "parse-info": ["test-metadata-file"]}
            },
        }


class ValidationTest(ValidationBaseTest):
    def test_summary_too_long(self):
        self.data["summary"] = "a" * 80
        raised = self.assertRaises(
            errors.YamlValidationError, Validator(self.data).validate
        )

        expected_message = (
            "The 'summary' property does not match the required schema: "
            "'{}' is too long (maximum length is 78)"
        ).format(self.data["summary"])
        self.assertThat(raised.message, Equals(expected_message), message=self.data)

    def test_apps_required_properties(self):
        self.data["apps"] = {"service1": {}}

        raised = self.assertRaises(
            errors.YamlValidationError, Validator(self.data).validate
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
            raised = self.assertRaises(errors.YamlValidationError, Validator, self.data)

        expected_message = "snapcraft validation file is missing from installation path"
        self.assertThat(raised.message, Equals(expected_message))

    def test_icon_missing_is_valid_yaml(self):
        self.mock_path_exists.return_value = False

        Validator(self.data).validate()

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
            errors.YamlValidationError, Validator(self.data).validate
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
            errors.YamlValidationError, Validator(self.data).validate
        )

        expected_message = (
            "'adopt-info' is a required property or 'summary' is a required property"
        )
        self.assertThat(raised.message, Equals(expected_message), message=self.data)


class DaemonDependencyTest(ValidationBaseTest):

    scenarios = [
        ("stop-command", dict(option="stop-command", value="binary1 --stop")),
        (
            "post-stop-command",
            dict(option="post-stop-command", value="binary1 --post-stop"),
        ),
        ("before", dict(option="before", value=["service2"])),
        ("after", dict(option="after", value=["service2"])),
    ]

    def test_daemon_dependency(self):
        self.data["apps"] = {
            "service1": {"command": "binary1", self.option: self.value}
        }
        raised = self.assertRaises(
            errors.YamlValidationError, Validator(self.data).validate
        )

        self.assertThat(
            str(raised),
            Contains(
                "The 'apps/service1' property does not match the required schema: "
                "'daemon' is a dependency of '{}'".format(self.option)
            ),
        )


class RequiredPropertiesTest(ValidationBaseTest):

    scenarios = [(key, dict(key=key)) for key in ["name", "parts"]]

    def test_required_properties(self):
        data = self.data.copy()
        del data[self.key]

        raised = self.assertRaises(errors.YamlValidationError, Validator(data).validate)

        expected_message = "'{}' is a required property".format(self.key)
        self.assertThat(raised.message, Equals(expected_message), message=data)


class InvalidNamesTest(ValidationBaseTest):
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

    def test_invalid_names(self):
        data = self.data.copy()
        data["name"] = self.name

        raised = self.assertRaises(errors.YamlValidationError, Validator(data).validate)

        expected_message = (
            "The 'name' property does not match the required schema: {!r} is {}"
        ).format(self.name, self.err)
        self.assertThat(raised.message, Equals(expected_message), message=data)


class ValidTypesTest(ValidationBaseTest):

    scenarios = [
        (type_, dict(type_=type_))
        for type_ in ["app", "base", "gadget", "kernel", "snapd"]
    ]

    def test_valid_types(self):
        data = self.data.copy()
        data["type"] = self.type_
        if self.type_ in ("base", "kernel", "snapd"):
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


class InvalidTypesTest(ValidationBaseTest):

    scenarios = [
        (type_, dict(type_=type_)) for type_ in ["apps", "framework", "platform", "oem"]
    ]

    def test_invalid_types(self):
        data = self.data.copy()
        data["type"] = self.type_

        raised = self.assertRaises(errors.YamlValidationError, Validator(data).validate)

        self.assertThat(
            raised.message,
            # The schema is not loaded in order, so we can get different results depending on what is loaded first.
            MatchesAny(
                Equals(_BASE_TYPE_MSG), Equals(_TYPE_ENUM_TMPL.format(self.type_))
            ),
            message=data,
        )


class CombinedBaseTypeTest(ValidationBaseTest):
    def test_type_base_and_no_base(self):
        data = self.data.copy()
        data.pop("base")
        data["type"] = "base"

        Validator(data).validate()

    def test_type_base_and_base(self):
        data = self.data.copy()
        data["type"] = "base"

        raised = self.assertRaises(errors.YamlValidationError, Validator(data).validate)

        self.assertThat(raised.message, Equals(_BASE_TYPE_MSG), message=data)

    def test_build_base_and_base(self):
        data = self.data.copy()
        data["build-base"] = "fake-base"

        Validator(data).validate()

    def test_build_base_and_type_base(self):
        data = self.data.copy()
        data.pop("base")
        data["type"] = "base"
        data["build-base"] = "fake-base"

        Validator(data).validate()

    def test_build_base_and_base_bare(self):
        data = self.data.copy()
        data["base"] = "bare"
        data["build-base"] = "fake-base"

        Validator(data).validate()


class ValidRestartConditionsTest(ValidationBaseTest):

    scenarios = [
        (condition, dict(condition=condition))
        for condition in [
            "always",
            "on-success",
            "on-failure",
            "on-abnormal",
            "on-abort",
            "on-watchdog",
            "never",
        ]
    ]

    def test_valid_restart_conditions(self):
        self.data["apps"] = {"service1": {"command": "binary1", "daemon": "simple"}}
        self.data["apps"]["service1"]["restart-condition"] = self.condition
        Validator(self.data).validate()


class RefreshModeTest(ValidationBaseTest):

    refresh_modes = ["endure", "restart"]
    scenarios = [(mode, dict(mode=mode)) for mode in refresh_modes]

    def test_valid_modes(self):
        self.data["apps"] = {
            "service1": {
                "command": "binary1",
                "daemon": "simple",
                "refresh-mode": self.mode,
            }
        }
        Validator(self.data).validate()

    def test_daemon_missing_errors(self):
        self.data["apps"] = {
            "service1": {"command": "binary1", "refresh-mode": self.mode}
        }

        self.assertRaises(errors.YamlValidationError, Validator(self.data).validate)


class StopModeTest(ValidationBaseTest):

    stop_modes = [
        "sigterm",
        "sigterm-all",
        "sighup",
        "sighup-all",
        "sigusr1",
        "sigusr1-all",
        "sigusr2",
        "sigusr2-all",
    ]
    scenarios = [(mode, dict(mode=mode)) for mode in stop_modes]

    def test_valid_modes(self):
        self.data["apps"] = {
            "service1": {
                "command": "binary1",
                "daemon": "simple",
                "stop-mode": self.mode,
            }
        }
        Validator(self.data).validate()

    def test_daemon_missing_errors(self):
        self.data["apps"] = {"service1": {"command": "binary1", "stop-mode": self.mode}}

        self.assertRaises(errors.YamlValidationError, Validator(self.data).validate)


class InvalidStopRefreshModesTest(ValidationBaseTest):

    bad_values = [(mode, dict(mode=mode)) for mode in ["sigterm-bad", "123", "-----"]]
    keys = [(k, dict(key=k)) for k in ["stop-mode", "refresh-mode"]]

    scenarios = multiply_scenarios(keys, bad_values)

    def setUp(self):
        super().setUp()
        self.valid_modes = {
            "stop-mode": (
                "['sigterm', 'sigterm-all', 'sighup', 'sighup-all', "
                "'sigusr1', 'sigusr1-all', 'sigusr2', 'sigusr2-all']"
            ),
            "refresh-mode": "['endure', 'restart']",
        }

    def test_invalid_modes(self):
        self.data["apps"] = {
            "service1": {"command": "binary1", "daemon": "simple", self.key: self.mode}
        }
        raised = self.assertRaises(
            errors.YamlValidationError, Validator(self.data).validate
        )

        expected_message = (
            "The 'apps/service1/{}' property does not match the "
            "required schema: '{}' is not one of {}"
        ).format(self.key, self.mode, self.valid_modes[self.key])
        self.assertThat(raised.message, Equals(expected_message), message=self.data)


class InvalidAppNamesTest(ValidationBaseTest):

    scenarios = [
        (name, dict(name=name))
        for name in ["qwe#rty", "qwe_rty", "queue rty", "queue  rty"]
    ]

    def test_invalid_app_names(self):
        data = self.data.copy()
        data["apps"] = {self.name: {"command": "1"}}

        raised = self.assertRaises(errors.YamlValidationError, Validator(data).validate)

        expected_message = (
            "The 'apps' property does not match the required schema: {!r} is "
            "not a valid app name. App names consist of upper- and lower-case "
            "alphanumeric characters and hyphens. They cannot start or end "
            "with a hyphen."
        ).format(self.name)
        self.assertThat(raised.message, Equals(expected_message), message=data)


class InvalidHookNamesTest(ValidationBaseTest):

    scenarios = [
        (name, dict(name=name))
        for name in ["qwe#rty", "qwe_rty", "queue rty", "queue  rty", "Hi"]
    ]

    def test_invalid_app_names(self):
        data = self.data.copy()
        data["hooks"] = {self.name: {"plugs": ["network"]}}

        raised = self.assertRaises(errors.YamlValidationError, Validator(data).validate)

        expected_message = (
            "The 'hooks' property does not match the required schema: {!r} is "
            "not a valid hook name. Hook names consist of lower-case "
            "alphanumeric characters and hyphens. They cannot start or end "
            "with a hyphen."
        ).format(self.name)
        self.assertThat(raised.message, Equals(expected_message), message=data)


class InvalidPartNamesTest(ValidationBaseTest):

    scenarios = [
        (name, dict(name=name))
        for name in [
            "plugins",
            "qwe#rty",
            "qwe_rty",
            "queue rty",
            "queue  rty",
            "part/sub",
        ]
    ]

    def test_invalid_part_names(self):
        data = self.data.copy()
        data["parts"] = {self.name: {"plugin": "nil"}}

        raised = self.assertRaises(errors.YamlValidationError, Validator(data).validate)

        expected_message = (
            "The 'parts' property does not match the required schema: {!r} is "
            "not a valid part name. Part names consist of lower-case "
            "alphanumeric characters, hyphens and plus signs. "
            "As a special case, 'plugins' is also not a valid part name."
        ).format(self.name)
        self.assertThat(raised.message, Equals(expected_message), message=data)


class InvalidArchitecturesTest(ValidationBaseTest):

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

    def test_invalid_architectures(self):
        data = self.data.copy()
        data["architectures"] = self.architectures

        raised = self.assertRaises(errors.YamlValidationError, Validator(data).validate)

        self.assertThat(
            raised.message,
            MatchesRegex(
                "The 'architectures.*?' property does not match the required "
                "schema: {}".format(self.message)
            ),
            message=data,
        )


class NameTest(ProjectBaseTest):
    def setUp(self):
        super().setUp()

        self.snapcraft_yaml = dedent(
            """\
            name: {}
            version: "1"
            base: core18
            summary: test
            description: nothing
            confinement: strict
            grade: stable

            parts:
                part1:
                    plugin: nil
        """
        )

    def test_invalid_yaml_invalid_name_as_number(self):
        raised = self.assertValidationRaises(self.snapcraft_yaml.format("1"))

        self.assertThat(
            raised.message,
            Equals(
                "The 'name' property does not match the required "
                "schema: snap names need to be strings."
            ),
        )

    def test_invalid_yaml_invalid_name_as_list(self):
        raised = self.assertValidationRaises(self.snapcraft_yaml.format("[]"))

        self.assertThat(
            raised.message,
            Equals(
                "The 'name' property does not match the required "
                "schema: snap names need to be strings."
            ),
        )


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
            raised.message, Equals("'icon' must be either a .png or a .svg")
        )

    def test_invalid_yaml_missing_icon(self):
        raised = self.assertValidationRaises(
            dedent(
                """\
            name: test
            base: core18
            version: "1"
            summary: test
            description: test
            icon: icon.png
            confinement: strict
            grade: stable

            parts:
              part1:
                plugin: nil
            """
            )
        )

        self.assertThat(
            raised.message, Equals("Specified icon 'icon.png' does not exist")
        )


class ValidTitleTest(ProjectBaseTest):
    scenarios = (
        ("normal", dict(title="a title")),
        ("normal with caps", dict(title="A Title")),
        ("upper limit (40)", dict(title="T" * 40)),
        ("upper limit (40) with emoji", dict(title="💩" * 40)),
        ("upper limit (40) with unicode (non emoji)", dict(title="’" * 40)),
    )

    def test_title(self):
        self.assertValidationPasses(
            dedent(
                """\
            name: test
            base: core18
            version: "1"
            title: {}
            summary: test
            description: nothing
            confinement: strict

            parts:
              part1:
                plugin: nil
            """
            ).format(self.title)
        )


class InvalidTitleTest(ProjectBaseTest):
    scenarios = (
        ("non string", dict(title=1, error_template="number")),
        ("over upper limit (40)", dict(title="T" * 41, error_template="length")),
        (
            "over upper limit (40) with emoji",
            dict(title="💩" * 41, error_template="length"),
        ),
        (
            "over upper limit (40) with unicode (non emoji)",
            dict(title="’" * 41, error_template="length"),
        ),
    )

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

    def test_invalid(self):
        raised = self.assertValidationRaises(
            dedent(
                """\
            name: test
            base: core18
            version: "1"
            title: {}
            summary: test
            description: nothing
            confinement: strict

            parts:
              part1:
                plugin: nil
            """
            ).format(self.title)
        )

        self.assertThat(
            str(raised),
            Contains(
                self._EXPECTED_ERROR_TEMPLATE[self.error_template].format(self.title)
            ),
        )


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


class ValidVersionTest(ProjectBaseTest):

    scenarios = [
        (version, dict(version=version))
        for version in [
            "'buttered-popcorn'",
            "'1.2.3'",
            '"1.2.3"',
            "'v12.4:1:2~'",
            "'HeLlo'",
            "'v+'",
        ]
    ]

    def test_valid_version(self):
        self.assertValidationPasses(
            dedent(
                """\
            name: test
            base: core18
            version: {}
            summary: test
            description: test
            confinement: strict
            grade: stable
            parts:
              part1:
                plugin: nil
        """
            ).format(self.version)
        )


class InvalidVersionGenericTest(ProjectBaseTest):

    scenarios = [
        (str(version), dict(version=version))
        for version in [
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
        ]
    ]

    def test_invalid_version(self):
        raised = self.assertValidationRaises(
            dedent(
                """\
            name: test
            base: core18
            version: {}
            summary: test
            description: test
            confinement: strict
            grade: stable
            parts:
              part1:
                plugin: nil
        """
            ).format(self.version)
        )

        self.assertThat(
            raised.message,
            Equals(
                "The 'version' property does not match the required "
                "schema: {} is not a valid snap version string. Snap versions "
                "consist of upper- and lower-case alphanumeric characters, "
                "as well as periods, colons, plus signs, tildes, and "
                "hyphens. They cannot begin with a period, colon, plus "
                "sign, tilde, or hyphen. They cannot end with a period, "
                "colon, or hyphen.".format(self.version)
            ),
        )


class InvalidVersionSpecificTest(ProjectBaseTest):
    def test_invalid_type(self):
        raised = self.assertValidationRaises(
            dedent(
                """
            name: test
            base: core18
            version: 0.1
            summary: test
            description: test
            confinement: strict
            grade: stable
            parts:
              part1:
                plugin: nil
        """
            )
        )

        self.assertThat(
            raised.message,
            Equals(
                "The 'version' property does not match the required "
                "schema: snap versions need to be strings. They must "
                "also be wrapped in quotes when the value will be "
                "interpreted by the YAML parser as a non-string. "
                "Examples: '1', '1.2', '1.2.3', git (will be replaced "
                "by a git describe based version string)."
            ),
        )

    def test_too_long(self):
        raised = self.assertValidationRaises(
            dedent(
                """
            name: test
            base: core18
            version: this.is.a.really.too.long.version
            summary: test
            description: test
            confinement: strict
            grade: stable
            parts:
              part1:
                plugin: nil
        """
            )
        )

        self.assertThat(
            raised.message,
            Equals(
                "The 'version' property does not match the required "
                "schema: 'this.is.a.really.too.long.version' is too long "
                "(maximum length is 32)"
            ),
        )


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


class ValidAppNamesTest(ProjectBaseTest):

    scenarios = [
        (name, dict(name=name))
        for name in [
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
        ]
    ]

    def test_valid_app_names(self):
        snapcraft_yaml = self.assertValidationPasses(
            dedent(
                dedent(
                    """\
            name: test
            base: core18
            version: "1"
            summary: test
            description: nothing
            confinement: strict
            grade: stable

            apps:
              {!r}:
                command: foo

            parts:
              part1:
                plugin: nil
        """
                ).format(self.name)
            )
        )

        self.assertThat(snapcraft_yaml["apps"], Contains(self.name))


class InvalidAppNamesYamlTest(ProjectBaseTest):

    scenarios = [
        (name, dict(name=name))
        for name in [
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
        ]
    ]

    def test_invalid_yaml_invalid_app_names(self):
        raised = self.assertValidationRaises(
            dedent(
                """\
            name: test
            base: core18
            version: "1"
            summary: test
            description: nothing
            confinement: strict
            grade: stable

            apps:
              {!r}:
                command: foo

            parts:
              part1:
                plugin: nil
        """
            ).format(self.name)
        )

        self.assertRegex(
            raised.message,
            "The 'apps' property does not match the required schema: .* is "
            "not a valid app name. App names consist of upper- and lower-case "
            "alphanumeric characters and hyphens",
        )


class InvalidHookNamesYamlTest(ProjectBaseTest):

    scenarios = [
        (name, dict(name=name))
        for name in [
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
            "Hi",
        ]
    ]

    def test_invalid_yaml_invalid_hook_names(self):
        raised = self.assertValidationRaises(
            dedent(
                """\
            name: test
            base: core18
            version: "1"
            summary: test
            description: nothing
            confinement: strict
            grade: stable

            hooks:
              {!r}:
                plugs: [network]

            parts:
              part1:
                plugin: nil
        """
            ).format(self.name)
        )

        self.assertRegex(
            raised.message,
            "The 'hooks' property does not match the required schema: .* is "
            "not a valid hook name. Hook names consist of lower-case "
            "alphanumeric characters and hyphens",
        )


class ValidConfinmentTest(ProjectBaseTest):

    scenarios = [
        (confinement, dict(confinement=confinement))
        for confinement in ["strict", "devmode", "classic"]
    ]

    def test_valid_confinement(self):
        snapcraft_yaml = self.assertValidationPasses(
            (
                dedent(
                    """\
            name: test
            base: core18
            version: "1"
            summary: test
            description: nothing
            confinement: {}
            grade: stable

            parts:
              part1:
                plugin: go
                stage-packages: [fswebcam]
        """
                ).format(self.confinement)
            )
        )

        self.assertThat(snapcraft_yaml["confinement"], Equals(self.confinement))


class InvalidConfinementTest(ProjectBaseTest):

    scenarios = [
        (confinement, dict(confinement=confinement))
        for confinement in ["foo", "strict-", "_devmode"]
    ]

    def test_invalid_confinement(self):
        raised = self.assertValidationRaises(
            dedent(
                """\
            name: test
            base: core18
            version: "1"
            summary: test
            description: nothing
            confinement: {}
            grade: stable

            parts:
              part1:
                plugin: nil
        """
            ).format(self.confinement)
        )

        self.assertThat(
            raised.message,
            Equals(
                "The 'confinement' property does not match the required "
                "schema: '{}' is not one of ['classic', 'devmode', "
                "'strict']".format(self.confinement)
            ),
        )


class ValidGradeTest(ProjectBaseTest):

    scenarios = [(grade, dict(grade=grade)) for grade in ["stable", "devel"]]

    def test_yaml_valid_grade_types(self):
        snapcraft_yaml = self.assertValidationPasses(
            dedent(
                """\
            name: test
            base: core18
            version: "1"
            summary: test
            description: nothing
            confinement: strict
            grade: {}

            parts:
              part1:
                plugin: nil
        """
            ).format(self.grade)
        )

        self.assertThat(snapcraft_yaml["grade"], Equals(self.grade))


class InvalidGradeTest(ProjectBaseTest):

    scenarios = [
        (grade, dict(grade=grade)) for grade in ["foo", "unstable-", "_experimental"]
    ]

    def test_invalid_yaml_invalid_grade_types(self):
        raised = self.assertValidationRaises(
            dedent(
                """\
                name: test
                base: core18
                version: "1"
                summary: test
                description: nothing
                confinement: strict
                grade: {}

                parts:
                  part1:
                    plugin: nil
                """
            ).format(self.grade)
        )

        self.assertThat(
            raised.message,
            Equals(
                "The 'grade' property does not match the required "
                "schema: '{}' is not one of ['stable', 'devel']".format(self.grade)
            ),
        )


class ValidEpochsTest(ProjectBaseTest):

    scenarios = [
        ("int 0", {"yaml": 0, "expected": 0}),
        ("int string 0", {"yaml": '"0"', "expected": "0"}),
        ("1*", {"yaml": "1*", "expected": "1*"}),
        ('"1*"', {"yaml": '"1*"', "expected": "1*"}),
        ("int 1", {"yaml": 1, "expected": 1}),
        ("inst string 1", {"yaml": '"1"', "expected": "1"}),
        ("400 *", {"yaml": "400*", "expected": "400*"}),
        ('"400*"', {"yaml": '"400*"', "expected": "400*"}),
        ("high int", {"yaml": 1234, "expected": 1234}),
        ("high int string", {"yaml": '"1234"', "expected": "1234"}),
        ("padded with 0", {"yaml": "0001", "expected": 1}),
    ]

    def test_yaml_valid_epochs(self):
        snapcraft_yaml = self.assertValidationPasses(
            dedent(
                """\
            name: test
            base: core18
            version: "1"
            summary: test
            description: nothing
            epoch: {}
            parts:
              part1:
                plugin: nil
        """
            ).format(self.yaml)
        )

        self.assertThat(snapcraft_yaml["epoch"], Equals(self.expected))


class InvalidEpochsTest(ProjectBaseTest):

    scenarios = [
        (epoch, dict(epoch=epoch))
        for epoch in [
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
        ]
    ]

    def test_invalid_yaml_invalid_epochs(self):
        raised = self.assertValidationRaises(
            dedent(
                """\
            name: test
            base: core18
            version: "1"
            summary: test
            description: nothing
            epoch: {}
            parts:
              part1:
                plugin: nil
        """
            ).format(self.epoch)
        )

        self.assertRegex(
            raised.message,
            r"The 'epoch' property does not match the required "
            r"schema:.*is not a 'epoch' \(epochs are positive integers "
            r"followed by an optional asterisk\)",
        )


class LicenseTest(ProjectBaseTest):
    def test_yaml_valid_license_string(self):
        self.assertValidationPasses(
            dedent(
                """\
                name: test
                base: core18
                version: "1"
                summary: test
                description: nothing
                license: MIT-0
                parts:
                  part1:
                    plugin: nil
                """
            )
        )

    def test_invalid_yaml_invalid_license_non_string(self):
        raised = self.assertValidationRaises(
            dedent(
                """\
            name: test
            base: core18
            version: "1"
            summary: test
            description: nothing
            license: 23.1
            parts:
              part1:
                plugin: nil
            """
            )
        )

        self.assertRegex(
            raised.message,
            "The 'license' property does not match the required schema:.*is "
            "not of type 'string'",
        )


class ValidAdaptersTest(ProjectBaseTest):
    scenarios = [("none", {"yaml": "none"}), ("legacy", {"yaml": "legacy"})]

    def test_yaml_valid_adapters(self):
        self.assertValidationPasses(
            dedent(
                """\
                name: test
                base: core18
                version: "1"
                summary: test
                description: nothing
                apps:
                  app:
                    command: foo
                    adapter: {}
                parts:
                  part1:
                    plugin: nil
                """
            ).format(self.yaml)
        )


class InvalidAdapterTest(ProjectBaseTest):
    scenarios = [
        ("NONE", {"yaml": "NONE"}),
        ("none-", {"yaml": "none-"}),
        ("invalid", {"yaml": "invalid"}),
        ("LeGaCY", {"yaml": "LeGaCY"}),
        ("leg-acy", {"yaml": "leg-acy"}),
    ]

    def test_invalid_yaml_invalid_adapters(self):
        raised = self.assertValidationRaises(
            dedent(
                """\
            name: test
            base: core18
            version: "1"
            summary: test
            description: nothing
            apps:
              app:
                command: foo
                adapter: {}
            parts:
              part1:
                plugin: nil
            """
            ).format(self.yaml)
        )

        self.assertRegex(
            raised.message,
            r"The 'apps/app/adapter' property does not match the required schema:.*is "
            r"not one of \['none', 'legacy', 'full'\]",
        )


class InvalidBuildEnvironmentTest(ProjectBaseTest):

    scenarios = [
        (
            "wrong type (string)",
            {
                "environment": "a string",
                "message": ".*property does not match the required schema: 'a string' is not of type 'array'.*",
            },
        ),
        (
            "wrong type (list of strings)",
            {
                "environment": "['a string']",
                "message": ".*property does not match the required schema: 'a string' is not of type 'object'.*",
            },
        ),
        (
            "too many properties",
            {
                "environment": "[{key1: value1, key2: value2}]",
                "message": ".*property does not match the required schema:.*has too many properties.*",
            },
        ),
        (
            "wrong property type",
            {
                "environment": "[{key1: 5}]",
                "message": ".*property does not match the required schema: 5 is not of type 'string'.*",
            },
        ),
    ]

    def test_build_environment(self):
        raised = self.assertValidationRaises(
            dedent(
                """\
            name: my-package-1
            base: core18
            version: 1.0-snapcraft1~ppa1
            summary: my summary less that 79 chars
            description: description which can be pretty long
            parts:
                part1:
                    plugin: nil
                    build-environment: {}
        """
            ).format(self.environment)
        )

        self.assertThat(raised.message, MatchesRegex(self.message))


class InvalidCommandChainTest(ProjectBaseTest):

    scenarios = [
        (
            "a string",
            {
                "command_chain": "a string",
                "message": ".*'a string' is not of type 'array'.*",
            },
        ),
        (
            "list of numbers",
            {
                "command_chain": [1],
                "message": ".*1 is not a valid command-chain entry.*",
            },
        ),
        (
            "spaces",
            {
                "command_chain": ["test chain"],
                "message": ".*'test chain' is not a valid command-chain entry.*",
            },
        ),
        (
            "quotes",
            {
                "command_chain": ["test'chain"],
                "message": '.*"test\'chain" is not a valid command-chain entry.*',
            },
        ),
    ]

    def test_command_chain(self):
        raised = self.assertValidationRaises(
            dedent(
                """\
            name: test-snap
            base: core18
            version: "1.0"
            summary: test summary
            description: test description

            apps:
                my-app:
                    command: foo
                    command-chain: {}

            parts:
                my-part:
                    plugin: nil
        """
            ).format(self.command_chain)
        )

        self.assertThat(raised.message, MatchesRegex(self.message))


class RepositoriesTests(ProjectBaseTest):
    def test_yaml_repostiory_minimal(self):
        self.assertValidationPasses(
            dedent(
                """\
                name: test
                base: core18
                version: "1"
                summary: test
                description: package-management test
                license: GPLv2
                parts:
                  part1:
                    plugin: nil
                package-management:
                  repositories:
                  - source: deb http://ppa.launchpad.net/mozillateam/ppa/ubuntu bionic main
                """
            )
        )

    def test_yaml_invalid_source(self):
        raised = self.assertValidationRaises(
            dedent(
                """\
                name: test
                base: core18
                version: "1"
                summary: test
                description: package-management test
                license: GPLv2
                parts:
                  part1:
                    plugin: nil
                package-management:
                  repositories:
                  - source: deb http://ppa.launchpad.net/mozillateam/ppa/ubuntu bionic main
                    foo: bar
                """
            )
        )

        self.assertThat(
            raised.message,
            Equals(
                "The 'package-management/repositories[0]' property does not match the required schema: 'source' is not a valid repository."
            ),
        )

    def test_yaml_source_minimal_ppa(self):
        self.assertValidationPasses(
            dedent(
                """\
                name: test
                base: core18
                version: "1"
                summary: test
                description: package-management test
                license: GPLv2
                parts:
                  part1:
                    plugin: nil
                package-management:
                  repositories:
                  - source: ppa:mozillateam/ppa
                """
            )
        )

    def test_yaml_source_keyid(self):
        self.assertValidationPasses(
            dedent(
                """\
                name: test
                base: core18
                version: "1"
                summary: test
                description: package-management test
                license: GPLv2
                parts:
                  part1:
                    plugin: nil
                package-management:
                  repositories:
                  - source: deb http://ppa.launchpad.net/mozillateam/ppa/ubuntu bionic main
                    gpg-key-server: keyserver.ubuntu.com
                    gpg-public-key-id: 0ab215679c571d1c8325275b9bdb3d89ce49ec21
                """
            )
        )

    def test_yaml_source_key(self):
        self.assertValidationPasses(
            dedent(
                """\
                name: test
                base: core18
                version: "1"
                summary: test
                description: package-management test
                license: GPLv2
                parts:
                  part1:
                    plugin: nil
                package-management:
                  repositories:
                  - source: deb http://ppa.launchpad.net/mozillateam/ppa/ubuntu bionic main
                    gpg-public-key: |
                      -----BEGIN PGP PUBLIC KEY BLOCK-----
                      ...
                      -----END PGP PUBLIC KEY BLOCK-----
                """
            )
        )

    def test_yaml_invalid_gpg_key(self):
        raised = self.assertValidationRaises(
            dedent(
                """\
                name: test
                base: core18
                version: "1"
                summary: test
                description: package-management test
                license: GPLv2
                parts:
                  part1:
                    plugin: nil
                package-management:
                  repositories:
                  - source: deb http://ppa.launchpad.net/mozillateam/ppa/ubuntu bionic main
                    gpg-public-key: |
                      xxx-----BEGIN PGP PUBLIC KEY BLOCK-----
                      ...
                      -----END PGP PUBLIC KEY BLOCK-----
                """
            )
        )

        self.assertThat(
            raised.message,
            Equals(
                "The 'package-management/repositories[0]/gpg-public-key' property does not match the required schema: <ValidationError: \"'xxx-----BEGIN PGP PUBLIC KEY BLOCK-----\\\\n...\\\\n-----END PGP PUBLIC KEY BLOCK-----\\\\n' does not match '^[\\\\\\\\s]*-----BEGIN PGP PUBLIC KEY BLOCK-----.*'\"> is not a valid GPG key.  A GPG key must be a string starting with \"-----BEGIN PGP PUBLIC KEY BLOCK-----\"."
            ),
        )

    def test_yaml_multiple_sources(self):
        self.assertValidationPasses(
            dedent(
                """\
                name: test
                base: core18
                version: "1"
                summary: test
                description: package-management test
                license: GPLv2
                parts:
                  part1:
                    plugin: nil
                package-management:
                  repositories:
                  - source: deb http://ppa.launchpad.net/mozillateam/ppa/ubuntu bionic main
                    gpg-public-key: |
                      -----BEGIN PGP PUBLIC KEY BLOCK-----
                      ...
                      -----END PGP PUBLIC KEY BLOCK-----
                  - source: deb http://ppa.launchpad.net/another/ppa/ubuntu bionic main
                  - source: deb http://ppa.launchpad.net/yetanother/ppa/ubuntu bionic main
                """
            )
        )


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
