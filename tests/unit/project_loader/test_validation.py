# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2018 Canonical Ltd
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
from textwrap import dedent
from unittest import mock

import fixtures
from testscenarios.scenarios import multiply_scenarios
from testtools.matchers import Contains, Equals, MatchesRegex

from . import ProjectLoaderBaseTest
from tests import fixture_setup, unit
from snapcraft.project import Project
from snapcraft.internal.errors import PluginError
from snapcraft.internal.project_loader import load_config, errors
from snapcraft.internal.project_loader import Validator


class ValidationBaseTest(unit.TestCase):
    def setUp(self):
        super().setUp()

        patcher = mock.patch("os.path.exists")
        self.mock_path_exists = patcher.start()
        self.mock_path_exists.return_value = True
        self.addCleanup(patcher.stop)

        self.data = {
            "name": "my-package-1",
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

        with mock.patch(
            "snapcraft.internal.project_loader._schema.open", mock_the_open, create=True
        ):
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
        }

        Validator(self.data).validate()

    def test_invalid_restart_condition(self):
        self.data["apps"] = {
            "service1": {
                "command": "binary1",
                "daemon": "simple",
                "restart-condition": "on-watchdog",
            }
        }

        raised = self.assertRaises(
            errors.YamlValidationError, Validator(self.data).validate
        )

        self.assertThat(
            str(raised),
            Contains(
                "The 'apps/service1/restart-condition' property does not match "
                "the required schema: 'on-watchdog' is not one of ['on-success', "
                "'on-failure', 'on-abnormal', 'on-abort', 'always', 'never']"
            ),
        )

    def test_both_snap_and_prime_specified(self):
        self.data["parts"]["part1"]["snap"] = ["foo"]
        self.data["parts"]["part1"]["prime"] = ["bar"]

        raised = self.assertRaises(
            errors.YamlValidationError, Validator(self.data).validate
        )

        self.assertThat(
            str(raised),
            MatchesRegex(
                ".*The 'parts/part1' property does not match the required "
                "schema: .* cannot contain both 'snap' and 'prime' keywords.*"
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


class OldConflictsWithNewScriptletTest(ValidationBaseTest):

    old_scriptlet_scenarios = [
        ("prepare", {"old_keyword": "prepare", "old_value": ["test-prepare"]}),
        ("build", {"old_keyword": "build", "old_value": ["test-build"]}),
        ("install", {"old_keyword": "install", "old_value": ["test-install"]}),
    ]

    new_scriptlet_scenarios = [
        (
            "override-pull",
            {"new_keyword": "override-pull", "new_value": ["test-override-pull"]},
        ),
        (
            "override-build",
            {"new_keyword": "override-build", "new_value": ["test-override-build"]},
        ),
        (
            "override-stage",
            {"new_keyword": "override-stage", "new_value": ["test-override-stage"]},
        ),
        (
            "override-prime",
            {"new_keyword": "override-prime", "new_value": ["test-override-prime"]},
        ),
    ]

    scenarios = multiply_scenarios(old_scriptlet_scenarios, new_scriptlet_scenarios)

    def test_both_old_and_new_keywords_specified(self):
        self.data["parts"]["part1"][self.old_keyword] = self.old_value
        self.data["parts"]["part1"][self.new_keyword] = self.new_value

        raised = self.assertRaises(
            errors.YamlValidationError, Validator(self.data).validate
        )

        self.assertThat(
            str(raised),
            MatchesRegex(
                (
                    ".*The 'parts/part1' property does not match the required "
                    "schema: Parts cannot contain both {0!r} and 'override-\*' "
                    "keywords. Use 'override-build' instead of {0!r}.*"
                ).format(self.old_keyword)
            ),
        )


class DaemonDependencyTest(ValidationBaseTest):

    scenarios = [
        ("stop-command", dict(option="stop-command", value="binary1 --stop")),
        (
            "post-stop-command",
            dict(option="post-stop-command", value="binary1 --post-stop"),
        ),
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
        (type_, dict(type_=type_)) for type_ in ["app", "gadget", "kernel", "os"]
    ]

    def test_valid_types(self):
        data = self.data.copy()
        data["type"] = self.type_
        Validator(data).validate()


class InvalidTypesTest(ValidationBaseTest):

    scenarios = [
        (type_, dict(type_=type_)) for type_ in ["apps", "framework", "platform", "oem"]
    ]

    def test_invalid_types(self):
        data = self.data.copy()
        data["type"] = self.type_

        raised = self.assertRaises(errors.YamlValidationError, Validator(data).validate)

        expected_message = (
            "The 'type' property does not match the required "
            "schema: '{}' is not one of "
            "['app', 'base', 'gadget', 'kernel', 'os']"
        ).format(self.type_)
        self.assertThat(raised.message, Equals(expected_message), message=data)


class ValidRestartConditionsTest(ValidationBaseTest):

    scenarios = [
        (condition, dict(condition=condition))
        for condition in [
            "always",
            "on-success",
            "on-failure",
            "on-abnormal",
            "on-abort",
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
        for name in ["plugins", "qwe#rty", "qwe_rty", "queue rty", "queue  rty"]
    ]

    def test_invalid_part_names(self):
        data = self.data.copy()
        data["parts"] = {self.name: {"plugin": "nil"}}

        raised = self.assertRaises(errors.YamlValidationError, Validator(data).validate)

        expected_message = (
            "The 'parts' property does not match the required schema: {!r} is "
            "not a valid part name. Part names consist of lower-case "
            "alphanumeric characters, hyphens, plus signs, and forward "
            "slashes. As a special case, 'plugins' is also not a valid part "
            "name."
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


class NameTest(unit.TestCase):
    def make_snapcraft_project(self, *, name):
        snapcraft_yaml = dedent(
            """\
            name: {}
            summary: test
            description: nothing
            confinement: strict
            grade: stable

            parts:
                part1:
                    plugin: nil
        """
        )
        snapcraft_yaml_file_path = super().make_snapcraft_yaml(snapcraft_yaml)
        project = Project(snapcraft_yaml_file_path=snapcraft_yaml_file_path)
        return load_config(project)

    def test_invalid_yaml_invalid_name_as_number(self):
        raised = self.assertRaises(
            errors.YamlValidationError, self.make_snapcraft_project, name="1"
        )

        self.assertThat(
            raised.message,
            Equals(
                "The 'name' property does not match the required "
                "schema: snap names need to be strings."
            ),
        )

    def test_invalid_yaml_invalid_name_as_list(self):
        raised = self.assertRaises(
            errors.YamlValidationError, self.make_snapcraft_project, name="[]"
        )

        self.assertThat(
            raised.message,
            Equals(
                "The 'name' property does not match the required "
                "schema: snap names need to be strings."
            ),
        )

    def test_invalid_yaml_invalid_name_as_huge_map(self):
        # making my point about not printing the thing for failed type check
        name = dedent(
            """\
            - a: &a {a: {a: {a: {a: {a: {a: {a: {a: {a: {a: {a: {a: {a: {}}}}}}}}}}}}}}
            - b: &b [*a, *a, *a, *a, *a, *a, *a, *a, *a, *a, *a, *a, *a, *a, *a, *a, *a]
            - c: &c [*b, *b, *b, *b, *b, *b, *b, *b, *b, *b, *b, *b, *b, *b, *b, *b, *b]
            - z: "I could go on but time and memory are both too short" """
        )  # noqa: E501
        raised = self.assertRaises(
            errors.YamlValidationError, self.make_snapcraft_project, name=name
        )

        self.assertThat(
            raised.message,
            Equals(
                "The 'name' property does not match the required "
                "schema: snap names need to be strings."
            ),
        )


class IconTest(ProjectLoaderBaseTest):
    def test_invalid_yaml_invalid_icon_extension(self):
        snapcraft_yaml = dedent(
            """\
            name: test
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

        raised = self.assertRaises(
            errors.YamlValidationError, self.make_snapcraft_project, snapcraft_yaml
        )

        self.assertThat(
            raised.message, Equals("'icon' must be either a .png or a .svg")
        )

    def test_invalid_yaml_missing_icon(self):
        snapcraft_yaml = dedent(
            """\
            name: test
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

        raised = self.assertRaises(
            errors.YamlValidationError, self.make_snapcraft_project, snapcraft_yaml
        )

        self.assertThat(
            raised.message, Equals("Specified icon 'icon.png' does not exist")
        )


class OrganizeTest(ProjectLoaderBaseTest):
    def test_yaml_organize_value_none(self):
        snapcraft_yaml = dedent(
            """\
            name: test
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

        raised = self.assertRaises(
            errors.YamlValidationError, self.make_snapcraft_project, snapcraft_yaml
        )

        self.assertThat(
            str(raised),
            Contains(
                "The 'parts/part1/organize/foo' property does not match the "
                "required schema: None is not of type 'string'"
            ),
        )

    def test_yaml_organize_value_empty(self):
        snapcraft_yaml = dedent(
            """\
            name: test
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

        raised = self.assertRaises(
            errors.YamlValidationError, self.make_snapcraft_project, snapcraft_yaml
        )

        self.assertThat(
            str(raised),
            Contains(
                "The 'parts/part1/organize/foo' property does not match the "
                "required schema: '' is too short (minimum length is 1)"
            ),
        )


class AliasesTest(ProjectLoaderBaseTest):
    def make_snapcraft_project(self, apps):
        snapcraft_yaml = fixture_setup.SnapcraftYaml(self.path)
        snapcraft_yaml.update_part("part1", dict(plugin="nil"))
        for app_name, app in apps:
            snapcraft_yaml.update_app(app_name, app)
        self.useFixture(snapcraft_yaml)

        project = Project(
            snapcraft_yaml_file_path=snapcraft_yaml.snapcraft_yaml_file_path
        )
        return load_config(project)

    def test_aliases(self,):
        fake_logger = fixtures.FakeLogger(level=logging.WARNING)
        self.useFixture(fake_logger)

        apps = [("test", dict(command="test", aliases=["test-it", "testing"]))]
        c = self.make_snapcraft_project(apps)

        self.maxDiff = None

        self.assertTrue(
            "aliases" in c.data["apps"]["test"],
            'Expected "aliases" property to be in snapcraft.yaml',
        )
        self.assertThat(
            c.data["apps"]["test"]["aliases"], Equals(["test-it", "testing"])
        )

        # Verify that aliases are properly deprecated
        self.assertThat(
            fake_logger.output,
            Contains(
                "Aliases are now handled by the store, and shouldn't be declared "
                "in the snap."
            ),
        )
        self.assertThat(
            fake_logger.output,
            Contains("See http://snapcraft.io/docs/deprecation-notices/dn5"),
        )

    def test_duplicate_aliases(self):
        apps = [
            ("test1", dict(command="test", aliases=["testing"])),
            ("test2", dict(command="test", aliases=["testing"])),
        ]
        raised = self.assertRaises(
            errors.DuplicateAliasError, self.make_snapcraft_project, apps
        )

        self.assertThat(
            str(raised),
            Equals(
                "Multiple parts have the same alias defined: {!r}".format("testing")
            ),
        )

    def test_invalid_alias(self):
        apps = [("test", dict(command="test", aliases=[".test"]))]
        raised = self.assertRaises(
            errors.YamlValidationError, self.make_snapcraft_project, apps
        )
        expected = (
            "The {path!r} property does not match the required schema: "
            "{alias!r} does not match ".format(
                path="apps/test/aliases[0]", alias=".test"
            )
        )
        self.assertThat(str(raised), Contains(expected))


class VersionTest(ProjectLoaderBaseTest):
    def test_invalid_yaml_version_too_long(self):
        snapcraft_yaml = dedent(
            """\
            name: test
            version: 'abcdefghijklmnopqrstuvwxyz1234567' # Max is 32 in the store
            summary: test
            description: test
            confinement: strict
            grade: stable
            parts:
              part1:
                plugin: nil
        """
        )  # noqa: E501
        raised = self.assertRaises(
            errors.YamlValidationError, self.make_snapcraft_project, snapcraft_yaml
        )

        self.assertThat(
            raised.message,
            Equals(
                "The 'version' property does not match the required "
                "schema: 'abcdefghijklmnopqrstuvwxyz1234567' is too long "
                "(maximum length is 32)"
            ),
        )


class ValidVersionTest(ProjectLoaderBaseTest):

    scenarios = [
        (version, dict(version=version))
        for version in ["buttered-popcorn", "1.2.3", "v12.4:1:2~", "HeLlo", "v+"]
    ]

    def test_valid_version(self):
        snapcraft_yaml = fixture_setup.SnapcraftYaml(self.path, version=self.version)
        snapcraft_yaml.update_part("part1", dict(plugin="nil"))
        self.useFixture(snapcraft_yaml)
        project = Project(
            snapcraft_yaml_file_path=snapcraft_yaml.snapcraft_yaml_file_path
        )

        # This call will should not fail
        load_config(project)


class InvalidVersionTest(ProjectLoaderBaseTest):

    scenarios = [
        (version, dict(version=version))
        for version in [
            "*",
            "",
            ":v",
            ".v",
            "+v",
            "~v",
            "_v",
            "-v",
            "v:",
            "v.",
            "v_",
            "v-",
            "underscores_are_bad",
        ]
    ]

    def test_invalid_version(self):
        snapcraft_yaml = fixture_setup.SnapcraftYaml(self.path, version=self.version)
        snapcraft_yaml.update_part("part1", dict(plugin="nil"))
        self.useFixture(snapcraft_yaml)
        project = Project(
            snapcraft_yaml_file_path=snapcraft_yaml.snapcraft_yaml_file_path
        )

        # This call will should not fail
        raised = self.assertRaises(errors.YamlValidationError, load_config, project)

        self.assertThat(
            raised.message,
            Equals(
                "The 'version' property does not match the required "
                "schema: {!r} is not a valid snap version. Snap versions "
                "consist of upper- and lower-case alphanumeric characters, "
                "as well as periods, colons, plus signs, tildes, and "
                "hyphens. They cannot begin with a period, colon, plus "
                "sign, tilde, or hyphen. They cannot end with a period, "
                "colon, or hyphen.".format(self.version)
            ),
        )


class EnvironmentTest(ProjectLoaderBaseTest):
    def test_valid_environment(self):
        project_config = self.make_snapcraft_project(
            dedent(
                """\
            name: project-name
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

        self.assertThat(
            project_config.data["environment"],
            Equals(dict(GLOBAL="1", OTHER="valid-value")),
        )
        self.assertThat(
            project_config.data["apps"]["app1"]["environment"],
            Equals(dict(LOCALE="C", PLUGIN_PATH="$SNAP_USER_DATA/plugins")),
        )

    def test_invalid_environment(self):
        snapcraft_yaml = dedent(
            """\
            name: project-name
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
        raised = self.assertRaises(
            errors.YamlValidationError, self.make_snapcraft_project, snapcraft_yaml
        )

        self.assertRegex(
            raised.message,
            "The 'environment/INVALID' property does not match the required "
            "schema: \[1, 2\].*",
        )


class ValidAppNamesTest(ProjectLoaderBaseTest):

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
        project_config = self.make_snapcraft_project(
            dedent(
                """\
            name: test
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

        self.assertThat(project_config.data["apps"], Contains(self.name))


class InvalidAppNamesYamlTest(ProjectLoaderBaseTest):

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
        snapcraft_yaml = dedent(
            """\
            name: test
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

        raised = self.assertRaises(
            errors.YamlValidationError, self.make_snapcraft_project, snapcraft_yaml
        )

        self.assertRegex(
            raised.message,
            "The 'apps' property does not match the required schema: .* is "
            "not a valid app name. App names consist of upper- and lower-case "
            "alphanumeric characters and hyphens",
        )


class InvalidHookNamesYamlTest(ProjectLoaderBaseTest):

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
        snapcraft_yaml = dedent(
            """\
            name: test
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

        raised = self.assertRaises(
            errors.YamlValidationError, self.make_snapcraft_project, snapcraft_yaml
        )

        self.assertRegex(
            raised.message,
            "The 'hooks' property does not match the required schema: .* is "
            "not a valid hook name. Hook names consist of lower-case "
            "alphanumeric characters and hyphens",
        )


class ValidConfinmentTest(ProjectLoaderBaseTest):

    scenarios = [
        (confinement, dict(confinement=confinement))
        for confinement in ["strict", "devmode", "classic"]
    ]

    def test_valid_confinement(self):
        project_config = self.make_snapcraft_project(
            dedent(
                """\
            name: test
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

        self.assertThat(project_config.data["confinement"], Equals(self.confinement))


class InvalidConfinementTest(ProjectLoaderBaseTest):

    scenarios = [
        (confinement, dict(confinement=confinement))
        for confinement in ["foo", "strict-", "_devmode"]
    ]

    def test_invalid_confinement(self):
        snapcraft_yaml = dedent(
            """\
            name: test
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

        raised = self.assertRaises(
            errors.YamlValidationError, self.make_snapcraft_project, snapcraft_yaml
        )

        self.assertThat(
            raised.message,
            Equals(
                "The 'confinement' property does not match the required "
                "schema: '{}' is not one of ['classic', 'devmode', "
                "'strict']".format(self.confinement)
            ),
        )


class ValidGradeTest(ProjectLoaderBaseTest):

    scenarios = [(grade, dict(grade=grade)) for grade in ["stable", "devel"]]

    def test_yaml_valid_grade_types(self):
        project_config = self.make_snapcraft_project(
            dedent(
                """\
            name: test
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

        self.assertThat(project_config.data["grade"], Equals(self.grade))


class InvalidGradeTest(ProjectLoaderBaseTest):

    scenarios = [
        (grade, dict(grade=grade)) for grade in ["foo", "unstable-", "_experimental"]
    ]

    def test_invalid_yaml_invalid_grade_types(self):
        snapcraft_yaml = (
            """\
            name: test
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

        raised = self.assertRaises(
            errors.YamlValidationError, self.make_snapcraft_project, snapcraft_yaml
        )

        self.assertThat(
            raised.message,
            Equals(
                "The 'grade' property does not match the required "
                "schema: '{}' is not one of ['stable', 'devel']".format(self.grade)
            ),
        )


class ValidEpochsTest(ProjectLoaderBaseTest):

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
        project_config = self.make_snapcraft_project(
            dedent(
                """\
            name: test
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

        self.assertThat(project_config.data["epoch"], Equals(self.expected))


class InvalidEpochsTest(ProjectLoaderBaseTest):

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
        snapcraft_yaml = dedent(
            """\
            name: test
            version: "1"
            summary: test
            description: nothing
            epoch: {}
            parts:
              part1:
                plugin: nil
        """
        ).format(self.epoch)

        raised = self.assertRaises(
            errors.YamlValidationError, self.make_snapcraft_project, snapcraft_yaml
        )

        self.assertRegex(
            raised.message,
            "The 'epoch' property does not match the required "
            "schema:.*is not a 'epoch' \(epochs are positive integers "
            "followed by an optional asterisk\)",
        )


class ValidArchitecturesTest(ProjectLoaderBaseTest):

    yaml_scenarios = [
        (
            "none",
            {
                "expected_amd64": ["amd64"],
                "expected_i386": ["i386"],
                "expected_armhf": ["armhf"],
                "yaml": None,
            },
        ),
        (
            "single string list",
            {
                "expected_amd64": ["amd64"],
                "expected_i386": ["i386"],
                "expected_armhf": ["armhf"],
                "yaml": "[amd64]",
            },
        ),
        (
            "multiple string list",
            {
                "expected_amd64": ["amd64", "i386"],
                "expected_i386": ["amd64", "i386"],
                "expected_armhf": ["armhf"],
                "yaml": "[amd64, i386]",
            },
        ),
        (
            "single object list",
            {
                "expected_amd64": ["amd64"],
                "expected_i386": ["i386"],
                "expected_armhf": ["armhf"],
                "yaml": dedent(
                    """
                - build-on: [amd64]
                  run-on: [amd64]
            """
                ),
            },
        ),
        (
            "multiple object list",
            {
                "expected_amd64": ["amd64"],
                "expected_i386": ["i386"],
                "expected_armhf": ["armhf", "arm64"],
                "yaml": dedent(
                    """
                - build-on: [amd64]
                  run-on: [amd64]
                - build-on: [i386]
                  run-on: [i386]
                - build-on: [armhf]
                  run-on: [armhf, arm64]
            """
                ),
            },
        ),
        (
            "omit run-on",
            {
                "expected_amd64": ["amd64"],
                "expected_i386": ["i386"],
                "expected_armhf": ["armhf"],
                "yaml": dedent(
                    """
                - build-on: [amd64]
            """
                ),
            },
        ),
        (
            "single build-on string, no list",
            {
                "expected_amd64": ["amd64"],
                "expected_i386": ["i386"],
                "expected_armhf": ["armhf"],
                "yaml": dedent(
                    """
                - build-on: amd64
            """
                ),
            },
        ),
        (
            "build- and run-on string, no lists",
            {
                "expected_amd64": ["amd64"],
                "expected_i386": ["amd64"],
                "expected_armhf": ["armhf"],
                "yaml": dedent(
                    """
                - build-on: i386
                  run-on: amd64
            """
                ),
            },
        ),
        (
            "build on all",
            {
                "expected_amd64": ["amd64"],
                "expected_i386": ["amd64"],
                "expected_armhf": ["amd64"],
                "yaml": dedent(
                    """
                - build-on: [all]
                  run-on: [amd64]
            """
                ),
            },
        ),
        (
            "run on all",
            {
                "expected_amd64": ["all"],
                "expected_i386": ["i386"],
                "expected_armhf": ["armhf"],
                "yaml": dedent(
                    """
                - build-on: [amd64]
                  run-on: [all]
            """
                ),
            },
        ),
    ]

    arch_scenarios = [
        ("amd64", {"target_arch": "amd64"}),
        ("i386", {"target_arch": "i386"}),
        ("armhf", {"target_arch": "armhf"}),
    ]

    scenarios = multiply_scenarios(yaml_scenarios, arch_scenarios)

    def test_architectures(self):
        snippet = ""
        if self.yaml:
            snippet = "architectures: {}".format(self.yaml)
        snapcraft_yaml = dedent(
            """\
            name: test
            version: "1"
            summary: test
            description: test
            {}
            parts:
              my-part:
                plugin: nil
        """
        ).format(snippet)

        try:
            project_kwargs = dict(target_deb_arch=self.target_arch)
            c = self.make_snapcraft_project(snapcraft_yaml, project_kwargs)

            expected = getattr(self, "expected_{}".format(self.target_arch))
            self.assertThat(c.data["architectures"], Equals(expected))
        except errors.YamlValidationError as e:
            self.fail("Expected YAML to be valid, got an error: {}".format(e))


class AdditionalPartPropertiesTest(ProjectLoaderBaseTest):

    scenarios = [("slots", dict(property="slots")), ("plugs", dict(property="plugs"))]

    def test_loading_properties(self):
        snapcraft_yaml = dedent(
            """\
            name: my-package-1
            version: 1.0-snapcraft1~ppa1
            summary: my summary less that 79 chars
            description: description which can be pretty long
            parts:
                part1:
                    plugin: nil
                    {property}: [{property}1]
        """
        ).format(property=self.property)

        raised = self.assertRaises(
            PluginError, self.make_snapcraft_project, snapcraft_yaml
        )

        self.assertThat(
            raised.message,
            Equals(
                "properties failed to load for part1: Additional properties are "
                "not allowed ('{}' was unexpected)".format(self.property)
            ),
        )
