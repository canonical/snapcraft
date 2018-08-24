# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018 Canonical Ltd
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

import textwrap

from testtools.matchers import Contains, Equals, Not

from snapcraft.internal.project_loader import errors
from snapcraft.internal.project_loader._templates._template import Template

from tests import fixture_setup
from .. import ProjectLoaderBaseTest


class TemplateTestBase(ProjectLoaderBaseTest):
    def setUp(self):
        super().setUp()

        # Create a few fake templates
        self.useFixture(_environment_template_fixture())
        self.useFixture(_plug_template_fixture())
        self.useFixture(_daemon_template_fixture())
        self.useFixture(_invalid_template_fixture())


class BasicTemplateTest(TemplateTestBase):
    scenarios = [
        (
            "app template",
            {
                "snapcraft_yaml": textwrap.dedent(
                    """\
                    name: test
                    version: "1"
                    summary: test
                    description: test
                    base: core18
                    grade: stable
                    confinement: strict

                    apps:
                        test-app:
                            command: echo "hello"
                            {templates}

                    parts:
                        part1:
                            plugin: nil
                    """
                )
            },
        ),
        (
            "root template",
            {
                "snapcraft_yaml": textwrap.dedent(
                    """\
                    name: test
                    version: "1"
                    summary: test
                    description: test
                    base: core18
                    grade: stable
                    confinement: strict

                    {templates}

                    apps:
                        test-app:
                            command: echo "hello"

                    parts:
                        part1:
                            plugin: nil
                    """
                )
            },
        ),
    ]

    def test_template(self):
        config = self.make_snapcraft_project(
            self.snapcraft_yaml.format(templates="templates: [environment]")
        )

        # Verify that the template was removed
        self.expectThat(config.data, Not(Contains("templates")))
        self.expectThat(config.data["apps"]["test-app"], Not(Contains("templates")))

        # Verify that the template took effect on the app
        self.assertThat(config.data["apps"]["test-app"], Contains("environment"))
        self.assertThat(
            config.data["apps"]["test-app"]["environment"], Contains("TEST_TEMPLATE")
        )
        self.expectThat(
            config.data["apps"]["test-app"]["environment"]["TEST_TEMPLATE"], Equals(1)
        )

        # Verify that the template took effect on the part
        self.assertThat(config.parts.after_requests, Contains("part1"))
        self.expectThat(config.parts.after_requests["part1"], Equals(["template-part"]))

        # Verify that the template added a part
        self.assertThat(config.data["parts"], Contains("template-part"))
        self.expectThat(config.data["parts"]["template-part"]["plugin"], Equals("nil"))


class TemplateMergeTest(TemplateTestBase):
    scenarios = [
        (
            "merge plugs",
            {
                "app_definition": {
                    "command": "echo 'hello'",
                    "plugs": ["foo"],
                    "templates": ["plug"],
                },
                "expected_app_definition": {
                    "command": "echo 'hello'",
                    "plugs": ["foo", "test-plug"],
                },
            },
        ),
        (
            "merge environment",
            {
                "app_definition": {
                    "command": "echo 'hello'",
                    "environment": {"FOO": "BAR"},
                    "templates": ["environment"],
                },
                "expected_app_definition": {
                    "command": "echo 'hello'",
                    "environment": {"FOO": "BAR", "TEST_TEMPLATE": 1},
                },
            },
        ),
        (
            "scalars aren't overridden",
            {
                "app_definition": {
                    "command": "echo 'hello'",
                    "daemon": "forking",
                    "templates": ["daemon"],
                },
                "expected_app_definition": {
                    "command": "echo 'hello'",
                    "daemon": "forking",
                },
            },
        ),
    ]

    def test_template_merge(self):
        snapcraft_yaml = textwrap.dedent(
            """\
            name: test
            version: "1"
            summary: test
            description: test
            base: core18
            grade: stable
            confinement: strict

            apps:
                test-app:
                    {app_definition}

            parts:
                part1:
                    plugin: nil
            """
        )
        config = self.make_snapcraft_project(
            snapcraft_yaml.format(app_definition=self.app_definition)
        )

        # Verify that the template was removed
        self.expectThat(config.data, Not(Contains("templates")))
        self.expectThat(config.data["apps"]["test-app"], Not(Contains("templates")))

        # Verify that the template took effect on the app
        self.assertThat(
            config.data["apps"]["test-app"], Equals(self.expected_app_definition)
        )


class InvalidTemplateTest(TemplateTestBase):
    def test_invalid_root_template_format(self):
        raised = self.assertRaises(
            errors.YamlValidationError,
            self.make_snapcraft_project,
            textwrap.dedent(
                """\
                name: test
                version: "1"
                summary: test
                description: test
                base: core18
                grade: stable
                confinement: strict

                templates: I-should-be-a-list

                apps:
                    test-app:
                        command: echo "hello"

                parts:
                    part1:
                        plugin: nil
                """
            ),
        )

        self.assertThat(
            str(raised),
            Contains(
                "The 'templates' property does not match the required schema: "
                "'I-should-be-a-list' is not of type 'array'"
            ),
        )

    def test_invalid_app_template_format(self):
        raised = self.assertRaises(
            errors.YamlValidationError,
            self.make_snapcraft_project,
            textwrap.dedent(
                """\
                name: test
                version: "1"
                summary: test
                description: test
                base: core16
                grade: stable
                confinement: strict

                apps:
                    test-app:
                        command: echo "hello"
                        templates: I-should-be-a-list

                parts:
                    part1:
                        plugin: nil
                """
            ),
        )

        self.assertThat(
            str(raised),
            Contains(
                "The 'templates' property does not match the required schema: "
                "'I-should-be-a-list' is not of type 'array'"
            ),
        )

    def test_invalid_template_is_validated(self):
        raised = self.assertRaises(
            errors.YamlValidationError,
            self.make_snapcraft_project,
            textwrap.dedent(
                """\
                name: test
                version: "1"
                summary: test
                description: test
                base: core18
                grade: stable
                confinement: strict

                apps:
                    test-app:
                        command: echo "hello"
                        templates: [invalid]

                parts:
                    part1:
                        plugin: nil
                """
            ),
        )

        self.assertThat(str(raised), Contains("'unsupported-key' was unexpected"))

    def test_non_existing_template(self):
        self.assertRaises(
            errors.TemplateNotFoundError,
            self.make_snapcraft_project,
            textwrap.dedent(
                """\
                name: test
                version: "1"
                summary: test
                description: test
                base: core16
                grade: stable
                confinement: strict

                apps:
                    test-app:
                        command: echo "hello"
                        templates: [not-a-template]

                parts:
                    part1:
                        plugin: nil
                """
            ),
        )

    def test_conflicting_part(self):
        raised = self.assertRaises(
            errors.TemplatePartConflictError,
            self.make_snapcraft_project,
            textwrap.dedent(
                """\
                name: test
                version: "1"
                summary: test
                description: test
                base: core18
                grade: stable
                confinement: strict

                apps:
                    test-app:
                        command: echo "hello"
                        templates: [environment]

                parts:
                    template-part:
                        plugin: nil
                """
            ),
        )

        self.assertThat(raised.template_name, Equals("environment"))
        self.assertThat(raised.part_name, Equals("template-part"))

    def test_no_base(self):
        self.assertRaises(
            errors.TemplateBaseRequiredError,
            self.make_snapcraft_project,
            textwrap.dedent(
                """\
                name: test
                version: "1"
                summary: test
                description: test
                grade: stable
                confinement: strict

                apps:
                    test-app:
                        command: echo "hello"
                        templates: [environment]

                parts:
                    template-part:
                        plugin: nil
                """
            ),
        )

    def test_unsupported_base(self):
        raised = self.assertRaises(
            errors.TemplateUnsupportedBaseError,
            self.make_snapcraft_project,
            textwrap.dedent(
                """\
                name: test
                version: "1"
                summary: test
                description: test
                grade: stable
                confinement: strict
                base: unsupported

                apps:
                    test-app:
                        command: echo "hello"
                        templates: [environment]

                parts:
                    template-part:
                        plugin: nil
                """
            ),
        )

        self.assertThat(raised.template_name, Equals("environment"))
        self.assertThat(raised.base, Equals("unsupported"))


def _environment_template_fixture():
    class EnvironmentTemplate(Template):
        @classmethod
        def supported_bases(self):
            return {"core18"}

        def app_snippet(self):
            return {"environment": {"TEST_TEMPLATE": 1}}

        def part_snippet(self):
            return {"after": ["template-part"]}

        def parts(self):
            return {"template-part": {"plugin": "nil"}}

    return fixture_setup.FakeTemplate("environment", EnvironmentTemplate)


def _plug_template_fixture():
    class PlugTemplate(Template):
        @classmethod
        def supported_bases(self):
            return {"core18"}

        def app_snippet(self):
            return {"plugs": ["test-plug"]}

    return fixture_setup.FakeTemplate("plug", PlugTemplate)


def _daemon_template_fixture():
    class DaemonTemplate(Template):
        @classmethod
        def supported_bases(self):
            return {"core18"}

        def app_snippet(self):
            return {"daemon": "simple"}

    return fixture_setup.FakeTemplate("daemon", DaemonTemplate)


def _invalid_template_fixture():
    class InvalidTemplate(Template):
        @classmethod
        def supported_bases(self):
            return {"core18"}

        def app_snippet(self):
            return {"unsupported-key": "value"}

    return fixture_setup.FakeTemplate("invalid", InvalidTemplate)
