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
from snapcraft.internal.project_loader._extensions._extension import Extension

from tests import fixture_setup
from .. import ProjectLoaderBaseTest


class ExtensionTestBase(ProjectLoaderBaseTest):
    def setUp(self):
        super().setUp()

        # Create a few fake extensions
        self.useFixture(_environment_extension_fixture())
        self.useFixture(_plug_extension_fixture())
        self.useFixture(_daemon_extension_fixture())
        self.useFixture(_invalid_extension_fixture())


class BasicExtensionTest(ExtensionTestBase):
    scenarios = [
        (
            "app extension",
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
                            {extensions}

                    parts:
                        part1:
                            plugin: nil
                    """
                )
            },
        ),
        (
            "root extension",
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

                    {extensions}

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

    def test_extension(self):
        config = self.make_snapcraft_project(
            self.snapcraft_yaml.format(extensions="extensions: [environment]")
        )

        # Verify that the extension was removed
        self.expectThat(config.data, Not(Contains("extensions")))
        self.expectThat(config.data["apps"]["test-app"], Not(Contains("extensions")))

        # Verify that the extension took effect on the app
        self.assertThat(config.data["apps"]["test-app"], Contains("environment"))
        self.assertThat(
            config.data["apps"]["test-app"]["environment"], Contains("TEST_EXTENSION")
        )
        self.expectThat(
            config.data["apps"]["test-app"]["environment"]["TEST_EXTENSION"], Equals(1)
        )

        # Verify that the extension took effect on the part
        self.assertThat(config.parts.after_requests, Contains("part1"))
        self.expectThat(
            config.parts.after_requests["part1"], Equals(["extension-part"])
        )

        # Verify that the extension added a part
        self.assertThat(config.data["parts"], Contains("extension-part"))
        self.expectThat(config.data["parts"]["extension-part"]["plugin"], Equals("nil"))


class ExtensionMergeTest(ExtensionTestBase):
    scenarios = [
        (
            "merge plugs",
            {
                "app_definition": {
                    "command": "echo 'hello'",
                    "plugs": ["foo"],
                    "extensions": ["plug"],
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
                    "extensions": ["environment"],
                },
                "expected_app_definition": {
                    "command": "echo 'hello'",
                    "environment": {"FOO": "BAR", "TEST_EXTENSION": 1},
                },
            },
        ),
        (
            "scalars aren't overridden",
            {
                "app_definition": {
                    "command": "echo 'hello'",
                    "daemon": "forking",
                    "extensions": ["daemon"],
                },
                "expected_app_definition": {
                    "command": "echo 'hello'",
                    "daemon": "forking",
                },
            },
        ),
    ]

    def test_extension_merge(self):
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

        # Verify that the extension was removed
        self.expectThat(config.data, Not(Contains("extensions")))
        self.expectThat(config.data["apps"]["test-app"], Not(Contains("extensions")))

        # Verify that the extension took effect on the app
        self.assertThat(
            config.data["apps"]["test-app"], Equals(self.expected_app_definition)
        )


class InvalidExtensionTest(ExtensionTestBase):
    def test_invalid_root_extension_format(self):
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

                extensions: I-should-be-a-list

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
                "The 'extensions' property does not match the required schema: "
                "'I-should-be-a-list' is not of type 'array'"
            ),
        )

    def test_invalid_app_extension_format(self):
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
                        extensions: I-should-be-a-list

                parts:
                    part1:
                        plugin: nil
                """
            ),
        )

        self.assertThat(
            str(raised),
            Contains(
                "The 'extensions' property does not match the required schema: "
                "'I-should-be-a-list' is not of type 'array'"
            ),
        )

    def test_invalid_extension_is_validated(self):
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
                        extensions: [invalid]

                parts:
                    part1:
                        plugin: nil
                """
            ),
        )

        self.assertThat(str(raised), Contains("'unsupported-key' was unexpected"))

    def test_non_existing_extension(self):
        self.assertRaises(
            errors.ExtensionNotFoundError,
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
                        extensions: [not-a-extension]

                parts:
                    part1:
                        plugin: nil
                """
            ),
        )

    def test_conflicting_part(self):
        raised = self.assertRaises(
            errors.ExtensionPartConflictError,
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
                        extensions: [environment]

                parts:
                    extension-part:
                        plugin: nil
                """
            ),
        )

        self.assertThat(raised.extension_name, Equals("environment"))
        self.assertThat(raised.part_name, Equals("extension-part"))

    def test_no_base(self):
        self.assertRaises(
            errors.ExtensionBaseRequiredError,
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
                        extensions: [environment]

                parts:
                    extension-part:
                        plugin: nil
                """
            ),
        )

    def test_unsupported_base(self):
        raised = self.assertRaises(
            errors.ExtensionUnsupportedBaseError,
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
                        extensions: [environment]

                parts:
                    extension-part:
                        plugin: nil
                """
            ),
        )

        self.assertThat(raised.extension_name, Equals("environment"))
        self.assertThat(raised.base, Equals("unsupported"))


def _environment_extension_fixture():
    class EnvironmentExtension(Extension):
        supported_bases = ("core18",)

        def __init__(self, yaml_data):
            super().__init__(yaml_data)
            self.app_snippet = {"environment": {"TEST_EXTENSION": 1}}
            self.part_snippet = {"after": ["extension-part"]}
            self.parts = {"extension-part": {"plugin": "nil"}}

    return fixture_setup.FakeExtension("environment", EnvironmentExtension)


def _plug_extension_fixture():
    class PlugExtension(Extension):
        supported_bases = ("core18",)

        def __init__(self, yaml_data):
            super().__init__(yaml_data)
            self.app_snippet = {"plugs": ["test-plug"]}

    return fixture_setup.FakeExtension("plug", PlugExtension)


def _daemon_extension_fixture():
    class DaemonExtension(Extension):
        supported_bases = ("core18",)

        def __init__(self, yaml_data):
            super().__init__(yaml_data)
            self.app_snippet = {"daemon": "simple"}

    return fixture_setup.FakeExtension("daemon", DaemonExtension)


def _invalid_extension_fixture():
    class InvalidExtension(Extension):
        supported_bases = ("core18",)

        def __init__(self, yaml_data):
            super().__init__(yaml_data)
            self.app_snippet = {"unsupported-key": "value"}

    return fixture_setup.FakeExtension("invalid", InvalidExtension)
