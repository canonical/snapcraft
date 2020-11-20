# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018-2019 Canonical Ltd
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
from typing import Tuple

from testtools.matchers import Contains, Equals, Not

import snapcraft.yaml_utils.errors
from snapcraft.internal.project_loader import errors
from snapcraft.internal.project_loader._extensions._extension import Extension
from tests import fixture_setup

from .. import ProjectLoaderBaseTest


class ExtensionTestBase(ProjectLoaderBaseTest):
    def set_attributes(self, kwargs):
        self.__dict__.update(kwargs)

    def setUp(self):
        super().setUp()

        # Create a few fake extensions
        self.useFixture(_build_environment_extension_fixture())
        self.useFixture(_build_environment2_extension_fixture())
        self.useFixture(_environment_extension_fixture())
        self.useFixture(_plug_extension_fixture())
        self.useFixture(_plug2_extension_fixture())
        self.useFixture(_daemon_extension_fixture())
        self.useFixture(_adopt_info_extension_fixture())
        self.useFixture(_invalid_extension_fixture())


class BasicExtensionTest(ExtensionTestBase):
    def test_one_app_one_extension(self):
        config = self.make_snapcraft_project(
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
                        part1:
                            plugin: nil
                    """
            )
        )

        # Verify that the extension was removed
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

        # Verify that the extension added a root property
        self.assertThat(config.data, Contains("environment"))
        self.expectThat(config.data["environment"], Contains("TEST_EXTENSION"))

    def test_two_apps_one_extension(self):
        config = self.make_snapcraft_project(
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
                        app1:
                            command: app1
                            extensions: [environment]
                        app2:
                            command: app2
                            extensions: [environment]

                    parts:
                        part1:
                            plugin: nil
                    """
            )
        )

        # Verify that the extension was removed from both apps, and took effect on both
        for app_name in ("app1", "app2"):
            self.expectThat(config.data["apps"][app_name], Not(Contains("extensions")))
            self.assertThat(config.data["apps"][app_name], Contains("environment"))
            self.assertThat(
                config.data["apps"][app_name]["environment"], Contains("TEST_EXTENSION")
            )
            self.expectThat(
                config.data["apps"][app_name]["environment"]["TEST_EXTENSION"],
                Equals(1),
            )

        # Verify that the extension took effect on the part only once
        self.assertThat(config.parts.after_requests, Contains("part1"))
        self.expectThat(
            config.parts.after_requests["part1"], Equals(["extension-part"])
        )

        # Verify that the extension added a single part
        self.assertThat(config.data["parts"], Contains("extension-part"))
        self.expectThat(config.data["parts"]["extension-part"]["plugin"], Equals("nil"))

        # Verify that the extension added a root property
        self.assertThat(config.data, Contains("environment"))
        self.expectThat(config.data["environment"], Contains("TEST_EXTENSION"))

    def test_multiple_extensions(self):
        config = self.make_snapcraft_project(
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
                            command: test-app
                            extensions: [environment, daemon]

                    parts:
                        part1:
                            plugin: nil
                    """
            )
        )

        # Verify that both extensions were applied to the app
        self.expectThat(config.data["apps"]["test-app"], Not(Contains("extensions")))
        self.assertThat(config.data["apps"]["test-app"], Contains("environment"))
        self.assertThat(
            config.data["apps"]["test-app"]["environment"], Contains("TEST_EXTENSION")
        )
        self.expectThat(
            config.data["apps"]["test-app"]["environment"]["TEST_EXTENSION"], Equals(1)
        )
        self.assertThat(config.data["apps"]["test-app"], Contains("daemon"))
        self.expectThat(config.data["apps"]["test-app"]["daemon"], Equals("simple"))

        # Verify that the environment took effect
        self.assertThat(config.parts.after_requests, Contains("part1"))
        self.expectThat(
            config.parts.after_requests["part1"], Equals(["extension-part"])
        )

        # Verify that the environment extension added a part
        self.assertThat(config.data["parts"], Contains("extension-part"))
        self.expectThat(config.data["parts"]["extension-part"]["plugin"], Equals("nil"))

        # Verify that the environment extension added a root property
        self.assertThat(config.data, Contains("environment"))
        self.expectThat(config.data["environment"], Contains("TEST_EXTENSION"))


class ExtensionOrderConsistencyTest(ExtensionTestBase):
    def assert_extensions(self, extensions):
        config = self.make_snapcraft_project(
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
                            command: test-app
                            extensions: {extensions}

                    parts:
                        part1:
                            plugin: nil
                    """
            ).format(extensions=extensions)
        )

        # Verify that both extensions were applied to the app
        self.expectThat(config.data["apps"]["test-app"], Not(Contains("extensions")))
        self.assertThat(config.data["apps"]["test-app"], Contains("plugs"))
        self.assertThat(
            config.data["apps"]["test-app"]["plugs"],
            Equals(["test-plug2", "test-plug"]),
        )

    def test_extension_merge_plug_plug2(self):
        self.assert_extensions("[plug, plug2]")

    def test_extension_merge_plug2_plug(self):
        self.assert_extensions("[plug2, plug]")


class ExtensionMergeTest(ExtensionTestBase):
    def run_test(self):
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
            {part_definition}
            """
        )
        config = self.make_snapcraft_project(
            snapcraft_yaml.format(
                app_definition=textwrap.indent(self.app_definition, " " * 8),
                part_definition=textwrap.indent(self.part_definition, " " * 8),
            )
        )

        # Verify that the extension was removed
        self.expectThat(config.data, Not(Contains("extensions")))
        self.expectThat(config.data["apps"]["test-app"], Not(Contains("extensions")))

        # Verify that the extension took effect on the app
        self.assertThat(
            config.data["apps"]["test-app"], Equals(self.expected_app_definition)
        )
        # Verify that the extension took effect on the part
        self.assertThat(
            config.data["parts"]["part1"], Equals(self.expected_part_definition)
        )

    def test_merge_plugs(self):
        self.set_attributes(
            {
                "app_definition": textwrap.dedent(
                    """\
                    command: echo 'hello'
                    plugs: [foo]
                    extensions: [plug]
                    """
                ),
                "expected_app_definition": {
                    "command": "echo 'hello'",
                    "plugs": ["test-plug", "foo"],
                },
                "part_definition": textwrap.dedent(
                    """\
                    plugin: nil
                    """
                ),
                "expected_part_definition": {"plugin": "nil", "prime": [], "stage": []},
            }
        )

        self.run_test()

    def test_merge_environment(self):
        self.set_attributes(
            {
                "app_definition": textwrap.dedent(
                    """\
                    command: echo 'hello'
                    extensions: [environment]
                    environment:
                      FOO: BAR
                    """
                ),
                "expected_app_definition": {
                    "command": "echo 'hello'",
                    "environment": {"FOO": "BAR", "TEST_EXTENSION": 1},
                },
                "part_definition": textwrap.dedent(
                    """\
                    plugin: nil
                    """
                ),
                "expected_part_definition": {"plugin": "nil", "prime": [], "stage": []},
            }
        )

        self.run_test()

    def test_merge_build_environment(self):
        self.set_attributes(
            {
                "app_definition": textwrap.dedent(
                    """\
                    command: echo 'hello'
                    extensions: [buildenvironment]
                    """
                ),
                "expected_app_definition": {"command": "echo 'hello'"},
                "part_definition": textwrap.dedent(
                    """\
                    plugin: nil
                    build-environment:
                      - PATH: "$PATH:/part-path"
                    """
                ),
                "expected_part_definition": {
                    "plugin": "nil",
                    "build-environment": [
                        {"PATH": "$PATH:/extension-path"},
                        {"EXTKEY": "EXTVAL"},
                        {"PATH": "$PATH:/part-path"},
                    ],
                    "prime": [],
                    "stage": [],
                },
            }
        )

        self.run_test()

    def test_merge_multiple_build_environment(self):
        self.set_attributes(
            {
                "app_definition": textwrap.dedent(
                    """\
                    command: echo 'hello'
                    extensions: [buildenvironment, buildenvironment2]
                    """
                ),
                "expected_app_definition": {"command": "echo 'hello'"},
                "part_definition": textwrap.dedent(
                    """\
                    plugin: nil
                    build-environment:
                      - PATH: "$PATH:/part-path"
                    """
                ),
                "expected_part_definition": {
                    "plugin": "nil",
                    "build-environment": [
                        {"PATH": "$PATH:/extension-path2"},
                        {"EXTKEY": "EXTVAL2"},
                        {"PATH": "$PATH:/extension-path"},
                        {"EXTKEY": "EXTVAL"},
                        {"PATH": "$PATH:/part-path"},
                    ],
                    "prime": [],
                    "stage": [],
                },
            }
        )

        self.run_test()

    def test_scalar_no_override(self):
        self.set_attributes(
            {
                "app_definition": textwrap.dedent(
                    """\
                    command: echo 'hello'
                    daemon: forking
                    extensions: [daemon]
                    """
                ),
                "expected_app_definition": {
                    "command": "echo 'hello'",
                    "daemon": "forking",
                },
                "part_definition": textwrap.dedent(
                    """\
                    plugin: nil
                    """
                ),
                "expected_part_definition": {"plugin": "nil", "prime": [], "stage": []},
            }
        )

        self.run_test()


class ExtensionRootMergeTest(ExtensionTestBase):
    def run_test(self):
        snapcraft_yaml = textwrap.dedent(
            """\
            name: test
            version: "1"
            summary: test
            description: test
            base: core18
            grade: stable
            confinement: strict

            {root_definition}

            apps:
                test-app:
                    command: test-command
                    extensions: {extensions}

            parts:
                part1:
                    plugin: nil
            """
        )
        config = self.make_snapcraft_project(
            snapcraft_yaml.format(
                root_definition=self.root_definition, extensions=self.extensions
            )
        )

        # Verify that the extension was removed
        self.expectThat(config.data, Not(Contains("extensions")))

        # Verify that the extension took effect on the root of the YAML
        for key, value in self.expected_root_definition.items():
            self.assertThat(config.data[key], Equals(value))

    def test_merge_environment(self):
        self.set_attributes(
            {
                "root_definition": textwrap.dedent(
                    """\
                    environment:
                      FOO: BAR
                    """
                ),
                "extensions": "[environment]",
                "expected_root_definition": {
                    "environment": {"FOO": "BAR", "TEST_EXTENSION": 1}
                },
            }
        )

        self.run_test()

    def test_scalars_no_override(self):
        self.set_attributes(
            {
                "root_definition": textwrap.dedent(
                    """\
                    adopt-info: "test-part"
                    """
                ),
                "extensions": "[adopt]",
                "expected_root_definition": {"adopt-info": "test-part"},
            }
        )

        self.run_test()


class InvalidExtensionTest(ExtensionTestBase):
    def test_invalid_app_extension_format(self):
        raised = self.assertRaises(
            snapcraft.yaml_utils.errors.YamlValidationError,
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

    def test_duplicate_extensions(self):
        raised = self.assertRaises(
            snapcraft.yaml_utils.errors.YamlValidationError,
            self.make_snapcraft_project,
            textwrap.dedent(
                """\
                name: test
                version: "1"
                summary: test
                description: test
                grade: stable
                confinement: strict
                base: core18

                apps:
                    test-app:
                        command: echo "hello"
                        extensions: [environment, environment]

                parts:
                    my-part:
                        plugin: nil
                """
            ),
        )

        self.assertThat(
            str(raised),
            Contains(
                "The 'extensions' property does not match the required schema: "
                "['environment', 'environment'] has non-unique elements"
            ),
        )

    def test_invalid_extension_is_validated(self):
        raised = self.assertRaises(
            snapcraft.yaml_utils.errors.YamlValidationError,
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

    def test_unsupported_default_confinement(self):
        raised = self.assertRaises(
            errors.ExtensionUnsupportedConfinementError,
            self.make_snapcraft_project,
            textwrap.dedent(
                """\
                name: test
                version: "1"
                summary: test
                description: test
                grade: stable
                base: core18

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
        self.assertThat(raised.confinement, Equals("devmode"))

    def test_unsupported_confinement(self):
        raised = self.assertRaises(
            errors.ExtensionUnsupportedConfinementError,
            self.make_snapcraft_project,
            textwrap.dedent(
                """\
                name: test
                version: "1"
                summary: test
                description: test
                grade: stable
                confinement: unsupported
                base: core18

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
        self.assertThat(raised.confinement, Equals("unsupported"))


def _environment_extension_fixture():
    class ExtensionImpl(Extension):
        @staticmethod
        def get_supported_bases() -> Tuple[str, ...]:
            return ("core18",)

        @staticmethod
        def get_supported_confinement() -> Tuple[str, ...]:
            return ("strict",)

        def __init__(self, extension_name, yaml_data):
            super().__init__(extension_name=extension_name, yaml_data=yaml_data)
            self.root_snippet = {"environment": {"TEST_EXTENSION": 1}}
            self.app_snippet = {"environment": {"TEST_EXTENSION": 1}}
            self.part_snippet = {"after": ["extension-part"]}
            self.parts = {"extension-part": {"plugin": "nil"}}

    return fixture_setup.FakeExtension("environment", ExtensionImpl)


def _build_environment_extension_fixture():
    class ExtensionImpl(Extension):
        @staticmethod
        def get_supported_bases() -> Tuple[str, ...]:
            return ("core18",)

        @staticmethod
        def get_supported_confinement() -> Tuple[str, ...]:
            return ("strict",)

        def __init__(self, extension_name, yaml_data):
            super().__init__(extension_name=extension_name, yaml_data=yaml_data)
            self.root_snippet = {}
            self.app_snippet = {}
            self.part_snippet = {
                "after": ["extension-part"],
                "build-environment": [
                    {"PATH": "$PATH:/extension-path"},
                    {"EXTKEY": "EXTVAL"},
                ],
            }
            self.parts = {"extension-part": {"plugin": "nil"}}

    return fixture_setup.FakeExtension("buildenvironment", ExtensionImpl)


def _build_environment2_extension_fixture():
    class ExtensionImpl(Extension):
        @staticmethod
        def get_supported_bases() -> Tuple[str, ...]:
            return ("core18",)

        @staticmethod
        def get_supported_confinement() -> Tuple[str, ...]:
            return ("strict",)

        def __init__(self, extension_name, yaml_data):
            super().__init__(extension_name=extension_name, yaml_data=yaml_data)
            self.root_snippet = {}
            self.app_snippet = {}
            self.part_snippet = {
                "after": ["extension-part2"],
                "build-environment": [
                    {"PATH": "$PATH:/extension-path2"},
                    {"EXTKEY": "EXTVAL2"},
                ],
            }
            self.parts = {"extension-part2": {"plugin": "nil"}}

    return fixture_setup.FakeExtension("buildenvironment2", ExtensionImpl)


def _plug_extension_fixture():
    class ExtensionImpl(Extension):
        @staticmethod
        def get_supported_bases() -> Tuple[str, ...]:
            return ("core18",)

        @staticmethod
        def get_supported_confinement() -> Tuple[str, ...]:
            return ("strict",)

        def __init__(self, extension_name, yaml_data):
            super().__init__(extension_name=extension_name, yaml_data=yaml_data)
            self.app_snippet = {"plugs": ["test-plug"]}

    return fixture_setup.FakeExtension("plug", ExtensionImpl)


def _plug2_extension_fixture():
    class ExtensionImpl(Extension):
        @staticmethod
        def get_supported_bases() -> Tuple[str, ...]:
            return ("core18",)

        @staticmethod
        def get_supported_confinement() -> Tuple[str, ...]:
            return ("strict",)

        def __init__(self, extension_name, yaml_data):
            super().__init__(extension_name=extension_name, yaml_data=yaml_data)
            self.app_snippet = {"plugs": ["test-plug2"]}

    return fixture_setup.FakeExtension("plug2", ExtensionImpl)


def _daemon_extension_fixture():
    class ExtensionImpl(Extension):
        @staticmethod
        def get_supported_bases() -> Tuple[str, ...]:
            return ("core18",)

        @staticmethod
        def get_supported_confinement() -> Tuple[str, ...]:
            return ("strict",)

        def __init__(self, extension_name, yaml_data):
            super().__init__(extension_name=extension_name, yaml_data=yaml_data)
            self.app_snippet = {"daemon": "simple"}

    return fixture_setup.FakeExtension("daemon", ExtensionImpl)


def _adopt_info_extension_fixture():
    class ExtensionImpl(Extension):
        @staticmethod
        def get_supported_bases() -> Tuple[str, ...]:
            return ("core18",)

        @staticmethod
        def get_supported_confinement() -> Tuple[str, ...]:
            return ("strict",)

        def __init__(self, extension_name, yaml_data):
            super().__init__(extension_name=extension_name, yaml_data=yaml_data)
            self.root_snippet = {"adopt-info": "some-part-name"}

    return fixture_setup.FakeExtension("adopt", ExtensionImpl)


def _invalid_extension_fixture():
    class ExtensionImpl(Extension):
        @staticmethod
        def get_supported_bases() -> Tuple[str, ...]:
            return ("core18",)

        @staticmethod
        def get_supported_confinement() -> Tuple[str, ...]:
            return ("strict",)

        def __init__(self, extension_name, yaml_data):
            super().__init__(extension_name=extension_name, yaml_data=yaml_data)
            self.app_snippet = {"unsupported-key": "value"}

    return fixture_setup.FakeExtension("invalid", ExtensionImpl)
