# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2018 Canonical Ltd
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
from unittest import mock

from testtools.matchers import Equals

from snapcraft.internal.project_loader import errors, supported_extension_names
from snapcraft.internal.project_loader._extensions._extension import Extension
from tests import fixture_setup

from . import CommandBaseTestCase


class ExtensionsCommandTest(CommandBaseTestCase):
    def setUp(self):
        super().setUp()

        # Create a few fake extensions
        self.useFixture(_test1_extension_fixture())
        self.useFixture(_test2_extension_fixture())
        self.useFixture(_test3_extension_fixture())
        self.useFixture(_test4_extension_fixture())

    @mock.patch(
        "pkgutil.iter_modules",
        return_value=(
            (None, "test1", None),
            (None, "test2", None),
            (None, "test3", None),
            (None, "_test4", None),
        ),
    )
    def test_list_extensions(self, fake_iter_modules):
        result = self.run_command(["extensions"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Equals(
                textwrap.dedent(
                    """\
                    Extension name    Supported bases
                    ----------------  -----------------
                    test1             core16
                    test2             core16
                    test3             core16, core18
                    """
                )
            ),
        )

    def test_extension(self):
        result = self.run_command(["extension", "test1"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Equals(
                textwrap.dedent(
                    """\
                    This is the Test1 extension.

                    It does stuff.
                    """
                )
            ),
        )

        result = self.run_command(["extension", "test2"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Equals(
                textwrap.dedent(
                    """\
                    This is the Test2 extension.

                    It does other stuff.
                    """
                )
            ),
        )

    def test_extension_missing_docs(self):
        raised = self.assertRaises(
            errors.ExtensionMissingDocumentationError,
            self.run_command,
            ["extension", "test3"],
        )

        self.assertThat(
            str(raised),
            Equals(
                "The 'test3' extension appears to be missing documentation.\n"
                "We would appreciate it if you created a bug report about this at "
                "https://launchpad.net/snapcraft/+filebug"
            ),
        )

    def test_expand_extensions(self):
        self.make_snapcraft_yaml(
            textwrap.dedent(
                """\
                name: test
                version: '1'
                summary: test
                description: test
                base: core16
                grade: stable
                confinement: strict

                apps:
                    test-app:
                        command: echo "hello"
                        extensions: [test1]

                parts:
                    test-part:
                        plugin: nil
                """
            )
        )

        result = self.run_command(["expand-extensions"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Equals(
                textwrap.dedent(
                    """\
                    name: test
                    version: '1'
                    summary: test
                    description: test
                    base: core16
                    grade: stable
                    confinement: strict
                    apps:
                      test-app:
                        command: echo "hello"
                        environment:
                          EXTENSION_NAME: test1
                    parts:
                      test-part:
                        plugin: nil
                        after:
                        - extension-part
                      extension-part:
                        plugin: nil
                    """
                )
            ),
        )

    def test_all_extension_docs(self):
        for extension_name in supported_extension_names():
            try:
                self.run_command(["extension", extension_name])
            except Exception:
                self.fail(
                    "The {!r} extension appears to be missing documentation".format(
                        extension_name
                    )
                )


def _test1_extension_fixture():
    class ExtensionImpl(Extension):
        """This is the Test1 extension.

        It does stuff.
        """

        @staticmethod
        def get_supported_bases() -> Tuple[str, ...]:
            return ("core16",)

        @staticmethod
        def get_supported_confinement() -> Tuple[str, ...]:
            return ("strict",)

        def __init__(self, extension_name, yaml_data):
            super().__init__(extension_name=extension_name, yaml_data=yaml_data)
            self.app_snippet = {"environment": {"EXTENSION_NAME": "test1"}}
            self.part_snippet = {"after": ["extension-part"]}
            self.parts = {"extension-part": {"plugin": "nil"}}

    return fixture_setup.FakeExtension("test1", ExtensionImpl)


def _test2_extension_fixture():
    class ExtensionImpl(Extension):
        """This is the Test2 extension.

        It does other stuff.
        """

        @staticmethod
        def get_supported_bases() -> Tuple[str, ...]:
            return ("core16",)

        @staticmethod
        def get_supported_confinement() -> Tuple[str, ...]:
            return ("strict",)

        def __init__(self, extension_name, yaml_data):
            super().__init__(extension_name=extension_name, yaml_data=yaml_data)
            self.part_snippet = {"after": ["extension-part"]}
            self.parts = {"extension-part": {"plugin": "nil"}}

    return fixture_setup.FakeExtension("test2", ExtensionImpl)


def _test3_extension_fixture():
    class ExtensionImpl(Extension):
        @staticmethod
        def get_supported_bases() -> Tuple[str, ...]:
            return ("core16", "core18")

        @staticmethod
        def get_supported_confinement() -> Tuple[str, ...]:
            return ("strict",)

        def __init__(self, extension_name, yaml_data):
            super().__init__(extension_name=extension_name, yaml_data=yaml_data)
            self.parts = {"extension-part": {"plugin": "nil"}}

    return fixture_setup.FakeExtension("test3", ExtensionImpl)


def _test4_extension_fixture():
    class ExtensionImpl(Extension):
        @staticmethod
        def get_supported_bases() -> Tuple[str, ...]:
            return ("core16", "core18")

        @staticmethod
        def get_supported_confinement() -> Tuple[str, ...]:
            return ("strict",)

        def __init__(self, extension_name, yaml_data):
            super().__init__(extension_name=extension_name, yaml_data=yaml_data)
            self.parts = {"extension-part": {"plugin": "nil"}}

    return fixture_setup.FakeExtension("_test4", ExtensionImpl)
