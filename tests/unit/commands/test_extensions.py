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

from unittest import mock
from testtools.matchers import Equals

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

    @mock.patch(
        "pkgutil.iter_modules",
        return_value=(
            (None, "test1", None),
            (None, "test2", None),
            (None, "test3", None),
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
                    The test1 extension adds the following to apps that use it:
                        environment:
                          EXTENSION_NAME: test1

                    It adds the following to all parts:
                        after:
                        - extension-part

                    It adds the following part definitions:
                        extension-part:
                          plugin: nil

                    """  # Extra line break due to click.echo
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
                    The test2 extension adds the following to all parts:
                        after:
                        - extension-part

                    It adds the following part definitions:
                        extension-part:
                          plugin: nil

                    """  # Extra line break due to click.echo
                )
            ),
        )

        result = self.run_command(["extension", "test3"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Equals(
                textwrap.dedent(
                    """\
                    The test3 extension adds the following part definitions:
                        extension-part:
                          plugin: nil

                    """  # Extra line break due to click.echo
                )
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


def _test1_extension_fixture():
    class Test1Extension(Extension):
        supported_bases = ("core16",)

        def __init__(self, yaml_data):
            super().__init__(yaml_data)
            self.app_snippet = {"environment": {"EXTENSION_NAME": "test1"}}
            self.part_snippet = {"after": ["extension-part"]}
            self.parts = {"extension-part": {"plugin": "nil"}}

    return fixture_setup.FakeExtension("test1", Test1Extension)


def _test2_extension_fixture():
    class Test2Extension(Extension):
        supported_bases = ("core16",)

        def __init__(self, yaml_data):
            super().__init__(yaml_data)
            self.part_snippet = {"after": ["extension-part"]}
            self.parts = {"extension-part": {"plugin": "nil"}}

    return fixture_setup.FakeExtension("test2", Test2Extension)


def _test3_extension_fixture():
    class Test3Extension(Extension):
        supported_bases = ("core16", "core18")

        def __init__(self, yaml_data):
            super().__init__(yaml_data)
            self.parts = {"extension-part": {"plugin": "nil"}}

    return fixture_setup.FakeExtension("test3", Test3Extension)
