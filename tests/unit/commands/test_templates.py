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

from snapcraft.internal.project_loader._templates._template import Template

from tests import fixture_setup
from . import CommandBaseTestCase


class TemplatesCommandTest(CommandBaseTestCase):
    def setUp(self):
        super().setUp()

        # Create a few fake templates
        self.useFixture(_test1_template_fixture())
        self.useFixture(_test2_template_fixture())
        self.useFixture(_test3_template_fixture())

    @mock.patch(
        "pkgutil.iter_modules",
        return_value=(
            (None, "test1", None),
            (None, "test2", None),
            (None, "test3", None),
        ),
    )
    def test_list_templates(self, fake_iter_modules):
        result = self.run_command(["templates"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Equals(
                textwrap.dedent(
                    """\
                    Template name    Supported bases
                    ---------------  -----------------
                    test1            core16
                    test2            core16
                    test3            core16, core18
                    """
                )
            ),
        )

    def test_template(self):
        result = self.run_command(["template", "test1"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Equals(
                textwrap.dedent(
                    """\
                    The test1 template adds the following to apps that use it:
                        environment:
                          TEMPLATE_NAME: test1

                    It adds the following to all parts:
                        after:
                        - template-part

                    It adds the following part definitions:
                        template-part:
                          plugin: nil

                    """  # Extra line break due to click.echo
                )
            ),
        )

        result = self.run_command(["template", "test2"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Equals(
                textwrap.dedent(
                    """\
                    The test2 template adds the following to all parts:
                        after:
                        - template-part

                    It adds the following part definitions:
                        template-part:
                          plugin: nil

                    """  # Extra line break due to click.echo
                )
            ),
        )

        result = self.run_command(["template", "test3"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Equals(
                textwrap.dedent(
                    """\
                    The test3 template adds the following part definitions:
                        template-part:
                          plugin: nil

                    """  # Extra line break due to click.echo
                )
            ),
        )

    def test_expand_templates(self):
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
                        templates: [test1]

                parts:
                    test-part:
                        plugin: nil
                """
            )
        )

        result = self.run_command(["expand-templates"])

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
                          TEMPLATE_NAME: test1
                    parts:
                      test-part:
                        plugin: nil
                        after:
                        - template-part
                      template-part:
                        plugin: nil
                    """
                )
            ),
        )


def _test1_template_fixture():
    class Test1Template(Template):
        supported_bases = ("core16",)

        def __init__(self, yaml_data):
            super().__init__(yaml_data)
            self.app_snippet = {"environment": {"TEMPLATE_NAME": "test1"}}
            self.part_snippet = {"after": ["template-part"]}
            self.parts = {"template-part": {"plugin": "nil"}}

    return fixture_setup.FakeTemplate("test1", Test1Template)


def _test2_template_fixture():
    class Test2Template(Template):
        supported_bases = ("core16",)

        def __init__(self, yaml_data):
            super().__init__(yaml_data)
            self.part_snippet = {"after": ["template-part"]}
            self.parts = {"template-part": {"plugin": "nil"}}

    return fixture_setup.FakeTemplate("test2", Test2Template)


def _test3_template_fixture():
    class Test3Template(Template):
        supported_bases = ("core16", "core18")

        def __init__(self, yaml_data):
            super().__init__(yaml_data)
            self.parts = {"template-part": {"plugin": "nil"}}

    return fixture_setup.FakeTemplate("test3", Test3Template)
