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

import os
import textwrap

import fixtures
from testtools.matchers import Equals

from snapcraft.internal import common

from . import CommandBaseTestCase


class TemplatesCommandTest(CommandBaseTestCase):
    def setUp(self):
        super().setUp()

        templates_dir = self.useFixture(fixtures.TempDir()).path
        common.set_templatesdir(templates_dir)

        # Create a a few fake templates
        for template_name in ("template1", "template2"):
            template_dir = os.path.join(templates_dir, template_name)
            os.mkdir(template_dir)
            with open(os.path.join(template_dir, "template.yaml"), "w") as f:
                f.write(
                    textwrap.dedent(
                        """\
                        core16:
                            apps:
                                '*':
                                    environment:
                                        TEMPLATE_NAME: {}

                            parts:
                                '*':
                                    after: [template-part]

                                template-part:
                                    plugin: nil
                        """
                    ).format(template_name)
                )

        template_dir = os.path.join(templates_dir, "template3")
        os.mkdir(template_dir)
        with open(os.path.join(template_dir, "template.yaml"), "w") as f:
            f.write(
                textwrap.dedent(
                    """\
                    core16:
                        parts:
                            template-part:
                                plugin: nil
                    core18:
                        parts:
                            template-part:
                                plugin: nil
                    """
                ).format(template_name)
            )

    def test_list_templates(self):
        result = self.run_command(["templates"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Equals(
                textwrap.dedent(
                    """\
                    Template name    Supported bases
                    ---------------  -----------------
                    template1        core16
                    template2        core16
                    template3        core16, core18
                    """
                )
            ),
        )

    def test_template(self):
        result = self.run_command(["template", "template1"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Equals(
                textwrap.dedent(
                    """\
                    core16:
                        apps:
                            '*':
                                environment:
                                    TEMPLATE_NAME: template1

                        parts:
                            '*':
                                after: [template-part]

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
                        templates: [template1]

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
                          TEMPLATE_NAME: template1
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
