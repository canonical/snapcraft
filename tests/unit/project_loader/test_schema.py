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

import logging
from textwrap import dedent

import fixtures
from testscenarios.scenarios import multiply_scenarios
from testtools.matchers import Contains, Equals

from . import ProjectLoaderBaseTest
from snapcraft.internal.errors import PluginError
from snapcraft.internal.project_loader import errors, load_config
from snapcraft import project
from tests import fixture_setup


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
            base: core18
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


class AliasesTest(ProjectLoaderBaseTest):
    def make_snapcraft_project(self, apps):
        snapcraft_yaml = fixture_setup.SnapcraftYaml(self.path)
        snapcraft_yaml.update_part("part1", dict(plugin="nil"))
        for app_name, app in apps:
            snapcraft_yaml.update_app(app_name, app)
        self.useFixture(snapcraft_yaml)

        p = project.Project(
            snapcraft_yaml_file_path=snapcraft_yaml.snapcraft_yaml_file_path
        )
        return load_config(p)

    def test_aliases(self):
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
            project.errors.YamlValidationError, self.make_snapcraft_project, apps
        )
        expected = (
            "The {path!r} property does not match the required schema: "
            "{alias!r} does not match ".format(
                path="apps/test/aliases[0]", alias=".test"
            )
        )
        self.assertThat(str(raised), Contains(expected))


class AdditionalPartPropertiesTest(ProjectLoaderBaseTest):

    scenarios = [("slots", dict(property="slots")), ("plugs", dict(property="plugs"))]

    def test_loading_properties(self):
        snapcraft_yaml = dedent(
            """\
            name: my-package-1
            base: core18
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
