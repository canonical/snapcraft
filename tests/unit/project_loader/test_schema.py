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
import pathlib
from textwrap import dedent

import fixtures
import pytest
from testscenarios.scenarios import multiply_scenarios
from testtools.matchers import Contains, Equals

import snapcraft.yaml_utils.errors
from snapcraft import project
from snapcraft.internal.errors import PluginError
from snapcraft.internal.project_loader import errors, load_config
from tests import fixture_setup

from . import ProjectLoaderBaseTest


def get_project_config(snapcraft_yaml_content, target_deb_arch=None):
    snapcraft_yaml_path = pathlib.Path("Snapcraft.yaml")
    with snapcraft_yaml_path.open("w") as snapcraft_yaml_file:
        print(snapcraft_yaml_content, file=snapcraft_yaml_file)

    snapcraft_project = project.Project(
        snapcraft_yaml_file_path=snapcraft_yaml_path.as_posix(),
        target_deb_arch=target_deb_arch,
    )
    return load_config(snapcraft_project)


class TestValidArchitectures:

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

    def test(
        self,
        tmp_work_path,
        yaml,
        target_arch,
        expected_amd64,
        expected_i386,
        expected_armhf,
    ):
        snippet = "architectures: {}".format(yaml) if yaml else ""
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

        c = get_project_config(snapcraft_yaml, target_deb_arch=target_arch)

        expected_targets = {
            "amd64": expected_amd64,
            "i386": expected_i386,
            "armhf": expected_armhf,
        }
        assert c.data["architectures"] == expected_targets[target_arch]


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
            snapcraft.yaml_utils.errors.YamlValidationError,
            self.make_snapcraft_project,
            apps,
        )
        expected = (
            "The {path!r} property does not match the required schema: "
            "{alias!r} does not match ".format(
                path="apps/test/aliases[0]", alias=".test"
            )
        )
        self.assertThat(str(raised), Contains(expected))


@pytest.mark.parametrize("entry", ["slots", "plugs"])
def test_loading_additional_properties_fails(tmp_work_path, entry):
    snapcraft_yaml = dedent(
        f"""\
            name: my-package-1
            base: core18
            version: 1.0-snapcraft1~ppa1
            summary: my summary less that 79 chars
            description: description which can be pretty long
            parts:
                part1:
                    plugin: nil
                    {entry}: [{entry}1]
        """
    )

    with pytest.raises(PluginError):
        get_project_config(snapcraft_yaml)
