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

import pathlib
from textwrap import dedent

from testtools.matchers import Equals

from snapcraft.internal import project_loader
from snapcraft.project import Project
from tests import fixture_setup

from . import LoadPartBaseTest, ProjectLoaderBaseTest


def get_project_config(snapcraft_yaml_content):
    snapcraft_yaml_path = pathlib.Path("Snapcraft.yaml")
    with snapcraft_yaml_path.open("w") as snapcraft_yaml_file:
        print(snapcraft_yaml_content, file=snapcraft_yaml_file)

    project = Project(snapcraft_yaml_file_path=snapcraft_yaml_path.as_posix())
    return project_loader.load_config(project)


class TestParts(ProjectLoaderBaseTest):
    def make_snapcraft_project(self, parts):
        snapcraft_yaml = fixture_setup.SnapcraftYaml(self.path)
        snapcraft_yaml.update_part("part1", dict(plugin="nil"))
        for part_name, part in parts:
            snapcraft_yaml.update_part(part_name, part)
        self.useFixture(snapcraft_yaml)

        project = Project(
            snapcraft_yaml_file_path=snapcraft_yaml.snapcraft_yaml_file_path
        )
        return project_loader.load_config(project)

    def test_get_parts_none(self):
        project_config = self.make_snapcraft_project([("part1", dict(plugin="nil"))])
        self.assertThat(project_config.parts.get_part("not-a-part"), Equals(None))

    def test_after_inexistent_part(self):
        raised = self.assertRaises(
            project_loader.errors.SnapcraftAfterPartMissingError,
            self.make_snapcraft_project,
            [("part1", dict(plugin="nil", after=["inexistent-part"]))],
        )

        self.assertThat(raised.part_name, Equals("part1"))
        self.assertThat(raised.after_part_name, Equals("inexistent-part"))


class TestPartOrder:

    scenarios = [
        (
            "part1 then part2",
            {
                "contents": dedent(
                    """\
                name: test
                base: core18
                version: "1"
                summary: test
                description: test
                confinement: strict

                parts:
                  part1:
                    plugin: nil
                  part2:
                    plugin: nil
                """
                ),
                "expected_order": ["part1", "part2"],
            },
        ),
        (
            "part2 then part1",
            {
                "contents": dedent(
                    """\
                name: test
                base: core18
                version: "1"
                summary: test
                description: test
                confinement: strict

                parts:
                  part2:
                    plugin: nil
                  part1:
                    plugin: nil
                """
                ),
                "expected_order": ["part1", "part2"],
            },
        ),
        (
            "single after",
            {
                "contents": dedent(
                    """\
                name: test
                base: core18
                version: "1"
                summary: test
                description: test
                confinement: strict

                parts:
                  part2:
                    plugin: nil
                    after: [part3]
                  part1:
                    plugin: nil
                  part3:
                    plugin: nil
                """
                ),
                "expected_order": ["part1", "part3", "part2"],
            },
        ),
        (
            "multiple after",
            {
                "contents": dedent(
                    """\
                name: test
                base: core18
                version: "1"
                summary: test
                description: test
                confinement: strict

                parts:
                  part2:
                    plugin: nil
                    after: [part3]
                  part1:
                    plugin: nil
                    after: [part3]
                  part3:
                    plugin: nil
                """
                ),
                "expected_order": ["part3", "part1", "part2"],
            },
        ),
    ]

    def test_part_order_consistency(self, tmp_work_path, contents, expected_order):
        """Test that parts are always processed in the same order."""

        project_config = get_project_config(contents)

        assert len(project_config.all_parts) == len(expected_order)

        for part, expected_name in zip(project_config.all_parts, expected_order):
            assert part.name == expected_name


class PluginLoadTest(LoadPartBaseTest):
    def test_plugin_loading(self):
        self.make_snapcraft_project(
            dedent(
                """\
            name: test
            base: core18
            version: "1"
            summary: test
            description: test
            confinement: strict
            grade: stable

            parts:
              part1:
                plugin: nil
                stage-packages: [fswebcam]
            """
            )
        )

        self.mock_load_part.assert_called_with(
            "part1",
            "nil",
            {"stage-packages": ["fswebcam"], "plugin": "nil", "stage": [], "prime": []},
        )
