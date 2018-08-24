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

from textwrap import dedent

from testtools.matchers import Contains, Equals

from . import LoadPartBaseTest, ProjectLoaderBaseTest

from snapcraft.internal.project_loader import errors
import snapcraft.internal.project_loader._config as _config
from tests import unit


class VariableExpansionTest(LoadPartBaseTest):
    def test_version_script(self):
        project_config = self.make_snapcraft_project(
            dedent(
                """\
            name: test
            version: "1.0"
            version-script: echo $SNAPCRAFT_PROJECT_VERSION-$SNAPCRAFT_PROJECT_GRADE
            summary: test
            description: nothing
            architectures: ['amd64']
            confinement: strict
            grade: devel

            parts:
              part1:
                plugin: go
                stage-packages: [fswebcam]
        """
            )
        )  # noqa: E501

        self.assertThat(project_config.data["version-script"], Equals("echo 1.0-devel"))

    def test_config_expands_filesets(self):
        self.make_snapcraft_project(
            dedent(
                """\
            name: test
            version: "1"
            summary: test
            description: test
            confinement: strict
            grade: stable

            parts:
              part1:
                plugin: go
                stage-packages: [fswebcam]
                filesets:
                  wget:
                    - /usr/lib/wget.so
                    - /usr/bin/wget
                  build-wget:
                    - /usr/lib/wget.a
                stage:
                  - $wget
                  - $build-wget
                prime:
                  - $wget
                  - /usr/share/my-icon.png
        """
            )
        )

        self.mock_load_part.assert_called_with(
            "part1",
            "go",
            {
                "prime": [
                    "/usr/lib/wget.so",
                    "/usr/bin/wget",
                    "/usr/share/my-icon.png",
                ],
                "plugin": "go",
                "stage-packages": ["fswebcam"],
                "stage": ["/usr/lib/wget.so", "/usr/bin/wget", "/usr/lib/wget.a"],
            },
        )

    def test_replace_snapcraft_variables(self):
        self.make_snapcraft_project(
            dedent(
                """\
            name: project-name
            version: "1"
            summary: test
            description: test
            confinement: strict

            parts:
              main:
                plugin: make
                source: $SNAPCRAFT_PROJECT_NAME-$SNAPCRAFT_PROJECT_VERSION
                make-options: [DEP=$SNAPCRAFT_STAGE]
            """
            )
        )

        self.mock_load_part.assert_called_with(
            "main",
            "make",
            {
                "source": "project-name-1",
                "plugin": "make",
                "stage": [],
                "prime": [],
                "make-options": ["DEP={}".format(self.stage_dir)],
            },
        )


class DependenciesTest(ProjectLoaderBaseTest):
    def setUp(self):
        super().setUp()

        self.config = self.make_snapcraft_project(
            dedent(
                dedent(
                    """\
            name: test
            version: "1"
            summary: test
            description: test
            confinement: strict
            grade: stable

            parts:
                main:
                    plugin: nil

                dependent:
                    plugin: nil
                    after: [main]

                nested-dependent:
                    plugin: nil
                    after: [dependent]"""
                )
            )
        )

    def assert_part_names(self, part_names, parts):
        self.assertThat({p.name for p in parts}, Equals(part_names))

    def test_get_dependencies(self):
        self.assertFalse(self.config.parts.get_dependencies("main"))
        self.assert_part_names(
            {"main"}, self.config.parts.get_dependencies("dependent")
        )
        self.assert_part_names(
            {"dependent"}, self.config.parts.get_dependencies("nested-dependent")
        )

    def test_get_dependencies_recursive(self):
        self.assertFalse(self.config.parts.get_dependencies("main", recursive=True))
        self.assert_part_names(
            {"main"}, self.config.parts.get_dependencies("dependent", recursive=True)
        )
        self.assert_part_names(
            {"main", "dependent"},
            self.config.parts.get_dependencies("nested-dependent", recursive=True),
        )

    def test_get_reverse_dependencies(self):
        self.assertFalse(self.config.parts.get_reverse_dependencies("nested-dependent"))
        self.assert_part_names(
            {"nested-dependent"},
            self.config.parts.get_reverse_dependencies("dependent"),
        )
        self.assert_part_names(
            {"dependent"}, self.config.parts.get_reverse_dependencies("main")
        )

    def test_get_reverse_dependencies_recursive(self):
        self.assertFalse(
            self.config.parts.get_reverse_dependencies(
                "nested-dependent", recursive=True
            )
        )
        self.assert_part_names(
            {"nested-dependent"},
            self.config.parts.get_reverse_dependencies("dependent", recursive=True),
        )
        self.assert_part_names(
            {"dependent", "nested-dependent"},
            self.config.parts.get_reverse_dependencies("main", recursive=True),
        )

    def test_dependency_loop(self):
        snapcraft_yaml = dedent(
            """\
            name: test
            version: "1"
            summary: test
            description: test
            confinement: strict
            grade: stable

            parts:
              p1:
                plugin: tar-content
                source: .
                after: [p2]
              p2:
                plugin: tar-content
                source: .
                after: [p1]
        """
        )
        raised = self.assertRaises(
            errors.SnapcraftLogicError, self.make_snapcraft_project, snapcraft_yaml
        )

        self.assertThat(
            raised.message,
            Equals("circular dependency chain found in parts definition"),
        )


class FilesetsTest(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.properties = {"filesets": {"1": ["1", "2", "3"], "2": []}}

    def test_expand_var(self):
        self.properties["stage"] = ["$1"]

        fs = _config._expand_filesets_for("stage", self.properties)
        self.assertThat(fs, Equals(["1", "2", "3"]))

    def test_no_expansion(self):
        self.properties["stage"] = ["1"]

        fs = _config._expand_filesets_for("stage", self.properties)
        self.assertThat(fs, Equals(["1"]))

    def test_invalid_expansion(self):
        self.properties["stage"] = ["$3"]

        raised = self.assertRaises(
            errors.SnapcraftLogicError,
            _config._expand_filesets_for,
            "stage",
            self.properties,
        )

        self.assertThat(
            str(raised),
            Contains(
                "'$3' referred to in the 'stage' fileset but it is not in filesets"
            ),
        )
