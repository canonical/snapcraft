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

import pytest
from testtools.matchers import Equals, Is, MatchesRegex

import snapcraft.yaml_utils.errors
from snapcraft.project._project_info import ProjectInfo
from tests import unit


class ProjectInfoTest(unit.TestCase):
    def test_properties(self):
        snapcraft_yaml_file_path = self.make_snapcraft_yaml(
            dedent(
                """\
            name: foo
            version: "1"
            summary: bar
            description: baz
            confinement: strict
        """
            )
        )

        info = ProjectInfo(snapcraft_yaml_file_path=snapcraft_yaml_file_path)
        self.assertThat(info.name, Equals("foo"))
        self.assertThat(info.version, Equals("1"))
        self.assertThat(info.summary, Equals("bar"))
        self.assertThat(info.description, Equals("baz"))
        self.assertThat(info.confinement, Equals("strict"))

    def test_empty_yaml(self):
        snapcraft_yaml_file_path = self.make_snapcraft_yaml("")

        raised = self.assertRaises(
            snapcraft.yaml_utils.errors.YamlValidationError,
            ProjectInfo,
            snapcraft_yaml_file_path=snapcraft_yaml_file_path,
        )

        self.assertThat(
            raised.message,
            Equals(
                "'name' is a required property in {!r}".format(snapcraft_yaml_file_path)
            ),
        )

    def test_minimal_load_with_name_only(self):
        snapcraft_yaml_file_path = self.make_snapcraft_yaml(
            dedent(
                """\
            name: foo
        """
            )
        )

        info = ProjectInfo(snapcraft_yaml_file_path=snapcraft_yaml_file_path)
        self.assertThat(info.name, Equals("foo"))
        self.assertThat(info.version, Is(None))
        self.assertThat(info.summary, Is(None))
        self.assertThat(info.description, Is(None))
        self.assertThat(info.confinement, Is(None))

    def test_name_is_required(self):
        snapcraft_yaml_file_path = self.make_snapcraft_yaml(
            dedent(
                """\
            version: "1"
            summary: bar
            description: baz
            confinement: strict
        """
            )
        )

        raised = self.assertRaises(
            snapcraft.yaml_utils.errors.YamlValidationError,
            ProjectInfo,
            snapcraft_yaml_file_path=snapcraft_yaml_file_path,
        )

        self.assertThat(
            raised.message,
            Equals(
                "'name' is a required property in {!r}".format(snapcraft_yaml_file_path)
            ),
        )

    def test_get_raw_dict(self):
        snapcraft_yaml_file_path = self.make_snapcraft_yaml(
            dedent(
                """\
            name: foo
            version: "1"
            summary: bar
            description: baz
            confinement: strict
        """
            )
        )

        info = ProjectInfo(snapcraft_yaml_file_path=snapcraft_yaml_file_path)
        raw_snapcraft = info.get_raw_snapcraft()

        self.assertThat(raw_snapcraft["name"], Equals("foo"))
        self.assertThat(raw_snapcraft["version"], Equals("1"))
        self.assertThat(raw_snapcraft["summary"], Equals("bar"))
        self.assertThat(raw_snapcraft["description"], Equals("baz"))
        self.assertThat(raw_snapcraft["confinement"], Equals("strict"))

    def test_get_raw_dict_is_a_copy(self):
        snapcraft_yaml_file_path = self.make_snapcraft_yaml(
            dedent(
                """\
            name: foo
            version: "1"
            summary: bar
            description: baz
            confinement: strict
        """
            )
        )

        info = ProjectInfo(snapcraft_yaml_file_path=snapcraft_yaml_file_path)
        raw_snapcraft = info.get_raw_snapcraft()
        raw_snapcraft.pop("name")
        self.assertThat(raw_snapcraft.get("name"), Is(None))

        raw_snapcraft = info.get_raw_snapcraft()
        self.assertThat(raw_snapcraft.get("name"), Equals("foo"))


class InvalidYamlTest(unit.TestCase):
    def test_tab_in_yaml(self):
        snapcraft_yaml_file_path = self.make_snapcraft_yaml(
            dedent(
                """\
            name: test
            version: "1"
            summary: test
            description: nothing
            \tconfinement: strict
            grade: stable

            parts:
              part1:
                plugin: go
                stage-packages: [fswebcam]
        """
            )
        )

        raised = self.assertRaises(
            snapcraft.yaml_utils.errors.YamlValidationError,
            ProjectInfo,
            snapcraft_yaml_file_path=snapcraft_yaml_file_path,
        )

        self.assertThat(raised.source, Equals(snapcraft_yaml_file_path))
        # libyaml had a spelling mistake indentation/intendation
        self.assertThat(
            raised.message,
            MatchesRegex(
                "found a tab character that violate (indentation|intendation)"
                " on line 5, column 1"
            ),
        )

    def test_invalid_yaml_invalid_unicode_chars(self):
        snapcraft_yaml_file_path = self.make_snapcraft_yaml(
            dedent(
                """\
            name: foobar
            version: "1"
            summary: test\uffff
            description: nothing
        """
            )
        )

        raised = self.assertRaises(
            snapcraft.yaml_utils.errors.YamlValidationError,
            ProjectInfo,
            snapcraft_yaml_file_path=snapcraft_yaml_file_path,
        )

        self.assertThat(raised.source, Equals(snapcraft_yaml_file_path))
        self.assertThat(
            raised.message,
            Equals(
                "invalid character '\\uffff' at position 40: control characters are not allowed"
            ),
        )

    def test_invalid_yaml_unhashable(self):
        snapcraft_yaml_file_path = self.make_snapcraft_yaml(
            dedent(
                """\
                name: test
                version: {{invalid}}
                summary: test
                description: test
                confinement: strict
                grade: stable
                parts:
                part1:
                    plugin: nil
                """
            )
        )

        raised = self.assertRaises(
            snapcraft.yaml_utils.errors.YamlValidationError,
            ProjectInfo,
            snapcraft_yaml_file_path=snapcraft_yaml_file_path,
        )

        self.assertThat(raised.source, Equals(snapcraft_yaml_file_path))
        self.assertThat(
            raised.message, Equals("found unhashable key on line 2, column 10")
        )

    def test_invalid_yaml_list_in_mapping(self):
        snapcraft_yaml_file_path = self.make_snapcraft_yaml(
            dedent(
                """\
                name: foobar
                - list item
                """
            )
        )

        raised = self.assertRaises(
            snapcraft.yaml_utils.errors.YamlValidationError,
            ProjectInfo,
            snapcraft_yaml_file_path=snapcraft_yaml_file_path,
        )

        self.assertThat(raised.source, Equals(snapcraft_yaml_file_path))
        self.assertThat(
            raised.message, Equals("did not find expected key on line 2, column 1")
        )


@pytest.mark.parametrize("encoding", ["utf-8", "utf-8-sig", "utf-16"])
def test_different_encodings_loads(tmp_work_path, encoding):
    snapcraft_yaml = dedent(
        """\
        name: test
        version: "1"
        summary: test
        description: ñoño test
        confinement: strict
        grade: stable

        parts:
          part1:
            plugin: go
            stage-packages: [fswebcam]
    """
    )

    snapcraft_yaml_path = tmp_work_path / "snapcraft.yaml"
    with snapcraft_yaml_path.open("w", encoding=encoding) as snapcraft_file:
        print(snapcraft_yaml, file=snapcraft_file)

    ProjectInfo(snapcraft_yaml_file_path=snapcraft_yaml_path.as_posix())
