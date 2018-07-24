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

from tests import unit
from textwrap import dedent

from testtools.matchers import Equals, Is

from snapcraft.project._project_info import ProjectInfo
from snapcraft.project import errors


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
            errors.YamlValidationError,
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
            errors.YamlValidationError,
            ProjectInfo,
            snapcraft_yaml_file_path=snapcraft_yaml_file_path,
        )

        self.assertThat(
            raised.message,
            Equals(
                "found character '\\t' that cannot start any token "
                "on line 5 of snap/snapcraft.yaml"
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
            errors.YamlValidationError,
            ProjectInfo,
            snapcraft_yaml_file_path=snapcraft_yaml_file_path,
        )

        self.assertThat(
            raised.message,
            Equals(
                "Invalid character '\\uffff' at position 40 "
                "of snap/snapcraft.yaml: special characters are not allowed"
            ),
        )


class YamlEncodingsTest(unit.TestCase):

    scenarios = [
        (encoding, dict(encoding=encoding))
        for encoding in ["utf-8", "utf-8-sig", "utf-16"]
    ]

    def test_config_loads_with_different_encodings_loads(self):
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

        snapcraft_yaml_file_path = self.make_snapcraft_yaml(
            snapcraft_yaml, encoding=self.encoding
        )
        ProjectInfo(snapcraft_yaml_file_path=snapcraft_yaml_file_path)
