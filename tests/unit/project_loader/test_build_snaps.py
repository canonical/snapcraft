# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
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

from testtools.matchers import Equals

from . import ProjectLoaderBaseTest


class TestBuildSnapsFromSnapcraftYaml(ProjectLoaderBaseTest):
    def test_build_snaps_v1(self):
        snapcraft_yaml = dedent(
            """\
            name: test
            base: core18
            version: "1.0"
            summary: test
            description: test
            confinement: strict
            grade: stable


            parts:
              part1:
                plugin: nil
                build-snaps: [foo/latest/stable, bar/default/edge]
            """
        )

        project_config = self.make_snapcraft_project(snapcraft_yaml)

        self.assertThat(
            project_config.get_build_snaps(),
            Equals({"foo/latest/stable", "core18", "bar/default/edge"}),
        )

    def test_build_snaps_v2(self):
        snapcraft_yaml = dedent(
            """\
            name: test
            base: core20
            version: "1.0"
            summary: test
            description: test
            confinement: strict
            grade: stable


            parts:
              part1:
                plugin: nil
                build-snaps: [foo/latest/stable, bar/default/edge]
            """
        )

        project_config = self.make_snapcraft_project(snapcraft_yaml)

        self.assertThat(
            project_config.get_build_snaps(),
            Equals({"foo/latest/stable", "core20", "bar/default/edge"}),
        )
