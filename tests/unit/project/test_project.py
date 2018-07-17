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

from testtools.matchers import Equals, Is

from snapcraft.project import Project


class ProjectTest(unit.TestCase):
    def test_project_with_arguments(self):
        project = Project(
            use_geoip=True, parallel_builds=False, target_deb_arch="armhf", debug=True
        )

        self.assertThat(project.use_geoip, Equals(True))
        self.assertThat(project.parallel_builds, Equals(False))
        self.assertThat(project.deb_arch, Equals("armhf"))
        self.assertThat(project.debug, Equals(True))
        # This is a backwards compatibility check
        self.assertThat(project.info, Is(None))

    def test_project_with_snapcraft_yaml_file_path_carries_info(self):
        snapcraft_yaml_file_path = self.make_snapcraft_yaml(
            """\
            name: foo
            version: "1"
            summary: bar
            description: baz
            confinement: strict

            parts:
              part1:
                plugin: go
            """
        )

        project = Project(snapcraft_yaml_file_path=snapcraft_yaml_file_path)

        # Only 1 value is enough
        self.assertThat(project.info.name, Equals("foo"))
