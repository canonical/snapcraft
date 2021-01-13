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
import snapcraft.yaml_utils.errors
from snapcraft.project import Project as _Project
from tests import unit


class ProjectBaseTest(unit.TestCase):
    def make_snapcraft_project(self, snapcraft_yaml, project_kwargs=None):
        snapcraft_yaml_file_path = self.make_snapcraft_yaml(snapcraft_yaml)
        if project_kwargs is None:
            project_kwargs = dict()
        return _Project(
            snapcraft_yaml_file_path=snapcraft_yaml_file_path, **project_kwargs
        )

    def assertValidationPasses(self, snapcraft_yaml):
        project = self.make_snapcraft_project(snapcraft_yaml)
        project.info.validate_raw_snapcraft()

        return project.info.get_raw_snapcraft()

    def assertValidationRaises(self, snapcraft_yaml):
        project = self.make_snapcraft_project(snapcraft_yaml)

        return self.assertRaises(
            snapcraft.yaml_utils.errors.YamlValidationError,
            project.info.validate_raw_snapcraft,
        )
