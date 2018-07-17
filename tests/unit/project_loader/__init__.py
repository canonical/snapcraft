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

from unittest import mock

from snapcraft.project import Project as _Project
from snapcraft.internal import project_loader
from tests import unit


class ProjectLoaderBaseTest(unit.TestCase):
    def make_snapcraft_project(self, snapcraft_yaml, project_kwargs=None):
        snapcraft_yaml_file_path = self.make_snapcraft_yaml(snapcraft_yaml)
        if project_kwargs is None:
            project_kwargs = dict()
        project = _Project(
            snapcraft_yaml_file_path=snapcraft_yaml_file_path, **project_kwargs
        )
        return project_loader.load_config(project)


class LoadPartBaseTest(ProjectLoaderBaseTest):
    def setUp(self):
        super().setUp()

        patcher = mock.patch(
            "snapcraft.internal.project_loader._parts_config.PartsConfig.load_part"
        )
        self.mock_load_part = patcher.start()
        self.addCleanup(patcher.stop)
