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

import os

from ._project_options import ProjectOptions
from ._project_info import ProjectInfo  # noqa: F401


class Project(ProjectOptions):
    """All details around building a project concerning the build environment
    and the snap being built."""

    def __init__(
        self,
        *,
        target_deb_arch=None,
        debug=False,
        snapcraft_yaml_file_path=None,
        work_dir: str = None,
        is_managed_host: bool = False
    ) -> None:

        project_dir = os.getcwd()
        if is_managed_host:
            work_dir = os.path.expanduser("~")
        else:
            work_dir = project_dir

        # This here check is mostly for backwards compatibility with the
        # rest of the code base.
        if snapcraft_yaml_file_path is None:
            self.info = None  # type: ProjectInfo
        else:
            self.info = ProjectInfo(snapcraft_yaml_file_path=snapcraft_yaml_file_path)

        self._is_managed_host = is_managed_host
        self._project_dir = project_dir
        self._work_dir = work_dir

        super().__init__(target_deb_arch, debug, work_dir=work_dir)

    def _get_snapcraft_assets_dir(self) -> str:
        if self.info.snapcraft_yaml_file_path.endswith(
            os.path.join("build-aux", "snap", "snapcraft.yaml")
        ):
            return os.path.join(self._project_dir, "build-aux", "snap")
        else:
            return os.path.join(self._project_dir, "snap")

    def _get_global_state_file_path(self) -> str:
        if self._is_managed_host:
            state_file_path = os.path.join(self._work_dir, "state")
        else:
            state_file_path = os.path.join(self._parts_dir, ".snapcraft_global_state")

        return state_file_path
