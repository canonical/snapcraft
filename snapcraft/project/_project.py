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
        use_geoip=False,
        parallel_builds=True,
        target_deb_arch=None,
        debug=False,
        snapcraft_yaml_file_path=None,
        project_dir: str = None
    ) -> None:

        if project_dir is None:
            self.project_dir = os.getcwd()
        else:
            self.project_dir = project_dir

        # This here check is mostly for backwards compatibility with the
        # rest of the code base.
        if snapcraft_yaml_file_path is None:
            self.info = None  # type: ProjectInfo
        else:
            self.info = ProjectInfo(snapcraft_yaml_file_path=snapcraft_yaml_file_path)

        # These paths maintain backwards compatibility.
        self._internal_dir = os.path.join(self.project_dir, "snap", ".snapcraft")
        self._global_state_file = os.path.join(self._internal_dir, "state")

        super().__init__(
            use_geoip, parallel_builds, target_deb_arch, debug, project_dir=project_dir
        )
