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
            internal_dir = work_dir
        else:
            work_dir = project_dir
            internal_dir = os.path.join(work_dir, "snap", ".snapcraft")

        # This here check is mostly for backwards compatibility with the
        # rest of the code base.
        if snapcraft_yaml_file_path is None:
            self.info = None  # type: ProjectInfo
        else:
            self.info = ProjectInfo(snapcraft_yaml_file_path=snapcraft_yaml_file_path)

        self._global_state_file = os.path.join(internal_dir, "state")
        self._project_dir = project_dir
        self._work_dir = work_dir
        self._internal_dir = internal_dir

        super().__init__(target_deb_arch, debug, work_dir=work_dir)
