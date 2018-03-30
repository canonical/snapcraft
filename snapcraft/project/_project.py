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

from ._project_options import ProjectOptions
from ._project_info import ProjectInfo                 # noqa: F401


class Project(ProjectOptions):
    """All details around building a project concerning the build environment
    and the snap being built."""

    def __init__(self, *, use_geoip=False, parallel_builds=True,
                 target_deb_arch: str=None, debug=False) -> None:
        self.info = None  # type: ProjectInfo

        super().__init__(use_geoip, parallel_builds, target_deb_arch, debug)
